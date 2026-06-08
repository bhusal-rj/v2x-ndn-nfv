/**
 * @file ndn_position_cache.cc
 * @brief Implementation of NDN-received position cache for MEC architecture
 */

#include "ndn_position_cache.h"
#include "v2i_metrics_hook.h"

#include <iostream>
#include <iomanip>

namespace v2x {
namespace ndn_cache {

// Global map for backward compatibility
std::map<std::string, NdnReceivedPosition> ndnReceivedPositions;

// ============================================================================
// NDN POSITION CACHE IMPLEMENTATION
// ============================================================================

NdnPositionCache& NdnPositionCache::GetInstance()
{
    static NdnPositionCache instance;
    return instance;
}

void NdnPositionCache::RecordInterestSent(const std::string& vehicleId)
{
    m_stats.interestsSent++;
    
    // Initialize or update query time
    if (m_positions.find(vehicleId) == m_positions.end())
    {
        m_positions[vehicleId] = NdnReceivedPosition();
    }
    m_positions[vehicleId].querySentAt = ns3::Simulator::Now();
    
    // Sync to global map
    ndnReceivedPositions[vehicleId] = m_positions[vehicleId];
}

void NdnPositionCache::UpdateFromNdnData(const std::string& vehicleId,
                                          double x, double y, double z,
                                          double speed, double heading,
                                          const std::string& omnetId)
{
    m_stats.dataReceived++;
    
    ns3::Time querySentAt = ns3::Seconds(0);
    if (m_positions.find(vehicleId) != m_positions.end())
    {
        querySentAt = m_positions[vehicleId].querySentAt;
    }
    
    NdnReceivedPosition& pos = m_positions[vehicleId];
    pos.x = x;
    pos.y = y;
    pos.z = z;
    pos.speed = speed;
    pos.heading = heading;
    pos.omnetId = omnetId;
    pos.receivedAt = ns3::Simulator::Now();
    pos.querySentAt = querySentAt;
    pos.valid = true;
    
    // Calculate RTT if Interest was tracked
    if (querySentAt > ns3::Seconds(0))
    {
        double rttMs = pos.GetRttLatency().GetMilliSeconds();
        m_stats.totalRttMs += rttMs;
        m_stats.rttSamples++;
        if (rttMs > 0.0) {
            RecordV2iAppLayerRttSample(rttMs);
        }
    }
    
    // Sync to global map
    ndnReceivedPositions[vehicleId] = pos;
}

bool NdnPositionCache::GetPosition(const std::string& vehicleId, 
                                    NdnReceivedPosition& position) const
{
    auto it = m_positions.find(vehicleId);
    if (it == m_positions.end() || !it->second.valid)
    {
        return false;
    }
    position = it->second;
    return true;
}

std::map<std::string, NdnReceivedPosition> NdnPositionCache::GetAllPositions(bool includeStale) const
{
    std::map<std::string, NdnReceivedPosition> result;
    
    for (const auto& [id, pos] : m_positions)
    {
        if (!pos.valid) continue;
        if (!includeStale && pos.IsStale()) continue;
        result[id] = pos;
    }
    
    return result;
}

bool NdnPositionCache::HasFreshPosition(const std::string& vehicleId) const
{
    auto it = m_positions.find(vehicleId);
    if (it == m_positions.end() || !it->second.valid)
    {
        return false;
    }
    return !it->second.IsStale();
}

void NdnPositionCache::RecordTimeout(const std::string& vehicleId)
{
    m_stats.timeouts++;
}

void NdnPositionCache::RecordStaleDataUsed()
{
    m_stats.staleDataUsed++;
}

void NdnPositionCache::MarkInactive(const std::string& vehicleId)
{
    auto it = m_positions.find(vehicleId);
    if (it != m_positions.end())
    {
        it->second.valid = false;
        ndnReceivedPositions[vehicleId].valid = false;
    }
}

void NdnPositionCache::PrintSummary() const
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  NDN Position Query Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Interests Sent:      " << m_stats.interestsSent << std::endl;
    std::cout << "Data Received:       " << m_stats.dataReceived << std::endl;
    std::cout << "Timeouts:            " << m_stats.timeouts << std::endl;
    std::cout << "Stale Data Used:     " << m_stats.staleDataUsed << std::endl;
    std::cout << "Average RTT:         " << std::fixed << std::setprecision(2) 
              << m_stats.GetAvgRttMs() << " ms" << std::endl;
    std::cout << "RTT Samples:         " << m_stats.rttSamples << std::endl;
    
    // Count fresh vs stale positions
    int fresh = 0, stale = 0, invalid = 0;
    for (const auto& [id, pos] : m_positions)
    {
        if (!pos.valid) { invalid++; continue; }
        if (pos.IsStale()) stale++;
        else fresh++;
    }
    std::cout << "Cached Positions:    " << m_positions.size() 
              << " (fresh: " << fresh << ", stale: " << stale 
              << ", invalid: " << invalid << ")" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

void NdnPositionCache::Clear()
{
    m_positions.clear();
    ndnReceivedPositions.clear();
}

} // namespace ndn_cache
} // namespace v2x
