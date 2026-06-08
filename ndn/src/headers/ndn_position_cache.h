/**
 * @file ndn_position_cache.h
 * @brief NDN-received position cache for MEC architecture
 * 
 * This module stores vehicle positions received via NDN Interest/Data exchange
 * instead of directly reading from vehicleStatuses (which bypasses NDN).
 * 
 * The MEC server queries vehicle positions via NDN:
 *   Interest: /v2x/v2i/vehicle/{id}/position
 *   Data: Contains x, y, z, speed, heading
 * 
 * This enables realistic V2I latency measurement.
 */

#ifndef NDN_POSITION_CACHE_H
#define NDN_POSITION_CACHE_H

#include "ns3/core-module.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"

#include "v2x_constants.h"

#include <map>
#include <string>
#include <functional>

namespace v2x {
namespace ndn_cache {

// ============================================================================
// NDN RECEIVED POSITION STRUCTURE
// ============================================================================

/**
 * @brief Stores a vehicle position received via NDN Interest/Data
 */
struct NdnReceivedPosition
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    std::string omnetId;
    ns3::Time receivedAt = ns3::Seconds(0);
    ns3::Time querySentAt = ns3::Seconds(0);  // When Interest was sent (for latency)
    bool valid = false;                        // Has data been received at least once?
    
    /**
     * @brief Check if position data is stale (>500ms old)
     */
    bool IsStale() const
    {
        return (ns3::Simulator::Now() - receivedAt) > ns3::MilliSeconds(POSITION_FRESHNESS_MS);
    }
    
    /**
     * @brief Get the age of this position data
     */
    ns3::Time GetAge() const
    {
        return ns3::Simulator::Now() - receivedAt;
    }
    
    /**
     * @brief Get the round-trip latency (Interest to Data)
     */
    ns3::Time GetRttLatency() const
    {
        if (querySentAt == ns3::Seconds(0) || receivedAt == ns3::Seconds(0))
            return ns3::MilliSeconds(0);
        return receivedAt - querySentAt;
    }
};

// ============================================================================
// NDN POSITION QUERY STATISTICS
// ============================================================================

/**
 * @brief Statistics for NDN position queries
 */
struct NdnPositionQueryStats
{
    uint64_t interestsSent = 0;
    uint64_t dataReceived = 0;
    uint64_t timeouts = 0;
    uint64_t staleDataUsed = 0;      // Times stale data was used in collision check
    double totalRttMs = 0.0;
    uint32_t rttSamples = 0;
    
    double GetAvgRttMs() const
    {
        return (rttSamples > 0) ? (totalRttMs / rttSamples) : 0.0;
    }
};

// ============================================================================
// NDN POSITION CACHE MANAGER
// ============================================================================

/**
 * @brief Singleton manager for NDN-received vehicle positions
 */
class NdnPositionCache
{
public:
    static NdnPositionCache& GetInstance();
    
    /**
     * @brief Record that an Interest was sent for a vehicle's position
     * @param vehicleId The NS-3 vehicle ID (e.g., "veh0")
     */
    void RecordInterestSent(const std::string& vehicleId);
    
    /**
     * @brief Update position from received NDN Data
     * @param vehicleId The NS-3 vehicle ID
     * @param x X position
     * @param y Y position
     * @param z Z position
     * @param speed Vehicle speed
     * @param heading Vehicle heading (radians)
     * @param omnetId The OMNeT++ vehicle ID
     */
    void UpdateFromNdnData(const std::string& vehicleId, 
                           double x, double y, double z,
                           double speed, double heading,
                           const std::string& omnetId = "");
    
    /**
     * @brief Get a vehicle's position (NDN-received)
     * @param vehicleId The NS-3 vehicle ID
     * @param[out] position Output position struct
     * @return true if position exists (may be stale), false if never received
     */
    bool GetPosition(const std::string& vehicleId, NdnReceivedPosition& position) const;
    
    /**
     * @brief Get all valid positions (for collision detection)
     * @param includeStale If true, include stale positions
     * @return Map of vehicle ID to position
     */
    std::map<std::string, NdnReceivedPosition> GetAllPositions(bool includeStale = false) const;
    
    /**
     * @brief Check if a vehicle has fresh position data
     */
    bool HasFreshPosition(const std::string& vehicleId) const;
    
    /**
     * @brief Record timeout for a position query
     */
    void RecordTimeout(const std::string& vehicleId);
    
    /**
     * @brief Record that stale data was used
     */
    void RecordStaleDataUsed();
    
    /**
     * @brief Mark a vehicle as inactive (left simulation)
     */
    void MarkInactive(const std::string& vehicleId);
    
    /**
     * @brief Get statistics
     */
    const NdnPositionQueryStats& GetStats() const { return m_stats; }
    
    /**
     * @brief Get mean measured V2I RTT in milliseconds
     * Used as calibrated latency baseline by other modules (e.g., accident notification)
     * @return Mean RTT in ms, or 0.0 if no samples yet
     */
    double GetMeanRttMs() const { return m_stats.GetAvgRttMs(); }
    
    /**
     * @brief Print statistics summary
     */
    void PrintSummary() const;
    
    /**
     * @brief Clear all cached positions
     */
    void Clear();

private:
    NdnPositionCache() = default;
    
    std::map<std::string, NdnReceivedPosition> m_positions;
    NdnPositionQueryStats m_stats;
};

// ============================================================================
// GLOBAL ACCESSOR FOR BACKWARD COMPATIBILITY
// ============================================================================

/**
 * @brief Global map of NDN-received positions (extern)
 * This is populated by NdnPositionCache and can be accessed directly
 * for backward compatibility with existing code.
 */
extern std::map<std::string, NdnReceivedPosition> ndnReceivedPositions;

} // namespace ndn_cache
} // namespace v2x

#endif // NDN_POSITION_CACHE_H
