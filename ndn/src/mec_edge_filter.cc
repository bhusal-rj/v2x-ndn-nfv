/**
 * @file mec_edge_filter.cc
 * @brief Implementation of MEC Edge Filtering for Architecture A
 * 
 * Use Case I: Intelligent Accident Proactive Caching
 * - RSU filters duplicate accident reports at the Edge
 * - Only unique reports are forwarded to Cloud (backhaul protection)
 * - MANO orchestrates proactive caching to relevant RSUs
 */

// Include ndnSIM FIRST to avoid namespace conflicts
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder.hpp"

#include "mec_edge_filter.h"
#include "simulation_state.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

// For KeyChain and signing
#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/security/signing-info.hpp>

namespace v2x {
namespace mec {

// ============================================================================
// RSU EDGE FILTER IMPLEMENTATION
// ============================================================================

RsuEdgeFilter::RsuEdgeFilter(const std::string& rsuId)
    : m_rsuId(rsuId)
{
}

bool RsuEdgeFilter::ProcessAccidentReport(const std::string& accidentId,
                                          const std::string& sourceVehicle,
                                          uint32_t payloadSize)
{
    m_stats.totalPacketsReceived++;
    
    // Check if already processed (MEC Fast Path)
    if (m_processedSet.find(accidentId) != m_processedSet.end())
    {
        // DUPLICATE - Drop at Edge, save backhaul
        m_stats.duplicatesDropped++;
        m_stats.backhaulBytesSaved += payloadSize;
        m_stats.filterEfficiency = (double)m_stats.duplicatesDropped / m_stats.totalPacketsReceived * 100.0;
        
        std::cout << "  🛡️  [MEC-" << m_rsuId << "] DROPPED duplicate accident " 
                  << accidentId << " from " << sourceVehicle 
                  << " (saved " << payloadSize << " bytes backhaul)" << std::endl;
        
        return false; // Duplicate - don't forward
    }
    
    // UNIQUE - Add to ProcessedSet and forward to Cloud
    m_processedSet.insert(accidentId);
    m_stats.uniqueForwarded++;
    m_stats.filterEfficiency = (double)m_stats.duplicatesDropped / m_stats.totalPacketsReceived * 100.0;
    
    std::cout << "  ✅ [MEC-" << m_rsuId << "] FORWARDING unique accident " 
              << accidentId << " from " << sourceVehicle 
              << " to Cloud (via 5G backhaul)" << std::endl;
    
    return true; // Unique - forward to Cloud
}

bool RsuEdgeFilter::IsDuplicate(const std::string& accidentId) const
{
    return m_processedSet.find(accidentId) != m_processedSet.end();
}

// ============================================================================
// MEC EDGE MANAGER IMPLEMENTATION
// ============================================================================

MecEdgeManager& MecEdgeManager::GetInstance()
{
    static MecEdgeManager instance;
    return instance;
}

void MecEdgeManager::RegisterRsu(const std::string& rsuId)
{
    if (m_rsuFilters.find(rsuId) == m_rsuFilters.end())
    {
        m_rsuFilters.emplace(rsuId, RsuEdgeFilter(rsuId));
        std::cout << "📡 [MEC] Registered Edge Filter for " << rsuId << std::endl;
    }
}

std::string MecEdgeManager::FindNearestRsu(double x, double y)
{
    std::string nearestRsu;
    double minDistance = std::numeric_limits<double>::max();
    
    // Get RSU positions from global nodeMapping
    for (const auto& [rsuId, filter] : m_rsuFilters)
    {
        auto it = nodeMapping.find(rsuId);
        if (it != nodeMapping.end())
        {
            ns3::Ptr<ns3::MobilityModel> mobility = it->second->GetObject<ns3::MobilityModel>();
            if (mobility)
            {
                ns3::Vector pos = mobility->GetPosition();
                double dx = pos.x - x;
                double dy = pos.y - y;
                double distance = std::sqrt(dx * dx + dy * dy);
                
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestRsu = rsuId;
                }
            }
        }
    }
    
    return nearestRsu;
}

std::string MecEdgeManager::ProcessAccidentAtEdge(const std::string& accidentId,
                                                  const std::string& vehicleId,
                                                  double vehicleX, double vehicleY,
                                                  uint32_t payloadSize)
{
    // Find nearest RSU to handle this report
    std::string nearestRsu = FindNearestRsu(vehicleX, vehicleY);
    
    if (nearestRsu.empty())
    {
        std::cerr << "⚠️  [MEC] No RSU available for accident " << accidentId << std::endl;
        return "";
    }
    
    // Process at Edge
    auto& filter = m_rsuFilters.at(nearestRsu);
    bool isUnique = filter.ProcessAccidentReport(accidentId, vehicleId, payloadSize);
    
    return isUnique ? nearestRsu : "";  // Return RSU ID if unique, empty if duplicate
}

EdgeFilterStats MecEdgeManager::GetAggregatedStats() const
{
    EdgeFilterStats aggregated;
    
    for (const auto& [rsuId, filter] : m_rsuFilters)
    {
        const auto& stats = filter.GetStats();
        aggregated.totalPacketsReceived += stats.totalPacketsReceived;
        aggregated.duplicatesDropped += stats.duplicatesDropped;
        aggregated.uniqueForwarded += stats.uniqueForwarded;
        aggregated.backhaulBytesSaved += stats.backhaulBytesSaved;
    }
    
    if (aggregated.totalPacketsReceived > 0)
    {
        aggregated.filterEfficiency = (double)aggregated.duplicatesDropped / 
                                      aggregated.totalPacketsReceived * 100.0;
    }
    
    return aggregated;
}

void MecEdgeManager::PrintSummary() const
{
    auto stats = GetAggregatedStats();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  MEC Edge Filter Summary (Arch A - UC1)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total Packets Received:  " << stats.totalPacketsReceived << std::endl;
    std::cout << "Duplicates Dropped:      " << stats.duplicatesDropped << std::endl;
    std::cout << "Unique Forwarded:        " << stats.uniqueForwarded << std::endl;
    std::cout << "Backhaul Bytes Saved:    " << stats.backhaulBytesSaved << " bytes" << std::endl;
    std::cout << "Filter Efficiency:       " << std::fixed << std::setprecision(1) 
              << stats.filterEfficiency << "%" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Per-RSU breakdown
    std::cout << "Per-RSU Breakdown:" << std::endl;
    for (const auto& [rsuId, filter] : m_rsuFilters)
    {
        const auto& s = filter.GetStats();
        if (s.totalPacketsReceived > 0)
        {
            std::cout << "  " << rsuId << ": " 
                      << s.uniqueForwarded << " fwd, "
                      << s.duplicatesDropped << " dropped ("
                      << std::fixed << std::setprecision(1) << s.filterEfficiency << "% eff)"
                      << std::endl;
        }
    }
}

// ============================================================================
// PROACTIVE CACHE MANAGER IMPLEMENTATION
// ============================================================================

ProactiveCacheManager& ProactiveCacheManager::GetInstance()
{
    static ProactiveCacheManager instance;
    return instance;
}

void ProactiveCacheManager::ExecuteCachingDecision(const CachingDecision& decision)
{
    std::cout << "\n📦 [MANO] Executing Proactive Caching Decision:" << std::endl;
    std::cout << "   Accident: " << decision.accidentId << std::endl;
    std::cout << "   Road: " << decision.roadId << std::endl;
    std::cout << "   Reason: " << decision.reason << std::endl;
    std::cout << "   Target RSUs: ";
    
    for (const auto& rsuId : decision.targetRsus)
    {
        std::cout << rsuId << " ";
    }
    std::cout << std::endl;
    
    // Create the accident data content
    // Ensure no double-prefix: accidentId should be just the ID like "crash_f_7.0_29"
    std::string dataName = "/v2x/safety/accident/" + decision.accidentId;
    // Safety check: remove any accidental double prefix
    size_t doublePrefix = dataName.find("/v2x/safety/accident//v2x/");
    if (doublePrefix != std::string::npos)
    {
        dataName = "/v2x/safety/accident/" + decision.accidentId.substr(decision.accidentId.rfind("/") + 1);
        std::cout << "   ⚠️  Fixed double-prefix: " << dataName << std::endl;
    }
    std::stringstream contentStream;
    contentStream << "ACCIDENT_DATA|"
                  << "id=" << decision.accidentId << "|"
                  << "road=" << decision.roadId << "|"
                  << "cached_at=" << ns3::Simulator::Now().GetSeconds();
    std::string content = contentStream.str();
    
    // Step 1: Cache at MEC (authoritative source)
    if (ndnMecNode)
    {
        PushToRsuCache("mec", dataName, content);
    }
    
    // Step 2: Proactively push to upstream RSUs (for cache hits)
    for (const auto& rsuId : decision.targetRsus)
    {
        PushToRsuCache(rsuId, dataName, content);
    }
    
    m_stats.dataPushed += decision.targetRsus.size();
}

void ProactiveCacheManager::PushToRsuCache(const std::string& nodeId,
                                           const std::string& dataName,
                                           const std::string& content)
{
    // Track what's cached where
    m_rsuCacheContents[nodeId].insert(dataName);
    
    std::cout << "   ➡️  Pushing " << dataName << " to " << nodeId << " cache..." << std::endl;
    
    // Find the node (RSU or MEC)
    ns3::Ptr<ns3::Node> targetNode = nullptr;
    
    if (nodeId == "mec")
    {
        targetNode = ndnMecNode;
    }
    else
    {
        auto it = nodeMapping.find(nodeId);
        if (it != nodeMapping.end())
        {
            targetNode = it->second;
        }
    }
    
    if (targetNode)
    {
        // Get the NDN L3 Protocol from the node
        auto ndnL3 = targetNode->GetObject<ns3::ndn::L3Protocol>();
        if (ndnL3)
        {
            auto forwarder = ndnL3->getForwarder();
            auto& cs = forwarder->getCs();
            
            // Create NDN Name and Data packet
            auto name = std::make_shared<ndn::Name>(dataName);
            auto data = std::make_shared<ndn::Data>(*name);
            
            // Set content
            auto contentBlock = ndn::encoding::makeStringBlock(ndn::tlv::Content, content);
            data->setContent(contentBlock);
            
            // Set freshness period (5 minutes for accident data)
            data->setFreshnessPeriod(ndn::time::seconds(300));
            
            // Sign the data (using SHA256 digest for simplicity)
            ndn::security::SigningInfo signingInfo(ndn::security::SigningInfo::SIGNER_TYPE_SHA256);
            ndn::KeyChain keyChain;
            keyChain.sign(*data, signingInfo);
            
            // ====================================================================
            // ARCHITECTURE NOTE: Proactive Content Store Population
            // ====================================================================
            // This implements proactive caching where MEC pre-populates the
            // Content Store with anticipated data. In a real NDN deployment,
            // this would be done via:
            //   1. Producer pushing data to cache
            //   2. Cache coordination protocol (e.g., NDN-DASH)
            //   
            // The cs.insert() call simulates MEC's ability to proactively cache
            // content without waiting for Interests. This is acceptable for
            // demonstrating edge caching benefits but note that:
            //   - No Interest/Data exchange occurs for pre-cached content
            //   - Cache hit metrics reflect proactive caching, not protocol hits
            // ====================================================================
            cs.insert(*data);
            
            m_stats.dataPushed++;
            std::cout << "   ✅ [NDN-CS] Cached at " << nodeId 
                      << " (CS size: " << cs.size() << ")" << std::endl;
        }
        else
        {
            std::cerr << "   ⚠️  [NDN-CS] No NDN L3 Protocol on " << nodeId << std::endl;
        }
    }
    else
    {
        std::cerr << "   ⚠️  [NDN-CS] Node " << nodeId << " not found" << std::endl;
    }
}

} // namespace mec
} // namespace v2x
