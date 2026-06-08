/**
 * @file mec_edge_filter.h
 * @brief MEC Edge Filtering for Architecture A - Use Case I
 * 
 * Implements local deduplication at RSU (Edge) to protect 5G backhaul.
 * Key features:
 * - ProcessedSet for duplicate detection
 * - Local filtering before forwarding to Cloud
 * - Backhaul bandwidth tracking
 */

#ifndef MEC_EDGE_FILTER_H
#define MEC_EDGE_FILTER_H

#include "ns3/core-module.h"
#include <string>
#include <set>
#include <map>
#include <vector>

namespace v2x {
namespace mec {

// ============================================================================
// MEC EDGE FILTER - Deduplication at RSU
// ============================================================================

/**
 * @brief Statistics for Edge filtering
 */
struct EdgeFilterStats
{
    uint64_t totalPacketsReceived = 0;
    uint64_t duplicatesDropped = 0;
    uint64_t uniqueForwarded = 0;
    uint64_t backhaulBytesSaved = 0;
    double filterEfficiency = 0.0; // (dropped / total) * 100
};

/**
 * @brief Per-RSU Edge Filter with ProcessedSet
 */
class RsuEdgeFilter
{
public:
    RsuEdgeFilter(const std::string& rsuId);
    
    /**
     * @brief Process incoming accident report at Edge
     * @param accidentId Unique accident identifier
     * @param sourceVehicle Vehicle that reported the accident
     * @param payloadSize Size of the report in bytes
     * @return true if unique (forward to Cloud), false if duplicate (drop)
     */
    bool ProcessAccidentReport(const std::string& accidentId, 
                               const std::string& sourceVehicle,
                               uint32_t payloadSize);
    
    /**
     * @brief Check if an accident is already processed
     */
    bool IsDuplicate(const std::string& accidentId) const;
    
    /**
     * @brief Get filter statistics
     */
    const EdgeFilterStats& GetStats() const { return m_stats; }
    
    /**
     * @brief Get RSU ID
     */
    const std::string& GetRsuId() const { return m_rsuId; }
    
    /**
     * @brief Clear processed set (for testing)
     */
    void ClearProcessedSet() { m_processedSet.clear(); }

private:
    std::string m_rsuId;
    std::set<std::string> m_processedSet;  // Set of processed accident IDs
    EdgeFilterStats m_stats;
};

// ============================================================================
// MEC EDGE MANAGER - Manages all RSU filters
// ============================================================================

/**
 * @brief Central manager for all Edge Filters
 */
class MecEdgeManager
{
public:
    static MecEdgeManager& GetInstance();
    
    /**
     * @brief Register an RSU with the Edge Manager
     */
    void RegisterRsu(const std::string& rsuId);
    
    /**
     * @brief Process accident at nearest RSU
     * @param accidentId Unique accident identifier
     * @param vehicleId Reporting vehicle
     * @param vehicleX Vehicle X position
     * @param vehicleY Vehicle Y position
     * @param payloadSize Report size in bytes
     * @return RSU ID that processed it, or empty if duplicate
     */
    std::string ProcessAccidentAtEdge(const std::string& accidentId,
                                      const std::string& vehicleId,
                                      double vehicleX, double vehicleY,
                                      uint32_t payloadSize);
    
    /**
     * @brief Find nearest RSU to a position
     */
    std::string FindNearestRsu(double x, double y);
    
    /**
     * @brief Get aggregated statistics
     */
    EdgeFilterStats GetAggregatedStats() const;
    
    /**
     * @brief Get per-RSU statistics
     */
    const std::map<std::string, RsuEdgeFilter>& GetRsuFilters() const { return m_rsuFilters; }

    /**
     * @brief Number of registered RSU edge filters
     */
    size_t GetRegisteredRsuCount() const { return m_rsuFilters.size(); }
    
    /**
     * @brief Print Edge Filter summary
     */
    void PrintSummary() const;

private:
    MecEdgeManager() = default;
    std::map<std::string, RsuEdgeFilter> m_rsuFilters;
};

// ============================================================================
// PROACTIVE CACHING - MANO-driven cache placement
// ============================================================================

/**
 * @brief Caching decision from MANO (OMNeT++)
 */
struct CachingDecision
{
    std::string accidentId;
    std::string roadId;
    std::vector<std::string> targetRsus;  // RSUs to cache data
    std::string reason;  // e.g., "upstream_traffic", "topology_aware"
};

/**
 * @brief Proactive Cache Manager
 */
class ProactiveCacheManager
{
public:
    static ProactiveCacheManager& GetInstance();
    
    /**
     * @brief Execute caching decision from MANO
     * @param decision Caching decision with target RSUs
     */
    void ExecuteCachingDecision(const CachingDecision& decision);
    
    /**
     * @brief Push data to specific RSU's NDN Content Store
     */
    void PushToRsuCache(const std::string& rsuId, 
                        const std::string& dataName,
                        const std::string& content);
    
    /**
     * @brief Get cache statistics
     */
    struct CacheStats
    {
        uint64_t proactiveCacheHits = 0;
        uint64_t proactiveCacheMisses = 0;
        uint64_t dataPushed = 0;
        std::map<std::string, uint64_t> perRsuCacheHits;
    };
    
    const CacheStats& GetStats() const { return m_stats; }

private:
    ProactiveCacheManager() = default;
    CacheStats m_stats;
    std::map<std::string, std::set<std::string>> m_rsuCacheContents;  // RSU -> cached data names
};

} // namespace mec
} // namespace v2x

#endif // MEC_EDGE_FILTER_H
