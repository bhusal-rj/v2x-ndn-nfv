#ifndef METRICS_COLLECTOR_H
#define METRICS_COLLECTOR_H

#include "ns3/core-module.h"
#include <string>
#include <vector>
#include <map>

using namespace ns3;

// ============================================================================
// QoS METRICS STRUCTURES
// ============================================================================

/**
 * @brief End-to-End Latency Tracking
 */
struct LatencyMetrics
{
    double minLatencyMs = std::numeric_limits<double>::max();
    double maxLatencyMs = 0.0;
    double totalLatencyMs = 0.0;
    uint64_t sampleCount = 0;
    std::vector<double> latencySamples;  // For percentile calculation
    
    double GetAverageLatency() const {
        return sampleCount > 0 ? totalLatencyMs / sampleCount : 0.0;
    }
    
    double GetJitter() const;  // Standard deviation of latency
    double GetPercentile(double p) const;  // Get p-th percentile (0-100)
};

/**
 * @brief Packet Delivery Ratio (PDR) Tracking
 */
struct DeliveryMetrics
{
    uint64_t packetsSent = 0;
    uint64_t packetsReceived = 0;
    uint64_t packetsLost = 0;
    uint64_t packetsRetransmitted = 0;
    
    double GetPDR() const {
        return packetsSent > 0 ? (double)packetsReceived / packetsSent * 100.0 : 0.0;
    }
    
    double GetPacketLossRate() const {
        return packetsSent > 0 ? (double)packetsLost / packetsSent * 100.0 : 0.0;
    }
};

/**
 * @brief Throughput Tracking
 */
struct ThroughputMetrics
{
    uint64_t totalBytesRx = 0;
    uint64_t totalBytesTx = 0;
    double measurementDuration = 0.0;
    std::vector<double> instantThroughputs;  // Mbps at each interval
    
    double GetAverageThroughputMbps() const {
        return measurementDuration > 0 ? 
               (totalBytesRx * 8.0) / (measurementDuration * 1e6) : 0.0;
    }
    
    double GetPeakThroughputMbps() const;
};

/**
 * @brief V2I/V2V Throughput Tracking (separated by traffic type)
 */
struct TypedThroughputMetrics
{
    uint64_t totalBytesReceived = 0;
    uint64_t totalBytesSent = 0;
    double samplingStartTime = 0.0;
    double samplingEndTime = 0.0;
    std::vector<double> throughputSamples;  // Mbps samples over time
    
    // For periodic sampling
    uint64_t lastSampleBytesRx = 0;
    double lastSampleTime = 0.0;
    
    double GetAverageThroughputMbps() const {
        double duration = samplingEndTime - samplingStartTime;
        if (duration <= 0) return 0.0;
        return (totalBytesReceived * 8.0) / (duration * 1e6);  // bytes to Mbps
    }
    
    double GetPeakThroughputMbps() const {
        if (throughputSamples.empty()) return 0.0;
        return *std::max_element(throughputSamples.begin(), throughputSamples.end());
    }
};

// Global V2I and V2V throughput trackers
extern TypedThroughputMetrics v2iThroughput;
extern TypedThroughputMetrics v2vThroughput;

/**
 * @brief Cache Performance Metrics
 */
struct CacheQoSMetrics
{
    uint64_t cacheHits = 0;
    uint64_t cacheMisses = 0;
    uint64_t proactiveCacheHits = 0;
    uint64_t reactiveCacheHits = 0;
    double avgCacheLatencyMs = 0.0;
    double avgOriginLatencyMs = 0.0;
    
    double GetHitRate() const {
        uint64_t total = cacheHits + cacheMisses;
        return total > 0 ? (double)cacheHits / total * 100.0 : 0.0;
    }
    
    double GetLatencyImprovement() const {
        return avgOriginLatencyMs > 0 ? 
               (1.0 - avgCacheLatencyMs / avgOriginLatencyMs) * 100.0 : 0.0;
    }
};

/**
 * @brief V2X Safety Message QoS
 */
struct SafetyQoSMetrics
{
    // BSM (Basic Safety Message) metrics
    uint64_t bsmSent = 0;
    uint64_t bsmReceived = 0;
    LatencyMetrics bsmLatency;
    
    // Collision Warning metrics
    uint64_t collisionWarningsSent = 0;
    uint64_t collisionWarningsReceived = 0;
    double avgWarningLatencyMs = 0.0;
    
    // Emergency Vehicle metrics
    uint64_t emergencyMsgSent = 0;
    uint64_t emergencyMsgReceived = 0;
    LatencyMetrics emergencyLatency;
    
    // Time-to-reaction
    double avgReactionTimeMs = 0.0;
    uint64_t successfulReactions = 0;
};

/**
 * @brief V2I Latency Metrics (through 5G NR to MEC)
 */
struct V2iLatencyMetrics
{
    double minLatencyMs = std::numeric_limits<double>::max();
    double maxLatencyMs = 0.0;
    double totalLatencyMs = 0.0;
    uint64_t sampleCount = 0;
    std::vector<double> latencySamples;
    
    double GetAverageLatency() const {
        return sampleCount > 0 ? totalLatencyMs / sampleCount : 0.0;
    }
    
    double GetJitter() const;
    double GetPercentile(double p) const;
    
    void AddSample(double latencyMs) {
        latencySamples.push_back(latencyMs);
        totalLatencyMs += latencyMs;
        sampleCount++;
        if (latencyMs < minLatencyMs) minLatencyMs = latencyMs;
        if (latencyMs > maxLatencyMs) maxLatencyMs = latencyMs;
    }
};

/**
 * @brief V2V Latency Metrics (V2V direct links between vehicles)
 */
struct V2vLatencyMetrics
{
    double minLatencyMs = std::numeric_limits<double>::max();
    double maxLatencyMs = 0.0;
    double totalLatencyMs = 0.0;
    uint64_t sampleCount = 0;
    std::vector<double> latencySamples;
    
    double GetAverageLatency() const {
        return sampleCount > 0 ? totalLatencyMs / sampleCount : 0.0;
    }
    
    double GetJitter() const;
    double GetPercentile(double p) const;
    
    void AddSample(double latencyMs) {
        latencySamples.push_back(latencyMs);
        totalLatencyMs += latencyMs;
        sampleCount++;
        if (latencyMs < minLatencyMs) minLatencyMs = latencyMs;
        if (latencyMs > maxLatencyMs) maxLatencyMs = latencyMs;
    }
};

/**
 * @brief 5G NR Configuration and QoS Metrics
 */
struct Nr5gQoSMetrics
{
    // NR Configuration (from numerology)
    uint32_t numerology = 1;              // μ value (0-4)
    double slotDurationMs = 0.5;          // 1ms / 2^μ
    double subcarrierSpacingKhz = 30.0;   // 15kHz × 2^μ
    
    // Mean V2I AppDelayTracer RTT (Interest→Data at app), same as qos_summary.v2i_latency_avg_ms;
    // full stack + MEC producer — not an MEC-only processing delay. Unset when no V2I app-delay samples.
    double edgeLatencyMs = 0.0;
    double targetReliability = 99.9;  // Measured reliability target
    
    // eMBB metrics
    double embbThroughputMbps = 0.0;
    
    // Resource utilization
    double avgPrbUtilization = 0.0;
    /// LteEnbRrc HandoverStart (attempt begins)
    uint64_t handovers = 0;
    /// LteUeRrc HandoverEndOk (completed successfully)
    uint64_t handover_completions = 0;
    /// LteUeRrc HandoverEndError
    uint64_t handoverFailures = 0;
    /// EndOk with no prior HandoverStart for this IMSI (restarts, tracing quirks)
    uint64_t handover_unpaired_end_ok = 0;
    /// Sum of (EndOk time − Start time) for paired events, milliseconds
    double handover_duration_total_ms = 0.0;
    /// Count of paired duration samples (same as paired completions with duration)
    uint64_t handover_duration_samples = 0;
    double handover_duration_min_ms = 0.0;
    double handover_duration_max_ms = 0.0;
    
    // V2V emulated direct links (PointToPoint), not 3GPP PC5 sidelink
    uint64_t v2vEmulatedLinkMessagesSent = 0;
    uint64_t v2vEmulatedLinkMessagesReceived = 0;
    /// Mean V2V latency from AppDelayTracer (V2V name prefixes), P2P-emulated V2V — not PC5.
    double v2vAppLatencyMs = 0.0;
    
    // Calculate NR parameters from numerology
    void SetNumerology(uint32_t mu) {
        numerology = mu;
        slotDurationMs = 1.0 / (1 << mu);  // 1ms / 2^μ
        subcarrierSpacingKhz = 15.0 * (1 << mu);  // 15kHz × 2^μ
    }
};

/**
 * @brief Comprehensive QoS Summary
 */
struct QoSSummary
{
    LatencyMetrics e2eLatency;        // End-to-end latency (all traffic)
    V2iLatencyMetrics v2iLatency;     // V2I latency (through 5G NR to MEC)
    V2vLatencyMetrics v2vLatency;     // V2V latency (V2V direct links)
    DeliveryMetrics delivery;          // Packet delivery
    ThroughputMetrics throughput;      // Network throughput
    CacheQoSMetrics caching;           // NDN caching
    SafetyQoSMetrics safety;           // V2X safety
    Nr5gQoSMetrics nr5g;               // 5G NR specific
    
    double simulationDuration = 0.0;
    uint64_t totalNodes = 0;
    uint64_t activeVehicles = 0;
    uint64_t activeRsus = 0;
};

// Global QoS summary
extern QoSSummary g_qosSummary;

// ============================================================================
// METRICS COLLECTION AND OUTPUT FUNCTIONS
// ============================================================================

/**
 * Helper to print and log NDN stats (CS, PIT, FIB) + Throughput
 * @param timestamp Current simulation time
 */
void PrintNdnStats(double timestamp);

/**
 * Connect NR RRC handover traces (gNB + UE). Call once after InstallUeDevice and
 * AttachToClosestEnb — UE-side paths must exist or Config::Connect aborts.
 */
void ConnectNrHandoverTraces();

/**
 * Helper to log data received from OMNeT++
 * @param msgType Type of message received
 * @param timestamp Timestamp from the message
 * @param rawJson Raw JSON string received
 * 
 * Logs to:
 * - omnet-data-log.txt (human readable format)
 * - omnet_messages.jsonl (JSON Lines format for easy parsing)
 */
void LogOmnetData(const std::string& msgType, double timestamp, const std::string& rawJson);

/**
 * Helper to log data sent from NS-3 to OMNeT++
 * @param msgType Type of message sent
 * @param timestamp Timestamp associated with the message
 * @param rawPayload Raw payload string sent over the socket
 *
 * Logs to:
 * - ns3-to-omnet-log.txt (human readable format)
 * - ns3_to_omnet_messages.jsonl (JSON Lines format for easy parsing)
 */
void LogNs3ToOmnetData(const std::string& msgType, double timestamp, const std::string& rawPayload);

/**
 * Close OMNeT++ log files
 * Call at end of simulation to flush and close log files
 */
void CloseOmnetLogs();

/**
 * Get total OMNeT++ message count
 * @return Total number of messages received from OMNeT++
 */
uint64_t GetOmnetMessageCount();

/**
 * Parse rate-trace.txt for Interest/Data packet counts
 * Updates nodeMetrics with packet statistics
 */
void ParseRateTrace();

/**
 * Parse ndn-cs-trace.txt for cache hits/misses
 * NOTE: Cache metrics are now tracked directly via CS signals
 */
void ParseCsTrace();

/**
 * Parse app-delays-trace.txt for latency
 * Updates nodeMetrics with latency information
 */
void ParseAppDelayTrace();

/**
 * Collect metrics from all trace files
 * Calls ParseRateTrace, ParseCsTrace, and ParseAppDelayTrace
 */
void CollectMetricsFromTracers();

/**
 * @brief Validate presence of critical measured metrics and emit diagnostics.
 * @param context Caller context for logging (e.g., "GenerateMetricsJSON")
 * @return true if all critical measured metrics are present, false otherwise
 *
 * This function is fail-fast in signaling only (logs warnings/errors),
 * and does not abort simulation/report generation.
 */
bool ValidateCriticalMetrics(const std::string& context = "end-of-run");

/**
 * Compute final metrics and generate JSON output
 * @param filename Output JSON filename
 */
void GenerateMetricsJSON(const std::string& filename);

/**
 * @brief Update QoS metrics with a new latency sample
 */
void RecordLatencySample(double latencyMs, const std::string& category = "general");

/**
 * @brief Update packet delivery metrics
 */
void RecordPacketDelivery(bool delivered, const std::string& category = "general");

/**
 * @brief Update cache metrics
 */
void RecordCacheAccess(bool hit, bool proactive = false, double latencyMs = 0.0);

/**
 * @brief Generate comprehensive QoS report
 */
void GenerateQoSReport(const std::string& filename);

/**
 * @brief Print QoS summary to console
 */
void PrintQoSSummary();

/**
 * @brief Calculate PDR from NDN forwarder counters
 * Uses nSatisfiedInterests/nOutInterests across all NDN nodes
 * @return PDR as percentage (0-100)
 */
double CalculatePacketDeliveryRatio();

/**
 * @brief Populate delivery metrics from NDN forwarder counters
 * Should be called before generating metrics JSON
 */
void CollectNdnDeliveryMetrics();

/**
 * @brief Connect throughput tracking signals to an NDN node
 * @param node The NDN node
 * @param isV2i True for V2I traffic (MEC), false for V2V traffic (vehicles)
 */
void ConnectThroughputTracking(ns3::Ptr<ns3::Node> node, bool isV2i);

/**
 * @brief Sample instantaneous throughput (called periodically)
 * Calculates throughput since last sample and stores in throughputSamples
 */
void SampleThroughput();

/**
 * @brief Start periodic throughput sampling
 * @param intervalSeconds Sampling interval in seconds (default 1.0)
 */
void StartThroughputSampling(double intervalSeconds = 1.0);

#endif // METRICS_COLLECTOR_H
