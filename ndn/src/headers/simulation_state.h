#ifndef SIMULATION_STATE_H
#define SIMULATION_STATE_H

// IMPORTANT: Include ndnSIM module BEFORE this header or before "using namespace ns3;"
// to avoid namespace conflicts between ndn:: and ns3::ndn::

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

#include <map>
#include <set>
#include <string>
#include <fstream>

// Forward declarations for ndnSIM types (avoid including ndnSIM-module.h here)
namespace ns3 {
namespace ndn {
    class StackHelper;
    class GlobalRoutingHelper;
}
}

// ============================================================================
// THROUGHPUT AND METRICS STRUCTURES
// ============================================================================

// Throughput tracking per node
struct NodeThroughputStats
{
    uint64_t totalRxBytes = 0;
    uint64_t totalTxBytes = 0;
    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    double lastUpdateTime = 0.0;
};

// Global V2X counters: V2I NDN-over-UDP datagrams (V2iIpv4UdpTransport doSend/HandleRead)
struct V2xThroughputStats
{
    uint64_t udpPacketsRx = 0;
    uint64_t udpBytesRx = 0;
    uint64_t udpPacketsTx = 0;
    uint64_t udpBytesTx = 0;
    uint64_t ndnInterestsTx = 0;
    uint64_t ndnDataRx = 0;
    uint64_t ndnBytesRx = 0;
};

// Comprehensive Metrics Tracking per node
struct NodeMetrics
{
    // Interest metrics
    uint64_t interestsSent = 0;
    uint64_t interestsReceived = 0;
    uint64_t interestsSatisfied = 0;
    uint64_t interestsTimedOut = 0;

    // Data metrics
    uint64_t dataSent = 0;
    uint64_t dataReceived = 0;

    // Interest/Data packet counters (tracked via signals)
    uint64_t interestsIn = 0;
    uint64_t interestsOut = 0;
    uint64_t dataIn = 0;
    uint64_t dataOut = 0;

    // Cache metrics (tracked via signals, not tracer files)
    uint64_t cacheHits = 0;
    uint64_t cacheMisses = 0;
    uint64_t maxCsSize = 0;
    uint64_t finalCsSize = 0;

    // Resource metrics
    uint64_t maxPitSize = 0;
    uint64_t finalPitSize = 0;
    uint64_t finalFibSize = 0;

    // Network performance
    double totalLatencyMs = 0.0;
    uint32_t latencySamples = 0;
    uint64_t totalBytes = 0;
};

// ============================================================================
// VEHICLE AND SAFETY STRUCTURES
// ============================================================================

// Vehicle Status for Collision Detection
struct VehicleStatus
{
    std::string id;
    std::string omnetId;
    double x = 0, y = 0, z = 0;
    double speed = 0;
    double heading = 0; // In radians
    double acceleration = 0;
    std::string vehicleType = "unknown";
    ns3::Time lastUpdate = ns3::Seconds(0);
    bool isActive = false; // Track if vehicle is active in simulation
};

// Safety event counters
struct SafetyStats
{
    uint64_t totalChecks = 0;
    uint64_t collisionWarnings = 0;
    uint64_t slowDownCommands = 0;
    uint64_t trafficQueries = 0;
    uint64_t positionUpdates = 0;
};

// ============================================================================
// SAFETY MESSAGE DELIVERY TRACKING
// ============================================================================

/**
 * @brief Track delivery status of individual safety messages
 * 
 * This struct tracks the lifecycle of each safety message from sent to delivered,
 * enabling accurate measurement of safety system effectiveness.
 */
struct SafetyMessageDelivery
{
    std::string messageId;       // Unique identifier (e.g., "warn_veh0_veh1_12345")
    std::string sourceVehicle;   // Vehicle that triggered the warning
    std::string targetVehicle;   // Vehicle that should receive the warning
    ns3::Time sentTime;          // When warning was generated at MEC
    ns3::Time receivedTime;      // When warning was received at target vehicle
    bool delivered = false;      // Whether delivery was confirmed
    double deliveryLatencyMs = 0.0;  // receivedTime - sentTime in milliseconds
    std::string warningType;     // CRITICAL, HIGH, MEDIUM, LOW
    std::string action;          // EMERGENCY_BRAKE, HARD_BRAKE, etc.
};

// ============================================================================
// V2V DIRECT LINK STRUCTURES
// ============================================================================
// Note: V2V uses P2P link emulation (100Mbps, 1ms delay), NOT 3GPP PC5.
// These parameters are for documentation/future use; the actual implementation
// uses ns3::PointToPointHelper with fixed data rate and delay settings.

// V2V Direct Link Configuration (P2P Emulation)
struct V2VDirectLinkConfig
{
    double linkDataRateMbps = 100.0;   // P2P link data rate (applied)
    double linkDelayMs = 1.0;          // P2P link delay (applied)
    // Reserved for future 3GPP-compliant implementation:
    double reservedBandwidth = 20e6;   // NOT APPLIED - reserved for future use
    double reservedFrequency = 5.9e9;  // NOT APPLIED - reserved for future use
    uint32_t reservedNumerology = 2;   // NOT APPLIED - reserved for future use
    double reservedTxPower = 23.0;     // NOT APPLIED - reserved for future use
    uint32_t reservedPoolSize = 100;   // NOT APPLIED - reserved for future use
    ns3::Time reservedPeriod = ns3::MilliSeconds(10); // NOT APPLIED - reserved
};

// Track V2V direct link statistics
struct V2VDirectLinkStats
{
    uint64_t v2vInterestsTx = 0;
    uint64_t v2vDataRx = 0;
    uint64_t v2vBytesTx = 0;
    uint64_t v2vBytesRx = 0;
    uint64_t directLinkConnections = 0;  // Number of P2P links created
    uint64_t v2vMessages = 0;
    
    // Canonical counters: v2vInterestsTx, v2vDataRx, v2vBytesTx, v2vBytesRx (P2P V2V, not PC5).
    // DEPRECATED: sidelink* member names below are legacy aliases only — misleading (not 3GPP
    // sidelink). Kept so existing code that references slStats.sidelink* still compiles; new code
    // should use v2v* fields. Metrics code keeps these in sync with v2v* where populated.
    uint64_t sidelinkInterestsTx = 0;    // DEPRECATED: use v2vInterestsTx
    uint64_t sidelinkDataRx = 0;         // DEPRECATED: use v2vDataRx
    uint64_t sidelinkBytesTx = 0;        // DEPRECATED: use v2vBytesTx
    uint64_t sidelinkBytesRx = 0;        // DEPRECATED: use v2vBytesRx
    uint64_t resourceAllocations = 0;    // Number of P2P link creations
    
    // Message transmission counters (for direct counting)
    uint64_t messagesTransmitted = 0;    // Direct V2V messages sent
    uint64_t messagesReceived = 0;       // Direct V2V messages received
};

// Legacy aliases for backward compatibility
// NOTE: Pc5SidelinkConfig/SidelinkStats names retained for code that references
// them, but the implementation uses P2P emulation, NOT 3GPP PC5 sidelink.
using Pc5SidelinkConfig = V2VDirectLinkConfig;
using SidelinkStats = V2VDirectLinkStats;

// ============================================================================
// GLOBAL STATE DECLARATIONS (extern)
// ============================================================================

// Simulation State - Node Management
extern std::map<std::string, ns3::Ptr<ns3::Node>> nodeMapping;
extern std::map<std::string, NodeThroughputStats> throughputStats;
extern std::map<std::string, NodeMetrics> nodeMetrics;
extern std::map<std::string, VehicleStatus> vehicleStatuses;

// 5G NR Helpers
extern ns3::Ptr<ns3::NrHelper> nrHelper;
extern ns3::Ptr<ns3::NrPointToPointEpcHelper> nrEpcHelper;
extern ns3::Ptr<ns3::IdealBeamformingHelper> idealBeamformingHelper;
extern ns3::Ptr<ns3::Node> pgw;
extern ns3::NetDeviceContainer gNbDevs;
extern ns3::BandwidthPartInfoPtrVector allBwps;

// NDN Helpers (defined in simple_ndn.cc where ndnSIM-module.h is included)
extern ns3::ndn::StackHelper ndnHelper;
extern ns3::ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;

// Global V2X Stats
extern V2xThroughputStats g_v2xStats;
extern SafetyStats safetyStats;

// Safety Message Delivery Tracking (for research validity)
extern std::map<std::string, SafetyMessageDelivery> safetyMessageTracking;
extern uint64_t totalSafetyMessagesSent;
extern uint64_t totalSafetyMessagesDelivered;

// Actions logging
extern std::ofstream actionsLog;

// RSU ID Mapping Pool
extern std::map<std::string, std::string> rsuIdMapping;
extern std::set<int> usedRsuSlots;
extern int maxRsuSlots;

// Vehicle ID Mapping Pool
extern std::map<std::string, std::string> vehicleIdMapping;
extern std::set<int> usedVehicleSlots;
extern int maxVehicleSlots;

// Flag to track if tracers are installed
extern bool tracersInstalled;

// NDN nodes
extern ns3::Ptr<ns3::Node> ndnMecNode;
extern ns3::NodeContainer rsuNdnNodes;
extern ns3::NodeContainer vehicleNdnNodes;

// UE node mapping (5G NR nodes used for UDP apps)
extern std::map<std::string, ns3::Ptr<ns3::Node>> ueNodeMapping;

// Global UE device containers
extern ns3::NodeContainer allUeNodes;
extern ns3::NetDeviceContainer allUeDevices;

// UE IPv4 addresses
extern std::map<std::string, ns3::Ipv4Address> ueIpAddresses;

// MEC IP address for NDN routing via 5G (used by FIB routes)
extern ns3::Ipv4Address mecIpFor5gNdn;

// V2V Direct Link Config and Stats (P2P emulation, not 3GPP PC5)
extern V2VDirectLinkConfig v2vDirectConfig;
extern V2VDirectLinkStats v2vStats;
// Legacy aliases
extern V2VDirectLinkConfig& pc5Config;
extern V2VDirectLinkStats& slStats;

// NDN node ID to name mapping (for metrics)
extern std::map<uint32_t, std::string> ndnNodeIdToName;

// NDN app mapping key "<nodeId>:<appId>" -> prefix (for latency categorization)
extern std::map<std::string, std::string> ndnAppIdToPrefix;

inline std::string
BuildNdnAppPrefixKey(uint32_t nodeId, uint32_t appId)
{
    return std::to_string(nodeId) + ":" + std::to_string(appId);
}

// ============================================================================
// NDN LINK TYPE CATEGORIZATION (for V2I/V2V metric separation)
// ============================================================================
// Determines whether NDN traffic is V2I (to/from MEC) or V2V (between vehicles)
// based on NDN name prefix. This replaces the inaccurate latency-based heuristic.

enum class NdnLinkType { V2I, V2V, UNKNOWN };

/**
 * @brief Get link type from NDN name prefix
 * 
 * V2I prefixes (traffic to/from MEC/RSU):
 *   - /v2x/v2i/...   : Explicit V2I prefix
 *   - /v2x/mec/...   : MEC-hosted content
 *   - /v2x/traffic/... : Traffic info from MEC
 *   - /v2x/safety/...  : Safety info from MEC
 *   - /v2x/count/...   : Vehicle count from MEC
 * 
 * V2V prefixes (traffic between vehicles):
 *   - /v2x/v2v/...   : Explicit V2V prefix
 *   - /v2x/position/... : Legacy position prefix (V2V)
 *   - /v2x/speed/...   : Legacy speed prefix (V2V)
 * 
 * @param prefix The NDN name prefix
 * @return NdnLinkType enum value
 */
inline NdnLinkType GetLinkTypeFromPrefix(const std::string& prefix)
{
    if (prefix.empty()) return NdnLinkType::UNKNOWN;
    
    // V2I prefixes (to/from MEC/infrastructure)
    // /v2x/safety (no trailing slash) is used by producers/consumers; treat as V2I.
    if (prefix.rfind("/v2x/v2i/", 0) == 0 ||
        prefix.rfind("/v2x/mec/", 0) == 0 ||
        prefix.rfind("/v2x/traffic/", 0) == 0 ||
        prefix == "/v2x/safety" ||
        prefix.rfind("/v2x/safety/", 0) == 0 ||
        prefix.rfind("/v2x/count/", 0) == 0)
    {
        return NdnLinkType::V2I;
    }
    
    // V2V prefixes (between vehicles)
    if (prefix.rfind("/v2x/v2v/", 0) == 0 ||
        prefix.rfind("/v2x/position/", 0) == 0 ||
        prefix.rfind("/v2x/speed/", 0) == 0)
    {
        return NdnLinkType::V2V;
    }
    
    return NdnLinkType::UNKNOWN;
}

/**
 * @brief Get link type name as string for logging/JSON
 */
inline std::string GetLinkTypeName(NdnLinkType type)
{
    switch (type) {
        case NdnLinkType::V2I: return "V2I";
        case NdnLinkType::V2V: return "V2V";
        default: return "UNKNOWN";
    }
}

// Global file streams for tracing
extern std::ofstream csTraceFile;
extern std::ofstream pitTraceFile;
extern std::ofstream fibTraceFile;
extern std::ofstream omnetDataFile;
extern std::ofstream ns3ToOmnetDataFile;
extern double currentSimTime;

// Runtime-configurable 5G NR numerology (can be set via --numerology=X)
extern uint32_t g_activeNumerology;

#endif // SIMULATION_STATE_H
