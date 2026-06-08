// IMPORTANT: Include ndnSIM FIRST to avoid namespace conflicts
#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder-counters.hpp"
#include "ns3/ndnSIM/NFD/daemon/face/face-endpoint.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"

#include "metrics_collector.h"
#include "simulation_state.h"
#include "ndn_position_cache.h"
#include "v2x_constants.h"

#include "ns3/config.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("MetricsCollector");

namespace
{
void WriteJsonDoubleOrNull(std::ostream& os, bool hasValue, double value)
{
    if (hasValue)
    {
        os << value;
    }
    else
    {
        os << "null";
    }
}
} // namespace

namespace {

std::map<uint64_t, Time> g_hoStartByImsi;

void
TraceHandoverStart(std::string /* context */, uint64_t imsi, uint16_t cellId, uint16_t /* rnti */,
                   uint16_t targetCellId)
{
    std::cout << "🔄 Handover START: IMSI=" << imsi << " from cell " << cellId << " to cell "
              << targetCellId << " at t=" << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_qosSummary.nr5g.handovers++;
    g_hoStartByImsi[imsi] = Simulator::Now();
}

void
TraceHandoverEndOk(std::string /* context */, uint64_t imsi, uint16_t cellId, uint16_t /* rnti */)
{
    std::cout << "✅ Handover END OK: IMSI=" << imsi << " now on cell " << cellId << " at t="
              << Simulator::Now().GetSeconds() << "s" << std::endl;

    g_qosSummary.nr5g.handover_completions++;
    const auto it = g_hoStartByImsi.find(imsi);
    if (it != g_hoStartByImsi.end())
    {
        const double ms = (Simulator::Now() - it->second).GetSeconds() * 1000.0;
        g_qosSummary.nr5g.handover_duration_total_ms += ms;
        g_qosSummary.nr5g.handover_duration_samples++;
        if (g_qosSummary.nr5g.handover_duration_samples == 1u)
        {
            g_qosSummary.nr5g.handover_duration_min_ms = ms;
            g_qosSummary.nr5g.handover_duration_max_ms = ms;
        }
        else
        {
            g_qosSummary.nr5g.handover_duration_min_ms =
                std::min(g_qosSummary.nr5g.handover_duration_min_ms, ms);
            g_qosSummary.nr5g.handover_duration_max_ms =
                std::max(g_qosSummary.nr5g.handover_duration_max_ms, ms);
        }
        g_hoStartByImsi.erase(it);
    }
    else
    {
        g_qosSummary.nr5g.handover_unpaired_end_ok++;
    }
}

void
TraceHandoverEndError(std::string /* context */, uint64_t imsi, uint16_t cellId, uint16_t /* rnti */)
{
    std::cout << "❌ Handover FAILED: IMSI=" << imsi << " cell " << cellId << " at t="
              << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_qosSummary.nr5g.handoverFailures++;
    (void)g_hoStartByImsi.erase(imsi);
}

} // namespace

void
ConnectNrHandoverTraces()
{
    static bool connected = false;
    if (connected)
    {
        return;
    }
    connected = true;

    bool ok = true;
    ok &= Config::ConnectFailSafe("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                                  MakeCallback(&TraceHandoverStart));
    ok &= Config::ConnectFailSafe("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                                  MakeCallback(&TraceHandoverEndOk));
    ok &= Config::ConnectFailSafe("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                                  MakeCallback(&TraceHandoverEndError));
    if (ok)
    {
        std::cout << "  ✓ NR handover traces connected (LteEnbRrc HandoverStart; LteUeRrc EndOk/EndError)"
                  << std::endl;
    }
    else
    {
        std::cerr << "  ⚠️  NR handover trace ConnectFailSafe failed (partial or no matches); "
                     "handover metrics may stay zero"
                  << std::endl;
    }
}

// Helper to print and log NDN stats (CS, PIT, FIB) + Throughput
void PrintNdnStats(double timestamp)
{
    std::cout << "\n📊 --- NDN + 5G + V2V Direct Links Stats at t=" << timestamp << "s ---" << std::endl;

    if (ndnMecNode)
    {
        auto ndnL3 = ndnMecNode->GetObject<ns3::ndn::L3Protocol>();
        if (ndnL3)
        {
            auto forwarder = ndnL3->getForwarder();

            // Content Store stats
            const auto& cs = forwarder->getCs();
            std::cout << "  MEC Content Store: " << cs.size() << " entries" << std::endl;

            // PIT stats
            const auto& pit = forwarder->getPit();
            std::cout << "  MEC PIT: " << pit.size() << " entries" << std::endl;

            // FIB stats
            const auto& fib = forwarder->getFib();
            std::cout << "  MEC FIB: " << fib.size() << " entries" << std::endl;

            // Update MEC node metrics
            nodeMetrics["MEC"].maxCsSize = std::max(nodeMetrics["MEC"].maxCsSize, (uint64_t)cs.size());
            nodeMetrics["MEC"].finalCsSize = cs.size();
            nodeMetrics["MEC"].maxPitSize = std::max(nodeMetrics["MEC"].maxPitSize, (uint64_t)pit.size());
            nodeMetrics["MEC"].finalPitSize = pit.size();
            nodeMetrics["MEC"].finalFibSize = fib.size();

            // Write data rows to custom trace files
            if (csTraceFile.is_open())
            {
                csTraceFile << timestamp << ",MEC,ContentStore," << cs.size() << std::endl;
            }
            if (pitTraceFile.is_open())
            {
                pitTraceFile << timestamp << ",MEC,PIT," << pit.size() << std::endl;
            }
            if (fibTraceFile.is_open())
            {
                fibTraceFile << timestamp << ",MEC,FIB," << fib.size() << std::endl;
            }
        }
    }

    // Also log per-vehicle NDN node stats if available
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        auto vehL3 = vehicleNdnNodes.Get(i)->GetObject<ns3::ndn::L3Protocol>();
        if (vehL3)
        {
            auto vehFw = vehL3->getForwarder();
            std::string vehName = "vehicle_" + std::to_string(i);
            
            const auto& vehCs = vehFw->getCs();
            const auto& vehPit = vehFw->getPit();
            const auto& vehFib = vehFw->getFib();

            nodeMetrics[vehName].maxCsSize = std::max(nodeMetrics[vehName].maxCsSize, (uint64_t)vehCs.size());
            nodeMetrics[vehName].finalCsSize = vehCs.size();
            nodeMetrics[vehName].maxPitSize = std::max(nodeMetrics[vehName].maxPitSize, (uint64_t)vehPit.size());
            nodeMetrics[vehName].finalPitSize = vehPit.size();
            nodeMetrics[vehName].finalFibSize = vehFib.size();

            if (csTraceFile.is_open())
            {
                csTraceFile << timestamp << "," << vehName << ",ContentStore," << vehCs.size() << std::endl;
            }
            if (pitTraceFile.is_open())
            {
                pitTraceFile << timestamp << "," << vehName << ",PIT," << vehPit.size() << std::endl;
            }
            if (fibTraceFile.is_open())
            {
                fibTraceFile << timestamp << "," << vehName << ",FIB," << vehFib.size() << std::endl;
            }
        }
    }

    // Aggregate NDN forwarder counters across ALL nodes (MEC + vehicles)
    uint64_t totalNdnInInterests = 0, totalNdnOutInterests = 0;
    uint64_t totalNdnInData = 0, totalNdnOutData = 0;
    uint64_t totalCsHits = 0, totalCsMisses = 0;
    uint64_t totalV2vInterests = 0, totalV2vData = 0;

    // MEC forwarder counters
    if (ndnMecNode)
    {
        auto mecL3 = ndnMecNode->GetObject<ns3::ndn::L3Protocol>();
        if (mecL3)
        {
            const auto& c = mecL3->getForwarder()->getCounters();
            totalNdnInInterests += c.nInInterests;
            totalNdnOutInterests += c.nOutInterests;
            totalNdnInData += c.nInData;
            totalNdnOutData += c.nOutData;
            totalCsHits += c.nCsHits;
            totalCsMisses += c.nCsMisses;
        }
    }

    // Vehicle forwarder counters (also counts V2V traffic)
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        auto vehL3 = vehicleNdnNodes.Get(i)->GetObject<ns3::ndn::L3Protocol>();
        if (vehL3)
        {
            const auto& c = vehL3->getForwarder()->getCounters();
            totalNdnInInterests += c.nInInterests;
            totalNdnOutInterests += c.nOutInterests;
            totalNdnInData += c.nInData;
            totalNdnOutData += c.nOutData;
            totalCsHits += c.nCsHits;
            totalCsMisses += c.nCsMisses;
            // Vehicle-originated interests/data = V2V traffic
            totalV2vInterests += c.nOutInterests;
            totalV2vData += c.nInData;
        }
    }

    // Update global stats from forwarder counters (authoritative source)
    g_v2xStats.ndnInterestsTx = totalNdnOutInterests;
    g_v2xStats.ndnDataRx = totalNdnInData;
    
    // Sync V2V stats: both v2vInterestsTx and sidelinkInterestsTx refer to the same metric
    slStats.v2vInterestsTx = totalV2vInterests;
    slStats.sidelinkInterestsTx = totalV2vInterests;
    slStats.v2vDataRx = totalV2vData;
    slStats.sidelinkDataRx = totalV2vData;
    slStats.v2vMessages = totalV2vInterests + totalV2vData;
    
    // Sync resource allocations count (number of P2P links = number of direct connections)
    slStats.resourceAllocations = slStats.directLinkConnections;

    // Print V2X throughput stats
    std::cout << "  V2X UDP Stats:" << std::endl;
    std::cout << "    - TX Packets: " << g_v2xStats.udpPacketsTx << " (" << g_v2xStats.udpBytesTx << " bytes)" << std::endl;
    std::cout << "    - RX Packets: " << g_v2xStats.udpPacketsRx << " (" << g_v2xStats.udpBytesRx << " bytes)" << std::endl;

    // Print V2V Direct Links stats
    std::cout << "  V2V Direct Links Stats:" << std::endl;
    std::cout << "    - Resource Allocations: " << slStats.resourceAllocations << std::endl;
    std::cout << "    - V2V Messages: " << slStats.v2vMessages << std::endl;
    std::cout << "    - V2V Interests TX: " << slStats.v2vInterestsTx << std::endl;
    std::cout << "    - V2V Data RX: " << slStats.v2vDataRx << std::endl;

    // Print NDN Forwarder stats (from actual NFD counters)
    std::cout << "  NDN Forwarder Stats (all nodes):" << std::endl;
    std::cout << "    - Interests In: " << totalNdnInInterests << std::endl;
    std::cout << "    - Interests Out: " << totalNdnOutInterests << std::endl;
    std::cout << "    - Data In: " << totalNdnInData << std::endl;
    std::cout << "    - Data Out: " << totalNdnOutData << std::endl;
    std::cout << "    - CS Hits: " << totalCsHits << std::endl;
    std::cout << "    - CS Misses: " << totalCsMisses << std::endl;

    // Print safety stats
    std::cout << "  Safety Stats:" << std::endl;
    std::cout << "    - Collision Warnings: " << safetyStats.collisionWarnings << std::endl;
    std::cout << "    - Slow Down Commands: " << safetyStats.slowDownCommands << std::endl;
    std::cout << "    - Position Updates: " << safetyStats.positionUpdates << std::endl;

    std::cout << "  -------------------------------------------" << std::endl;
}

// Static file for JSON logging
static std::ofstream omnetJsonLog;
static bool omnetJsonLogInitialized = false;
static uint64_t omnetMsgCounter = 0;
static std::ofstream ns3ToOmnetJsonLog;
static bool ns3ToOmnetJsonLogInitialized = false;
static uint64_t ns3ToOmnetMsgCounter = 0;
static std::string outputDir = "";

// Get output directory from environment or use default
static std::string GetOutputDir()
{
    if (outputDir.empty())
    {
        const char* envDir = std::getenv("V2X_RESULTS_DIR");
        if (envDir != nullptr && strlen(envDir) > 0)
        {
            outputDir = std::string(envDir);
            // Ensure trailing slash
            if (outputDir.back() != '/')
                outputDir += "/";
            std::cout << "📁 [LOG] Output directory: " << outputDir << std::endl;
        }
        else
        {
            outputDir = "./";  // Default to current directory
        }
    }
    return outputDir;
}

static bool LooksLikeJson(const std::string& value)
{
    for (char ch : value)
    {
        if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r')
        {
            continue;
        }
        return ch == '{' || ch == '[';
    }
    return false;
}

static std::string EscapeJsonString(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size() + 16);
    for (char ch : value)
    {
        switch (ch)
        {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += ch; break;
        }
    }
    return escaped;
}

// Helper to log data received from OMNeT++
void LogOmnetData(const std::string& msgType, double timestamp, const std::string& rawJson)
{
    omnetMsgCounter++;
    
    // Log to text file (human readable)
    if (omnetDataFile.is_open())
    {
        omnetDataFile << "========================================" << std::endl;
        omnetDataFile << "[MSG #" << omnetMsgCounter << "] Time: " << std::fixed << std::setprecision(3) << timestamp << "s" << std::endl;
        omnetDataFile << "Type: " << msgType << std::endl;
        omnetDataFile << "Raw JSON:" << std::endl;
        omnetDataFile << rawJson << std::endl;
        omnetDataFile << "========================================" << std::endl << std::endl;
        omnetDataFile.flush();
    }
    
    // Also log to JSON Lines file for easy parsing
    if (!omnetJsonLogInitialized)
    {
        std::string jsonLogPath = GetOutputDir() + "omnet_messages.jsonl";
        omnetJsonLog.open(jsonLogPath, std::ios::out | std::ios::trunc);
        if (omnetJsonLog.is_open())
        {
            std::cout << "📝 [LOG] Streaming OMNeT++ messages to: " << jsonLogPath << std::endl;
        }
        omnetJsonLogInitialized = true;
    }
    
    if (omnetJsonLog.is_open())
    {
        // Write as JSON Lines format (one JSON object per line)
        omnetJsonLog << "{\"msg_id\":" << omnetMsgCounter 
                     << ",\"type\":\"" << msgType << "\""
                     << ",\"timestamp\":" << std::fixed << std::setprecision(6) << timestamp
                     << ",\"data\":" << rawJson << "}" << std::endl;
        omnetJsonLog.flush();
    }
    
    // Log summary to console for important message types
    if (msgType == "accident_report")
    {
        std::cout << "📝 [LOG] Accident report logged (msg #" << omnetMsgCounter << ")" << std::endl;
    }
    else if (msgType == "traffic_update")
    {
        std::cout << "📝 [LOG] Traffic update logged (msg #" << omnetMsgCounter << ")" << std::endl;
    }
}

// Helper to log data sent from NS-3 to OMNeT++
void LogNs3ToOmnetData(const std::string& msgType, double timestamp, const std::string& rawPayload)
{
    ns3ToOmnetMsgCounter++;

    if (ns3ToOmnetDataFile.is_open())
    {
        ns3ToOmnetDataFile << "========================================" << std::endl;
        ns3ToOmnetDataFile << "[MSG #" << ns3ToOmnetMsgCounter << "] Time: " << std::fixed << std::setprecision(3)
                           << timestamp << "s" << std::endl;
        ns3ToOmnetDataFile << "Type: " << msgType << std::endl;
        ns3ToOmnetDataFile << "Raw Payload:" << std::endl;
        ns3ToOmnetDataFile << rawPayload << std::endl;
        ns3ToOmnetDataFile << "========================================" << std::endl << std::endl;
        ns3ToOmnetDataFile.flush();
    }

    if (!ns3ToOmnetJsonLogInitialized)
    {
        std::string jsonLogPath = GetOutputDir() + "ns3_to_omnet_messages.jsonl";
        ns3ToOmnetJsonLog.open(jsonLogPath, std::ios::out | std::ios::trunc);
        if (ns3ToOmnetJsonLog.is_open())
        {
            std::cout << "📝 [LOG] Streaming NS-3 outbound messages to: " << jsonLogPath << std::endl;
        }
        ns3ToOmnetJsonLogInitialized = true;
    }

    if (ns3ToOmnetJsonLog.is_open())
    {
        ns3ToOmnetJsonLog << "{\"msg_id\":" << ns3ToOmnetMsgCounter
                          << ",\"type\":\"" << msgType << "\""
                          << ",\"timestamp\":" << std::fixed << std::setprecision(6) << timestamp
                          << ",\"data\":";
        if (LooksLikeJson(rawPayload))
        {
            ns3ToOmnetJsonLog << rawPayload;
        }
        else
        {
            ns3ToOmnetJsonLog << "\"" << EscapeJsonString(rawPayload) << "\"";
        }
        ns3ToOmnetJsonLog << "}" << std::endl;
        ns3ToOmnetJsonLog.flush();
    }
}

// Close OMNeT++ log files
void CloseOmnetLogs()
{
    if (omnetJsonLog.is_open())
    {
        omnetJsonLog.close();
        std::cout << "📊 [LOG] OMNeT++ JSON log closed. Total messages: " << omnetMsgCounter << std::endl;
    }
    if (ns3ToOmnetJsonLog.is_open())
    {
        ns3ToOmnetJsonLog.close();
        std::cout << "📊 [LOG] NS-3 outbound JSON log closed. Total messages: " << ns3ToOmnetMsgCounter << std::endl;
    }
}

// Get total OMNeT++ message count
uint64_t GetOmnetMessageCount()
{
    return omnetMsgCounter;
}

// Parse rate-trace.txt for Interest/Data packet counts
void ParseRateTrace()
{
    std::ifstream rateFile("rate-trace.txt");
    if (!rateFile.is_open())
    {
        std::cerr << "Warning: rate-trace.txt not found" << std::endl;
        return;
    }

    std::string line;
    std::getline(rateFile, line); // Skip header

    while (std::getline(rateFile, line))
    {
        std::istringstream iss(line);
        double time;
        std::string node, faceId, faceDescr, type, packets, kilobytes, packetsRaw, kilobytesRaw;

        if (iss >> time >> node >> faceId >> faceDescr >> type >> packets >> kilobytes >> packetsRaw >> kilobytesRaw)
        {
            uint64_t pktCount = std::stoull(packets);
            uint64_t bytes = static_cast<uint64_t>(std::stod(kilobytes) * 1024);

            if (type == "OutInterests")
            {
                nodeMetrics[node].interestsSent += pktCount;
            }
            else if (type == "InInterests")
            {
                nodeMetrics[node].interestsReceived += pktCount;
            }
            else if (type == "OutData")
            {
                nodeMetrics[node].dataSent += pktCount;
                nodeMetrics[node].totalBytes += bytes;
            }
            else if (type == "InData")
            {
                nodeMetrics[node].dataReceived += pktCount;
                nodeMetrics[node].totalBytes += bytes;
            }
            else if (type == "SatisfiedInterests")
            {
                nodeMetrics[node].interestsSatisfied += pktCount;
            }
            else if (type == "TimedOutInterests")
            {
                nodeMetrics[node].interestsTimedOut += pktCount;
            }
        }
    }
    rateFile.close();
}

// Parse ndn-cs-trace.txt for cache hits/misses
void ParseCsTrace()
{
    // Cache metrics are now tracked directly via CS signals
    // This function is kept for backward compatibility
    std::cout << "  Cache metrics are tracked via CS signals" << std::endl;
}

// Helper to check if prefix is V2I (to MEC/RSU)
// NOTE: Using GetLinkTypeFromPrefix() from simulation_state.h is preferred
// These functions kept for backward compatibility with existing code
static bool IsV2iPrefix(const std::string& prefix)
{
    return GetLinkTypeFromPrefix(prefix) == NdnLinkType::V2I;
}

// Helper to check if prefix is V2V (between vehicles)
static bool IsV2vPrefix(const std::string& prefix)
{
    return GetLinkTypeFromPrefix(prefix) == NdnLinkType::V2V;
}

// Parse app-delays-trace.txt for latency with V2I/V2V separation
void ParseAppDelayTrace()
{
// =================================================================
// LATENCY MEASUREMENT METHODOLOGY (IEEE TMC/TVT publication)
// =================================================================
// All latency values are end-to-end NDN application-layer RTT,
// measured by ndnSIM AppDelayTracer (Interest TX to Data RX at app).
//
// V2I latency composition:
//   NDN forwarder processing
//   + UDP/IP encapsulation (V2iIpv4UdpTransport)
//   + 5G NR Uu scheduling and transmission (NrUeNetDevice)
//   + EPC traversal (gNB to PGW)
//   + MEC P2P link (PGW to MEC, 10Gbps/1ms)
//   + MEC NDN producer response time
//   + full return path (symmetric)
//
// V2V latency composition:
//   NDN forwarder processing
//   + P2P emulated direct link (100Mbps, 1ms fixed delay)
//   NOTE: P2P emulation only — not 3GPP PC5 sidelink.
//   Valid for NDN protocol evaluation; not valid for PC5 claims.
//
// Categorization: prefix-based via GetLinkTypeFromPrefix()
//   V2I: /v2x/v2i/, /v2x/traffic/, /v2x/safety/, /v2x/mec/
//   V2V: /v2x/v2v/, /v2x/position/, /v2x/speed/
//
// Cross-validation: NdnPositionCache RTT (MEC ConsumerCbr
//   Interest-to-Data) exported as v2i_mec_position_query_rtt_avg_ms.
//   Both measurements should converge for /v2x/v2i/ prefixes.
// =================================================================

    std::ifstream delayFile("app-delays-trace.txt");
    if (!delayFile.is_open())
    {
        std::cerr << "Warning: app-delays-trace.txt not found" << std::endl;
        return;
    }

    std::string line;
    std::getline(delayFile, line); // Skip header

    // Temporary accumulators
    double v2iTotalLatencyMs = 0.0;
    uint64_t v2iSamples = 0;
    double v2vTotalLatencyMs = 0.0;
    uint64_t v2vSamples = 0;
    uint64_t unknownSamples = 0;  // Track samples with unknown link type
    double v2iMinMs = std::numeric_limits<double>::max();
    double v2iMaxMs = 0.0;
    double v2vMinMs = std::numeric_limits<double>::max();
    double v2vMaxMs = 0.0;
    std::vector<double> v2iLatencies;
    std::vector<double> v2vLatencies;

    while (std::getline(delayFile, line))
    {
        std::istringstream iss(line);
        double time;
        std::string node, seqNo, type;
        uint32_t appId = 0;
        double delayS, delayUs, retxCount, hopCount;

        if (iss >> time >> node >> appId >> seqNo >> type >> delayS >> delayUs >> retxCount >> hopCount)
        {
            // AppDelayTracer logs both LastDelay (post-retx) and FullDelay (first answer).
            // Counting both doubles samples; qos_summary uses FullDelay only (first RTT).
            if (type != "FullDelay") continue;

            double latencyMs = delayS * 1000.0 + delayUs / 1000.0;

            // Skip invalid rows only. Very small (including ~0 ms) RTTs can occur for fast
            // paths; excluding latency<=0 dropped real V2I samples when delayS/delayUs
            // rounded to zero in the trace.
            if (latencyMs <= 0.0) continue;
            
            nodeMetrics[node].totalLatencyMs += latencyMs;
            nodeMetrics[node].latencySamples++;

            // Categorize by prefix
            uint32_t nodeId = 0;
            try
            {
                nodeId = static_cast<uint32_t>(std::stoul(node));
            }
            catch (const std::exception&)
            {
                nodeId = 0;
            }

            const std::string appPrefixKey = BuildNdnAppPrefixKey(nodeId, appId);
            auto prefixIt = ndnAppIdToPrefix.find(appPrefixKey);
            std::string prefix = "";
            if (prefixIt != ndnAppIdToPrefix.end())
            {
                prefix = prefixIt->second;
            }
            
            // Categorize V2I vs V2V using prefix-based classification
            // NOTE: Previous latency heuristic (>3ms = V2I) removed - ~50% inaccurate
            // because V2I and V2V latency ranges overlap significantly.
            NdnLinkType linkType = GetLinkTypeFromPrefix(prefix);
            
            // If prefix is empty/unmapped but we have a node:appId, try to log debug info
            if (linkType == NdnLinkType::UNKNOWN && !prefix.empty())
            {
                NS_LOG_DEBUG("Unknown link type for prefix: " << prefix << " (node=" << node << ", appId=" << appId << ")");
            }

            switch (linkType)
            {
                case NdnLinkType::V2I:
                    v2iTotalLatencyMs += latencyMs;
                    v2iSamples++;
                    v2iLatencies.push_back(latencyMs);
                    if (latencyMs < v2iMinMs) v2iMinMs = latencyMs;
                    if (latencyMs > v2iMaxMs) v2iMaxMs = latencyMs;
                    g_qosSummary.v2iLatency.AddSample(latencyMs);
                    break;
                    
                case NdnLinkType::V2V:
                    v2vTotalLatencyMs += latencyMs;
                    v2vSamples++;
                    v2vLatencies.push_back(latencyMs);
                    if (latencyMs < v2vMinMs) v2vMinMs = latencyMs;
                    if (latencyMs > v2vMaxMs) v2vMaxMs = latencyMs;
                    g_qosSummary.v2vLatency.AddSample(latencyMs);
                    break;
                    
                case NdnLinkType::UNKNOWN:
                default:
                    // Track uncategorized samples for diagnostics
                    unknownSamples++;
                    break;
            }
        }
    }
    delayFile.close();

    // Update V2V latency for backward compatibility
    if (v2vSamples > 0)
    {
        g_qosSummary.nr5g.v2vAppLatencyMs = v2vTotalLatencyMs / v2vSamples;
    }
    // NR block: mirror mean V2I AppDelay RTT (E2E incl. NDN, NR, EPC, MEC producer); omit from JSON if no samples.
    if (g_qosSummary.v2iLatency.sampleCount > 0)
    {
        g_qosSummary.nr5g.edgeLatencyMs = g_qosSummary.v2iLatency.GetAverageLatency();
    }

    std::cout << "  📡 V2I Latency (AppDelay trace, prefix-classified → qos_summary): ";
    if (v2iSamples > 0)
    {
        std::cout << "avg=" << (v2iTotalLatencyMs / v2iSamples) << "ms, "
                  << "min=" << v2iMinMs << "ms, max=" << v2iMaxMs << "ms "
                  << "(" << v2iSamples << " samples)" << std::endl;
    }
    else
    {
        std::cout << "no V2I-classified AppDelay rows" << std::endl;
    }

    std::cout << "  🚗 V2V Latency: ";
    if (v2vSamples > 0)
    {
        std::cout << "avg=" << (v2vTotalLatencyMs / v2vSamples) << "ms, "
                  << "min=" << v2vMinMs << "ms, max=" << v2vMaxMs << "ms "
                  << "(" << v2vSamples << " samples)" << std::endl;
    }
    else
    {
        std::cout << "no samples" << std::endl;
    }
    
    // Log categorization method and unknown samples for transparency
    std::cout << "  📊 Categorization: prefix_based (V2I: /v2x/v2i/, /v2x/traffic/, /v2x/safety/; V2V: /v2x/v2v/)" << std::endl;
    if (unknownSamples > 0)
    {
        std::cout << "  ⚠️  Unknown link type: " << unknownSamples << " samples (unregistered app prefixes)" << std::endl;
    }
}

// Collect metrics from all trace files
void CollectMetricsFromTracers()
{
    std::cout << "\n📊 Collecting metrics from tracers..." << std::endl;
    // AppDelayTracer::Destroy() is invoked from simple_ndn.cc after the simulation loop exits
    // (flush/close trace files). Do not call it here — mid-run periodic snapshots would free
    // tracer state while NDN callbacks are still active.
    ParseRateTrace();
    ParseCsTrace();
    ParseAppDelayTrace();

    // AppDelayTracer often omits MEC node and ConsumerCacheFriendly FullDelay rows; MEC still
    // measures true V2I Interest→Data RTT in NdnPositionCache (/v2x/v2i/vehicle/*/position).
    if (g_qosSummary.v2iLatency.sampleCount == 0)
    {
        const auto& ndnSt = v2x::ndn_cache::NdnPositionCache::GetInstance().GetStats();
        if (ndnSt.rttSamples > 0)
        {
            const double avgMs = ndnSt.GetAvgRttMs();
            g_qosSummary.v2iLatency.minLatencyMs = avgMs;
            g_qosSummary.v2iLatency.maxLatencyMs = avgMs;
            g_qosSummary.v2iLatency.totalLatencyMs = ndnSt.totalRttMs;
            g_qosSummary.v2iLatency.sampleCount = ndnSt.rttSamples;
            g_qosSummary.v2iLatency.latencySamples.clear();
            g_qosSummary.v2iLatency.latencySamples.push_back(avgMs);
            g_qosSummary.nr5g.edgeLatencyMs = avgMs;
            std::cout << "  📡 V2I Latency (NdnPositionCache /v2x/v2i/ MEC query RTT — AppDelay had 0 "
                         "V2I rows): avg="
                      << avgMs << " ms, samples=" << ndnSt.rttSamples << std::endl;
        }
    }

    if (g_qosSummary.v2iLatency.sampleCount > 0)
    {
        std::cout << "  📡 V2I Latency (qos_summary): avg="
                  << g_qosSummary.v2iLatency.GetAverageLatency() << " ms, samples="
                  << g_qosSummary.v2iLatency.sampleCount << std::endl;
    }
    std::cout << "  ✓ Metrics collected" << std::endl;
}

// ============================================================================
// QoS METRICS IMPLEMENTATION
// ============================================================================

// Global QoS Summary
QoSSummary g_qosSummary;

// Global V2I and V2V throughput trackers
TypedThroughputMetrics v2iThroughput;
TypedThroughputMetrics v2vThroughput;

// Calculate jitter (standard deviation of latency)
double LatencyMetrics::GetJitter() const
{
    if (sampleCount < 2) return 0.0;
    
    double mean = GetAverageLatency();
    double sumSquaredDiff = 0.0;
    
    for (double sample : latencySamples)
    {
        double diff = sample - mean;
        sumSquaredDiff += diff * diff;
    }
    
    return std::sqrt(sumSquaredDiff / (sampleCount - 1));
}

// Calculate percentile
double LatencyMetrics::GetPercentile(double p) const
{
    if (latencySamples.empty()) return 0.0;
    
    std::vector<double> sorted = latencySamples;
    std::sort(sorted.begin(), sorted.end());
    
    double index = (p / 100.0) * (sorted.size() - 1);
    size_t lower = static_cast<size_t>(std::floor(index));
    size_t upper = static_cast<size_t>(std::ceil(index));
    
    if (lower == upper) return sorted[lower];
    
    double fraction = index - lower;
    return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

// V2I Latency Metrics - Jitter
double V2iLatencyMetrics::GetJitter() const
{
    if (sampleCount < 2) return 0.0;
    
    double mean = GetAverageLatency();
    double sumSquaredDiff = 0.0;
    
    for (double sample : latencySamples)
    {
        double diff = sample - mean;
        sumSquaredDiff += diff * diff;
    }
    
    return std::sqrt(sumSquaredDiff / (sampleCount - 1));
}

// V2I Latency Metrics - Percentile
double V2iLatencyMetrics::GetPercentile(double p) const
{
    if (latencySamples.empty()) return 0.0;
    
    std::vector<double> sorted = latencySamples;
    std::sort(sorted.begin(), sorted.end());
    
    double index = (p / 100.0) * (sorted.size() - 1);
    size_t lower = static_cast<size_t>(std::floor(index));
    size_t upper = static_cast<size_t>(std::ceil(index));
    
    if (lower == upper) return sorted[lower];
    
    double fraction = index - lower;
    return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

// V2V Latency Metrics - Jitter
double V2vLatencyMetrics::GetJitter() const
{
    if (sampleCount < 2) return 0.0;
    
    double mean = GetAverageLatency();
    double sumSquaredDiff = 0.0;
    
    for (double sample : latencySamples)
    {
        double diff = sample - mean;
        sumSquaredDiff += diff * diff;
    }
    
    return std::sqrt(sumSquaredDiff / (sampleCount - 1));
}

// V2V Latency Metrics - Percentile
double V2vLatencyMetrics::GetPercentile(double p) const
{
    if (latencySamples.empty()) return 0.0;
    
    std::vector<double> sorted = latencySamples;
    std::sort(sorted.begin(), sorted.end());
    
    double index = (p / 100.0) * (sorted.size() - 1);
    size_t lower = static_cast<size_t>(std::floor(index));
    size_t upper = static_cast<size_t>(std::ceil(index));
    
    if (lower == upper) return sorted[lower];
    
    double fraction = index - lower;
    return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

// Calculate peak throughput
double ThroughputMetrics::GetPeakThroughputMbps() const
{
    if (instantThroughputs.empty()) return 0.0;
    return *std::max_element(instantThroughputs.begin(), instantThroughputs.end());
}

// Record a latency sample
void RecordLatencySample(double latencyMs, const std::string& category)
{
    // Update global E2E latency
    g_qosSummary.e2eLatency.totalLatencyMs += latencyMs;
    g_qosSummary.e2eLatency.sampleCount++;
    g_qosSummary.e2eLatency.latencySamples.push_back(latencyMs);
    
    if (latencyMs < g_qosSummary.e2eLatency.minLatencyMs)
        g_qosSummary.e2eLatency.minLatencyMs = latencyMs;
    if (latencyMs > g_qosSummary.e2eLatency.maxLatencyMs)
        g_qosSummary.e2eLatency.maxLatencyMs = latencyMs;
    
    // Category-specific tracking
    if (category == "safety" || category == "bsm")
    {
        g_qosSummary.safety.bsmLatency.totalLatencyMs += latencyMs;
        g_qosSummary.safety.bsmLatency.sampleCount++;
        g_qosSummary.safety.bsmLatency.latencySamples.push_back(latencyMs);
    }
    else if (category == "emergency")
    {
        g_qosSummary.safety.emergencyLatency.totalLatencyMs += latencyMs;
        g_qosSummary.safety.emergencyLatency.sampleCount++;
        g_qosSummary.safety.emergencyLatency.latencySamples.push_back(latencyMs);
    }
    else if (category == "v2i")
    {
        // Measured when Data actually reaches the app (ConsumerCacheFriendly) or MEC position cache RTT.
        // AppDelayTracer often omits these paths in the V2X stack; do not rely on app-delays-trace.txt alone.
        g_qosSummary.v2iLatency.AddSample(latencyMs);
    }
}

void RecordV2iAppLayerRttSample(double latencyMs)
{
    // Exclude zero-latency local CS hits — these are served from
    // the vehicle's own Content Store without any network transmission.
    // Only record samples that traversed the 5G NR network.
    if (latencyMs <= 0.0) return;
    RecordLatencySample(latencyMs, "v2i");
}

// Record packet delivery
void RecordPacketDelivery(bool delivered, const std::string& category)
{
    g_qosSummary.delivery.packetsSent++;
    
    if (delivered)
    {
        g_qosSummary.delivery.packetsReceived++;
    }
    else
    {
        g_qosSummary.delivery.packetsLost++;
    }
    
    // Category-specific tracking
    if (category == "bsm")
    {
        g_qosSummary.safety.bsmSent++;
        if (delivered) g_qosSummary.safety.bsmReceived++;
    }
    else if (category == "collision_warning")
    {
        g_qosSummary.safety.collisionWarningsSent++;
        if (delivered) g_qosSummary.safety.collisionWarningsReceived++;
    }
    else if (category == "emergency")
    {
        g_qosSummary.safety.emergencyMsgSent++;
        if (delivered) g_qosSummary.safety.emergencyMsgReceived++;
    }
    else if (category == "pc5" || category == "v2v_emulated_link")
    {
        // Legacy category "pc5" kept for compatibility; not 3GPP PC5.
        g_qosSummary.nr5g.v2vEmulatedLinkMessagesSent++;
        if (delivered)
        {
            g_qosSummary.nr5g.v2vEmulatedLinkMessagesReceived++;
        }
    }
}

// Record cache access
void RecordCacheAccess(bool hit, bool proactive, double latencyMs)
{
    if (hit)
    {
        g_qosSummary.caching.cacheHits++;
        if (proactive)
        {
            g_qosSummary.caching.proactiveCacheHits++;
        }
        else
        {
            g_qosSummary.caching.reactiveCacheHits++;
        }
        
        // Update cache latency
        double totalHitLatency = g_qosSummary.caching.avgCacheLatencyMs * 
                                 (g_qosSummary.caching.cacheHits - 1);
        g_qosSummary.caching.avgCacheLatencyMs = 
            (totalHitLatency + latencyMs) / g_qosSummary.caching.cacheHits;
    }
    else
    {
        g_qosSummary.caching.cacheMisses++;
        
        // Update origin latency
        double totalMissLatency = g_qosSummary.caching.avgOriginLatencyMs * 
                                  (g_qosSummary.caching.cacheMisses - 1);
        g_qosSummary.caching.avgOriginLatencyMs = 
            (totalMissLatency + latencyMs) / g_qosSummary.caching.cacheMisses;
    }
}

// Print QoS Summary
void PrintQoSSummary()
{
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  📊 QoS METRICS SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // End-to-End Latency
    std::cout << "\n📈 End-to-End Latency:" << std::endl;
    std::cout << "   Average: " << std::fixed << std::setprecision(3) 
              << g_qosSummary.e2eLatency.GetAverageLatency() << " ms" << std::endl;
    std::cout << "   Min: " << g_qosSummary.e2eLatency.minLatencyMs << " ms" << std::endl;
    std::cout << "   Max: " << g_qosSummary.e2eLatency.maxLatencyMs << " ms" << std::endl;
    std::cout << "   Jitter: " << g_qosSummary.e2eLatency.GetJitter() << " ms" << std::endl;
    std::cout << "   P50: " << g_qosSummary.e2eLatency.GetPercentile(50) << " ms" << std::endl;
    std::cout << "   P95: " << g_qosSummary.e2eLatency.GetPercentile(95) << " ms" << std::endl;
    std::cout << "   P99: " << g_qosSummary.e2eLatency.GetPercentile(99) << " ms" << std::endl;
    
    // Packet Delivery
    std::cout << "\n📦 Packet Delivery:" << std::endl;
    std::cout << "   Packets Sent: " << g_qosSummary.delivery.packetsSent << std::endl;
    std::cout << "   Packets Received: " << g_qosSummary.delivery.packetsReceived << std::endl;
    std::cout << "   Packets Lost: " << g_qosSummary.delivery.packetsLost << std::endl;
    std::cout << "   PDR: " << std::fixed << std::setprecision(2) 
              << g_qosSummary.delivery.GetPDR() << "%" << std::endl;
    std::cout << "   Loss Rate: " << g_qosSummary.delivery.GetPacketLossRate() << "%" << std::endl;
    
    // Throughput
    std::cout << "\n📶 Throughput:" << std::endl;
    std::cout << "   Total TX: " << g_qosSummary.throughput.totalBytesTx / 1024 << " KB" << std::endl;
    std::cout << "   Total RX: " << g_qosSummary.throughput.totalBytesRx / 1024 << " KB" << std::endl;
    std::cout << "   Avg Throughput: " << std::fixed << std::setprecision(3)
              << g_qosSummary.throughput.GetAverageThroughputMbps() << " Mbps" << std::endl;
    std::cout << "   Peak Throughput: " << g_qosSummary.throughput.GetPeakThroughputMbps() 
              << " Mbps" << std::endl;
    
    // NDN Caching
    std::cout << "\n💾 NDN Caching:" << std::endl;
    std::cout << "   Cache Hits: " << g_qosSummary.caching.cacheHits << std::endl;
    std::cout << "   Cache Misses: " << g_qosSummary.caching.cacheMisses << std::endl;
    std::cout << "   Hit Rate: " << std::fixed << std::setprecision(2) 
              << g_qosSummary.caching.GetHitRate() << "%" << std::endl;
    std::cout << "   Proactive Hits: " << g_qosSummary.caching.proactiveCacheHits << std::endl;
    std::cout << "   Reactive Hits: " << g_qosSummary.caching.reactiveCacheHits << std::endl;
    std::cout << "   Avg Cache Latency: " << g_qosSummary.caching.avgCacheLatencyMs << " ms" << std::endl;
    std::cout << "   Avg Origin Latency: " << g_qosSummary.caching.avgOriginLatencyMs << " ms" << std::endl;
    std::cout << "   Latency Improvement: " << g_qosSummary.caching.GetLatencyImprovement() 
              << "%" << std::endl;
    
    // Safety QoS
    std::cout << "\n🚗 V2X Safety QoS:" << std::endl;
    std::cout << "   BSM Sent/Received: " << g_qosSummary.safety.bsmSent << "/" 
              << g_qosSummary.safety.bsmReceived << std::endl;
    std::cout << "   Collision Warnings: " << g_qosSummary.safety.collisionWarningsSent << "/" 
              << g_qosSummary.safety.collisionWarningsReceived << std::endl;
    std::cout << "   Emergency Messages: " << g_qosSummary.safety.emergencyMsgSent << "/" 
              << g_qosSummary.safety.emergencyMsgReceived << std::endl;
    std::cout << "   Avg BSM Latency: " << g_qosSummary.safety.bsmLatency.GetAverageLatency() 
              << " ms" << std::endl;
    std::cout << "   Avg Emergency Latency: " << g_qosSummary.safety.emergencyLatency.GetAverageLatency() 
              << " ms" << std::endl;
    
    // 5G NR Metrics
    std::cout << "\n📡 5G NR Metrics:" << std::endl;
    std::cout << "   V2V emulated-link msgs: " << g_qosSummary.nr5g.v2vEmulatedLinkMessagesSent << "/"
              << g_qosSummary.nr5g.v2vEmulatedLinkMessagesReceived << std::endl;
    std::cout << "   V2V app-layer latency (AppDelay, P2P): " << g_qosSummary.nr5g.v2vAppLatencyMs
              << " ms" << std::endl;
    std::cout << "   Handover starts: " << g_qosSummary.nr5g.handovers << std::endl;
    std::cout << "   Handover completions (EndOk): " << g_qosSummary.nr5g.handover_completions
              << std::endl;
    std::cout << "   Handover failures: " << g_qosSummary.nr5g.handoverFailures << std::endl;
    if (g_qosSummary.nr5g.handover_unpaired_end_ok > 0)
    {
        std::cout << "   Handover EndOk without matching Start: " << g_qosSummary.nr5g.handover_unpaired_end_ok
                  << std::endl;
    }
    if (g_qosSummary.nr5g.handover_duration_samples > 0)
    {
        const double avgHoMs =
            g_qosSummary.nr5g.handover_duration_total_ms /
            static_cast<double>(g_qosSummary.nr5g.handover_duration_samples);
        std::cout << "   Handover duration (paired Start→EndOk): avg=" << std::fixed << std::setprecision(2)
                  << avgHoMs << " ms, min=" << g_qosSummary.nr5g.handover_duration_min_ms
                  << " ms, max=" << g_qosSummary.nr5g.handover_duration_max_ms << " ms (n="
                  << g_qosSummary.nr5g.handover_duration_samples << ")" << std::endl;
    }
    
    std::cout << std::string(70, '=') << std::endl;
}

// Generate comprehensive QoS report
void GenerateQoSReport(const std::string& filename)
{
    // Collect NDN delivery metrics before generating report
    CollectNdnDeliveryMetrics();
    
    ValidateCriticalMetrics("GenerateQoSReport");

    std::ofstream qosFile(filename);
    if (!qosFile.is_open())
    {
        std::cerr << "Error: Cannot create QoS report " << filename << std::endl;
        return;
    }
    
    qosFile << "{\n";
    qosFile << "  \"qos_report\": {\n";
    qosFile << "    \"simulation_duration_s\": " << g_qosSummary.simulationDuration << ",\n";
    qosFile << "    \"total_nodes\": " << g_qosSummary.totalNodes << ",\n";
    qosFile << "    \"active_vehicles\": " << g_qosSummary.activeVehicles << ",\n";
    qosFile << "    \"active_rsus\": " << g_qosSummary.activeRsus << ",\n";
    
    // E2E Latency
    qosFile << "    \"e2e_latency\": {\n";
    qosFile << "      \"avg_ms\": " << std::fixed << std::setprecision(3) 
            << g_qosSummary.e2eLatency.GetAverageLatency() << ",\n";
    qosFile << "      \"min_ms\": " << g_qosSummary.e2eLatency.minLatencyMs << ",\n";
    qosFile << "      \"max_ms\": " << g_qosSummary.e2eLatency.maxLatencyMs << ",\n";
    qosFile << "      \"jitter_ms\": " << g_qosSummary.e2eLatency.GetJitter() << ",\n";
    qosFile << "      \"p50_ms\": " << g_qosSummary.e2eLatency.GetPercentile(50) << ",\n";
    qosFile << "      \"p95_ms\": " << g_qosSummary.e2eLatency.GetPercentile(95) << ",\n";
    qosFile << "      \"p99_ms\": " << g_qosSummary.e2eLatency.GetPercentile(99) << ",\n";
    qosFile << "      \"sample_count\": " << g_qosSummary.e2eLatency.sampleCount << "\n";
    qosFile << "    },\n";
    
    // V2I Latency (through 5G NR to MEC)
    const bool hasV2iSamples = g_qosSummary.v2iLatency.sampleCount > 0;
    qosFile << "    \"v2i_latency\": {\n";
    qosFile << "      \"avg_ms\": " << std::fixed << std::setprecision(3)
            << g_qosSummary.v2iLatency.GetAverageLatency() << ",\n";
    qosFile << "      \"min_ms\": ";
    WriteJsonDoubleOrNull(qosFile, hasV2iSamples, g_qosSummary.v2iLatency.minLatencyMs);
    qosFile << ",\n";
    qosFile << "      \"max_ms\": ";
    WriteJsonDoubleOrNull(qosFile, hasV2iSamples, g_qosSummary.v2iLatency.maxLatencyMs);
    qosFile << ",\n";
    qosFile << "      \"jitter_ms\": " << g_qosSummary.v2iLatency.GetJitter() << ",\n";
    qosFile << "      \"p50_ms\": " << g_qosSummary.v2iLatency.GetPercentile(50) << ",\n";
    qosFile << "      \"p95_ms\": " << g_qosSummary.v2iLatency.GetPercentile(95) << ",\n";
    qosFile << "      \"p99_ms\": " << g_qosSummary.v2iLatency.GetPercentile(99) << ",\n";
    qosFile << "      \"sample_count\": " << g_qosSummary.v2iLatency.sampleCount << ",\n";
    qosFile << "      \"description\": \"V2I latency through 5G NR to MEC\"\n";
    qosFile << "    },\n";
    
    // V2V Latency (V2V direct links)
    const bool hasV2vSamples = g_qosSummary.v2vLatency.sampleCount > 0;
    qosFile << "    \"v2v_latency\": {\n";
    qosFile << "      \"avg_ms\": " << std::fixed << std::setprecision(3)
            << g_qosSummary.v2vLatency.GetAverageLatency() << ",\n";
    qosFile << "      \"min_ms\": ";
    WriteJsonDoubleOrNull(qosFile, hasV2vSamples, g_qosSummary.v2vLatency.minLatencyMs);
    qosFile << ",\n";
    qosFile << "      \"max_ms\": ";
    WriteJsonDoubleOrNull(qosFile, hasV2vSamples, g_qosSummary.v2vLatency.maxLatencyMs);
    qosFile << ",\n";
    qosFile << "      \"jitter_ms\": " << g_qosSummary.v2vLatency.GetJitter() << ",\n";
    qosFile << "      \"p50_ms\": " << g_qosSummary.v2vLatency.GetPercentile(50) << ",\n";
    qosFile << "      \"p95_ms\": " << g_qosSummary.v2vLatency.GetPercentile(95) << ",\n";
    qosFile << "      \"p99_ms\": " << g_qosSummary.v2vLatency.GetPercentile(99) << ",\n";
    qosFile << "      \"sample_count\": " << g_qosSummary.v2vLatency.sampleCount << ",\n";
    qosFile << "      \"description\": \"V2V latency via V2V direct links\"\n";
    qosFile << "    },\n";
    
    // Delivery
    qosFile << "    \"packet_delivery\": {\n";
    qosFile << "      \"packets_sent\": " << g_qosSummary.delivery.packetsSent << ",\n";
    qosFile << "      \"packets_received\": " << g_qosSummary.delivery.packetsReceived << ",\n";
    qosFile << "      \"packets_lost\": " << g_qosSummary.delivery.packetsLost << ",\n";
    qosFile << "      \"pdr_percent\": " << std::fixed << std::setprecision(2) 
            << g_qosSummary.delivery.GetPDR() << ",\n";
    qosFile << "      \"loss_rate_percent\": " << g_qosSummary.delivery.GetPacketLossRate() << "\n";
    qosFile << "    },\n";
    
    // Throughput
    qosFile << "    \"throughput\": {\n";
    qosFile << "      \"total_bytes_tx\": " << g_qosSummary.throughput.totalBytesTx << ",\n";
    qosFile << "      \"total_bytes_rx\": " << g_qosSummary.throughput.totalBytesRx << ",\n";
    qosFile << "      \"avg_throughput_mbps\": " << std::fixed << std::setprecision(3)
            << g_qosSummary.throughput.GetAverageThroughputMbps() << ",\n";
    qosFile << "      \"peak_throughput_mbps\": " << g_qosSummary.throughput.GetPeakThroughputMbps() << ",\n";
    qosFile << "      \"v2i_avg_mbps\": " << std::fixed << std::setprecision(3)
            << v2iThroughput.GetAverageThroughputMbps() << ",\n";
    qosFile << "      \"v2i_peak_mbps\": " << v2iThroughput.GetPeakThroughputMbps() << ",\n";
    qosFile << "      \"v2v_avg_mbps\": " << std::fixed << std::setprecision(3)
            << v2vThroughput.GetAverageThroughputMbps() << ",\n";
    qosFile << "      \"v2v_peak_mbps\": " << v2vThroughput.GetPeakThroughputMbps() << "\n";
    qosFile << "    },\n";
    
    // NDN Caching
    qosFile << "    \"ndn_caching\": {\n";
    qosFile << "      \"cache_hits\": " << g_qosSummary.caching.cacheHits << ",\n";
    qosFile << "      \"cache_misses\": " << g_qosSummary.caching.cacheMisses << ",\n";
    qosFile << "      \"hit_rate_percent\": " << std::fixed << std::setprecision(2) 
            << g_qosSummary.caching.GetHitRate() << ",\n";
    qosFile << "      \"proactive_cache_hits\": " << g_qosSummary.caching.proactiveCacheHits << ",\n";
    qosFile << "      \"reactive_cache_hits\": " << g_qosSummary.caching.reactiveCacheHits << ",\n";
    qosFile << "      \"avg_cache_latency_ms\": " << g_qosSummary.caching.avgCacheLatencyMs << ",\n";
    qosFile << "      \"avg_origin_latency_ms\": " << g_qosSummary.caching.avgOriginLatencyMs << ",\n";
    qosFile << "      \"latency_improvement_percent\": " << g_qosSummary.caching.GetLatencyImprovement() << "\n";
    qosFile << "    },\n";
    
    // Safety QoS
    qosFile << "    \"safety_qos\": {\n";
    qosFile << "      \"bsm_sent\": " << g_qosSummary.safety.bsmSent << ",\n";
    qosFile << "      \"bsm_received\": " << g_qosSummary.safety.bsmReceived << ",\n";
    qosFile << "      \"bsm_avg_latency_ms\": " << g_qosSummary.safety.bsmLatency.GetAverageLatency() << ",\n";
    qosFile << "      \"collision_warnings_sent\": " << g_qosSummary.safety.collisionWarningsSent << ",\n";
    qosFile << "      \"collision_warnings_received\": " << g_qosSummary.safety.collisionWarningsReceived << ",\n";
    qosFile << "      \"emergency_msg_sent\": " << g_qosSummary.safety.emergencyMsgSent << ",\n";
    qosFile << "      \"emergency_msg_received\": " << g_qosSummary.safety.emergencyMsgReceived << ",\n";
    qosFile << "      \"emergency_avg_latency_ms\": " << g_qosSummary.safety.emergencyLatency.GetAverageLatency() << ",\n";
    qosFile << "      \"avg_reaction_time_ms\": " << g_qosSummary.safety.avgReactionTimeMs << ",\n";
    qosFile << "      \"successful_reactions\": " << g_qosSummary.safety.successfulReactions << "\n";
    qosFile << "    },\n";
    
    // 5G NR Metrics
    qosFile << "    \"nr_5g_metrics\": {\n";
    qosFile << "      \"numerology\": " << g_qosSummary.nr5g.numerology << ",\n";
    qosFile << "      \"slot_duration_ms\": " << std::fixed << std::setprecision(4) 
            << g_qosSummary.nr5g.slotDurationMs << ",\n";
    qosFile << "      \"subcarrier_spacing_khz\": " << g_qosSummary.nr5g.subcarrierSpacingKhz << ",\n";
    qosFile << "      \"edge_latency_ms\": ";
    WriteJsonDoubleOrNull(qosFile, g_qosSummary.v2iLatency.sampleCount > 0, g_qosSummary.nr5g.edgeLatencyMs);
    qosFile << ",\n";
    qosFile << "      \"edge_latency_ms_description\": "
            << "\"mean V2I RTT (AppDelayTracer or NdnPositionCache MEC /v2x/v2i/ query); null if neither "
               "has samples\",\n";
    qosFile << "      \"target_reliability_percent\": " << g_qosSummary.nr5g.targetReliability << ",\n";
    qosFile << "      \"embb_throughput_mbps\": " << g_qosSummary.nr5g.embbThroughputMbps << ",\n";
    qosFile << "      \"avg_prb_utilization\": " << g_qosSummary.nr5g.avgPrbUtilization << ",\n";
    qosFile << "      \"handover_starts\": " << g_qosSummary.nr5g.handovers << ",\n";
    qosFile << "      \"handover_completions\": " << g_qosSummary.nr5g.handover_completions << ",\n";
    qosFile << "      \"handover_failures\": " << g_qosSummary.nr5g.handoverFailures << ",\n";
    qosFile << "      \"handover_unpaired_end_ok\": " << g_qosSummary.nr5g.handover_unpaired_end_ok << ",\n";
    qosFile << "      \"handover_duration_samples\": " << g_qosSummary.nr5g.handover_duration_samples
            << ",\n";
    qosFile << "      \"handover_duration_total_ms\": " << std::fixed << std::setprecision(3)
            << g_qosSummary.nr5g.handover_duration_total_ms << ",\n";
    qosFile << "      \"handover_duration_avg_ms\": ";
    if (g_qosSummary.nr5g.handover_duration_samples > 0)
    {
        qosFile << std::fixed << std::setprecision(3)
                << (g_qosSummary.nr5g.handover_duration_total_ms /
                    static_cast<double>(g_qosSummary.nr5g.handover_duration_samples));
    }
    else
    {
        qosFile << "null";
    }
    qosFile << ",\n";
    qosFile << "      \"handover_duration_min_ms\": ";
    WriteJsonDoubleOrNull(qosFile, g_qosSummary.nr5g.handover_duration_samples > 0,
                          g_qosSummary.nr5g.handover_duration_min_ms);
    qosFile << ",\n";
    qosFile << "      \"handover_duration_max_ms\": ";
    WriteJsonDoubleOrNull(qosFile, g_qosSummary.nr5g.handover_duration_samples > 0,
                          g_qosSummary.nr5g.handover_duration_max_ms);
    qosFile << ",\n";
    qosFile << "      \"handovers\": " << g_qosSummary.nr5g.handovers
            << ",\n"; // legacy alias = handover_starts
    qosFile << "      \"v2v_emulated_link_messages_sent\": " << g_qosSummary.nr5g.v2vEmulatedLinkMessagesSent
            << ",\n";
    qosFile << "      \"v2v_emulated_link_messages_received\": "
            << g_qosSummary.nr5g.v2vEmulatedLinkMessagesReceived << ",\n";
    qosFile << "      \"v2v_app_latency_ms\": " << g_qosSummary.nr5g.v2vAppLatencyMs << ",\n";
    qosFile << "      \"v2v_app_latency_description\": "
            << "\"mean V2V AppDelayTracer RTT over PointToPoint-emulated V2V; not 3GPP PC5\"\n";
    qosFile << "    }\n";
    
    qosFile << "  }\n";
    qosFile << "}\n";
    
    qosFile.close();
    std::cout << "  ✅ QoS report saved to " << filename << std::endl;
}

// Compute final metrics and generate JSON output
void GenerateMetricsJSON(const std::string& filename)
{
    // Collect NDN delivery metrics before generating report
    CollectNdnDeliveryMetrics();
    
    ValidateCriticalMetrics("GenerateMetricsJSON");

    std::ofstream jsonFile(filename);
    if (!jsonFile.is_open())
    {
        std::cerr << "Error: Cannot create " << filename << std::endl;
        return;
    }

    jsonFile << "{\n";
    jsonFile << "  \"simulation_duration\": " << currentSimTime << ",\n";

    // V2X Stats
    jsonFile << "  \"v2x_stats\": {\n";
    jsonFile << "    \"udp_packets_tx\": " << g_v2xStats.udpPacketsTx << ",\n";
    jsonFile << "    \"udp_packets_rx\": " << g_v2xStats.udpPacketsRx << ",\n";
    jsonFile << "    \"udp_bytes_tx\": " << g_v2xStats.udpBytesTx << ",\n";
    jsonFile << "    \"udp_bytes_rx\": " << g_v2xStats.udpBytesRx << ",\n";
    jsonFile << "    \"ndn_interests_tx\": " << g_v2xStats.ndnInterestsTx << ",\n";
    jsonFile << "    \"ndn_data_rx\": " << g_v2xStats.ndnDataRx << ",\n";
    jsonFile << "    \"ndn_bytes_rx\": " << g_v2xStats.ndnBytesRx << "\n";
    jsonFile << "  },\n";

    // V2V Direct Links Stats
    jsonFile << "  \"v2v_direct_link_stats\": {\n";
    jsonFile << "    \"resource_allocations\": " << slStats.resourceAllocations << ",\n";
    jsonFile << "    \"v2v_messages\": " << slStats.v2vMessages << ",\n";
    jsonFile << "    \"v2v_interests_tx\": " << slStats.v2vInterestsTx << ",\n";
    jsonFile << "    \"v2v_data_rx\": " << slStats.v2vDataRx << ",\n";
    jsonFile << "    \"v2v_bytes_tx\": " << slStats.v2vBytesTx << ",\n";
    jsonFile << "    \"v2v_bytes_rx\": " << slStats.v2vBytesRx << "\n";
    jsonFile << "  },\n";

    // Safety Stats
    jsonFile << "  \"safety_stats\": {\n";
    jsonFile << "    \"total_checks\": " << safetyStats.totalChecks << ",\n";
    jsonFile << "    \"collision_warnings\": " << safetyStats.collisionWarnings << ",\n";
    jsonFile << "    \"slow_down_commands\": " << safetyStats.slowDownCommands << ",\n";
    jsonFile << "    \"traffic_queries\": " << safetyStats.trafficQueries << ",\n";
    jsonFile << "    \"position_updates\": " << safetyStats.positionUpdates << "\n";
    jsonFile << "  },\n";

    // Node Metrics
    jsonFile << "  \"node_metrics\": {\n";
    bool first = true;
    for (const auto& kv : nodeMetrics)
    {
        if (!first) jsonFile << ",\n";
        first = false;

        const auto& m = kv.second;
        double avgLatency = m.latencySamples > 0 ? m.totalLatencyMs / m.latencySamples : 0.0;
        double cacheHitRate = (m.cacheHits + m.cacheMisses) > 0
            ? 100.0 * m.cacheHits / (m.cacheHits + m.cacheMisses) : 0.0;

        jsonFile << "    \"" << kv.first << "\": {\n";
        jsonFile << "      \"interests_sent\": " << m.interestsSent << ",\n";
        jsonFile << "      \"interests_received\": " << m.interestsReceived << ",\n";
        jsonFile << "      \"interests_satisfied\": " << m.interestsSatisfied << ",\n";
        jsonFile << "      \"interests_timed_out\": " << m.interestsTimedOut << ",\n";
        jsonFile << "      \"interests_in\": " << m.interestsIn << ",\n";
        jsonFile << "      \"interests_out\": " << m.interestsOut << ",\n";
        jsonFile << "      \"data_sent\": " << m.dataSent << ",\n";
        jsonFile << "      \"data_received\": " << m.dataReceived << ",\n";
        jsonFile << "      \"data_in\": " << m.dataIn << ",\n";
        jsonFile << "      \"data_out\": " << m.dataOut << ",\n";
        jsonFile << "      \"cache_hits\": " << m.cacheHits << ",\n";
        jsonFile << "      \"cache_misses\": " << m.cacheMisses << ",\n";
        jsonFile << "      \"cache_hit_rate\": " << std::fixed << std::setprecision(2) << cacheHitRate << ",\n";
        jsonFile << "      \"max_cs_size\": " << m.maxCsSize << ",\n";
        jsonFile << "      \"final_cs_size\": " << m.finalCsSize << ",\n";
        jsonFile << "      \"max_pit_size\": " << m.maxPitSize << ",\n";
        jsonFile << "      \"final_pit_size\": " << m.finalPitSize << ",\n";
        jsonFile << "      \"final_fib_size\": " << m.finalFibSize << ",\n";
        jsonFile << "      \"avg_latency_ms\": " << std::fixed << std::setprecision(3) << avgLatency << ",\n";
        jsonFile << "      \"total_bytes\": " << m.totalBytes << "\n";
        jsonFile << "    }";
    }
    jsonFile << "\n  },\n";
    
    // QoS Summary Section
    jsonFile << "  \"qos_summary\": {\n";
    jsonFile << "    \"e2e_latency_avg_ms\": " << std::fixed << std::setprecision(3) 
             << g_qosSummary.e2eLatency.GetAverageLatency() << ",\n";
    jsonFile << "    \"e2e_latency_jitter_ms\": " << g_qosSummary.e2eLatency.GetJitter() << ",\n";
    jsonFile << "    \"e2e_latency_p50_ms\": " << g_qosSummary.e2eLatency.GetPercentile(50) << ",\n";
    jsonFile << "    \"e2e_latency_p95_ms\": " << g_qosSummary.e2eLatency.GetPercentile(95) << ",\n";
    jsonFile << "    \"e2e_latency_p99_ms\": " << g_qosSummary.e2eLatency.GetPercentile(99) << ",\n";
    // V2I Latency (measured from app-delays-trace.txt)
    jsonFile << "    \"v2i_latency_avg_ms\": " << g_qosSummary.v2iLatency.GetAverageLatency() << ",\n";
    jsonFile << "    \"v2i_latency_min_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0, g_qosSummary.v2iLatency.minLatencyMs);
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_max_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0, g_qosSummary.v2iLatency.maxLatencyMs);
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_samples\": " << g_qosSummary.v2iLatency.sampleCount << ",\n";
    jsonFile << "    \"v2i_latency_jitter_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0,
                          g_qosSummary.v2iLatency.GetJitter());
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_p50_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0,
                          g_qosSummary.v2iLatency.GetPercentile(50));
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_p95_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0,
                          g_qosSummary.v2iLatency.GetPercentile(95));
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_p99_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2iLatency.sampleCount > 0,
                          g_qosSummary.v2iLatency.GetPercentile(99));
    jsonFile << ",\n";
    jsonFile << "    \"v2i_latency_description\": "
             << "\"V2I Interest→Data RTT: AppDelayTracer when available; else NdnPositionCache "
                "(MEC ConsumerCbr /v2x/v2i/vehicle/*/position over NR Uu + EPC + UDP to MEC)\",\n";
    // V2V Latency (measured from app-delays-trace.txt)
    jsonFile << "    \"v2v_latency_avg_ms\": " << g_qosSummary.v2vLatency.GetAverageLatency() << ",\n";
    jsonFile << "    \"v2v_latency_min_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0, g_qosSummary.v2vLatency.minLatencyMs);
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_max_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0, g_qosSummary.v2vLatency.maxLatencyMs);
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_samples\": " << g_qosSummary.v2vLatency.sampleCount << ",\n";
    jsonFile << "    \"v2v_latency_jitter_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0,
                          g_qosSummary.v2vLatency.GetJitter());
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_p50_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0,
                          g_qosSummary.v2vLatency.GetPercentile(50));
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_p95_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0,
                          g_qosSummary.v2vLatency.GetPercentile(95));
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_p99_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.v2vLatency.sampleCount > 0,
                          g_qosSummary.v2vLatency.GetPercentile(99));
    jsonFile << ",\n";
    jsonFile << "    \"v2v_latency_description\": "
             << "\"End-to-end NDN AppDelayTracer RTT via P2P emulated V2V links (100Mbps/1ms fixed, "
                "not 3GPP PC5)\",\n";
    {
        auto& ndnCacheStats = v2x::ndn_cache::NdnPositionCache::GetInstance().GetStats();
        jsonFile << "    \"v2i_mec_position_query_rtt_avg_ms\": " << std::fixed << std::setprecision(3)
                 << ndnCacheStats.GetAvgRttMs() << ",\n";
        jsonFile << "    \"v2i_mec_position_query_rtt_samples\": " << ndnCacheStats.rttSamples << ",\n";
        jsonFile << "    \"v2i_mec_position_query_rtt_description\": "
                 << "\"Interest send to Data recv at MEC for /v2x/v2i/vehicle/*/position; cross-check vs "
                    "v2i_latency_avg_ms (AppDelayTracer)\",\n";
    }
    jsonFile << "    \"packet_delivery_ratio\": " << std::fixed << std::setprecision(2) 
             << g_qosSummary.delivery.GetPDR() << ",\n";
    jsonFile << "    \"cache_hit_rate\": " << g_qosSummary.caching.GetHitRate() << ",\n";
    jsonFile << "    \"avg_throughput_mbps\": " << g_qosSummary.throughput.GetAverageThroughputMbps() << ",\n";
    jsonFile << "    \"handover_metrics\": {\n";
    jsonFile << "      \"starts\": " << g_qosSummary.nr5g.handovers << ",\n";
    jsonFile << "      \"completions\": " << g_qosSummary.nr5g.handover_completions << ",\n";
    jsonFile << "      \"failures\": " << g_qosSummary.nr5g.handoverFailures << ",\n";
    jsonFile << "      \"unpaired_end_ok\": " << g_qosSummary.nr5g.handover_unpaired_end_ok << ",\n";
    {
        const uint64_t hs = g_qosSummary.nr5g.handovers;
        const double cr =
            (hs > 0) ? (100.0 * static_cast<double>(g_qosSummary.nr5g.handover_completions) /
                        static_cast<double>(hs))
                     : 0.0;
        const double pr =
            (hs > 0) ? (100.0 * static_cast<double>(g_qosSummary.nr5g.handover_duration_samples) /
                        static_cast<double>(hs))
                     : 0.0;
        jsonFile << "      \"completion_rate_percent\": " << std::fixed << std::setprecision(2) << cr
                 << ",\n";
        jsonFile << "      \"paired_duration_rate_percent\": " << std::fixed << std::setprecision(2) << pr
                 << ",\n";
    }
    jsonFile << "      \"duration_samples\": " << g_qosSummary.nr5g.handover_duration_samples << ",\n";
    jsonFile << "      \"duration_avg_ms\": ";
    if (g_qosSummary.nr5g.handover_duration_samples > 0)
    {
        jsonFile << std::fixed << std::setprecision(3)
                 << (g_qosSummary.nr5g.handover_duration_total_ms /
                     static_cast<double>(g_qosSummary.nr5g.handover_duration_samples));
    }
    else
    {
        jsonFile << "null";
    }
    jsonFile << ",\n";
    jsonFile << "      \"duration_min_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.nr5g.handover_duration_samples > 0,
                          g_qosSummary.nr5g.handover_duration_min_ms);
    jsonFile << ",\n";
    jsonFile << "      \"duration_max_ms\": ";
    WriteJsonDoubleOrNull(jsonFile, g_qosSummary.nr5g.handover_duration_samples > 0,
                          g_qosSummary.nr5g.handover_duration_max_ms);
    jsonFile << ",\n";
    jsonFile << "      \"description\": "
             << "\"starts=LteEnbRrc/HandoverStart; completions=LteUeRrc/HandoverEndOk; failures="
                "HandoverEndError; duration_ms pairs Start→EndOk by IMSI\"\n";
    jsonFile << "    }\n";
    jsonFile << "  },\n";

    {
        const uint64_t ndnSent = g_qosSummary.delivery.packetsSent;
        const uint64_t ndnSat = g_qosSummary.delivery.packetsReceived;
        const uint64_t ndnUnsat = g_qosSummary.delivery.packetsLost;
        const double isr = (ndnSent > 0) ? (static_cast<double>(ndnSat) / static_cast<double>(ndnSent)) : 0.0;
        jsonFile << "  \"ndn_performance\": {\n";
        jsonFile << "    \"interest_satisfaction_ratio\": ";
        WriteJsonDoubleOrNull(jsonFile, ndnSent > 0, isr);
        jsonFile << ",\n";
        jsonFile << "    \"total_interests_sent\": " << ndnSent << ",\n";
        jsonFile << "    \"total_interests_satisfied\": " << ndnSat << ",\n";
        jsonFile << "    \"total_interests_unsatisfied\": " << ndnUnsat << ",\n";
        jsonFile << "    \"ndn_interests_tx\": " << g_v2xStats.ndnInterestsTx << ",\n";
        jsonFile << "    \"ndn_data_rx\": " << g_v2xStats.ndnDataRx << ",\n";
        jsonFile << "    \"description\": "
                 << "\"ISR = satisfied/sent Interests across all NDN nodes (MEC + vehicles), from NFD "
                    "forwarder counters\"\n";
        jsonFile << "  },\n";
    }

    // 5G NR Configuration (for parameterized comparison across runs)
    double slotDurationMs = 1.0 / (1 << g_activeNumerology);
    jsonFile << "  \"nr_5g_config\": {\n";
    jsonFile << "    \"numerology\": " << g_activeNumerology << ",\n";
    jsonFile << "    \"slot_duration_ms\": " << std::fixed << std::setprecision(4) << slotDurationMs << ",\n";
    jsonFile << "    \"central_freq_ghz\": " << std::fixed << std::setprecision(1) 
             << (NR_CENTRAL_FREQ_HZ / 1e9) << ",\n";
    jsonFile << "    \"bandwidth_mhz\": " << (NR_BANDWIDTH_HZ / 1e6) << ",\n";
    jsonFile << "    \"subcarrier_spacing_khz\": " << (15 * (1 << g_activeNumerology)) << "\n";
    jsonFile << "  },\n";
    
    // NDN Position Cache metrics (V2I measurement quality)
    jsonFile << "  \"ndn_position_cache\": {\n";
    auto& ndnCacheStats = v2x::ndn_cache::NdnPositionCache::GetInstance().GetStats();
    jsonFile << "    \"interests_sent\": " << ndnCacheStats.interestsSent << ",\n";
    jsonFile << "    \"data_received\": " << ndnCacheStats.dataReceived << ",\n";
    jsonFile << "    \"timeouts\": " << ndnCacheStats.timeouts << ",\n";
    jsonFile << "    \"stale_data_used\": " << ndnCacheStats.staleDataUsed << ",\n";
    jsonFile << "    \"avg_v2i_rtt_ms\": " << std::fixed << std::setprecision(3) 
             << ndnCacheStats.GetAvgRttMs() << ",\n";
    jsonFile << "    \"rtt_samples\": " << ndnCacheStats.rttSamples << "\n";
    jsonFile << "  }\n";
    
    jsonFile << "}\n";

    jsonFile.close();
    std::cout << "  ✓ Metrics saved to " << filename << std::endl;
}

bool ValidateCriticalMetrics(const std::string& context)
{
    // Keep QoS summary run metadata synchronized.
    g_qosSummary.simulationDuration = currentSimTime;
    g_qosSummary.totalNodes = nodeMapping.size();
    g_qosSummary.activeVehicles = std::count_if(
        vehicleStatuses.begin(),
        vehicleStatuses.end(),
        [](const std::pair<const std::string, VehicleStatus>& entry) {
            return entry.second.isActive;
        });
    g_qosSummary.activeRsus = std::count_if(
        nodeMapping.begin(),
        nodeMapping.end(),
        [](const std::pair<const std::string, Ptr<Node> >& entry) {
            return entry.first.find("rsu") != std::string::npos;
        });

    if (g_qosSummary.throughput.measurementDuration <= 0.0 && currentSimTime > 0.0)
    {
        g_qosSummary.throughput.measurementDuration = currentSimTime;
    }

    bool hasCriticalErrors = false;

    auto emitWarning = [&](const std::string& msg) {
        NS_LOG_WARN("[CriticalMetrics][" << context << "][WARN] " << msg);
        std::cerr << "⚠️ [CriticalMetrics][" << context << "][WARN] " << msg << std::endl;
    };

    auto emitError = [&](const std::string& msg) {
        NS_LOG_WARN("[CriticalMetrics][" << context << "][ERROR] " << msg);
        std::cerr << "❌ [CriticalMetrics][" << context << "][ERROR] " << msg << std::endl;
        hasCriticalErrors = true;
    };

    if (g_v2xStats.ndnInterestsTx == 0)
    {
        emitWarning("No NDN Interests transmitted; verify NDN app startup/routing.");
    }

    if (g_qosSummary.v2iLatency.sampleCount == 0)
    {
        emitError("No V2I AppDelayTracer samples found. Check: (1) UDP faces wired via "
                  "SetupUdpFacesForV2i, (2) FIB has /v2x routes toward MEC, (3) app-delays-trace.txt "
                  "exists and contains /v2x/v2i/ or /v2x/traffic/ prefixes, (4) ConsumerCbr apps "
                  "started before simulation end.");
    }

    if (g_qosSummary.v2vLatency.sampleCount == 0)
    {
        emitWarning("Missing measured V2V latency samples (qos_summary.v2v_latency_*).");
    }

    if (g_qosSummary.delivery.packetsSent == 0 &&
        g_v2xStats.udpPacketsTx == 0 &&
        g_v2xStats.ndnInterestsTx == 0)
    {
        emitError("No packet-delivery samples; PDR is unavailable for this run.");
    }

    if (g_qosSummary.throughput.totalBytesTx == 0 &&
        g_qosSummary.throughput.totalBytesRx == 0 &&
        g_v2xStats.udpBytesTx == 0 &&
        g_v2xStats.udpBytesRx == 0 &&
        g_v2xStats.ndnBytesRx == 0 &&
        slStats.v2vBytesTx == 0 &&
        slStats.v2vBytesRx == 0)
    {
        emitError("No throughput byte counters recorded (all tx/rx byte counters are zero).");
    }

    if (g_qosSummary.throughput.measurementDuration <= 0.0)
    {
        emitWarning("throughput.measurementDuration <= 0; avg throughput may be reported as 0.");
    }

    if (hasCriticalErrors)
    {
        emitWarning("Critical metrics are incomplete; preserving JSON schema and emitting null for missing latency min/max.");
    }
    else
    {
        std::cout << "  ✓ Critical metrics validation passed (" << context << ")" << std::endl;
    }

    return !hasCriticalErrors;
}

// ============================================================================
// PDR AND THROUGHPUT MEASUREMENT FUNCTIONS
// ============================================================================

// Calculate PDR from NDN forwarder counters
double CalculatePacketDeliveryRatio()
{
    uint64_t totalSent = 0;
    uint64_t totalSatisfied = 0;
    
    // From MEC NDN forwarder counters
    if (ndnMecNode)
    {
        auto mecL3 = ndnMecNode->GetObject<ns3::ndn::L3Protocol>();
        if (mecL3)
        {
            const auto& c = mecL3->getForwarder()->getCounters();
            totalSatisfied += c.nSatisfiedInterests;
        }
    }
    
    // From all vehicle NDN forwarder counters
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        auto vehL3 = vehicleNdnNodes.Get(i)->GetObject<ns3::ndn::L3Protocol>();
        if (vehL3)
        {
            const auto& c = vehL3->getForwarder()->getCounters();
            totalSent += c.nOutInterests;
            totalSatisfied += c.nSatisfiedInterests;
        }
    }
    
    if (totalSent == 0) return 0.0;
    return (static_cast<double>(totalSatisfied) / totalSent) * 100.0;
}

// Populate delivery metrics from NDN forwarder counters
void CollectNdnDeliveryMetrics()
{
    uint64_t totalSent = 0;
    uint64_t totalSatisfied = 0;
    uint64_t totalUnsatisfied = 0;
    
    // Collect from MEC: do not add nOutInterests — MEC forwards vehicle-originated Interests;
    // those are already counted on vehicle nodes. Use satisfaction counters only for MEC.
    if (ndnMecNode)
    {
        auto mecL3 = ndnMecNode->GetObject<ns3::ndn::L3Protocol>();
        if (mecL3)
        {
            const auto& c = mecL3->getForwarder()->getCounters();
            totalSatisfied += c.nSatisfiedInterests;
            totalUnsatisfied += c.nUnsatisfiedInterests;
        }
    }
    
    // Collect from vehicles
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        auto vehL3 = vehicleNdnNodes.Get(i)->GetObject<ns3::ndn::L3Protocol>();
        if (vehL3)
        {
            const auto& c = vehL3->getForwarder()->getCounters();
            totalSent += c.nOutInterests;
            totalSatisfied += c.nSatisfiedInterests;
            totalUnsatisfied += c.nUnsatisfiedInterests;
        }
    }
    
    // Update global delivery metrics
    g_qosSummary.delivery.packetsSent = totalSent;
    g_qosSummary.delivery.packetsReceived = totalSatisfied;
    g_qosSummary.delivery.packetsLost = totalUnsatisfied;
    
    NS_LOG_INFO("CollectNdnDeliveryMetrics: sent=" << totalSent 
                << " satisfied=" << totalSatisfied 
                << " unsatisfied=" << totalUnsatisfied
                << " PDR=" << CalculatePacketDeliveryRatio() << "%");
}

// Connect throughput tracking to an NDN node
// Uses beforeSatisfyInterest signal to track incoming Data (satisfaction = Data received)
void ConnectThroughputTracking(Ptr<Node> node, bool isV2i)
{
    auto l3 = node->GetObject<ns3::ndn::L3Protocol>();
    if (!l3)
    {
        NS_LOG_WARN("ConnectThroughputTracking: No L3Protocol on node");
        return;
    }
    
    auto forwarder = l3->getForwarder();
    
    // Track satisfied Interests (Data received) for throughput calculation
    // beforeSatisfyInterest fires when Data satisfies a pending Interest
    forwarder->beforeSatisfyInterest.connect(
        [isV2i](const nfd::pit::Entry& pitEntry, const nfd::Face& inFace, const ::ndn::Data& data)
        {
            size_t dataSize = data.wireEncode().size();
            if (isV2i)
            {
                v2iThroughput.totalBytesReceived += dataSize;
                g_qosSummary.throughput.totalBytesRx += dataSize;
            }
            else
            {
                v2vThroughput.totalBytesReceived += dataSize;
                g_qosSummary.throughput.totalBytesRx += dataSize;
            }
        });
    
    // Track CS hits for separate accounting (cached Data)
    forwarder->afterCsHit.connect(
        [isV2i](const ::ndn::Interest& interest, const ::ndn::Data& data)
        {
            size_t dataSize = data.wireEncode().size();
            if (isV2i)
            {
                v2iThroughput.totalBytesReceived += dataSize;
            }
            else
            {
                v2vThroughput.totalBytesReceived += dataSize;
            }
        });
}

// Sample instantaneous throughput
void SampleThroughput()
{
    double now = Simulator::Now().GetSeconds();
    
    // V2I throughput sampling
    if (v2iThroughput.lastSampleTime > 0)
    {
        double interval = now - v2iThroughput.lastSampleTime;
        if (interval > 0)
        {
            uint64_t bytesDelta = v2iThroughput.totalBytesReceived - v2iThroughput.lastSampleBytesRx;
            double throughputMbps = (bytesDelta * 8.0) / (interval * 1e6);
            v2iThroughput.throughputSamples.push_back(throughputMbps);
            g_qosSummary.throughput.instantThroughputs.push_back(throughputMbps);
        }
    }
    else
    {
        v2iThroughput.samplingStartTime = now;
    }
    v2iThroughput.lastSampleTime = now;
    v2iThroughput.lastSampleBytesRx = v2iThroughput.totalBytesReceived;
    v2iThroughput.samplingEndTime = now;
    
    // V2V throughput sampling
    if (v2vThroughput.lastSampleTime > 0)
    {
        double interval = now - v2vThroughput.lastSampleTime;
        if (interval > 0)
        {
            uint64_t bytesDelta = v2vThroughput.totalBytesReceived - v2vThroughput.lastSampleBytesRx;
            double throughputMbps = (bytesDelta * 8.0) / (interval * 1e6);
            v2vThroughput.throughputSamples.push_back(throughputMbps);
        }
    }
    else
    {
        v2vThroughput.samplingStartTime = now;
    }
    v2vThroughput.lastSampleTime = now;
    v2vThroughput.lastSampleBytesRx = v2vThroughput.totalBytesReceived;
    v2vThroughput.samplingEndTime = now;
    
    // Update measurement duration
    g_qosSummary.throughput.measurementDuration = now;
    
    // Reschedule every 1 second
    Simulator::Schedule(Seconds(1.0), &SampleThroughput);
}

// Start periodic throughput sampling
void StartThroughputSampling(double intervalSeconds)
{
    NS_LOG_INFO("Starting throughput sampling with interval " << intervalSeconds << "s");
    
    // Initialize sampling start times
    double now = Simulator::Now().GetSeconds();
    v2iThroughput.samplingStartTime = now;
    v2vThroughput.samplingStartTime = now;
    
    // Schedule first sample after warmup period
    Simulator::Schedule(Seconds(WARMUP_PERIOD_S + intervalSeconds), &SampleThroughput);
}
