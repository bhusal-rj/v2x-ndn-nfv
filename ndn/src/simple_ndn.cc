/**
 * @file simple_ndn.cc
 * @brief Main entry point for NDN over 5G V2X Co-simulation with OMNeT++
 * 
 * This file contains:
 * - Global variable definitions (declared extern in simulation_state.h)
 * - 5G NR infrastructure initialization
 * - Main simulation loop with OMNeT++ socket communication
 * 
 * The codebase has been refactored into the following modules:
 * - simulation_state.h: Global state structures and extern declarations
 * - ndn_setup.h/.cc: NDN stack installation and configuration
 * - nr_5g_setup.h/.cc: 5G NR infrastructure setup
 * - safety_detection.h/.cc: Collision detection and safety warnings
 * - metrics_collector.h/.cc: Metrics collection and JSON output
 * - node_management.h/.cc: Node creation and ID mapping
 * - network_utils.h/.cc: Socket communication utilities
 * - v2x_constants.h: Configuration constants
 */

// NS-3 Core Includes
#include "ns3/core-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/network-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/antenna-module.h"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"

// System Includes
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <cmath>
#include <cstdlib>
#include <execinfo.h>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <map>
#include <set>
#include <algorithm>

// Project Headers
#include "v2x_constants.h"
#include "network_utils.h"
#include "simulation_state.h"
#include "ndn_setup.h"
#include "nr_5g_setup.h"
#include "safety_detection.h"
#include "metrics_collector.h"
#include "node_management.h"

// Architecture A Headers
#include "arch_a_integration.h"
#include "mec_edge_filter.h"
#include "traffic_light_preemption.h"
#include "collision_avoidance.h"

using namespace ns3;

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS
// ============================================================================
// These are declared as extern in simulation_state.h

// Simulation State - Node Management
std::map<std::string, Ptr<Node>> nodeMapping;
std::map<std::string, Ptr<Node>> ueNodeMapping;
std::map<std::string, NodeThroughputStats> throughputStats;
std::map<std::string, NodeMetrics> nodeMetrics;
std::map<std::string, VehicleStatus> vehicleStatuses;

// 5G NR Helpers
Ptr<NrHelper> nrHelper;
Ptr<NrPointToPointEpcHelper> nrEpcHelper;
Ptr<IdealBeamformingHelper> idealBeamformingHelper;
Ptr<Node> pgw;
NetDeviceContainer gNbDevs;
BandwidthPartInfoPtrVector allBwps;

// NDN Helpers
ns3::ndn::StackHelper ndnHelper;
ns3::ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;

// Global V2X Stats
V2xThroughputStats g_v2xStats;
SafetyStats safetyStats;

// Safety Message Delivery Tracking (for research validity)
std::map<std::string, SafetyMessageDelivery> safetyMessageTracking;
uint64_t totalSafetyMessagesSent = 0;
uint64_t totalSafetyMessagesDelivered = 0;

// Actions logging
std::ofstream actionsLog;

// RSU ID Mapping Pool
std::map<std::string, std::string> rsuIdMapping;
std::set<int> usedRsuSlots;
int maxRsuSlots = 32;

// Vehicle ID Mapping Pool
std::map<std::string, std::string> vehicleIdMapping;
std::set<int> usedVehicleSlots;
int maxVehicleSlots = 50;

// Flag to track if tracers are installed
bool tracersInstalled = false;

// NDN nodes
Ptr<Node> ndnMecNode;
NodeContainer rsuNdnNodes;
NodeContainer vehicleNdnNodes;

// Global UE device containers
NodeContainer allUeNodes;
NetDeviceContainer allUeDevices;

// Helper to flush pending commands to OMNeT++ immediately
static void SendPendingCommandsToOmnet(int sock)
{
    auto pendingCommands = v2x::arch_a::GetPendingOmnetCommands();
    if (pendingCommands.empty())
    {
        return;
    }

    std::cout << "<- Sending " << pendingCommands.size() << " commands to OMNeT++ (immediate flush):" << std::endl;
    for (const auto& cmd : pendingCommands)
    {
        std::string cmdWithNewline = cmd + "\n";
        ssize_t cmdSent = send_data(sock, cmdWithNewline.c_str());
        if (cmdSent > 0)
        {
            std::cout << "   ✓ Sent: " << cmd.substr(0, 60) << "..." << std::endl;
            std::string cmdType = ExtractJsonString(cmd, "message_type");
            double cmdTimestamp = ExtractJsonNumber(cmd, "timestamp");
            if (cmd.find("\"timestamp\"") == std::string::npos)
            {
                cmdTimestamp = Simulator::Now().GetSeconds();
            }
            LogNs3ToOmnetData(cmdType.empty() ? "unknown" : cmdType, cmdTimestamp, cmd);
        }
        else
        {
            std::cerr << "   ✗ Failed to send command" << std::endl;
        }
    }
    v2x::arch_a::ClearPendingCommands();
}

// UE IPv4 addresses
std::map<std::string, Ipv4Address> ueIpAddresses;

// MEC IP address for NDN routing via 5G (used by FIB routes)
Ipv4Address mecIpFor5gNdn;

// V2V Direct Links Config and Stats
V2VDirectLinkConfig v2vDirectConfig;
V2VDirectLinkStats v2vStats;
// Legacy aliases (reference to primary objects)
V2VDirectLinkConfig& pc5Config = v2vDirectConfig;
V2VDirectLinkStats& slStats = v2vStats;

// NDN node ID to name mapping
std::map<uint32_t, std::string> ndnNodeIdToName;

// NDN app ID to prefix mapping (for latency categorization)
std::map<std::string, std::string> ndnAppIdToPrefix;

// Global file streams for tracing
std::ofstream csTraceFile;
std::ofstream pitTraceFile;
std::ofstream fibTraceFile;
std::ofstream omnetDataFile;
std::ofstream ns3ToOmnetDataFile;
double currentSimTime = 0.0;

// Runtime-configurable 5G NR numerology
uint32_t g_activeNumerology = NR_NUMEROLOGY_DEFAULT;

// Runtime-configurable random seed for reproducibility
uint32_t g_activeSeed = DEFAULT_SEED;

// ============================================================================
// SIGNAL HANDLING FOR GRACEFUL SHUTDOWN
// ============================================================================

// Flag to indicate if shutdown signal was received
volatile sig_atomic_t g_shutdownRequested = 0;

// Handler for SIGINT (Ctrl+C) and SIGTERM
void HandleShutdownSignal(int signum)
{
    if (g_shutdownRequested) 
    {
        // Second signal - force exit immediately
        std::cout << "\n❌ Second signal received, forcing exit..." << std::endl;
        _exit(signum);
    }
    
    g_shutdownRequested = signum;
    std::cout << "\n⚠️  Caught signal " << signum << " (";
    if (signum == SIGINT) 
        std::cout << "SIGINT/Ctrl+C";
    else if (signum == SIGTERM) 
        std::cout << "SIGTERM";
    std::cout << "), preparing graceful shutdown..." << std::endl;
    std::cout << "    (Press Ctrl+C again to force immediate exit)" << std::endl;
    
    // Note: Actual cleanup happens in main loop when it detects this flag
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

static void segfault_handler(int sig) {
    void* bt[30];
    int n = backtrace(bt, 30);
    fprintf(stderr, "\n=== SEGFAULT (signal %d) ===\n", sig);
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    fprintf(stderr, "=== END BACKTRACE ===\n");
    _exit(139);
}

int main(int argc, char* argv[])
{
    signal(SIGSEGV, segfault_handler);
    signal(SIGABRT, segfault_handler);

    // Parse command line arguments for configurable parameters
    uint32_t cmdNumerology = NR_NUMEROLOGY_DEFAULT;
    uint32_t cmdSeed = DEFAULT_SEED;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.find("--numerology=") == 0)
        {
            cmdNumerology = std::stoi(arg.substr(13));
            if (cmdNumerology > 4)
            {
                std::cerr << "Invalid numerology (must be 0-4): " << cmdNumerology << std::endl;
                return -1;
            }
        }
        else if (arg.find("--seed=") == 0)
        {
            cmdSeed = std::stoi(arg.substr(7));
        }
    }

    // Set global numerology for metrics reporting
    g_activeNumerology = cmdNumerology;
    
    // Set global seed for reproducibility
    g_activeSeed = cmdSeed;
    
    // Initialize NR metrics with numerology-derived values
    g_qosSummary.nr5g.SetNumerology(cmdNumerology);

    // Configure NS-3 random number generator for reproducibility
    RngSeedManager::SetSeed(cmdSeed);
    RngSeedManager::SetRun(1);  // Run number for different streams with same seed

    std::cout << "--- ns-3 Client Script ---" << std::endl;
    std::cout << "Attempting to connect to OMNeT++ server at " << HOST << ":" << PORT << "..." << std::endl;
    std::cout << "5G NR Numerology: " << cmdNumerology << " (SCS=" << (15 * (1 << cmdNumerology)) << " kHz)" << std::endl;
    std::cout << "Random Seed: " << cmdSeed << " (deterministic mode)" << std::endl;

    // Seed C++ stdlib random number generator for position jitter (deterministic)
    srand(cmdSeed);

    // Open actions log for collision events
    actionsLog.open("actions.txt", std::ios::out);
    if (actionsLog.is_open())
    {
        actionsLog << "=== ITS V2X Safety Application Log ===" << std::endl;
        actionsLog << "Simulation Start: " << GetCurrentTimeString() << std::endl;
        actionsLog << "Configuration:" << std::endl;
        actionsLog << "  RSU Check Interval: " << RSU_CHECK_INTERVAL_MS << "ms" << std::endl;
        actionsLog << "  Vehicle Query Interval: " << VEHICLE_QUERY_INTERVAL_MS << "ms" << std::endl;
        actionsLog << "  TTC Threshold: " << TTC_THRESHOLD_S << "s" << std::endl;
        actionsLog << "  Min Safe Distance: " << MIN_SAFE_DISTANCE_M << "m" << std::endl;
        actionsLog << "  Collision Risk Distance: " << COLLISION_RISK_DISTANCE_M << "m" << std::endl;
        actionsLog << "======================================\n" << std::endl;
    }

    // Connect to OMNeT++ server
    int sock = create_socket(HOST, PORT);
    if (sock < 0)
    {
        return -1;
    }

    std::cout << "Connected to OMNeT++ server at 127.0.0.1:" << PORT << std::endl;

    // --- NS-3 5G NR Initialization ---
    // Configure LTE EPC parameters (NR uses LTE EPC internally for core network)
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    // Create the helpers
    nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper = CreateObject<NrHelper>();

    // Connect helpers
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Configure 5G NR spectrum and bandwidth from parameterized constants
    double centralFrequencyBand = NR_CENTRAL_FREQ_HZ;
    double bandwidthBand = NR_BANDWIDTH_HZ;
    uint32_t numerology = cmdNumerology;  // Use command-line value if provided

    // Create bandwidth part configuration
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    // 5G NR CHANNEL MODEL CONFIGURATION:
    // ====================================================================
    // Scenario: UMa_LoS (Urban Macro Line-of-Sight) per 3GPP TS 38.901
    // - Pathloss: ThreeGppUmaPropagationLossModel
    // - Fading: 3GPP Spatial Channel Model (multipath Rayleigh fading)
    // - Spectrum Loss: ThreeGppSpectrumPropagationLossModel (ENABLED by default)
    // - Channel Condition: Line-of-Sight (LoS) with shadow fading
    // - This is initialized with flags: INIT_PROPAGATION | INIT_FADING | INIT_CHANNEL
    // ====================================================================
    CcBwpCreator::SimpleOperationBandConf bandConf(
        centralFrequencyBand, bandwidthBand, numCcPerBand, BandwidthPartInfo::UMa_LoS);

    // Create operation band
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Initialize channel model with full 3GPP 38.901 support (pathloss + fading)
    // Flags: INIT_PROPAGATION | INIT_FADING | INIT_CHANNEL (default = 0xFF)
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure NR MAC and PHY parameters for C-V2X
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(46.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
    // Numerology is set on gNB PHY only; UEs inherit it during attachment
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // Configure antenna
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));

    // Get UPF (User Plane Function) in 5G Core
    pgw = nrEpcHelper->GetPgwNode();

    // Create gNodeBs (5G NR base stations)
    NodeContainer gNbNodes;
    gNbNodes.Create(2);

    // Install Mobility for gNodeB (fixed positions)
    MobilityHelper gNbMobility;
    gNbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gNbMobility.Install(gNbNodes);

    // gNodeB 0 — covers west/south zone of vehicle area
    Ptr<MobilityModel> gNb0Mob = gNbNodes.Get(0)->GetObject<MobilityModel>();
    gNb0Mob->SetPosition(Vector(300.0, 400.0, 30.0));

    // gNodeB 1 — covers east/north zone of vehicle area
    Ptr<MobilityModel> gNb1Mob = gNbNodes.Get(1)->GetObject<MobilityModel>();
    gNb1Mob->SetPosition(Vector(900.0, 600.0, 30.0));

    std::cout << "📡 Two gNodeBs placed:" << std::endl;
    std::cout << "   gNB-0: (300, 400, 30) — west/south coverage" << std::endl;
    std::cout << "   gNB-1: (900, 600, 30) — east/north coverage" << std::endl;

    // Install 5G NR gNodeB device
    gNbDevs = nrHelper->InstallGnbDevice(gNbNodes, allBwps);

    // Update gNodeB device configurations
    for (auto it = gNbDevs.Begin(); it != gNbDevs.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    // X2 is configured on the EPC helper (NrHelper has no AddX2Interface; see LteHelper for same pattern).
    const uint32_t nGnb = gNbNodes.GetN();
    for (uint32_t i = 0; i < nGnb; ++i)
    {
        for (uint32_t j = i + 1; j < nGnb; ++j)
        {
            nrEpcHelper->AddX2Interface(gNbNodes.Get(i), gNbNodes.Get(j));
        }
    }

    // Handover Config::Connect is deferred to ConnectNrHandoverTraces() in nr_5g_setup.cc
    // after UEs are installed — Config::Connect fails fatally if LteUeRrc paths match zero objects.

    std::cout << "📡 5G NR Infrastructure initialized with 2 gNodeBs" << std::endl;
    std::cout << "   gNB-0 at (300, 400, 30) | gNB-1 at (900, 600, 30)" << std::endl;
    std::cout << "   Mode: 5G NR FR1 (" << (NR_CENTRAL_FREQ_HZ / 1e9) << " GHz), "
              << (NR_BANDWIDTH_HZ / 1e6) << " MHz bandwidth, numerology " << numerology << std::endl;
    std::cout << "   Beamforming: Ideal (4x8 gNB, 2x4 UE antenna arrays)" << std::endl;

    // Pre-create ALL nodes BEFORE starting simulation
    PreCreateAllNodes(maxRsuSlots, maxVehicleSlots); // 8 RSUs + 8 Vehicles
    std::cout << "✅ Pre-created " << (maxRsuSlots + maxVehicleSlots) << " nodes (" << maxRsuSlots << " RSUs + " << maxVehicleSlots << " Vehicles)" << std::endl << std::endl;

    // =========================================================================
    // ARCHITECTURE A INITIALIZATION
    // =========================================================================
    v2x::arch_a::InitializeArchA();
    
    // Register pre-created RSUs with Architecture A subsystems
    for (int i = 0; i < maxRsuSlots; i++)
    {
        std::string rsuId = "rsu_" + std::to_string(i);
        // Initial positions will be updated when RSU_STATE is received from OMNeT++
        v2x::arch_a::RegisterRsuWithArchA(rsuId, 100.0 * i, 100.0);
    }
    
    // Schedule Architecture A periodic tasks (collision detection, etc.)
    v2x::arch_a::ScheduleArchATasks();
    std::cout << "✅ Architecture A initialized" << std::endl << std::endl;
    
    // Schedule periodic metrics snapshots (every 60s) for long runs
    std::function<void()> periodicMetricsSnapshot;
    periodicMetricsSnapshot = [&periodicMetricsSnapshot]() {
        double t = Simulator::Now().GetSeconds();
        std::string snapshotSuffix = "_snapshot_" + std::to_string((int)t) + "s";
        
        std::cout << "\n📊 [Periodic Snapshot] Writing metrics at t=" << t << "s..." << std::endl;
        PrintNdnStats(t);
        v2x::arch_a::ExportArchAMetrics("arch_a_metrics" + snapshotSuffix + ".json");
        std::cout << "    ✓ Snapshot saved to *" << snapshotSuffix << ".json" << std::endl;
        
        // Schedule next snapshot
        Simulator::Schedule(Seconds(60.0), periodicMetricsSnapshot);
    };
    Simulator::Schedule(Seconds(60.0), periodicMetricsSnapshot);
    std::cout << "✅ Periodic metrics snapshots scheduled (every 60s)" << std::endl << std::endl;

    // Schedule periodic safety checks
    std::cout << "📡 Scheduling ITS Safety Application periodic checks..." << std::endl;
    std::cout << "   - RSU position checks: every " << RSU_CHECK_INTERVAL_MS << "ms" << std::endl;
    std::cout << "   - Vehicle traffic queries: every " << VEHICLE_QUERY_INTERVAL_MS << "ms" << std::endl;
    Simulator::Schedule(MilliSeconds(100), &RsuPeriodicPositionCheck);
    Simulator::Schedule(MilliSeconds(100), &VehiclePeriodicTrafficQuery);
    std::cout << "✅ ITS Safety Application scheduled" << std::endl << std::endl;

    // Open custom trace files
    csTraceFile.open("cs-trace.txt", std::ios::out | std::ios::trunc);
    if (!csTraceFile.is_open())
    {
        std::cerr << "Failed to open cs-trace.txt" << std::endl;
        return -1;
    }
    csTraceFile << "Time,Node,Type,CacheSize" << std::endl;

    pitTraceFile.open("pit-trace.txt", std::ios::out | std::ios::trunc);
    if (!pitTraceFile.is_open())
    {
        std::cerr << "Failed to open pit-trace.txt" << std::endl;
        return -1;
    }
    pitTraceFile << "Time,Node,Type,PitSize" << std::endl;

    fibTraceFile.open("fib-trace.txt", std::ios::out | std::ios::trunc);
    if (!fibTraceFile.is_open())
    {
        std::cerr << "Failed to open fib-trace.txt" << std::endl;
        return -1;
    }
    fibTraceFile << "Time,Node,Type,FibSize" << std::endl;

    omnetDataFile.open("omnet-data-log.txt", std::ios::out | std::ios::trunc);
    if (!omnetDataFile.is_open())
    {
        std::cerr << "Failed to open omnet-data-log.txt" << std::endl;
        return -1;
    }
    omnetDataFile << "=== OMNeT++ Data Received by NS-3 ===" << std::endl;

    ns3ToOmnetDataFile.open("ns3-to-omnet-log.txt", std::ios::out | std::ios::trunc);
    if (!ns3ToOmnetDataFile.is_open())
    {
        std::cerr << "Failed to open ns3-to-omnet-log.txt" << std::endl;
        return -1;
    }
    ns3ToOmnetDataFile << "=== NS-3 Data Sent to OMNeT++ ===" << std::endl;

    // Ignore SIGPIPE to prevent crash when writing to closed socket
    signal(SIGPIPE, SIG_IGN);
    
    // Install handlers for graceful shutdown on Ctrl+C or SIGTERM
    signal(SIGINT, HandleShutdownSignal);
    signal(SIGTERM, HandleShutdownSignal);

    char buffer[4096];
    std::string accumulator = "";
    double lastTNext = 1.0;  // Track the last valid t_next (default to 1 second step)

    std::ofstream dataFile("data.txt", std::ios::out | std::ios::trunc);
    if (!dataFile.is_open())
    {
        std::cerr << "Failed to open data.txt" << std::endl;
        return -1;
    }

    bool isTerminated = false;

    // Main simulation loop - communicate with OMNeT++
    while (true)
    {
        // Check if shutdown signal was received
        if (g_shutdownRequested)
        {
            std::cout << "\n📊 Shutdown signal detected, writing metrics before exit..." << std::endl;
            isTerminated = true;
            break;
        }
        
        // Blocking receive
        memset(buffer, 0, sizeof(buffer));
        ssize_t valread = recv(sock, buffer, sizeof(buffer) - 1, 0);

        if (valread <= 0)
        {
            std::cout << "🔌 Server closed the connection." << std::endl;
            // Will fall through to metrics collection after loop
            break;
        }

        accumulator.append(buffer, valread);

        // Process all complete JSON objects in the accumulator
        while (true)
        {
            size_t openBrace = accumulator.find('{');
            if (openBrace == std::string::npos)
            {
                if (accumulator.length() > 0 && accumulator.find_first_not_of(" \t\n\r") == std::string::npos)
                {
                    accumulator.clear();
                }
                break;
            }

            // Check if we have a full object by counting braces
            int braceCount = 0;
            size_t closeBrace = std::string::npos;
            for (size_t i = openBrace; i < accumulator.length(); ++i)
            {
                if (accumulator[i] == '{')
                    braceCount++;
                else if (accumulator[i] == '}')
                    braceCount--;

                if (braceCount == 0)
                {
                    closeBrace = i;
                    break;
                }
            }

            if (closeBrace == std::string::npos)
            {
                break; // Incomplete object, wait for more data
            }

            // Extract the message
            std::string messageStr = accumulator.substr(openBrace, closeBrace - openBrace + 1);
            accumulator.erase(0, closeBrace + 1);

            // Write to file
            dataFile << messageStr << "\n";
            dataFile.flush();

            // Parse JSON
            std::string msgType = ExtractJsonString(messageStr, "message_type");
            if (msgType.empty())
                msgType = "unknown";

            double timestamp = ExtractJsonNumber(messageStr, "timestamp");
            double tNext = ExtractJsonNumber(messageStr, "t_next");
            
            // If we got a valid t_next (not 0 or negative), remember it
            if (tNext > 0) {
                lastTNext = tNext;
            } else {
                // Use the last known t_next (from mobility_update or rsu_state)
                tNext = lastTNext;
            }

            // Log all data received from OMNeT++
            LogOmnetData(msgType, timestamp, messageStr);

            if (msgType == "mobility_update")
            {
                int count = ExtractCount(messageStr);
                std::cout << "-> Received MOBILITY_UPDATE for time window [" << timestamp << "s, " << tNext << "s] with " << count << " vehicles." << std::endl;

                // Parse payload to update/create nodes
                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    size_t arrayStart = messageStr.find("[", payloadPos);
                    size_t arrayEnd = messageStr.rfind("]");
                    if (arrayStart != std::string::npos && arrayEnd != std::string::npos && arrayEnd > arrayStart)
                    {
                        std::string payload = messageStr.substr(arrayStart, arrayEnd - arrayStart + 1);

                        size_t curr = 0;
                        int vehiclesParsed = 0;
                        while ((curr = payload.find("{", curr)) != std::string::npos)
                        {
                            int braceCount = 0;
                            size_t endObj = curr;
                            for (size_t i = curr; i < payload.length(); ++i)
                            {
                                if (payload[i] == '{')
                                    braceCount++;
                                else if (payload[i] == '}')
                                    braceCount--;
                                if (braceCount == 0)
                                {
                                    endObj = i;
                                    break;
                                }
                            }
                            if (endObj == curr)
                                break;

                            std::string obj = payload.substr(curr, endObj - curr + 1);

                            std::string id = ExtractJsonString(obj, "id");
                            double x = ExtractJsonNumber(obj, "x");
                            double y = ExtractJsonNumber(obj, "y");
                            double z = ExtractJsonNumber(obj, "z");
                            double speed = ExtractJsonNumber(obj, "speed");
                            double heading = ExtractJsonNumber(obj, "heading");
                            double acceleration = ExtractJsonNumber(obj, "acceleration");
                            std::string vehicleType = ExtractJsonString(obj, "vehicle_type");

                            if (!id.empty())
                            {
                                std::string mappedId = GetOrMapVehicleId(id);
                                if (!mappedId.empty())
                                {
                                    Ptr<Node> node = nodeMapping[mappedId];
                                    Ptr<Node> ueNode = nullptr;
                                    auto ueIt = ueNodeMapping.find(mappedId);
                                    if (ueIt != ueNodeMapping.end())
                                    {
                                        ueNode = ueIt->second;
                                    }

                                    if (node || ueNode)
                                    {
                                        // Add small random offset to prevent same-position assertion
                                        // This is necessary because SUMO may report two vehicles at identical positions
                                        // which causes 3GPP spectrum model to crash
                                        double offsetX = (rand() % 100) * 0.001; // 0-0.1m random offset
                                        double offsetY = (rand() % 100) * 0.001;
                                        double adjustedX = x + offsetX;
                                        double adjustedY = y + offsetY;

                                        if (node)
                                        {
                                            Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
                                            if (mobility)
                                            {
                                                mobility->SetPosition(Vector(adjustedX, adjustedY, z));
                                            }
                                        }
                                        if (ueNode && ueNode != node)
                                        {
                                            Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
                                            if (ueMobility)
                                            {
                                                ueMobility->SetPosition(Vector(adjustedX, adjustedY, z));
                                            }
                                        }

                                        VehicleStatus& status = vehicleStatuses[mappedId];
                                        status.id = mappedId;
                                        status.omnetId = id;
                                        status.x = adjustedX;
                                        status.y = adjustedY;
                                        status.z = z;
                                        status.speed = speed;
                                        status.heading = heading;
                                        status.acceleration = acceleration;
                                        status.vehicleType = vehicleType;
                                        status.lastUpdate = Simulator::Now();
                                        status.isActive = true;
                                    }
                                    vehiclesParsed++;
                                }
                            }
                            curr = endObj + 1;
                        }
                        std::cout << "   📍 Parsed " << vehiclesParsed << " vehicle positions." << std::endl;
                        
                        // =====================================================
                        // ARCHITECTURE A - Use Case III: Update Digital Twin
                        // =====================================================
                        v2x::arch_a::ProcessMobilityUpdate();
                        
                        // =====================================================
                        // ARCHITECTURE A - Use Case II: Update Preemption Tracking
                        // Check for vehicle passage and timeouts
                        // =====================================================
                        auto preemptionReleases = v2x::traffic::TrafficPreemptionManager::GetInstance().PeriodicPreemptionUpdate();
                        if (!preemptionReleases.empty())
                        {
                            std::cout << "   🚦 Generated " << preemptionReleases.size() << " preemption release commands" << std::endl;
                        }
                    }
                }
            }
            else if (msgType == "accident_report")
            {
                std::string accId = ExtractJsonString(messageStr, "accident_id");
                std::string vehicleId = ExtractJsonString(messageStr, "vehicle_id");
                if (vehicleId.empty()) vehicleId = ExtractJsonString(messageStr, "vehicleId");
                std::string roadId = ExtractJsonString(messageStr, "lane_id");
                if (roadId.empty()) roadId = ExtractJsonString(messageStr, "road_id");
                double posX = ExtractJsonNumber(messageStr, "pos_x");
                double posY = ExtractJsonNumber(messageStr, "pos_y");
                
                if (accId.empty() || vehicleId.empty() || roadId.empty())
                {
                    size_t payloadPos = messageStr.find("\"payload\"");
                    if (payloadPos != std::string::npos)
                    {
                        std::string payloadStr = messageStr.substr(payloadPos);
                        if (accId.empty()) accId = ExtractJsonString(payloadStr, "accident_id");
                        if (vehicleId.empty())
                        {
                            vehicleId = ExtractJsonString(payloadStr, "vehicleId");
                            if (vehicleId.empty()) vehicleId = ExtractJsonString(payloadStr, "vehicle_id");
                        }
                        if (roadId.empty())
                        {
                            roadId = ExtractJsonString(payloadStr, "lane_id");
                            if (roadId.empty()) roadId = ExtractJsonString(payloadStr, "road_id");
                        }
                        if (posX == 0.0 && posY == 0.0)
                        {
                            posX = ExtractJsonNumber(payloadStr, "pos_x");
                            posY = ExtractJsonNumber(payloadStr, "pos_y");
                        }
                    }
                }
                
                std::cout << "💥 Received ACCIDENT_REPORT at time " << timestamp << "s. ID: " << accId << ", Vehicle: " << vehicleId << std::endl;
                
                // =========================================================
                // ARCHITECTURE A - Use Case I: Process through Edge Filter
                // =========================================================
                double vehX = posX, vehY = posY;
                
                // If pos_x/pos_y are 0, try to get from vehicleStatuses
                if (vehX < 1.0 && vehY < 1.0 && !vehicleId.empty())
                {
                    // Try direct lookup first
                    auto vehIt = vehicleStatuses.find(vehicleId);
                    if (vehIt == vehicleStatuses.end())
                    {
                        // Try mapped ID
                        vehIt = vehicleStatuses.find(GetOrMapVehicleId(vehicleId));
                    }
                    if (vehIt != vehicleStatuses.end() && vehIt->second.isActive)
                    {
                        vehX = vehIt->second.x;
                        vehY = vehIt->second.y;
                        std::cout << "  📍 Got position from vehicleStatuses: (" << vehX << ", " << vehY << ")" << std::endl;
                    }
                    else
                    {
                        // Use centroid of all active vehicles as fallback
                        double sumX = 0, sumY = 0;
                        int count = 0;
                        for (const auto& [id, status] : vehicleStatuses)
                        {
                            if (status.isActive && status.x > 1 && status.y > 1)
                            {
                                sumX += status.x;
                                sumY += status.y;
                                count++;
                            }
                        }
                        if (count > 0)
                        {
                            vehX = sumX / count;
                            vehY = sumY / count;
                            std::cout << "  📍 Using centroid fallback: (" << vehX << ", " << vehY << ")" << std::endl;
                        }
                    }
                }
                
                auto cachingCommands = v2x::arch_a::ProcessAccidentReport(
                    accId, vehicleId, roadId, vehX, vehY);
                
                // Update V2X stats
                g_v2xStats.ndnInterestsTx += cachingCommands.size();
                safetyStats.totalChecks++;
                
                // Flush pending commands immediately (do not wait for next TIME_SYNC)
                SendPendingCommandsToOmnet(sock);
            }
            else if (msgType == "rsu_state")
            {
                int count = ExtractCount(messageStr);
                std::cout << "-> Received RSU_STATE for time window [" << timestamp << "s, " << tNext << "s] with " << count << " RSUs." << std::endl;

                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    size_t arrayStart = messageStr.find("[", payloadPos);
                    size_t arrayEnd = messageStr.rfind("]");
                    if (arrayStart != std::string::npos && arrayEnd != std::string::npos && arrayEnd > arrayStart)
                    {
                        std::string payload = messageStr.substr(arrayStart, arrayEnd - arrayStart + 1);

                        size_t curr = 0;
                        int rsusMapped = 0;
                        int trafficLightsProcessed = 0;
                        
                        while ((curr = payload.find("{", curr)) != std::string::npos)
                        {
                            // Find matching closing brace
                            int braceCount = 0;
                            size_t endObj = curr;
                            for (size_t i = curr; i < payload.length(); ++i)
                            {
                                if (payload[i] == '{') braceCount++;
                                else if (payload[i] == '}') braceCount--;
                                if (braceCount == 0) { endObj = i; break; }
                            }
                            if (endObj == curr) break;
                            
                            std::string obj = payload.substr(curr, endObj - curr + 1);

                            // Extract RSU ID as STRING (format: "rsu_j_0", "rsu_j_1", etc.)
                            std::string omnetRsuId = ExtractJsonString(obj, "id");
                            double x = ExtractJsonNumber(obj, "x");
                            double y = ExtractJsonNumber(obj, "y");
                            double z = ExtractJsonNumber(obj, "z");
                            
                            // =========================================================
                            // ARCHITECTURE A - Extract Traffic Light data from RSU state
                            // =========================================================
                            int isTrafficLight = (int)ExtractJsonNumber(obj, "isTrafficLight");
                            std::string tlsState = ExtractJsonString(obj, "trafficLightState");
                            std::string tlsId = ExtractJsonString(obj, "tlsId");
                            int vehicleCount = (int)ExtractJsonNumber(obj, "vehicleCount");
                            (void)vehicleCount; // reserved for Arch A / RSU load metrics

                            // Map RSU ID to NS-3 node
                            std::string mappedId = GetOrMapRsuId(omnetRsuId);

                            if (!mappedId.empty())
                            {
                                Ptr<Node> node = nodeMapping[mappedId];
                                Ptr<Node> ueNode = nullptr;
                                auto ueIt = ueNodeMapping.find(mappedId);
                                if (ueIt != ueNodeMapping.end())
                                {
                                    ueNode = ueIt->second;
                                }

                                if (node)
                                {
                                    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
                                    if (mobility)
                                    {
                                        mobility->SetPosition(Vector(x, y, z > 0 ? z : 0.0));
                                    }
                                }
                                if (ueNode && ueNode != node)
                                {
                                    Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
                                    if (ueMobility)
                                    {
                                        ueMobility->SetPosition(Vector(x, y, z > 0 ? z : 0.0));
                                    }
                                }
                                
                                // Register RSU with Architecture A
                                v2x::arch_a::RegisterRsuWithArchA(mappedId, x, y);
                                rsusMapped++;
                                
                                // =========================================================
                                // ARCHITECTURE A - Use Case II: Process Traffic Light
                                // =========================================================
                                if (isTrafficLight && !tlsId.empty())
                                {
                                    // Register traffic light with Zone Controller
                                    v2x::arch_a::RegisterTrafficLightWithArchA(tlsId, x, y);
                                    
                                    // Update traffic light state
                                    v2x::arch_a::ProcessTrafficUpdate(tlsId, tlsState, x, y);
                                    trafficLightsProcessed++;
                                }
                            }

                            curr = endObj + 1;
                        }
                        std::cout << "   >> Mapped and updated " << rsusMapped << " RSU positions." << std::endl;
                        if (trafficLightsProcessed > 0)
                        {
                            std::cout << "   🚦 Processed " << trafficLightsProcessed << " traffic lights for UC-II." << std::endl;
                        }
                    }
                }
            }
            else if (msgType == "traffic_update")
            {
                int count = ExtractCount(messageStr);
                std::cout << "-> Received TRAFFIC_UPDATE for time window [" << timestamp << "s, " << tNext << "s] with " << count << " traffic lights." << std::endl;
                
                // =========================================================
                // ARCHITECTURE A - Use Case II: Process traffic light states
                // =========================================================
                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    size_t arrayStart = messageStr.find("[", payloadPos);
                    size_t arrayEnd = messageStr.find("]", arrayStart);
                    if (arrayStart != std::string::npos && arrayEnd != std::string::npos)
                    {
                        std::string payload = messageStr.substr(arrayStart, arrayEnd - arrayStart + 1);
                        size_t curr = 0;
                        while ((curr = payload.find("{", curr)) != std::string::npos)
                        {
                            size_t endObj = payload.find("}", curr);
                            if (endObj == std::string::npos) break;
                            std::string obj = payload.substr(curr, endObj - curr + 1);
                            
                            std::string tlsId = ExtractJsonString(obj, "id");
                            std::string state = ExtractJsonString(obj, "state");
                            double x = ExtractJsonNumber(obj, "x");
                            double y = ExtractJsonNumber(obj, "y");
                            
                            if (!tlsId.empty())
                            {
                                v2x::arch_a::ProcessTrafficUpdate(tlsId, state, x, y);
                            }
                            curr = endObj + 1;
                        }
                    }
                }
            }
            else if (msgType == "mano_decision")
            {
                // =========================================================
                // MANO DECISION: Upstream Propagation Algorithm Response
                // OMNeT++ MANO analyzed road topology and determined which
                // RSUs cover upstream (feeder) roads leading to accident
                // =========================================================
                std::cout << "\n📥 [MANO] Received MANO_DECISION - Upstream Propagation Response" << std::endl;
                
                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    std::string queryId = ExtractJsonString(messageStr.substr(payloadPos), "query_id");
                    std::string contentName = ExtractJsonString(messageStr.substr(payloadPos), "content_name");
                    std::string accidentId = ExtractJsonString(messageStr.substr(payloadPos), "accident_id");
                    std::string roadId = ExtractJsonString(messageStr.substr(payloadPos), "lane_id");
                    
                    std::cout << "   Query ID: " << queryId << std::endl;
                    std::cout << "   Content: " << contentName << std::endl;
                    std::cout << "   Road (Event Edge): " << roadId << std::endl;
                    
                    // Parse targets array - these are the UPSTREAM RSUs identified by MANO
                    size_t targetsPos = messageStr.find("\"targets\"", payloadPos);
                    if (targetsPos != std::string::npos)
                    {
                        size_t arrayStart = messageStr.find("[", targetsPos);
                        size_t arrayEnd = messageStr.find("]", arrayStart);
                        if (arrayStart != std::string::npos && arrayEnd != std::string::npos)
                        {
                            std::string targetsStr = messageStr.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                            
                            // Parse RSU targets and translate from OMNeT++ IDs to NS-3 IDs
                            std::vector<std::string> targetRsus;
                            std::cout << "   Upstream RSU Targets (from topology analysis):" << std::endl;
                            
                            size_t curr = 0;
                            while ((curr = targetsStr.find("\"", curr)) != std::string::npos)
                            {
                                size_t endQuote = targetsStr.find("\"", curr + 1);
                                if (endQuote != std::string::npos)
                                {
                                    std::string omnetRsuId = targetsStr.substr(curr + 1, endQuote - curr - 1);
                                    if (!omnetRsuId.empty())
                                    {
                                        // Translate OMNeT++ RSU ID (e.g., "rsu_j_0") to NS-3 ID (e.g., "rsu_0")
                                        std::string ns3RsuId = GetOrMapRsuId(omnetRsuId);
                                        if (!ns3RsuId.empty())
                                        {
                                            targetRsus.push_back(ns3RsuId);
                                            std::cout << "     ✓ " << omnetRsuId << " -> " << ns3RsuId 
                                                      << " (upstream feeder road coverage)" << std::endl;
                                        }
                                        else
                                        {
                                            std::cerr << "     ⚠️  Could not map RSU: " << omnetRsuId << std::endl;
                                        }
                                    }
                                    curr = endQuote + 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            
                            if (targetRsus.empty())
                            {
                                std::cout << "     (no upstream RSUs identified - accident may be at terminal road)" << std::endl;
                            }
                            else
                            {
                                std::cout << "   Executing proactive cache push to " << targetRsus.size() 
                                          << " upstream RSUs..." << std::endl;
                                
                                // Execute caching decision using ProactiveCacheManager
                                // This pushes accident data to upstream RSU Content Stores
                                v2x::mec::CachingDecision decision;
                                // Fix: Use accident_id directly, not contentName which has /v2x/safety/ prefix
                                decision.accidentId = accidentId.empty() ? 
                                    (contentName.find("/v2x/safety/") == 0 ? contentName.substr(12) : contentName) 
                                    : accidentId;
                                decision.roadId = roadId;
                                decision.targetRsus = targetRsus;
                                decision.reason = "mano_upstream_propagation";
                                v2x::mec::ProactiveCacheManager::GetInstance().ExecuteCachingDecision(decision);
                                
                                std::cout << "   ✅ Proactive caching complete - vehicles approaching from upstream will get cache hits" << std::endl;

                                // After proactive caching, trigger notifications/actuations
                                auto postCacheCmds = v2x::arch_a::ProcessAccidentPostCaching(decision.accidentId);
                                // Commands are already added to pending inside, but keep for completeness
                                for (const auto& cmd : postCacheCmds)
                                {
                                    (void)cmd; // already enqueued
                                }
                            }
                        }
                    }
                    else
                    {
                        std::cout << "   ⚠️  No 'targets' array in MANO decision" << std::endl;
                    }
                }
                std::cout << std::endl;
            }
            else if (msgType == "traffic_preemption_request")
            {
                // =========================================================
                // ARCHITECTURE A - Use Case II: Traffic Light Preemption
                // Process emergency vehicle request at the Edge (Zone Controller)
                // =========================================================
                std::cout << "🚑 Received TRAFFIC_PREEMPTION_REQUEST at time " << timestamp << "s" << std::endl;
                
                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    std::string payloadStr = messageStr.substr(payloadPos);
                    
                    std::string tlsId = ExtractJsonString(payloadStr, "tls_id");
                    std::string senderId = ExtractJsonString(payloadStr, "sender_id");
                    std::string laneId = ExtractJsonString(payloadStr, "lane_id");
                    double originTime = ExtractJsonNumber(payloadStr, "origin_time");
                    
                    std::cout << "   TLS ID: " << tlsId << ", Sender: " << senderId << ", Lane: " << laneId
                              << ", origin_time=" << originTime << "s" << std::endl;
                    
                    // Get vehicle position and speed from vehicleStatuses
                    double vehicleX = 0.0, vehicleY = 0.0, vehicleSpeed = 0.0;
                    
                    // Try to find the vehicle in vehicleStatuses
                    std::string mappedId = GetOrMapVehicleId(senderId);
                    auto vehIt = vehicleStatuses.find(mappedId);
                    if (vehIt != vehicleStatuses.end() && vehIt->second.isActive)
                    {
                        vehicleX = vehIt->second.x;
                        vehicleY = vehIt->second.y;
                        vehicleSpeed = vehIt->second.speed;
                    }
                    else
                    {
                        // Try direct lookup
                        vehIt = vehicleStatuses.find(senderId);
                        if (vehIt != vehicleStatuses.end() && vehIt->second.isActive)
                        {
                            vehicleX = vehIt->second.x;
                            vehicleY = vehIt->second.y;
                            vehicleSpeed = vehIt->second.speed;
                        }
                    }
                    
                    if (vehicleX > 0 || vehicleY > 0)
                    {
                        // Process the preemption request through Architecture A with lane ID
                        std::string preemptionCommand = v2x::arch_a::ProcessPreemptionRequestWithLane(
                            senderId, tlsId, laneId, vehicleX, vehicleY, vehicleSpeed);
                        
                        if (!preemptionCommand.empty())
                        {
                            std::cout << "   ✓ Preemption GRANTED - command queued for OMNeT++" << std::endl;
                        }
                        else
                        {
                            std::cout << "   ✗ Preemption DENIED (too far or conflict)" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "   ⚠️ Could not find vehicle position for " << senderId << std::endl;
                    }
                }
            }
            else if (msgType == "termination_signal")
            {
                std::string reason = "Unknown";
                size_t payloadPos = messageStr.find("\"payload\"");
                if (payloadPos != std::string::npos)
                {
                    reason = ExtractJsonString(messageStr.substr(payloadPos), "reason");
                }
                std::cout << "\n =======================================" << std::endl;
                std::cout << " TERMINATION SIGNAL RECEIVED" << std::endl;
                std::cout << " Reason: " << reason << std::endl;
                std::cout << " Final Simulation Time: " << timestamp << "s" << std::endl;
                std::cout << " =======================================" << std::endl;

                omnetDataFile << "\n=== TERMINATION SIGNAL ===" << std::endl;
                omnetDataFile << "Reason: " << reason << std::endl;
                omnetDataFile << "Final Time: " << timestamp << "s" << std::endl;
                omnetDataFile << "===========================" << std::endl;

                isTerminated = true;
            }
            else if (msgType == "end_of_step")
            {
                std::cout << "-> Received END_OF_STEP. Running NS-3 simulation from " << timestamp << "s to " << tNext << "s." << std::endl;

                currentSimTime = timestamp;

                // Calculate the time to simulate: from current NS-3 time to tNext
                double ns3CurrentTime = Simulator::Now().GetSeconds();
                double timeToSimulate = tNext - ns3CurrentTime;
                
                if (timeToSimulate > 0) {
                    std::cerr << "   >> Simulator::Run() for " << timeToSimulate << "s..." << std::endl;
                    Simulator::Stop(Seconds(timeToSimulate));
                    Simulator::Run();
                    std::cerr << "   >> Simulator::Run() completed at " << Simulator::Now().GetSeconds() << "s" << std::endl;
                } else {
                    std::cout << "   ⚠️ NS-3 already at or past time " << tNext << "s (currently at " << ns3CurrentTime << "s)" << std::endl;
                }

                PrintNdnStats(currentSimTime);

                // Send all pending commands to OMNeT++ BEFORE TIME_SYNC
                auto pendingCommands = v2x::arch_a::GetPendingOmnetCommands();
                if (!pendingCommands.empty())
                {
                    std::cout << "<- Sending " << pendingCommands.size() << " commands to OMNeT++:" << std::endl;
                    for (const auto& cmd : pendingCommands)
                    {
                        std::string cmdWithNewline = cmd + "\n";
                        ssize_t cmdSent = send_data(sock, cmdWithNewline.c_str());
                        if (cmdSent > 0)
                        {
                            std::cout << "   ✓ Sent: " << cmd.substr(0, 60) << "..." << std::endl;
                            std::string cmdType = ExtractJsonString(cmd, "message_type");
                            double cmdTimestamp = ExtractJsonNumber(cmd, "timestamp");
                            if (cmd.find("\"timestamp\"") == std::string::npos)
                            {
                                cmdTimestamp = Simulator::Now().GetSeconds();
                            }
                            LogNs3ToOmnetData(cmdType.empty() ? "unknown" : cmdType, cmdTimestamp, cmd);
                        }
                        else
                        {
                            std::cerr << "   ✗ Failed to send command" << std::endl;
                        }
                    }
                    v2x::arch_a::ClearPendingCommands();
                }

                std::string sync_msg = "TIME_SYNC\n";
                std::cout << "<- Sending synchronization signal: " << sync_msg;

                ssize_t sent_bytes = send_data(sock, sync_msg.c_str());
                if (sent_bytes < 0)
                {
                    if (errno == EPIPE || errno == ECONNRESET)
                    {
                        std::cerr << "⚠️ Server closed connection (Broken Pipe). Writing metrics before exit..." << std::endl;
                        
                        // Collect and write metrics before shutdown
                        std::cout << "\n📊 === Collecting final metrics ===" << std::endl;
                        PrintNdnStats(currentSimTime);
                        v2x::arch_a::PrintArchASummary();
                        v2x::arch_a::ExportArchAMetrics("arch_a_metrics.json");
                        ns3::ndn::AppDelayTracer::Destroy();
                        CollectMetricsFromTracers();
                        GenerateMetricsJSON("simulation_metrics.json");
                        
                        close(sock);
                        Simulator::Destroy();
                        return 0;
                    }
                    else
                    {
                        perror("Send failed");
                    }
                }
                else
                {
                    LogNs3ToOmnetData("TIME_SYNC", Simulator::Now().GetSeconds(), "TIME_SYNC");
                }
            }
            else
            {
                std::cout << "-> Received unknown message type: '" << msgType << "'" << std::endl;
            }

            if (isTerminated)
            {
                break;
            }
        }

        if (isTerminated)
            break;
    }

    // Final summary
    std::cout << "\n ========================================" << std::endl;
    std::cout << " === Final NDN Co-Simulation Summary ===" << std::endl;
    std::cout << " ========================================" << std::endl;
    std::cout << "Total simulation time: " << currentSimTime << "s" << std::endl;
    std::cout << "Total nodes created: " << nodeMapping.size() << std::endl;
    std::cout << "  - Vehicles: " << std::count_if(nodeMapping.begin(), nodeMapping.end(), [](const auto& p)
                                                   { return p.first.find("rsu") == std::string::npos; })
              << std::endl;
    std::cout << "  - RSUs: " << std::count_if(nodeMapping.begin(), nodeMapping.end(), [](const auto& p)
                                               { return p.first.find("rsu") != std::string::npos; })
              << std::endl;

    PrintNdnStats(currentSimTime);
    
    // =========================================================================
    // ARCHITECTURE A - Final Summary and Metrics Export
    // =========================================================================
    v2x::arch_a::PrintArchASummary();
    v2x::arch_a::ExportArchAMetrics("arch_a_metrics.json");

    omnetDataFile << "\n=== FINAL SUMMARY ===" << std::endl;
    omnetDataFile << "Total simulation time: " << currentSimTime << "s" << std::endl;
    omnetDataFile << "Total nodes: " << nodeMapping.size() << std::endl;
    omnetDataFile << "=====================" << std::endl;

    std::cout << "📊 ========================================\n" << std::endl;

    // Flush/close AppDelayTracer after the simulation loop exits (safe: no further NDN tracer use).
    ns3::ndn::AppDelayTracer::Destroy();
    // Collect metrics from tracer files and generate JSON
    CollectMetricsFromTracers();
    GenerateMetricsJSON("simulation_metrics.json");

    // Close all files
    dataFile.close();
    csTraceFile.close();
    pitTraceFile.close();
    fibTraceFile.close();
    omnetDataFile.close();
    if (ns3ToOmnetDataFile.is_open())
    {
        ns3ToOmnetDataFile.close();
    }
    
    // Close OMNeT++ JSON log
    CloseOmnetLogs();
    std::cout << "📊 Total messages received from OMNeT++: " << GetOmnetMessageCount() << std::endl;

    // Close actions log with final stats
    if (actionsLog.is_open())
    {
        actionsLog << "\n======================================" << std::endl;
        actionsLog << "=== ITS Safety Application Summary ===" << std::endl;
        actionsLog << "======================================" << std::endl;
        actionsLog << "End Time: " << GetCurrentTimeString() << std::endl;
        actionsLog << "Simulation Duration: " << currentSimTime << " seconds" << std::endl;
        actionsLog << "\nSafety Statistics:" << std::endl;
        actionsLog << "  Total Safety Checks: " << safetyStats.totalChecks << std::endl;
        actionsLog << "  Collision Warnings: " << safetyStats.collisionWarnings << std::endl;
        actionsLog << "  Slow Down Commands: " << safetyStats.slowDownCommands << std::endl;
        actionsLog << "  Traffic Queries: " << safetyStats.trafficQueries << std::endl;
        actionsLog << "  Position Updates: " << safetyStats.positionUpdates << std::endl;
        actionsLog << "\nActive Vehicles at End: " << std::count_if(vehicleStatuses.begin(), vehicleStatuses.end(), [](const auto& p)
                                                                    { return p.second.isActive; })
                   << std::endl;
        actionsLog << "======================================" << std::endl;
        actionsLog.close();
    }

    close(sock);
    Simulator::Destroy();
    std::cout << "--- ns-3 Client Script Finished ---" << std::endl;

    return 0;
}
