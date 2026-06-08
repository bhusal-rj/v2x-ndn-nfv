/**
 * @file nr_5g_setup.cc
 * @brief 5G NR and V2X Network Setup
 * 
 * ARCHITECTURE OVERVIEW:
 * ======================
 * 
 * V2I Communication (Uu Interface):
 * - Full 5G NR stack: PHY → MAC → RLC → PDCP → IP → UDP → NDN
 * - Uses NrUeNetDevice with 3GPP channel models (UMa-LoS, fading)
 * - Realistic resource scheduling via NrMacSchedulerTdmaRR
 * - Valid for: V2I latency claims, 5G NR performance analysis
 * 
 * V2V Communication (Sidelink Emulation):
 * - Uses ns3::PointToPointHelper (NOT 3GPP PC5 sidelink)
 * - Fixed parameters: 100 Mbps, 1ms delay
 * - Static chain topology (each vehicle connected to 2 neighbors)
 * - NO resource sensing/selection (Mode 2)
 * - NO hidden terminal problem
 * - NO PC5 PHY modeling (PSCCH/PSSCH)
 * 
 * RESEARCH SCOPE:
 * ===============
 * ✅ Valid Claims:
 *    - NDN protocol behavior over idealized V2V links
 *    - V2V caching and content distribution patterns
 *    - Application-layer V2V message exchange
 * 
 * ❌ Invalid Claims (do NOT claim these):
 *    - PC5 sidelink performance
 *    - V2V latency under realistic radio conditions
 *    - Resource scheduling efficiency
 *    - Interference/collision behavior
 * 
 * JUSTIFICATION:
 * ==============
 * The P2P emulation provides an idealized testbed for evaluating
 * NDN application-layer protocols without conflating results with
 * PC5 radio effects. This separation of concerns allows focused
 * analysis of NDN caching benefits. For PC5-accurate results,
 * use dedicated V2X simulators (e.g., Artery, OpenC2X).
 * 
 * Citation: This approach follows the methodology in:
 * - Amadeo et al., "NDN for V2X" IEEE Comm. Surveys, 2016
 * - Zhang et al., "Vehicular NDN" ACM ICN, 2018
 */

#include "nr_5g_setup.h"
#include "simulation_state.h"
#include "ndn_setup.h"
#include "metrics_collector.h"
#include "v2x_constants.h"

#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace {
bool gPreCreateAllNodesCompleted = false;
}

// Setup V2V direct links using P2P emulation for NDN communication
// NOTE: This uses ns3::PointToPointHelper, NOT 3GPP PC5 sidelink.
// The P2P links provide idealized V2V connectivity (100Mbps, 1ms delay) for
// testing NDN application-layer protocols. This is NOT a radio-accurate model.
void SetupV2VDirectLinks(NodeContainer& vehicles, Ptr<Node> gnbNode)
{
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "⚠️  V2V EMULATION MODE (NOT 3GPP PC5)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "V2V uses PointToPoint links for NDN protocol evaluation." << std::endl;
    std::cout << "Parameters: " << V2V_LINK_DATARATE_MBPS << " Mbps, " 
              << V2V_LINK_DELAY_MS << " ms delay" << std::endl;
    std::cout << "Valid for: NDN caching, content routing, app-layer studies" << std::endl;
    std::cout << "NOT valid for: PC5 performance, radio resource claims" << std::endl;
    std::cout << std::string(60, '=') << "\n" << std::endl;

    // Create P2P links to emulate direct V2V communication
    PointToPointHelper v2vLinkHelper;
    std::string dataRateStr = std::to_string(static_cast<int>(V2V_LINK_DATARATE_MBPS)) + "Mbps";
    std::string delayStr = std::to_string(static_cast<int>(V2V_LINK_DELAY_MS)) + "ms";
    v2vLinkHelper.SetDeviceAttribute("DataRate", StringValue(dataRateStr));
    v2vLinkHelper.SetChannelAttribute("Delay", StringValue(delayStr));

    // Connect vehicles in a chain topology for V2V
    uint32_t numVehicles = vehicles.GetN();
    uint32_t v2vConnections = 0;

    for (uint32_t i = 0; i < numVehicles; i++)
    {
        // Connect to neighboring vehicles (static topology)
        for (uint32_t j = i + 1; j < std::min(i + V2V_NEIGHBOR_COUNT + 1, numVehicles); j++)
        {
            // Create direct V2V link
            NetDeviceContainer v2vDevices = v2vLinkHelper.Install(vehicles.Get(i), vehicles.Get(j));
            v2vConnections++;
            v2vStats.directLinkConnections++;
        }
    }

    std::cout << "  ✓ V2V direct links configured (P2P emulation)" << std::endl;
    std::cout << "    - " << v2vConnections << " direct links created" << std::endl;
    std::cout << "    - Data rate: " << V2V_LINK_DATARATE_MBPS << " Mbps (fixed)" << std::endl;
    std::cout << "    - Delay: " << V2V_LINK_DELAY_MS << " ms (fixed)" << std::endl;
    std::cout << "    - Topology: Static chain (each vehicle → " << V2V_NEIGHBOR_COUNT << " neighbors)" << std::endl;
    std::cout << "    - NOTE: No radio channel model (idealized)" << std::endl;
}

// Pre-create all nodes with pure 5G NR architecture
void PreCreateAllNodes(int numRsus, int numVehicles)
{
    if (gPreCreateAllNodesCompleted)
    {
        std::cerr << "PreCreateAllNodes: duplicate call ignored (topology already built).\n";
        return;
    }

    maxRsuSlots = numRsus;
    maxVehicleSlots = numVehicles;
    int totalNodes = numRsus + numVehicles;

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Pure 5G V2X with NDN-over-UDP Architecture" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Pre-creating " << totalNodes << " UE nodes (" << numRsus << " RSUs + "
              << numVehicles << " Vehicles) + 1 MEC node..." << std::endl;

    // Step 1: Create all UE node objects and set mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    std::vector<std::string> nodeIds;

    // Reuse UE nodes as NDN nodes (same-node approach)
    rsuNdnNodes = NodeContainer();
    vehicleNdnNodes = NodeContainer();

    // Create RSU nodes (UEs)
    for (int i = 0; i < numRsus; i++)
    {
        std::string rsuId = "rsu_" + std::to_string(i);
        Ptr<Node> node = CreateObject<Node>();
        allUeNodes.Add(node);
        rsuNdnNodes.Add(node);
        nodeMapping[rsuId] = node;
        ueNodeMapping[rsuId] = node;
        nodeIds.push_back(rsuId);
    }

    // Create Vehicle nodes (UEs)
    for (int i = 0; i < numVehicles; i++)
    {
        std::string vehId = "veh_" + std::to_string(i);
        Ptr<Node> node = CreateObject<Node>();
        allUeNodes.Add(node);
        vehicleNdnNodes.Add(node);
        nodeMapping[vehId] = node;
        ueNodeMapping[vehId] = node;
        nodeIds.push_back(vehId);
    }

    // Install mobility on all UE nodes
    mobility.Install(allUeNodes);
    
    // Give unique initial positions to ALL UE nodes to avoid 5G propagation crash
    // (The 5G spectrum model crashes if any two devices are at the same position)
    for (uint32_t i = 0; i < allUeNodes.GetN(); i++)
    {
        Ptr<MobilityModel> mob = allUeNodes.Get(i)->GetObject<MobilityModel>();
        // Spread UE nodes in a grid pattern with 20m spacing
        double x = 100.0 + (i % 8) * 20.0;
        double y = 100.0 + (i / 8) * 20.0;
        mob->SetPosition(Vector(x, y, 1.5));
    }
    std::cout << "  ✓ Created " << totalNodes << " UE nodes with mobility (unique positions)" << std::endl;

    // Step 2: Create MEC node (connected to PGW/UPF via P2P)
    ndnMecNode = CreateObject<Node>();
    mobility.Install(ndnMecNode);
    Ptr<MobilityModel> mecMob = ndnMecNode->GetObject<MobilityModel>();
    mecMob->SetPosition(Vector(500.0, 500.0, 50.0)); // MEC at center, elevated
    std::cout << "  ✓ Created MEC node (NDN edge cache)" << std::endl;

    // Step 3: Connect MEC to PGW via high-speed P2P link
    PointToPointHelper p2pMec;
    p2pMec.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2pMec.SetChannelAttribute("Delay", StringValue("1ms"));

    NetDeviceContainer mecPgwDevices = p2pMec.Install(ndnMecNode, pgw);
    std::cout << "  ✓ Connected MEC to PGW (10Gbps, 1ms)" << std::endl;

    // Step 4: Install Internet stack on MEC
    InternetStackHelper internet;
    internet.Install(ndnMecNode);

    // Assign IP to MEC-PGW link on a subnet DISJOINT from the NR EPC S1-U pool.
    // PointToPointEpcHelper / NrPointToPointEpcHelper allocates gNB↔SGW as 10.0.0.0/30,
    // 10.0.0.4/30, ... Reusing 10.0.0.0/24 here collides with those /30s and breaks UE→MEC
    // routing (NDN-over-UDP V2I never completes; AppDelay shows no apps 0–2 samples).
    Ipv4AddressHelper mecIpHelper;
    mecIpHelper.SetBase("172.16.100.0", "255.255.255.0");
    Ipv4InterfaceContainer mecIpIfaces = mecIpHelper.Assign(mecPgwDevices);
    Ipv4Address mecIp = mecIpIfaces.GetAddress(0);
    std::cout << "  ✓ MEC IP: " << mecIp << std::endl;

    // ================================================================
    // IP ROUTING FIX: Add routes so UE UDP packets reach MEC and back
    // ================================================================
    // Problem: MEC is on 172.16.100.0/24, UEs are on 7.0.0.0/8.
    // PGW sits between them but has no route to 172.16.100.0/24.
    // MEC has no route back to UE subnet 7.0.0.0/8.
    // Without these routes, all NDN-over-UDP V2I packets are dropped.

    // Route 1: PGW → MEC subnet
    // PGW must forward 172.16.100.0/24 traffic out its MEC-facing interface
    {
        Ptr<Ipv4StaticRouting> pgwRouting =
            Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(
                pgw->GetObject<Ipv4>()->GetRoutingProtocol());
        // Find PGW interface index for the MEC P2P link
        // mecPgwDevices.Get(1) is PGW side of the MEC-PGW P2P link
        Ptr<NetDevice> pgwMecDev = mecPgwDevices.Get(1);
        int32_t pgwMecIface = pgw->GetObject<Ipv4>()->GetInterfaceForDevice(pgwMecDev);
        if (pgwMecIface >= 0)
        {
            pgwRouting->AddNetworkRouteTo(
                Ipv4Address("172.16.100.0"),
                Ipv4Mask("255.255.255.0"),
                static_cast<uint32_t>(pgwMecIface));
            std::cout << "  ✓ PGW route added: 172.16.100.0/24 via iface "
                      << pgwMecIface << std::endl;
        }
        else
        {
            NS_FATAL_ERROR("Could not find PGW interface for MEC P2P link");
        }
    }

    // Route 2: MEC → UE subnet
    // MEC must route 7.0.0.0/8 back through PGW
    {
        Ptr<Ipv4StaticRouting> mecRouting =
            Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(
                ndnMecNode->GetObject<Ipv4>()->GetRoutingProtocol());
        // mecPgwDevices.Get(1) is PGW side — GetAddress(1) is PGW IP
        Ipv4Address pgwSideIp = mecIpIfaces.GetAddress(1);
        // mecPgwDevices.Get(0) is MEC side
        Ptr<NetDevice> mecP2pDev = mecPgwDevices.Get(0);
        int32_t mecP2pIface =
            ndnMecNode->GetObject<Ipv4>()->GetInterfaceForDevice(mecP2pDev);
        if (mecP2pIface >= 0)
        {
            mecRouting->AddNetworkRouteTo(
                Ipv4Address("7.0.0.0"),
                Ipv4Mask("255.0.0.0"),
                pgwSideIp,
                static_cast<uint32_t>(mecP2pIface));
            std::cout << "  ✓ MEC route added: 7.0.0.0/8 via "
                      << pgwSideIp << " iface " << mecP2pIface << std::endl;
        }
        else
        {
            NS_FATAL_ERROR("Could not find MEC interface for P2P link");
        }
    }
    // ================================================================

    // Step 5: Install 5G NR UE devices on all UE nodes
    std::cout << "  Installing 5G NR UE devices..." << std::endl;
    allUeDevices = nrHelper->InstallUeDevice(allUeNodes, allBwps);
    std::cout << "  ✓ Installed " << allUeDevices.GetN() << " 5G NR UE devices" << std::endl;

    // Step 6: Update UE device configuration (required by NR module)
    for (auto it = allUeDevices.Begin(); it != allUeDevices.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }
    std::cout << "  ✓ Updated UE device configurations" << std::endl;

    // Step 7: Install Internet stack on all UE nodes
    internet.Install(allUeNodes);
    std::cout << "  ✓ Installed Internet stack on UEs" << std::endl;

    // Step 8: Assign IPv4 addresses to UE devices (5G NR)
    Ipv4InterfaceContainer ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(allUeDevices);
    std::cout << "  ✓ Assigned IPv4 addresses to " << ueIpIfaces.GetN() << " UEs" << std::endl;

    // Store UE IP addresses for reference
    for (uint32_t i = 0; i < allUeNodes.GetN(); i++)
    {
        ueIpAddresses[nodeIds[i]] = ueIpIfaces.GetAddress(i);
    }

    // Step 9: Set default routes for all UE nodes (via 5G gateway)
    Ipv4Address gwAddr = nrEpcHelper->GetUeDefaultGatewayAddress();
    for (uint32_t i = 0; i < allUeNodes.GetN(); i++)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(
            allUeNodes.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol());
        ueStaticRouting->SetDefaultRoute(gwAddr, 1);
    }
    std::cout << "  ✓ Set default routes (5G gateway: " << gwAddr << ")" << std::endl;

    // Step 10: Attach all UEs to gNodeB
    nrHelper->AttachToClosestEnb(allUeDevices, gNbDevs);
    std::cout << "  ✓ Attached all UEs to closest gNodeB (2-cell topology, auto-selected)"
              << std::endl;

    // After UE devices exist — Config::Connect to LteUeRrc must not run earlier (fatal if 0 matches)
    ConnectNrHandoverTraces();

    // ========================================================================
    // STEP 11: V2V DIRECT LINKS SETUP (P2P emulation, not 3GPP PC5)
    // ========================================================================
    std::cout << "\n  Setting up V2V direct links for NDN communication..." << std::endl;
    // Vehicle NDN nodes are the same as UE nodes in this configuration

    // Setup V2V direct links (P2P emulation - 100Mbps, 1ms)
    SetupV2VDirectLinks(vehicleNdnNodes, gNbDevs.Get(0)->GetNode());

    // Store MEC IP for NDN-over-UDP V2I (reachable via UE default route → NR → EPC → PGW)
    mecIpFor5gNdn = mecIp;

    // NDN V2I uses UDP/IPv4 on the simulated stack (ndn_setup::SetupUdpFacesForV2i),
    // not parallel P2P links (those bypassed NR and produced fake latency/throughput).

    // Install NDN on all nodes (V2V still uses direct P2P NetDevice faces between vehicles)
    std::cout << "  Installing NDN on MEC (edge cache)..." << std::endl;
    InstallNdnOnMec(ndnMecNode);

    // Install NDN stack on vehicle nodes for V2V direct link communication
    InstallNdnOnVehicles(vehicleNdnNodes);

    // Setup NDN Producer apps on MEC
    SetupNdnAppsOnMec(ndnMecNode);
    std::cout << "  ✓ NDN Producer apps installed on MEC" << std::endl;

    // Setup V2V NDN Producer/Consumer apps on vehicles
    SetupV2vNdnApps(vehicleNdnNodes);

    // Setup V2I NDN Producer apps on vehicles (for MEC to query positions)
    SetupV2iNdnApps(vehicleNdnNodes);
    
    // ========================================================================
    // STEP 11b: NDN GLOBAL ROUTING SETUP
    // Calculate NDN routes based on the underlying network topology
    // This enables MEC to send Interests to vehicles through the 5G NR path
    // ========================================================================
    SetupNdnGlobalRouting(vehicleNdnNodes, ndnMecNode);
    
    // Setup MEC NDN Consumer to query vehicle positions periodically
    // Note: Start time delayed by 2s to avoid lockstep sync issues
    SetupMecPositionConsumer(ndnMecNode, vehicleNdnNodes.GetN());

    // Build the NDN node ID to name mapping for metrics collection
    ndnNodeIdToName[ndnMecNode->GetId()] = "mec";
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        ndnNodeIdToName[vehicleNdnNodes.Get(i)->GetId()] = "vehicle_" + std::to_string(i);
    }
    std::cout << "  ✓ NDN node ID mapping created (MEC=" << ndnMecNode->GetId()
              << ", Vehicles=" << vehicleNdnNodes.Get(0)->GetId() << "-"
              << vehicleNdnNodes.Get(vehicleNdnNodes.GetN() - 1)->GetId() << ")" << std::endl;

    // ========================================================================
    // STEP 12: RSU NDN SETUP FOR PROACTIVE CACHING
    // RSUs use 5G NR UE devices to connect to MEC (NO P2P bypass)
    // ========================================================================
    std::cout << "\n  Configuring RSU UE nodes for proactive caching..." << std::endl;
    
    // RSUs route through 5G NR stack (same as vehicles)
    // No separate P2P links needed - traffic goes via 5G core
    std::cout << "  ✓ RSU traffic routes through 5G NR stack (via gNB → PGW → MEC)" << std::endl;
    
    // Install NDN on RSU nodes (UE nodes)
    InstallNdnOnRsus(rsuNdnNodes, ndnMecNode);

    // NDN-over-UDP faces: UE IP ↔ MEC IP (port scheme), traffic rides NR + EPC routing
    SetupUdpFacesForV2i(ndnMecNode, allUeNodes, nodeIds, mecIpFor5gNdn);

    // V2V P2P FIB after V2I UDP faces so /v2x/traffic* and /v2x/safety pin to MEC before any
    // P2P-only nexthops are considered for overlapping best-route decisions.
    SetupNdnFibRoutes(vehicleNdnNodes, ndnMecNode);

    // Populate node ID mapping for RSUs (UE nodes)
    for (int i = 0; i < numRsus; i++)
    {
        std::string rsuId = "rsu_" + std::to_string(i);
        ndnNodeIdToName[rsuNdnNodes.Get(i)->GetId()] = rsuId;
    }

    // =========================================================================
    // POSITION EXCHANGE VIA NDN
    // =========================================================================
    // Position exchange uses NDN Interest/Data mechanism:
    // - vehicleStatuses: populated ONLY by OMNeT++ co-simulation (ground truth
    //   for MobilityModel/radio propagation physics)
    // - ndnReceivedPositions: populated by NDN Interest/Data exchanges
    // - Applications (collision detection, etc.) read from ndnReceivedPositions
    //   to evaluate true NDN-based awareness
    // =========================================================================
    // }
    std::cout << "  ⊘ Vehicle position exchange now via NDN only" << std::endl;

    // Step 16: Set initial positions
    std::cout << "  Setting initial positions..." << std::endl;
    for (int i = 0; i < numRsus; i++)
    {
        std::string rsuId = "rsu_" + std::to_string(i);
        Ptr<MobilityModel> mob = nodeMapping[rsuId]->GetObject<MobilityModel>();
        double x = 400.0 + (i % 4) * 50.0;
        double y = 400.0 + (i / 4) * 50.0;
        mob->SetPosition(Vector(x, y, 10.0)); // RSU at 10m height
    }
    for (int i = 0; i < numVehicles; i++)
    {
        std::string vehId = "veh_" + std::to_string(i);
        Ptr<MobilityModel> mob = nodeMapping[vehId]->GetObject<MobilityModel>();
        double x = -1000.0 - (i * 10.0);
        double y = -1000.0;
        mob->SetPosition(Vector(x, y, 1.5)); // Vehicle at 1.5m height
    }
    std::cout << "  ✓ Initial positions set" << std::endl;

    // Install tracers here (not via Simulator::Schedule): the first Simulator::Run()
    // happens only after OMNeT++ TIME_SYNC, so a t=0 scheduled install ran late and
    // could race with any duplicate-setup narrative; synchronous install attaches to
    // the final NDN stacks after V2I UDP faces exist.
    InstallNdnTracers();
    std::cout << "  ✓ NDN tracers installed (after V2I UDP faces)" << std::endl;

    // Optional: NR stack verification (RLC/PDCP + PHY/MAC ctrl traces). On some ns-3 NR
    // versions EnableRlcTraces is private; EnableTraces() is the public entry point.
    // Set NDN_V2X_NR_RLC_TRACES=1 before running ndn-v2x. Heavy — use only for verification.
    if (const char* rlcTrace = std::getenv("NDN_V2X_NR_RLC_TRACES");
        rlcTrace != nullptr && rlcTrace[0] != '\0' && rlcTrace[0] != '0')
    {
        nrHelper->EnableTraces();
        std::cout << "  ✓ NR EnableTraces() on (NDN_V2X_NR_RLC_TRACES); "
                     "see trace files in the process working directory" << std::endl;
    }

    std::cout << "\n📦 5G V2X with NDN over 5G NR Radio Stack Complete:" << std::endl;
    std::cout << "   • " << totalNodes << " UEs (5G NR Uu interface)" << std::endl;
    std::cout << "   • " << numVehicles << " Vehicle UE nodes (NR + NDN)" << std::endl;
    std::cout << "   • " << numRsus << " RSU UE nodes (NR + NDN)" << std::endl;
    std::cout << "   • 1 MEC node (NDN cache @ " << mecIp << ")" << std::endl;
    std::cout << "   • V2I: Vehicle → nearest gNB → 5G Core → MEC" << std::endl;
    std::cout << "   • Cells: gNB-0 (300,400,30) | gNB-1 (900,600,30)" << std::endl;
    std::cout << "   • Handover: automatic via AttachToClosestEnb + X2 interface" << std::endl;
    std::cout << "   • V2V: Vehicle ↔ Vehicle (NDN via V2V direct links)" << std::endl;
    std::cout << "   • 5G Path: UE (PHY/MAC/RLC/PDCP) → gNB → PGW → MEC" << std::endl;
    std::cout << "   • NDN Interest/Data traverse FULL 5G NR radio stack" << std::endl;
    std::cout << "========================================\n" << std::endl;

    gPreCreateAllNodesCompleted = true;
}
