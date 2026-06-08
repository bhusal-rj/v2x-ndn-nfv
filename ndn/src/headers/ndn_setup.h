#ifndef NDN_SETUP_H
#define NDN_SETUP_H

// Note: ndnSIM-module.h MUST be included in source files BEFORE "using namespace ns3;"
// to avoid namespace conflicts between ::ndn and ns3::ndn

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-address.h"

#include <string>
#include <functional>
#include <vector>

// ============================================================================
// NDN SETUP FUNCTIONS
// ============================================================================

/** True if \p nodeA and \p nodeB share a PointToPoint (V2V direct) channel. */
bool SharesP2PChannel(ns3::Ptr<ns3::Node> nodeA, ns3::Ptr<ns3::Node> nodeB);

/**
 * Connect to forwarder CS signals for a node to track cache hits/misses
 * @param node The NS-3 node to connect signals for
 * @param nodeName The name used for metrics tracking
 */
void ConnectCsSignals(ns3::Ptr<ns3::Node> node, const std::string& nodeName);

/**
 * Install NDN tracers on all NDN nodes (MEC + RSUs + Vehicles)
 * Should be called after nodes are created
 */
void InstallNdnTracers();

/**
 * Install NDN stack on the MEC node (edge cache)
 * NDN runs ONLY on MEC - not on UE devices (which use 5G NR)
 * @param node The MEC node
 */
void InstallNdnOnMec(ns3::Ptr<ns3::Node> node);

/**
 * Install NDN on vehicle nodes for V2V direct links communication
 * @param vehicles NodeContainer of vehicle nodes
 */
void InstallNdnOnVehicles(ns3::NodeContainer& vehicles);

/**
 * Install NDN on RSU nodes for proactive caching
 * @param rsuNodes NodeContainer of RSU nodes
 * @param mecNode MEC node for FIB routing
 */
void InstallNdnOnRsus(ns3::NodeContainer& rsuNodes, ns3::Ptr<ns3::Node> mecNode);

/**
 * Setup NDN Producer apps on MEC node
 * MEC aggregates and serves V2X content to all UEs via 5G
 * @param mecNode The MEC node
 */
void SetupNdnAppsOnMec(ns3::Ptr<ns3::Node> mecNode);

/**
 * Setup V2V NDN Producer/Consumer apps on vehicles
 * @param vehicles NodeContainer of vehicle nodes
 */
void SetupV2vNdnApps(ns3::NodeContainer& vehicles);

/**
 * Setup V2I NDN Producer apps on vehicles (for MEC to query positions)
 * Each vehicle produces: /v2x/v2i/vehicle/{id}/position
 * MEC sends Interest → Vehicle returns Data with position from vehicleStatuses
 * @param vehicles NodeContainer of vehicle nodes
 */
void SetupV2iNdnApps(ns3::NodeContainer& vehicles);

/**
 * Setup MEC NDN Consumer to query vehicle positions
 * MEC sends periodic Interests to all vehicles: /v2x/v2i/vehicle/{id}/position
 * Updates ndnReceivedPositions cache when Data packets arrive
 * @param mecNode The MEC node
 * @param numVehicles Number of vehicles to query
 */
void SetupMecPositionConsumer(ns3::Ptr<ns3::Node> mecNode, uint32_t numVehicles);

/**
 * Setup NDN Consumer apps on UE nodes
 * UEs only run NDN Consumers - they query the MEC for V2X data
 * @param node The UE node
 * @param id Node identifier
 */
void SetupNdnConsumerOnUe(ns3::Ptr<ns3::Node> node, const std::string& id);

/**
 * Setup NDN FIB routes manually for V2V and V2I communication
 * @param vehicles Vehicle NDN nodes
 * @param mecNode MEC node
 */
void SetupNdnFibRoutes(ns3::NodeContainer& vehicles, ns3::Ptr<ns3::Node> mecNode);

/**
 * Setup NDN Global Routing for V2I communication
 * Calculates NDN routes based on the underlying network topology.
 * This must be called BEFORE SetupMecPositionConsumer to ensure MEC can reach vehicles.
 * @param vehicles Vehicle NDN nodes
 * @param mecNode MEC node
 */
void SetupNdnGlobalRouting(ns3::NodeContainer& vehicles, ns3::Ptr<ns3::Node> mecNode);

/**
 * Wire NDN V2I over simulated UDP/IPv4 (standard port 6363 on UE side; MEC listens on 6363+index).
 * Interest/Data traverse the IP stack (NR UE → gNB → EPC → PGW → MEC), not parallel ndnSIM P2P links.
 */
void SetupUdpFacesForV2i(ns3::Ptr<ns3::Node> mecNode,
                         const ns3::NodeContainer& ueNodes,
                         const std::vector<std::string>& ueNodeIdsInOrder,
                         ns3::Ipv4Address mecIp);

// ============================================================================
// MEC NDN POSITION QUERY FUNCTIONS
// ============================================================================

/**
 * @brief Start periodic NDN position queries from MEC
 * MEC sends Interests to query vehicle positions: /v2x/v2i/vehicle/{id}/position
 * Results are stored in ndnReceivedPositions map
 * @param mecNode The MEC node
 * @param queryIntervalMs Query interval in milliseconds (default 100ms = 10Hz)
 */
void StartMecPositionQueries(ns3::Ptr<ns3::Node> mecNode, uint32_t queryIntervalMs = 100);

/**
 * @brief Stop MEC position queries
 */
void StopMecPositionQueries();

/**
 * @brief Setup vehicle NDN Producer for position data
 * Each vehicle produces: /v2x/v2i/vehicle/{id}/position
 * @param vehicleNode The vehicle node
 * @param vehicleId The vehicle ID (NS-3 ID, e.g., "veh0")
 */
void SetupVehiclePositionProducer(ns3::Ptr<ns3::Node> vehicleNode, const std::string& vehicleId);

/**
 * @brief Install position producers on all vehicles
 * @param vehicles NodeContainer of vehicle nodes
 */
void InstallVehiclePositionProducers(ns3::NodeContainer& vehicles);

#endif // NDN_SETUP_H
