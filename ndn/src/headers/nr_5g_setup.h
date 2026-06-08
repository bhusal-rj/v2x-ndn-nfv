#ifndef NR_5G_SETUP_H
#define NR_5G_SETUP_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/cc-bwp-helper.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

// ============================================================================
// 5G NR SETUP FUNCTIONS
// ============================================================================

/**
 * Initialize 5G NR infrastructure (helpers, gNodeB, etc.)
 * Creates and configures NrHelper, NrEpcHelper, BeamformingHelper
 * Sets up gNodeB with antenna arrays and channel model
 */
void Initialize5gNrInfrastructure();

/**
 * Setup V2V direct links using P2P emulation for NDN communication
 * NOTE: This uses ns3::PointToPointHelper (100Mbps, 1ms), NOT 3GPP PC5 sidelink.
 * The implementation provides idealized V2V connectivity for NDN application testing.
 * @param vehicles NodeContainer of vehicle NDN nodes
 * @param gnbNode The gNodeB node (unused in current P2P implementation)
 */
void SetupV2VDirectLinks(NodeContainer& vehicles, Ptr<Node> gnbNode);

/**
 * Create and configure all UE nodes with 5G NR devices
 * @param numRsus Number of RSU nodes to create
 * @param numVehicles Number of vehicle nodes to create
 */
void PreCreateAllNodes(int numRsus = 16, int numVehicles = 50);

#endif // NR_5G_SETUP_H
