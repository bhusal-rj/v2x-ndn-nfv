#ifndef NODE_MANAGEMENT_H
#define NODE_MANAGEMENT_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <string>

using namespace ns3;

// ============================================================================
// NODE MANAGEMENT AND ID MAPPING FUNCTIONS
// ============================================================================

/**
 * Get or create a node (for runtime dynamic nodes)
 * WARNING: Should not be called after Simulator::Run() - use pre-created nodes
 * @param id Node identifier
 * @return Pointer to the node, or nullptr if not found
 */
Ptr<Node> GetOrCreateNode(const std::string& id);

/**
 * Update node position in the simulation
 * @param id Node identifier
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 */
void UpdateNodePosition(const std::string& id, double x, double y, double z);

/**
 * Get or map RSU ID from OMNeT++ to pre-created pool
 * @param omnetRsuId OMNeT++ RSU identifier (e.g., "rsu_52")
 * @return Pre-created RSU ID (e.g., "rsu_0") or empty string if pool exhausted
 */
std::string GetOrMapRsuId(const std::string& omnetRsuId);

/**
 * Get or map Vehicle ID from OMNeT++ to pre-created pool
 * @param omnetVehId OMNeT++ vehicle identifier (e.g., "t_0", "f_4.0")
 * @return Pre-created vehicle ID (e.g., "veh_0") or empty string if pool exhausted
 */
std::string GetOrMapVehicleId(const std::string& omnetVehId);

/**
 * Get OMNeT++ RSU ID from NS-3 mapped ID (reverse lookup)
 * @param ns3RsuId NS-3 RSU identifier (e.g., "rsu_0")
 * @return OMNeT++ RSU ID (e.g., "rsu_j_5") or empty string if not found
 */
std::string GetOmnetRsuId(const std::string& ns3RsuId);

/**
 * Get OMNeT++ Vehicle ID from NS-3 mapped ID (reverse lookup)
 * @param ns3VehId NS-3 vehicle identifier (e.g., "veh_0")
 * @return OMNeT++ vehicle ID (e.g., "node[0]") or empty string if not found
 */
std::string GetOmnetVehicleId(const std::string& ns3VehId);

/**
 * Legacy function - RSUs are now created by PreCreateAllNodes()
 * @param numRsus Number of RSUs (ignored)
 */
void PreCreateRSUs(int numRsus = 16);

/**
 * Legacy function - Vehicles are now created by PreCreateAllNodes()
 * @param numVehicles Number of vehicles (ignored)
 */
void PreCreateVehicles(int numVehicles = 50);

// ============================================================================
// JSON PARSING HELPER FUNCTIONS
// ============================================================================

/**
 * Extract JSON string value by key
 * @param json JSON string to parse
 * @param key Key to search for
 * @return Extracted string value or empty string if not found
 */
std::string ExtractJsonString(const std::string& json, const std::string& key);

/**
 * Extract JSON number value by key
 * @param json JSON string to parse
 * @param key Key to search for
 * @return Extracted number value or 0.0 if not found
 */
double ExtractJsonNumber(const std::string& json, const std::string& key);

/**
 * Extract nested count from meta object in JSON
 * @param json JSON string containing meta object
 * @return Count value or 0 if not found
 */
int ExtractCount(const std::string& json);

#endif // NODE_MANAGEMENT_H
