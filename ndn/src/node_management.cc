#include "node_management.h"
#include "simulation_state.h"

#include "ns3/mobility-module.h"
#include "ns3/log.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NodeManagement");

// ============================================================================
// REVERSE ID MAPS (NS-3 → OMNeT++)
// ============================================================================
// These provide O(1) lookup for NS-3 ID → OMNeT++ ID translation.
// They are automatically populated when forward mappings are created.
// ============================================================================

static std::map<std::string, std::string> g_vehicleIdReverseMap;
static std::map<std::string, std::string> g_rsuIdReverseMap;

// Get or create a node (for runtime dynamic nodes - should not be called after Simulator::Run())
Ptr<Node> GetOrCreateNode(const std::string& id)
{
    auto it = nodeMapping.find(id);
    if (it != nodeMapping.end())
    {
        return it->second;
    }

    // If we get here during runtime, it means a new node was requested
    // that wasn't pre-created. This shouldn't happen with proper pre-creation.
    std::cerr << "⚠️  WARNING: Node " << id << " not found in pre-created pool!" << std::endl;
    std::cerr << "    All nodes should be pre-created before Simulator::Run()" << std::endl;
    return nullptr;
}

// Helper to update node position
void UpdateNodePosition(const std::string& id, double x, double y, double z)
{
    auto it = nodeMapping.find(id);
    if (it != nodeMapping.end())
    {
        Ptr<MobilityModel> mobility = it->second->GetObject<MobilityModel>();
        if (mobility)
        {
            mobility->SetPosition(Vector(x, y, z));
        }
    }
}

// Get or map RSU ID from OMNeT++ to pre-created pool
std::string GetOrMapRsuId(const std::string& omnetRsuId)
{
    // Check if we already have a mapping
    auto it = rsuIdMapping.find(omnetRsuId);
    if (it != rsuIdMapping.end())
    {
        return it->second; // Return existing mapping
    }

    // Need to allocate a new slot from the pool
    for (int i = 0; i < maxRsuSlots; i++)
    {
        if (usedRsuSlots.find(i) == usedRsuSlots.end())
        {
            // Found a free slot
            std::string preCreatedId = "rsu_" + std::to_string(i);
            rsuIdMapping[omnetRsuId] = preCreatedId;
            g_rsuIdReverseMap[preCreatedId] = omnetRsuId;  // Populate reverse map
            usedRsuSlots.insert(i);
            NS_LOG_DEBUG("Mapped OMNeT++ RSU '" << omnetRsuId << "' → NS-3 ID '" << preCreatedId << "'");
            std::cout << "  >> Mapped " << omnetRsuId << " -> " << preCreatedId
                      << " (" << usedRsuSlots.size() << "/" << maxRsuSlots << " slots used)" << std::endl;
            return preCreatedId;
        }
    }

    // Pool exhausted
    NS_LOG_WARN("RSU pool exhausted! Cannot map " << omnetRsuId);
    std::cerr << "  WARNING: RSU pool exhausted! Cannot map " << omnetRsuId << std::endl;
    return ""; // Return empty string to indicate failure
}

// Get or map Vehicle ID from OMNeT++ to pre-created pool
std::string GetOrMapVehicleId(const std::string& omnetVehId)
{
    // Check if we already have a mapping
    auto it = vehicleIdMapping.find(omnetVehId);
    if (it != vehicleIdMapping.end())
    {
        return it->second; // Return existing mapping
    }

    // Need to allocate a new slot from the pool
    for (int i = 0; i < maxVehicleSlots; i++)
    {
        if (usedVehicleSlots.find(i) == usedVehicleSlots.end())
        {
            // Found a free slot
            std::string preCreatedId = "veh_" + std::to_string(i);
            vehicleIdMapping[omnetVehId] = preCreatedId;
            g_vehicleIdReverseMap[preCreatedId] = omnetVehId;  // Populate reverse map
            usedVehicleSlots.insert(i);
            NS_LOG_DEBUG("Mapped OMNeT++ vehicle '" << omnetVehId << "' → NS-3 ID '" << preCreatedId << "'");
            std::cout << "  >> Mapped vehicle " << omnetVehId << " -> " << preCreatedId
                      << " (" << usedVehicleSlots.size() << "/" << maxVehicleSlots << " slots used)" << std::endl;
            return preCreatedId;
        }
    }

    // Pool exhausted
    NS_LOG_WARN("Vehicle pool exhausted! Cannot map " << omnetVehId);
    std::cerr << "  WARNING: Vehicle pool exhausted! Cannot map " << omnetVehId << std::endl;
    return ""; // Return empty string to indicate failure
}

// Get OMNeT++ RSU ID from NS-3 mapped ID (reverse lookup)
std::string GetOmnetRsuId(const std::string& ns3RsuId)
{
    // Use reverse map for O(1) lookup
    auto it = g_rsuIdReverseMap.find(ns3RsuId);
    if (it != g_rsuIdReverseMap.end())
    {
        return it->second;  // Return OMNeT++ ID
    }
    
    // Not found in reverse map - log warning
    NS_LOG_WARN("No reverse mapping for NS-3 RSU ID: " << ns3RsuId);
    return "";  // Return empty string, let caller handle fallback
}

// Get OMNeT++ Vehicle ID from NS-3 mapped ID (reverse lookup)
std::string GetOmnetVehicleId(const std::string& ns3VehId)
{
    // Use reverse map for O(1) lookup
    auto it = g_vehicleIdReverseMap.find(ns3VehId);
    if (it != g_vehicleIdReverseMap.end())
    {
        return it->second;  // Return OMNeT++ ID
    }
    
    // Not found in reverse map - log warning
    NS_LOG_WARN("No reverse mapping for NS-3 vehicle ID: " << ns3VehId);
    return "";  // Return empty string, let caller handle fallback
}

// Legacy functions (kept for compatibility but now do nothing)
void PreCreateRSUs(int numRsus)
{
    // RSUs are now created by PreCreateAllNodes()
    std::cout << "  (RSU creation handled by PreCreateAllNodes)" << std::endl;
}

void PreCreateVehicles(int numVehicles)
{
    // Vehicles are now created by PreCreateAllNodes()
    std::cout << "  (Vehicle creation handled by PreCreateAllNodes)" << std::endl;
}

// ============================================================================
// JSON PARSING HELPER FUNCTIONS
// ============================================================================

// Helper to extract JSON string value
std::string ExtractJsonString(const std::string& json, const std::string& key)
{
    size_t keyPos = json.find("\"" + key + "\"");
    if (keyPos != std::string::npos)
    {
        size_t colonPos = json.find(":", keyPos);
        if (colonPos != std::string::npos)
        {
            size_t valueStart = json.find("\"", colonPos);
            if (valueStart != std::string::npos)
            {
                size_t valueEnd = json.find("\"", valueStart + 1);
                if (valueEnd != std::string::npos)
                {
                    return json.substr(valueStart + 1, valueEnd - valueStart - 1);
                }
            }
        }
    }
    return "";
}

// Helper to extract JSON number value
double ExtractJsonNumber(const std::string& json, const std::string& key)
{
    size_t keyPos = json.find("\"" + key + "\"");
    if (keyPos != std::string::npos)
    {
        size_t colonPos = json.find(":", keyPos);
        if (colonPos != std::string::npos)
        {
            size_t valueStart = colonPos + 1;
            while (valueStart < json.length() && (json[valueStart] == ' ' || json[valueStart] == '\t'))
            {
                valueStart++;
            }
            size_t valueEnd = json.find_first_of(",}]", valueStart);
            if (valueEnd != std::string::npos)
            {
                try
                {
                    return std::stod(json.substr(valueStart, valueEnd - valueStart));
                }
                catch (...)
                {
                    return 0.0;
                }
            }
        }
    }
    return 0.0;
}

// Helper to extract nested count from meta
int ExtractCount(const std::string& json)
{
    size_t metaPos = json.find("\"meta\"");
    if (metaPos != std::string::npos)
    {
        size_t countPos = json.find("\"count\"", metaPos);
        if (countPos != std::string::npos)
        {
            size_t colonPos = json.find(":", countPos);
            if (colonPos != std::string::npos)
            {
                size_t valueStart = colonPos + 1;
                while (valueStart < json.length() && (json[valueStart] == ' ' || json[valueStart] == '\t'))
                {
                    valueStart++;
                }
                size_t valueEnd = json.find_first_of(",}]", valueStart);
                if (valueEnd != std::string::npos)
                {
                    try
                    {
                        return std::stoi(json.substr(valueStart, valueEnd - valueStart));
                    }
                    catch (...)
                    {
                        return 0;
                    }
                }
            }
        }
    }
    return 0;
}
