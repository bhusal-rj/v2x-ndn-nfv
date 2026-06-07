#ifndef V2X_TYPES_H
#define V2X_TYPES_H

#include <string>
#include <vector>
#include <omnetpp.h>
#include "veins/base/utils/Coord.h"
using namespace omnetpp;
using namespace veins;
struct MANOQuery
{
    std::string queryId;
    std::string contentName;
    std::string laneId;
    std::string sourceRSU;
    double posX;
    double posY;
};
struct MANOResponse
{
    std::string queryId;
    std::string contentName;
    std::vector<std::string> rsuIds; // List of RSU IDs selected for proactive caching
};
struct AccidentData
{
    std::string sourceType; // "vehicle", "rsu", "proactive_cache"
    std::string laneId;
    std::string crashedVehId;
    double posX;
    double posY;
    std::string accidentId;
    double originTime;

    std::string action;    // "EMERGENCY_STOP", "HARD_BRAKE", "SLOW_DOWN", "CAUTION"
    std::string riskLevel; // "CRITICAL", "HIGH", "MEDIUM"
    double ttc;
    double distance;
};
struct PendingSafetyCmd
{
    std::string targetId;
    AccidentData accidentData;
};

struct TrafficPreemptionCommand
{
    std::string targetId; // RSU ID that should receive the command
    std::string tlsId;
    std::string laneId;
    std::string requestorId;
    double originTime;
};
struct RSUCandidate
{
    std::string id;
    Coord position;
    bool isJunction;        // True = Permanent, False = Dynamic
    std::string roadId;     // The road this covers (for In-Fill)
    std::string junctionId; // For Junction RSUs
    bool active = false;
    cModule *module = nullptr; // Pointer to actual RSU if instantiated
    simtime_t lowDensityStartTime = 0;
};
struct JunctionInfo
{
    std::string id;
    std::vector<std::string> outgoingEdges; // Roads starting here
    // We mostly care about outgoing, because these feed into the network
};
// Structure for the Upstream Graph
struct UpstreamConnection
{
    std::string sourceEdgeId;
    std::string direction; // How the source connects to us
};
struct NodeState
{
    std::string id;
    std::string nodeName;
    Coord position;
    double speed;
    double heading;
    double acceleration;
    bool parkingState;
    std::string vehicleType;
    std::string laneId;
    // std::vector<std::string> signalState;
};
struct RsuState
{
    std::string id;                // RSU Module Name
    Coord position;                // RSU Coordinates
    bool isTrafficLight;           // True if RSU is at a traffic light junction
    std::string tlsId;             // Associated Traffic Light ID (if applicable)
    std::string trafficLightState; // Current traffic light phase
    int vehicleCount;              // Number of vehicles currently at this RSU
    // Equality operator overload
    bool operator==(const RsuState &other) const
    {
        return id == other.id &&
               position.x == other.position.x &&
               position.y == other.position.y &&
               position.z == other.position.z &&
               isTrafficLight == other.isTrafficLight &&
               trafficLightState == other.trafficLightState &&
               tlsId == other.tlsId &&
               vehicleCount == other.vehicleCount;
    }

    // Inequality operator
    bool operator!=(const RsuState &other) const
    {
        return !(*this == other);
    }
};

#endif