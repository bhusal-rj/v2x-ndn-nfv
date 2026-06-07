#ifndef __V2X_LEADER_CENTRALRSUMANAGER_H_
#define __V2X_LEADER_CENTRALRSUMANAGER_H_

#include <map>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <omnetpp.h>
#include "rsuApp.h"
#include "vehicleApp.h"
#include "v2xTypes.h"

class SocketInterface;
using namespace omnetpp;
using namespace veins;

class CentralRSUManager : public cSimpleModule
{
private:
  // Emitting signals
  simsignal_t sigRsuCreationLatency;
  simsignal_t sigRsuDeletionLatency;
  // Module references
  TraCIScenarioManager *manager;
  TraCICommandInterface *traci;
  SocketInterface *socketInterfaceModule;
  // Data Structures
  std::vector<RSUCandidate> candidateList;
  std::vector<Coord> placedLocations;
  std::map<std::string, JunctionInfo> junctionTopology;
  std::unordered_map<std::string, NodeState> nodeStates;
  std::unordered_map<std::string, RsuState> rsuStates;
  std::map<std::string, std::vector<UpstreamConnection>> upstreamMap;
  // Simulation parameters
  int rsuSpacing;
  int minSpacingFromJunctionRsus;
  int rsuCoverage;
  std::string rsuModuleType;
  double updateInterval;
  int deploymentThreshold;
  int removalThreshold;
  bool enableDynamicRsu;
  double hysteresisTime;

protected:
  // helper functions
  RSUCandidate *findCandidate(std::string id);
  std::string getRoadId(std::string laneId) const;
  cMessage *updateEvent;

public:
  int getActiveRSUCount() const;
  void handleVNFQuery(MANOQuery query);
  int getTotalVehicleCount() const;
  int getEmergencyVehicleCount();
  double getAverageNetworkSpeed() const;
  void updateAllNodes();
  void updateRSUStates();
  std::unordered_map<std::string, RsuState> getRSUStates();
  std::unordered_map<std::string, NodeState> getAllNodeStates();

protected:
  void initialize() override;
  void handleMessage(cMessage *msg) override;
  void buildTopologyGraph();
  void finish() override;
  void buildJunctionCandidates();
  void buildInFillCandidates();
  Coord getPointOnPolyline(const std::vector<Coord> &shape, double targetDist);
  // Core Logic Functions
  void updateRSUPlacement();
  // RSU Instantiation/Activation/Deactivation
  void createRSU(RSUCandidate &cand);
  void deactivateRSU(RSUCandidate &cand);
};
#endif
