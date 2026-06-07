#ifndef __V2X_LEADER_SOCKETINTERFACE_H_
#define __V2X_LEADER_SOCKETINTERFACE_H_

#include <thread>
#include <atomic>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <unistd.h>
#include <omnetpp.h>
#include <sys/un.h>
#include <unistd.h> // for unlink()
#include <unordered_map>
#include <jsoncpp/json/json.h>
#include "v2xTypes.h"
#include <chrono>
#include "timeSynchronizer.h"
#include "metricsCollector.h"
#include "veins/modules/mobility/traci/TraCIScenarioManagerForker.h"

using namespace omnetpp;
using namespace veins;
class CentralRSUManager;
class SocketInterface : public cSimpleModule
{
private:
  // Module References
  TimeSynchronizer *timeSynchronizerModule = nullptr;
  TraCIScenarioManagerForker *traciManagerModule = nullptr;
  CentralRSUManager *mano = nullptr;
  MetricsCollector *metricsCollector = nullptr;

  // sent bytes signals
  simsignal_t sigSockMobilityBytes;
  simsignal_t sigSockRSUBytes;
  simsignal_t sigEmergencyReqBytes;
  simsignal_t sigSockAccidentBytes;
  simsignal_t sigSockEndOfStep;
  simsignal_t sigSockTerminate;
  simsignal_t sigSockEmptyPayload;
  simsignal_t sigSockManoDecisionBytes;

  // received bytes signals
  simsignal_t sigSockManoQueryBytes;
  simsignal_t sigSafetyCmdBytes;
  simsignal_t sigTrafficPreemptionBytes;
  simsignal_t sigTimeSyncBytes;

  std::unordered_map<std::string, RsuState> lastSentRsuStates;

private:
  void startVeinsManager();
  //  void runVeinsUpdateStep();  TO DO to dynamically change the update step during simulation

protected:
  int port;
  int server_fd = -1;
  int client_socket = -1;
  simtime_t timeStep;
  bool useTcp = true;
  std::thread server_thread;
  cMessage *socket_check_event = nullptr;
  cMessage *connection_check_event = nullptr;
  cMessage *create_mano = nullptr;
  cMessage *send_state_event = nullptr;

  std::atomic<bool> client_connected{false}; // : Use std::atomic for thread safety
  std::atomic<bool> server_thread_should_stop{false};

  // This flag is only accessed by the main OMNeT++ thread, so it doesn't need to be atomic.
  bool simulation_started = false;
  bool simulation_ended = false;
  bool processing_request = false;
  std::chrono::time_point<std::chrono::high_resolution_clock> wallClockStart;
  std::chrono::time_point<std::chrono::high_resolution_clock> wallClockEnd;

  std::string receiveBuffer;
  std::vector<PendingSafetyCmd> commandQueue;
  std::vector<MANOQuery> manoQueue;
  std::vector<TrafficPreemptionCommand> trafficPreemptionQueue;

protected:
  virtual void initialize() override;
  void createCentralModules();
  void startTcpServer();
  void startUnixServer();
  void sendNodeStates();
  bool isDataAvailableOnSocket();
  void handleClientData();
  void sendRSUStates();
  void sendEmptyPayload();
  void sendTerminate();
  void sendEndOfStep();
  void processSafetyMessages();
  void processManoQueue();
  void processTrafficPreemptionQueue();
  virtual void handleMessage(cMessage *msg) override;
  virtual void finish() override;

public:
  void sendCommandToClient(const std::string &command);
  void sendAccidentData(AccidentData accidentData);
  void sendManoResponse(MANOResponse response);
  void sendTrafficPreemption(TrafficPreemptionCommand command);
};

#endif
