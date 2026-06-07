#ifndef __V2X_LEADER_METRICSCOLLECTOR_H_
#define __V2X_LEADER_METRICSCOLLECTOR_H_

#include <omnetpp.h>
#include <fstream> // Required for std::ofstream
using namespace omnetpp;

class MetricsCollector : public cSimpleModule, public cListener
{
private:
  //  1. FILE STREAMS 
  std::ofstream fileInfrastructure;
  std::ofstream fileServiceFlows;  
  std::ofstream fileTraffic;        
  std::ofstream fileSentSocket;     
  std::ofstream fileRecvSocket;     

  //  2. PARAMETERS 
  simtime_t interval;
  cMessage *collectionTimer = nullptr;

  //  3. MODULE REFERENCES 
  cModule *centralRSUManager = nullptr;

  //  4. METRIC ACCUMULATORS (Reset every interval) 
  // Category A: Infrastructure (MANO)
  long cntScaleOut = 0;
  long cntScaleIn = 0;
  double sumCreateTime = 0.0;
  long cntCreateTime = 0;
  double sumDeleteTime = 0.0;
  long cntDeleteTime = 0;

  // Category B: Service Flows (QoS)
  long cntVehEmergSent = 0;
  long cntRsuEmergRx = 0;
  long cntCoreEmergProcessed = 0;
  double sumTlsLatency = 0.0;
  long cntTlsLatency = 0;
  double sumVehicleActionLatency = 0.0;
  long cntVehicleActionLatency = 0;
  long cntVehAccSent = 0;

  // Category C: Context (Client)
  long cntVehAccReactRSU = 0;
  long cntVehAccReactV2V = 0;
  long cntVehAccReactRSUStrat = 0;
  long cntVehAccRxV2V = 0;
  long cntVehAccRxRSU = 0;
  long cntVehAccRxStrat = 0;
  long cntAtRedLight = 0;
  long cntEmergRedLight = 0;

  // --- 5. SIGNAL HANDLES ---
  // Infrastructure
  simsignal_t sigRsuScaledOut;
  simsignal_t sigRsuScaledIn;
  simsignal_t sigRsuCreationTime; // Double
  simsignal_t sigRsuDeletionTime; // Double

  // Service Flows
  simsignal_t sigVehEmergSent;
  simsignal_t sigRsuEmergRx;
  simsignal_t sigCoreEmergProcessed;
  simsignal_t sigTlsLatency; // Double
  simsignal_t sigVehActionLatency;
  simsignal_t sigVehAccSent;

  // Context
  simsignal_t sigVehAccReactRSU;
  simsignal_t sigVehAccReactV2V;
  simsignal_t sigVehAccReactRSUStrat;
  simsignal_t sigVehAccRxV2V;
  simsignal_t sigVehAccRxRSU;
  simsignal_t sigVehAccRxStrat;
  simsignal_t sigAtRedLight;
  simsignal_t sigEmergRedLight;

  // Socket Sent bytes signal
  simsignal_t sigSockMobility;
  simsignal_t sigSockRSU;
  simsignal_t sigEmergencyReqBytes;
  simsignal_t sigSockAccident;
  simsignal_t sigSockEndOfStep;
  simsignal_t sigSockTerminate;
  simsignal_t sigSockEmptyPayload;
  // Socket Received bytes signals
  simsignal_t sigSockManoQueryBytes;
  simsignal_t sigSafetyCmdBytes;
  simsignal_t sigTrafficPreemptionBytes;
  simsignal_t sigTimeSyncBytes;

  // Sent bytes
  long bytesMobility = 0;
  long bytesRSU = 0;
  long bytesAccident = 0;
  long bytesEmergencyReq = 0;
  long bytesStepSent = 0;
  long bytesTerminateSent = 0;
  long bytesEmptyPayloadSent = 0;
  //Received Bytes 
  long bytesManoQueryReceived = 0;
  long bytesSafetyCmdReceived = 0;
  long bytesTrafficPreemptionReceived = 0;
  long bytesTimeSyncReceived = 0;

protected:
  virtual void initialize() override;
  virtual void handleMessage(cMessage *msg) override;
  virtual void finish() override;
  // Signal Receivers
  virtual void receiveSignal(cComponent *source, simsignal_t id, long value, cObject *details) override;
  virtual void receiveSignal(cComponent *source, simsignal_t id, double value, cObject *details) override;
  // Core Logic
  void writeMetricRows();
};
#endif
