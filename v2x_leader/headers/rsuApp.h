#ifndef __V2X_LEADER_RSUAPP_H_
#define __V2X_LEADER_RSUAPP_H_

#include <omnetpp.h>
#include "v2xTypes.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

using namespace omnetpp;
using namespace veins;
class RsuApp : public cSimpleModule
{
private:
    // Signals
    simsignal_t sigRsuAccRx;
    simsignal_t sigRsuEmergRx;
    simsignal_t sigTlsLatency;
    simsignal_t sigCoreEmergProcessed;
    // RSU parameters
    bool isOperational = true;
    bool isJunction = false;
    std::string tlsId = "";
    // Module References
    TraCIScenarioManager *manager;
    TraCICommandInterface *traci;
    TraCICommandInterface::Trafficlight *trafficLight;
    // std::unordered_map<int, std::string> emergencyVehicleRequests;

protected:
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

public:
    void handlePreemptionCommand(TrafficPreemptionCommand trafficLightPreemption);
    void setOperationalState(bool isActive);
    void setJunctionState(bool isJunction);
    void setTlsId(std::string tlsId);
    std::string getTlsId();
    std::string getCurrentTrafficState();
    int getVehicleCount();
    bool getJunctionState();
};
#endif
