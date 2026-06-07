
#ifndef __V2X_LEADER_VEHICLEAPP_H_
#define __V2X_LEADER_VEHICLEAPP_H_

#include <list>
#include <set>
#include <omnetpp.h>
#include "v2xTypes.h"
#include "socketInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using namespace omnetpp;
using namespace veins;
class VehicleApp : public cSimpleModule, public cListener
{
private:
    std::string vehicleId;
    // Data Structures
    std::set<std::string> processedAccidentIds;
    std::set<std::string> receivedViaRSU;
    std::set<std::string> receivedViaV2V;
    // Module References
    TraCIMobility *mobility = nullptr;
    TraCICommandInterface::Vehicle *vehicle = nullptr;
    TraCICommandInterface *traci = nullptr;
    SocketInterface *socketInterface;
    TimeSynchronizer *timeSynchronizerModule = nullptr;
    // Signals for metric collection
    simsignal_t sigVehEmergSent;
    simsignal_t sigVehAccReactV2V;
    simsignal_t sigVehAccReactRSU;
    simsignal_t sigVehAccRxV2V;
    simsignal_t sigVehAccRxRSU;
    simsignal_t sigVehAccRxStrat;
    simsignal_t sigVehActionLatency;
    simsignal_t sigAtRedLight;
    simsignal_t sigEmergRedLight;
    simsignal_t sigVehAccReactRSUStrat;
    simsignal_t sigVehAccSent;
    // Utility for accident functions
    simtime_t lastAccidentTime = 0;
    simtime_t collisionResetThreshold;
    simtime_t lastDroveAt;

    bool sentMessage = false;          // Set when accident vehicle sends message
    bool enablePreemption = false;     // Set to true to enable preemption at traffic lights
    cMessage *checkTlsEvent = nullptr; // Message to check tls proximity
    bool isEmergencyVehicle = false;   // To distinguish Normal vehicle and emergency
    simtime_t checkInterval = 1.0;     // seconds Time interval to check tls proximity
    double ttlPreemptionDistance;

protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    void handlePositionUpdate(cObject *obj);
    void handleParkingUpdate(cObject *obj);
    void checkTrafficLightProximity();
    void checkCollisionEventForVehicle();
    std::string getStopReasonString(uint8_t stopState);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;

public:
    // Utility Functions
    virtual Coord getCurrentPosition();
    virtual double getCurrentSpeed();
    virtual double getHeading();
    virtual double getAcceleration();
    virtual bool getParkingState();
    std::string getLaneId();
    virtual std::string getVehicleId();
    std::string getNodeName();
    virtual std::string getVehicleType();
    virtual std::vector<std::string> getVehicleSignals();
    // Accident Collision Handling and Prevention
    void handleNetworkAccident(AccidentData accidentData);
};
#endif
