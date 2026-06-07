#include "headers/rsuApp.h"

Define_Module(RsuApp);

void RsuApp::initialize(int stage)
{
    sigRsuAccRx = registerSignal("nfvRsuAccidentRx");
    sigRsuEmergRx = registerSignal("nfvRsuEmergRx");
    sigTlsLatency = registerSignal("nfvTlsSwitchLatency");
    sigCoreEmergProcessed = registerSignal("nfvCoreEmergProc");
    manager = TraCIScenarioManagerAccess().get();
    traci = manager->getCommandInterface();
    if (!tlsId.empty())
    {
        trafficLight = new TraCICommandInterface::Trafficlight(traci, tlsId);
    }
}

int RsuApp::getVehicleCount()
{
    auto hosts = manager->getManagedHosts();
    int count = 0;
    for (const auto &entry : hosts)
    {
        std::string vid = entry.first;
        auto nextTlsList = traci->vehicle(vid).getNextTls();
        for (const auto &t : nextTlsList)
        {
            std::string nextTlsId;
            int index;
            double distance;
            char state;
            std::tie(nextTlsId, index, distance, state) = t;
            if (!(tlsId == nextTlsId))
                continue;

            double threshold = 30.0;
            if (distance <= threshold)
                count += 1;
        }
    }
    return count;
}
std::string RsuApp::getCurrentTrafficState()
{
    if (trafficLight)
    {
        return trafficLight->getCurrentState();
    }
    return "";
}
bool RsuApp::getJunctionState()
{
    return isJunction;
}
void RsuApp::setOperationalState(bool isActive)
{
    this->isOperational = isActive;
}
void RsuApp::setJunctionState(bool isJunction)
{
    this->isJunction = isJunction;
}
void RsuApp::setTlsId(std::string tlsId)
{
    this->tlsId = tlsId;
}
void RsuApp::handleMessage(cMessage *msg)
{
}
void RsuApp::handlePreemptionCommand(TrafficPreemptionCommand trafficLightPreemption)
{
    if (!isJunction || !isOperational)
    {
        std::cout << "RSU inactive or not a junction" << std::endl;
        return;
    }

    if (tlsId.empty())
    {
        std::cout << "Missing TLS ID" << std::endl;
        return;
    }

    if (tlsId != trafficLightPreemption.tlsId)
    {
        std::cout << "TLS ID mismatch" << std::endl;
        return;
    }

    emit(sigRsuEmergRx, 1);
    double sendTime = trafficLightPreemption.originTime;
    std::string senderId = trafficLightPreemption.requestorId;
    std::string laneId = trafficLightPreemption.laneId;

    if (trafficLight)
    {

        std::string originalState = trafficLight->getCurrentState();
        std::string modifiedState = originalState;
        auto program = trafficLight->getProgramDefinition(); // TraCITrafficLightProgram
        std::string programId = trafficLight->getCurrentProgramID();
        TraCITrafficLightProgram::Logic logic = program.getLogic(programId.c_str());

        int numPhases = logic.phases.size();

        int targetPhaseIndex = -1;
        auto controlledLinks = trafficLight->getControlledLinks();
        for (int i = 0; i < numPhases; ++i)
        {
            const std::string &phaseState = logic.phases[i].state;
            int idx = 0;
            for (const auto &linkGroup : controlledLinks)
            {
                for (const auto &link : linkGroup)
                {
                    if (link.incoming == laneId)
                    {
                        if (phaseState[idx] == 'G')
                        {
                            targetPhaseIndex = i;
                            break;
                        }
                    }
                    idx++;
                }
                if (targetPhaseIndex != -1)
                    break;
            }
            if (targetPhaseIndex != -1)
                break;
        }
        if (targetPhaseIndex != -1)
        {
            int emergencyGreenTime = 10;                        // seconds
            trafficLight->setPhaseIndex(targetPhaseIndex);      // switch immediately to desired phase
            trafficLight->setPhaseDuration(emergencyGreenTime); // keep for specified seconds
            double latency = simTime().dbl() - sendTime;
            emit(sigTlsLatency, latency);
            emit(sigCoreEmergProcessed, 1);
        }
        else
        {
            std::cout << "No suitable phase found to set green for lane " << laneId << std::endl;
        }
    }
}
std::string RsuApp ::getTlsId()
{
    return this->tlsId;
}
void RsuApp::finish()
{
}
