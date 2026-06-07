#include "headers/vehicleApp.h"
Define_Module(VehicleApp);

void VehicleApp::initialize(int stage)
{
    sigVehAccReactV2V = registerSignal("nfvVehAccidentReactV2V");
    sigVehAccReactRSU = registerSignal("nfvVehAccidentReactRSU");
    sigVehAccRxV2V = registerSignal("nfvVehicleAccRxV2V");
    sigVehAccRxRSU = registerSignal("nfvVehicleAccRxRSU");
    sigVehAccRxStrat = registerSignal("nfvVehicleAccRxStrat");
    sigVehAccReactRSUStrat = registerSignal("nfvVehAccidentReactRSUStrat");
    sigVehActionLatency = registerSignal("nfvVehicleActLatency");
    sigAtRedLight = registerSignal("nfvAtRedLight");
    sigEmergRedLight = registerSignal("nfvEmergRedLight");
    sigVehAccSent = registerSignal("nfvVehAccSent");
    socketInterface = check_and_cast<SocketInterface *>(
        getModuleByPath("socketInterface"));
    timeSynchronizerModule = check_and_cast<TimeSynchronizer *>(getModuleByPath("timeSynchronizer"));
    collisionResetThreshold = timeSynchronizerModule->getTimeStep();
    ttlPreemptionDistance = par("ttlPreemptionDistance").doubleValue();
    enablePreemption = par("enablePreemption").boolValue();

    if (stage == 0)
    {
        sentMessage = false;
        lastDroveAt = simTime();
        if (FindModule<TraCIMobility *>::findSubModule(getParentModule()))
        {
            mobility = TraCIMobilityAccess().get(getParentModule());
            mobility->subscribe(BaseMobility::mobilityStateChangedSignal, this);
            mobility->subscribe(TraCIMobility::parkingStateChangedSignal, this);
            traci = mobility->getCommandInterface();
            vehicle = mobility->getVehicleCommandInterface();
        }

        vehicleId = mobility->getExternalId();

        std::string vehicleType = getVehicleType();

        if (vehicleType == "emergency" || vehicleType == "ambulance" || vehicleType == "fire_truck")
        {
            isEmergencyVehicle = true;
            sigVehEmergSent = registerSignal("nfvVehEmergSent");
            checkTlsEvent = new cMessage("checkTlsEvent");
            std::cout << "Scheduling TLS check for emergency vehicle " << vehicleId << std::endl;
            if (enablePreemption)
            {
                scheduleAt(simTime() + checkInterval, checkTlsEvent);
            }
        }
    }
}
void VehicleApp::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal)
    {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal)
    {
        handleParkingUpdate(obj);
    }
}
void VehicleApp::handleParkingUpdate(cObject *obj)
{
    bool isParked = mobility->getParkingState();
    if (isParked)
    {
        std::cout << "Vehicle " << vehicleId << " has parked at " << simTime() << std::endl;
    }
    else
    {
        std::cout << "Vehicle " << vehicleId << " has unparked at " << simTime() << std::endl;
    }
}
std::string VehicleApp::getStopReasonString(uint8_t state)
{
    std::string reason = "";

    // Check individual bits
    if (state & 1)
        reason += "[Stopped] ";
    if (state & 2)
        reason += "[Parking] ";
    if (state & 4)
        reason += "[Triggered] ";
    if (state & 8)
        reason += "[ContainerTriggered] ";
    if (state & 16)
        reason += "[AtBusStop] ";
    if (state & 32)
        reason += "[AtContainerStop] ";
    if (state & 64)
        reason += "[AtChargingStation] ";
    if (state & 128)
        reason += "[AtParkingArea] ";

    if (reason.empty())
        return "Moving/Unknown";
    return reason;
}

void VehicleApp::handlePositionUpdate(cObject *obj)
{
    double stopSpeedThreshold = 0.1;     // Lower threshold for better precision
    double tlsProximityThreshold = 25.0; // Increased to account for vehicle queues
    auto nextTLS = vehicle->getNextTls();
    bool atRedLight = false;

    if (traci)
    {
        checkCollisionEventForVehicle();
    }
    if (sentMessage)
    {
        // After 3 seconds of last accident we can repor
        if (simTime() - lastAccidentTime > collisionResetThreshold)
        {
            sentMessage = false;
            std::cout << "Resetting sentMessage after " << collisionResetThreshold << "s for vehicle " << vehicleId << " at " << simTime().dbl() << endl;
            return;
        }
    }

    // 1. Identify if vehicle is at a red light link
    for (const auto &tpl : nextTLS)
    {
        double distance = std::get<2>(tpl);
        char state = std::get<3>(tpl);

        // State 'r' is red; 'u' is red-yellow (often treated as red)
        if (distance <= tlsProximityThreshold && (state == 'r'))
        {
            atRedLight = true;
            break;
        }
    }

    double currentSpeed = mobility->getSpeed();

    // 2. Logic for timing the stop
    if (currentSpeed > stopSpeedThreshold)
    {
        // Vehicle is moving; reset the timer to current time
        lastDroveAt = simTime();
    }
    else
    {
        // Vehicle is stopped. Check if it has been stopped for >= 5 seconds
        if (atRedLight && (simTime() - lastDroveAt >= 5))
        {
            emit(sigAtRedLight, 1);
            if (isEmergencyVehicle)
            {
                emit(sigEmergRedLight, 1);
            }
        }
    }
}
void VehicleApp::finish()
{
    processedAccidentIds.clear();

    if (checkTlsEvent)
    {
        cancelAndDelete(checkTlsEvent);
    }
}

void VehicleApp::handleMessage(cMessage *msg)
{
    if (msg == checkTlsEvent)
    {
        checkTrafficLightProximity();
        scheduleAt(simTime() + checkInterval, checkTlsEvent);
        return;
    }

    delete msg;
}

void VehicleApp::handleNetworkAccident(AccidentData accidentData)
{
    std::cout << "Received accident reaction for " << accidentData.accidentId << endl;
    std::string ownId = mobility->getExternalId();
    std::string ownLaneId = vehicle->getLaneId();

    // Check if we already counted this metric (Deduplication)
    bool isRSU = (accidentData.sourceType.find("rsu") != std::string::npos);
    bool isVehicle = (accidentData.sourceType.find("vehicle") != std::string::npos);
    bool isProactiveCache = (accidentData.sourceType.find("proactive_cache") != std::string::npos);

    if (isRSU && receivedViaRSU.find(accidentData.accidentId) == receivedViaRSU.end())
    {
        emit(sigVehAccRxRSU, 1);
        receivedViaRSU.insert(accidentData.accidentId);
    }
    if (isProactiveCache && receivedViaRSU.find(accidentData.accidentId) == receivedViaRSU.end())
    {
        emit(sigVehAccRxStrat, 1);
        receivedViaRSU.insert(accidentData.accidentId);
    }
    if (isVehicle && receivedViaV2V.find(accidentData.accidentId) == receivedViaV2V.end())
    {
        emit(sigVehAccRxV2V, 1);
        receivedViaV2V.insert(accidentData.accidentId);
    }

    if (processedAccidentIds.find(accidentData.accidentId) != processedAccidentIds.end())
    {
        return; // Already executed maneuvers for this specific event
    }

    // Identify if this is a direct MEC active command
    bool isCollisionAvoidance = (accidentData.accidentId.rfind("collision_", 0) == 0 || !accidentData.action.empty());

    if (isCollisionAvoidance)
    {
        // --- PATH A: DIRECT ACTIVE ACTUATION (MEC DECIDED) ---
        double targetSpeed = 0.0;
        double decelTime = 0.0;
        processedAccidentIds.insert(accidentData.accidentId);

        if (isRSU)
            emit(sigVehAccReactRSU, 1);
        if (isProactiveCache)
            emit(sigVehAccReactRSUStrat, 1);
        if (isVehicle)
            emit(sigVehAccReactV2V, 1);

        // Map ns-3 string commands to SUMO decel durations
        if (accidentData.action == "EMERGENCY_STOP" || accidentData.action == "EMERGENCY_BRAKE")
        {
            targetSpeed = 0.0;
            decelTime = 0.5; // Instant braking
        }
        else if (accidentData.action == "HARD_BRAKE")
        {
            targetSpeed = std::max(0.0, getCurrentSpeed() * 0.2);
            decelTime = 1.5;
        }
        else if (accidentData.action == "SLOW_DOWN")
        {
            targetSpeed = std::max(0.0, getCurrentSpeed() * 0.5);
            decelTime = 3.0;
        }
        else if (accidentData.action == "CAUTION")
        {
            targetSpeed = std::max(0.0, getCurrentSpeed() * 0.8);
            decelTime = 5.0;
        }

        if (decelTime > 0.0)
        {
            std::cout << "MEC Active Actuation applied: " << accidentData.action << " on " << ownId << endl;
            vehicle->slowDown(targetSpeed, decelTime);
        }

        double vehicleActionLatency = simTime().dbl() - accidentData.originTime;
        emit(sigVehActionLatency, vehicleActionLatency);
    }
    else
    {
        // --- PATH B: PASSIVE WARNING (VEHICLE-SIDE CALCULATION) ---
        bool sameLane = (accidentData.laneId == ownLaneId);

        if (sameLane)
        {
            Coord myPos = getCurrentPosition();
            Coord accidentPos = Coord(accidentData.posX, accidentData.posY, 0);
            double distance = myPos.distance(accidentPos);
            double mySpeed = mobility->getSpeed();

            double targetSpeed = mySpeed;
            double decelTime = 0.0;
            processedAccidentIds.insert(accidentData.accidentId);

            if (isRSU)
                emit(sigVehAccReactRSU, 1);
            if (isProactiveCache)
                emit(sigVehAccReactRSUStrat, 1);
            if (isVehicle)
                emit(sigVehAccReactV2V, 1);

            // Graded fallback braking based on distance
            if (distance < 10.0)
            {
                targetSpeed = 0.0;
                decelTime = 1.0;
            }
            else if (distance < 30.0)
            {
                targetSpeed = std::max(0.0, mySpeed * 0.2);
                decelTime = 2.0;
            }
            else if (distance < 60.0)
            {
                targetSpeed = std::max(0.0, mySpeed * 0.5);
                decelTime = 3.0;
            }

            if (decelTime > 0.0)
            {
                std::cout << "Passive Warning Braking applied on " << ownId << " (Dist: " << distance << "m)" << endl;
                vehicle->slowDown(targetSpeed, decelTime);
            }

            double vehicleActionLatency = simTime().dbl() - accidentData.originTime;
            emit(sigVehActionLatency, vehicleActionLatency);
        }
    }
}

void VehicleApp::checkTrafficLightProximity()
{
    if (!isEmergencyVehicle)
        return;

    auto nextTls = vehicle->getNextTls();
    for (const auto &tpl : nextTls)
    {
        const std::string &tlsId = std::get<0>(tpl);
        int tlsLinkIndex = std::get<1>(tpl);
        double distance = std::get<2>(tpl);
        char state = std::get<3>(tpl);
        // if vehicle is close to TLS and the light is red for its lane
        if (distance <= ttlPreemptionDistance && state == 'r')
        {
            TrafficPreemptionCommand request;
            request.tlsId = tlsId;
            request.requestorId = getParentModule()->getFullName();
            request.laneId = getLaneId();
            request.originTime = simTime().dbl();
            socketInterface->sendTrafficPreemption(request);
            emit(sigVehEmergSent, 1);
        }
    }
}
void VehicleApp::checkCollisionEventForVehicle()
{
    // 1. Check TraCI List (The Source of Truth)
    std::list<std::string> collisionVehicleIds = traci->getCollidingVehiclesIDList();

    if (collisionVehicleIds.empty())
        return;

    // 2. Am I involved?
    auto it = std::find(collisionVehicleIds.begin(), collisionVehicleIds.end(), vehicleId);

    if (it != collisionVehicleIds.end())
    {
        // I am in the list.
        lastAccidentTime = simTime();

        if (!sentMessage) // Only report the *start* of the accident
        {
            // 3. IMMEDIATE REPORTING (Zero Latency)
            // Don't wait for speed == 0.
            AccidentData accidentData;
            std::string nodeId = getParentModule()->getFullName();

            accidentData.laneId = getLaneId();
            accidentData.crashedVehId = nodeId;
            Coord pos = getCurrentPosition();
            accidentData.posX = pos.x;
            accidentData.posY = pos.y;
            accidentData.accidentId = ("crash_" + vehicleId + "_" + std::to_string((int)simTime().dbl()));
            accidentData.originTime = simTime().dbl();

            socketInterface->sendAccidentData(accidentData);

            emit(sigVehAccSent, 1);

            double currentSpeed = getCurrentSpeed();
            double deceleration = getAcceleration();

            std::cout << "CRITICAL: Collision detected! Speed: " << currentSpeed
                      << " Accel: " << deceleration << endl;

            sentMessage = true;
        }
    }
}
Coord VehicleApp::getCurrentPosition()
{
    return mobility->getCurrentPosition();
}
std::string VehicleApp::getNodeName()
{
    return getParentModule()->getFullName();
}
std::string VehicleApp::getVehicleId()
{
    return vehicleId;
}
double VehicleApp::getCurrentSpeed()
{
    return mobility->getSpeed();
}
double VehicleApp::getAcceleration()
{
    return vehicle->getAcceleration();
}
double VehicleApp::getHeading()
{
    return mobility->getHeading().getRad();
}
bool VehicleApp::getParkingState()
{
    return mobility->getParkingState();
}
std::string VehicleApp::getVehicleType()
{
    return vehicle->getTypeId();
}
std::string VehicleApp::getLaneId()
{
    std::string laneId = vehicle->getLaneId();

    return laneId;
}

std::vector<std::string> VehicleApp::getVehicleSignals()
{
    VehicleSignalSet signals = mobility->getSignals();
    std::vector<std::string> activeSignals;

    for (int i = 0; i <= static_cast<int>(veins::VehicleSignal::undefined); ++i)
    {
        auto signal = static_cast<veins::VehicleSignal>(i);
        if (signals.test(signal))
        {
            switch (signal)
            {
            case veins::VehicleSignal::blinker_right:
                activeSignals.push_back("blinker_right");
                break;
            case veins::VehicleSignal::blinker_left:
                activeSignals.push_back("blinker_left");
                break;
            case veins::VehicleSignal::blinker_emergency:
                activeSignals.push_back("blinker_emergency");
                break;
            case veins::VehicleSignal::brakelight:
                activeSignals.push_back("brakelight");
                break;
            case veins::VehicleSignal::frontlight:
                activeSignals.push_back("frontlight");
                break;
            case veins::VehicleSignal::foglight:
                activeSignals.push_back("foglight");
                break;
            case veins::VehicleSignal::highbeam:
                activeSignals.push_back("highbeam");
                break;
            case veins::VehicleSignal::backdrive:
                activeSignals.push_back("backdrive");
                break;
            case veins::VehicleSignal::wiper:
                activeSignals.push_back("wiper");
                break;
            case veins::VehicleSignal::door_open_left:
                activeSignals.push_back("door_open_left");
                break;
            case veins::VehicleSignal::door_open_right:
                activeSignals.push_back("door_open_right");
                break;
            case veins::VehicleSignal::emergency_blue:
                activeSignals.push_back("emergency_blue");
                break;
            case veins::VehicleSignal::emergency_red:
                activeSignals.push_back("emergency_red");
                break;
            case veins::VehicleSignal::emergency_yellow:
                activeSignals.push_back("emergency_yellow");
                break;
            case veins::VehicleSignal::undefined:
                break;
            }
        }
    }

    return activeSignals;
}
