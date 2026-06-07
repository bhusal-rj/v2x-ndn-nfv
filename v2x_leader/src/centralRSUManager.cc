#include "headers/centralRSUManager.h"
#include "headers/socketInterface.h"
Define_Module(CentralRSUManager);

void CentralRSUManager::initialize()
{
    // 1. Read parameters
    updateInterval = par("updateInterval").doubleValue();
    rsuCoverage = par("rsuCoverage");
    enableDynamicRsu = par("enableDynamicRsu");
    minSpacingFromJunctionRsus = par("minSpacingFromJunctionRsus");
    rsuSpacing = par("rsuSpacing");
    deploymentThreshold = par("deploymentThreshold").intValue();
    removalThreshold = par("removalThreshold").intValue();
    hysteresisTime = par("hysteresisTime").doubleValue();
    rsuModuleType = par("rsuModuleType").stdstringValue();
    sigRsuCreationLatency = registerSignal("nfvRsuCreationTimeMs");
    sigRsuDeletionLatency = registerSignal("nfvRsuDeletionTimeMs");
    socketInterfaceModule = check_and_cast<SocketInterface *>(
        getModuleByPath("socketInterface"));
    manager = TraCIScenarioManagerAccess().get();
    traci = manager->getCommandInterface();
    buildTopologyGraph();
    buildJunctionCandidates();
    buildInFillCandidates();

    // Immediately activate the Permanent ones
    for (auto &cand : candidateList)
    {
        if (cand.isJunction || !enableDynamicRsu)
        {
            createRSU(cand);
        }
    }

    // 3. Schedule the first update event
    if (enableDynamicRsu)
    {

        updateEvent = new cMessage("updateEvent");
        scheduleAt(simTime() + updateInterval, updateEvent);
    }
}

void CentralRSUManager::buildJunctionCandidates()
{
    if (!manager)
        return;

    auto trafficJunctions = traci->getTrafficlightIds();
    auto junctionIds = traci->getJunctionIds();
    std::unordered_set<std::string> validJunctions(junctionIds.begin(), junctionIds.end());

    int jCount = 0;
    for (auto jID : trafficJunctions)
    {
        Coord rawPos;

        if (validJunctions.find(jID) == validJunctions.end())
        {
            // Case: Traffic Light exists, but there is no Junction with this ID.
            // This happens if the TLS controls multiple nodes or is a virtual TLS.
            std::cout << "Skipping Traffic Light '" << jID << "' - No matching Junction ID found for positioning." << endl;
            continue;
        }

        rawPos = traci->junction(jID).getPosition();
        JunctionInfo info;
        info.id = jID;
        // Convert list to vector for easier usage
        std::list<std::string> out = traci->junction(jID).getOutgoingEdges();
        info.outgoingEdges.assign(out.begin(), out.end());
        // Map RSU ID to this info
        std::string rsuId = "rsu_j_" + std::to_string(jCount);
        junctionTopology[rsuId] = info;
        RSUCandidate cand;
        cand.id = rsuId;
        cand.position = rawPos;
        cand.isJunction = true;
        cand.active = false;
        cand.junctionId = jID;
        candidateList.push_back(cand);
        placedLocations.push_back(rawPos);
        jCount++;
    }

    std::cout << "MANO: Junction RSUs  "
              << candidateList.size() << endl;
}
void CentralRSUManager::buildInFillCandidates()
{
    auto edges = traci->getRoadIds();

    for (auto edge : edges)
    {
        // Skip internal
        if (edge.rfind(":", 0) == 0)
            continue;

        double len = traci->lane(edge + "_0").getLength();
        if (len <= rsuSpacing)
            continue;

        // Get Shape
        std::list<Coord> shapeList = traci->lane(edge + "_0").getShape();
        std::vector<Coord> shapeVec(shapeList.begin(), shapeList.end());

        int segments = floor(len / rsuSpacing);

        for (int k = 1; k <= segments; k++)
        {
            double dist = k * rsuSpacing;

            // 1. Calculate Candidate Position
            Coord rawPos = getPointOnPolyline(shapeVec, dist);

            // 2. SPATIAL CHECK against TLS RSUs
            bool tooClose = false;
            for (const auto &loc : placedLocations)
            {
                if (rawPos.distance(loc) < minSpacingFromJunctionRsus)
                {
                    tooClose = true;
                    break;
                }
            }
            if (tooClose)
                continue; // Skip this one, existing infra covers it

            // 3. Create Dynamic Candidate
            RSUCandidate cand;
            cand.id = "rsu_e_" + std::to_string(candidateList.size() + 1);
            cand.position = rawPos;
            cand.isJunction = false; // DYNAMIC (Density Based)
            cand.roadId = edge;      // Linked to this road
            cand.active = false;
            candidateList.push_back(cand);
        }
    }
}

Coord CentralRSUManager::getPointOnPolyline(const std::vector<Coord> &shape, double targetDist)
{
    // 1. Safety Checks
    if (shape.empty())
        return Coord(0, 0, 0);
    if (shape.size() == 1)
        return shape[0];
    if (targetDist <= 0.0)
        return shape[0];

    double currentDistTraveled = 0.0;

    // 2. Iterate through segments (Point A -> Point B)
    for (size_t i = 0; i < shape.size() - 1; ++i)
    {
        const Coord &p1 = shape[i];
        const Coord &p2 = shape[i + 1];

        double segmentLen = p1.distance(p2);

        // 3. Check if the target point lies on THIS segment
        if (currentDistTraveled + segmentLen >= targetDist)
        {
            // The point is here. Calculate exact position.
            double distOnSegment = targetDist - currentDistTraveled;

            // Avoid division by zero if two points are identical
            if (segmentLen <= 0.0001)
                return p1;

            // Linear Interpolation ratio (0.0 to 1.0)
            double ratio = distOnSegment / segmentLen;

            // Formula: P = P1 + (P2 - P1) * ratio
            double x = p1.x + (p2.x - p1.x) * ratio;
            double y = p1.y + (p2.y - p1.y) * ratio;

            // Preserve Z if 3D, otherwise 0
            double z = p1.z + (p2.z - p1.z) * ratio;

            return Coord(x, y, z);
        }

        // 4. Move to next segment
        currentDistTraveled += segmentLen;
    }

    // 5. Fallback: If targetDist > total length, return the very last point
    return shape.back();
}
void CentralRSUManager::buildTopologyGraph()
{
    // 1. Get all Edge IDs from TraCI
    std::list<std::string> allEdgeIds = traci->getRoadIds();

    for (const std::string &sourceEdge : allEdgeIds)
    {
        // Skip internal edges (inside junctions) for a  cleaner graph
        if (sourceEdge.rfind(":", 0) == 0)
            continue;

        // 2. Find out how many lanes this edge has
        int numLanes = traci->road(sourceEdge).getLaneNumber();
        for (int i = 0; i < numLanes; ++i)
        {
            // Construct the Lane ID: "edgeID_index"
            std::string laneId = sourceEdge + "_" + std::to_string(i);
            // 3. Get links for THIS specific lane
            auto links = traci->lane(laneId).getLinks();
            for (const auto &link : links)
            {
                std::string approachedLaneId = link.approachedLane;
                std::string targetEdge = getRoadId(approachedLaneId);
                // Skip U-turns or internal junction edges
                if (targetEdge.rfind(":", 0) == 0)
                    continue;

                // 4. Store in Upstream Map (Reverse Graph)
                // "Traffic on 'targetEdge' comes from 'sourceEdge'"

                // Avoid duplicates! (Lane 0 and Lane 1 might both go to the same edge)
                // We need a helper to check if this connection already exists in the vector
                bool exists = false;
                for (auto &conn : upstreamMap[targetEdge])
                {
                    if (conn.sourceEdgeId == sourceEdge)
                    {
                        exists = true;
                        break;
                    }
                }

                if (!exists)
                {
                    UpstreamConnection conn;
                    conn.sourceEdgeId = sourceEdge;
                    conn.direction = link.direction; // 's', 'l', 'r'
                    upstreamMap[targetEdge].push_back(conn);
                }
            }
        }
    }
    std::cout << "[ Built topology graph of size ] " << upstreamMap.size() << endl;
}
RSUCandidate *CentralRSUManager::findCandidate(std::string id)
{
    for (auto &candidate : candidateList)
    {
        if (candidate.id == id)
        {
            return &candidate;
        }
    }
    // Not found
    return nullptr;
}
int CentralRSUManager::getActiveRSUCount() const
{
    int activeCount = 0;
    for (const auto &entry : candidateList)
    {
        if (entry.active)
        {
            activeCount++;
        }
    }

    return activeCount;
}

void CentralRSUManager::handleMessage(cMessage *msg)
{
    if (msg == updateEvent)
    {
        if (!enableDynamicRsu)
        {
            return;
        }
        updateRSUPlacement();
        std::cout << " AFTER UPDATE AT " << simTime().dbl() << " RSU COUNT IS " << candidateList.size() << endl;
        scheduleAt(simTime() + updateInterval, updateEvent);
    }
}

void CentralRSUManager::updateRSUPlacement()
{
    // 1. Reset counters
    // Map: Candidate Index -> Vehicle Count
    std::map<int, int> densityCounts;

    // 2. Scan Vehicles
    auto hosts = manager->getManagedHosts();
    for (auto it = hosts.begin(); it != hosts.end(); ++it)
    {
        // ... get vehicleApp ...
        cModule *appl = it->second->getSubmodule("appl");
        if (!appl)
            continue;
        VehicleApp *carApp = check_and_cast<VehicleApp *>(appl);
        Coord carPos = carApp->getCurrentPosition();

        // Find CLOSEST Candidate (Simple Distance check)
        // Optimization: In a real grid, we'd use a QuadTree.
        // For <100 RSUs, a simple loop is fine.

        int bestIdx = -1;
        int minDist = rsuCoverage; // Max capture range (slightly > spacing)

        for (size_t i = 0; i < candidateList.size(); ++i)
        {
            double d = carPos.distance(candidateList[i].position);
            if (d < minDist)
            {
                minDist = d;
                bestIdx = i;
            }
        }

        if (bestIdx != -1)
        {
            densityCounts[bestIdx]++;
        }
    }

    // 3. Evaluation Loop
    for (size_t i = 0; i < candidateList.size(); ++i)
    {
        RSUCandidate &cand = candidateList[i];

        // SKIP JUNCTIONS (They are static)
        if (cand.isJunction)
            continue;

        int count = densityCounts[i]; // Default 0

        // --- ACTIVATION ---
        if (count >= deploymentThreshold)
        {
            if (!cand.active)
            {
                std::cout << "ACTIVATING RSU " << cand.id << " AT " << simTime().dbl() << " denisty is " << count << endl;
                createRSU(cand); // Implementation below

                cand.lowDensityStartTime = 0;
            }
            else
            {
                // Already active, reset timer
                cand.lowDensityStartTime = 0;
            }
        }
        // --- DEACTIVATION ---
        else if (cand.active && count < removalThreshold)
        {
            if (cand.lowDensityStartTime == 0)
            {
                cand.lowDensityStartTime = simTime();
            }

            if (simTime() - cand.lowDensityStartTime >= hysteresisTime)
            {
                deactivateRSU(cand);
            }
        }
    }
}
int CentralRSUManager::getTotalVehicleCount() const
{
    return nodeStates.size();
}

double CentralRSUManager::getAverageNetworkSpeed() const
{
    if (nodeStates.empty())
        return 0.0;
    double totalSpeed = 0.0;
    for (const auto &pair : nodeStates)
    {
        totalSpeed += pair.second.speed;
    }
    return totalSpeed / nodeStates.size();
}

std::unordered_map<std::string, NodeState> CentralRSUManager::getAllNodeStates()
{
    updateAllNodes();
    return nodeStates;
}
std::unordered_map<std::string, RsuState> CentralRSUManager::getRSUStates()
{
    updateRSUStates();
    return rsuStates;
}
void CentralRSUManager::updateAllNodes()
{
    nodeStates.clear();
    if (manager)
    {
        const std::map<std::string, cModule *> &hosts = manager->getManagedHosts();
        for (std::map<std::string, cModule *>::const_iterator it = hosts.begin(); it != hosts.end(); ++it)
        {
            cModule *applModule = it->second->getSubmodule("appl");

            if (applModule)
            {
                VehicleApp *carApp = dynamic_cast<VehicleApp *>(applModule);
                if (carApp)
                {
                    NodeState state;
                    state.id = carApp->getVehicleId();
                    state.nodeName = carApp->getNodeName();
                    state.position = carApp->getCurrentPosition();
                    state.speed = carApp->getCurrentSpeed();
                    state.acceleration = carApp->getAcceleration();
                    state.heading = carApp->getHeading();
                    state.parkingState = carApp->getParkingState();
                    state.vehicleType = carApp->getVehicleType();
                    state.laneId = carApp->getLaneId();
                    // std::vector<std::string> signalState = carApp->getVehicleSignals();
                    nodeStates[state.id] = state;
                }
            }
        }
    }
}
void CentralRSUManager::updateRSUStates()
{

    rsuStates.clear();
    for (RSUCandidate &cand : candidateList)
    {
        if (!cand.active || !cand.module)
            continue;
        RsuState rsuState;
        rsuState.id = cand.id;
        rsuState.isTrafficLight = cand.isJunction;
        rsuState.position = cand.position;
        cModule *rsu = cand.module->getSubmodule("appl");

        RsuApp *rsuApp = check_and_cast<RsuApp *>(rsu);
        if (rsuState.isTrafficLight)
        {
            rsuState.tlsId = rsuApp->getTlsId();
            rsuState.trafficLightState = rsuApp->getCurrentTrafficState();
            rsuState.vehicleCount = rsuApp->getVehicleCount();
        }
        rsuStates[rsuState.id] = rsuState;
    }
}
int CentralRSUManager::getEmergencyVehicleCount()
{
    int count = 0;
    // Iterate over the map of all active vehicles
    for (auto &entry : nodeStates)
    {
        std::string type = entry.second.vehicleType;

        if (type == "emergency" || type == "ambulance" || type == "fire_truck")
        {
            count++;
        }
    }
    return count;
}
void CentralRSUManager::createRSU(RSUCandidate &cand)
{
    // Use generic factory
    auto start_time = std::chrono::high_resolution_clock::now();

    cModuleType *rsuType = cModuleType::find(rsuModuleType.c_str());
    if (!rsuType)
    {
        throw cRuntimeError("RSU module type not found.");
    }
    cModule *rsu = rsuType->create(cand.id.c_str(), getParentModule());

    rsu->finalizeParameters();
    rsu->buildInside();
    rsu->setDisplayString("i=veins/sign/yellowdiamond;is=vs");

    // Set Position
    cModule *rsuMobility = rsu->getSubmodule("mobility");
    rsuMobility->par("x").setDoubleValue(cand.position.x);
    rsuMobility->par("y").setDoubleValue(cand.position.y);
    rsuMobility->par("z").setDoubleValue(0);
    cModule *app = rsu->getSubmodule("appl");
    RsuApp *rsuApp = check_and_cast<RsuApp *>(app);
    rsuApp->setOperationalState(true);
    rsuApp->setJunctionState(cand.isJunction);
    if (cand.isJunction)
    {
        rsuApp->setTlsId(cand.junctionId);
    }
    rsu->callInitialize();

    // Update State
    cand.module = rsu;
    cand.active = true;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    double durationMs = duration / 1000.0; // convert to milliseconds
    // 4. Emit the "Cost" (Real CPU time taken)
    emit(sigRsuCreationLatency, durationMs);

    emit(registerSignal("nfvRsuScaledOut"), 1);
}

void CentralRSUManager::deactivateRSU(RSUCandidate &cand)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cand.module)
        return;
    cDisplayString &disp = cand.module->getDisplayString();
    disp.setTagArg("i", 0, ""); // visually hidden
    RsuApp *app = check_and_cast<RsuApp *>(cand.module->getSubmodule("appl"));
    app->setOperationalState(false);
    // cand.module->callFinish();
    // cand.module->deleteModule();

    // cand.module = nullptr;
    cand.active = false;
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    double durationMs = duration / 1000.0; // convert to milliseconds

    emit(sigRsuDeletionLatency, durationMs);
    emit(registerSignal("nfvRsuScaledIn"), 1);
    std::cout << "MANO: Terminated RSU VNF at " << cand.id << " (Load: Low)" << endl;
}

void CentralRSUManager::finish()
{
    // Clean up all dynamically created RSUs on finish
    if (updateEvent)
    {
        cancelAndDelete(updateEvent);
        updateEvent = nullptr;
    }
}
void CentralRSUManager::handleVNFQuery(MANOQuery query)
{
    MANOResponse response;
    response.contentName = query.contentName;
    response.queryId = query.queryId;
    std::string sourceRsu = query.sourceRSU;

    // 1. Identify Target Roads (The "Interest Graph")
    std::set<std::string> targetRoads;
    std::string roadId = getRoadId(query.laneId);
    targetRoads.insert(roadId);

    // 2. Add Upstream Roads (Proactive Step)
    // Look at the topology graph to find roads feeding INTO this road
    if (upstreamMap.find(roadId) != upstreamMap.end())
    {
        for (const auto &conn : upstreamMap[roadId])
        {
            targetRoads.insert(conn.sourceEdgeId);

            // Optional: Go 2 Hops Upstream (Deep Proactive Caching)
            /*
            if (upstreamMap.find(conn.sourceEdgeId) != upstreamMap.end()) {
                for (const auto& hop2 : upstreamMap[conn.sourceEdgeId]) {
                    targetRoads.insert(hop2.sourceEdgeId);
                }
            }
            */
        }
    }

    std::cout << "MANO: Accident on " << roadId << ". Identified "
              << targetRoads.size() << " target roads for caching." << endl;

    // 3. Find Active RSUs covering these roads
    // We iterate through our 'candidateList' (Hybrid Junction/In-Fill list)
    for (auto &cand : candidateList)
    {
        // Skip inactive or placeholders
        if (!cand.active || !cand.module)
            continue;

        // Skip the source (it already has the data)
        std::string rsuId = cand.module->getFullName();
        if (rsuId == sourceRsu)
            continue;

        bool isSelected = false;

        // --- CHECK 1: Explicit Road Assignment (In-Fill RSUs) ---
        // If this RSU was explicitly created for one of our target roads
        if (!cand.isJunction && targetRoads.count(cand.roadId))
        {
            isSelected = true;
        }

        // --- CHECK 2: Topographic Proximity (Junction RSUs) ---
        else if (cand.isJunction)
        {

            // Check cached topology
            if (junctionTopology.find(rsuId) != junctionTopology.end())
            {
                const auto &info = junctionTopology[rsuId];

                // Does this junction feed into ANY of our target roads?
                for (const auto &outEdge : info.outgoingEdges)
                {
                    if (targetRoads.count(outEdge))
                    {
                        isSelected = true;
                        std::cout << "Selected " << rsuId << " because it feeds into " << outEdge << endl;
                        break;
                    }
                }
            }

            // FALLBACK: Keep distance check as a safety net
            // If topology fails (e.g. weird map data), use distance.
            if (!isSelected)
            {
                Coord rsuPos = cand.position;
                Coord accPos = Coord(query.posX, query.posY, 0);
                if (rsuPos.distance(accPos) < 100.0)
                {
                    isSelected = true;
                }
            }
        }

        if (isSelected)
        {
            response.rsuIds.push_back(rsuId);
        }
    }

    // 4. Send Decision
    socketInterfaceModule->sendManoResponse(response);
}
std::string CentralRSUManager::getRoadId(std::string laneId) const
{
    size_t lastUnderscore = laneId.find_last_of('_');
    if (lastUnderscore != std::string::npos)
    {
        return laneId.substr(0, lastUnderscore);
    }
    return laneId;
}