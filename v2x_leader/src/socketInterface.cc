
#include "headers/vehicleApp.h"
#include "headers/socketInterface.h"
#include "headers/centralRSUManager.h"
using namespace veins;
using namespace omnetpp;
Define_Module(SocketInterface);
void SocketInterface::initialize()
{
    port = par("port");
    useTcp = par("useTcp");
    //  Create necessary message objects upfront
    socket_check_event = new cMessage("socketCheck");
    create_mano = new cMessage("sendInitialState");
    send_state_event = new cMessage("sendStateEvent");
    connection_check_event = new cMessage("checkForConnection");
    timeSynchronizerModule = check_and_cast<TimeSynchronizer *>(getModuleByPath("timeSynchronizer"));
    EV << "🚀 SocketInterface initializing on port " << port << std::endl;
    EV << "⏳ Simulation is PAUSED, waiting for a client to connect..." << std::endl;
    timeStep = timeSynchronizerModule->getTimeStep();

    // The socket server must run in a separate thread. This is correct.
    if (useTcp)
        server_thread = std::thread(&SocketInterface::startTcpServer, this);
    else
        server_thread = std::thread(&SocketInterface::startUnixServer, this);
    // sent bytes signals
    sigSockMobilityBytes = registerSignal("sockMobilityBytes");
    sigSockRSUBytes = registerSignal("sockRSUBytes");
    sigEmergencyReqBytes = registerSignal("sockEmergencyReqBytes");
    sigSockAccidentBytes = registerSignal("sockAccidentBytes");
    sigSockEndOfStep = registerSignal("sockEndOfStep");
    sigSockTerminate = registerSignal("sockTerminate");
    sigSockEmptyPayload = registerSignal("sockEmptyPayload");

    // recv bytes
    sigSockManoQueryBytes = registerSignal("sockManoQueryBytes");
    sigSafetyCmdBytes = registerSignal("safetyCmdBytes");
    sigTrafficPreemptionBytes = registerSignal("trafficPreemptionBytes");
    sigTimeSyncBytes = registerSignal("timeSyncBytes");

    scheduleAt(simTime(), connection_check_event);
}
void SocketInterface::startTcpServer()
{
    struct sockaddr_in server_addr;
    int opt = 1;
    int addrlen = sizeof(server_addr);
    // 1. Create the server socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        EV_ERROR << "FATAL: Socket creation failed: " << strerror(errno)
                 << std::endl;
        return;
    }
    // 2. Set socket options to allow reusing the address
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))
    {
        EV_ERROR << "FATAL: setsock opt failed: " << strerror(errno)
                 << std::endl;
        close(server_fd); // Ensure server_fd is closed on error
        return;
    }
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on any network interface
    server_addr.sin_port = htons(port);

    // 3. Bind the socket to the specified port
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        EV_ERROR << "FATAL: Bind failed on port " << port << ": "
                 << strerror(errno) << std::endl;
        close(server_fd); // Ensure server_fd is closed on error
        return;
    }

    // 4. Listen for incoming connections (allow a small backlog)
    if (listen(server_fd, 5) < 0)
    { // Increased backlog queue for more robustness
        EV_ERROR << "FATAL: Listen failed: " << strerror(errno) << std::endl;
        close(server_fd); // Ensure server_fd is closed on error
        return;
    }

    EV << "👂 Server listening... Waiting for clients to connect." << std::endl;
    // Loop indefinitely to accept multiple client connections sequentially.
    // This thread will block on `accept()` until a client connects.
    // When a client disconnects, `handleClientData` will close `client_socket`
    // and set `client_connected = false`, allowing this loop to accept a new one.
    while (!server_thread_should_stop)

    {
        // Only attempt to accept a new connection if no client is currently connected.
        // This prevents overwriting an active client_socket if the main thread is still processing it.
        if (!client_connected)
        {
            EV << "DEBUG: Server thread is ready to accept a new connection..."
               << std::endl;
            // `accept` is a blocking call. This thread will pause here until a client attempts to connect.
            if ((client_socket = accept(server_fd,
                                        (struct sockaddr *)&server_addr, (socklen_t *)&addrlen)) < 0)
            {
                // If accept fails, log the error and try again after a short delay.
                // This can happen if the server_fd is closed unexpectedly or other transient network issues.
                if (server_thread_should_stop)
                {
                    EV << "DEBUG: accept() failed because server is shutting down. Exiting thread." << endl;
                    break;
                }
                EV_ERROR << "FATAL: Accept failed: " << strerror(errno)
                         << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep to prevent busy-looping on error
                continue;                                                    // Go back to the start of the while loop to try accepting again
            }

            // A client has successfully connected.
            EV << "✅ Client connected! Client socket FD: " << client_socket
               << std::endl;
            client_connected = true;
        }
        else
        {
            // If a client is already connected, this thread will just wait a bit
            // before checking `client_connected` again. This is to avoid a busy-wait
            // while the main OMNeT++ thread is handling the current client.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
void SocketInterface::startUnixServer()
{
    struct sockaddr_un server_addr;
    int addrlen = sizeof(server_addr);

    // Suggestion: Read this from omnetpp.ini: par("unixSocketPath").stringValue()
    const char *socket_path = "/tmp/v2x_sim_socket";

    // 1. Create the Unix socket
    if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)
    {
        EV_ERROR << "FATAL: Unix Socket creation failed: " << strerror(errno) << std::endl;
        return;
    }

    // 2. Prepare the address (Unix socket address)
    memset(&server_addr, 0, sizeof(server_addr)); // Clear the structure
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, socket_path, sizeof(server_addr.sun_path) - 1);

    // 3. Remove the existing socket file (if left over from a previous crash)
    unlink(socket_path);

    // 4. Bind the socket to the specified file path
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        EV_ERROR << "FATAL: Bind failed on socket path " << socket_path << ": "
                 << strerror(errno) << std::endl;
        close(server_fd);
        return;
    }

    // 5. Listen for incoming connections
    if (listen(server_fd, 5) < 0)
    {
        EV_ERROR << "FATAL: Listen failed: " << strerror(errno) << std::endl;
        close(server_fd);
        return;
    }

    EV << "👂 Unix Server listening on " << socket_path << "... Waiting for clients." << std::endl;

    // 6. Loop indefinitely to accept client connections
    while (!server_thread_should_stop)
    {
        if (!client_connected)
        {
            if ((client_socket = accept(server_fd, (struct sockaddr *)&server_addr, (socklen_t *)&addrlen)) < 0)
            {
                if (server_thread_should_stop)
                    break;

                EV_ERROR << "FATAL: Accept failed: " << strerror(errno) << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            EV << "✅ Client connected via Unix Socket! FD: " << client_socket << std::endl;
            client_connected = true;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
void SocketInterface::handleMessage(cMessage *msg)
{

    if (simulation_ended)
    {
        delete msg;
        return;
    }

    // --- PHASE 0: INITIALIZATION AND HANDSHAKE ---
    if (msg == connection_check_event)
    {
        if (client_connected)
        {
            EV << "✅ Client has connected! Starting Veins Manager." << std::endl;
            std::cout << "✅ Client has connected! Starting Veins Manager." << std::endl;

            simulation_started = true;
            cancelAndDelete(connection_check_event);
            startVeinsManager();
            // traci is null upto this point so have to create mano after scheduling a message in startveinsmanager
        }
        else
        {
            // Reschedule to wait for connection, freezing time at 0.
            scheduleAt(simTime(), connection_check_event);
        }
        return;
    }
    if (msg == create_mano)
    {
        createCentralModules();
        std::cout << "Central modules created and initialized NOW sending initial State at" << std::chrono::high_resolution_clock::now().time_since_epoch().count() << endl;
        scheduleAt(simTime(), send_state_event);
        return;
    }

    // --- PHASE 1 (Initial & Recurring): SEND DATA AND START THE BUSY-WAIT LOOP ---
    if (msg == send_state_event)
    {
        if (simTime() == 0)
        {
            wallClockStart = std::chrono::high_resolution_clock::now();
            std::cout << "SIMULATION STARTED at wall clock time: " << wallClockStart.time_since_epoch().count() << endl;
        }
        // Check for simulation termination condition first
        if (simTime() >= SimTime::getMaxTime())
        {
            EV << "[OMNeT++] Simulation time limit reached. Sending termination message." << endl;
            std::cout << "[OMNeT++] Simulation time limit reached. Sending termination message." << endl;
            sendTerminate();
            simulation_ended = true;
            delete msg;
            endSimulation();
            return;
        }

        processTrafficPreemptionQueue();
        processSafetyMessages();
        processManoQueue();
        // 3. Micro-Stepping Check
        double sumoInterval = 0.1;
        double t = simTime().dbl();
        double remainder = fmod(t, sumoInterval);
        bool isMajorStep = (remainder < 1e-7 || (sumoInterval - remainder) < 1e-7);
        if (isMajorStep)
        {
            // HEAVY PAYLOADS: Only send every 0.1s
            sendNodeStates();
            sendRSUStates();
        }
        sendEndOfStep();

        EV << "[OMNeT++ -> ns-3] State sent for t=" << simTime() << "s. Pausing and waiting for ns-3 feedback." << std::endl;

        // After sending, we immediately start the high-frequency "busy-wait" poll.
        // This will block all other modules (like TraCI) from advancing.
        if (!socket_check_event->isScheduled())
        {
            scheduleAt(simTime(), socket_check_event);
        }
        return;
    }

    // --- PHASE 3 & 4: THE BUSY-WAIT LOOP LOGIC ---
    if (msg == socket_check_event)
    {
        // 1. Always check for data
        if (isDataAvailableOnSocket())
        {
            EV << "[Socket] Data received." << endl;
            handleClientData(); // May or may not unlock the clock
        }

        // We keep polling if the synchronizer is still waiting, regardless of
        // whether we received data or not.
        if (timeSynchronizerModule->isWaiting() && !simulation_ended)
        {
            // Prevent stacking events if handleClientData already scheduled one (rare edge case)
            if (!socket_check_event->isScheduled())
            {

                scheduleAt(simTime(), socket_check_event);
            }
        }
        return;
    }

    delete msg;
}
void SocketInterface::processManoQueue()
{
    if (!manoQueue.empty())
    {
        std::cout << "Processing the Mano Queue" << endl;
        for (const auto &q : manoQueue)
        {
            mano->handleVNFQuery(q);
        }
        manoQueue.clear();
    }
}
void SocketInterface::processTrafficPreemptionQueue()
{
    if (!trafficPreemptionQueue.empty())
    {
        std::cout << "[Sync] Processing " << trafficPreemptionQueue.size() << " queued preemption commands." << endl;
        for (const auto &cmd : trafficPreemptionQueue)
        {
            cModule *targetMod = getParentModule()->getSubmodule(cmd.targetId.c_str());
            if (targetMod)
            {
                cModule *appMod = targetMod->getSubmodule("appl");
                if (appMod)
                {
                    RsuApp *rsuApp = check_and_cast<RsuApp *>(appMod);
                    rsuApp->handlePreemptionCommand(cmd);
                    std::cout << "[Traffic light] preemption done for " << cmd.tlsId << endl;
                }
                else
                {
                    std::cout << "RSU app not found" << endl;
                }
            }
            else
            {
                std::cout << ("V2X." + cmd.targetId).c_str() << " not found " << endl;
            }
        }
        trafficPreemptionQueue.clear();
    }
}
void SocketInterface::processSafetyMessages()
{
    if (!commandQueue.empty())
    {
        std::cout << "[Sync] Processing " << commandQueue.size() << " queued safety commands." << endl;

        for (const auto &cmd : commandQueue)
        {
            cModule *targetMod = getModuleByPath(("V2X." + cmd.targetId).c_str());
            if (targetMod)
            {
                cModule *appMod = targetMod->getSubmodule("appl");
                if (appMod)
                {
                    VehicleApp *app = check_and_cast<VehicleApp *>(appMod);
                    app->handleNetworkAccident(cmd.accidentData);
                }
            }
        }
        // Clear the queue for the next step
        commandQueue.clear();
    }
}
void SocketInterface::handleClientData()
{
    // 1. Receive into temporary char array
    char buffer[4096] = {0};
    ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);

    if (bytes_received <= 0)
    {
        EV_WARN << "Client disconnected or recv error. Pausing simulation." << endl;
        client_connected = false;
        close(client_socket);
        client_socket = -1;
        return;
    }

    // 2. Append new data to the Persistent Buffer
    receiveBuffer.append(buffer);

    // 3. Process complete lines from the buffer
    bool trigger_sync = false;

    // While there is a newline in the buffer, we have at least one full message
    std::string::size_type pos;
    while ((pos = receiveBuffer.find('\n')) != std::string::npos)
    {
        // Extract the line (including newline)
        std::string line = receiveBuffer.substr(0, pos);

        // Remove this line from the buffer so we don't process it again
        receiveBuffer.erase(0, pos + 1);

        // Skip empty lines
        if (line.empty())
            continue;

        // --- PARSE THIS SPECIFIC LINE ---
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        std::stringstream ss(line);

        if (!Json::parseFromStream(builder, ss, &root, &errs))
        {
            // Fallback for legacy raw string check
            if (line.find("TIME_SYNC") != std::string::npos)
            {
                EV << "[Legacy] Raw TIME_SYNC received." << endl;
                emit(sigTimeSyncBytes, (long)bytes_received);

                trigger_sync = true;
            }
            else
            {
                EV_ERROR << "Failed to parse JSON line: " << errs << " | Raw: " << line << endl;
            }
            continue; // Move to next line in buffer
        }
        std::cout << "Recevied data from client" << root << endl;

        // --- MESSAGE DISPATCHING ---
        std::string msgType = root["message_type"].asString();

        if (msgType == "time_sync")
        {
            emit(sigTimeSyncBytes, (long)bytes_received);
            trigger_sync = true;
        }
        else if (msgType == "safety_command")
        {
            emit(sigSafetyCmdBytes, (long)bytes_received);
            std::cout << "[Command] Received Safety Command from ns-3" << endl;
            Json::Value payload = root["payload"];
            Json::Value accData = payload["accident_data"];

            PendingSafetyCmd cmd;
            AccidentData accidentData;

            cmd.targetId = payload["target_vehicle_id"].asString();
            accidentData.sourceType = payload["source_type"].asString();

            // Parse common accident data fields
            accidentData.accidentId = accData["accident_id"].asString();
            accidentData.originTime = accData["origin_time"].asDouble();
            accidentData.posX = accData["pos_x"].asDouble();
            accidentData.posY = accData["pos_y"].asDouble();
            accidentData.crashedVehId = accData["crashed_vehicle_id"].asString();

            // --- DISTINGUISH BETWEEN ACTIVE ACTUATION AND PASSIVE WARNING ---
            // Collision avoidance IDs start with "collision_" in ns-3
            bool isCollisionAvoidance = (accidentData.accidentId.rfind("collision_", 0) == 0 || payload.isMember("action"));

            if (isCollisionAvoidance)
            {
                // 1. Parse Active Actuation Command
                accidentData.laneId = ""; // No lane ID required for direct MEC commands

                if (payload.isMember("action"))
                    accidentData.action = payload["action"].asString();
                if (payload.isMember("risk_level"))
                    accidentData.riskLevel = payload["risk_level"].asString();

                // Parse parsed metrics from ns-3 Digital Twin
                if (accData.isMember("ttc"))
                    accidentData.ttc = accData["ttc"].asDouble();
                if (accData.isMember("distance"))
                    accidentData.distance = accData["distance"].asDouble();
            }
            else
            {
                // 2. Parse Standard Passive Accident Report
                accidentData.laneId = accData["lane_id"].asString();
                accidentData.action = ""; // Mark as passive

                if (accidentData.laneId.empty())
                {
                    std::cout << "Warning: laneId is empty in received accident report. Discarding." << endl;
                    return;
                }
            }

            cmd.accidentData = accidentData;
            commandQueue.push_back(cmd);
        }

        else if (msgType == "vnf_query")
        {
            emit(sigSockManoQueryBytes, (long)bytes_received);
            std::cout << "[MANO] Received Placement Query. Queuing..." << endl;

            // 1. Create Struct
            MANOQuery q;
            Json::Value payload = root["payload"];

            q.queryId = payload["query_id"].asString();
            q.contentName = payload["content_name"].asString();
            q.laneId = payload["lane_id"].asString(); // Or road_id if you send that
            q.sourceRSU = payload["origin_rsu"].asString();
            q.posX = payload["pos_x"].asDouble();
            q.posY = payload["pos_y"].asDouble();

            // 2. Push to Queue (Do not process yet)
            manoQueue.push_back(q);
        }
        else if (msgType == "traffic_preemption_command")
        {
            emit(sigTrafficPreemptionBytes, (long)bytes_received);
            TrafficPreemptionCommand tpc;
            Json::Value payload = root["payload"];

            tpc.requestorId = payload["requestor_id"].asString();
            tpc.laneId = payload["lane_id"].asString();
            tpc.tlsId = payload["tls_id"].asString();
            tpc.targetId = payload["target_rsu_id"].asString();
            tpc.originTime = payload["origin_time"].asDouble();
            trafficPreemptionQueue.push_back(tpc);
        }
        std::cout << " Queue size is " << commandQueue.size() << "\t " << "MANO Queue size is " << manoQueue.size() << endl;
    } // End While Loop

    // --- SYNCHRONIZATION LOGIC ---
    if (trigger_sync)
    {
        if (timeSynchronizerModule->isWaiting())
        {
            std::cout << "[Sync] Advancing simulation time." << std::endl;
            timeSynchronizerModule->advanceStep();
            scheduleAt(simTime() + timeSynchronizerModule->getTimeStep(), send_state_event);
        }
    }
}
void SocketInterface::startVeinsManager()
{

    cModule *network = getParentModule();

    // Find the module type for the Veins manager
    cModuleType *managerType = cModuleType::find("org.car2x.veins.modules.mobility.traci.TraCIScenarioManagerForker");
    if (!managerType)
    {
        throw cRuntimeError("TraCIScenarioManagerForker module type not found. Are Veins libraries linked?");
    }

    // 1. CREATE the module and name it "manager"
    auto *traciManager = check_and_cast<cSimpleModule *>(managerType->create("manager", network));
    this->traciManagerModule = check_and_cast<TraCIScenarioManagerForker *>(traciManager);
    traciManagerModule->setDisplayString("p=512,128;i=block/network2");
    // 2. BUILD its internals (read parameters from omnetpp.ini)
    cModuleType *visualizerType = cModuleType::find("org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer");
    if (!visualizerType)
    {
        // This is a good sanity check, but it should be found if Veins is linked
        throw cRuntimeError("RoadsCanvasVisualizer module type not found.");
    }
    cModule *visualizer = visualizerType->create("roadsCanvasVisualizer", network);
    visualizer->setDisplayString("p=500,0");

    traciManagerModule->buildInside();
    visualizer->buildInside();
    // Initializing
    traciManagerModule->callInitialize();
    visualizer->callInitialize();

    std::cout << "TraCIScenarioManager and Road Canvas started successfully." << endl;
    scheduleAt(simTime(), create_mano);
}
void SocketInterface::sendCommandToClient(const std::string &command)
{

    if (!client_connected || client_socket == -1)
    {
        std::cout << "❌ Client not connected, cannot send command" << std::endl;
        return;
    }

    EV << "📤 Sending to client: " << command << endl;
    std::cout << "📤 Sending to client: " << command << std::endl;

    ssize_t sent = ::send(client_socket, command.c_str(), command.length(), MSG_NOSIGNAL);
    if (sent < 0)
    {
        std::cout << "❌ Send failed: " << strerror(errno) << std::endl;
        EV << "❌ Send failed: " << strerror(errno) << endl;

        // If send fails, client might have disconnected
        if (errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN)
        {
            std::cout << "ℹ️ Client appears to have disconnected during send" << std::endl;
            close(client_socket);
            client_socket = -1;
            client_connected = false;
        }
    }
    else
    {
        std::cout << "✅ Sent " << sent << " bytes successfully" << std::endl;
    }
}

void SocketInterface::finish()
{
    EV << "🏁 Simulation finished. Signaling server thread to stop." << std::endl;
    std::cout << "🏁 Simulation finished. Signaling server thread to stop." << std::endl;
    unlink("/tmp/v2x_sim_socket");

    if (client_connected)

    {
        sendTerminate();
    }
    if (metricsCollector)
    {
        metricsCollector->callFinish();
    }

    // cancelAndDelete(create_mano);
    // cancelAndDelete(socket_check_event);
    // cancelAndDelete(send_state_event);
    auto wallClockEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = wallClockEnd - wallClockStart;

    EV << "========================================\n";
    EV << "TOTAL WALL CLOCK EXECUTION TIME: " << diff.count() << " seconds\n";
    EV << "========================================" << endl;

    TraCIScenarioManagerForker *forker = FindModule<TraCIScenarioManagerForker *>::findSubModule(getParentModule());

    if (forker)
    {
        forker->callFinish();
        if (forker != nullptr)
        {
            // Delteing the forker module so that all scheduled messages not deleted in finish get deleted
            forker->deleteModule();
        }
    }
    if (traciManagerModule)
    {
        // std::cout << "calling finish of manager" << endl;
        traciManagerModule->callFinish();
    }

    server_thread_should_stop = true;

    // Add a small delay or a more robust mechanism to ensure the thread sees the stop signal
    // Forcing a connection to unblock accept() is a good pattern.
    // However, for simplicity here we just proceed.
    // If the thread is stuck in accept(), it won't join until a connection is made.
    // Closing the server_fd from this thread can also help unblock it.
    if (server_fd >= 0)
    {
        std::cout << "Shutting down socket" << endl;

        shutdown(server_fd, SHUT_RDWR); // Gracefully shutdown socket
        close(server_fd);
        server_fd = -1;
    }

    if (server_thread.joinable())
    {
        EV << "Waiting for server thread to join..." << endl;
        server_thread.join();
        EV << "Server thread joined successfully." << std::endl;
    }

    if (client_socket >= 0)
    {
        std::cout << "Closing socket" << endl;
        close(client_socket);
        client_socket = -1;
    }
}
void SocketInterface::createCentralModules()
{
    cModule *network = getParentModule();
    cModuleType *centralRSUManagerType = cModuleType::find("ned.CentralRSUManager");
    if (!centralRSUManagerType)
    {
        throw cRuntimeError("Central RSU Manager module type not found.");
    }
    cModule *centralRSUManager = centralRSUManagerType->create("rsuManager", network);
    this->mano = check_and_cast<CentralRSUManager *>(centralRSUManager);
    mano->buildInside();
    mano->callInitialize();
    mano->setDisplayString("p=800,400");

    cModuleType *metricsCollectorType = cModuleType::find("ned.MetricsCollector");
    if (!metricsCollectorType)
    {
        throw cRuntimeError("Metrics Collector module type not found.");
    }
    cModule *metricsCollectorModule = metricsCollectorType->create("metricsCollector", network);
    this->metricsCollector = check_and_cast<MetricsCollector *>(metricsCollectorModule);
    metricsCollector->buildInside();
    metricsCollector->callInitialize();
    metricsCollector->setDisplayString("p=900,400");
}

void SocketInterface::sendManoResponse(MANOResponse response)
{
    Json::Value root;
    root["message_type"] = "mano_decision";
    root["timestamp"] = simTime().dbl();

    Json::Value payload;
    payload["query_id"] = response.queryId;
    payload["content_name"] = response.contentName;

    // --- OPTIMIZED TARGET LIST ---
    // Create a simple JSON Array of Strings ["rsu1", "rsu2"]
    Json::Value targets(Json::arrayValue);

    for (const auto &rsuId : response.rsuIds)
    {
        targets.append(rsuId); // Directly append string, no object wrapper
    }

    payload["targets"] = targets;
    root["payload"] = payload;

    // Serialize and Send
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string jsonString = Json::writeString(builder, root);

    sendCommandToClient(jsonString + "\n");
}
void SocketInterface::sendTrafficPreemption(TrafficPreemptionCommand command)
{

    std::stringstream oss;

    // Construct the JSON structure
    oss << "{"
        << "\"message_type\":\"traffic_preemption_request\","
        << "\"timestamp\":" << simTime().dbl() << ","
        << "\"payload\":{"
        << "\"tls_id\":\"" << command.tlsId << "\","
        << "\"sender_id\":\"" << command.requestorId << "\","
        << "\"lane_id\":\"" << command.laneId << "\","
        << "\"origin_time\":" << command.originTime
        << "},"

        << "\"meta\":{"
        << "\"priority\":\"URLLC\"," // Tagging it as Ultra-Reliable Low Latency
        << "\"description\":\"Emergency vehicle requesting green light\""
        << "}"
        << "}\n"; // Terminating newline for TCP framing

    // Send to ns-3
    std::string msg = oss.str();
    sendCommandToClient(msg);
    emit(sigEmergencyReqBytes, (long)msg.length());

    std::cout << "[Socket] Sent Traffic Preemption Request for TLS: "
              << command.tlsId << " from " << command.requestorId << endl;
}
void SocketInterface::sendNodeStates()
{

    simtime_t T_current = simTime();
    simtime_t T_next = T_current + timeStep;

    std::unordered_map<std::string, NodeState> nodeStates = mano->getAllNodeStates();
    std::ostringstream oss;
    if (nodeStates.empty())
    {
        sendEmptyPayload();
        return;
    }
    oss << "{"
        << "\"message_type\":\"mobility_update\","
        << "\"timestamp\":" << T_current.dbl() << ","
        << "\"t_next\":" << T_next.dbl() << ","
        << "\"payload\":[";
    for (auto it = nodeStates.begin(); it != nodeStates.end(); ++it)
    {
        const auto &s = it->second;
        oss << "{"
            << "\"id\":\"" << s.nodeName << "\","
            << "\"x\":" << s.position.x << ","
            << "\"y\":" << s.position.y << ","
            << "\"speed\":" << s.speed << ","
            << "\"acceleration\":" << s.acceleration << ","
            << "\"heading\":" << s.heading << ","
            << "\"parkingState\":" << s.parkingState << ","
            << "\"vehicle_type\":\"" << s.vehicleType << "\""
            << "}";
        auto next = it;
        ++next;
        if (next != nodeStates.end())
            oss << ",";
    }
    oss << "],"
        << "\"meta\":{\"count\":" << nodeStates.size() << "}"
        << "}\n";
    std::string msg = oss.str();
    emit(sigSockMobilityBytes, (long)msg.length());

    sendCommandToClient(msg);
}

void SocketInterface::sendEndOfStep()
{
    std::ostringstream oss;
    oss << "{"
        << "\"message_type\":\"end_of_step\","
        << "\"timestamp\":" << simTime().dbl() << ","
        << "\"payload\":{},"
        << "\"meta\":{"
        << "\"status\":\"sync\","
        << "\"description\":\"end of current time step updates\""
        << "}"
        << "}\n";
    std::string msg = oss.str();
    emit(sigSockEndOfStep, (long)msg.length());
    sendCommandToClient(msg);
}
void SocketInterface::sendTerminate()
{
    std::ostringstream oss;
    oss << "{"
        << "\"message_type\":\"termination_signal\","
        << "\"timestamp\":" << simTime().dbl() << ","
        << "\"payload\":{"
        << "\"reason\":\"simulation_completed\""
        << "},"
        << "\"meta\":{"
        << "\"status\":\"terminated\","
        << "\"description\":\"no further updates, simulation has ended\""
        << "}"
        << "}\n";
    std::string msg = oss.str();
    emit(sigSockTerminate, (long)msg.length());
    sendCommandToClient(msg);
}
void SocketInterface::sendRSUStates()
{

    std::unordered_map<std::string, RsuState> rsuStates = mano->getRSUStates();
    if (rsuStates.empty())
    {
        return;
    }
    if (rsuStates == lastSentRsuStates)
    {
        return;
    }

    lastSentRsuStates = rsuStates;
    std::ostringstream oss;

    oss << "{"
        << "\"message_type\":\"rsu_state\","
        << "\"timestamp\":" << simTime().dbl() << ","
        << "\"t_next\":" << (simTime().dbl() + timeStep) << ","
        << "\"payload\":[";
    for (auto it = rsuStates.begin(); it != rsuStates.end(); ++it)
    {
        const RsuState &rsu = it->second;

        oss << "{"
            << "\"id\":\"" << rsu.id << "\","
            << "\"x\":" << rsu.position.x << ","
            << "\"y\":" << rsu.position.y << ","
            << "\"isTrafficLight\":" << (rsu.isTrafficLight ? 1 : 0);

        if (rsu.isTrafficLight)
        {
            oss << ",\"trafficLightState\":\"" << rsu.trafficLightState << "\""
                << ",\"tlsId\":\"" << rsu.tlsId << "\","
                << "\"vehicleCount\":" << rsu.vehicleCount;
        }

        oss << "}";

        auto next = it;
        ++next;
        if (next != rsuStates.end())
            oss << ",";
    }
    oss << "],"

        << "\"meta\":{\"count\":" << rsuStates.size() << "}"
        << "}\n";
    std::string msg = oss.str();
    emit(sigSockRSUBytes, (long)msg.length());
    sendCommandToClient(oss.str());
}
void SocketInterface::sendEmptyPayload()
{
    std::ostringstream oss;
    oss << "{"
        << "\"message_type\":\"mobility_update\","
        << "\"timestamp\":" << simTime().dbl() << ","
        << "\"t_next\":" << (simTime().dbl() + timeStep) << ","
        << "\"payload\":[],"
        << "\"meta\":{"
        << "\"count\":0,"
        << "\"status\":\"empty\","
        << "\"description\":\"no active nodes or mobility data available\""
        << "}"
        << "}\n";
    std::string msg = oss.str();
    emit(sigSockEmptyPayload, (long)msg.length());
    sendCommandToClient(msg);
}
void SocketInterface::sendAccidentData(AccidentData accidentData)
{
    simtime_t T_current = simTime();
    simtime_t T_next = T_current + timeStep;
    std::string vehicleId = accidentData.crashedVehId;
    std::ostringstream oss;

    oss << "{"
        << "\"message_type\":\"accident_report\","
        << "\"timestamp\":" << T_current.dbl() << ","
        << "\"payload\":{"
        << "\"origin_time\":" << accidentData.originTime << ","
        << "\"accident_id\":\"" << accidentData.accidentId << "\","
        << "\"vehicleId\":\"" << accidentData.crashedVehId << "\","
        << "\"lane_id\":\"" << accidentData.laneId << "\","
        << "\"pos_x\":" << accidentData.posX << ","
        << "\"pos_y\":" << accidentData.posY << "},"
        << "\"meta\":{"
        << "\"status\":\"alert\","
        << "\"description\":\"accident event detected and reported\""
        << "}"
        << "}\n";
    auto latency = T_current.dbl() - accidentData.originTime;
    std::string msg = oss.str();
    emit(sigSockAccidentBytes, (long)msg.length());
    sendCommandToClient(oss.str());
}
bool SocketInterface::isDataAvailableOnSocket()
{
    if (!client_connected)
        return false;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(client_socket, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0; // Zero timeout for a non-blocking poll

    int result = select(client_socket + 1, &read_fds, NULL, NULL, &timeout);

    if (result < 0)
    {
        EV_ERROR << "FATAL: select() error on socket: " << strerror(errno) << endl;
        endSimulation();
        return false;
    }

    return result > 0;
}
