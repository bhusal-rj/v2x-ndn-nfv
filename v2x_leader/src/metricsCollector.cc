
#include "headers/metricsCollector.h"
#include "headers/centralRSUManager.h"
Define_Module(MetricsCollector);

void MetricsCollector::initialize()
{
    // 1. Read parameters
    interval = par("collectionInterval");
    std::string infraFile = par("nfv_infra").stdstringValue();
    std::string serviceFile = par("nfv_service").stdstringValue();
    std::string trafficFile = par("traffic").stdstringValue();
    std::string socketSentFile = par("socket_sent").stdstringValue();
    std::string socketRecvFile = par("socket_recv").stdstringValue();

    // 1. Open Files
    fileInfrastructure.open(infraFile, std::ios::trunc);
    fileServiceFlows.open(serviceFile, std::ios::trunc);
    fileTraffic.open(trafficFile, std::ios::trunc);
    fileSentSocket.open(socketSentFile, std::ios::trunc);
    fileRecvSocket.open(socketRecvFile, std::ios::trunc);

    // 2. Write Headers
    fileInfrastructure << "SimTime,Active_RSUs,Scale_Out,Scale_In,Avg_Create_Cost_ms,Avg_Delete_Cost_ms,Ctrl_Overhead_Est_KB\n";
    fileServiceFlows << "SimTime,Emergency_Vehicles,Emerg_At_Red_Light,Veh_Emerg_Sent,RSU_Emerg_Rx,Core_Emerg_Proc,Avg_TLS_Lat_ms,Avg_VehActionLatency_ms,Veh_Acc_Sent\n";
    fileTraffic << "SimTime,Total_Vehicles,Avg_Net_Speed_mps,Veh_At_Red_light,Emerg_At_Red_Light,Veh_Receiving_V2V,Veh_Receiving_RSU,Veh_Receiving_Strat,Veh_ReactingV2V,Veh_Reacting_RSUStrat,Veh_ReactingRSU\n";
    fileSentSocket << "SimTime,Total_Bytes_Sent,Mobility_Bytes,RSU_Bytes,EmergencyReq_Bytes,Accident_Bytes,End_Of_Step_Bytes,Empty_Payload_Bytes,Terminate_Bytes\n";
    fileRecvSocket << "SimTime,Total_Bytes_Received,MANOQuery_Bytes,SafetyCmd_Bytes,TrafficPreemption_Bytes,TimeSync_Bytes\n";
    // 3. Register Signals
    sigRsuScaledOut = registerSignal("nfvRsuScaledOut");
    sigRsuScaledIn = registerSignal("nfvRsuScaledIn");
    sigRsuCreationTime = registerSignal("nfvRsuCreationTimeMs");
    sigRsuDeletionTime = registerSignal("nfvRsuDeletionTimeMs");

    sigVehEmergSent = registerSignal("nfvVehEmergSent");
    sigRsuEmergRx = registerSignal("nfvRsuEmergRx");
    sigCoreEmergProcessed = registerSignal("nfvCoreEmergProc");
    sigTlsLatency = registerSignal("nfvTlsSwitchLatency");
    sigVehActionLatency = registerSignal("nfvVehicleActLatency");

    sigVehAccSent = registerSignal("nfvVehAccSent");
    sigVehAccRxV2V = registerSignal("nfvVehicleAccRxV2V");
    sigVehAccRxRSU = registerSignal("nfvVehicleAccRxRSU");
    sigVehAccRxStrat = registerSignal("nfvVehicleAccRxStrat");

    sigVehAccReactV2V = registerSignal("nfvVehAccidentReactV2V");
    sigVehAccReactRSU = registerSignal("nfvVehAccidentReactRSU");
    sigVehAccReactRSUStrat = registerSignal("nfvVehAccidentReactRSUStrat");

    sigAtRedLight = registerSignal("nfvAtRedLight");
    sigEmergRedLight = registerSignal("nfvEmergRedLight");

    // sent socketSignals
    sigSockMobility = registerSignal("sockMobilityBytes");
    sigSockRSU = registerSignal("sockRSUBytes");
    sigSockAccident = registerSignal("sockAccidentBytes");
    sigEmergencyReqBytes = registerSignal("sockEmergencyReqBytes");

    sigSockEndOfStep = registerSignal("sockEndOfStep");
    sigSockTerminate = registerSignal("sockTerminate");
    sigSockEmptyPayload = registerSignal("sockEmptyPayload");

    // recv socketsignals
    sigSockManoQueryBytes = registerSignal("sockManoQueryBytes");
    sigSafetyCmdBytes = registerSignal("safetyCmdBytes");
    sigTrafficPreemptionBytes = registerSignal("trafficPreemptionBytes");
    sigTimeSyncBytes = registerSignal("timeSyncBytes");

    // 4. Subscribe (System-wide)
    cModule *root = getSimulation()->getSystemModule();
    root->subscribe(sigRsuScaledOut, this);
    root->subscribe(sigRsuScaledIn, this);
    root->subscribe(sigRsuCreationTime, this);
    root->subscribe(sigRsuDeletionTime, this);
    root->subscribe(sigVehEmergSent, this);
    root->subscribe(sigRsuEmergRx, this);
    root->subscribe(sigCoreEmergProcessed, this);
    root->subscribe(sigTlsLatency, this);
    root->subscribe(sigVehAccSent, this);
    root->subscribe(sigVehAccReactRSU, this);
    root->subscribe(sigVehAccReactV2V, this);
    root->subscribe(sigVehAccReactRSUStrat, this);
    root->subscribe(sigVehAccRxV2V, this);
    root->subscribe(sigVehAccRxRSU, this);
    root->subscribe(sigVehAccRxStrat, this);

    root->subscribe(sigVehActionLatency, this);
    root->subscribe(sigAtRedLight, this);
    root->subscribe(sigEmergRedLight, this);
    // subscribe sent bytes socket signals
    root->subscribe(sigSockMobility, this);
    root->subscribe(sigSockRSU, this);
    root->subscribe(sigEmergencyReqBytes, this);
    root->subscribe(sigSockAccident, this);
    root->subscribe(sigSockEndOfStep, this);
    root->subscribe(sigSockTerminate, this);
    root->subscribe(sigSockEmptyPayload, this);
    // subscribe received bytes socket signals
    root->subscribe(sigSockManoQueryBytes, this);
    root->subscribe(sigSafetyCmdBytes, this);
    root->subscribe(sigTrafficPreemptionBytes, this);
    root->subscribe(sigTimeSyncBytes, this);
    // 5. Find Modules for Polling
    centralRSUManager = getModuleByPath("V2X.rsuManager");

    // 6. Start Timer
    collectionTimer = new cMessage("collect");
    scheduleAt(simTime() + interval, collectionTimer);
}

void MetricsCollector::handleMessage(cMessage *msg)
{
    if (msg == collectionTimer)
    {
        writeMetricRows();
        scheduleAt(simTime() + interval, collectionTimer);
    }
}

void MetricsCollector::receiveSignal(cComponent *source, simsignal_t id, long value, cObject *details)
{
    // Infrastructure
    if (id == sigRsuScaledOut)
        cntScaleOut++;
    else if (id == sigRsuScaledIn)
        cntScaleIn++;

    // Service Flows
    else if (id == sigVehEmergSent)
        cntVehEmergSent++;
    else if (id == sigRsuEmergRx)
        cntRsuEmergRx++;
    else if (id == sigCoreEmergProcessed)
        cntCoreEmergProcessed++;

    else if (id == sigVehAccSent)
        cntVehAccSent++;
    // Context
    else if (id == sigVehAccReactRSU)
        cntVehAccReactRSU++;
    else if (id == sigVehAccReactV2V)
        cntVehAccReactV2V++;
    else if (id == sigVehAccReactRSUStrat)
        cntVehAccReactRSUStrat++;
    else if (id == sigVehAccRxV2V)
        cntVehAccRxV2V++;
    else if (id == sigVehAccRxRSU)
        cntVehAccRxRSU++;
    else if (id == sigVehAccRxStrat)
        cntVehAccRxStrat++;
    else if (id == sigAtRedLight)
        cntAtRedLight++;
    else if (id == sigEmergRedLight)
        cntEmergRedLight++;
    // socket signals
    else if (id == sigSockMobility)
        bytesMobility += value;
    else if (id == sigSockRSU)
        bytesRSU += value;
    else if (id == sigSockAccident)
        bytesAccident += value;
    else if (id == sigEmergencyReqBytes)
        bytesEmergencyReq += value;
    else if (id == sigSockEndOfStep)
        bytesStepSent += value;
    else if (id == sigSockTerminate)
        bytesTerminateSent += value;
    else if (id == sigSockEmptyPayload)
        bytesEmptyPayloadSent += value;
    // socket recv bytes signals
    else if (id == sigSockManoQueryBytes)
        bytesManoQueryReceived += value;
    else if (id == sigSafetyCmdBytes)
        bytesSafetyCmdReceived += value;
    else if (id == sigTrafficPreemptionBytes)
        bytesTrafficPreemptionReceived += value;
    else if (id == sigTimeSyncBytes)
        bytesTimeSyncReceived += value;
}

// --- Signal Handler: Double/Float Latencies ---
void MetricsCollector::receiveSignal(cComponent *source, simsignal_t id, double value, cObject *details)
{
    if (id == sigRsuCreationTime)
    {
        sumCreateTime += value;
        cntCreateTime++;
    }
    else if (id == sigRsuDeletionTime)
    {
        sumDeleteTime += value;
        cntDeleteTime++;
    }
    else if (id == sigTlsLatency)
    {
        sumTlsLatency += value;
        cntTlsLatency++;
    }
    else if (id == sigVehActionLatency)
    {
        sumVehicleActionLatency += value;
        cntVehicleActionLatency++;
    }
}

void MetricsCollector::writeMetricRows()
{
    double t = simTime().dbl();

    // --- 1. POLLING DATA (State) ---
    int activeRSUs = 0;
    int totalVehicles = 0;
    int emergencyVehicles = 0;
    double avgSpeed = 0.0;
    if (centralRSUManager)
    {
        CentralRSUManager *rsuMgr = check_and_cast<CentralRSUManager *>(centralRSUManager);
        activeRSUs = rsuMgr->getActiveRSUCount();

        totalVehicles = rsuMgr->getTotalVehicleCount();
        avgSpeed = rsuMgr->getAverageNetworkSpeed();
        emergencyVehicles = rsuMgr->getEmergencyVehicleCount();
    }

    // Placeholder for Traffic Data (You need to implement getters in MobilityManager)

    // --- 2. CALCULATIONS ---
    // Averages (Prevent Divide by Zero)
    double avgCreateMs = (cntCreateTime > 0) ? (sumCreateTime / cntCreateTime) : 0.0;
    double avgDeleteMs = (cntDeleteTime > 0) ? (sumDeleteTime / cntDeleteTime) : 0.0;
    double avgTlsLat = (cntTlsLatency > 0) ? (sumTlsLatency / cntTlsLatency) : 0.0;
    double avgVehicleActionLatency = (cntVehicleActionLatency > 0) ? (sumVehicleActionLatency / cntVehicleActionLatency) : 0.0;
    // Overhead Est (1KB per active RSU)
    long overheadKB = activeRSUs * 1;

    // --- 3. WRITE FILES ---

    // File A: Infrastructure
    if (fileInfrastructure.is_open())
    {
        fileInfrastructure << t << ","
                           << activeRSUs << ","
                           << cntScaleOut << ","
                           << cntScaleIn << ","
                           << avgCreateMs << ","
                           << avgDeleteMs << ","
                           << overheadKB << "\n";
        fileInfrastructure.flush();
    }

    // File B: Service Flows
    if (fileServiceFlows.is_open())
    {
        fileServiceFlows << t << ","
                         << emergencyVehicles << ","
                         << cntEmergRedLight << ","
                         << cntVehEmergSent << ","
                         << cntRsuEmergRx << ","
                         << cntCoreEmergProcessed << ","
                         << avgTlsLat * 1000 << ","
                         << avgVehicleActionLatency * 1000 << ","
                         << cntVehAccSent << "\n";
        fileServiceFlows.flush();
    }

    // File C: Traffic Context
    if (fileTraffic.is_open())
    {
        fileTraffic << t << ","
                    << totalVehicles << ","
                    << avgSpeed << ","
                    << cntAtRedLight << ","
                    << cntEmergRedLight << ","
                    << cntVehAccRxV2V << ","
                    << cntVehAccRxRSU << ","
                    << cntVehAccRxStrat << ","
                    << cntVehAccReactV2V << ","
                    << cntVehAccReactRSUStrat << ","
                    << cntVehAccReactRSU << "\n";
        fileTraffic.flush();
    }
    // File D: Socket Sent Throughput
    if (fileSentSocket.is_open())
    {
        long totalBytes = bytesMobility + bytesRSU + bytesEmergencyReq + bytesAccident +
                          bytesStepSent + bytesEmptyPayloadSent + bytesTerminateSent;
        fileSentSocket << t << "," << totalBytes << ","
                       << bytesMobility << ","
                       << bytesRSU << ","
                       << bytesEmergencyReq << ","
                       << bytesAccident << ","
                       << bytesStepSent << ","
                       << bytesEmptyPayloadSent << ","
                       << bytesTerminateSent
                       << "\n";
        fileSentSocket.flush();
    }
    // File E: Socket Received Throughput
    if (fileRecvSocket.is_open())
    {
        long totalBytesReceived = bytesManoQueryReceived + bytesSafetyCmdReceived +
                                  bytesTrafficPreemptionReceived + bytesTimeSyncReceived;
        fileRecvSocket << t << "," << totalBytesReceived << ","
                       << bytesManoQueryReceived << ","
                       << bytesSafetyCmdReceived << ","
                       << bytesTrafficPreemptionReceived << ","
                       << bytesTimeSyncReceived
                       << "\n";
        fileRecvSocket.flush();
    }

    // --- 4. RESET COUNTERS (For "Per Interval" Stats) ---
    cntScaleOut = 0;
    cntScaleIn = 0;
    sumCreateTime = 0;
    cntCreateTime = 0;
    sumDeleteTime = 0;
    cntDeleteTime = 0;
    cntVehEmergSent = 0;
    cntRsuEmergRx = 0;
    cntCoreEmergProcessed = 0;
    sumTlsLatency = 0;
    cntTlsLatency = 0;

    sumVehicleActionLatency = 0;
    cntVehicleActionLatency = 0;

    cntVehAccRxV2V = 0;
    cntVehAccRxRSU = 0;
    cntVehAccRxStrat = 0;
    cntVehAccSent = 0;
    cntVehAccReactV2V = 0;
    cntVehAccReactRSU = 0;
    cntVehAccReactRSUStrat = 0;
    cntAtRedLight = 0;
    cntEmergRedLight = 0;

    bytesMobility = 0;
    bytesRSU = 0;
    bytesAccident = 0;
    bytesEmergencyReq = 0;
    bytesStepSent = 0;
    bytesTerminateSent = 0;
    bytesEmptyPayloadSent = 0;

    bytesManoQueryReceived = 0;
    bytesSafetyCmdReceived = 0;
    bytesTrafficPreemptionReceived = 0;
    bytesTimeSyncReceived = 0;
}

void MetricsCollector::finish()
{
    cancelAndDelete(collectionTimer);
    if (fileInfrastructure.is_open())
        fileInfrastructure.close();
    if (fileServiceFlows.is_open())
        fileServiceFlows.close();
    if (fileTraffic.is_open())
        fileTraffic.close();
    if (fileSentSocket.is_open())
        fileSentSocket.close();
    if (fileRecvSocket.is_open())
        fileRecvSocket.close();
}
