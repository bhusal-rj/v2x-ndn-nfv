/**
 * @file arch_a_integration.cc
 * @brief Implementation of Architecture A Integration Layer
 * 
 * This file integrates all three use cases of Architecture A:
 * 1. Intelligent Accident Proactive Caching (Edge Filter + MANO)
 * 2. Edge-Based Traffic Light Preemption (Zone Controller)
 * 3. Closed-Loop MEC Collision Avoidance (Digital Twin + Actuation)
 * 4. Radius-based Accident Notification System
 */

#include "arch_a_integration.h"
#include "mec_edge_filter.h"
#include "traffic_light_preemption.h"
#include "collision_avoidance.h"
#include "accident_notification.h"
#include "simulation_state.h"
#include "node_management.h"
#include "v2x_constants.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <unordered_map>

namespace v2x {
namespace arch_a {

// Pending commands buffer
static std::vector<std::string> g_pendingCommands;

struct AccidentContext
{
    std::string accidentId;
    std::string vehicleId;
    std::string roadId;
    double vehicleX = 0.0;
    double vehicleY = 0.0;
};

// Store accident context until MANO_DECISION completes caching
static std::unordered_map<std::string, AccidentContext> g_accidentContexts;

// Track if initialized
static bool g_initialized = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void InitializeArchA()
{
    if (g_initialized)
    {
        return;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Initializing Architecture A" << std::endl;
    std::cout << "  Fully Integrated Closed-Loop V2X-NFV-NDN-5G" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::cout << "📋 Use Cases Enabled:" << std::endl;
    std::cout << "  [UC1] Intelligent Accident Proactive Caching" << std::endl;
    std::cout << "  [UC2] Edge-Based Traffic Light Preemption" << std::endl;
    std::cout << "  [UC3] Closed-Loop MEC Collision Avoidance" << std::endl;
    std::cout << "  [UC4] Radius-based Accident Notification" << std::endl;
    std::cout << "" << std::endl;
    
    // Initialize notification system with large radius to notify all vehicles
    notification::AccidentNotificationManager::GetInstance().SetNotificationRadius(10000.0);  // 10km - covers entire simulation
    notification::AccidentNotificationManager::GetInstance().SetNotifyAllVehicles(true);
    
    g_initialized = true;
}

void RegisterRsuWithArchA(const std::string& rsuId, double x, double y)
{
    bool isNewRegistration = false;

    // Use Case I: MEC Edge Filter
    const auto beforeFilters = mec::MecEdgeManager::GetInstance().GetRegisteredRsuCount();
    mec::MecEdgeManager::GetInstance().RegisterRsu(rsuId);

    // Use Case II: Zone Controller for Traffic
    const auto beforeZones = traffic::TrafficPreemptionManager::GetInstance().GetZoneControllerCount();
    traffic::TrafficPreemptionManager::GetInstance().RegisterZoneController(rsuId);

    if (mec::MecEdgeManager::GetInstance().GetRegisteredRsuCount() > beforeFilters ||
        traffic::TrafficPreemptionManager::GetInstance().GetZoneControllerCount() > beforeZones)
    {
        isNewRegistration = true;
    }

    // Use Case II: Zone Controller for Traffic
    // Use Case III: Collision Detector
    safety::CollisionAvoidanceManager::GetInstance().RegisterRsuDetector(rsuId, x, y, 200.0);

    if (isNewRegistration)
    {
        std::cout << "✅ [ArchA] RSU " << rsuId << " registered with all subsystems" << std::endl;
    }
}

void RegisterTrafficLightWithArchA(const std::string& tlsId, double x, double y)
{
    traffic::TrafficPreemptionManager::GetInstance().RegisterTrafficLight(tlsId, x, y);
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

std::vector<std::string> ProcessAccidentReport(const std::string& accidentId,
                                               const std::string& vehicleId,
                                               const std::string& roadId,
                                               double vehicleX, double vehicleY)
{
    std::vector<std::string> commands;
    
    std::cout << "\n🚨 [ArchA-UC1] Processing Accident Report: " << accidentId << std::endl;
    
    // Step 1: Edge Filtering (MEC Fast Path) - Check for duplicates
    std::string processingRsu = mec::MecEdgeManager::GetInstance().ProcessAccidentAtEdge(
        accidentId, vehicleId, vehicleX, vehicleY, 400  // ~400 bytes payload
    );
    
    if (processingRsu.empty())
    {
        // Duplicate - dropped at Edge
        std::cout << "  ➡️  Duplicate dropped at Edge (backhaul protected)" << std::endl;
        return commands;  // Empty - no Cloud forwarding needed
    }
    
    // Step 2: Unique report - Forward to MANO for proactive caching decision
    // DO NOT cache locally - let MANO decide based on road topology analysis
    std::cout << "  ➡️  Unique report - forwarding to MANO for topology-aware caching decision" << std::endl;
    
    // Generate VNF_QUERY to request MANO decision on WHERE to cache
    // MANO will analyze upstream roads and return target RSUs
    static int queryCounter = 0;
    queryCounter++;
    std::stringstream queryId;
    queryId << "q_" << (int)(ns3::Simulator::Now().GetSeconds() * 100) << "_" << queryCounter;
    
    // Translate NS-3 RSU ID to OMNeT++ RSU ID for cross-simulator compatibility
    std::string omnetOriginRsu = GetOmnetRsuId(processingRsu);
    if (omnetOriginRsu.empty()) omnetOriginRsu = processingRsu;  // Fallback
    
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{\"message_type\":\"vnf_query\","
       << "\"timestamp\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"payload\":{"
       << "\"query_id\":\"" << queryId.str() << "\","
       << "\"action\":\"cache_decision\","
       << "\"accident_id\":\"" << accidentId << "\","
       << "\"content_name\":\"/v2x/safety/" << accidentId << "\","
       << "\"lane_id\":\"" << roadId << "\","
       << "\"pos_x\":" << vehicleX << ","
       << "\"pos_y\":" << vehicleY << ","
       << "\"origin_rsu\":\"" << omnetOriginRsu << "\""
       << "}"
       << "}";
    
    commands.push_back(ss.str());
    g_pendingCommands.push_back(ss.str());
    
    std::cout << "  📤 Sent VNF_QUERY to OMNeT++ MANO:" << std::endl;
    std::cout << "     Query ID: " << queryId.str() << std::endl;
    std::cout << "     Action: cache_decision" << std::endl;
    std::cout << "     Accident: " << accidentId << std::endl;
    std::cout << "     Location: (" << vehicleX << ", " << vehicleY << ")" << std::endl;
    std::cout << "     Road: " << roadId << std::endl;
    std::cout << "  ⏳ Waiting for MANO_DECISION with upstream RSU targets..." << std::endl;
    
    // NOTE: Caching will happen when MANO_DECISION is received in simple_ndn.cc
    // The MANO in OMNeT++ will:
    //   1. Analyze the road topology graph
    //   2. Find upstream edges (feeder roads) leading to the accident location
    //   3. Identify RSUs covering those upstream roads
    //   4. Send mano_decision with target RSU list
    
    // Step 3: Notify nearby vehicles (this is immediate, separate from caching)
    // Defer notifications/actuations until MANO_DECISION is received (after proactive caching)
    AccidentContext ctx;
    ctx.accidentId = accidentId;
    ctx.vehicleId = vehicleId;
    ctx.roadId = roadId;
    ctx.vehicleX = vehicleX;
    ctx.vehicleY = vehicleY;
    g_accidentContexts[accidentId] = ctx;
    
    return commands;
}

void ProcessTrafficUpdate(const std::string& tlsId, const std::string& state,
                          double x, double y)
{
    // Register TLS if new
    RegisterTrafficLightWithArchA(tlsId, x, y);
    
    // Update state
    traffic::TrafficPreemptionManager::GetInstance().UpdateTrafficLightState(tlsId, state);
}

void ProcessMobilityUpdate()
{
    // Digital Twin is already updated in vehicleStatuses
    // Collision avoidance will use this data in periodic checks
    
    // Update RSU positions for collision detectors
    for (const auto& [id, node] : nodeMapping)
    {
        if (id.find("rsu") != std::string::npos)
        {
            ns3::Ptr<ns3::MobilityModel> mobility = node->GetObject<ns3::MobilityModel>();
            if (mobility)
            {
                ns3::Vector pos = mobility->GetPosition();
                safety::CollisionAvoidanceManager::GetInstance().UpdateRsuPosition(id, pos.x, pos.y);
            }
        }
    }
}

std::string ProcessPreemptionRequest(const std::string& emergencyVehicleId,
                                     const std::string& tlsId,
                                     double vehicleX, double vehicleY,
                                     double vehicleSpeed)
{
    std::cout << "\n🚑 [ArchA-UC2] Emergency Preemption Request" << std::endl;
    std::cout << "  Vehicle: " << emergencyVehicleId << std::endl;
    std::cout << "  Target TLS: " << tlsId << std::endl;
    
    std::string interestName = "/traffic/" + tlsId + "/force_green";
    
    std::string command = traffic::TrafficPreemptionManager::GetInstance().ProcessPreemptionInterest(
        interestName, emergencyVehicleId, vehicleX, vehicleY, vehicleSpeed
    );
    
    if (!command.empty())
    {
        g_pendingCommands.push_back(command);
    }
    
    return command;
}

std::string ProcessPreemptionRequestWithLane(const std::string& emergencyVehicleId,
                                              const std::string& tlsId,
                                              const std::string& laneId,
                                              double vehicleX, double vehicleY,
                                              double vehicleSpeed)
{
    std::cout << "\n🚑 [ArchA-UC2] Emergency Preemption Request with Lane" << std::endl;
    std::cout << "  Vehicle: " << emergencyVehicleId << std::endl;
    std::cout << "  Target TLS: " << tlsId << std::endl;
    std::cout << "  Lane: " << laneId << std::endl;
    
    // Call the TrafficPreemptionManager with the correct argument order:
    // (tlsId, emergencyVehicleId, laneId, x, y, speed)
    std::string command = traffic::TrafficPreemptionManager::GetInstance().ProcessDirectPreemptionRequest(
        tlsId, emergencyVehicleId, laneId, vehicleX, vehicleY, vehicleSpeed
    );
    
    if (!command.empty())
    {
        g_pendingCommands.push_back(command);
        std::cout << "  ✓ Preemption command queued for OMNeT++" << std::endl;
    }
    
    return command;
}

// ============================================================================
// POST-CACHING ACCIDENT NOTIFICATION (invoked on MANO_DECISION)
// ============================================================================
std::vector<std::string> ProcessAccidentPostCaching(const std::string& accidentId)
{
    std::vector<std::string> cmds;

    auto ctxIt = g_accidentContexts.find(accidentId);
    if (ctxIt == g_accidentContexts.end())
    {
        std::cerr << "  ⚠️  [ArchA] No stored context for accident " << accidentId << std::endl;
        return cmds;
    }
    const auto& ctx = ctxIt->second;

    // Notify nearby vehicles (now that proactive caching should be in place)
    auto notification = notification::AccidentNotificationManager::GetInstance().NotifyNearbyVehicles(
        ctx.accidentId, ctx.vehicleId, ctx.vehicleX, ctx.vehicleY, ctx.roadId, "MODERATE");

    // Generate actuation commands for notified vehicles
    for (const auto& notifiedVehicle : notification.notifiedVehicles)
    {
        auto vehIt = vehicleStatuses.find(notifiedVehicle);
        if (vehIt != vehicleStatuses.end())
        {
            double dx = vehIt->second.x - ctx.vehicleX;
            double dy = vehIt->second.y - ctx.vehicleY;
            double distance = std::sqrt(dx * dx + dy * dy);

            std::string actuationCmd = notification::AccidentNotificationManager::GetInstance()
                .GenerateActuationCommand(notifiedVehicle, notification, distance);

            cmds.push_back(actuationCmd);
            g_pendingCommands.push_back(actuationCmd);
        }
    }

    // Once processed, erase context to avoid duplicate notifications
    g_accidentContexts.erase(ctxIt);

    return cmds;
}

// ============================================================================
// PERIODIC TASKS
// ============================================================================

void PeriodicCollisionCheck()
{
    if (vehicleStatuses.empty())
    {
        ns3::Simulator::Schedule(ns3::MilliSeconds(ARCHA_COLLISION_CHECK_INTERVAL_MS), &PeriodicCollisionCheck);
        return;
    }

    bool hasActiveVehicle = false;
    for (const auto& [id, status] : vehicleStatuses)
    {
        if (status.isActive)
        {
            hasActiveVehicle = true;
            break;
        }
    }
    if (!hasActiveVehicle)
    {
        ns3::Simulator::Schedule(ns3::MilliSeconds(ARCHA_COLLISION_CHECK_INTERVAL_MS), &PeriodicCollisionCheck);
        return;
    }

    // Perform collision check at all RSUs
    auto commands = safety::CollisionAvoidanceManager::GetInstance().PerformGlobalCollisionCheck();
    
    // Add to pending commands for OMNeT++
    for (const auto& cmd : commands)
    {
        g_pendingCommands.push_back(cmd);
    }
    
    // Schedule next check
    ns3::Simulator::Schedule(ns3::MilliSeconds(ARCHA_COLLISION_CHECK_INTERVAL_MS), &PeriodicCollisionCheck);
}

void ScheduleArchATasks()
{
    std::cout << "📅 [ArchA] Scheduling periodic tasks..." << std::endl;
    std::cout << "  - Collision checks: every " << ARCHA_COLLISION_CHECK_INTERVAL_MS << "ms" << std::endl;
    
    // Start collision checks after 100ms (allow setup to complete)
    ns3::Simulator::Schedule(ns3::MilliSeconds(100), &PeriodicCollisionCheck);
}

// ============================================================================
// FEEDBACK LOOP
// ============================================================================

std::vector<std::string> GetPendingOmnetCommands()
{
    // Also get commands from Traffic Preemption
    auto trafficCmds = traffic::TrafficPreemptionManager::GetInstance().GetPendingCommands();
    for (const auto& cmd : trafficCmds)
    {
        g_pendingCommands.push_back(cmd);
    }
    
    // Also get collision actuations
    auto collisionCmds = safety::CollisionAvoidanceManager::GetInstance().GetPendingActuations();
    for (const auto& cmd : collisionCmds)
    {
        g_pendingCommands.push_back(cmd);
    }
    
    return g_pendingCommands;
}

void ClearPendingCommands()
{
    g_pendingCommands.clear();
}

// ============================================================================
// METRICS AND SUMMARY
// ============================================================================

void PrintArchASummary()
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║          ARCHITECTURE A - FINAL SUMMARY                       ║" << std::endl;
    std::cout << "║  Fully Integrated Closed-Loop V2X-NFV-NDN-5G                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    // Use Case I: Edge Filter
    mec::MecEdgeManager::GetInstance().PrintSummary();
    
    // Accident Notification System
    notification::AccidentNotificationManager::GetInstance().PrintSummary();
    
    // Use Case II: Traffic Preemption
    traffic::TrafficPreemptionManager::GetInstance().PrintSummary();
    
    // Use Case III: Collision Avoidance
    safety::CollisionAvoidanceManager::GetInstance().PrintSummary();
}

std::string GenerateArchAMetricsJson()
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    
    auto edgeStats = mec::MecEdgeManager::GetInstance().GetAggregatedStats();
    auto trafficStats = traffic::TrafficPreemptionManager::GetInstance().GetAggregatedStats();
    auto collisionStats = safety::CollisionAvoidanceManager::GetInstance().GetAggregatedStats();
    auto blindSpotStats = safety::BlindSpotTracker::GetInstance().GetMetrics();
    auto notificationStats = notification::AccidentNotificationManager::GetInstance().GetStats();
    
    ss << "{\n";
    ss << "  \"architecture\": \"A\",\n";
    ss << "  \"version\": \"2.0\",\n";
    ss << "  \"timestamp\": " << ns3::Simulator::Now().GetSeconds() << ",\n";
    
    // Use Case I: Edge Filter
    ss << "  \"use_case_1_edge_filter\": {\n";
    ss << "    \"total_packets_received\": " << edgeStats.totalPacketsReceived << ",\n";
    ss << "    \"duplicates_dropped\": " << edgeStats.duplicatesDropped << ",\n";
    ss << "    \"unique_forwarded\": " << edgeStats.uniqueForwarded << ",\n";
    ss << "    \"backhaul_bytes_saved\": " << edgeStats.backhaulBytesSaved << ",\n";
    ss << "    \"filter_efficiency_percent\": " << edgeStats.filterEfficiency << "\n";
    ss << "  },\n";
    
    // Accident Notification
    ss << "  \"accident_notification\": {\n";
    ss << "    \"notification_radius_m\": " << notification::AccidentNotificationManager::GetInstance().GetNotificationRadius() << ",\n";
    ss << "    \"total_accidents\": " << notificationStats.totalAccidents << ",\n";
    ss << "    \"total_notifications_sent\": " << notificationStats.totalNotificationsSent << ",\n";
    ss << "    \"total_vehicles_in_range\": " << notificationStats.totalVehiclesInRange << "\n";
    ss << "  },\n";
    
    // Use Case II: Traffic Preemption
    ss << "  \"use_case_2_traffic_preemption\": {\n";
    ss << "    \"preemption_requests\": " << trafficStats.preemptionRequests << ",\n";
    ss << "    \"preemptions_granted\": " << trafficStats.preemptionsGranted << ",\n";
    ss << "    \"preemptions_denied\": " << trafficStats.preemptionsDenied << ",\n";
    ss << "    \"avg_processing_latency_ms\": " << trafficStats.avgPreemptionLatencyMs << "\n";
    ss << "  },\n";
    
    // Use Case III: Collision Avoidance
    ss << "  \"use_case_3_collision_avoidance\": {\n";
    ss << "    \"checks_performed\": " << collisionStats.checksPerformed << ",\n";
    ss << "    \"warnings_generated\": " << collisionStats.warningsGenerated << ",\n";
    ss << "    \"actuations_triggered\": " << collisionStats.actuationsTriggered << ",\n";
    ss << "    \"avg_ttc_at_warning_s\": " << collisionStats.avgTtcAtWarning << "\n";
    ss << "  },\n";
    
    // Blind Spot Metrics
    ss << "  \"blind_spot_analysis\": {\n";
    ss << "    \"total_collision_events\": " << blindSpotStats.totalCollisionEvents << ",\n";
    ss << "    \"events_in_covered_zones\": " << blindSpotStats.eventsInCoveredZones << ",\n";
    ss << "    \"events_in_blind_spots\": " << blindSpotStats.eventsInBlindSpots << ",\n";
    ss << "    \"blind_spot_ratio_percent\": " << blindSpotStats.blindSpotRatio << ",\n";
    ss << "    \"active_rsus\": " << blindSpotStats.activeRsus.size() << ",\n";
    ss << "    \"inactive_rsus\": " << blindSpotStats.inactiveRsus.size() << "\n";
    ss << "  }\n";
    
    ss << "}\n";
    
    return ss.str();
}

void ExportArchAMetrics(const std::string& filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        file << GenerateArchAMetricsJson();
        file.close();
        std::cout << "📊 [ArchA] Metrics exported to " << filename << std::endl;
    }
    else
    {
        std::cerr << "⚠️  [ArchA] Failed to export metrics to " << filename << std::endl;
    }
}

} // namespace arch_a
} // namespace v2x
