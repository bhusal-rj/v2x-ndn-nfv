/**
 * @file traffic_light_preemption.h
 * @brief Traffic Light Preemption for Architecture A - Use Case II
 * 
 * Edge-Based Emergency Traffic Light Preemption via Edge Computing (MEC).
 * Key features:
 * - Edge processing (no Cloud round-trip for low latency)
 * - Zone Controller (RSU manages all TLS in its cell)
 * - Multi-tenancy (RSU handles Safety + Traffic)
 * 
 * Note: Uses edge proximity for low latency (Edge-Based processing - no QCI 82/83 or GBR)
 */

#ifndef TRAFFIC_LIGHT_PREEMPTION_H
#define TRAFFIC_LIGHT_PREEMPTION_H

#include "ns3/core-module.h"
#include <string>
#include <map>
#include <vector>
#include <set>

namespace v2x {
namespace traffic {

// ============================================================================
// TRAFFIC LIGHT STATE
// ============================================================================

enum class TrafficLightState
{
    RED,
    YELLOW,
    GREEN,
    PREEMPTED_GREEN  // Forced green for emergency vehicle
};

/**
 * @brief Emergency Vehicle Priority Levels (standard EVP hierarchy)
 */
enum class EmergencyPriority
{
    NONE = 0,
    PRIORITY_3 = 1,   // Police vehicles
    PRIORITY_2 = 2,   // Ambulances
    PRIORITY_1 = 3    // Fire trucks (highest)
};

/**
 * @brief Preemption phases following standard EVP sequence
 */
enum class PreemptionPhase
{
    NONE,           // No active preemption
    CLEARANCE,      // Yellow/All-red to clear intersection (2-5s)
    GREEN,          // Green for emergency approach
    DWELL,          // Hold green while vehicle in intersection
    TRACK,          // Monitor vehicle passage
    EXIT            // Transition back to normal operation
};

struct TrafficLight
{
    std::string id;
    double x, y;
    TrafficLightState currentState = TrafficLightState::RED;
    std::string controllingRsu;  // Zone Controller RSU
    bool isPreempted = false;
    std::string preemptedBy;     // Emergency vehicle ID
    EmergencyPriority preemptedPriority = EmergencyPriority::NONE;
    PreemptionPhase preemptionPhase = PreemptionPhase::NONE;
    ns3::Time preemptionStart;
    ns3::Time phaseStartTime;
    ns3::Time normalCycleDuration = ns3::Seconds(30);
    ns3::Time maxPreemptionDuration = ns3::Seconds(120);  // Safety timeout
    double lastVehicleDistance = 999.0;  // For tracking vehicle passage
};

// ============================================================================
// ZONE CONTROLLER - RSU as MEC Host for Traffic
// ============================================================================

struct ZoneControllerStats
{
    uint64_t preemptionRequests = 0;
    uint64_t preemptionsGranted = 0;
    uint64_t preemptionsDenied = 0;
    uint64_t preemptionsPreempted = 0;  // Higher priority took over
    uint64_t preemptionsTimedOut = 0;   // Safety timeout triggered
    double avgPreemptionLatencyMs = 0.0;
    double avgClearanceTimeMs = 0.0;
    double avgTotalPreemptionDurationS = 0.0;
    uint64_t latencySamples = 0;
    std::map<std::string, uint64_t> perTlsPreemptions;
};

/**
 * @brief Zone Controller - RSU managing traffic lights in its cell
 */
class ZoneController
{
public:
    ZoneController(const std::string& rsuId);
    
    /**
     * @brief Register a traffic light under this zone controller
     */
    void RegisterTrafficLight(const std::string& tlsId, double x, double y);
    
    /**
     * @brief Process emergency preemption request (Edge Processing)
     * @param emergencyVehicleId ID of emergency vehicle
     * @param targetTlsId Target traffic light
     * @param vehicleX Vehicle X position
     * @param vehicleY Vehicle Y position
     * @param vehicleSpeed Vehicle speed (m/s)
     * @return true if preemption granted
     */
    bool ProcessPreemptionRequest(const std::string& emergencyVehicleId,
                                  const std::string& targetTlsId,
                                  double vehicleX, double vehicleY,
                                  double vehicleSpeed);
    
    /**
     * @brief Generate command for OMNeT++ to actuate traffic light
     * @param tlsId Traffic light system ID
     * @param newState New state to set
     * @param reason Reason for the change (e.g., "EMERGENCY:vehicleId")
     * @param laneId Lane ID for the preemption (optional)
     * @return JSON command string
     */
    std::string GenerateActuationCommand(const std::string& tlsId, 
                                         TrafficLightState newState,
                                         const std::string& reason,
                                         const std::string& laneId = "");
    
    /**
     * @brief Release preemption after emergency vehicle passes
     */
    void ReleasePreemption(const std::string& tlsId);
    
    /**
     * @brief Update vehicle tracking for preemption release detection
     * @param vehicleId Emergency vehicle ID
     * @param vehicleX Current X position
     * @param vehicleY Current Y position
     * @return Commands to send if preemption should be released
     */
    std::vector<std::string> UpdateVehicleTracking(const std::string& vehicleId,
                                                   double vehicleX, double vehicleY);
    
    /**
     * @brief Check for preemption timeouts (safety mechanism)
     * @return Commands to release timed-out preemptions
     */
    std::vector<std::string> CheckPreemptionTimeouts();
    
    /**
     * @brief Determine emergency vehicle priority from vehicle type
     */
    static EmergencyPriority GetVehiclePriority(const std::string& vehicleId);
    
    /**
     * @brief Get zone controller statistics
     */
    const ZoneControllerStats& GetStats() const { return m_stats; }
    
    /**
     * @brief Get managed traffic lights
     */
    const std::map<std::string, TrafficLight>& GetTrafficLights() const { return m_trafficLights; }

private:
    std::string m_rsuId;
    std::map<std::string, TrafficLight> m_trafficLights;
    ZoneControllerStats m_stats;
    std::set<std::string> m_activePreemptions;  // Currently preempted TLS IDs
    
    double CalculateETA(double vehicleX, double vehicleY, double vehicleSpeed,
                        double tlsX, double tlsY);
    
    double CalculateDistance(double x1, double y1, double x2, double y2);
};

// ============================================================================
// TRAFFIC PREEMPTION MANAGER - Central coordinator
// ============================================================================

/**
 * @brief Central manager for traffic light preemption
 */
class TrafficPreemptionManager
{
public:
    static TrafficPreemptionManager& GetInstance();
    
    /**
     * @brief Register zone controller
     */
    void RegisterZoneController(const std::string& rsuId);
    
    /**
     * @brief Register traffic light to nearest zone controller
     */
    void RegisterTrafficLight(const std::string& tlsId, double x, double y);
    
    /**
     * @brief Process NDN Interest: /traffic/{tlsId}/force_green
     * @return Command to send to OMNeT++ or empty if denied
     */
    std::string ProcessPreemptionInterest(const std::string& interestName,
                                          const std::string& emergencyVehicleId,
                                          double vehicleX, double vehicleY,
                                          double vehicleSpeed);
    
    /**
     * @brief Process direct preemption request (from OMNeT++ traffic_preemption_request)
     * @param tlsId Target traffic light system ID
     * @param emergencyVehicleId ID of the requesting emergency vehicle
     * @param laneId Lane ID where the vehicle is approaching from
     * @param vehicleX Vehicle X position
     * @param vehicleY Vehicle Y position  
     * @param vehicleSpeed Vehicle speed (m/s)
     * @return Command to send to OMNeT++ or empty if denied
     */
    std::string ProcessDirectPreemptionRequest(const std::string& tlsId,
                                               const std::string& emergencyVehicleId,
                                               const std::string& laneId,
                                               double vehicleX, double vehicleY,
                                               double vehicleSpeed);
    
    /**
     * @brief Update traffic light state from OMNeT++ feedback
     */
    void UpdateTrafficLightState(const std::string& tlsId, 
                                 const std::string& state);
    
    /**
     * @brief Periodic update to track vehicle positions and check timeouts
     * Called from mobility_update processing
     * @return Commands for releasing preemptions
     */
    std::vector<std::string> PeriodicPreemptionUpdate();
    
    /**
     * @brief Get aggregated statistics
     */
    ZoneControllerStats GetAggregatedStats() const;
    
    /**
     * @brief Print summary
     */
    void PrintSummary() const;
    
    /**
     * @brief Get pending commands for OMNeT++
     */
    std::vector<std::string> GetPendingCommands();

    /**
     * @brief Number of registered zone controllers
     */
    size_t GetZoneControllerCount() const { return m_zoneControllers.size(); }

private:
    TrafficPreemptionManager() = default;
    std::map<std::string, ZoneController> m_zoneControllers;
    std::vector<std::string> m_pendingCommands;
    
    std::string FindZoneController(double x, double y);
};

} // namespace traffic
} // namespace v2x

#endif // TRAFFIC_LIGHT_PREEMPTION_H
