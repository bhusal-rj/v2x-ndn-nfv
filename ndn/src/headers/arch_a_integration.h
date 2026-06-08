/**
 * @file arch_a_integration.h
 * @brief Architecture A Integration Layer
 * 
 * Integrates all three use cases:
 * - Use Case I: Intelligent Accident Proactive Caching (MEC Edge Filter)
 * - Use Case II: Edge-Based Traffic Light Preemption (Zone Controller)
 * - Use Case III: Closed-Loop MEC Collision Avoidance (Digital Twin + Actuation)
 * 
 * This header provides the main integration functions to be called from simple_ndn.cc
 */

#ifndef ARCH_A_INTEGRATION_H
#define ARCH_A_INTEGRATION_H

#include "ns3/core-module.h"
#include <string>
#include <vector>

namespace v2x {
namespace arch_a {

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize all Architecture A components
 * Called once at simulation start after nodes are created
 */
void InitializeArchA();

/**
 * @brief Register an RSU with all subsystems (Edge Filter, Zone Controller, Collision Detector)
 */
void RegisterRsuWithArchA(const std::string& rsuId, double x, double y);

/**
 * @brief Register a traffic light with the Traffic Preemption Manager
 */
void RegisterTrafficLightWithArchA(const std::string& tlsId, double x, double y);

// ============================================================================
// MESSAGE PROCESSING (from OMNeT++)
// ============================================================================

/**
 * @brief Process accident report through MEC Edge Filter
 * @return Commands to send back to OMNeT++ (for proactive caching)
 */
std::vector<std::string> ProcessAccidentReport(const std::string& accidentId,
                                               const std::string& vehicleId,
                                               const std::string& roadId,
                                               double vehicleX, double vehicleY);

/**
 * @brief Process traffic update (traffic light states from OMNeT++)
 */
void ProcessTrafficUpdate(const std::string& tlsId, const std::string& state,
                          double x, double y);

/**
 * @brief Process mobility update (Digital Twin sync for collision avoidance)
 * Called after updating vehicleStatuses from OMNeT++ data
 */
void ProcessMobilityUpdate();

/**
 * @brief Check for emergency vehicle preemption requests
 * @param emergencyVehicleId Vehicle requesting preemption
 * @param tlsId Target traffic light
 * @param vehicleX Vehicle position X
 * @param vehicleY Vehicle position Y
 * @param vehicleSpeed Vehicle speed (m/s)
 * @return Command for OMNeT++ or empty string
 */
std::string ProcessPreemptionRequest(const std::string& emergencyVehicleId,
                                     const std::string& tlsId,
                                     double vehicleX, double vehicleY,
                                     double vehicleSpeed);

/**
 * @brief Process preemption request with lane ID (from OMNeT++ traffic_preemption_request)
 * @param emergencyVehicleId Vehicle requesting preemption
 * @param tlsId Target traffic light
 * @param laneId Lane ID where the vehicle is approaching from
 * @param vehicleX Vehicle position X
 * @param vehicleY Vehicle position Y
 * @param vehicleSpeed Vehicle speed (m/s)
 * @return Command for OMNeT++ or empty string
 */
std::string ProcessPreemptionRequestWithLane(const std::string& emergencyVehicleId,
                                             const std::string& tlsId,
                                             const std::string& laneId,
                                             double vehicleX, double vehicleY,
                                             double vehicleSpeed);

// ============================================================================
// PERIODIC TASKS (scheduled in NS-3)
// ============================================================================

/**
 * @brief Periodic collision check at all RSUs (every 5-10ms)
 * Generates actuation commands for OMNeT++
 */
void PeriodicCollisionCheck();

/**
 * @brief Schedule Architecture A periodic tasks
 */
void ScheduleArchATasks();

// ============================================================================
// FEEDBACK LOOP (NS-3 → OMNeT++)
// ============================================================================

/**
 * @brief Get all pending commands to send to OMNeT++
 * Call this before sending TIME_SYNC to include commands
 */
std::vector<std::string> GetPendingOmnetCommands();

/**
 * @brief Clear pending commands after sending
 */
void ClearPendingCommands();

/**
 * @brief Trigger accident notifications/actuations after proactive caching (called on MANO_DECISION)
 */
std::vector<std::string> ProcessAccidentPostCaching(const std::string& accidentId);

// ============================================================================
// METRICS AND SUMMARY
// ============================================================================

/**
 * @brief Print comprehensive Architecture A summary
 */
void PrintArchASummary();

/**
 * @brief Generate Architecture A metrics JSON
 */
std::string GenerateArchAMetricsJson();

/**
 * @brief Export Architecture A metrics to file
 */
void ExportArchAMetrics(const std::string& filename);

} // namespace arch_a
} // namespace v2x

#endif // ARCH_A_INTEGRATION_H
