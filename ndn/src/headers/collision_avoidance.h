/**
 * @file collision_avoidance.h
 * @brief MEC-based Collision Avoidance for Architecture A - Use Case III
 * 
 * Closed-Loop MEC Collision Avoidance with Digital Twin.
 * Key features:
 * - RSU computes TTC at Edge (not Cloud) for low-latency response
 * - Digital Twin from OMNeT++ mobility data
 * - Bidirectional feedback loop (NS-3 → OMNeT++ actuation)
 * - Blind Spot Metric for NFV cost-safety tradeoff
 */

#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "ns3/core-module.h"
#include <string>
#include <map>
#include <vector>
#include <set>

namespace v2x {
namespace safety {

// ============================================================================
// COLLISION RISK LEVELS
// ============================================================================

enum class RiskLevel
{
    NONE,
    LOW,           // TTC > 3s, monitor
    MEDIUM,        // TTC 1.5-3s, advisory warning
    HIGH,          // TTC 0.5-1.5s, hard brake
    CRITICAL       // TTC < 0.5s or distance < 5m, emergency brake
};

// ============================================================================
// ACTUATION COMMANDS (NS-3 → OMNeT++)
// ============================================================================

enum class ActuationCommand
{
    NONE,
    MAINTAIN_DISTANCE,
    SLOW_DOWN,
    HARD_BRAKE,
    EMERGENCY_BRAKE,
    EVADE_LEFT,
    EVADE_RIGHT
};

struct CollisionWarning
{
    std::string vehicle1Id;
    std::string vehicle2Id;
    RiskLevel riskLevel;
    ActuationCommand recommendedAction;
    double ttc;           // Time-to-Collision (seconds) - raw from geometry
    double ttcAdjusted;   // TTC adjusted for position data age (latency-compensated)
    double distance;      // Current distance (meters)
    double closingSpeed;  // Relative closing speed (m/s)
    double positionAgeMs; // Max age of position data used in calculation (milliseconds)
    std::string detectedByRsu;
    ns3::Time detectionTime;
};

// ============================================================================
// MEC COLLISION DETECTOR - Per-RSU Edge Logic
// ============================================================================

struct CollisionDetectorStats
{
    uint64_t checksPerformed = 0;
    uint64_t warningsGenerated = 0;
    uint64_t actuationsTriggered = 0;
    std::map<RiskLevel, uint64_t> warningsByLevel;
    double avgTtcAtWarning = 0.0;
    uint64_t ttcSamples = 0;
    // Delivery tracking
    uint64_t messagesSent = 0;
    uint64_t messagesDelivered = 0;
    double totalDeliveryLatencyMs = 0.0;
};

/**
 * @brief MEC Collision Detector at RSU
 */
class MecCollisionDetector
{
public:
    MecCollisionDetector(const std::string& rsuId, double sensorRadius = 200.0);
    
    /**
     * @brief Set RSU position
     */
    void SetPosition(double x, double y);
    
    /**
     * @brief Check if a vehicle is in range of this RSU's sensors
     */
    bool IsInRange(double vehicleX, double vehicleY) const;
    
    /**
     * @brief Perform collision check on vehicles in range
     * Uses Digital Twin data from OMNeT++ mobility updates
     * @return List of collision warnings
     */
    std::vector<CollisionWarning> PerformCollisionCheck();
    
    /**
     * @brief Generate actuation command for vehicle 1 (OMNeT++ format)
     */
    std::string GenerateActuationCommand(const CollisionWarning& warning);
    
    /**
     * @brief Generate actuation command for vehicle 2 (OMNeT++ format)
     */
    std::string GenerateActuationCommandForVehicle2(const CollisionWarning& warning);
    
    /**
     * @brief Get statistics
     */
    const CollisionDetectorStats& GetStats() const { return m_stats; }

private:
    std::string m_rsuId;
    double m_sensorRadius;
    double m_x = 0, m_y = 0;
    CollisionDetectorStats m_stats;
    
    RiskLevel CalculateRiskLevel(double ttc, double distance, double closingSpeed);
    ActuationCommand DetermineAction(RiskLevel level, double ttc, double distance);
};

// ============================================================================
// BLIND SPOT TRACKER - NFV Cost vs Safety Tradeoff
// ============================================================================

struct BlindSpotMetrics
{
    uint64_t totalCollisionEvents = 0;
    uint64_t eventsInCoveredZones = 0;
    uint64_t eventsInBlindSpots = 0;
    double blindSpotRatio = 0.0;  // eventsInBlindSpots / totalCollisionEvents
    
    // Detailed tracking
    std::set<std::string> activeRsus;
    std::set<std::string> inactiveRsus;  // Scaled-in by MANO
    std::map<std::string, uint64_t> eventsPerZone;
};

/**
 * @brief Tracks Blind Spot Metric for NFV analysis
 */
class BlindSpotTracker
{
public:
    static BlindSpotTracker& GetInstance();
    
    /**
     * @brief Register RSU status (active/inactive based on MANO decision)
     */
    void SetRsuStatus(const std::string& rsuId, bool isActive);
    
    /**
     * @brief Record a potential collision event
     * @param x Event X position
     * @param y Event Y position
     * @param wasDetected true if RSU detected it (covered zone), false if blind spot
     */
    void RecordCollisionEvent(double x, double y, bool wasDetected, 
                              const std::string& detectedByRsu = "");
    
    /**
     * @brief Get current metrics
     */
    const BlindSpotMetrics& GetMetrics() const { return m_metrics; }
    
    /**
     * @brief Print summary
     */
    void PrintSummary() const;

private:
    BlindSpotTracker() = default;
    BlindSpotMetrics m_metrics;
};

// ============================================================================
// CENTRAL COLLISION AVOIDANCE MANAGER
// ============================================================================

/**
 * @brief Central manager for collision avoidance system
 */
class CollisionAvoidanceManager
{
public:
    static CollisionAvoidanceManager& GetInstance();
    
    /**
     * @brief Register RSU as collision detector
     */
    void RegisterRsuDetector(const std::string& rsuId, double x, double y, 
                             double sensorRadius = 200.0);
    
    /**
     * @brief Update RSU position (from OMNeT++ RSU_STATE)
     */
    void UpdateRsuPosition(const std::string& rsuId, double x, double y);
    
    /**
     * @brief Perform collision checks at all active RSUs
     * Called periodically (every 5-10ms in simulation time)
     * @return List of actuation commands for OMNeT++
     */
    std::vector<std::string> PerformGlobalCollisionCheck();
    
    /**
     * @brief Get aggregated statistics
     */
    CollisionDetectorStats GetAggregatedStats() const;
    
    /**
     * @brief Print summary
     */
    void PrintSummary() const;
    
    /**
     * @brief Get pending actuation commands
     */
    std::vector<std::string> GetPendingActuations();

private:
    CollisionAvoidanceManager() = default;
    std::map<std::string, MecCollisionDetector> m_detectors;
    std::vector<std::string> m_pendingActuations;
    std::set<std::string> m_recentWarnings;  // Dedup warnings within time window
};

} // namespace safety
} // namespace v2x

// ============================================================================
// SAFETY MESSAGE DELIVERY TRACKING FUNCTIONS
// ============================================================================

namespace v2x {
namespace safety {

/**
 * @brief Generate a unique message ID for tracking
 */
std::string GenerateSafetyMessageId(const std::string& sourceVehicle, 
                                     const std::string& targetVehicle);

/**
 * @brief Record a safety message being sent
 * Called when MEC generates a collision warning
 */
void RecordSafetyMessageSent(const std::string& messageId,
                              const std::string& sourceVehicle,
                              const std::string& targetVehicle,
                              const std::string& warningType,
                              const std::string& action);

/**
 * @brief Record a safety message being delivered to target vehicle
 * Called when vehicle receives warning via NDN Data callback
 */
void RecordSafetyMessageDelivered(const std::string& messageId);

/**
 * @brief Check if a message ID exists in tracking (for duplicate detection)
 */
bool IsSafetyMessageTracked(const std::string& messageId);

/**
 * @brief Get delivery statistics for safety messages
 */
struct SafetyDeliveryStats
{
    uint64_t messagesSent = 0;
    uint64_t messagesDelivered = 0;
    double deliveryRatio = 0.0;
    double avgDeliveryLatencyMs = 0.0;
    double minDeliveryLatencyMs = std::numeric_limits<double>::max();
    double maxDeliveryLatencyMs = 0.0;
};

SafetyDeliveryStats GetSafetyDeliveryStats();

/**
 * @brief Print safety delivery summary
 */
void PrintSafetyDeliverySummary();

} // namespace safety
} // namespace v2x

#endif // COLLISION_AVOIDANCE_H
