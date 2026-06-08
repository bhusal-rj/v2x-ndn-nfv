/**
 * @file accident_notification.h
 * @brief Radius-based Accident Notification System Header
 * 
 * This module provides the interface for notifying nearby vehicles about accidents.
 * Features:
 * - Radius-based proximity detection
 * - NDN-based notification delivery
 * - Urgency classification based on distance
 * - Metrics tracking for QoS analysis
 */

#ifndef ACCIDENT_NOTIFICATION_H
#define ACCIDENT_NOTIFICATION_H

#include "ns3/core-module.h"
#include <string>
#include <vector>
#include <map>

namespace v2x {
namespace notification {

// Default notification radius (meters)
constexpr double DEFAULT_NOTIFICATION_RADIUS = 500.0;

/**
 * @brief Per-vehicle accident response record
 * Tracks which entity responded to a vehicle's accident query
 */
struct AccidentResponseRecord
{
    std::string vehicleId;          // Vehicle that was notified
    std::string responderId;        // Who responded: "local_cache", "rsu_X", or "MEC"
    std::string responderType;      // "CACHE_HIT" (local CS), "RSU" (nearby RSU cache), "MEC" (origin)
    double distanceToAccident = 0;  // Distance from vehicle to accident (meters)
    std::string urgency;            // CRITICAL / HIGH / MEDIUM / LOW
    std::string interestName;       // NDN Interest name used
    double responseTime = 0;        // Timestamp of notification
    double measuredLatencyMs = 0;   // Estimated/measured RTT latency in milliseconds
    
    // Per-Interest latency tracking (March 2026)
    ns3::Time interestSendTime;     // When Interest was sent
    ns3::Time dataReceiveTime;      // When Data was received
    double actualRttMs = -1.0;      // Actual measured RTT from trace callback (-1 = pending)
    int32_t hopCount = -1;          // Hop count from NDN trace (-1 = unknown)
    bool latencyMeasured = false;   // True when RTT measured via callback
};

/**
 * @brief Accident notification record
 */
struct AccidentNotification
{
    std::string accidentId;
    std::string crashedVehicleId;
    double accidentX = 0;
    double accidentY = 0;
    std::string roadId;
    std::string severity;  // MINOR, MODERATE, SEVERE
    ns3::Time timestamp;
    
    // Notification results
    std::vector<std::string> notifiedVehicles;
    uint32_t vehiclesInRadius = 0;
    uint32_t notificationsSent = 0;
    
    // Per-vehicle response tracking
    std::vector<AccidentResponseRecord> responseRecords;
    uint32_t cacheHitResponses = 0;   // Served from vehicle's local CS
    uint32_t rsuResponses = 0;        // Served from RSU cache
    uint32_t mecResponses = 0;        // Served from MEC origin
};

/**
 * @brief Statistics for accident notifications
 */
struct NotificationStats
{
    uint64_t totalAccidents = 0;
    uint64_t totalNotificationsSent = 0;
    uint64_t totalVehiclesInRange = 0;
    uint64_t successfulDeliveries = 0;
    uint64_t failedDeliveries = 0;
    double avgNotificationsPerAccident = 0.0;
};

/**
 * @brief Accident Notification Manager
 * 
 * Singleton class that manages accident notifications using radius-based
 * proximity detection and NDN-based message delivery.
 */
class AccidentNotificationManager
{
public:
    /**
     * @brief Get singleton instance
     */
    static AccidentNotificationManager& GetInstance();
    
    /**
     * @brief Set the notification radius
     * @param radius Radius in meters (default: 500m)
     */
    void SetNotificationRadius(double radius);
    
    /**
     * @brief Set whether to notify all vehicles regardless of radius
     * @param notifyAll If true, all active vehicles will be notified
     */
    void SetNotifyAllVehicles(bool notifyAll);
    
    /**
     * @brief Get the notification radius
     */
    double GetNotificationRadius() const { return m_notificationRadius; }
    
    /**
     * @brief Find all vehicles within radius of accident
     * @param accidentX X coordinate of accident
     * @param accidentY Y coordinate of accident
     * @param radius Search radius in meters
     * @return Vector of vehicle IDs within radius, sorted by distance
     */
    std::vector<std::string> FindVehiclesInRadius(double accidentX, double accidentY, 
                                                   double radius);
    
    /**
     * @brief Notify all nearby vehicles about an accident
     * @param accidentId Unique accident identifier
     * @param crashedVehicleId Vehicle involved in accident
     * @param accidentX X coordinate
     * @param accidentY Y coordinate
     * @param roadId Road segment ID
     * @param severity Accident severity (MINOR, MODERATE, SEVERE)
     * @return Notification record with results
     */
    AccidentNotification NotifyNearbyVehicles(const std::string& accidentId,
                                               const std::string& crashedVehicleId,
                                               double accidentX, double accidentY,
                                               const std::string& roadId,
                                               const std::string& severity = "MODERATE");
    
    /**
     * @brief Send NDN notification to a specific vehicle
     * @param targetVehicleId Vehicle to notify
     * @param notification Accident notification data
     * @param distance Distance from accident
     * @return true if notification was sent successfully
     */
    bool SendNdnNotification(const std::string& targetVehicleId,
                             const AccidentNotification& notification,
                             double distance);
    
    /**
     * @brief Generate actuation command for vehicle response
     * @param vehicleId Target vehicle
     * @param notification Accident notification
     * @param distance Distance from accident
     * @return JSON command string for OMNeT++
     */
    std::string GenerateActuationCommand(const std::string& vehicleId,
                                         const AccidentNotification& notification,
                                         double distance);
    
    /**
     * @brief Get notification statistics
     */
    const NotificationStats& GetStats() const { return m_stats; }
    
    /**
     * @brief Get all notifications
     */
    const std::map<std::string, AccidentNotification>& GetNotifications() const 
    { 
        return m_notifications; 
    }
    
    /**
     * @brief Print notification summary
     */
    void PrintSummary() const;
    
    /**
     * @brief Generate metrics JSON
     */
    std::string GenerateMetricsJson() const;

private:
    AccidentNotificationManager() : m_notificationRadius(DEFAULT_NOTIFICATION_RADIUS), m_notifyAllVehicles(true) {}
    
    double m_notificationRadius;
    bool m_notifyAllVehicles;  // Notify all vehicles regardless of radius
    NotificationStats m_stats;
    std::map<std::string, AccidentNotification> m_notifications;
    std::map<std::string, double> m_vehicleDistances;  // Temporary storage for sorting
    AccidentResponseRecord m_latestResponseRecord;     // Temp record from SendNdnNotification
};

} // namespace notification
} // namespace v2x

#endif // ACCIDENT_NOTIFICATION_H
