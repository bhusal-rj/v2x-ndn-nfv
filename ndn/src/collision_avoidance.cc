/**
 * @file collision_avoidance.cc
 * @brief Implementation of MEC-based Collision Avoidance for Architecture A - Use Case III
 * 
 * Closed-Loop MEC Collision Avoidance:
 * 1. OMNeT++ sends mobility_update (Digital Twin sync)
 * 2. NS-3 RSU computes TTC at Edge (low-latency)
 * 3. NS-3 generates actuation command (HARD_BRAKE, etc.)
 * 4. Command sent to OMNeT++ for physics execution
 * 
 * ARCHITECTURE UPDATE (March 2026):
 * - MEC now queries vehicle positions via NDN Interest/Data exchange
 * - Uses ndnReceivedPositions cache instead of direct vehicleStatuses access
 * - Enables realistic V2I latency measurement for position queries
 * 
 * DELIVERY TRACKING UPDATE (2026):
 * - Track sent vs delivered safety messages for research validity
 * - Measure actual delivery latency for safety system effectiveness
 */

#include "collision_avoidance.h"
#include "node_management.h"
#include "simulation_state.h"
#include "ndn_position_cache.h"
#include "v2x_constants.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <atomic>

namespace v2x {
namespace safety {

// Atomic counter for unique message IDs
static std::atomic<uint64_t> g_safetyMessageCounter{0};

// ============================================================================
// MEC COLLISION DETECTOR IMPLEMENTATION
// ============================================================================

MecCollisionDetector::MecCollisionDetector(const std::string& rsuId, double sensorRadius)
    : m_rsuId(rsuId), m_sensorRadius(sensorRadius)
{
}

void MecCollisionDetector::SetPosition(double x, double y)
{
    m_x = x;
    m_y = y;
}

bool MecCollisionDetector::IsInRange(double vehicleX, double vehicleY) const
{
    double dx = vehicleX - m_x;
    double dy = vehicleY - m_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance <= m_sensorRadius;
}

RiskLevel MecCollisionDetector::CalculateRiskLevel(double ttc, double distance, double closingSpeed)
{
    // TTC-based risk assessment using documented thresholds
    // Citation: ETSI EN 302 637-2 V1.4.1 Section 6.1.3 - CAM generation rules
    // Citation: NHTSA DOT HS 812 115 - FCW Performance Specifications
    
    // Critical distance threshold: ~1/3 of NHTSA minimum safe distance
    constexpr double CRITICAL_DISTANCE_M = NHTSA_MIN_SAFE_DISTANCE_M / 3.0;  // ~5m
    
    if (distance < CRITICAL_DISTANCE_M)
    {
        return RiskLevel::CRITICAL;
    }
    else if (ttc < TTC_CRITICAL_S || (distance < NHTSA_MIN_SAFE_DISTANCE_M && closingSpeed > 5.0))
    {
        return RiskLevel::CRITICAL;
    }
    else if (ttc < TTC_HIGH_S)
    {
        return RiskLevel::HIGH;
    }
    else if (ttc < TTC_MEDIUM_S)
    {
        return RiskLevel::MEDIUM;
    }
    else if (ttc < TTC_LOW_S && closingSpeed > 2.0)
    {
        return RiskLevel::LOW;
    }
    
    return RiskLevel::NONE;
}

ActuationCommand MecCollisionDetector::DetermineAction(RiskLevel level, double ttc, double distance)
{
    switch (level)
    {
        case RiskLevel::CRITICAL:
            return ActuationCommand::EMERGENCY_BRAKE;
        case RiskLevel::HIGH:
            return ActuationCommand::HARD_BRAKE;
        case RiskLevel::MEDIUM:
            return ActuationCommand::SLOW_DOWN;
        case RiskLevel::LOW:
            return ActuationCommand::MAINTAIN_DISTANCE;
        default:
            return ActuationCommand::NONE;
    }
}

std::vector<CollisionWarning> MecCollisionDetector::PerformCollisionCheck()
{
    m_stats.checksPerformed++;
    std::vector<CollisionWarning> warnings;
    
    // ARCHITECTURE CHANGE: Use NDN-received positions instead of direct vehicleStatuses
    // This enables realistic V2I latency measurement (MEC queries via NDN Interest/Data)
    auto& ndnCache = v2x::ndn_cache::NdnPositionCache::GetInstance();
    auto ndnPositions = ndnCache.GetAllPositions(true);  // Include stale for continuity
    
    // Collect vehicles in range from NDN-received data
    std::vector<std::string> vehiclesInRange;
    for (const auto& [id, ndnPos] : ndnPositions)
    {
        if (!ndnPos.valid) continue;
        if (ndnPos.x < -500 || ndnPos.y < -500) continue;  // Skip invalid positions
        
        // If NDN data is stale (>500ms), fallback to vehicleStatuses for continuity
        if (ndnPos.IsStale())
        {
            auto it = vehicleStatuses.find(id);
            if (it != vehicleStatuses.end() && it->second.isActive)
            {
                if (IsInRange(it->second.x, it->second.y))
                {
                    vehiclesInRange.push_back(id);
                    ndnCache.RecordStaleDataUsed();
                }
            }
            continue;
        }
        
        // Use fresh NDN-received position
        if (IsInRange(ndnPos.x, ndnPos.y))
        {
            vehiclesInRange.push_back(id);
        }
    }
    
    // Check all pairs (O(n²) but n is small per RSU zone)
    for (size_t i = 0; i < vehiclesInRange.size(); i++)
    {
        for (size_t j = i + 1; j < vehiclesInRange.size(); j++)
        {
            std::string id1 = vehiclesInRange[i];
            std::string id2 = vehiclesInRange[j];
            
            // Get positions from NDN cache
            v2x::ndn_cache::NdnReceivedPosition pos1, pos2;
            bool hasPos1 = ndnCache.GetPosition(id1, pos1);
            bool hasPos2 = ndnCache.GetPosition(id2, pos2);
            
            // Fallback to vehicleStatuses if NDN data is stale
            double v1_x, v1_y, v1_speed, v1_heading;
            double v2_x, v2_y, v2_speed, v2_heading;
            
            if (!hasPos1 || pos1.IsStale())
            {
                auto it = vehicleStatuses.find(id1);
                if (it == vehicleStatuses.end()) continue;
                v1_x = it->second.x;
                v1_y = it->second.y;
                v1_speed = it->second.speed;
                v1_heading = it->second.heading;
            }
            else
            {
                v1_x = pos1.x;
                v1_y = pos1.y;
                v1_speed = pos1.speed;
                v1_heading = pos1.heading;
            }
            
            if (!hasPos2 || pos2.IsStale())
            {
                auto it = vehicleStatuses.find(id2);
                if (it == vehicleStatuses.end()) continue;
                v2_x = it->second.x;
                v2_y = it->second.y;
                v2_speed = it->second.speed;
                v2_heading = it->second.heading;
            }
            else
            {
                v2_x = pos2.x;
                v2_y = pos2.y;
                v2_speed = pos2.speed;
                v2_heading = pos2.heading;
            }
            
            // Calculate distance
            double dx = v1_x - v2_x;
            double dy = v1_y - v2_y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // Skip if too far apart
            if (distance > 100.0) continue;
            
            // Calculate velocity components
            double v1_vx = v1_speed * cos(v1_heading);
            double v1_vy = v1_speed * sin(v1_heading);
            double v2_vx = v2_speed * cos(v2_heading);
            double v2_vy = v2_speed * sin(v2_heading);
            
            // Relative velocity
            double rel_vx = v1_vx - v2_vx;
            double rel_vy = v1_vy - v2_vy;
            
            // Line-of-sight closing rate: projection of relative velocity
            // onto displacement unit vector (Minderhoud & Bovy, 2001)
            // Positive value means vehicles are approaching each other
            double closingRate = (distance > 0.01) ? (rel_vx * dx + rel_vy * dy) / distance : 0.0;
            double closingSpeed = std::sqrt(rel_vx * rel_vx + rel_vy * rel_vy);
            
            // TTC uses closing rate along line of sight, not total relative speed
            double ttc = (closingRate > 0.1) ? (distance / closingRate) : 999.0;
            
            // ==============================================================
            // LATENCY COMPENSATION FOR RESEARCH VALIDITY (March 2026)
            // ==============================================================
            // Position data received via NDN is already "old" by the time we
            // use it. We must subtract the data age from TTC to account for
            // the fact that vehicles have been moving during the RTT latency.
            // This is critical for research validity: safety decisions must
            // reflect the actual worst-case time margin, not the optimistic
            // TTC computed from stale positions.
            //
            // Example: If TTC=2.0s but position data is 50ms old, the actual
            // remaining time is ~1.95s (vehicles moved during the 50ms).
            // ==============================================================
            double pos1AgeMs = 0.0, pos2AgeMs = 0.0;
            if (hasPos1 && pos1.receivedAt > ns3::Seconds(0))
            {
                pos1AgeMs = (ns3::Simulator::Now() - pos1.receivedAt).GetMilliSeconds();
            }
            if (hasPos2 && pos2.receivedAt > ns3::Seconds(0))
            {
                pos2AgeMs = (ns3::Simulator::Now() - pos2.receivedAt).GetMilliSeconds();
            }
            double maxAgeMs = std::max(pos1AgeMs, pos2AgeMs);
            double latencyCompensationSec = maxAgeMs / 1000.0;
            
            // Adjusted TTC accounts for data staleness
            double ttc_adjusted = ttc - latencyCompensationSec;
            if (ttc_adjusted < 0.01) ttc_adjusted = 0.01;  // Minimum positive TTC
            
            // Assess risk using latency-adjusted TTC for conservative safety
            RiskLevel riskLevel = CalculateRiskLevel(ttc_adjusted, distance, closingSpeed);
            
            if (riskLevel != RiskLevel::NONE)
            {
                CollisionWarning warning;
                warning.vehicle1Id = id1;
                warning.vehicle2Id = id2;
                warning.riskLevel = riskLevel;
                warning.recommendedAction = DetermineAction(riskLevel, ttc_adjusted, distance);
                warning.ttc = ttc;                    // Raw geometric TTC
                warning.ttcAdjusted = ttc_adjusted;   // Latency-compensated TTC
                warning.distance = distance;
                warning.closingSpeed = closingSpeed;
                warning.positionAgeMs = maxAgeMs;     // Position data age used
                warning.detectedByRsu = m_rsuId;
                warning.detectionTime = ns3::Simulator::Now();
                
                warnings.push_back(warning);
                
                m_stats.warningsGenerated++;
                m_stats.warningsByLevel[riskLevel]++;
                m_stats.ttcSamples++;
                // Use adjusted TTC for statistics (reflects actual safety margin)
                m_stats.avgTtcAtWarning = 
                    (m_stats.avgTtcAtWarning * (m_stats.ttcSamples - 1) + ttc_adjusted) / m_stats.ttcSamples;
                
                // Record for Blind Spot analysis (this zone covered)
                BlindSpotTracker::GetInstance().RecordCollisionEvent(
                    (v1_x + v2_x) / 2, (v1_y + v2_y) / 2, true, m_rsuId);
            }
        }
    }
    
    return warnings;
}

std::string MecCollisionDetector::GenerateActuationCommand(const CollisionWarning& warning)
{
    std::string actionStr;
    switch (warning.recommendedAction)
    {
        case ActuationCommand::EMERGENCY_BRAKE: actionStr = "EMERGENCY_BRAKE"; break;
        case ActuationCommand::HARD_BRAKE: actionStr = "HARD_BRAKE"; break;
        case ActuationCommand::SLOW_DOWN: actionStr = "SLOW_DOWN"; break;
        case ActuationCommand::MAINTAIN_DISTANCE: actionStr = "MAINTAIN_DISTANCE"; break;
        case ActuationCommand::EVADE_LEFT: actionStr = "EVADE_LEFT"; break;
        case ActuationCommand::EVADE_RIGHT: actionStr = "EVADE_RIGHT"; break;
        default: actionStr = "NONE"; break;
    }
    
    std::string riskStr;
    switch (warning.riskLevel)
    {
        case RiskLevel::CRITICAL: riskStr = "CRITICAL"; break;
        case RiskLevel::HIGH: riskStr = "HIGH"; break;
        case RiskLevel::MEDIUM: riskStr = "MEDIUM"; break;
        case RiskLevel::LOW: riskStr = "LOW"; break;
        default: riskStr = "NONE"; break;
    }
    
    // Get vehicle positions and OMNeT++ IDs from vehicleStatuses for the command
    double v1_x = 0, v1_y = 0, v2_x = 0, v2_y = 0;
    std::string targetOmnetId;
    std::string otherOmnetId;
    auto it1 = vehicleStatuses.find(warning.vehicle1Id);
    if (it1 != vehicleStatuses.end())
    {
        v1_x = it1->second.x;
        v1_y = it1->second.y;
        targetOmnetId = it1->second.omnetId;
    }
    auto it2 = vehicleStatuses.find(warning.vehicle2Id);
    if (it2 != vehicleStatuses.end())
    {
        v2_x = it2->second.x;
        v2_y = it2->second.y;
        otherOmnetId = it2->second.omnetId;
    }

    // Fallback to reverse mapping if omnetId wasn't captured
    if (targetOmnetId.empty()) targetOmnetId = GetOmnetVehicleId(warning.vehicle1Id);
    if (otherOmnetId.empty()) otherOmnetId = GetOmnetVehicleId(warning.vehicle2Id);
    if (targetOmnetId.empty()) targetOmnetId = warning.vehicle1Id;
    if (otherOmnetId.empty()) otherOmnetId = warning.vehicle2Id;

    // ========================================================================
    // SAFETY MESSAGE DELIVERY TRACKING - Record message sent to vehicle 1
    // ========================================================================
    std::string messageId = GenerateSafetyMessageId(warning.vehicle1Id, warning.vehicle1Id);
    RecordSafetyMessageSent(messageId, warning.vehicle1Id, warning.vehicle1Id, riskStr, actionStr);
    m_stats.messagesSent++;

    // Generate safety_command in OMNeT++ expected format
    // Send command to BOTH vehicles involved in the collision risk
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    
    // Format matches OMNeT++ socketInterface.cc expectation for safety_command
    // Include message_id for delivery tracking
    ss << "{\"message_type\":\"safety_command\","
       << "\"timestamp\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"message_id\":\"" << messageId << "\","
       << "\"payload\":{"
       << "\"target_vehicle_id\":\"" << targetOmnetId << "\","
       << "\"source_type\":\"rsu\","
       << "\"action\":\"" << actionStr << "\","
       << "\"risk_level\":\"" << riskStr << "\","
       << "\"accident_data\":{"
       << "\"accident_id\":\"collision_" << warning.vehicle1Id << "_" << warning.vehicle2Id << "_" << (int)ns3::Simulator::Now().GetSeconds() << "\","
       << "\"origin_time\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"lane_id\":\"\","
       << "\"pos_x\":" << v1_x << ","
       << "\"pos_y\":" << v1_y << ","
       << "\"crashed_vehicle_id\":\"" << otherOmnetId << "\","
       << "\"other_vehicle_id\":\"" << otherOmnetId << "\","
       << "\"ttc\":" << warning.ttc << ","
       << "\"ttc_adjusted\":" << warning.ttcAdjusted << ","
       << "\"position_age_ms\":" << warning.positionAgeMs << ","
       << "\"distance\":" << warning.distance << ","
       << "\"closing_speed\":" << warning.closingSpeed << ","
       << "\"detected_by\":\"" << m_rsuId << "\""
       << "}"
       << "}"
       << "}";
    
    m_stats.actuationsTriggered++;
    
    return ss.str();
}

std::string MecCollisionDetector::GenerateActuationCommandForVehicle2(const CollisionWarning& warning)
{
    std::string actionStr;
    switch (warning.recommendedAction)
    {
        case ActuationCommand::EMERGENCY_BRAKE: actionStr = "EMERGENCY_BRAKE"; break;
        case ActuationCommand::HARD_BRAKE: actionStr = "HARD_BRAKE"; break;
        case ActuationCommand::SLOW_DOWN: actionStr = "SLOW_DOWN"; break;
        case ActuationCommand::MAINTAIN_DISTANCE: actionStr = "MAINTAIN_DISTANCE"; break;
        default: actionStr = "SLOW_DOWN"; break;
    }
    
    std::string riskStr;
    switch (warning.riskLevel)
    {
        case RiskLevel::CRITICAL: riskStr = "CRITICAL"; break;
        case RiskLevel::HIGH: riskStr = "HIGH"; break;
        case RiskLevel::MEDIUM: riskStr = "MEDIUM"; break;
        default: riskStr = "LOW"; break;
    }
    
    double v2_x = 0, v2_y = 0;
    std::string targetOmnetId;
    std::string otherOmnetId;
    auto it2 = vehicleStatuses.find(warning.vehicle2Id);
    if (it2 != vehicleStatuses.end())
    {
        v2_x = it2->second.x;
        v2_y = it2->second.y;
        targetOmnetId = it2->second.omnetId;
    }
    auto it1 = vehicleStatuses.find(warning.vehicle1Id);
    if (it1 != vehicleStatuses.end())
    {
        otherOmnetId = it1->second.omnetId;
    }
    
    if (targetOmnetId.empty()) targetOmnetId = GetOmnetVehicleId(warning.vehicle2Id);
    if (otherOmnetId.empty()) otherOmnetId = GetOmnetVehicleId(warning.vehicle1Id);
    if (targetOmnetId.empty()) targetOmnetId = warning.vehicle2Id;
    if (otherOmnetId.empty()) otherOmnetId = warning.vehicle1Id;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{\"message_type\":\"safety_command\","
       << "\"timestamp\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"payload\":{"
       << "\"target_vehicle_id\":\"" << targetOmnetId << "\","
       << "\"source_type\":\"rsu\","
       << "\"action\":\"" << actionStr << "\","
       << "\"risk_level\":\"" << riskStr << "\","
       << "\"accident_data\":{"
       << "\"accident_id\":\"collision_" << warning.vehicle2Id << "_" << warning.vehicle1Id << "_" << (int)ns3::Simulator::Now().GetSeconds() << "\","
       << "\"origin_time\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"lane_id\":\"\","
       << "\"pos_x\":" << v2_x << ","
       << "\"pos_y\":" << v2_y << ","
       << "\"crashed_vehicle_id\":\"" << otherOmnetId << "\","
       << "\"other_vehicle_id\":\"" << otherOmnetId << "\","
       << "\"ttc\":" << warning.ttc << ","
       << "\"distance\":" << warning.distance << ","
       << "\"closing_speed\":" << warning.closingSpeed << ","
       << "\"detected_by\":\"" << m_rsuId << "\""
       << "}"
       << "}"
       << "}";
    
    return ss.str();
}

// ============================================================================
// BLIND SPOT TRACKER IMPLEMENTATION
// ============================================================================

BlindSpotTracker& BlindSpotTracker::GetInstance()
{
    static BlindSpotTracker instance;
    return instance;
}

void BlindSpotTracker::SetRsuStatus(const std::string& rsuId, bool isActive)
{
    if (isActive)
    {
        m_metrics.activeRsus.insert(rsuId);
        m_metrics.inactiveRsus.erase(rsuId);
    }
    else
    {
        m_metrics.inactiveRsus.insert(rsuId);
        m_metrics.activeRsus.erase(rsuId);
    }
}

void BlindSpotTracker::RecordCollisionEvent(double x, double y, bool wasDetected,
                                            const std::string& detectedByRsu)
{
    m_metrics.totalCollisionEvents++;
    
    if (wasDetected)
    {
        m_metrics.eventsInCoveredZones++;
        if (!detectedByRsu.empty())
        {
            m_metrics.eventsPerZone[detectedByRsu]++;
        }
    }
    else
    {
        m_metrics.eventsInBlindSpots++;
    }
    
    // Update ratio
    if (m_metrics.totalCollisionEvents > 0)
    {
        m_metrics.blindSpotRatio = (double)m_metrics.eventsInBlindSpots / 
                                   m_metrics.totalCollisionEvents * 100.0;
    }
}

void BlindSpotTracker::PrintSummary() const
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Blind Spot Analysis (NFV Cost vs Safety)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total Collision Events:   " << m_metrics.totalCollisionEvents << std::endl;
    std::cout << "Events in Covered Zones:  " << m_metrics.eventsInCoveredZones << std::endl;
    std::cout << "Events in Blind Spots:    " << m_metrics.eventsInBlindSpots << std::endl;
    std::cout << "Blind Spot Ratio:         " << std::fixed << std::setprecision(1) 
              << m_metrics.blindSpotRatio << "%" << std::endl;
    std::cout << "Active RSUs:              " << m_metrics.activeRsus.size() << std::endl;
    std::cout << "Inactive RSUs (scaled-in):" << m_metrics.inactiveRsus.size() << std::endl;
    
    if (m_metrics.blindSpotRatio > 10.0)
    {
        std::cout << "⚠️  WARNING: High blind spot ratio indicates NFV over-optimization!" << std::endl;
    }
    else if (m_metrics.blindSpotRatio < 1.0)
    {
        std::cout << "✅ Excellent coverage - safety maintained with current NFV allocation" << std::endl;
    }
    
    std::cout << "========================================\n" << std::endl;
}

// ============================================================================
// COLLISION AVOIDANCE MANAGER IMPLEMENTATION
// ============================================================================

CollisionAvoidanceManager& CollisionAvoidanceManager::GetInstance()
{
    static CollisionAvoidanceManager instance;
    return instance;
}

void CollisionAvoidanceManager::RegisterRsuDetector(const std::string& rsuId, double x, double y,
                                                    double sensorRadius)
{
    if (m_detectors.find(rsuId) == m_detectors.end())
    {
        m_detectors.emplace(rsuId, MecCollisionDetector(rsuId, sensorRadius));
        m_detectors.at(rsuId).SetPosition(x, y);
        
        BlindSpotTracker::GetInstance().SetRsuStatus(rsuId, true);
        
        std::cout << "🛡️  [MEC] Registered Collision Detector: " << rsuId 
                  << " at (" << x << ", " << y << ") radius=" << sensorRadius << "m" << std::endl;
    }
}

void CollisionAvoidanceManager::UpdateRsuPosition(const std::string& rsuId, double x, double y)
{
    auto it = m_detectors.find(rsuId);
    if (it != m_detectors.end())
    {
        it->second.SetPosition(x, y);
    }
}

std::vector<std::string> CollisionAvoidanceManager::PerformGlobalCollisionCheck()
{
    std::vector<std::string> commands;
    
    for (auto& [rsuId, detector] : m_detectors)
    {
        auto warnings = detector.PerformCollisionCheck();
        
        for (const auto& warning : warnings)
        {
            // Dedup: don't generate same warning within 1 second
            std::string warningKey = warning.vehicle1Id + "_" + warning.vehicle2Id;
            if (m_recentWarnings.find(warningKey) != m_recentWarnings.end())
            {
                continue;  // Already warned recently
            }
            
            // Only generate commands for HIGH or CRITICAL risks
            if (warning.riskLevel == RiskLevel::HIGH || 
                warning.riskLevel == RiskLevel::CRITICAL)
            {
                std::string cmd = detector.GenerateActuationCommand(warning);
                commands.push_back(cmd);
                m_pendingActuations.push_back(cmd);
                m_recentWarnings.insert(warningKey);
                
                // Log to console
                std::string riskStr = (warning.riskLevel == RiskLevel::CRITICAL) ? "CRITICAL" : "HIGH";
                std::cout << "🚨 [MEC-" << rsuId << "] " << riskStr << " RISK: "
                          << warning.vehicle1Id << " <-> " << warning.vehicle2Id
                          << " | TTC: " << std::fixed << std::setprecision(1) << warning.ttc << "s"
                          << " | Dist: " << (int)warning.distance << "m"
                          << " | Action: " << cmd.substr(cmd.find("action") + 10, 15) << "..."
                          << std::endl;
            }
        }
    }
    
    // Clear old warnings (simple approach - clear every 100 checks)
    static int checkCount = 0;
    if (++checkCount >= 100)
    {
        m_recentWarnings.clear();
        checkCount = 0;
    }
    
    return commands;
}

CollisionDetectorStats CollisionAvoidanceManager::GetAggregatedStats() const
{
    CollisionDetectorStats aggregated;
    
    for (const auto& [rsuId, detector] : m_detectors)
    {
        const auto& stats = detector.GetStats();
        aggregated.checksPerformed += stats.checksPerformed;
        aggregated.warningsGenerated += stats.warningsGenerated;
        aggregated.actuationsTriggered += stats.actuationsTriggered;
        aggregated.ttcSamples += stats.ttcSamples;
        
        for (const auto& [level, count] : stats.warningsByLevel)
        {
            aggregated.warningsByLevel[level] += count;
        }
        
        if (stats.ttcSamples > 0)
        {
            aggregated.avgTtcAtWarning = 
                (aggregated.avgTtcAtWarning * (aggregated.ttcSamples - stats.ttcSamples) +
                 stats.avgTtcAtWarning * stats.ttcSamples) / aggregated.ttcSamples;
        }
    }
    
    return aggregated;
}

std::vector<std::string> CollisionAvoidanceManager::GetPendingActuations()
{
    std::vector<std::string> actuations = std::move(m_pendingActuations);
    m_pendingActuations.clear();
    return actuations;
}

void CollisionAvoidanceManager::PrintSummary() const
{
    auto stats = GetAggregatedStats();
    
    // Update global safetyStats for metrics extraction (UC3 compliance)
    safetyStats.collisionWarnings += stats.warningsGenerated;
    safetyStats.slowDownCommands += stats.actuationsTriggered;
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Collision Avoidance Summary (Arch A - UC3)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Checks Performed:        " << stats.checksPerformed << std::endl;
    std::cout << "Warnings Generated:      " << stats.warningsGenerated << std::endl;
    std::cout << "Actuations Triggered:    " << stats.actuationsTriggered << std::endl;
    std::cout << "Avg TTC at Warning:      " << std::fixed << std::setprecision(2) 
              << stats.avgTtcAtWarning << " s" << std::endl;
    std::cout << "Active RSU Detectors:    " << m_detectors.size() << std::endl;
    
    std::cout << "\nWarnings by Risk Level:" << std::endl;
    for (const auto& [level, count] : stats.warningsByLevel)
    {
        std::string levelStr;
        switch (level)
        {
            case RiskLevel::CRITICAL: levelStr = "CRITICAL"; break;
            case RiskLevel::HIGH: levelStr = "HIGH"; break;
            case RiskLevel::MEDIUM: levelStr = "MEDIUM"; break;
            case RiskLevel::LOW: levelStr = "LOW"; break;
            default: levelStr = "NONE"; break;
        }
        std::cout << "  " << levelStr << ": " << count << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
    
    // Also print Blind Spot analysis
    BlindSpotTracker::GetInstance().PrintSummary();
}

} // namespace safety
} // namespace v2x

// ============================================================================
// SAFETY MESSAGE DELIVERY TRACKING IMPLEMENTATION
// ============================================================================

namespace v2x {
namespace safety {

std::string GenerateSafetyMessageId(const std::string& sourceVehicle, 
                                     const std::string& targetVehicle)
{
    uint64_t counter = g_safetyMessageCounter.fetch_add(1);
    std::stringstream ss;
    ss << "warn_" << sourceVehicle << "_" << targetVehicle << "_" << counter;
    return ss.str();
}

void RecordSafetyMessageSent(const std::string& messageId,
                              const std::string& sourceVehicle,
                              const std::string& targetVehicle,
                              const std::string& warningType,
                              const std::string& action)
{
    SafetyMessageDelivery delivery;
    delivery.messageId = messageId;
    delivery.sourceVehicle = sourceVehicle;
    delivery.targetVehicle = targetVehicle;
    delivery.sentTime = ns3::Simulator::Now();
    delivery.receivedTime = ns3::Time(0);
    delivery.delivered = false;
    delivery.deliveryLatencyMs = 0.0;
    delivery.warningType = warningType;
    delivery.action = action;
    
    safetyMessageTracking[messageId] = delivery;
    totalSafetyMessagesSent++;
    
    // Also update QoS metrics
    // (collision_warning category tracked in metrics_collector)
}

void RecordSafetyMessageDelivered(const std::string& messageId)
{
    auto it = safetyMessageTracking.find(messageId);
    if (it == safetyMessageTracking.end())
    {
        // Message not found - might be a duplicate or late delivery
        return;
    }
    
    if (it->second.delivered)
    {
        // Already marked as delivered
        return;
    }
    
    it->second.receivedTime = ns3::Simulator::Now();
    it->second.delivered = true;
    it->second.deliveryLatencyMs = 
        (it->second.receivedTime - it->second.sentTime).GetMilliSeconds();
    
    totalSafetyMessagesDelivered++;
}

bool IsSafetyMessageTracked(const std::string& messageId)
{
    return safetyMessageTracking.find(messageId) != safetyMessageTracking.end();
}

SafetyDeliveryStats GetSafetyDeliveryStats()
{
    SafetyDeliveryStats stats;
    stats.messagesSent = totalSafetyMessagesSent;
    stats.messagesDelivered = totalSafetyMessagesDelivered;
    
    if (stats.messagesSent > 0)
    {
        stats.deliveryRatio = 
            (double)stats.messagesDelivered / stats.messagesSent * 100.0;
    }
    
    // Calculate latency statistics from delivered messages
    double totalLatency = 0.0;
    uint64_t deliveredCount = 0;
    
    for (const auto& [id, delivery] : safetyMessageTracking)
    {
        if (delivery.delivered)
        {
            totalLatency += delivery.deliveryLatencyMs;
            deliveredCount++;
            
            if (delivery.deliveryLatencyMs < stats.minDeliveryLatencyMs)
            {
                stats.minDeliveryLatencyMs = delivery.deliveryLatencyMs;
            }
            if (delivery.deliveryLatencyMs > stats.maxDeliveryLatencyMs)
            {
                stats.maxDeliveryLatencyMs = delivery.deliveryLatencyMs;
            }
        }
    }
    
    if (deliveredCount > 0)
    {
        stats.avgDeliveryLatencyMs = totalLatency / deliveredCount;
    }
    
    // Reset min if no delivered messages
    if (deliveredCount == 0)
    {
        stats.minDeliveryLatencyMs = 0.0;
    }
    
    return stats;
}

void PrintSafetyDeliverySummary()
{
    auto stats = GetSafetyDeliveryStats();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Safety Message Delivery Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Messages Sent:           " << stats.messagesSent << std::endl;
    std::cout << "Messages Delivered:      " << stats.messagesDelivered << std::endl;
    std::cout << "Delivery Ratio:          " << std::fixed << std::setprecision(1) 
              << stats.deliveryRatio << "%" << std::endl;
    
    if (stats.messagesDelivered > 0)
    {
        std::cout << "Avg Delivery Latency:    " << std::fixed << std::setprecision(2) 
                  << stats.avgDeliveryLatencyMs << " ms" << std::endl;
        std::cout << "Min Delivery Latency:    " << stats.minDeliveryLatencyMs << " ms" << std::endl;
        std::cout << "Max Delivery Latency:    " << stats.maxDeliveryLatencyMs << " ms" << std::endl;
    }
    
    if (stats.deliveryRatio < 99.0 && stats.messagesSent > 0)
    {
        std::cout << "⚠️  WARNING: Delivery ratio below 99% - safety system may have gaps!" << std::endl;
    }
    else if (stats.messagesSent > 0)
    {
        std::cout << "✅ Excellent delivery ratio for safety messages" << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
}

} // namespace safety
} // namespace v2x
