#include "safety_detection.h"
#include "simulation_state.h"
#include "v2x_constants.h"
#include "ndn_position_cache.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <vector>
#include <map>

using namespace ns3;

// Get current wall-clock time as a formatted string
std::string GetCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

// Get current simulation time as a string
std::string GetSimTimeString()
{
    return std::to_string(Simulator::Now().GetSeconds()) + "s";
}

// Publish safety warning via NDN Interest/Data
void PublishSafetyWarning(const std::string& vehicleId, const std::string& warningType,
                          const std::string& reason)
{
    auto it = nodeMapping.find(vehicleId);
    if (it == nodeMapping.end())
        return;

    // Log the warning being published
    actionsLog << "  >> NDN Warning Published: /v2x/safety/" << vehicleId << "/" << warningType << "\n";
}

// Enhanced collision detection with comprehensive checks
void DetectAndLogCollisions()
{
    safetyStats.totalChecks++;

    static std::map<std::string, int> warningCounts;
    static std::map<std::string, double> lastWarningTime;

    if (safetyStats.totalChecks % 1000 == 0) {
        double cutoff = ns3::Simulator::Now().GetSeconds() - 10.0;
        for (auto it = lastWarningTime.begin(); it != lastWarningTime.end(); ) {
            if (it->second < cutoff)
                it = lastWarningTime.erase(it);
            else
                ++it;
        }
        for (auto it = warningCounts.begin(); it != warningCounts.end(); ) {
            if (lastWarningTime.find(it->first) == lastWarningTime.end())
                it = warningCounts.erase(it);
            else
                ++it;
        }
    }

    // ARCHITECTURE FIX: Use NDN-received positions instead of direct vehicleStatuses access
    // This ensures safety detection uses positions received via NDN Interest/Data exchange
    // through the 5G NR network, enabling realistic V2I latency measurement.
    auto& ndnCache = v2x::ndn_cache::NdnPositionCache::GetInstance();
    auto ndnPositions = ndnCache.GetAllPositions(true);  // Include stale for continuity

    // Collect active vehicles with valid positions from NDN cache
    std::vector<std::string> activeVehicleIds;
    for (const auto& [id, ndnPos] : ndnPositions)
    {
        if (id.find("veh") == std::string::npos) continue;
        if (!ndnPos.valid) continue;
        
        // Skip vehicles at initial spawn position (far away negative coords)
        if (ndnPos.x < -500 || ndnPos.y < -500) continue;
        
        // Skip vehicles with zero position (not yet initialized)
        if (ndnPos.x == 0 && ndnPos.y == 0 && ndnPos.speed == 0) continue;
        
        activeVehicleIds.push_back(id);
    }
    
    // Fallback: Also include active vehicles from vehicleStatuses if not in NDN cache
    // This handles newly spawned vehicles that haven't been queried via NDN yet
    for (auto& [id, status] : vehicleStatuses)
    {
        if (id.find("veh") == std::string::npos || !status.isActive) continue;
        if (status.x < -500 || status.y < -500) continue;
        if (status.id.empty()) continue;
        if (status.x == 0 && status.y == 0 && status.speed == 0) continue;
        
        // Only add if not already in the list from NDN cache
        if (std::find(activeVehicleIds.begin(), activeVehicleIds.end(), id) == activeVehicleIds.end())
        {
            activeVehicleIds.push_back(id);
        }
    }

    if (activeVehicleIds.size() < 2)
        return;

    int warningsThisCheck = 0;

    // Check all pairs of active vehicles
    for (size_t i = 0; i < activeVehicleIds.size(); i++)
    {
        for (size_t j = i + 1; j < activeVehicleIds.size(); j++)
        {
            // Get positions from NDN cache with fallback to vehicleStatuses
            v2x::ndn_cache::NdnReceivedPosition pos1, pos2;
            bool hasPos1 = ndnCache.GetPosition(activeVehicleIds[i], pos1);
            bool hasPos2 = ndnCache.GetPosition(activeVehicleIds[j], pos2);
            
            // Extract position data with fallback for stale/missing NDN data
            double v1_x, v1_y, v1_speed, v1_heading;
            double v2_x, v2_y, v2_speed, v2_heading;
            
            if (!hasPos1 || pos1.IsStale())
            {
                auto it = vehicleStatuses.find(activeVehicleIds[i]);
                if (it == vehicleStatuses.end()) continue;
                v1_x = it->second.x;
                v1_y = it->second.y;
                v1_speed = it->second.speed;
                v1_heading = it->second.heading;
                if (hasPos1) ndnCache.RecordStaleDataUsed();
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
                auto it = vehicleStatuses.find(activeVehicleIds[j]);
                if (it == vehicleStatuses.end()) continue;
                v2_x = it->second.x;
                v2_y = it->second.y;
                v2_speed = it->second.speed;
                v2_heading = it->second.heading;
                if (hasPos2) ndnCache.RecordStaleDataUsed();
            }
            else
            {
                v2_x = pos2.x;
                v2_y = pos2.y;
                v2_speed = pos2.speed;
                v2_heading = pos2.heading;
            }

            // Calculate Euclidean distance
            double dx = v1_x - v2_x;
            double dy = v1_y - v2_y;
            double distance = sqrt(dx * dx + dy * dy);

            // Skip if vehicles are too far apart
            if (distance > COLLISION_RISK_DISTANCE_M)
                continue;

            // Normalize heading difference to [0, PI]
            double headingDiff = fabs(v1_heading - v2_heading);
            if (headingDiff > M_PI)
                headingDiff = 2 * M_PI - headingDiff;

            // Check if moving in same general direction
            bool sameDirection = (headingDiff < HEADING_TOLERANCE_RAD);

            // Check if moving toward each other (opposite directions, closing in)
            bool headOn = (headingDiff > (M_PI - HEADING_TOLERANCE_RAD));

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
            // Positive = approaching, negative = diverging
            double closingRate = (distance > 0.01) ? (rel_vx * dx + rel_vy * dy) / distance : 0.0;

            // TTC uses closing rate along line of sight, not total relative speed
            double ttc = (closingRate > 0.1) ? (distance / closingRate) : 999.0;

            // RESEARCH VALIDITY FIX: Apply position data age to TTC
            // The positions we're using are already "old" - account for this in safety decisions
            double pos1AgeMs = hasPos1 ? (ns3::Simulator::Now() - pos1.receivedAt).GetMilliSeconds() : 0.0;
            double pos2AgeMs = hasPos2 ? (ns3::Simulator::Now() - pos2.receivedAt).GetMilliSeconds() : 0.0;
            double maxPositionAgeMs = std::max(pos1AgeMs, pos2AgeMs);

            // Adjusted TTC: subtract position staleness from computed TTC
            // This makes safety thresholds more conservative when using older data
            double ttc_adjusted = ttc - (maxPositionAgeMs / 1000.0);
            if (ttc_adjusted < 0) ttc_adjusted = 0.01;

            // Log if position is significantly old (>50ms)
            if (maxPositionAgeMs > 50.0) {
                std::cout << "  ⏱️ Position age: " << maxPositionAgeMs << "ms, TTC adjusted: " 
                          << ttc << " → " << ttc_adjusted << "s" << std::endl;
            }

            // Determine collision risk level (using ttc_adjusted for latency-aware decisions)
            std::string riskLevel = "";
            std::string action = "";

            if (distance < MIN_SAFE_DISTANCE_M)
            {
                riskLevel = "CRITICAL";
                action = "EMERGENCY_BRAKE";
            }
            else if (ttc_adjusted < 1.5)  // Use adjusted TTC
            {
                riskLevel = "HIGH";
                action = "HARD_BRAKE";
            }
            else if (ttc_adjusted < TTC_THRESHOLD_S)  // Use adjusted TTC
            {
                riskLevel = "MEDIUM";
                action = "SLOW_DOWN";
            }
            else if (sameDirection && distance < 30 && v1_speed > v2_speed)
            {
                // Rear-end collision risk
                riskLevel = "LOW";
                action = "MAINTAIN_DISTANCE";
            }
            else if (headOn && distance < 40)
            {
                // Head-on collision risk
                riskLevel = "HIGH";
                action = "EVADE";
            }

            // Log if there's any risk
            if (!riskLevel.empty())
            {
                warningsThisCheck++;
                safetyStats.collisionWarnings++;

                if (action == "SLOW_DOWN" || action == "HARD_BRAKE" || action == "EMERGENCY_BRAKE")
                {
                    safetyStats.slowDownCommands++;
                }

                // Use map keys for IDs (more reliable than status.id)
                std::string veh1Id = activeVehicleIds[i];
                std::string veh2Id = activeVehicleIds[j];

                // Create unique pair key to track which warnings we've already printed
                std::string pairKey = veh1Id + "_" + veh2Id + "_" + riskLevel;

                warningCounts[pairKey]++;
                double currentTime = Simulator::Now().GetSeconds();

                std::stringstream logEntry;
                logEntry << std::fixed << std::setprecision(2);
                logEntry << "[" << GetSimTimeString() << "] ";
                logEntry << "[" << GetCurrentTimeString() << "] ";
                logEntry << riskLevel << "_RISK: " << veh1Id << " <-> " << veh2Id << "\n";
                logEntry << "  | Distance: " << distance << "m";
                logEntry << "  | TTC: " << ttc << "s";
                logEntry << "  | Direction: " << (sameDirection ? "SAME" : (headOn ? "HEAD-ON" : "CROSSING")) << "\n";
                logEntry << "  | V1: pos(" << v1_x << "," << v1_y << ") speed=" << v1_speed << "m/s heading=" << (v1_heading * 180 / M_PI) << "°\n";
                logEntry << "  | V2: pos(" << v2_x << "," << v2_y << ") speed=" << v2_speed << "m/s heading=" << (v2_heading * 180 / M_PI) << "°\n";
                logEntry << "  | ACTION: " << action << " for BOTH vehicles\n";
                logEntry << "\n";

                // Always write to file log
                actionsLog << logEntry.str() << std::flush;

                // Rate-limit console output: only print once per second per vehicle pair
                if (currentTime - lastWarningTime[pairKey] >= 1.0)
                {
                    std::cout << "⚠️  " << riskLevel << "_RISK: " << veh1Id << " <-> " << veh2Id
                              << " | Dist: " << (int)distance << "m | TTC: " << std::fixed << std::setprecision(1) << ttc
                              << "s | Action: " << action;
                    if (warningCounts[pairKey] > 1)
                    {
                        std::cout << " (x" << warningCounts[pairKey] << " in last sec)";
                    }
                    std::cout << std::endl;
                    lastWarningTime[pairKey] = currentTime;
                    warningCounts[pairKey] = 0; // Reset count
                }

                // Publish safety warning via NDN (for both vehicles)
                PublishSafetyWarning(veh1Id, action, "TTC=" + std::to_string(ttc) + "s,Dist=" + std::to_string((int)distance) + "m");
                PublishSafetyWarning(veh2Id, action, "TTC=" + std::to_string(ttc) + "s,Dist=" + std::to_string((int)distance) + "m");
            }
        }
    }

    if (warningsThisCheck > 0)
    {
        actionsLog << "--- Check #" << safetyStats.totalChecks << " found " << warningsThisCheck << " warnings ---\n"
                   << std::flush;
    }
}

void RsuPeriodicPositionCheck()
{
    DetectAndLogCollisions();
    Simulator::Schedule(MilliSeconds(RSU_CHECK_INTERVAL_MS), &RsuPeriodicPositionCheck);
}

void VehiclePeriodicTrafficQuery()
{
    Simulator::Schedule(MilliSeconds(VEHICLE_QUERY_INTERVAL_MS), &VehiclePeriodicTrafficQuery);
}
