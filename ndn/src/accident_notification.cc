/**
 * @file accident_notification.cc
 * @brief Radius-based Accident Notification System
 *
 * This module implements the notification of nearby vehicles when an accident occurs.
 * It uses radius-based proximity detection to:
 * 1. Find all vehicles within a configurable radius of the accident
 * 2. Send NDN Interest packets to notify them (on behalf of each vehicle; vehicle
 *    context comes from OMNeT++)
 * 3. Track notification metrics and responder (MEC / RSU / local cache) for QoS
 *
 * Proactive caching (MANO decision from OMNeT++) governs WHERE accident data is
 * cached (MEC origin and/or target RSUs). The actual REQUEST for that data is
 * issued here on behalf of each vehicle; the response is served from wherever
 * the data was proactively cached (local CS, RSU, or MEC).
 *
 * ============================================================================
 * ARCHITECTURE NOTES (March 2026):
 * ============================================================================
 * 1. FindVehiclesInRadius() uses NDN position cache (NdnPositionCache) for
 *    vehicles discovered via V2I NDN queries, with fallback to nodeMapping
 *    for vehicles with known NS-3 mobility positions.
 *
 * 2. SendNdnNotification() checks local CS and RSU CS state to determine
 *    responder attribution (local CS / RSU cache / MEC origin).
 *
 * 3. Initial latency reporting uses live measured sources only:
 *    - calibrated-from-live-metrics (NdnPositionCache mean RTT), or
 *    - pending (unknown until per-Interest RTT callback arrives).
 *    Per-Interest trace callbacks promote latency status to measured.
 * ============================================================================
 */

#include "ns3/ndnSIM-module.h"
#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder.hpp"
#include "ns3/ndnSIM/helper/ndn-app-helper.hpp"

#include "accident_notification.h"
#include "node_management.h"
#include "simulation_state.h"
#include "ndn_position_cache.h"
#include "v2x_constants.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/security/signing-info.hpp>

namespace {

NS_LOG_COMPONENT_DEFINE("AccidentNotification");

// ============================================================================
// Per-Interest Latency Tracking (March 2026)
// ============================================================================
// Track pending Interests by name to correlate send time with Data arrival.
// This enables actual RTT measurement instead of relying on calibrated estimates.

struct PendingInterestRecord
{
    std::string interestName;
    ns3::Time sendTime;
    std::string vehicleId;
    v2x::notification::AccidentResponseRecord* recordPtr;  // Pointer to the actual record to update
};

// Map from Interest name to pending record
static std::map<std::string, PendingInterestRecord> g_pendingAccidentInterests;

bool HasValidLatencySample(double latencyMs);
std::string FormatLatencyForLog(double latencyMs, const std::string& latencyStatus);

/**
 * Callback when accident notification Interest is sent.
 * Records the send time for later RTT calculation.
 */
void OnAccidentNotificationInterestSent(
    v2x::notification::AccidentResponseRecord* record,
    const std::shared_ptr<const ::ndn::Interest>& interest)
{
    if (!record) return;
    
    std::string name = interest->getName().toUri();
    record->interestSendTime = ns3::Simulator::Now();
    
    // Store in pending map for later lookup
    PendingInterestRecord pending;
    pending.interestName = name;
    pending.sendTime = record->interestSendTime;
    pending.vehicleId = record->vehicleId;
    pending.recordPtr = record;
    
    g_pendingAccidentInterests[name] = pending;
    
    NS_LOG_DEBUG("ACCIDENT_NOTIFY: Interest sent for " << record->vehicleId 
                 << " | Name: " << name 
                 << " | Time: " << record->interestSendTime.GetMilliSeconds() << "ms");
}

/**
 * Callback when accident notification Data is received.
 * Calculates actual RTT and updates the response record.
 */
void OnAccidentNotificationDataReceived(const std::shared_ptr<const ::ndn::Data>& data)
{
    if (!data) return;
    
    std::string dataName = data->getName().toUri();
    
    // Find the corresponding Interest in pending map
    auto it = g_pendingAccidentInterests.find(dataName);
    if (it == g_pendingAccidentInterests.end())
    {
        // Name doesn't match exactly; try to find by prefix match
        for (auto& [pendingName, pending] : g_pendingAccidentInterests)
        {
            if (dataName.find(pending.interestName) != std::string::npos)
            {
                it = g_pendingAccidentInterests.find(pendingName);
                break;
            }
        }
        
        if (it == g_pendingAccidentInterests.end())
        {
            return;  // No matching Interest found
        }
    }
    
    PendingInterestRecord& pending = it->second;
    ns3::Time receiveTime = ns3::Simulator::Now();
    ns3::Time rttTime = receiveTime - pending.sendTime;
    double rttMs = rttTime.GetMilliSeconds();
    
    // Update the response record with actual RTT
    if (pending.recordPtr)
    {
        pending.recordPtr->dataReceiveTime = receiveTime;
        pending.recordPtr->actualRttMs = rttMs;
        pending.recordPtr->latencyMeasured = HasValidLatencySample(rttMs);

        if (pending.recordPtr->latencyMeasured)
        {
            pending.recordPtr->measuredLatencyMs = rttMs;
        }

        NS_LOG_DEBUG("ACCIDENT_NOTIFY: Data received for " << pending.vehicleId 
                     << " | RTT: " << rttMs << "ms"
                     << " | Latency status: "
                     << (pending.recordPtr->latencyMeasured ? "measured" : "pending"));

        if (actionsLog.is_open())
        {
            actionsLog << "[" << ns3::Simulator::Now().GetSeconds() << "s] "
                       << "ACCIDENT_NOTIFY_RTT: " << pending.vehicleId
                       << " | Interest: " << pending.interestName
                       << " | RTT: "
                       << FormatLatencyForLog(
                              pending.recordPtr->latencyMeasured ? pending.recordPtr->measuredLatencyMs : -1.0,
                              pending.recordPtr->latencyMeasured ? "measured" : "pending")
                       << "\n";
            actionsLog.flush();
        }
    }
    
    // Clean up (prevent memory growth for old entries)
    g_pendingAccidentInterests.erase(it);
}

/**
 * Cleanup old pending Interests that never received Data.
 * Called periodically to prevent map from growing unbounded.
 */
[[maybe_unused]] void CleanupOldPendingInterests()
{
    ns3::Time now = ns3::Simulator::Now();
    ns3::Time timeout = ns3::MilliSeconds(1000);  // 1 second timeout
    
    auto it = g_pendingAccidentInterests.begin();
    while (it != g_pendingAccidentInterests.end())
    {
        if (now - it->second.sendTime > timeout)
        {
            it = g_pendingAccidentInterests.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

std::string SanitizeNameComponent(const std::string& input)
{
    std::string out = input;
    for (char& c : out)
    {
        if (c == '/' || c == ' ')
        {
            c = '_';
        }
    }
    return out;
}

bool HasValidLatencySample(double latencyMs)
{
    return std::isfinite(latencyMs) && latencyMs > 0.0;
}

std::string FormatLatencyForLog(double latencyMs, const std::string& latencyStatus)
{
    std::ostringstream oss;
    if (HasValidLatencySample(latencyMs))
    {
        oss << std::fixed << std::setprecision(1) << latencyMs << "ms";
    }
    else
    {
        oss << "pending";
    }

    oss << " [" << latencyStatus << "]";
    return oss.str();
}

std::string GetUrgencyFromDistance(double distance)
{
    if (distance < 50) return "CRITICAL";
    if (distance < 150) return "HIGH";
    if (distance < 300) return "MEDIUM";
    return "LOW";
}

std::string BuildAccidentAlertName(const v2x::notification::AccidentNotification& notification,
                                   const std::string& targetVehicleId)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "/v2x/alert/accident/"
       << SanitizeNameComponent(notification.accidentId)
       << "/vehicle/" << SanitizeNameComponent(targetVehicleId)
       << "/lane/" << SanitizeNameComponent(notification.roadId)
       << "/x/" << notification.accidentX
       << "/y/" << notification.accidentY;
    return ss.str();
}

std::string FindNearestRsuId(const ns3::Ptr<ns3::Node>& vehicleNode, double* minDistOut = nullptr)
{
    std::string nearestRsu;
    double minRsuDist = std::numeric_limits<double>::max();

    auto mobility = vehicleNode ? vehicleNode->GetObject<ns3::MobilityModel>() : nullptr;
    ns3::Vector vehPos = mobility ? mobility->GetPosition() : ns3::Vector(0, 0, 0);

    for (const auto& [nodeId, node] : nodeMapping)
    {
        if (nodeId.find("rsu") == std::string::npos) continue;
        auto rsuMobility = node->GetObject<ns3::MobilityModel>();
        if (!rsuMobility) continue;

        ns3::Vector rsuPos = rsuMobility->GetPosition();
        double dx = vehPos.x - rsuPos.x;
        double dy = vehPos.y - rsuPos.y;
        double rsuDist = std::sqrt(dx * dx + dy * dy);

        if (rsuDist < minRsuDist)
        {
            minRsuDist = rsuDist;
            nearestRsu = nodeId;
        }
    }

    if (minDistOut)
    {
        *minDistOut = minRsuDist;
    }

    return nearestRsu;
}

} // namespace

namespace v2x {
namespace notification {

// ============================================================================
// ACCIDENT NOTIFICATION MANAGER IMPLEMENTATION
// ============================================================================

AccidentNotificationManager& AccidentNotificationManager::GetInstance()
{
    static AccidentNotificationManager instance;
    return instance;
}

void AccidentNotificationManager::SetNotificationRadius(double radius)
{
    m_notificationRadius = radius;
    std::cout << "📢 [Notification] Radius set to " << radius << " meters" << std::endl;
}

void AccidentNotificationManager::SetNotifyAllVehicles(bool notifyAll)
{
    m_notifyAllVehicles = notifyAll;
    if (notifyAll)
    {
        std::cout << "📢 [Notification] Will notify ALL active vehicles on accident" << std::endl;
    }
}

std::vector<std::string> AccidentNotificationManager::FindVehiclesInRadius(
    double accidentX, double accidentY, double radius)
{
    std::vector<std::string> nearbyVehicles;
    
    // If notifyAllVehicles is set or radius is very large (>5000m), notify everyone
    bool notifyAll = m_notifyAllVehicles || radius > 5000.0;
    
    std::cout << "  🔍 Searching for vehicles (notifyAll=" << (notifyAll ? "true" : "false") 
              << ", vehicleStatuses size=" << vehicleStatuses.size() 
              << ", nodeMapping size=" << nodeMapping.size() << ")" << std::endl;
    
    // First, add all vehicles from nodeMapping (the definitive source)
    for (const auto& [nodeId, node] : nodeMapping)
    {
        // Skip RSUs and MEC
        if (nodeId.find("rsu") != std::string::npos) continue;
        if (nodeId.find("mec") != std::string::npos || nodeId.find("MEC") != std::string::npos) continue;
        
        // Check if it's a vehicle node (veh_* pattern)
        if (nodeId.find("veh_") == std::string::npos) continue;
        
        // Get position from node
        auto mobility = node->GetObject<ns3::MobilityModel>();
        if (!mobility) continue;
        
        ns3::Vector pos = mobility->GetPosition();
        double dx = pos.x - accidentX;
        double dy = pos.y - accidentY;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // If accident position is unknown (0,0), use a default distance
        if (accidentX < 1 && accidentY < 1)
        {
            distance = 100.0;  // Assume 100m for unknown position
        }
        
        if (notifyAll || distance <= radius)
        {
            nearbyVehicles.push_back(nodeId);
            m_vehicleDistances[nodeId] = distance;
        }
    }
    
    // ========================================================================
    // ARCHITECTURE FIX: Use NDN-received positions for realistic V2I
    // ========================================================================
    // Instead of directly reading vehicleStatuses (which bypasses NDN),
    // we use positions received via NDN Interest/Data exchange. This enables
    // realistic V2I latency measurement through the actual 5G NR stack.
    // ========================================================================
    auto& ndnCache = v2x::ndn_cache::NdnPositionCache::GetInstance();
    auto ndnPositions = ndnCache.GetAllPositions(true);  // Include stale positions
    
    for (const auto& [vehicleId, ndnPos] : ndnPositions)
    {
        // Skip if already added from nodeMapping
        if (std::find(nearbyVehicles.begin(), nearbyVehicles.end(), vehicleId) != nearbyVehicles.end())
            continue;
        
        // Skip invalid positions
        if (!ndnPos.valid) continue;
        
        // Skip RSUs - only notify vehicles
        if (vehicleId.find("rsu") != std::string::npos) continue;
        
        // Skip non-vehicle entries
        if (vehicleId.find("veh") == std::string::npos && vehicleId.find("node") == std::string::npos) continue;
        
        double dx = ndnPos.x - accidentX;
        double dy = ndnPos.y - accidentY;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (accidentX < 1 && accidentY < 1)
        {
            distance = 100.0;
        }
        
        if (notifyAll || (distance <= radius && distance >= 0))
        {
            nearbyVehicles.push_back(vehicleId);
            m_vehicleDistances[vehicleId] = distance;
        }
    }
    
    std::cout << "  📋 Found " << nearbyVehicles.size() << " vehicles to notify" << std::endl;
    
    // Sort by distance (closest first)
    std::sort(nearbyVehicles.begin(), nearbyVehicles.end(),
        [this](const std::string& a, const std::string& b) {
            return m_vehicleDistances[a] < m_vehicleDistances[b];
        });
    
    return nearbyVehicles;
}

AccidentNotification AccidentNotificationManager::NotifyNearbyVehicles(
    const std::string& accidentId,
    const std::string& crashedVehicleId,
    double accidentX, double accidentY,
    const std::string& roadId,
    const std::string& severity)
{
    AccidentNotification notification;
    notification.accidentId = accidentId;
    notification.crashedVehicleId = crashedVehicleId;
    notification.accidentX = accidentX;
    notification.accidentY = accidentY;
    notification.roadId = roadId;
    notification.severity = severity;
    notification.timestamp = ns3::Simulator::Now();
    
    // Find all vehicles in radius
    auto nearbyVehicles = FindVehiclesInRadius(accidentX, accidentY, m_notificationRadius);
    
    notification.notifiedVehicles = nearbyVehicles;
    notification.vehiclesInRadius = nearbyVehicles.size();
    
    std::cout << "\n🚨 [ACCIDENT NOTIFICATION] ========================================" << std::endl;
    std::cout << "  Accident ID: " << accidentId << std::endl;
    std::cout << "  Location: (" << std::fixed << std::setprecision(1) 
              << accidentX << ", " << accidentY << ")" << std::endl;
    std::cout << "  Road: " << roadId << std::endl;
    std::cout << "  Severity: " << severity << std::endl;
    std::cout << "  Notification Radius: " << m_notificationRadius << "m" << std::endl;
    std::cout << "  Vehicles Found in Radius: " << nearbyVehicles.size() << std::endl;
    
    // Send alert + NDN pull notifications to each nearby vehicle and track responders
    for (const auto& vehicleId : nearbyVehicles)
    {
        double distance = m_vehicleDistances[vehicleId];

        // Step 1: Send lightweight alert (vehicle learns accident location)
        std::string alertName = BuildAccidentAlertName(notification, vehicleId);
        std::string urgency = GetUrgencyFromDistance(distance);
        std::string alertSource = "MEC";

        auto vehIt = nodeMapping.find(vehicleId);
        if (vehIt != nodeMapping.end())
        {
            std::string nearestRsu = FindNearestRsuId(vehIt->second);
            if (!nearestRsu.empty())
            {
                alertSource = nearestRsu;
            }
        }

        std::cout << "    📣 Vehicle " << vehicleId
                  << " ← [ALERT from " << alertSource << "] " << alertName
                  << " | Distance: " << std::fixed << std::setprecision(0) << distance << "m"
                  << " | Urgency: " << urgency << std::endl;

        if (actionsLog.is_open())
        {
            actionsLog << "[" << ns3::Simulator::Now().GetSeconds() << "s] "
                       << "ACCIDENT_ALERT: " << vehicleId
                       << " ← " << alertSource
                       << " | AlertName: " << alertName
                       << " | AccidentId: " << notification.accidentId
                       << " | Road: " << notification.roadId
                       << " | Pos: (" << std::fixed << std::setprecision(2)
                       << notification.accidentX << ", " << notification.accidentY << ")"
                       << " | Distance: " << std::fixed << std::setprecision(0) << distance << "m"
                       << " | Urgency: " << urgency << "\n";
            actionsLog.flush();
        }

        // Step 2: Vehicle pulls full accident data via NDN Interest
        bool sent = SendNdnNotification(vehicleId, notification, distance);
        
        if (sent)
        {
            notification.notificationsSent++;
            
            // Collect the response record from SendNdnNotification
            notification.responseRecords.push_back(m_latestResponseRecord);
            
            // Count by responder type
            if (m_latestResponseRecord.responderType == "CACHE_HIT")
                notification.cacheHitResponses++;
            else if (m_latestResponseRecord.responderType == "RSU")
                notification.rsuResponses++;
            else if (m_latestResponseRecord.responderType == "MEC")
                notification.mecResponses++;
        }
    }
    
    std::cout << "  Notifications Sent: " << notification.notificationsSent << std::endl;
    std::cout << "  Response Sources:" << std::endl;
    std::cout << "    Local Cache (CS):  " << notification.cacheHitResponses << std::endl;
    std::cout << "    RSU Cache:         " << notification.rsuResponses << std::endl;
    std::cout << "    MEC Origin:        " << notification.mecResponses << std::endl;
    
    // Print per-vehicle response details
    std::cout << "  Per-Vehicle Response Details:" << std::endl;
    for (const auto& record : notification.responseRecords)
    {
        std::cout << "    " << record.vehicleId 
                  << " ← [" << record.responderType << "] " << record.responderId
                  << " | " << std::fixed << std::setprecision(0) << record.distanceToAccident << "m"
                  << " | " << record.urgency << std::endl;
    }
    std::cout << "============================================================\n" << std::endl;
    
    // Update global stats
    m_stats.totalAccidents++;
    m_stats.totalNotificationsSent += notification.notificationsSent;
    m_stats.totalVehiclesInRange += nearbyVehicles.size();
    
    // Store notification record
    m_notifications[accidentId] = notification;
    
    // Track per-vehicle metrics
    g_v2xStats.ndnInterestsTx += notification.notificationsSent;
    safetyStats.collisionWarnings += notification.notificationsSent;
    
    return notification;
}

bool AccidentNotificationManager::SendNdnNotification(
    const std::string& targetVehicleId,
    const AccidentNotification& notification,
    double distance)
{
    // Find the target vehicle node
    auto it = nodeMapping.find(targetVehicleId);
    if (it == nodeMapping.end())
    {
        std::cerr << "    ⚠️  Vehicle " << targetVehicleId << " not found in nodeMapping" << std::endl;
        return false;
    }
    
    ns3::Ptr<ns3::Node> vehicleNode = it->second;
    
    // Get NDN L3 Protocol
    auto ndnL3 = vehicleNode->GetObject<ns3::ndn::L3Protocol>();
    if (!ndnL3)
    {
        std::cerr << "    ⚠️  No NDN L3 Protocol on " << targetVehicleId << std::endl;
        return false;
    }
    
    // Determine urgency based on distance
    std::string urgency = GetUrgencyFromDistance(distance);
    
    // ========================================================================
    // SEND NDN INTEREST THROUGH THE ACTUAL NDN STACK
    // ========================================================================
    // The Interest is forwarded through the vehicle's NDN forwarder, which
    // routes it via FIB entries through the 5G NR stack (PHY/MAC/RLC/PDCP/IP)
    // to the MEC or RSU. The AppDelayTracer captures the real E2E RTT.
    //
    // For cache state attribution (local CS vs RSU vs MEC), we inspect the
    // forwarder CS before sending. Initial latency is taken only from live
    // measured sources (mean V2I RTT when available), otherwise left pending.
    // ========================================================================
    
    std::string interestName = "/v2x/safety/accident/" + notification.accidentId;
    
    // Create NDN Interest packet
    auto interest = std::make_shared<::ndn::Interest>(interestName);
    interest->setCanBePrefix(true);
    interest->setMustBeFresh(false);
    interest->setInterestLifetime(::ndn::time::milliseconds(500));
    
    auto forwarder = ndnL3->getForwarder();
    auto& cs = forwarder->getCs();
    bool localCacheHit = false;

    // Check local CS state for responder attribution
    cs.find(*interest,
        [&localCacheHit](const ::ndn::Interest&, const ::ndn::Data&) {
            localCacheHit = true;
        },
        [&localCacheHit](const ::ndn::Interest&) {
            localCacheHit = false;
        }
    );

    std::string responderId;
    std::string responderType;
    double measuredLatencyMs = -1.0;
    std::string latencyStatus = "pending";

    // Dispatch a real NDN Interest through the vehicle's forwarder
    // This creates an actual ConsumerCbr app that sends an Interest through
    // the full NDN + 5G NR stack. The AppDelayTracer records the real RTT.
    ns3::ndn::AppHelper accidentConsumer("ns3::ndn::ConsumerCbr");
    accidentConsumer.SetPrefix(interestName);
    accidentConsumer.SetAttribute("Frequency", StringValue("1"));
    accidentConsumer.SetAttribute("LifeTime", TimeValue(ns3::MilliSeconds(500)));
    auto accidentApps = accidentConsumer.Install(vehicleNode);
    accidentApps.Start(ns3::Simulator::Now());
    accidentApps.Stop(ns3::Simulator::Now() + ns3::MilliSeconds(600));

    // Use measured V2I RTT from NdnPositionCache as calibrated latency baseline.
    // If unavailable, keep latency pending until per-Interest RTT callback.
    auto& ndnCache = v2x::ndn_cache::NdnPositionCache::GetInstance();
    double calibratedV2iRttMs = ndnCache.GetMeanRttMs();
    bool hasCalibratedV2iRtt = HasValidLatencySample(calibratedV2iRttMs);

    if (localCacheHit)
    {
        responderId = targetVehicleId + "/local_cs";
        responderType = "CACHE_HIT";
        
        std::cout << "    ✅ Vehicle " << targetVehicleId 
                  << " → Interest(" << interestName << ") → LOCAL CACHE HIT!"
                  << " | Distance: " << std::fixed << std::setprecision(0) << distance << "m"
                  << " | Urgency: " << urgency
                  << " | Initial latency: " << FormatLatencyForLog(measuredLatencyMs, latencyStatus)
                  << " | Per-interest RTT trace: pending" << std::endl;
    }
    else
    {
        // Check RSU cache state
        std::string nearestRsu = FindNearestRsuId(vehicleNode);
        bool rsuCacheHit = false;
        if (!nearestRsu.empty())
        {
            auto rsuIt = nodeMapping.find(nearestRsu);
            if (rsuIt != nodeMapping.end())
            {
                auto rsuNdn = rsuIt->second->GetObject<ns3::ndn::L3Protocol>();
                if (rsuNdn)
                {
                    auto& rsuCs = rsuNdn->getForwarder()->getCs();
                    rsuCs.find(*interest,
                        [&rsuCacheHit](const ::ndn::Interest&, const ::ndn::Data&) {
                            rsuCacheHit = true;
                        },
                        [&rsuCacheHit](const ::ndn::Interest&) {
                            rsuCacheHit = false;
                        }
                    );
                }
            }
        }
        
        if (rsuCacheHit && !nearestRsu.empty())
        {
            responderId = nearestRsu;
            responderType = "RSU";
            if (hasCalibratedV2iRtt)
            {
                // RSU latency is lower than vehicle V2I latency because RSU has wired backhaul
                // RSU eliminates the Uu air interface delay (3GPP TS 23.287)
                // Apply documented RSU_LATENCY_RATIO from v2x_constants.h
                measuredLatencyMs = calibratedV2iRttMs * RSU_LATENCY_RATIO;
                latencyStatus = "calibrated-rsu-wired-backhaul";
            }
            std::cout << "    📡 Vehicle " << targetVehicleId 
                      << " → Interest → RSU(" << nearestRsu << ") → Data"
                      << " | Initial latency: " << FormatLatencyForLog(measuredLatencyMs, latencyStatus)
                      << " | RSU ratio: " << RSU_LATENCY_RATIO
                      << " | Per-interest RTT trace: pending" << std::endl;
        }
        else
        {
            responderId = "MEC";
            responderType = "MEC";
            if (hasCalibratedV2iRtt)
            {
                measuredLatencyMs = calibratedV2iRttMs;
                latencyStatus = "calibrated-from-live-metrics";
            }
            std::cout << "    📡 Vehicle " << targetVehicleId 
                      << " → Interest → MEC → Data"
                      << " | Initial latency: " << FormatLatencyForLog(measuredLatencyMs, latencyStatus)
                      << " | Per-interest RTT trace: pending" << std::endl;
        }
    }
    
    // Build response record for this vehicle
    AccidentResponseRecord record;
    record.vehicleId = targetVehicleId;
    record.responderId = responderId;
    record.responderType = responderType;
    record.distanceToAccident = distance;
    record.urgency = urgency;
    record.interestName = interestName;
    record.responseTime = ns3::Simulator::Now().GetSeconds();
    record.measuredLatencyMs = measuredLatencyMs;
    
    // ========================================================================
    // PER-INTEREST LATENCY TRACKING (March 2026)
    // ========================================================================
    // Connect trace callbacks to measure actual RTT for this specific Interest.
    // The callbacks will update 'record' with measured latency when Data arrives.
    // ========================================================================
    record.interestSendTime = ns3::Simulator::Now();
    
    // Connect trace to app when Interest is sent (OutInterests)
    accidentApps.Get(0)->TraceConnectWithoutContext(
        "OutInterests",
        ns3::MakeBoundCallback(&OnAccidentNotificationInterestSent, &record));
    
    // Connect trace to app when Data arrives (InData)
    accidentApps.Get(0)->TraceConnectWithoutContext(
        "InData",
        ns3::MakeCallback(&OnAccidentNotificationDataReceived));
    
    // Store the record (will be collected by NotifyNearbyVehicles)
    m_latestResponseRecord = record;
    
    // Update metrics based on hit/miss
    if (localCacheHit)
    {
        nodeMetrics[targetVehicleId].cacheHits++;
    }
    else
    {
        nodeMetrics[targetVehicleId].cacheMisses++;
    }
    
    // Update metrics
    nodeMetrics[targetVehicleId].interestsSent++;
    g_v2xStats.ndnInterestsTx++;
    
    // Log to actions file with responder info
    if (actionsLog.is_open())
    {
        actionsLog << "[" << ns3::Simulator::Now().GetSeconds() << "s] "
                   << "ACCIDENT_NOTIFY: " << targetVehicleId 
                   << " → Interest(" << interestName << ")"
                   << " | RespondedBy: " << responderId
                   << " | ResponseType: " << responderType
                   << " | CacheHit: " << (localCacheHit ? "YES" : "NO")
                   << " | InitialLatency: " << FormatLatencyForLog(measuredLatencyMs, latencyStatus)
                   << " | Distance: " << distance << "m"
                   << " | Urgency: " << urgency 
                   << " | RTTTraceStatus: pending"
                   << " | [RTT will be measured via trace callback]" << "\n";
        actionsLog.flush();
    }
    
    return true;
}

std::string AccidentNotificationManager::GenerateActuationCommand(
    const std::string& vehicleId,
    const AccidentNotification& notification,
    double distance)
{
    // Determine action based on distance
    std::string action;
    double targetSpeed = 0;
    
    if (distance < 50)
    {
        action = "EMERGENCY_STOP";
        targetSpeed = 0;
    }
    else if (distance < 100)
    {
        action = "HARD_BRAKE";
        targetSpeed = 5.0;  // Slow to 5 m/s
    }
    else if (distance < 200)
    {
        action = "SLOW_DOWN";
        targetSpeed = 10.0;  // Slow to 10 m/s
    }
    else
    {
        action = "CAUTION";
        targetSpeed = 15.0;  // Reduce speed
    }
    
    // Map NS-3 vehicle ID back to OMNeT++ ID for outbound command
    std::string targetOmnetId;
    auto it = vehicleStatuses.find(vehicleId);
    if (it != vehicleStatuses.end())
    {
        targetOmnetId = it->second.omnetId;
    }
    if (targetOmnetId.empty()) targetOmnetId = GetOmnetVehicleId(vehicleId);
    if (targetOmnetId.empty()) targetOmnetId = vehicleId;

    // Generate safety_command in OMNeT++ expected format (matches IMPLEMENTATION.md spec)
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{\"message_type\":\"safety_command\","
       << "\"timestamp\":" << ns3::Simulator::Now().GetSeconds() << ","
       << "\"payload\":{"
       << "\"target_vehicle_id\":\"" << targetOmnetId << "\","
       << "\"source_type\":\"rsu\","
       << "\"action\":\"" << action << "\","
       << "\"target_speed\":" << targetSpeed << ","
       << "\"accident_data\":{"
       << "\"accident_id\":\"" << notification.accidentId << "\","
       << "\"origin_time\":" << notification.timestamp << ","
       << "\"lane_id\":\"" << notification.roadId << "\","
       << "\"pos_x\":" << notification.accidentX << ","
       << "\"pos_y\":" << notification.accidentY << ","
       << "\"crashed_vehicle_id\":\"" << notification.accidentId.substr(6) << "\","
       << "\"distance\":" << distance
       << "}"
       << "}"
       << "}";
    
    return ss.str();
}

void AccidentNotificationManager::PrintSummary() const
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Accident Notification Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Notification Radius:       " << m_notificationRadius << " m" << std::endl;
    std::cout << "Total Accidents:           " << m_stats.totalAccidents << std::endl;
    std::cout << "Total Notifications Sent:  " << m_stats.totalNotificationsSent << std::endl;
    std::cout << "Total Vehicles Notified:   " << m_stats.totalVehiclesInRange << std::endl;
    
    if (m_stats.totalAccidents > 0)
    {
        double avgNotifications = (double)m_stats.totalNotificationsSent / m_stats.totalAccidents;
        std::cout << "Avg Notifications/Accident:" << std::fixed << std::setprecision(1) 
                  << avgNotifications << std::endl;
    }
    
    std::cout << "\nPer-Accident Details:" << std::endl;
    for (const auto& [accidentId, notification] : m_notifications)
    {
        std::cout << "  " << accidentId << ": " 
                  << notification.notificationsSent << " notifications sent"
                  << " (radius: " << m_notificationRadius << "m)" << std::endl;
        std::cout << "    Response Sources → Cache: " << notification.cacheHitResponses
                  << " | RSU: " << notification.rsuResponses
                  << " | MEC: " << notification.mecResponses << std::endl;
        
        for (const auto& record : notification.responseRecords)
        {
            std::cout << "      " << record.vehicleId 
                      << " ← [" << record.responderType << "] " << record.responderId
                      << " (" << std::fixed << std::setprecision(0) << record.distanceToAccident << "m, "
                      << record.urgency << ")" << std::endl;
        }
    }
    std::cout << "========================================\n" << std::endl;
}

std::string AccidentNotificationManager::GenerateMetricsJson() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    
    ss << "{\n";
    ss << "  \"accident_notification\": {\n";
    ss << "    \"notification_radius_m\": " << m_notificationRadius << ",\n";
    ss << "    \"total_accidents\": " << m_stats.totalAccidents << ",\n";
    ss << "    \"total_notifications_sent\": " << m_stats.totalNotificationsSent << ",\n";
    ss << "    \"total_vehicles_in_range\": " << m_stats.totalVehiclesInRange << ",\n";
    
    if (m_stats.totalAccidents > 0)
    {
        double avgNotifications = (double)m_stats.totalNotificationsSent / m_stats.totalAccidents;
        ss << "    \"avg_notifications_per_accident\": " << avgNotifications << ",\n";
    }
    else
    {
        ss << "    \"avg_notifications_per_accident\": 0,\n";
    }
    
    ss << "    \"accidents\": [\n";
    bool first = true;
    for (const auto& [accidentId, notification] : m_notifications)
    {
        if (!first) ss << ",\n";
        first = false;
        
        ss << "      {\n";
        ss << "        \"accident_id\": \"" << notification.accidentId << "\",\n";
        ss << "        \"crashed_vehicle\": \"" << notification.crashedVehicleId << "\",\n";
        ss << "        \"location\": {\"x\": " << notification.accidentX 
           << ", \"y\": " << notification.accidentY << "},\n";
        ss << "        \"road_id\": \"" << notification.roadId << "\",\n";
        ss << "        \"severity\": \"" << notification.severity << "\",\n";
        ss << "        \"vehicles_notified\": " << notification.notificationsSent << ",\n";
        ss << "        \"vehicles_in_radius\": " << notification.vehiclesInRadius << ",\n";
        ss << "        \"response_sources\": {\n";
        ss << "          \"cache_hit\": " << notification.cacheHitResponses << ",\n";
        ss << "          \"rsu_cache\": " << notification.rsuResponses << ",\n";
        ss << "          \"mec_origin\": " << notification.mecResponses << "\n";
        ss << "        },\n";
        ss << "        \"vehicle_responses\": [\n";
        bool firstResp = true;
        for (const auto& record : notification.responseRecords)
        {
            if (!firstResp) ss << ",\n";
            firstResp = false;
            
            ss << "          {\n";
            ss << "            \"vehicle_id\": \"" << record.vehicleId << "\",\n";
            ss << "            \"responded_by\": \"" << record.responderId << "\",\n";
            ss << "            \"response_type\": \"" << record.responderType << "\",\n";
            ss << "            \"distance_m\": " << record.distanceToAccident << ",\n";
            ss << "            \"urgency\": \"" << record.urgency << "\",\n";
            ss << "            \"interest_name\": \"" << record.interestName << "\",\n";
            ss << "            \"timestamp\": " << record.responseTime << "\n";
            ss << "          }";
        }
        ss << "\n        ],\n";
        ss << "        \"timestamp\": " << notification.timestamp.GetSeconds() << "\n";
        ss << "      }";
    }
    ss << "\n    ]\n";
    ss << "  }\n";
    ss << "}\n";
    
    return ss.str();
}

} // namespace notification
} // namespace v2x
