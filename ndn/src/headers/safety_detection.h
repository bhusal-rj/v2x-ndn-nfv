#ifndef SAFETY_DETECTION_H
#define SAFETY_DETECTION_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <string>

// ============================================================================
// SAFETY DETECTION AND COLLISION WARNING FUNCTIONS
// ============================================================================

std::string GetCurrentTimeString();
std::string GetSimTimeString();

void PublishSafetyWarning(const std::string& vehicleId, const std::string& warningType,
                          const std::string& reason);

void DetectAndLogCollisions();
void RsuPeriodicPositionCheck();
void VehiclePeriodicTrafficQuery();

void SetupRsuVehicleMonitoring(ns3::Ptr<ns3::Node> rsuNode, const std::string& rsuId);
void SetupVehicleTrafficQuery(ns3::Ptr<ns3::Node> vehNode, const std::string& vehId);

#endif // SAFETY_DETECTION_H
