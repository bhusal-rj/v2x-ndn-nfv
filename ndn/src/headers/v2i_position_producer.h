// V2I Position Producer Header
// Allows MEC to query vehicle positions via NDN Interest/Data exchange

#ifndef V2I_POSITION_PRODUCER_H
#define V2I_POSITION_PRODUCER_H

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"

#include <string>

namespace ns3 {
namespace ndn {

/**
 * @brief V2I Position Producer - Vehicle produces its own position for MEC queries
 *
 * This producer responds to NDN Interests from MEC requesting vehicle position.
 * The position data is retrieved from vehicleStatuses (the vehicle's own position).
 *
 * Naming scheme: /v2x/v2i/vehicle/{vehicle_id}/position
 *
 * Data flow:
 *   MEC sends Interest: /v2x/v2i/vehicle/veh_0/position
 *   Interest travels: MEC → PGW → gNB → Vehicle (via 5G NR)
 *   Vehicle produces Data with position from its own vehicleStatuses entry
 *   Data travels: Vehicle → gNB → PGW → MEC (via 5G NR)
 */
class V2iPositionProducer : public App
{
public:
    static TypeId GetTypeId();

    V2iPositionProducer();

    // Inherited from App
    virtual void StartApplication() override;
    virtual void StopApplication() override;

    // Handle incoming Interest packets
    virtual void OnInterest(std::shared_ptr<const ::ndn::Interest> interest) override;

private:
    /**
     * Build JSON payload with vehicle position data from vehicleStatuses
     * Returns JSON with: id, x, y, z, speed, heading, type, time
     */
    std::string BuildPositionPayload();

    std::string m_prefix;      // NDN prefix (e.g., /v2x/v2i/vehicle/0/position)
    std::string m_vehicleId;   // Vehicle ID for looking up position
    Time m_freshness;          // Data freshness period
    uint32_t m_signature;      // Signature type
    ::ndn::Name m_keyLocator;  // Key locator name
};

} // namespace ndn
} // namespace ns3

#endif // V2I_POSITION_PRODUCER_H
