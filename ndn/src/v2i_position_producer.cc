// V2I Position Producer - Allows MEC to query vehicle positions via NDN
// Vehicles produce their own position data that MEC can query

#include "ns3/ndnSIM-module.h"
#include "v2i_position_producer.h"
#include "simulation_state.h"

#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <sstream>
#include <iomanip>

NS_LOG_COMPONENT_DEFINE("V2iPositionProducer");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(V2iPositionProducer);

TypeId
V2iPositionProducer::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ndn::V2iPositionProducer")
        .SetParent<App>()
        .SetGroupName("Ndn")
        .AddConstructor<V2iPositionProducer>()
        .AddAttribute("Prefix",
                      "Prefix for the producer (e.g., /v2x/v2i/vehicle/0/position)",
                      StringValue("/"),
                      MakeStringAccessor(&V2iPositionProducer::m_prefix),
                      MakeStringChecker())
        .AddAttribute("VehicleId",
                      "Vehicle ID used to look up position in vehicleStatuses",
                      StringValue("veh_0"),
                      MakeStringAccessor(&V2iPositionProducer::m_vehicleId),
                      MakeStringChecker())
        .AddAttribute("Freshness",
                      "Freshness period for data packets",
                      TimeValue(MilliSeconds(100)),
                      MakeTimeAccessor(&V2iPositionProducer::m_freshness),
                      MakeTimeChecker())
        .AddAttribute("Signature",
                      "Fake signature, 0 for none (default)",
                      UintegerValue(0),
                      MakeUintegerAccessor(&V2iPositionProducer::m_signature),
                      MakeUintegerChecker<uint32_t>())
        .AddAttribute("KeyLocator",
                      "Name to be used for key locator (default: /)",
                      NameValue("/"),
                      MakeNameAccessor(&V2iPositionProducer::m_keyLocator),
                      MakeNameChecker());
    return tid;
}

V2iPositionProducer::V2iPositionProducer()
{
    NS_LOG_FUNCTION(this);
}

void
V2iPositionProducer::StartApplication()
{
    NS_LOG_FUNCTION(this);
    App::StartApplication();

    // Register prefix with NFD
    FibHelper::AddRoute(GetNode(), m_prefix, m_face, 0);

    NS_LOG_INFO("V2iPositionProducer started on node " << GetNode()->GetId()
                << " with prefix " << m_prefix << " for vehicle " << m_vehicleId);
}

void
V2iPositionProducer::StopApplication()
{
    NS_LOG_FUNCTION(this);
    App::StopApplication();
}

void
V2iPositionProducer::OnInterest(std::shared_ptr<const ::ndn::Interest> interest)
{
    App::OnInterest(interest);

    NS_LOG_FUNCTION(this << interest->getName());

    if (!m_active) {
        return;
    }

    // Look up vehicle position from vehicleStatuses (vehicle's own position)
    std::string payload = BuildPositionPayload();

    // Create Data packet
    auto data = std::make_shared<::ndn::Data>(interest->getName());
    data->setFreshnessPeriod(::ndn::time::milliseconds(m_freshness.GetMilliSeconds()));

    // Set content with position data (using span for modern ndn-cxx API)
    auto buffer = std::make_shared<::ndn::Buffer>(payload.begin(), payload.end());
    data->setContent(buffer);

    // Add signature using KeyChain for proper signing
    static ::ndn::KeyChain keyChain;
    keyChain.sign(*data);

    // Send Data packet
    NS_LOG_INFO("V2iPositionProducer responding to Interest " << interest->getName()
                << " with position: " << payload);

    m_transmittedDatas(data, this, m_face);
    m_appLink->onReceiveData(*data);
}

std::string
V2iPositionProducer::BuildPositionPayload()
{
    // Look up vehicle's own position from vehicleStatuses
    auto it = vehicleStatuses.find(m_vehicleId);
    if (it != vehicleStatuses.end() && it->second.isActive) {
        const VehicleStatus& status = it->second;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "{\"id\":\"" << status.id << "\","
            << "\"x\":" << status.x << ","
            << "\"y\":" << status.y << ","
            << "\"z\":" << status.z << ","
            << "\"speed\":" << status.speed << ","
            << "\"heading\":" << status.heading << ","
            << "\"type\":\"" << status.vehicleType << "\","
            << "\"time\":" << Simulator::Now().GetSeconds() << "}";
        return oss.str();
    }

    // Return empty position if vehicle not found
    std::ostringstream oss;
    oss << "{\"id\":\"" << m_vehicleId << "\","
        << "\"x\":0,\"y\":0,\"z\":0,\"speed\":0,\"heading\":0,"
        << "\"type\":\"unknown\","
        << "\"time\":" << Simulator::Now().GetSeconds() << "}";
    return oss.str();
}

} // namespace ndn
} // namespace ns3
