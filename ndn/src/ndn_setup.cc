// IMPORTANT: Include ndnSIM FIRST to avoid namespace conflicts
#include "ns3/ndnSIM-module.h"
#include "ndn_setup.h"
#include "simulation_state.h"
#include "v2i_position_producer.h"
#include "ndn_consumer_cache_friendly.h"
#include "ndn_position_cache.h"
#include "v2x_constants.h"

#include "ns3/ndnSIM/model/ndn-l3-protocol.hpp"
#include "ns3/ndnSIM/NFD/daemon/table/cs.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/forwarder.hpp"
#include "ns3/ndnSIM/helper/ndn-app-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-strategy-choice-helper.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-cs-tracer.hpp"
#include "ns3/ndnSIM/utils/tracers/ndn-app-delay-tracer.hpp"

#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4.h"
#include "ns3/net-device.h"
#include "ns3/channel.h"
#include "ns3/log.h"

#include "ns3/ndnSIM/model/ndn-block-header.hpp"
#include "ns3/ndnSIM/NFD/daemon/face/face.hpp"
#include "ns3/ndnSIM/NFD/daemon/face/generic-link-service.hpp"

#include <exception>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ndn.V2xSetup");

namespace {

/// NDN UDP port on each UE (canonical unicast port in NDN deployments).
constexpr uint16_t NDN_V2I_UDP_PORT_UE = 6363;

/** NetDevice that owns \p addr on \p node (NR UE tun stack); nullptr if not found. */
Ptr<NetDevice>
GetNetDeviceForUeIpv4(Ptr<Node> node, const Ipv4Address& addr)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return nullptr;
    }
    const int32_t ifIndex = ipv4->GetInterfaceForAddress(addr);
    if (ifIndex < 0)
    {
        return nullptr;
    }
    return ipv4->GetNetDevice(static_cast<uint32_t>(ifIndex));
}

enum class V2iUdpSocketRole {
  kUe, ///< bind wildcard IPv4 so NR UE stack picks correct source (binding to one UE address breaks routing)
  kMec ///< bind explicit MEC address (many listeners on distinct ports)
};

/**
 * NFD transport that carries TLV over ns-3 UDP/IPv4 (simulated), so packets use Ue → NR → EPC routing.
 */
class V2iIpv4UdpTransport : public nfd::face::Transport
{
public:
  V2iIpv4UdpTransport(Ptr<Node> node,
                      const Ipv4Address& localIpForDisplay,
                      uint16_t localPort,
                      const Ipv4Address& remoteIp,
                      uint16_t remotePort,
                      V2iUdpSocketRole role)
    : m_socket(Socket::CreateSocket(node, UdpSocketFactory::GetTypeId()))
  {
    NS_ASSERT_MSG(m_socket != nullptr, "UDP socket creation failed");

    if (role == V2iUdpSocketRole::kUe) {
      // Tie the socket to the NR UE interface that actually carries AssignUeIpv4Address(); a
      // plain bind to that IP or to 0.0.0.0 often fails to egress toward the PGW/MEC in this
      // stack, so V2I NDN never completes (AppDelay stays V2V-only).
      Ptr<NetDevice> ueDev = GetNetDeviceForUeIpv4(node, localIpForDisplay);
      if (ueDev)
      {
        m_socket->BindToNetDevice(ueDev);
        if (m_socket->Bind(InetSocketAddress(localIpForDisplay, localPort)) < 0)
        {
          NS_FATAL_ERROR("V2I UDP bind failed on UE " << localIpForDisplay << ":" << localPort);
        }
      }
      else
      {
        NS_LOG_WARN("V2I UDP: no NetDevice for UE IP " << localIpForDisplay
                      << "; fallback bind 0.0.0.0:" << localPort);
        if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), localPort)) < 0)
        {
          NS_FATAL_ERROR("V2I UDP bind failed on 0.0.0.0:" << localPort << " (UE fallback)");
        }
      }
    }
    else {
      if (m_socket->Bind(InetSocketAddress(localIpForDisplay, localPort)) < 0) {
        NS_FATAL_ERROR("V2I UDP bind failed on " << localIpForDisplay << ":" << localPort);
      }
    }
    m_socket->Connect(InetSocketAddress(remoteIp, remotePort));
    m_socket->SetRecvCallback(MakeCallback(&V2iIpv4UdpTransport::HandleRead, this));
    m_socket->SetAllowBroadcast(false);

    this->setLocalUri(::ndn::FaceUri(MakeUdp4Uri(localIpForDisplay, localPort)));
    this->setRemoteUri(::ndn::FaceUri(MakeUdp4Uri(remoteIp, remotePort)));
    this->setScope(::ndn::nfd::FACE_SCOPE_NON_LOCAL);
    this->setPersistency(::ndn::nfd::FACE_PERSISTENCY_PERSISTENT);
    this->setLinkType(::ndn::nfd::LINK_TYPE_POINT_TO_POINT);
    this->setMtu(1200);
    this->setSendQueueCapacity(nfd::face::QUEUE_UNSUPPORTED);
  }

  ~V2iIpv4UdpTransport() override
  {
    if (m_socket) {
      m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
      m_socket->Close();
      m_socket = nullptr;
    }
  }

private:
  static std::string
  MakeUdp4Uri(const Ipv4Address& ip, uint16_t port)
  {
    std::ostringstream oss;
    oss << "udp4://" << ip << ":" << static_cast<unsigned>(port);
    return oss.str();
  }

  void
  HandleRead(Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    while ((packet = socket->Recv())) {
      const uint32_t n = packet->GetSize();
      if (n == 0) {
        continue;
      }
      // Do not call GetSerializedSize() on a default BlockHeader — in ndnSIM it can touch an
      // invalid inner block and throw. Parse only inside try; drop garbage UDP (wrong peer,
      // truncated datagram, non-NDN payload) instead of terminating the process.
      constexpr uint32_t kMinNdnWire = 3; // smallest well-formed TLV needs T + L + ≥0 V
      if (n < kMinNdnWire) {
        NS_LOG_WARN("V2iIpv4UdpTransport: drop short UDP payload (" << n << " B)");
        continue;
      }
      try {
        ns3::ndn::BlockHeader header;
        packet->RemoveHeader(header);
        this->receive(std::move(header.getBlock()));
        g_v2xStats.udpPacketsRx++;
        g_v2xStats.udpBytesRx += n;
      }
      catch (const std::exception& ex) {
        NS_LOG_WARN("V2iIpv4UdpTransport: drop invalid NDN on UDP (" << n << " B): " << ex.what());
      }
      catch (...) {
        NS_LOG_WARN("V2iIpv4UdpTransport: drop invalid NDN on UDP (" << n << " B): unknown exception");
      }
    }
  }

  void
  doSend(const ::ndn::Block& packet) override
  {
    if (getState() != nfd::face::TransportState::UP && getState() != nfd::face::TransportState::DOWN) {
      return;
    }
    ns3::ndn::BlockHeader header(packet);
    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(header);
    const int sent = m_socket->Send(p);
    if (sent < 0) {
      NS_LOG_WARN("V2iIpv4UdpTransport Send() failed");
    }
    else {
      g_v2xStats.udpPacketsTx++;
      g_v2xStats.udpBytesTx += static_cast<uint64_t>(sent);
    }
  }

  void
  doClose() override
  {
    if (m_socket) {
      m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
      m_socket->Close();
      m_socket = nullptr;
    }
    this->setState(nfd::face::TransportState::CLOSED);
  }

  Ptr<Socket> m_socket;
};

void RegisterNdnAppPrefixes(const ApplicationContainer& apps)
{
    for (uint32_t i = 0; i < apps.GetN(); i++)
    {
        Ptr<Application> app = apps.Get(i);
        if (!app) continue;

        std::string prefix;

        // ConsumerCacheFriendly stores "Prefix" as ::ndn::Name via
        // MakeNameAccessor. StringValue::GetAttributeFailSafe fails
        // silently for NameValue attributes — must use NameValue.
        Ptr<ns3::ndn::ConsumerCacheFriendly> ccf =
            DynamicCast<ns3::ndn::ConsumerCacheFriendly>(app);
        if (ccf)
        {
            // Read NameValue attribute correctly
            ns3::ndn::NameValue nameVal;
            if (app->GetAttributeFailSafe("Prefix", nameVal))
            {
                prefix = nameVal.Get().toUri();
            }
            // Fallback: GetInterestPrefixUri() if NameValue read fails
            if (prefix.empty())
            {
                prefix = ccf->GetInterestPrefixUri();
            }
        }
        else
        {
            // Producer/Consumer: Prefix may be NameValue (ndn::Consumer, custom apps)
            // or StringValue (some producer helpers). Try NameValue first.
            ns3::ndn::NameValue prefixName;
            if (app->GetAttributeFailSafe("Prefix", prefixName))
            {
                prefix = prefixName.Get().toUri();
            }
            if (prefix.empty())
            {
                StringValue prefixValue;
                if (app->GetAttributeFailSafe("Prefix", prefixValue))
                {
                    prefix = prefixValue.Get();
                }
            }
        }

        if (prefix.empty()) continue;

        Ptr<Node> node = app->GetNode();
        if (!node) continue;

        // AppDelayTracer logs AppId = ndn::App::m_appId, set in App::DoInitialize() to
        // this index (0..N-1 on the node). Keys must use the same index, not Object::GetId().
        for (uint32_t appIndex = 0; appIndex < node->GetNApplications(); ++appIndex)
        {
            if (node->GetApplication(appIndex) == app)
            {
                const std::string key =
                    BuildNdnAppPrefixKey(node->GetId(), appIndex);
                ndnAppIdToPrefix[key] = prefix;
                break;
            }
        }
    }
}

} // namespace

bool
SharesP2PChannel(Ptr<Node> nodeA, Ptr<Node> nodeB)
{
    if (!nodeA || !nodeB || nodeA == nodeB)
    {
        return false;
    }
    for (uint32_t d = 0; d < nodeA->GetNDevices(); d++)
    {
        Ptr<NetDevice> dev = nodeA->GetDevice(d);
        if (!dev->GetChannel())
        {
            continue;
        }
        Ptr<Channel> ch = dev->GetChannel();
        for (uint32_t cd = 0; cd < ch->GetNDevices(); cd++)
        {
            if (ch->GetDevice(cd)->GetNode() == nodeB)
            {
                return true;
            }
        }
    }
    return false;
}

// Connect to forwarder CS signals for a node
void ConnectCsSignals(Ptr<Node> node, const std::string& nodeName)
{
    Ptr<ns3::ndn::L3Protocol> l3 = node->GetObject<ns3::ndn::L3Protocol>();
    if (!l3)
        return;

    auto forwarder = l3->getForwarder();

    // Connect to forwarder's afterCsHit/afterCsMiss signals
    forwarder->afterCsHit.connect([nodeName](const ::ndn::Interest&, const ::ndn::Data&)
                                  { nodeMetrics[nodeName].cacheHits++; });

    forwarder->afterCsMiss.connect([nodeName](const ::ndn::Interest&)
                                   { nodeMetrics[nodeName].cacheMisses++; });
}

// Connect to forwarder Interest/Data signals for a node
// NOTE: These signals may not be available in all ndnSIM versions.
// The metrics are tracked via L3RateTracer instead when signals unavailable.
void ConnectForwarderSignals(Ptr<Node> node, const std::string& nodeName)
{
    Ptr<ns3::ndn::L3Protocol> l3 = node->GetObject<ns3::ndn::L3Protocol>();
    if (!l3)
        return;

    // For ndnSIM versions without direct forwarder signals, metrics are
    // collected via L3RateTracer callbacks configured elsewhere.
    // This function is kept for backward compatibility.
    
    // Initialize metrics entry for this node
    nodeMetrics[nodeName].interestsIn = 0;
    nodeMetrics[nodeName].dataIn = 0;
    nodeMetrics[nodeName].interestsOut = 0;
    nodeMetrics[nodeName].dataOut = 0;
}

// Connect to V2V-specific forwarder signals (track V2V prefix traffic)
// NOTE: These signals may not be available in all ndnSIM versions.
// V2V metrics are tracked via alternate mechanisms when unavailable.
void ConnectV2vForwarderSignals(Ptr<Node> node, const std::string& nodeName)
{
    Ptr<ns3::ndn::L3Protocol> l3 = node->GetObject<ns3::ndn::L3Protocol>();
    if (!l3)
        return;

    // For ndnSIM versions without direct forwarder signals, V2V metrics
    // are collected via L3RateTracer or application-level callbacks.
    // This function is kept for backward compatibility.
    
    // V2V stats are tracked at application level instead
    (void)nodeName;  // Suppress unused parameter warning
}


// Function to install NDN tracers on ALL NDN nodes (MEC + RSUs + Vehicles)
void InstallNdnTracers()
{
    if (tracersInstalled || ndnMecNode == nullptr)
        return;

    std::cout << " Installing NDN tracers on all NDN nodes..." << std::endl;

    // Create NodeContainer with ALL NDN nodes (MEC + Vehicles + RSUs)
    NodeContainer ndnNodes;
    ndnNodes.Add(ndnMecNode);

    // Add Vehicle NDN nodes (V2V direct links)
    for (uint32_t i = 0; i < vehicleNdnNodes.GetN(); i++)
    {
        ndnNodes.Add(vehicleNdnNodes.Get(i));
    }

    for (uint32_t i = 0; i < rsuNdnNodes.GetN(); i++)
    {
        ndnNodes.Add(rsuNdnNodes.Get(i));
    }

    std::cout << "  Installing tracers on " << ndnNodes.GetN() << " NDN nodes (1 MEC + "
              << vehicleNdnNodes.GetN() << " Vehicles + " << rsuNdnNodes.GetN() << " RSUs)" << std::endl;

    // Install L3 Rate Tracer (tracks Interest/Data packets)
    // Use 0.25s intervals for fine-grained rate tracking
    ns3::ndn::L3RateTracer::Install(ndnNodes, "rate-trace.txt", Seconds(0.25));
    std::cout << "  ✓ L3RateTracer installed (rate-trace.txt, 0.25s intervals)" << std::endl;

    // Install App Delay Tracer (tracks end-to-end latency)
    ns3::ndn::AppDelayTracer::Install(ndnNodes, "app-delays-trace.txt");
    std::cout << "  ✓ AppDelayTracer installed (app-delays-trace.txt)" << std::endl;

    // Install CS Tracer for detailed cache metrics
    ns3::ndn::CsTracer::Install(ndnNodes, "ndn-cs-trace.txt", Seconds(0.25));
    std::cout << "  ✓ CsTracer installed (ndn-cs-trace.txt, 0.25s intervals)" << std::endl;

    tracersInstalled = true;
}

// Helper function to install NDN stack on the MEC node (edge cache)
// NDN traffic now routes through 5G NR stack (via PGW connection)
void InstallNdnOnMec(Ptr<Node> node)
{
    ns3::ndn::StackHelper ndnStackHelper;
    ndnStackHelper.SetDefaultRoutes(false);
    ndnStackHelper.setCsSize(10000);       // Large Content Store for edge caching
    ndnStackHelper.Install(node);

    // Install Strategy - best-route for efficient forwarding
    ns3::ndn::StrategyChoiceHelper::Install(node, "/", "/localhost/nfd/strategy/best-route");
    ns3::ndn::StrategyChoiceHelper::Install(node, "/v2x", "/localhost/nfd/strategy/best-route");

    // Connect to CS signals for cache hit/miss tracking
    ConnectCsSignals(node, "mec");
    
    // Connect to Interest/Data signals for packet counting
    ConnectForwarderSignals(node, "mec");
    
    // Connect to V2V-specific signals for V2V message counting (MEC also participates in V2X)
    ConnectV2vForwarderSignals(node, "mec");

    std::cout << "  ✓ NDN stack installed on MEC node (CS size: 10000)" << std::endl;
    std::cout << "    - V2I FIB/nexthops: SetupUdpFacesForV2i (UDP/IPv4 over simulated NR path)" << std::endl;
}

// Install NDN on vehicle nodes
// ARCHITECTURE FIX: NDN traffic now routes through 5G NR UE devices
// instead of separate P2P bypass links

void InstallNdnOnVehicles(NodeContainer& vehicles)
{
    ns3::ndn::StackHelper ndnStackHelper;
    // NrUeNetDevice: no native NDN face (StackHelper skips in DefaultNetDeviceCallback).
    // V2V: PointToPoint faces; V2I: SetupUdpFacesForV2i adds UDP/IPv4 faces to MEC.
    ndnStackHelper.SetDefaultRoutes(false);
    ndnStackHelper.setCsSize(500); // Smaller CS for vehicles (limited memory)
    ndnStackHelper.Install(vehicles);

    // Install multicast strategy for V2V broadcast (direct P2P links)
    // best-route strategy for V2I (to MEC via 5G NR)
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        ns3::ndn::StrategyChoiceHelper::Install(vehicles.Get(i), "/", "/localhost/nfd/strategy/best-route");
        ns3::ndn::StrategyChoiceHelper::Install(vehicles.Get(i), "/v2x/v2v", "/localhost/nfd/strategy/multicast");
        ns3::ndn::StrategyChoiceHelper::Install(vehicles.Get(i), "/v2x/traffic", "/localhost/nfd/strategy/best-route");
        ns3::ndn::StrategyChoiceHelper::Install(vehicles.Get(i), "/v2x/safety", "/localhost/nfd/strategy/best-route");

        // Connect to CS signals for cache hit/miss tracking
        ConnectCsSignals(vehicles.Get(i), "vehicle_" + std::to_string(i));
        
        // Connect to Interest/Data signals for packet counting
        ConnectForwarderSignals(vehicles.Get(i), "vehicle_" + std::to_string(i));
        
        // Connect to V2V-specific signals for V2V message counting
        ConnectV2vForwarderSignals(vehicles.Get(i), "vehicle_" + std::to_string(i));
        
        // Install consumer apps on each vehicle to query MEC for traffic data
        std::string vehId = "veh_" + std::to_string(i);
        SetupNdnConsumerOnUe(vehicles.Get(i), vehId);
    }

    std::cout << "  ✓ NDN stack installed on " << vehicles.GetN() << " vehicle nodes (CS size: 500)" << std::endl;
    std::cout << "    - V2V: NDN faces on V2V direct links (P2P emulation)" << std::endl;
    std::cout << "    - V2I: NDN over UDP/IPv4 via NR (SetupUdpFacesForV2i)" << std::endl;
    std::cout << "    - Consumer apps installed to query MEC for traffic/count/safety" << std::endl;
}

// Install NDN on RSU nodes for proactive caching
// ARCHITECTURE FIX: RSUs also use 5G NR UE devices (no P2P bypass)
void InstallNdnOnRsus(NodeContainer& rsuNodes, Ptr<Node> mecNode)
{
    std::cout << "  Installing NDN stack on RSU nodes for proactive caching..." << std::endl;
    
    ns3::ndn::StackHelper ndnStackHelper;
    ndnStackHelper.SetDefaultRoutes(false);
    ndnStackHelper.setCsSize(2000); // Medium-sized CS for RSU edge caching
    ndnStackHelper.Install(rsuNodes);

    // Install best-route strategy for efficient forwarding
    for (uint32_t i = 0; i < rsuNodes.GetN(); i++)
    {
        ns3::ndn::StrategyChoiceHelper::Install(rsuNodes.Get(i), "/", "/localhost/nfd/strategy/best-route");
        ns3::ndn::StrategyChoiceHelper::Install(rsuNodes.Get(i), "/v2x", "/localhost/nfd/strategy/best-route");

        // Connect to CS signals for cache hit/miss tracking
        ConnectCsSignals(rsuNodes.Get(i), "rsu_" + std::to_string(i));
        
        // Connect to Interest/Data signals for packet counting
        ConnectForwarderSignals(rsuNodes.Get(i), "rsu_" + std::to_string(i));
        
        // Connect to V2V-specific signals for V2V message counting
        ConnectV2vForwarderSignals(rsuNodes.Get(i), "rsu_" + std::to_string(i));
        
        // Install consumer apps on each RSU to query MEC for traffic data
        std::string rsuId = "rsu_" + std::to_string(i);
        SetupNdnConsumerOnUe(rsuNodes.Get(i), rsuId);
    }

    std::cout << "  ✓ NDN stack installed on " << rsuNodes.GetN() << " RSU nodes (CS size: 2000)" << std::endl;
    std::cout << "    - V2I: NDN over UDP/IPv4 via NR (SetupUdpFacesForV2i)" << std::endl;
    std::cout << "    - Consumer apps installed to query MEC" << std::endl;
}

// Setup NDN Producer apps on MEC node
void SetupNdnAppsOnMec(Ptr<Node> mecNode)
{
    // ========================================================================
    // NDN Freshness Period Configuration (Data Validity Based)
    // ========================================================================
    // Freshness periods set based on DATA VALIDITY requirements, not query patterns.
    // Citation: ETSI EN 302 637-2 - data validity window considerations
    // 
    // Traffic Info: Valid for 500ms (traffic conditions change at intersection level)
    // Vehicle Count: Valid for 1000ms (aggregate metric, slowly changing)  
    // Safety Warnings: Valid for 100ms (time-critical, must be fresh)
    //
    // NOTE: Query intervals should be <= freshness for cache benefit,
    // but freshness is set by data semantics, not to engineer cache hits.
    // ========================================================================

    // MEC produces aggregated traffic info. ndn::Producer sets Data name from
    // interest->getName(), so bucket suffixes on ConsumerCacheFriendly Interests
    // (e.g. /v2x/traffic/info/b/0) are echoed exactly for pending lookup.
    ns3::ndn::AppHelper trafficProducer("ns3::ndn::Producer");
    trafficProducer.SetPrefix("/v2x/traffic/info");
    trafficProducer.SetAttribute("PayloadSize", StringValue("512"));
    // Freshness: Traffic info valid for 500ms per ETSI EN 302 637-2 data validity guidelines
    trafficProducer.SetAttribute("Freshness", TimeValue(MilliSeconds(TRAFFIC_INFO_FRESHNESS_MS)));
    auto trafficProducerApps = trafficProducer.Install(mecNode);
    RegisterNdnAppPrefixes(trafficProducerApps);

    // MEC produces vehicle count
    ns3::ndn::AppHelper countProducer("ns3::ndn::Producer");
    countProducer.SetPrefix("/v2x/traffic/count");
    countProducer.SetAttribute("PayloadSize", StringValue("64"));
    // Freshness: Vehicle count is aggregate data, valid for 1s
    countProducer.SetAttribute("Freshness", TimeValue(MilliSeconds(VEHICLE_COUNT_FRESHNESS_MS)));
    auto countProducerApps = countProducer.Install(mecNode);
    RegisterNdnAppPrefixes(countProducerApps);

    // MEC produces safety warnings
    ns3::ndn::AppHelper safetyProducer("ns3::ndn::Producer");
    safetyProducer.SetPrefix("/v2x/safety");
    safetyProducer.SetAttribute("PayloadSize", StringValue("128"));
    // Freshness: Safety warnings are time-critical, must be fresh (100ms max validity)
    safetyProducer.SetAttribute("Freshness", TimeValue(MilliSeconds(SAFETY_WARNING_FRESHNESS_MS)));
    auto safetyProducerApps = safetyProducer.Install(mecNode);
    RegisterNdnAppPrefixes(safetyProducerApps);

    // NOTE: Do not install synthetic MEC self-consumers here.
    // Running ConsumerZipfMandelbrot and Producer on the same MEC node can trigger
    // a forwarder CS-hit path crash in this ndnSIM version during early co-sim steps.
    // Real V2I/V2V demand is generated by vehicle/RSU-side applications.
    std::cout << "  ✓ NDN apps on MEC: Producer (traffic/count/safety)" << std::endl;
}

// Setup V2V NDN Producer/Consumer apps on vehicles
void SetupV2vNdnApps(NodeContainer& vehicles)
{
    std::cout << "  Setting up V2V NDN applications..." << std::endl;

    uint32_t totalConsumers = 0;
    uint32_t totalProducers = 0;

    // Each vehicle produces its own position/status data
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        std::string prefix = "/v2x/v2v/vehicle/" + std::to_string(i);

        // Producer: Share position data with nearby vehicles
        ns3::ndn::AppHelper producerHelper("ns3::ndn::Producer");
        producerHelper.SetPrefix(prefix + "/position");
        producerHelper.SetAttribute("PayloadSize", StringValue("128"));
        producerHelper.SetAttribute("Freshness", TimeValue(MilliSeconds(150))); // 150ms freshness for cache hits
        auto producerApps = producerHelper.Install(vehicles.Get(i));
        producerApps.Start(Seconds(0.5));  // Start early to be ready for requests
        RegisterNdnAppPrefixes(producerApps);
        totalProducers++;

        // Producer: Share speed data
        ns3::ndn::AppHelper speedProducer("ns3::ndn::Producer");
        speedProducer.SetPrefix(prefix + "/speed");
        speedProducer.SetAttribute("PayloadSize", StringValue("64"));
        speedProducer.SetAttribute("Freshness", TimeValue(MilliSeconds(150))); // 150ms freshness for cache hits
        auto speedProducerApps = speedProducer.Install(vehicles.Get(i));
        speedProducerApps.Start(Seconds(0.5));  // Start early to be ready for requests
        RegisterNdnAppPrefixes(speedProducerApps);
        totalProducers++;
        
        // NOTE: REMOVED duplicate shared content producers from vehicles.
        // MEC is the SOLE authoritative producer for /v2x/traffic/info, /v2x/traffic/count, /v2x/safety
        // Having both MEC and vehicles produce the SAME prefixes causes Content Store
        // conflicts and crashes when returning cached data via AppLinkService faces.
        // 
        // Hybrid naming architecture:
        // - Shared (MEC-only): /v2x/traffic/info, /v2x/traffic/count, /v2x/safety
        // - Private (vehicle-specific): /v2x/v2v/vehicle/<id>/position, /v2x/v2v/vehicle/<id>/speed
    }
    
    std::cout << "    - V2V: Each vehicle produces position/speed data for its own ID (" << totalProducers << " producers)" << std::endl;
    std::cout << "    - SHARED: MEC is sole producer for traffic/count/safety (no duplicate producers)" << std::endl;

    // Each vehicle consumes data from specific neighboring vehicles
    // This matches the actual producer prefixes
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        // Query position data from 2-3 neighbors (matching actual producer prefixes)
        for (uint32_t j = 0; j < std::min((uint32_t)3, vehicles.GetN()); j++)
        {
            if (i == j) continue; // Don't query yourself
            if (!SharesP2PChannel(vehicles.Get(i), vehicles.Get(j)))
            {
                continue;
            }
            
            // Consumer for neighbor's position
            std::string posPrefix = "/v2x/v2v/vehicle/" + std::to_string(j) + "/position";
            ns3::ndn::AppHelper posConsumer("ns3::ndn::ConsumerCbr");
            posConsumer.SetPrefix(posPrefix);
            posConsumer.SetAttribute("Frequency", StringValue("20")); // 20 Hz (50ms interval)
            posConsumer.SetAttribute("LifeTime", TimeValue(MilliSeconds(100)));
            posConsumer.SetAttribute("StartTime", TimeValue(Seconds(1.0)));  // Start after producers are ready
            posConsumer.SetAttribute("StopTime", TimeValue(Seconds(100.0)));  // Run for duration
            // ConsumerCbr appends seq numbers, so needs proper prefix to work with Producer
            auto posConsumerApps = posConsumer.Install(vehicles.Get(i));
            posConsumerApps.Start(Seconds(1.0));  // Start after producers
            // Position consumer direct registration
            for (uint32_t ci = 0; ci < posConsumerApps.GetN(); ci++) {
                Ptr<Application> app = posConsumerApps.Get(ci);
                if (!app) continue;
                Ptr<Node> n = app->GetNode();
                if (!n) continue;
                for (uint32_t idx = 0; idx < n->GetNApplications(); idx++) {
                    if (n->GetApplication(idx) == app) {
                        ndnAppIdToPrefix[BuildNdnAppPrefixKey(n->GetId(), idx)]
                            = posPrefix;
                        break;
                    }
                }
            }
            totalConsumers++;
            
            // Consumer for neighbor's speed
            std::string speedPrefix = "/v2x/v2v/vehicle/" + std::to_string(j) + "/speed";
            ns3::ndn::AppHelper speedConsumer("ns3::ndn::ConsumerCbr");
            speedConsumer.SetPrefix(speedPrefix);
            speedConsumer.SetAttribute("Frequency", StringValue("10")); // 10 Hz
            speedConsumer.SetAttribute("LifeTime", TimeValue(MilliSeconds(100)));
            speedConsumer.SetAttribute("StartTime", TimeValue(Seconds(1.0)));  // Start after producers are ready
            speedConsumer.SetAttribute("StopTime", TimeValue(Seconds(100.0)));  // Run for duration
            auto speedConsumerApps = speedConsumer.Install(vehicles.Get(i));
            speedConsumerApps.Start(Seconds(1.0));  // Start after producers
            // Speed consumer direct registration
            for (uint32_t ci = 0; ci < speedConsumerApps.GetN(); ci++) {
                Ptr<Application> app = speedConsumerApps.Get(ci);
                if (!app) continue;
                Ptr<Node> n = app->GetNode();
                if (!n) continue;
                for (uint32_t idx = 0; idx < n->GetNApplications(); idx++) {
                    if (n->GetApplication(idx) == app) {
                        ndnAppIdToPrefix[BuildNdnAppPrefixKey(n->GetId(), idx)]
                            = speedPrefix;
                        break;
                    }
                }
            }
            totalConsumers++;
        }
    }
    std::cout << "  ✓ V2V NDN apps installed on " << vehicles.GetN() << " vehicles" << std::endl;
    std::cout << "    - V2V Producers: " << totalProducers << " (ready at t=0.5s)" << std::endl;
    std::cout << "    - V2V Consumers: " << totalConsumers << " (active from t=1.0s)" << std::endl;
    std::cout << "    - Each vehicle produces: /v2x/v2v/vehicle/<id>/position, /speed" << std::endl;
    std::cout << "    - Each vehicle consumes: neighbor positions/speeds @ 20/10 Hz" << std::endl;
}

// Setup V2I NDN Producer apps on vehicles (for MEC to query positions)
void SetupV2iNdnApps(NodeContainer& vehicles)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  V2I NDN Position Query Setup" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Setting up V2I NDN applications (MEC → Vehicle queries)..." << std::endl;
    std::cout << "  Number of vehicles: " << vehicles.GetN() << std::endl;

    // Each vehicle produces its own position for MEC to query via NDN
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        std::string vehicleId = "veh_" + std::to_string(i);
        std::string prefix = "/v2x/v2i/vehicle/" + std::to_string(i) + "/position";

        // Install custom V2I Position Producer that returns actual position from vehicleStatuses
        ns3::ndn::AppHelper v2iProducerHelper("ns3::ndn::V2iPositionProducer");
        v2iProducerHelper.SetPrefix(prefix);
        v2iProducerHelper.SetAttribute("VehicleId", StringValue(vehicleId));
        v2iProducerHelper.SetAttribute("Freshness", TimeValue(MilliSeconds(100)));
        auto v2iProducerApps = v2iProducerHelper.Install(vehicles.Get(i));
        
        // Register V2I producer prefix for latency categorization
        RegisterNdnAppPrefixes(v2iProducerApps);

        // Install strategy for V2I prefix on this vehicle
        ns3::ndn::StrategyChoiceHelper::Install(vehicles.Get(i), "/v2x/v2i", "/localhost/nfd/strategy/best-route");
    }

    std::cout << "  ✓ V2I NDN apps installed on " << vehicles.GetN() << " vehicles" << std::endl;
    std::cout << "    - Each vehicle produces: /v2x/v2i/vehicle/<id>/position" << std::endl;
    std::cout << "    - MEC queries vehicle positions via NDN Interest/Data" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// Callback to parse position Data packet and update NDN position cache
void OnMecPositionDataReceived(const std::shared_ptr<const ::ndn::Data>& data)
{
    // Extract vehicle ID from Interest name: /v2x/v2i/vehicle/{id}/position/{seq}
    std::string name = data->getName().toUri();
    size_t vehPos = name.find("/vehicle/");
    if (vehPos == std::string::npos) return;
    
    size_t idStart = vehPos + 9;  // "/vehicle/" length
    size_t idEnd = name.find("/", idStart);
    if (idEnd == std::string::npos) return;
    
    std::string vehicleIdNum = name.substr(idStart, idEnd - idStart);
    std::string vehicleId = "veh_" + vehicleIdNum;
    
    // Parse JSON payload: {"id":"veh_0","x":123.45,"y":234.56,"z":0,"speed":15.2,"heading":1.57,...}
    const uint8_t* payload = data->getContent().value();
    size_t payloadSize = data->getContent().value_size();
    std::string jsonStr(reinterpret_cast<const char*>(payload), payloadSize);
    
    // Simple JSON parsing (no external library)
    double x = 0, y = 0, z = 0, speed = 0, heading = 0;
    std::string omnetId;
    
    auto parseField = [&jsonStr](const std::string& field) -> std::string {
        size_t pos = jsonStr.find("\"" + field + "\":");
        if (pos == std::string::npos) return "";
        
        size_t valStart = jsonStr.find(":", pos) + 1;
        while (valStart < jsonStr.length() && (jsonStr[valStart] == ' ' || jsonStr[valStart] == '"'))
            valStart++;
        
        size_t valEnd = valStart;
        while (valEnd < jsonStr.length() && jsonStr[valEnd] != ',' && 
               jsonStr[valEnd] != '}' && jsonStr[valEnd] != '"')
            valEnd++;
        
        return jsonStr.substr(valStart, valEnd - valStart);
    };
    
    try {
        x = std::stod(parseField("x"));
        y = std::stod(parseField("y"));
        z = std::stod(parseField("z"));
        speed = std::stod(parseField("speed"));
        heading = std::stod(parseField("heading"));
        omnetId = parseField("id");
    } catch (...) {
        std::cerr << "Warning: Failed to parse position data for " << vehicleId << std::endl;
        return;
    }
    
    // Update NDN position cache
    v2x::ndn_cache::NdnPositionCache::GetInstance().UpdateFromNdnData(
        vehicleId, x, y, z, speed, heading, omnetId);
}

// App trace callback for Interest transmission (exact App::InterestTraceCallback signature)
void OnMecPositionInterestSentApp(std::string vehicleId,
                                  std::shared_ptr<const ::ndn::Interest>,
                                  Ptr<ns3::ndn::App>,
                                  std::shared_ptr<nfd::Face>)
{
    v2x::ndn_cache::NdnPositionCache::GetInstance().RecordInterestSent(vehicleId);
}

// App trace callback for Data reception (exact App::DataTraceCallback signature)
void OnMecPositionDataReceivedApp(std::shared_ptr<const ::ndn::Data> data,
                                  Ptr<ns3::ndn::App>,
                                  std::shared_ptr<nfd::Face>)
{
    OnMecPositionDataReceived(data);
}

// Setup MEC position consumer - queries vehicle positions via NDN
void SetupMecPositionConsumer(Ptr<Node> mecNode, uint32_t numVehicles)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  MEC Position Consumer Setup" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Setting up MEC position consumer (queries vehicle positions via NDN)..." << std::endl;
    std::cout << "  Number of vehicles to query: " << numVehicles << std::endl;
    
    // MEC queries each vehicle's position periodically
    // Use ConsumerCbr to send periodic Interests
    for (uint32_t i = 0; i < numVehicles; i++)
    {
        std::string prefix = "/v2x/v2i/vehicle/" + std::to_string(i) + "/position";
        std::string vehicleId = "veh_" + std::to_string(i);
        
        ns3::ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCbr");
        consumerHelper.SetPrefix(prefix);
        consumerHelper.SetAttribute("Frequency", StringValue("10"));  // 10 Hz (100ms interval)
        consumerHelper.SetAttribute("LifeTime", TimeValue(MilliSeconds(200)));
        
        auto consumerApps = consumerHelper.Install(mecNode);
        
        // Direct registration — guaranteed to work regardless of
        // attribute accessor type
        for (uint32_t ci = 0; ci < consumerApps.GetN(); ci++) {
            Ptr<Application> app = consumerApps.Get(ci);
            if (!app) continue;
            Ptr<Node> n = app->GetNode();
            if (!n) continue;
            for (uint32_t idx = 0; idx < n->GetNApplications(); idx++) {
                if (n->GetApplication(idx) == app) {
                    ndnAppIdToPrefix[BuildNdnAppPrefixKey(n->GetId(), idx)]
                        = prefix;
                    break;
                }
            }
        }
        
        // Start after 2 seconds to avoid lockstep synchronization issues
        consumerApps.Start(Seconds(2.0 + i * 0.01));  // Staggered start
        
        // Connect app-level callbacks. ConsumerCbr exposes App trace sources
        // (TransmittedInterests / ReceivedDatas), while OutInterests/InData are L3 traces.
        consumerApps.Get(0)->TraceConnectWithoutContext(
            "TransmittedInterests",
            MakeBoundCallback(&OnMecPositionInterestSentApp, vehicleId));
        consumerApps.Get(0)->TraceConnectWithoutContext(
            "ReceivedDatas",
            MakeCallback(&OnMecPositionDataReceivedApp));
    }
    
    std::cout << "  ✓ MEC position consumer installed for " << numVehicles << " vehicles @ 10 Hz" << std::endl;
    std::cout << "    - MEC queries: /v2x/v2i/vehicle/{id}/position" << std::endl;
    std::cout << "    - Updates ndnReceivedPositions cache on Data arrival" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

// Setup NDN Consumer apps on UE nodes
void SetupNdnConsumerOnUe(Ptr<Node> node, const std::string& id)
{
    bool isRsu = (id.find("rsu") != std::string::npos);

    // Direct registration lambda — bypasses attribute system entirely.
    // Prefix is known at call site; no GetAttributeFailSafe needed.
    // Works regardless of NameValue/StringValue accessor type in ndnSIM 2.9.
    auto registerDirect = [&](const ApplicationContainer& apps,
                               const std::string& prefix) {
        for (uint32_t i = 0; i < apps.GetN(); i++) {
            Ptr<Application> app = apps.Get(i);
            if (!app) continue;
            Ptr<Node> n = app->GetNode();
            if (!n) continue;
            for (uint32_t idx = 0; idx < n->GetNApplications(); idx++) {
                if (n->GetApplication(idx) == app) {
                    const std::string key =
                        BuildNdnAppPrefixKey(n->GetId(), idx);
                    ndnAppIdToPrefix[key] = prefix;
                    NS_LOG_DEBUG("Direct register: " << key
                        << " -> " << prefix);
                    break;
                }
            }
        }
    };

    // Traffic info consumer
    ns3::ndn::AppHelper trafficConsumer("ns3::ndn::ConsumerCacheFriendly");
    trafficConsumer.SetPrefix("/v2x/traffic/info");
    trafficConsumer.SetAttribute("Frequency", DoubleValue(10.0));
    trafficConsumer.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    auto trafficConsumerApps = trafficConsumer.Install(node);
    registerDirect(trafficConsumerApps, "/v2x/traffic/info");

    // Vehicle count consumer
    ns3::ndn::AppHelper countConsumer("ns3::ndn::ConsumerCacheFriendly");
    countConsumer.SetPrefix("/v2x/traffic/count");
    countConsumer.SetAttribute("Frequency", DoubleValue(5.0));
    countConsumer.SetAttribute("StartTime", TimeValue(Seconds(1.5)));
    auto countConsumerApps = countConsumer.Install(node);
    registerDirect(countConsumerApps, "/v2x/traffic/count");

    // Safety consumer
    ns3::ndn::AppHelper safetyConsumer("ns3::ndn::ConsumerCacheFriendly");
    safetyConsumer.SetPrefix("/v2x/safety");
    safetyConsumer.SetAttribute("Frequency", DoubleValue(20.0));
    safetyConsumer.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
    auto safetyConsumerApps = safetyConsumer.Install(node);
    registerDirect(safetyConsumerApps, "/v2x/safety");

    if (isRsu)
        std::cout << "    RSU " << id
            << ": NDN Consumer (CacheFriendly) for traffic/count/safety"
            << std::endl;
    else
        std::cout << "    Vehicle " << id
            << ": NDN Consumer (CacheFriendly) for traffic/count/safety"
            << std::endl;
}

// Setup NDN FIB routes manually
// ARCHITECTURE FIX: Routes now use 5G NR path (no P2P bypass)
void SetupNdnFibRoutes(NodeContainer& vehicles, Ptr<Node> mecNode)
{
    std::cout << "  Setting up NDN FIB entries (via 5G NR stack)..." << std::endl;

    uint32_t numVeh = vehicles.GetN();
    
    // V2I: explicit /v2x nexthops via UDP/IPv4 faces (SetupUdpFacesForV2i).
    // V2V: explicit routes below for prefixes reachable on P2P neighbor links.
    
    // V2V FIB routes via direct P2P links (V2V direct connections).
    //
    // NEVER add FibHelper routes for /v2x/traffic, /v2x/traffic/count, or /v2x/safety toward
    // V2V neighbors (e.g. AddRoute(nodeI, "/v2x/traffic", nodeJ, 1)): ConsumerCacheFriendly
    // interests would match those prefixes, best-route could pick the P2P face at equal cost
    // vs the MEC UDP face, and neighbors have no producer — Interests time out (no FullDelay
    // in app-delays-trace). MEC-only names must use the UDP face from SetupUdpFacesForV2i only.
    for (uint32_t i = 0; i < numVeh; i++)
    {
        // V2V FIB routes via direct P2P links
        // Only add routes for node pairs that share a P2P channel
        for (uint32_t j = i + 1; j < std::min(i + 3, numVeh); j++)
        {
            Ptr<Node> nodeI = vehicles.Get(i);
            Ptr<Node> nodeJ = vehicles.Get(j);
            if (!SharesP2PChannel(nodeI, nodeJ))
            {
                continue;
            }
            // V2V position/speed routes (per-vehicle prefix)
            std::string prefixJ = "/v2x/v2v/vehicle/" + std::to_string(j);
            ns3::ndn::FibHelper::AddRoute(nodeI, prefixJ, nodeJ, 1);
            std::string prefixI = "/v2x/v2v/vehicle/" + std::to_string(i);
            ns3::ndn::FibHelper::AddRoute(nodeJ, prefixI, nodeI, 1);
        }
    }

    std::cout << "  ✓ NDN FIB routes configured (V2V P2P + V2I via SetupUdpFacesForV2i)" << std::endl;
    std::cout << "    - V2I: /v2x and /v2x/v2i/vehicle/* wired to UDP/IPv4 NDN faces" << std::endl;
    std::cout << "    - V2V routes: Vehicle ↔ Vehicle (direct P2P links, direct NDN faces)" << std::endl;
}

// Setup NDN Global Routing for V2I communication
// This function calculates NDN routes based on the underlying network topology
// It must be called BEFORE SetupMecPositionConsumer to ensure MEC can reach vehicles
void SetupNdnGlobalRouting(NodeContainer& vehicles, Ptr<Node> mecNode)
{
    std::cout << "  Setting up NDN global routing (MEC ↔ Vehicles via 5G core)..." << std::endl;

    // GlobalRoutingHelper is not used (non-NDN hops in the 5G path). V2I uses
    // SetupUdpFacesForV2i; V2V uses explicit P2P FIB entries from SetupNdnFibRoutes.
    Ptr<ns3::ndn::L3Protocol> mecL3 = mecNode ? mecNode->GetObject<ns3::ndn::L3Protocol>() : nullptr;
    uint32_t vehicleL3Count = 0;
    for (uint32_t i = 0; i < vehicles.GetN(); i++)
    {
        if (vehicles.Get(i) && vehicles.Get(i)->GetObject<ns3::ndn::L3Protocol>())
        {
            vehicleL3Count++;
        }
    }

    if (!mecL3 || vehicleL3Count == 0)
    {
        std::cerr << "  ✗ NDN L3 missing on MEC or vehicles; skipping V2I route setup" << std::endl;
        return;
    }

    std::cout << "  ✓ Skipping ndnSIM GlobalRoutingHelper for NDN-over-IP 5G path" << std::endl;
    std::cout << "    - V2I uses explicit UDP/IPv4 faces (SetupUdpFacesForV2i), not topology walking" << std::endl;
    std::cout << "    - V2V explicit routes are configured on direct P2P links" << std::endl;
}

void
SetupUdpFacesForV2i(Ptr<Node> mecNode,
                    const NodeContainer& ueNodes,
                    const std::vector<std::string>& ueNodeIdsInOrder,
                    Ipv4Address mecIp)
{
    NS_ASSERT_MSG(ueNodeIdsInOrder.size() >= ueNodes.GetN(),
                  "UE id list must align with allUeNodes order");
    Ptr<ns3::ndn::L3Protocol> mecL3 = mecNode->GetObject<ns3::ndn::L3Protocol>();
    NS_ASSERT_MSG(mecL3 != nullptr, "MEC must have NDN installed before SetupUdpFacesForV2i");

    std::cout << "  Setting up NDN V2I over UDP/IPv4 (" << ueNodes.GetN() << " UEs → MEC " << mecIp
              << ", UE port " << NDN_V2I_UDP_PORT_UE << ", MEC ports " << 6363 << "…)" << std::endl;

    nfd::face::GenericLinkService::Options opts;
    opts.allowFragmentation = true;
    opts.allowReassembly = true;
    opts.allowCongestionMarking = true;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        const std::string& nodeId = ueNodeIdsInOrder.at(i);
        Ptr<Node> ueNode = ueNodes.Get(i);
        auto ipIt = ueIpAddresses.find(nodeId);
        NS_ASSERT_MSG(ipIt != ueIpAddresses.end(), "Missing UE IPv4 for " << nodeId);
        Ipv4Address ueIp = ipIt->second;
        const uint16_t mecListenPort = static_cast<uint16_t>(6363u + i);

        Ptr<ns3::ndn::L3Protocol> ueL3 = ueNode->GetObject<ns3::ndn::L3Protocol>();
        NS_ASSERT_MSG(ueL3 != nullptr, "NDN L3 missing on UE node " << nodeId);

        auto ueTransport = std::make_unique<V2iIpv4UdpTransport>(
            ueNode, ueIp, NDN_V2I_UDP_PORT_UE, mecIp, mecListenPort, V2iUdpSocketRole::kUe);
        auto ueLink = std::make_unique<nfd::face::GenericLinkService>(opts);
        auto ueFace = std::make_shared<nfd::face::Face>(std::move(ueLink), std::move(ueTransport));
        ueL3->addFace(ueFace);
        // Broad /v2x at cost 1; MEC-only prefixes at cost 0 so they beat any competing P2P or
        // neighbor FIB entries (ConsumerCacheFriendly: /v2x/traffic/info, /v2x/traffic/count,
        // /v2x/safety). Extra-specific names further pin the UDP face for LPM.
        ns3::ndn::FibHelper::AddRoute(ueNode, ::ndn::Name("/v2x"), ueFace, 1);
        ns3::ndn::FibHelper::AddRoute(ueNode, ::ndn::Name("/v2x/traffic"), ueFace, 0);
        ns3::ndn::FibHelper::AddRoute(ueNode, ::ndn::Name("/v2x/traffic/info"), ueFace, 0);
        ns3::ndn::FibHelper::AddRoute(ueNode, ::ndn::Name("/v2x/traffic/count"), ueFace, 0);
        ns3::ndn::FibHelper::AddRoute(ueNode, ::ndn::Name("/v2x/safety"), ueFace, 0);

        auto mecTransport = std::make_unique<V2iIpv4UdpTransport>(
            mecNode, mecIp, mecListenPort, ueIp, NDN_V2I_UDP_PORT_UE, V2iUdpSocketRole::kMec);
        auto mecLink = std::make_unique<nfd::face::GenericLinkService>(opts);
        auto mecFace = std::make_shared<nfd::face::Face>(std::move(mecLink), std::move(mecTransport));
        mecL3->addFace(mecFace);

        if (nodeId.rfind("veh_", 0) == 0) {
            uint32_t vix = static_cast<uint32_t>(std::stoul(nodeId.substr(4)));
            ns3::ndn::FibHelper::AddRoute(mecNode, ::ndn::Name("/v2x/v2i/vehicle/" + std::to_string(vix)),
                                          mecFace, 1);
        }
    }

    std::cout << "  ✓ NDN V2I UDP faces installed (simulated IP path: NR/EPC/PGW)" << std::endl;
}
