/**
 * @file ndn_consumer_cache_friendly.cc
 * @brief Cache-friendly NDN Consumer for V2X shared content
 * 
 * Unlike ConsumerCbr which appends unique sequence numbers to each Interest,
 * this consumer sends Interests with FIXED names (no sequence numbers).
 * This enables realistic NDN caching behavior:
 * 
 * - Vehicle A requests /v2x/traffic/info → MISS (fetched from MEC)
 * - Data cached at intermediate node (gNB)
 * - Vehicle B requests /v2x/traffic/info → HIT (served from cache!)
 * 
 * This is the realistic behavior for shared V2X content like:
 * - Traffic light status
 * - Road conditions
 * - Emergency alerts
 * - Map/infrastructure data
 * 
 * RTT measurement uses time-bucketed Interest names (prefix + b=<bucket_id>) so pending
 * entries align with exact Data names after PIT aggregation; see header comment.
 * 
 * @author V2X-NDN Research Team
 * @date March 2026
 */

#include "ndn_consumer_cache_friendly.h"

#include "ns3/ptr.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/integer.h"
#include "ns3/double.h"

#include "ns3/ndnSIM/helper/ndn-stack-helper.hpp"
#include "ns3/ndnSIM/helper/ndn-fib-helper.hpp"

#include <ndn-cxx/lp/tags.hpp>

#include <string>

namespace {

::ndn::Name
MakeBucketedInterestName(const ::ndn::Name& prefix, ns3::Time now, ns3::Time interestLifetime)
{
    const int64_t nowMs = now.GetMilliSeconds();
    const int64_t bucketMs = interestLifetime.GetMilliSeconds();
    int64_t bucketId = 0;
    if (bucketMs > 0) {
        bucketId = nowMs / bucketMs;
    }
    ::ndn::Name n(prefix);
    // Two components (/b/<id>) avoid URI "b=..." typed-component parsing errors on round-trip.
    n.append("b");
    n.appendNumber(static_cast<uint64_t>(bucketId < 0 ? 0 : bucketId));
    return n;
}

} // namespace

NS_LOG_COMPONENT_DEFINE("ndn.ConsumerCacheFriendly");

namespace ns3 {
namespace ndn {

NS_OBJECT_ENSURE_REGISTERED(ConsumerCacheFriendly);

TypeId
ConsumerCacheFriendly::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::ndn::ConsumerCacheFriendly")
            .SetGroupName("Ndn")
            .SetParent<App>()
            .AddConstructor<ConsumerCacheFriendly>()
            .AddAttribute("Prefix", "Interest prefix (sent as-is, no seq number appended)",
                          StringValue("/"),
                          MakeNameAccessor(&ConsumerCacheFriendly::m_prefix),
                          MakeNameChecker())
            .AddAttribute("Frequency", "Frequency of Interest packets (Hz)",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ConsumerCacheFriendly::m_frequency),
                          MakeDoubleChecker<double>())
            .AddAttribute("LifeTime", "Interest lifetime",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&ConsumerCacheFriendly::m_interestLifeTime),
                          MakeTimeChecker())
            .AddAttribute("MustBeFresh", "Set MustBeFresh flag on Interests",
                          BooleanValue(false),  // Default false for cache-friendly behavior
                          MakeBooleanAccessor(&ConsumerCacheFriendly::m_mustBeFresh),
                          MakeBooleanChecker())
            .AddTraceSource("SentInterest", "Interest sent",
                            MakeTraceSourceAccessor(&ConsumerCacheFriendly::m_sentInterest),
                            "ns3::ndn::ConsumerCacheFriendly::InterestCallback")
            .AddTraceSource("ReceivedData", "Data received",
                            MakeTraceSourceAccessor(&ConsumerCacheFriendly::m_receivedData),
                            "ns3::ndn::ConsumerCacheFriendly::DataCallback")
            // AppDelayTracer connects to ApplicationList/*/FirstInterestDataDelay (same as ndn::Consumer).
            .AddTraceSource("FirstInterestDataDelay",
                            "Delay from Interest send to Data receive (per-Interest tracking)",
                            MakeTraceSourceAccessor(&ConsumerCacheFriendly::m_firstInterestDataDelay),
                            "ns3::ndn::Consumer::FirstInterestDataDelayCallback");
    return tid;
}

ConsumerCacheFriendly::ConsumerCacheFriendly()
    : m_frequency(1.0)
    , m_mustBeFresh(false)  // Allow cache hits by accepting cached data
    , m_interestsSent(0)
    , m_dataReceived(0)
    , m_timeouts(0)
{
    NS_LOG_FUNCTION(this);
}

ConsumerCacheFriendly::~ConsumerCacheFriendly()
{
    NS_LOG_FUNCTION(this);
}

void
ConsumerCacheFriendly::StartApplication()
{
    NS_LOG_FUNCTION(this);
    App::StartApplication();
    
    m_pendingByName.clear();
    
    // Schedule first Interest
    ScheduleNextPacket();
    
    NS_LOG_INFO("ConsumerCacheFriendly started: prefix=" << m_prefix 
                << " freq=" << m_frequency << "Hz"
                << " mustBeFresh=" << m_mustBeFresh);
}

void
ConsumerCacheFriendly::StopApplication()
{
    NS_LOG_FUNCTION(this);
    
    // Cancel pending events
    if (m_sendEvent.IsRunning()) {
        Simulator::Cancel(m_sendEvent);
    }
    
    size_t pendingRemaining = 0;
    for (const auto& kv : m_pendingByName) {
        pendingRemaining += kv.second.size();
    }
    m_timeouts += pendingRemaining;
    m_pendingByName.clear();
    
    NS_LOG_INFO("ConsumerCacheFriendly stopped: sent=" << m_interestsSent 
                << " received=" << m_dataReceived 
                << " timeouts=" << m_timeouts);
    
    App::StopApplication();
}

void
ConsumerCacheFriendly::CleanupExpiredInterests()
{
    Time now = Simulator::Now();
    Time expiryThreshold = m_interestLifeTime;

    for (auto it = m_pendingByName.begin(); it != m_pendingByName.end();) {
        auto& dq = it->second;
        while (!dq.empty()) {
            Time sendTime = dq.front().first;
            if ((now - sendTime) > expiryThreshold) {
                dq.pop_front();
                m_timeouts++;
                NS_LOG_DEBUG("Interest expired (timeout): name=" << it->first
                                                                  << " age="
                                                                  << (now - sendTime).GetMilliSeconds()
                                                                  << "ms");
            } else {
                break;
            }
        }
        if (dq.empty()) {
            it = m_pendingByName.erase(it);
        } else {
            ++it;
        }
    }

    // Remove stale bucket keys — bucket names contain the bucket_id
    // which encodes the time window. Any bucket older than 2× lifetime
    // is guaranteed to have no valid pending entries.
    // Parse bucket_id from key name suffix /b/<id> and compare to now.
    const int64_t bucketMs = m_interestLifeTime.GetMilliSeconds();
    if (bucketMs <= 0) {
        return;
    }
    const int64_t nowBucket = Simulator::Now().GetMilliSeconds() / bucketMs;
    for (auto it = m_pendingByName.begin(); it != m_pendingByName.end();) {
        const std::string& key = it->first;
        const auto bpos = key.rfind("/b/");
        if (bpos != std::string::npos) {
            try {
                const int64_t keyBucket = std::stoll(key.substr(bpos + 3));
                if (nowBucket - keyBucket > 2) {
                    for (const auto& entry : it->second) {
                        (void)entry;
                        m_timeouts++;
                    }
                    it = m_pendingByName.erase(it);
                    continue;
                }
            } catch (...) {
            }
        }
        ++it;
    }
}

void
ConsumerCacheFriendly::ScheduleNextPacket()
{
    if (!m_active) return;
    
    // Schedule next Interest based on frequency
    double interval = 1.0 / m_frequency;
    m_sendEvent = Simulator::Schedule(Seconds(interval), 
                                       &ConsumerCacheFriendly::SendPacket, this);
}

void
ConsumerCacheFriendly::SendPacket()
{
    if (!m_active) return;
    
    NS_LOG_FUNCTION(this);

    if (m_pendingByName.size() > 1000) {
        NS_LOG_ERROR("m_pendingByName size=" << m_pendingByName.size()
                     << " — possible bucket leak, clearing stale entries");
    }
    
    // Clean up expired pending Interests before sending new one
    CleanupExpiredInterests();
    
    Time sendTime = Simulator::Now();
    const ::ndn::Name interestName = MakeBucketedInterestName(m_prefix, sendTime, m_interestLifeTime);
    const std::string bucketKey = interestName.toUri();
    auto interest = std::make_shared<::ndn::Interest>(interestName);
    
    // Set Interest parameters
    interest->setInterestLifetime(::ndn::time::milliseconds(m_interestLifeTime.GetMilliSeconds()));
    interest->setMustBeFresh(m_mustBeFresh);
    interest->setCanBePrefix(false);  // Exact name match for CS (prefix Interest bypasses exact CS in NFD 22.02)
    
    // Generate unique nonce to avoid loop detection (using timestamp-based value)
    uint32_t nonce = static_cast<uint32_t>(Simulator::Now().GetNanoSeconds() & 0xFFFFFFFF);
    interest->setNonce(nonce);
    
    uint64_t seqNum = m_interestsSent;
    auto& pendDeque = m_pendingByName[bucketKey];
    if (pendDeque.size() >= MAX_PENDING_INTERESTS) {
        NS_LOG_WARN("Pending Interest queue full for bucket (" << MAX_PENDING_INTERESTS
                    << "), dropping oldest entry: " << bucketKey);
        pendDeque.pop_front();
        m_timeouts++;
    }
    pendDeque.push_back(std::make_pair(sendTime, seqNum));

    NS_LOG_DEBUG("Sending Interest: " << interest->getName() << " seq=" << seqNum
                                        << " mustBeFresh=" << interest->getMustBeFresh()
                                        << " pending_this_bucket=" << pendDeque.size());
    
    // Send via face
    m_transmittedInterests(interest, this, m_face);
    m_appLink->onReceiveInterest(*interest);
    
    m_interestsSent++;
    m_sentInterest(interest, this);
    
    // Schedule next packet
    ScheduleNextPacket();
}

void
ConsumerCacheFriendly::OnData(std::shared_ptr<const ::ndn::Data> data)
{
    if (!m_active) return;
    
    NS_LOG_FUNCTION(this << data->getName());
    
    App::OnData(data);
    
    CleanupExpiredInterests();

    // Match pending using the Data name only — never recompute a bucket from
    // Simulator::Now(). A send at t=199ms may map to bucket 0 while Data arrives
    // at t=206ms (next wall-clock bucket); the Interest/Data name still carries
    // the exact suffix (/b/<id>) from send time, which matches m_pendingByName.
    const std::string dataName = data->getName().toUri();
    Time delay = Time(0);
    uint64_t matchedSeq = 0;

    auto it = m_pendingByName.find(dataName);
    if (it != m_pendingByName.end() && !it->second.empty()) {
        Time sendTime = it->second.front().first;
        matchedSeq = it->second.front().second;
        it->second.pop_front();
        delay = Simulator::Now() - sendTime;
        if (it->second.empty()) {
            m_pendingByName.erase(it);
        }
        NS_LOG_DEBUG("Matched Data to Interest seq=" << matchedSeq
                     << " RTT=" << delay.GetMilliSeconds() << "ms"
                     << " dataName=" << dataName);
    } else {
        NS_LOG_WARN("Received Data with no pending Interest (possible duplicate): " << dataName);
    }
    
    // Get hop count from Data packet
    int32_t hopCount = 0;
    auto hopCountTag = data->getTag<lp::HopCountTag>();
    if (hopCountTag != nullptr) {
        hopCount = *hopCountTag;
    }
    
    // Fire trace source for AppDelayTracer (use matchedSeq for proper tracking)
    const uint32_t seq = static_cast<uint32_t>(matchedSeq);
    // Only fire tracer for non-zero delays — zero means Data was
    // served from local CS without network traversal. Firing with
    // delay=0 pollutes V2I RTT distribution with local cache hits.
    if (delay.GetMilliSeconds() > 0) {
        m_firstInterestDataDelay(this, seq, delay, 0, hopCount);
    }

    m_dataReceived++;
    m_receivedData(data, this);
    
    NS_LOG_INFO("Received Data: " << data->getName() 
                << " freshness=" << data->getFreshnessPeriod().count() << "ms"
                << " rtt=" << delay.GetMilliSeconds() << "ms"
                << " hopCount=" << hopCount);
}

} // namespace ndn
} // namespace ns3
