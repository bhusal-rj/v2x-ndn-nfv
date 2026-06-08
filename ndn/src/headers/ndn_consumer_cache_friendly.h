/**
 * @file ndn_consumer_cache_friendly.h
 * @brief Cache-friendly NDN Consumer for V2X shared content
 * 
 * This consumer sends Interests WITHOUT sequence numbers, enabling realistic
 * NDN caching behavior where multiple vehicles can receive the same cached Data.
 * 
 * Key difference from ConsumerCbr:
 * - ConsumerCbr: /v2x/traffic/info/seq=0, /seq=1, /seq=2... (NO cache hits possible)
 * - This consumer: /v2x/traffic/info, /v2x/traffic/info... (CACHE HITS!)
 * 
 * RTT Measurement (time-bucketed names):
 * - Appends /b/<bucket_id> (two name components) using floor(now_ms / LifeTime_ms)
 * - Pending sends are keyed by full bucketed name URI; OnData matches exact Data name
 * - Avoids FIFO mismatch when PIT aggregation merges fixed-name Interests at the forwarder
 * 
 * @author V2X-NDN Research Team
 * @date March 2026
 */

#ifndef NDN_CONSUMER_CACHE_FRIENDLY_H
#define NDN_CONSUMER_CACHE_FRIENDLY_H

#include "ns3/ndnSIM/apps/ndn-app.hpp"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/traced-callback.h"

#include <deque>
#include <string>
#include <unordered_map>

namespace ns3 {
namespace ndn {

/**
 * @brief Cache-friendly NDN Consumer that sends Interests WITHOUT sequence numbers
 * 
 * This enables realistic NDN caching for shared V2X content:
 * - Traffic light status
 * - Road conditions
 * - Emergency alerts
 * - Map/infrastructure data
 */
class ConsumerCacheFriendly : public App
{
public:
    static TypeId GetTypeId();

    ConsumerCacheFriendly();
    virtual ~ConsumerCacheFriendly();

    // Callbacks for tracing
    typedef void (*InterestCallback)(std::shared_ptr<const ::ndn::Interest>, App*);
    typedef void (*DataCallback)(std::shared_ptr<const ::ndn::Data>, App*);
    /// Same signature as ns3::ndn::Consumer::FirstInterestDataDelay (required for AppDelayTracer).
    typedef void (*FirstInterestDataDelayCallback)(Ptr<App> app, uint32_t seqno, Time delay,
                                                   uint32_t retxCount, int32_t hopCount);

    /// URI for metrics prefix registration (RegisterNdnAppPrefixes / AppDelayTracer).
    std::string GetInterestPrefixUri() const { return m_prefix.toUri(); }

    // Statistics getters
    uint64_t GetInterestsSent() const { return m_interestsSent; }
    uint64_t GetDataReceived() const { return m_dataReceived; }
    uint64_t GetTimeouts() const { return m_timeouts; }
    double GetSatisfactionRatio() const { 
        return m_interestsSent > 0 ? (double)m_dataReceived / m_interestsSent : 0.0; 
    }

protected:
    virtual void StartApplication() override;
    virtual void StopApplication() override;
    
    /**
     * @brief Called when Data is received
     */
    virtual void OnData(std::shared_ptr<const ::ndn::Data> data) override;

private:
    /**
     * @brief Schedule the next Interest packet
     */
    void ScheduleNextPacket();
    
    /**
     * @brief Send an Interest packet (without sequence number!)
     */
    void SendPacket();
    
    /**
     * @brief Clean up expired pending Interests (older than InterestLifetime)
     */
    void CleanupExpiredInterests();

    // Configuration
    ::ndn::Name m_prefix;           ///< Interest prefix (sent as-is)
    double m_frequency;             ///< Interest frequency in Hz
    Time m_interestLifeTime;        ///< Interest lifetime
    bool m_mustBeFresh;             ///< MustBeFresh flag

    // State
    EventId m_sendEvent;            ///< Next send event
    
    /**
     * Pending (sendTime, seqNum) per bucketed Interest name URI.
     * FIFO per name is correct for same PIT/cache line; keys separate LifeTime windows.
     */
    std::unordered_map<std::string, std::deque<std::pair<Time, uint64_t>>> m_pendingByName;
    
    /// Maximum pending Interests to track (prevents unbounded memory growth)
    static constexpr size_t MAX_PENDING_INTERESTS = 100;

    // Statistics
    uint64_t m_interestsSent;
    uint64_t m_dataReceived;
    uint64_t m_timeouts;

    // Trace sources
    TracedCallback<std::shared_ptr<const ::ndn::Interest>, App*> m_sentInterest;
    TracedCallback<std::shared_ptr<const ::ndn::Data>, App*> m_receivedData;
    TracedCallback<Ptr<App>, uint32_t, Time, uint32_t, int32_t> m_firstInterestDataDelay;
};

} // namespace ndn
} // namespace ns3

#endif // NDN_CONSUMER_CACHE_FRIENDLY_H
