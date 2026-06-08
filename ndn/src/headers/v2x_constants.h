#ifndef V2X_CONSTANTS_H //Check if V2X_CONSTANTS_H is not defined
#define V2X_CONSTANTS_H //If not define it

#include <string>

#ifdef HOST //check if macro named HOST is already defined
# undef HOST //if defined, undefine it to avoid conflicts
#endif
#ifdef PORT
# undef PORT
#endif



// Network constants
inline const std::string HOST = "127.0.0.1"; // Server address at which omenetpp is runing
inline constexpr int PORT = 9998; // Server port for socket communication
inline constexpr int BUFFER_SIZE = 62814; // Maximum UDP packet size
inline const std::string SYNC_MESSAGE = "TIME_SYNC\n";

// 5G NR Radio Configuration (parameterized for multi-run comparison)
// numerology: 0=15kHz, 1=30kHz, 2=60kHz, 3=120kHz, 4=240kHz SCS per 3GPP TS 38.211
inline constexpr uint32_t NR_NUMEROLOGY_DEFAULT = 1;  // 30 kHz SCS — matches run-simple-cosim.sh default
inline constexpr uint32_t NR_NUMEROLOGY_LEGACY = 4;   // retained for backward compat; not used in runs
inline constexpr double NR_CENTRAL_FREQ_HZ = 3.5e9;       // 3.5 GHz C-Band (FR1)
inline constexpr double NR_BANDWIDTH_HZ = 100e6;           // 100 MHz bandwidth

// 5G NR Channel Model Configuration
// Pathloss: 3GPP UMa-LoS (Urban Macro Line-of-Sight)
// Fading: 3GPP Spatial Channel Model (3GPP38.901) with multipath fading
inline const std::string NR_CHANNEL_MODEL = "3GPP-UMa-LoS";
inline constexpr bool NR_FADING_ENABLED = true;
inline const std::string NR_PATHLOSS_MODEL = "3GPP Urban Macro (UMa) Pathloss";
inline const std::string NR_SPECTRUM_PROPAGATION_MODEL = "ThreeGppSpectrumPropagationLossModel (3GPP 38.901)";

// ============================================================================
// V2V COMMUNICATION CONFIGURATION
// ============================================================================
// IMPORTANT: V2V uses PointToPoint emulation, NOT 3GPP PC5 sidelink.
// This is an intentional design choice for NDN protocol evaluation.
//
// Parameters below represent IDEALIZED V2V connectivity:
// - No radio channel effects (fading, shadowing, interference)
// - No resource scheduling (Mode 1/2)
// - No hidden terminal problem
//
// Valid for: NDN caching studies, content routing, app-layer protocols
// Invalid for: PC5 performance claims, radio resource efficiency
// ============================================================================

inline constexpr double V2V_LINK_DATARATE_MBPS = 100.0;  // Idealized capacity
inline constexpr double V2V_LINK_DELAY_MS = 1.0;         // Idealized latency
inline constexpr uint32_t V2V_NEIGHBOR_COUNT = 2;        // Neighbors per vehicle

inline const std::string V2V_EMULATION_NOTE = 
    "V2V uses P2P emulation (not 3GPP PC5). Valid for NDN protocol studies only.";

// Simulation Seeding Configuration
// Use RngSeedManager::SetSeed() for NS-3 random streams
// NOTE: This only affects NS-3 side randomness (5G NR scheduling, NDN caching, etc.)
// OMNeT++/SUMO vehicle mobility uses its own seed - controlled separately in OMNeT++ config
inline constexpr uint32_t DEFAULT_SEED = 1;

// Safety Application Configuration
// Co-sim runs in lock-step with OMNeT++; ultra-fine 5ms polling was creating
// large redundant workloads with unchanged mobility snapshots.
inline constexpr double RSU_CHECK_INTERVAL_MS = 50.0;      // RSU checks positions every 50ms
inline constexpr double VEHICLE_QUERY_INTERVAL_MS = 50.0;  // Vehicles query traffic info every 50ms
inline constexpr double ARCHA_COLLISION_CHECK_INTERVAL_MS = 50.0; // Arch-A collision checks every 50ms
inline constexpr double COLLISION_RISK_DISTANCE_M = 50.0; // Distance to start monitoring
inline constexpr double HEADING_TOLERANCE_RAD = 0.35;     // ~20 degrees tolerance for "same direction"

// ============================================================================
// RSU Latency Configuration (Documented Assumption)
// ============================================================================
// RSUs are wired to 5G core, eliminating air interface delay.
// Vehicle path: UE → gNB (3ms air) → Core (1ms) → MEC (1ms) = 5ms typical
// RSU path: RSU → Core (1ms) → MEC (1ms) = 2ms typical
// Ratio: 2/5 = 0.4 (conservative estimate)
// Citation: 3GPP TS 23.287 V16.6.0 (2021-09) - V2X Architecture
inline constexpr double RSU_LATENCY_RATIO = 0.4;
inline const std::string RSU_LATENCY_RATIO_CITATION = 
    "RSU wired backhaul eliminates Uu air interface delay (3GPP TS 23.287)";

// ============================================================================
// TTC Safety Thresholds (ETSI EN 302 637-2 V1.4.1)
// ============================================================================
// Time-to-Collision thresholds for Cooperative Awareness Messages
// Citation: ETSI EN 302 637-2 V1.4.1 (2019-04) - CAM Generation Rules
inline constexpr double TTC_CRITICAL_S = 0.5;   // Emergency brake required
inline constexpr double TTC_HIGH_S = 1.5;       // Hard brake recommended
inline constexpr double TTC_MEDIUM_S = 3.0;     // Slow down advised
inline constexpr double TTC_LOW_S = 5.0;        // Maintain awareness
inline constexpr double TTC_THRESHOLD_S = TTC_MEDIUM_S;  // Default threshold (alias for compatibility)
inline const std::string TTC_THRESHOLD_CITATION = 
    "ETSI EN 302 637-2 V1.4.1 Section 6.1.3 - CAM generation rules";

// ============================================================================
// Minimum Safe Distance (NHTSA Guidelines)
// ============================================================================
// Following distance for collision avoidance
// Citation: NHTSA DOT HS 812 115 - Forward Collision Warning System
inline constexpr double NHTSA_MIN_SAFE_DISTANCE_M = 15.0;
inline constexpr double MIN_SAFE_DISTANCE_M = NHTSA_MIN_SAFE_DISTANCE_M;  // Alias for compatibility
inline const std::string MIN_DISTANCE_CITATION = 
    "NHTSA DOT HS 812 115 - FCW Performance Specifications";

// ============================================================================
// NDN Freshness Period Configuration (Data Validity Based)
// ============================================================================
// Freshness periods based on DATA VALIDITY requirements, not query intervals.
// Citation: ETSI EN 302 637-2 - data validity window considerations
// NOTE: Query intervals should be <= freshness for cache benefit,
// but freshness is set by data semantics, not to engineer hits.
inline constexpr uint32_t TRAFFIC_INFO_FRESHNESS_MS = 500;   // Traffic updates every 500ms
inline constexpr uint32_t VEHICLE_COUNT_FRESHNESS_MS = 1000; // Count updates every 1s
inline constexpr uint32_t SAFETY_WARNING_FRESHNESS_MS = 100; // Safety must be fresh
inline constexpr uint32_t POSITION_FRESHNESS_MS = 200;       // Position validity window (vehicle at 30m/s moves 6m in 200ms)

// Metrics Collection Configuration
// Warm-up period to exclude cold-start artifacts and transient conditions
// Excludes: FIB population, 5G connection setup, initial cache misses
inline constexpr double WARMUP_PERIOD_S = 2.0;            // Exclude first 2s from latency statistics


#endif
