#ifndef V2I_METRICS_HOOK_H
#define V2I_METRICS_HOOK_H

/**
 * @file v2i_metrics_hook.h
 * @brief Minimal declaration for recording V2I RTT without pulling metrics_collector.h
 *        (which uses `using namespace ns3` and breaks ndnSIM includes in app .cc files).
 *
 * Implemented in metrics_collector.cc (calls RecordLatencySample(ms, "v2i")).
 */
void RecordV2iAppLayerRttSample(double latencyMs);

#endif
