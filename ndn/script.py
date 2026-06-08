python3 -c "
import json, os

print('Numerology Comparison — 50s Seed 2')
print('=' * 55)
header = 'Metric'.ljust(30) + 'Num1'.rjust(8) + 'Num2'.rjust(8) + 'Num3'.rjust(8)
print(header)
print('-' * 55)

data = {}
for num in [1, 2, 3]:
    path = '/home/rajesh/v2x/ndn/results/metrics_50s_num' + str(num) + '_seed2.json'
    if os.path.exists(path):
        with open(path) as f:
            data[num] = json.load(f)
    else:
        data[num] = None
        print('Missing: num' + str(num))

def val(num, *keys):
    d = data[num]
    if d is None: return 'N/A'
    for k in keys:
        if d is None: return 'N/A'
        d = d.get(k) if isinstance(d, dict) else None
    if isinstance(d, float): return str(round(d, 3))
    return str(d) if d is not None else 'N/A'

rows = [
    ('--- LATENCY ---',        None),
    ('V2I RTT avg (ms)',       ['qos','e2e_latency_avg_ms']),
    ('V2I RTT min (ms)',       ['qos','e2e_latency_min_ms']),
    ('V2I RTT max (ms)',       ['qos','e2e_latency_max_ms']),
    ('V2I P50 (ms)',           ['qos','e2e_latency_p50_ms']),
    ('V2I P95 (ms)',           ['qos','e2e_latency_p95_ms']),
    ('V2I P99 (ms)',           ['qos','e2e_latency_p99_ms']),
    ('V2I jitter (ms)',        ['qos','jitter_ms']),
    ('V2V RTT avg (ms)',       ['ndn','e2e_latency','v2v_roundtrip_ms']),
    ('--- NDN FORWARDING ---', None),
    ('ISR (%)',                ['qos','interest_satisfaction_ratio_percent']),
    ('Satisfied',              ['ndn','forwarding_rates','satisfied_interests']),
    ('Timed out',              ['ndn','forwarding_rates','timed_out_interests']),
    ('NACKs',                  ['ndn','forwarding_rates','nacks']),
    ('--- CACHING ---',        None),
    ('CS hits',                ['ndn','forwarder_counters','ndn_cs_hits']),
    ('CS misses',              ['ndn','forwarder_counters','ndn_cs_misses']),
    ('CS hit rate (%)',        None),
    ('CS entries (final)',     ['ndn','content_store','mec_entries']),
    ('--- DELIVERY ---',       None),
    ('PDR (%)',                ['qos','packet_delivery_ratio_percent']),
    ('Throughput (kbps)',      ['qos','throughput_kbps']),
    ('--- SAFETY ---',         None),
    ('Collision warnings',     ['safety','collision_warnings']),
    ('Emergency brakes',       ['safety','emergency_brake_commands']),
    ('Min TTC (s)',            ['safety','ttc','min_seconds']),
    ('NDN safety messages',    ['safety','ndn_messages_published']),
    ('--- SIMULATION ---',     None),
    ('Wall clock (s)',         ['simulation','wall_clock_time_seconds']),
    ('Real-time factor',       ['simulation','real_time_factor']),
]

for name, keys in rows:
    if keys is None:
        if name == None:
            # CS hit rate special case
            vals = []
            for num in [1,2,3]:
                d = data[num]
                if d is None:
                    vals.append('N/A'.rjust(8))
                    continue
                fc = d.get('ndn',{}).get('forwarder_counters',{})
                hits   = fc.get('ndn_cs_hits', 0)
                misses = fc.get('ndn_cs_misses', 0)
                total  = hits + misses
                hr = str(round(hits/total*100, 2)) if total > 0 else 'N/A'
                vals.append(hr.rjust(8))
            print('CS hit rate (%)'.ljust(30) + vals[0] + vals[1] + vals[2])
        else:
            print()
            print(name)
        continue
    v1 = val(1, *keys)
    v2 = val(2, *keys)
    v3 = val(3, *keys)
    print(name.ljust(30) + v1.rjust(8) + v2.rjust(8) + v3.rjust(8))
" 2>/dev/null
