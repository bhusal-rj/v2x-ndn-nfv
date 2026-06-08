# Cross-Layer NFV and Named Data Networking Integration for 5G NR-V2X Edge Services

Research artifact accompanying the IEEE Transactions manuscript in [`paper.tex`](paper.tex).

| | |
|---|---|
| **Authors** | Rajesh Bhusal, Amit Aacharya, Prakash Mahato, Arun C. Bhusal, Babu R. Dawadi, Pietro Manzoni |
| **Affiliations** | Tribhuvan University Institute of Engineering, Pulchowk Campus, Nepal; Universitat Politècnica de València, Spain |
| **Corresponding author** | Babu R. Dawadi — [baburd@ioe.edu.np](mailto:baburd@ioe.edu.np) |
| **Funding** | Tribhuvan University Research Directorate, National Priority Area Research Grant TU-NPAR-081/82-MRG-01 |

---

## Abstract

This repository archives the co-simulation framework and NS-3 source code used in the paper. The study couples **SUMO** mobility, **OMNeT++/Veins** NFV/MEC orchestration, and **ns-3/ndnSIM** networking to compare a documented **IP/UDP baseline** with **NDN over UDP/IP** across NR numerologies μ ∈ {1, 2, 3} on a **200 s** urban trace (35 vehicles, single gNodeB, 23–25 active RSU instances).

| Stack | Key result (three-seed means, μ = 1 / 2 / 3) |
|-------|---------------------------------------------|
| **IP/UDP V2I median RTT** | 8.945 / 5.636 / 4.558 ms |
| **NDN V2I median FullDelay** | 15.987 / 10.094 / ~7.416 ms |
| **NDN CS hit rate** | 55.93–57.02 % across numerologies |
| **NDN ISR** | 99.67–99.97 %; no NACKs observed |
| **Latency tiers** | NDN at μ=3 meets the 10 ms median reference; both stacks satisfy TS 22.185 (100 ms) |

Full methodology, tables, and limitations are in [`paper.tex`](paper.tex) (Appendix A: Reproducibility).

---

## Repository Contents

This archive is intentionally minimal. It contains the manuscript, NS-3 simulation sources, and run scripts. External simulators and the OMNeT++ control-plane component are **not** bundled here.

```
ndn/
├── paper.tex              # Manuscript (IEEEtran, self-contained)
├── README.md              # This file
├── run-simple-cosim.sh    # Main co-simulation launcher
├── experiment.sh          # Optional batch runner (edit parameters before use)
├── script.py              # Quick numerology comparison utility
├── wscript                # NS-3 build configuration
├── src/                   # NS-3 / ndnSIM simulation source
│   ├── simple_ndn.cc      # Entry point and co-simulation loop
│   ├── nr_5g_setup.cc     # 5G NR infrastructure
│   ├── ndn_setup.cc       # NDN stack configuration
│   ├── metrics_collector.cc
│   ├── arch_a_integration.cc
│   ├── mec_edge_filter.cc           # UC1: Edge filtering
│   ├── traffic_light_preemption.cc  # UC2: Emergency preemption
│   ├── collision_avoidance.cc       # UC3: TTC collision avoidance
│   ├── accident_notification.cc     # UC4: Topology-aware notification
│   └── headers/
└── experiments/
    └── manifest_template.yaml   # Experiment configuration template
```

**Not included in git** (required separately):

| Component | Role | Expected location |
|-----------|------|-------------------|
| `v2x_leader/` | OMNeT++/Veins NFV control plane | `$PROJECT_ROOT/v2x_leader` |
| NS-3 + ndnSIM + 5G-LENA | Network data plane | `~/ndn2/ns-3` (default in run script) |
| OMNeT++ | Discrete-event simulator | `~/omnetpp` |
| SUMO + Veins | Traffic mobility | `~/sumo`, `~/veins` |

---

## System Overview

Lock-step co-simulation across three layers:

```
SUMO ──► OMNeT++/Veins (NFV/MANO, safety apps) ◄── TCP:9998 ──► ns-3/ndnSIM (IP or NDN over 5G NR)
```

| Path | Description |
|------|-------------|
| **V2I (Uu)** | Vehicle UE → PHY → MAC → RLC → PDCP → IP → gNodeB → EPC → MEC |
| **NDN carriage** | NDN TLV over UDP/IPv4 on the same V2I route |
| **V2V** | Point-to-point emulation (100 Mb/s, 1 ms) — **not** 3GPP PC5 sidelink |

### Safety use cases

| UC | Function | Activation in paper dataset |
|----|----------|----------------------------|
| UC1 | MEC/RSU edge filtering and duplicate suppression | All numerologies |
| UC2 | Emergency traffic-light preemption | μ = 3 only |
| UC3 | TTC-based collision avoidance | All numerologies |
| UC4 | Topology-aware accident notification (proactive cache warming) | All numerologies |

---

## Prerequisites

Tested on **Ubuntu 20.04 / 22.04 LTS**. Recommended: 8+ CPU cores, 16+ GB RAM.

| Software | Version | Purpose |
|----------|---------|---------|
| NS-3 | 3.37 | Network simulator |
| ndnSIM | 2.8 | NDN stack |
| ndn-cxx | 0.8.0 | NDN C++ library |
| NFD | 22.02 | NDN forwarder |
| 5G-LENA (nr) | 5g-lena-v2.4 | 5G NR module |
| OMNeT++ | 5.x / 6.x | Control-plane simulator |
| SUMO | 1.x | Traffic simulator |
| Veins | compatible with OMNeT++ | V2X framework for OMNeT++ |
| Python | 3.8+ | Optional post-run utilities |

### Setup summary

1. Install NS-3, ndnSIM, and 5G-LENA under `~/ndn2/ns-3`.
2. Install OMNeT++, SUMO, and Veins; add their binaries to `PATH`.
3. Place the OMNeT++ control-plane project at `~/v2x/ndn/v2x_leader` and build it (`make -j$(nproc)`).
4. Clone this repository to `~/v2x/ndn`.
5. On each run, `run-simple-cosim.sh` copies `src/` into `~/ndn2/ns-3/scratch/ndn-v2x` and builds the NS-3 target.

Edit path variables at the top of `run-simple-cosim.sh` if your layout differs from the defaults above.

---

## Quick Start

```bash
cd ~/v2x/ndn

# Build OMNeT++ control plane (once)
cd v2x_leader && make -j$(nproc) && cd ..

# Run one NDN configuration from the paper grid
./run-simple-cosim.sh --duration 200 --numerology 1 --seed 1
```

### Run-script options

| Flag | Default | Paper value |
|------|---------|-------------|
| `--duration` | 100 | **200** |
| `--numerology` | 1 | **1, 2, or 3** |
| `--seed` | 1 | **1, 2, or 3** |
| `--quick` | — | 30 s smoke test |
| `--wall-timeout` | disabled | Optional wall-clock limit (seconds) |

A 200 s run typically requires several hours of wall-clock time (RTF ≈ 0.015).

---

## Reproducing Paper Experiments

Parameters match **Appendix A (Reproducibility)** in `paper.tex`:

- Duration: **200 s**
- Numerology: **μ ∈ {1, 2, 3}**
- Seeds: **1, 2, 3**
- Warm-up excluded from analysis: **first 2 s**
- Scenario: urban grid, **35 vehicles**, **1 gNodeB**, **3.5 GHz / 100 MHz**, 3GPP UMa-LoS

### Paper experiment grid (18 runs)

| Dataset | Runs | Primary metrics |
|---------|------|-----------------|
| NDN | 9 (3 numerologies × 3 seeds) | FullDelay, ISR, PDR, CS hit rate, safety counters |
| IP/UDP baseline | 9 (3 numerologies × 3 seeds) | UDP RTT, PDR, throughput |
| NFV telemetry | OMNeT++ traces | RSU lifecycle, scale-out/in, service-flow counters |

### Reproduce full NDN grid

```bash
cd ~/v2x/ndn
for num in 1 2 3; do
  for seed in 1 2 3; do
    ./run-simple-cosim.sh --duration 200 --numerology $num --seed $seed
  done
done
```

### Build the manuscript

`paper.tex` is self-contained (inline bibliography). From the repository root:

```bash
pdflatex paper.tex
pdflatex paper.tex   # second pass for cross-references
```

If you add a `main.tex` wrapper later, use that as the build entry point instead.

---

## Output Files

After each run, outputs are collected under `results/` with the tag `{duration}s_num{N}_seed{S}`:

| File | Content |
|------|---------|
| `simulation_metrics_{tag}.json` | NS-3 QoS, latency, NDN forwarding counters |
| `arch_a_metrics_{tag}.json` | Safety use-case metrics |
| `app-delays-trace_{tag}.txt` | ndnSIM FullDelay samples |
| `rate-trace_{tag}.txt` | NDN L3 rate tracer |
| `cs-trace_{tag}.txt` / `ndn-cs-trace_{tag}.txt` | Content Store events |
| `omnet_{tag}_live.log` | OMNeT++ / NFV telemetry |
| `ns3_{tag}_live.log` | NS-3 simulation log |
| `omnet_messages_{tag}.jsonl` | Lockstep co-simulation messages |
| `run_metadata.txt` | Run provenance (timestamp, seed, commit, paths) |

Compare numerology results across existing JSON files:

```bash
python3 script.py
```

---

## Metric Definitions

Aligned with Section IV of `paper.tex`:

| KPI | IP | NDN |
|-----|----|-----|
| V2I latency | UDP request–reply RTT | ndnSIM application `FullDelay` |
| V2V latency | P2P emulation counters | FullDelay samples below 5 ms |
| PDR | IP QoS logs | NDN QoS records |
| ISR | N/A | satisfied / (satisfied + timed out) |
| CS hit rate | structurally zero | NDN forwarder counters |
| NFV lifecycle | N/A | OMNeT++ RSU VNF counters |

Analysis excludes the 2 s warm-up, zero-latency local CS hits, and samples above 200 ms.

Statistical reporting in the paper uses mean, sample standard deviation, and 95 % t-intervals over seeds 1–3 per numerology.

---

## Limitations

See Section VII (*Threats to Validity and Limitations*) in `paper.tex`. Summary:

- **V2I** uses the full 5G NR stack (5G-LENA) with 3GPP UMa-LoS; latency claims are trace-backed.
- **V2V** is point-to-point emulation, not PC5 sidelink — do not cite as PC5 radio performance.
- **IP RTT ≠ NDN FullDelay** — different instrumentation points; compare as aligned application-level observables, not identical probes.
- **NFV telemetry** confirms RSU lifecycle and service-flow counters; emergency preemption delay is not inferred from NFV logs alone.
- **Scope**: single scenario, one vehicle density, one gNodeB, nine runs per stack.

---

## Citation

```bibtex
@article{bhusal2026crosslayer,
  title   = {Cross-Layer {NFV} and Named Data Networking Integration for
             {5G} {NR-V2X} Edge Services},
  author  = {Bhusal, Rajesh and Aacharya, Amit and Mahato, Prakash and
             Bhusal, Arun C. and Dawadi, Babu R. and Manzoni, Pietro},
  journal = {IEEE Transactions on Intelligent Transportation Systems},
  year    = {2026},
  note    = {Research artifact: \url{<repository-url>}}
}
```

Replace `<repository-url>` with the final archive DOI or repository URL before submission.

---

## License

Contact the authors for licensing information. Third-party components (NS-3, ndnSIM, 5G-LENA, OMNeT++, SUMO, Veins) are subject to their respective licenses.

---

## Contact

**Babu R. Dawadi** (corresponding author) — [baburd@ioe.edu.np](mailto:baburd@ioe.edu.np)  
Tribhuvan University Institute of Engineering, Pulchowk Campus, Lalitpur, Nepal
