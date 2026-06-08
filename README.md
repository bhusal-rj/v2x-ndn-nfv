# Cross-Layer NFV and Named Data Networking Integration for 5G NR-V2X Edge Services

Research artifact accompanying the IEEE Transactions manuscript in [`ndn/paper.tex`](ndn/paper.tex).

| | |
|---|---|
| **Authors** | Rajesh Bhusal, Amit Aacharya, Prakash Mahato, Arun C. Bhusal, Babu R. Dawadi, Pietro Manzoni |
| **Affiliations** | Tribhuvan University Institute of Engineering, Pulchowk Campus, Nepal; Universitat Politècnica de València, Spain |
| **Corresponding author** | Babu R. Dawadi — [baburd@ioe.edu.np](mailto:baburd@ioe.edu.np) |
| **Funding** | Tribhuvan University Research Directorate, National Priority Area Research Grant TU-NPAR-081/82-MRG-01 |

---

## Abstract

This repository archives the co-simulation framework used in the paper. It bundles two components:

- **`v2x_leader/`** — OMNeT++/Veins NFV/MEC control plane (SUMO mobility, RSU lifecycle, safety apps)
- **`ndn/`** — NS-3 / ndnSIM network data plane (5G NR V2I, NDN over UDP/IP, safety use-case hooks)

Together they compare a documented **IP/UDP baseline** with **NDN over UDP/IP** across NR numerologies μ ∈ {1, 2, 3} on a **200 s** urban trace (35 vehicles, single gNodeB, 23–25 active RSU instances).

| Stack | Key result (three-seed means, μ = 1 / 2 / 3) |
|-------|---------------------------------------------|
| **IP/UDP V2I median RTT** | 8.945 / 5.636 / 4.558 ms |
| **NDN V2I median FullDelay** | 15.987 / 10.094 / ~7.416 ms |
| **NDN CS hit rate** | 55.93–57.02 % across numerologies |
| **NDN ISR** | 99.67–99.97 %; no NACKs observed |
| **Latency tiers** | NDN at μ=3 meets the 10 ms median reference; both stacks satisfy TS 22.185 (100 ms) |

Full methodology, tables, and limitations are in [`ndn/paper.tex`](ndn/paper.tex) (Appendix A: Reproducibility).

---

## Repository Contents

```
.
├── README.md                  # This file
├── v2x_leader/                # OMNeT++/Veins control plane
│   ├── src/                   # C++ application modules
│   ├── headers/               # Shared types and module headers
│   ├── ned/                   # OMNeT++ network definitions
│   ├── simulations/           # SUMO scenarios (pulchowk, ratnapark)
│   ├── docs/                  # Architecture and implementation notes
│   ├── run_simulation.sh      # Standalone OMNeT++ build and run
│   └── omnetpp.ini            # Simulation configuration
└── ndn/                       # NS-3 / ndnSIM data plane
    ├── paper.tex              # Manuscript (IEEEtran, self-contained)
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

**Not included in git** (you must install these separately before running):

| Component | Role | Expected location |
|-----------|------|-------------------|
| NS-3 + ndnSIM + 5G-LENA | Pre-built network simulator tree with all required modules | `~/ndn2/ns-3` (default in run script) |
| OMNeT++ | Discrete-event simulator | `~/omnetpp` |
| SUMO + Veins | Traffic mobility | `~/sumo`, `~/veins` |

> **Important:** This repository ships only the `ndn/` scratch sources (`simple_ndn.cc` and helpers). It does **not** bundle NS-3, ndnSIM, ndn-cxx, NFD, or 5G-LENA. You must have a working NS-3 installation with ndnSIM and the NR module integrated and built **before** running co-simulation.

---

## System Overview

Lock-step co-simulation across three layers:

```
SUMO ──► OMNeT++/Veins (v2x_leader: NFV/MANO, safety apps) ◄── TCP:9998 ──► ns-3/ndnSIM (ndn: IP or NDN over 5G NR)
```

| Component | Directory | Role |
|-----------|-----------|------|
| **Control plane** | `v2x_leader/` | Vehicle/RSU mobility, NFV orchestration, safety event detection, TCP bridge to NS-3 |
| **Data plane** | `ndn/` | 5G NR V2I stack, NDN forwarding, MEC edge services, metrics collection |

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

## v2x_leader (Control Plane)

The OMNeT++/Veins project models the V2X control plane: SUMO-driven vehicle mobility, RSU lifecycle management, NFV service orchestration, and safety application logic. It exposes a TCP socket on port **9998** for lock-step time synchronization and JSON message exchange with the NS-3 client.

### Key modules

| Module | Purpose |
|--------|---------|
| `socketInterface` | TCP bridge to NS-3 (time sync, mobility, safety commands) |
| `centralRSUManager` | NFV/MANO decisions, proactive cache target selection |
| `vehicleApp` / `rsuApp` | Per-node safety and traffic applications |
| `timeSynchronizer` | Pause/resume coordination with NS-3 |
| `metricsCollector` | NFV lifecycle and service-flow telemetry |

### Standalone run (OMNeT++ only)

```bash
cd v2x_leader
chmod +x run_simulation.sh
./run_simulation.sh
```

For architecture details see [`v2x_leader/docs/SYSTEM_REFERENCE.md`](v2x_leader/docs/SYSTEM_REFERENCE.md) and [`v2x_leader/docs/IMPLEMENTATION.md`](v2x_leader/docs/IMPLEMENTATION.md).

---

## ndn (Data Plane)

The `ndn/` directory contains NS-3 scratch sources — not a standalone NS-3 tree. They implement the 5G NR network stack, NDN forwarding over UDP/IP, and the four safety use-case integrations. On each co-simulation run, `run-simple-cosim.sh` copies `ndn/src/` into your external NS-3 tree at `~/ndn2/ns-3/scratch/ndn-v2x` and builds the `ndn-v2x` target against your pre-installed ndnSIM and 5G-LENA modules.

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

### Required NS-3 / ndnSIM source tree

You must obtain and build a complete NS-3 installation with ndnSIM and 5G-LENA integrated. The versions tested for this artifact are:

| Package | Version | Install location (typical) |
|---------|---------|----------------------------|
| NS-3 | **3.37** | `~/ndn2/ns-3` |
| ndnSIM | **2.8** | `~/ndn2/ns-3/src/ndnSIM` |
| ndn-cxx | **0.8.0** | linked by ndnSIM build |
| NFD | **22.02** | linked by ndnSIM build |
| 5G-LENA (`nr` module) | **5g-lena-v2.4** | `~/ndn2/ns-3/src/nr` |

The following NS-3 modules must be present and enabled in your tree (used by `ndn/src/simple_ndn.cc`):

| Module | Purpose in this study |
|--------|----------------------|
| `ndnSIM` | NDN stack, NFD forwarder, PIT/FIB/CS, application delay tracers |
| `nr` (5G-LENA) | 5G NR gNodeB/UE, EPC, numerology, beamforming |
| `internet` | IPv4/UDP carriage for NDN faces and IP/UDP baseline |
| `mobility` | Vehicle and RSU position updates from OMNeT++ |
| `point-to-point` | V2V emulated links |
| `antenna` | NR antenna models |
| `network`, `core` | NS-3 base infrastructure |

**Verify your installation** before cloning this repository:

```bash
cd ~/ndn2/ns-3
./waf configure --enable-examples
./waf build
./waf --run scratch/ndnSIM/ndn-congestion-alt-topo-plugin   # ndnSIM smoke test
```

If ndnSIM or the `nr` module is missing, `./waf build` will fail once `run-simple-cosim.sh` copies the `ndn/` sources into `scratch/ndn-v2x/`.

Follow the official integration guides:

- [ndnSIM installation](https://ndnsim.net/current/getting-started.html) (ndn-cxx + NFD + ndnSIM into NS-3)
- [5G-LENA (nr module)](https://5g-lena.cttc.es/) (add `nr` under `ns-3/src/`)

### Setup summary

1. **First:** build NS-3 3.37 with ndnSIM 2.8 and 5G-LENA v2.4 under `~/ndn2/ns-3` (see above).
2. Clone this repository (e.g. to `~/v2x`).
3. Install OMNeT++, SUMO, and Veins; add their binaries to `PATH`.
4. Build the control plane: `cd v2x_leader && make -j$(nproc)`.
5. Run co-simulation from `ndn/`: `./run-simple-cosim.sh` copies scratch sources into your NS-3 tree and launches both simulators.

Edit path variables at the top of `ndn/run-simple-cosim.sh` if your layout differs from the defaults (`PROJECT_ROOT`, `OMNET_DIR`, `NS3_PATH`).

---

## Quick Start

```bash
# From repository root
cd v2x_leader && make -j$(nproc) && cd ..

# Run one NDN configuration from the paper grid
cd ndn
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

Parameters match **Appendix A (Reproducibility)** in `ndn/paper.tex`:

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
| NFV telemetry | OMNeT++ traces (`v2x_leader`) | RSU lifecycle, scale-out/in, service-flow counters |

### Reproduce full NDN grid

```bash
cd ndn
for num in 1 2 3; do
  for seed in 1 2 3; do
    ./run-simple-cosim.sh --duration 200 --numerology $num --seed $seed
  done
done
```

### Build the manuscript

`ndn/paper.tex` is self-contained (inline bibliography). From the `ndn/` directory:

```bash
cd ndn
pdflatex paper.tex
pdflatex paper.tex   # second pass for cross-references
```

---

## Output Files

After each run, outputs are collected under `ndn/results/` with the tag `{duration}s_num{N}_seed{S}`:

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
cd ndn
python3 script.py
```

---

## Metric Definitions

Aligned with Section IV of `ndn/paper.tex`:

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

See Section VII (*Threats to Validity and Limitations*) in `ndn/paper.tex`. Summary:

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
