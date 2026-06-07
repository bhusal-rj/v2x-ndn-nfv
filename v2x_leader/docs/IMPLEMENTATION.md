
# Implementation — OMNeT++ V2X Simulation (NFV-Orchestrated NDN over 5G)

## Table of Contents
1. [Purpose and Scope](#1-purpose-and-scope)
2. [System Overview](#2-system-overview)
3. [Runtime Phases (Lockstep Synchronization)](#3-runtime-phases-lockstep-synchronization)
4. [Major Modules and Responsibilities](#4-major-modules-and-responsibilities)
5. [Hybrid RSU Placement Strategy](#5-hybrid-rsu-placement-strategy)
6. [Data Frame Specifications](#6-data-frame-specifications)
7. [Operational Workflows](#7-operational-workflows)
8. [Integration Notes](#8-integration-notes)

---

## 1. Purpose and Scope

This document is the authoritative implementation reference for the OMNeT++ Control Plane simulation. It details the architecture where OMNeT++ handles application logic, physics, and NFV orchestration, while running in strict lockstep with an external ns-3 Data Plane via a custom TCP JSON protocol.

**Version:** 3.0 (Consolidated Architecture)

---

## 2. System Overview

### Architecture Diagram (Consolidated)

The system has been streamlined into **6 Core Modules**, removing legacy helpers like `AccidentModule` and `TrafficLightApp`.

```text
┌────────────────────────────────────────────────────────────────────────────┐
│                          External ns-3 Client                              │
│                 (NDN Forwarding, 5G PHY/MAC, Packet Routing)               │
└────────────────────────────────┬───────────────────────────────────────────┘
                                 │ TCP (JSON frames)
                                 │ Port: 9998
                                 ↓
┌────────────────────────────────────────────────────────────────────────────┐
│                      OMNeT++ V2X Control Plane                             │
│ ┌─────────────────────────────────────────────────────────────────────┐    │
│ │ Network Module (V2X)                                                │    │
│ ├─────────────────────────────────────────────────────────────────────┤    │
│ │                                                                     │    │
│ │ ┌──────────────────────────────────────────────────────────────┐    │    │
│ │ │ ORCHESTRATION LAYER                                          │    │    │
│ │ ├──────────────────────────────────────────────────────────────┤    │    │
│ │ │ • CentralRSUManager        - MANO, Topology Graph, Mobility  │    │    │
│ │ │ • SocketInterface          - ns-3 Gateway, Buffering         │    │    │
│ │ │ • TimeSynchronizer         - Lockstep Clock Control          │    │    │
│ │ │ • MetricsCollector         - CSV Statistics                  │    │    │
│ │ └──────────────────────────────────────────────────────────────┘    │    │
│ │                                                                     │    │
│ │ ┌──────────────────────────────────────────────────────────────┐    │    │
│ │ │ INFRASTRUCTURE LAYER (Edge VNFs)                             │    │    │
│ │ ├──────────────────────────────────────────────────────────────┤    │    │
│ │ │ • RsuApp [Backbone]        - Traffic Light Control (TLS)     │    │    │
│ │ │ • RsuApp [In-Fill]         - V2X Relaying (Dynamic)          │    │    │
│ │ └──────────────────────────────────────────────────────────────┘    │    │
│ │                                                                     │    │
│ │ ┌──────────────────────────────────────────────────────────────┐    │    │
│ │ │ USER EQUIPMENT LAYER                                         │    │    │
│ │ ├──────────────────────────────────────────────────────────────┤    │    │
│ │ │ • VehicleApp [Veh 1..M]    - Accident Detect, Safety Action  │    │    │
│ │ └──────────────────────────────────────────────────────────────┘    │    │
│ └─────────────────────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Runtime Phases (Lockstep Synchronization)

### Phase 1: State Transmission (OMNeT++ $\to$ ns-3)
At the beginning of every time step `T`:
1.  **CentralRSUManager** iterates active vehicles and RSUs.
2.  **SocketInterface** serializes and sends:
    *   `mobility_update`: All vehicle positions/speeds.
    *   `rsu_state`: Positions and TLS states of all RSUs.
    *   `end_of_step`: Marker indicating data transmission is complete.
3.  **Polling:** SocketInterface enters a non-blocking poll loop waiting for ns-3.

### Phase 2: Asynchronous Events (Bidirectional)
During the step (or triggered by logic):
*   **Accident:** `VehicleApp` detects crash $\to$ sends `accident_report` to ns-3.
*   **Safety:** ns-3 NDN App receives packet $\to$ sends `safety_command` to OMNeT++.
*   **MANO:** ns-3 Core VNF requests caching $\to$ sends `vnf_query` $\to$ OMNeT++ replies with `mano_decision`.

### Phase 3: Synchronization (ns-3 $\to$ OMNeT++)
1.  ns-3 finishes simulating network traffic for duration `step_size`.
2.  ns-3 sends `time_sync` message.
3.  **SocketInterface** receives sync, processes any pending queued commands, and calls `timeSynchronizer->advanceStep()`.
4.  Simulation time advances to `T + step_size`.

---

## 4. Major Modules and Responsibilities

### 4.1 SocketInterface
**Role:** API Gateway.
*   **Buffering:** Maintains a persistent string accumulator to handle TCP packet fragmentation (handling split JSONs).
*   **Command Queue:** Queues incoming `safety_command` and `vnf_query` messages to ensure they are executed atomically before the time advance.
*   **Routing:**
    *   `safety_command` $\to$ Target `VehicleApp`.
    *   `vnf_query` $\to$ `CentralRSUManager`.
    *   `traffic_preemption_command` $\to$ Target `RsuApp`.

### 4.2 CentralRSUManager (MANO)
**Role:** The "Brain" and Orchestrator.
*   **Mobility Aggregation:** Replaces the old MobilityManager. Collects state from all nodes for the periodic update.
*   **Topology Graph:** Builds an "Upstream/Downstream" graph of the road network using `Lane::getLinks()` to support intelligent caching.
*   **MANO Logic:** Handles `PROACTIVE_CACHE_PLACEMENT` queries. It identifies which RSUs cover the roads feeding into an accident site.
*   **RSU Lifecycle:** Manages the creation/deletion of Dynamic In-Fill RSUs based on vehicle density.

### 4.3 VehicleApp
**Role:** End-User Logic & Physics.
*   **Accident Detection:**
    *   Queries `traci->getCollidingVehiclesIDList()`.
    *   If own ID is present, generates a deduplicated `AccidentReport` (Sorted Pair ID).
    *   Sends report via SocketInterface (triggering the NDN flow in ns-3).
    *   *Note:* Relies on `.sumocfg` settings (`collision.stoptime`) to ensure the vehicle remains active in simulation after a crash.
*   **Safety Reaction:**
    *   Receives `safety_command` from ns-3 (via Socket).
    *   Calculates distance/TTC to the hazard.
    *   Executes `traci->slowDown()` or `stop()`.

### 4.4 RsuApp
**Role:** Infrastructure Edge Node.
*   **Traffic Light Control:**
    *   If initialized as a **Junction RSU**, it holds a pointer to `TraCICommandInterface::Trafficlight`.
    *   Receives Preemption commands from ns-3 (URLLC).
    *   Executes phase changes (Green Wave) for emergency vehicles.

---

## 5. Hybrid RSU Placement Strategy

The system abandons the abstract "Grid" approach for a **Topology-Centric** approach.

### 5.1 Static Backbone (Junction RSUs)
*   **Logic:** Placed at physical Traffic Light locations extracted from SUMO.
*   **Naming:** `rsu_tls_<TLS_ID>` (e.g., `rsu_tls_cluster_J1`).
*   **Function:** Permanent infrastructure, Traffic Control, Anchor points.

### 5.2 Dynamic In-Fill (Elastic Edge)
*   **Logic:** Placed along the geometry of long road segments to fill coverage gaps between junctions.
*   **Algorithm:** Uses `getPointOnPolyline` to interpolate positions every 300m along the road shape.
*   **Lifecycle:** Instantiated only when vehicle density in that segment exceeds a threshold (NFV Scaling).

---

## 6. Data Frame Specifications

### 6.1 Periodic Updates (OMNeT++ $\to$ ns-3)

**Mobility Update**
```json
{
  "message_type": "mobility_update",
  "timestamp": 0.5,
  "payload": [
    {
      "id": "node[0]",
      "x": 35.79,
      "y": 443.23,
      "speed": 12.5, 
      "heading": 1.21,
      "vehicle_type": "DEFAULT_VEHTYPE"
    }
  ],
  "meta": { "count": 1 }
}
```

**RSU State** (Includes TLS context)
```json
{
  "message_type": "rsu_state",
  "timestamp": 9.5,
  "payload": [
    {
      "id": "rsu_j_6",
      "x": 187.85, "y": 129.77,
      "isTrafficLight": 1,
      "tlsId": "cluster_1880066230",
      "trafficLightState": "rrGGgGG"
    },
    {
      "id": "rsu_e_9",
      "x": 339.67, "y": 536.95,
      "isTrafficLight": 0
    }
  ]
}
```

### 6.2 Event Triggers (OMNeT++ $\to$ ns-3)

**Accident Report**
```json
{
  "message_type": "accident_report",
  "timestamp": 28.5,
  "payload": {
    "origin_time": 28.5,
    "accident_id": "crash_node0_28",
    "vehicleId": "node[0]",
    "lane_id": "232351912#0_1",
    "pos_x": 71.87,
    "pos_y": 358.27
  },
  "meta": { "status": "alert" }
}
```
**Emergency Vehicle Request**
```json
{
  "message_type": "traffic_preemption_request",
  "timestamp": 28.5,
  "payload":{
    "tls_id":"434978486",
    "sender_id":"node[7]",
    "lane_id":"271375318#1_0",
    "origin_time":47
    },
  "meta":{
    "priority":"URLLC",
    "description":"Emergency vehicle requesting green light"
  }
}
```
**MANO Decision** (Response to Query)
```json
{
  "message_type": "mano_decision",
  "timestamp": 29.0,
  "payload": {
    "query_id": "q_2850_409",
    "content_name": "/v2x/safety/crash_node0_node1_28",
    "targets": [
      "rsu_j_6",
      "rsu_j_2",
      "rsu_e_9"
    ]
  }
}
```

### 6.3 Incoming Commands (ns-3 $\to$ OMNeT++)

**Safety Command** (Vehicle Reaction)
```json
{
  "message_type": "safety_command",
  "timestamp": 28.51,
  "payload": {
    "target_vehicle_id": "node[1]",
    "source_type": "vehicle",
    "accident_data": {
      "accident_id": "crash_node0_28",
      "origin_time": 28.5,
      "lane_id": "232351912#0_1",
      "pos_x": 71.87,
      "pos_y": 358.27,
      "crashed_vehicle_id": "node[0]"
    }
  }
}
```

**VNF Query** (MANO Request)
```json
{
  "message_type": "vnf_query",
  "timestamp": 28.55,
  "payload": {
    "query_id": "q_2850_983",
    "content_name": "/v2x/safety/crash_node0_28",
    "lane_id": "232351912#0_1",
    "pos_x": 71.87, 
    "pos_y": 358.27,
    "origin_rsu": "rsu_j_4"
  }
}
```

**Traffic Preemption Command**
```json
{
  "message_type": "traffic_preemption_command",
  "timestamp": 21,
    "payload": {
    "target_rsu_id": "rsi_j_5",
    "tls_id": "cluster_1880066230",
    "lane_id":"232351912#0_1" ,
    "requestor_id":"node[0]",
    "origin_time":"20.5"
  }                               
}

```

---

## 7. Operational Workflows

### Flow A: Accident Reporting (V2V/V2I)
1.  **Detection:** `VehicleApp` checks `traci->getCollidingVehiclesIDList()`.
2.  **Uplink:** If involved, sends `accident_report` JSON to ns-3.
3.  **Network:** ns-3 Node creates NDN Interest `/v2x/safety/...` and broadcasts.
4.  **Downlink:** ns-3 Consumer receives Interest $\to$ sends `safety_command` to OMNeT++.
5.  **Actuation:** `VehicleApp` receives command $\to$ applies braking physics.

### Flow B: Proactive Caching (MANO)
1.  **Trigger:** ns-3 Core VNF receives accident data.
2.  **Query:** ns-3 sends `vnf_query` to OMNeT++.
3.  **Logic:** `CentralRSUManager` checks **Upstream Graph** (Roads feeding into the accident lane).
4.  **Decision:** OMNeT++ returns `mano_decision` with a list of RSUs covering those feeder roads.
5.  **Execution:** ns-3 Core pushes content to those RSU caches.

### Flow C: Emergency Preemption (URLLC)
1.  **Trigger:** Emergency vehicle approaches Red Light.
2.  **Network:** ns-3 sends command to the specific RSU ID (e.g., `rsu_tls_J1`) handling that junction.
3.  **Actuation:** `RsuApp` (J1) calls `trafficLight->setPhase()` to force Green.

---

## 8. Integration Notes

*   **TCP Fragmentation:** The `SocketInterface` implements a persistent string buffer. It splits incoming data by `\n` to handle cases where multiple JSONs arrive in a single TCP packet.
*   **Race Conditions:** We rely on SUMO configuration `<collision.stoptime value="300"/>` to ensure vehicles are not deleted immediately upon crash, allowing `VehicleApp` to report successfully.
*   **Topology Graph:** The `CentralRSUManager` iterates all lanes at startup using a custom `getLinks()` implementation to build the Upstream/Downstream connectivity map for MANO logic.