# System Reference: V2X Leader Implementation

**Version:** 1.0
**Scope:** C++ Class References, Data Structures, and Module Logic
**Role:** Technical Manual for the OMNeT++ Control Plane

## 1. Shared Data Structures (`headers/V2XTypes.h`)

These structures define the data contract between modules and the JSON serialization layer.

### 1.1 Communication & Protocol Structs

| Struct | Description |
| :--- | :--- |
| **`MANOQuery`** | Represents an incoming request from the 5G Core (ns-3) asking for proactive caching locations. |
| **`MANOResponse`** | The decision computed by `CentralRSUManager`, containing the list of target RSUs. |
| **`TrafficPreemptionCommand`** | The URLLC command sent from ns-3 (Edge/Cloud) to a specific Junction RSU to change signal phases. |

```cpp
struct MANOQuery {
    std::string queryId; 
    std::string contentName;
    std::string laneId;     // The edge where the event occurred
    std::string sourceRSU;  // The RSU that reported the event
    double posX;
    double posY;
};

struct MANOResponse {
    std::string queryId;
    std::string contentName;
    std::vector<std::string> rsuIds; // List of selected RSU IDs
};

struct TrafficPreemptionCommand {
    std::string targetId;    // The RSU ID (e.g., "rsu_tls_J1")
    std::string tlsId;       // The Traffic Light ID
    std::string laneId;      // Where the emergency vehicle is
    std::string requestorId; // The ambulance ID
    double originTime;       // Timestamp of request generation
};
```

### 1.2 Safety & Event Structs

| Struct | Description |
| :--- | :--- |
| **`AccidentData`** | Contains details of a collision detected by a vehicle. Used for both reporting (Uplink) and receiving warnings (Downlink). |
| **`PendingSafetyCmd`** | Wrapper used in the `SocketInterface` queue to store incoming safety commands before processing. |

```cpp
struct AccidentData {
    std::string sourceType;   // "vehicle", "rsu", "proactive_cache"
    std::string laneId;
    std::string crashedVehId; // The ID of the partner/obstacle
    double posX;
    double posY;
    std::string accidentId;   // Canonical sorted ID
    double originTime;        // Time of collision
};

struct PendingSafetyCmd {
    std::string targetId;     // The OMNeT++ vehicle that must react
    AccidentData accidentData;
};
```

### 1.3 State & Topology Structs

| Struct | Description |
| :--- | :--- |
| **`NodeState`** | Snapshot of a vehicle's physics (position, speed, heading) for the current tick. |
| **`RsuState`** | Snapshot of an RSU, including Traffic Light status if applicable. Includes `operator==` for differential updates. |
| **`RSUCandidate`** | Internal descriptor used by `CentralRSUManager` to manage RSU lifecycle (Static vs Dynamic). |
| **`JunctionInfo`** | Topology cache storing outgoing edges for a junction. |
| **`UpstreamConnection`** | Graph node representing road connectivity (`Source -> Target`). |

---

## 2. Module Reference

### 2.1 Communication Gateway (`SocketInterface`)

**Role:** The bridge between OMNeT++ and ns-3. It manages the simulation lifecycle and TCP stream.

**Lifecycle Methods:**
*   `startTcpServer()`: Opens port 9998 and waits for connection.
*   `startVeinsManager()`: **Critical Entry Point.** Instantiated only after a client connects. It triggers the creation of `TraCIScenarioManagerForker`.
*   `createCentralModules()`: Instantiates `CentralRSUManager` and `MetricsCollector` once TraCI is available.

**Outbound Methods (Serialization):**
*   `sendNodeStates()`: Iterates `MobilityManager` data to send vehicle positions.
*   `sendRSUStates()`: Sends RSU positions and TLS states.
*   `sendAccidentData(AccidentData)`: Serializes collision events.
*   `sendManoResponse(MANOResponse)`: Serializes MANO decisions.
*   `sendTrafficPreemption(TrafficPreemptionCommand)`: Forwards emergency requests to the network.
*   `sendEndOfStep()`: Sends the synchronization footer to unlock ns-3.

**Inbound Processing:**
*   `handleClientData()`: Reads from the TCP buffer, handles fragmentation, parses JSON, and populates processing queues.
*   `processSafetyMessages()`: Flushes the queue of incoming braking commands to `VehicleApp`.
*   `processManoQueue()`: Flushes the queue of `vnf_query` to `CentralRSUManager`.
*   `processTrafficPreemptionQueue()`: Flushes the queue of URLLC commands to `RsuApp`.

### 2.2 Orchestration (`CentralRSUManager`)

**Role:** MANO Entity and Topology Manager.

**Topology Construction:**
*   `buildTopologyGraph()`: Iterates all lanes using `getLinks()` to build the Upstream/Downstream graph.
*   `buildJunctionCandidates()`: Identifies valid intersections for **Backbone RSU** placement.
*   `buildInFillCandidates()`: Interpolates **Dynamic RSU** positions along road geometry.
*   `getPointOnPolyline(...)`: Helper for geometric interpolation.

**Runtime Logic:**
*   `updateRSUPlacement()`: Runs periodically. Checks vehicle density against thresholds to trigger Scale-Out/Scale-In.
*   `createRSU(...)` / `deactivateRSU(...)`: Handles dynamic module creation/deletion.
*   `handleVNFQuery(...)`: (Implied) Processes MANO queries using the topology graph.

### 2.3 User Equipment (`VehicleApp`)

**Inheritance:** `cSimpleModule`, `cListener`.

**Role:** Logic for standard and emergency vehicles.

**Key Methods:**
*   `receiveSignal(...)`: Listens for `mobilityStateChanged`. Triggers `handlePositionUpdate`.
*   `handlePositionUpdate(cObject*)`: The main heartbeat. Checks status and resets flags.
*   `checkCollisionEventForVehicle()`: Queries TraCI for collisions. Generates `AccidentData` if involved.
*   `checkTrafficLightProximity()`: **Emergency Vehicles Only.** Scans ahead for Red lights and triggers `sendTrafficPreemption`.
*   `handleNetworkAccident(AccidentData)`: Receives command from ns-3. Calculates braking distance and calls `traci->slowDown`.

### 2.4 Infrastructure (`RsuApp`)

**Role:** Edge VNF logic.

**Key Methods:**
*   `handlePreemptionCommand(TrafficPreemptionCommand)`:
    *   Validates if this RSU controls the target TLS.
    *   Executes the phase change (Green Wave) via TraCI.

### 2.5 Utilities

**TimeSynchronizer:**
*   `advanceStep()`: Unblocks the simulation time, allowing it to progress to `t + stepSize`.

**MetricsCollector:**
*   Listens to signals emitted by all other modules (e.g., `sigVehAccGen`, `sigNetworkLatency`) and writes them to CSV files.

---

## 3. Initialization Workflow

1.  **OMNeT++ Starts:** `SocketInterface` initializes and blocks on `accept()`.
2.  **Client Connects:** ns-3 connects to Port 9998.
3.  **Boot Sequence:** `SocketInterface` calls `startVeinsManager()`.
4.  **TraCI Sync:** Veins connects to SUMO.
5.  **Module Creation:** `createCentralModules()` instantiates the MANO and Metric layers.
6.  **Topology Scan:** `CentralRSUManager` builds the road graph and places Static RSUs.
7.  **Simulation Loop:** The lockstep cycle begins.