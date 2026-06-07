
---

# Guide: Extending Veins 5.3.1 for SUMO 1.22

**Prerequisites:** Veins 5.3.1 and SUMO 1.22 installed.

This guide details the modifications required in the Veins source code to expose specific TraCI variables needed for:
1.  **Collision Detection:** Retrieving real-time collision lists.
2.  **Topology Graphing:** Getting lane counts and junction connectivity (Incoming/Outgoing edges) for proactive caching logic.

---

## Part 1: Collision Detection Fix

To correctly retrieve the list of colliding vehicles from the SUMO simulation, you must update the `TraCICommandInterface` to use the correct variable constant.

### Step 1.1: Update the Header File
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.h`

Ensure the function `getCollidingVehiclesIDList` is declared in the `public` section of the `TraCICommandInterface` class.

```cpp
class VEINS_API TraCICommandInterface : public HasLogProxy
{
public:
    // ... existing public members ...

    // Retrieves the list of IDs of currently colliding vehicles
    std::list<std::string> getCollidingVehiclesIDList();

    // ... existing public members ...
};
```

### Step 1.2: Update the Source File
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.cc`

Add the implementation. Ensure you use `VAR_COLLIDING_VEHICLES_IDS` (0x81).

```cpp
std::list<std::string> TraCICommandInterface::getCollidingVehiclesIDList()
{
    // VAR_COLLIDING_VEHICLES_IDS = 0x81
    return genericGetStringList(
        CMD_GET_SIM_VARIABLE,       // Command Group
        "",                         // Object ID (Empty for simulation-wide variables)
        VAR_COLLIDING_VEHICLES_IDS, // Variable ID
        RESPONSE_GET_SIM_VARIABLE   // Expected Response ID
    );
}
```

> **Note:** If `VAR_COLLIDING_VEHICLES_IDS` is undefined, check `TraCIConstants.h` and ensure it is defined as `0x81`.

---

## Part 2: Road Lane Count

To build the topology graph, the `CentralRSUManager` needs to iterate through *every* lane of a road (Edge). This requires a function to retrieve the total number of lanes.

### Step 2.1: Update the Header File
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.h`

Inside the `Road` class definition:

```cpp
class VEINS_API Road {
public:
    // ... existing methods ...
    // NEW: Returns the number of lanes on this edge
    int getLaneNumber(); 
};
```

### Step 2.2: Implement the Function
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.cc`

```cpp
int TraCICommandInterface::Road::getLaneNumber()
{
    // Uses CMD_GET_EDGE_VARIABLE with VAR_LANE_INDEX (0x52)
    // In the context of an Edge, 0x52 returns the lane count
    return traci->genericGetInt(CMD_GET_EDGE_VARIABLE, roadId, VAR_LANE_INDEX, RESPONSE_GET_EDGE_VARIABLE);
}
```

---

## Part 3: Junction Connectivity (Incoming/Outgoing)

Veins 5.3.1 does not natively expose the Incoming/Outgoing edge lists for junctions, which are critical for the Upstream Topology logic. We must add the constants and wrapper methods.

### Step 3.1: Define Constants
**File:** `veins/src/veins/modules/mobility/traci/TraCIConstants.h`

Add the hex codes for the new variables if they are missing.

```cpp
const uint8_t VAR_INCOMING_EDGES = 0x7b;
const uint8_t VAR_OUTGOING_EDGES = 0x7c;

```

### Step 3.2: Update the Header File
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.h`

Inside the `Junction` class definition:

```cpp
class VEINS_API Junction {
public:
    // ... existing members ...
    
    // NEW: Topology Methods
    std::list<std::string> getIncomingEdges();
    std::list<std::string> getOutgoingEdges();

protected:
    // ...
};
```

### Step 3.3: Implement the Functions
**File:** `veins/src/veins/modules/mobility/traci/TraCICommandInterface.cc`

```cpp
std::list<std::string> TraCICommandInterface::Junction::getIncomingEdges() {
    return traci->genericGetStringList(
        CMD_GET_JUNCTION_VARIABLE, 
        junctionId, 
        VAR_INCOMING_EDGES, 
        RESPONSE_GET_JUNCTION_VARIABLE
    );
}

std::list<std::string> TraCICommandInterface::Junction::getOutgoingEdges() {
    return traci->genericGetStringList(
        CMD_GET_JUNCTION_VARIABLE, 
        junctionId, 
        VAR_OUTGOING_EDGES, 
        RESPONSE_GET_JUNCTION_VARIABLE
    );
}
```

---

## Part 4: Rebuild

For the changes to take effect, you must recompile the Veins framework. Navigate to the root directory of your Veins installation (where the `Makefile` is located) and run:

```bash
make clean
make
```