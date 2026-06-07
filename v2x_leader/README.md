# V2X Simulation Framework using OMNeT++, Veins, and NS-3

This project presents a comprehensive simulation framework for Vehicle-to-Everything (V2X) communication scenarios. It leverages the power of **OMNeT++** for network simulation, the **Veins** framework for realistic vehicular mobility modeling, and **SUMO** for traffic simulation. A key feature of this project is its ability to synchronize with an external **NS-3** client for sophisticated time control and data exchange, enabling complex co-simulation environments.

## Overview

The simulation environment is designed to model and analyze various V2X applications, such as safety warnings, traffic information systems, and autonomous vehicle coordination. By integrating OMNeT++ with SUMO via the Veins framework, the simulation benefits from detailed network protocol modeling and realistic vehicle movement patterns. The co-simulation capability with an NS-3 client via a TCP socket allows for the integration of other simulation domains or external hardware-in-the-loop setups.

## Main Features

*   **TCP Connection to NS-3 Client**: Establishes a socket connection on port `9998` for robust time synchronization and data exchange with an external NS-3 simulator.
*   **Data Exchange**: Facilitates the transfer of mobility and accident data between the OMNeT++ simulation and the NS-3 client.
*   **Time Synchronization**: Implements a pause/resume mechanism for the OMNeT++ simulation based on time synchronization messages from the NS-3 client, ensuring all components operate in a coordinated manner.
*   **SUMO Integration**: Utilizes SUMO for microscopic traffic simulation, providing realistic vehicle mobility patterns for the V2X network simulation.

## Architecture

The simulation framework is built upon a modular architecture:
1.  **OMNeT++**: The core network simulator where V2X communication protocols and applications are modeled.
2.  **Veins**: A framework that connects OMNeT++ with SUMO, providing mobility models and V2X-specific modules.
3.  **SUMO**: The traffic simulator responsible for generating and managing vehicle movements within a defined road network.
4.  **NS-3 Client (External)**: An external network simulator that connects to the OMNeT++ environment to control the simulation time and exchange relevant data.

The communication between OMNeT++ and the NS-3 client is handled by a custom `socketInterface` module that manages the TCP connection and data serialization.

## Prerequisites

Ensure you have the following software installed and properly configured before proceeding:

*   **OMNeT++**: A modular, component-based C++ simulation library and framework.
*   **Veins**: The open-source framework for running vehicular network simulations. Ensure it is correctly installed and its path is known.
*   **SUMO**: A microscopic, open-source traffic simulation package. Required for running mobility simulations.

## Setup and Execution

Follow these steps to set up and run the simulation project.

### 1. Clone the Repository

First, clone the project repository to your local machine.

```bash
git clone https://github.com/V2X-Eval/v2x_leader.git
cd v2x_leader
```
### 2 Run the Simulation

Simply execute the prepared shell script to build and run:
```bash
chmod +x run_simulation.sh
./run_simulation.sh
```

### 3. Connect NS3 Client

Start the NS3 client and connect it to port 9998 for time synchronization and data exchange.For test simply run
```bash
python ns3Client.py
```



See docs/implementation.md for detailed workflow and architecture.