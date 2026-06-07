import socket
import time
import json
import re
import random 

# --- Configuration ---
HOST = '127.0.0.1'
PORT = 9998
BUFFER_SIZE = 65536 
SOCKET_PATH = '/tmp/v2x_sim_socket'
USE_TCP=True
SYNC_MESSAGE = json.dumps({"message_type": "time_sync", "payload": {}}).encode('utf-8') + b'\n'

def run_client():
    print("--- ns-3 Robust Client (Safety + MANO Orchestration) ---")
    if USE_TCP:
        print(f"Connecting to {HOST}:{PORT}...")
    else: 
        print(f"Connecting to {SOCKET_PATH}...")
    # Cycle options for the source type
    SOURCE_TYPES = ["vehicle", "rsu", "proactive_cache"]
    # Counter to rotate through the types
    msg_counter = 0 
    
    # State tracking for MANO context
    active_rsu_ids = []  # To store IDs like "rsu[0]", "rsu[1]"
    tls_to_rsu_map = {}       # Map: "cluster_J1" -> "rsu_tls_cluster_J1"

    accumulator = ""
    family = socket.AF_INET if USE_TCP else socket.AF_UNIX
    address = (HOST, PORT) if USE_TCP else SOCKET_PATH


    try:
        with socket.socket(family, socket.SOCK_STREAM) as s:
            s.connect(address)
        

            print(f"✅ Connected to OMNeT++")

            with open('docs/data.txt', 'w') as data_file:
                while True:
                    try:
                        chunk = s.recv(BUFFER_SIZE)
                    except socket.error:
                        break
                    
                    if not chunk:
                        print("🔌 Server disconnected.")
                        break

                    accumulator += chunk.decode('utf-8')

                    while '\n' in accumulator:
                        message_str, accumulator = accumulator.split('\n', 1)
                        if not message_str.strip(): continue

                        # Log INCOMING message
                        data_file.write(message_str + "\n")
                        data_file.flush()

                        is_terminated = False
                        needs_sync = False

                        try:
                            json_data = json.loads(message_str)
                            msg_type = json_data.get('message_type', 'unknown')
                            timestamp = json_data.get('timestamp', 0.0)

                            # --- 1. CLOCK DRIVERS ---
                            if msg_type == "end_of_step":
                                needs_sync = True

                            # --- 2. RSU STATE TRACKING (Needed for MANO Query) ---
                            elif msg_type == 'rsu_state':
                                # Update our list of active RSUs so queries use valid IDs
                                payload = json_data.get('payload', [])
                                for rsu in payload:
                                    r_id = rsu.get('id')
                                    t_id = rsu.get('tlsId') # Extract TLS ID if present
                                    
                                    active_rsu_ids.append(r_id)
                                    
                                    if t_id:
                                        tls_to_rsu_map[t_id] = r_id

                            elif msg_type == 'mobility_update':
                                count = json_data.get('meta', {}).get('count', 0)
                                print(f"-> [Mobility] Time: {timestamp}s | Vehs: {count}")
                            # --- EVENT: TRAFFIC PREEMPTION (URLLC) ---
                            elif msg_type == 'traffic_preemption_request':
                                print(f"🚑 [URLLC] Received Preemption Request at {timestamp}s!")
                                
                                incoming_payload = json_data.get('payload', {})
                                tls_id = incoming_payload.get('tls_id')
                                sender_id = incoming_payload.get('sender_id')
                                lane_id = incoming_payload.get('lane_id')
                                origin_time=incoming_payload.get("origin_time")

                                # 1. Select the RSU that handles this request
                                # In a real scenario, this would be based on geo-location.
                                # For testing, we pick a random active RSU, or fallback to rsu[0].
                                # --- INTELLIGENT SELECTION ---
                                selected_rsu = None
                                
                                # 1. Try to find the specific RSU for this Traffic Light
                                if tls_id in tls_to_rsu_map:
                                    selected_rsu = tls_to_rsu_map[tls_id]
                                    print(f"   🎯 Match Found: TLS '{tls_id}' is controlled by '{selected_rsu}'")
                                else:
                                    # 2. Fallback (e.g. In-Fill RSUs acting as relays, or map not ready)
                                    if active_rsu_ids:
                                        selected_rsu = random.choice(active_rsu_ids)
                                        print(f"   ⚠️ No direct match for TLS '{tls_id}'. Using random relay: '{selected_rsu}'")
                                    else:
                                        print("   ❌ No active RSUs found to handle request!")
                                        continue
                                
                                # Simulate 5G Uplink/Edge Processing Delay (Low Latency ~5ms)
                                time.sleep(0.005)

                                print(f"   ⚡ Sending Switch Command to Controller: {selected_rsu}")
                                
                                # 2. Construct the Command
                                preemption_cmd = {
                                    "message_type": "traffic_preemption_command",
                                    "timestamp": timestamp + 0.005,
                                    "payload": {
                                        "target_rsu_id": selected_rsu,
                                        "tls_id": tls_id,
                                        "lane_id": lane_id,
                                        "requestor_id": sender_id,
                                        "origin_time":origin_time
                                    }
                                    
                                }
                                
                                json_out = json.dumps(preemption_cmd)
                                
                                # 3. Log and Send
                                data_file.write("[SENT_PREEMPT] " + json_out + "\n")
                                s.sendall(json_out.encode('utf-8') + b'\n')
                                
                                # Short pause to ensure buffer separation
                                time.sleep(0.01)

                            # --- 3. EVENT: ACCIDENT REPORT (Triggers Both Paths) ---
                            elif msg_type == 'accident_report':
                                print(f"💥 [ACCIDENT] Received report at {timestamp}s!")
                                
                                # Extract Context for both Safety and MANO
                                incoming_payload = json_data.get('payload', {})
                                acc_id = incoming_payload.get('accident_id')
                                lane_id = incoming_payload.get('lane_id')
                                pos_x = incoming_payload.get('pos_x')
                                pos_y = incoming_payload.get('pos_y')
                                
                                crashed_veh_id = incoming_payload.get('vehicle_id', 'node[0]')
                                target_ids=[]
                                # --- PATH 1: SAFETY COMMAND (Edge/V2V) ---
                                # (Existing logic ...)
                                match = re.search(r'node\[(\d+)\]', crashed_veh_id)
                                if match:
                                    center_id = int(match.group(1))
                                    # Target 1: Previous vehicle (if exists)
                                    if center_id > 0: 
                                        target_ids.append(f"node[{center_id - 1}]")
                                    # Target 2: Next vehicle
                                    target_ids.append(f"node[{center_id + 1}]")
                                    # Target 3: Next-Next vehicle
                                    target_ids.append(f"node[{center_id + 2}]")
                                else:
                                    # Fallback
                                    target_ids = ["node[0]", "node[1]", "node[2]"]

                                # Simulate Network Delay
                                time.sleep(0.1) 

                                for target in target_ids:
                                    # ... (Sending safety_command code remains same) ...
                                    # For brevity, reusing the safety command block you already have:
                                    current_source = SOURCE_TYPES[msg_counter % 3]
                                    msg_counter +=1 
                                    
                                    safety_command = {
                                        "message_type": "safety_command",
                                        "timestamp": timestamp + 0.01,
                                        "payload": {
                                            "target_vehicle_id": target,
                                            "source_type": current_source,
                                            "accident_data": {
                                                "accident_id": acc_id,
                                                "origin_time": incoming_payload.get('origin_time'),
                                                "lane_id": lane_id,
                                                "pos_x": pos_x,
                                                "pos_y": pos_y,
                                                "crashed_vehicle_id": crashed_veh_id
                                            }
                                        }
                                    }
                                    
                                    json_safe = json.dumps(safety_command)
                                    data_file.write( json_safe + "\n")
                                    s.sendall(json_safe.encode('utf-8') + b'\n')
                                    time.sleep(0.02)

                                # --- PATH 2: MANO QUERY (Core Network) ---
                                print("   ☁️  [5G CORE] Initiating MANO Placement Query...")
                                
                                # Pick a random RSU as the "Origin" (Reporter)
                                origin_rsu = random.choice(active_rsu_ids) if active_rsu_ids else "rsu[0]"
                                
                                # Construct the VNF Query
                                vnf_query = {
                                "message_type": "vnf_query",
                                "timestamp": timestamp + 0.05,
                                 "payload": {
                                   "query_id": f"q_{int(timestamp*100)}_{random.randint(100,999)}",
                                    "content_name": f"/v2x/safety/{acc_id}",
                                    "lane_id": incoming_payload.get('lane_id'), # or road_id
                                    "pos_x": float(pos_x),
                                    "pos_y": float(pos_y),
                                     "origin_rsu": origin_rsu
                                            }
                                          }

                                json_query = json.dumps(vnf_query)
                                data_file.write(json_query + "\n")
                                s.sendall(json_query.encode('utf-8') + b'\n')
                                print(f"   -> Query Sent: origin={origin_rsu}, lane={lane_id}")


                            # --- 4. HANDLE MANO DECISION (Reply from OMNeT++) ---
                            elif msg_type == 'mano_decision':
                             payload = json_data.get('payload', {})
                             q_id = payload.get('query_id')
                             targets = payload.get('targets', []) # This is now a list of strings
    
                             print(f"   ⚡ [MANO] Targets for {q_id}: {targets}")
                                # In real ns-3, we would now inject data into these RSUs

                            elif msg_type == 'termination_signal':
                                print("-> Terminating.")
                                is_terminated = True

                        except json.JSONDecodeError:
                            print(f"-> [Error] Partial/Invalid JSON")

                        if is_terminated: break

                        if needs_sync:
                            s.sendall(SYNC_MESSAGE)
                    
                    if is_terminated:
                        break

    except FileNotFoundError:
        print(f"❌ Connection failed. Socket file '{SOCKET_PATH}' not found. Is OMNeT++ running?")
    except ConnectionRefusedError:
        print(f"❌ Connection refused. OMNeT++ might be restarting.")
    except Exception as e:
        print(f"Error: {e}")
    
   
    finally:
        print("--- Client Finished ---")

if __name__ == "__main__":
    run_client()

