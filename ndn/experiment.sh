#!/bin/bash

# Ensure logs folder exists
mkdir -p ./logs

# Run all numerologies (0..3) and seeds (1..4)
nohup bash -c '
for num in {0..3}; do
  for seed in {1..4}; do
    echo "===== Starting numerology $num seed $seed ====="
    # Clean any leftover processes before starting
    pkill -f opp_run
    pkill -f ns3
    sleep 5

    # Run co-simulation and log output
    ./run-simple-cosim.sh --duration 200 --numerology $num --seed $seed >> ./logs/run_200s_num${num}_seed${seed}.log 2>&1

    # Short pause to avoid conflicts
    sleep 5
  done
done
' > ./logs/master_run.log 2>&1 &

echo "All 16 runs queued sequentially in the background with nohup. Check ./logs/master_run.log for overall progress."
