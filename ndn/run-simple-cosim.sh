#!/bin/bash

# ============================================================================
# V2X-NDN Simple Co-Simulation Script
# ============================================================================
# This script runs the OMNeT++ ↔ NS-3 co-simulation in a simple, reliable way.
#
# Usage: ./run-simple-cosim.sh [--duration SECONDS] [--seed SEED] [--numerology N] [--quick] [--wall-timeout SECONDS]
# ============================================================================

set -e

# --- Configuration ---
PROJECT_ROOT="$HOME/v2x/ndn"
OMNET_DIR="$PROJECT_ROOT/v2x_leader"
NS3_PATH="$HOME/ndn2/ns-3"
RESULTS_DIR="$PROJECT_ROOT/results"

# Default simulation-time target (passed to OMNeT++ sim-time-limit)
DURATION=100
NUMEROLOGY=1  # Default 5G NR numerology (0-4), 1=30kHz SCS
SEED=1        # Default seed for deterministic simulation
# Optional wall-clock timeout safety guard (disabled by default)
WALL_TIMEOUT=""
QUICK_MODE=0
DURATION_EXPLICIT=0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration) DURATION="$2"; DURATION_EXPLICIT=1; shift 2 ;;
        --numerology) NUMEROLOGY="$2"; shift 2 ;;
        --seed) SEED="$2"; shift 2 ;;
        --wall-timeout) WALL_TIMEOUT="$2"; shift 2 ;;
        --quick) QUICK_MODE=1; shift ;;
        *) shift ;;
    esac
done

# Keep quick runs sensible while allowing explicit duration override.
if [ "$QUICK_MODE" -eq 1 ] && [ "$DURATION_EXPLICIT" -eq 0 ]; then
    DURATION=30
fi

RUN_TAG="${DURATION}s_num${NUMEROLOGY}_seed${SEED}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}  V2X-NDN Co-Simulation${NC}"
echo -e "${BLUE}  Simulation Duration Target: ${DURATION}s${NC}"
if [ -n "$WALL_TIMEOUT" ]; then
    echo -e "${BLUE}  Wall-clock Timeout: ${WALL_TIMEOUT}s${NC}"
else
    echo -e "${BLUE}  Wall-clock Timeout: disabled${NC}"
fi
echo -e "${BLUE}  5G NR Numerology: ${NUMEROLOGY} (SCS=$((15 * (1 << NUMEROLOGY))) kHz)${NC}"
echo -e "${BLUE}  Random Seed: ${SEED}${NC}"
echo -e "${BLUE}================================================================${NC}"

# --- Setup Environment ---
export PATH="$HOME/omnetpp/bin:$HOME/sumo/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/omnetpp/lib:$HOME/ndn2/ns-3/build/lib:$LD_LIBRARY_PATH"
export OMNETPP_ROOT="$HOME/omnetpp"
export SUMO_HOME="$HOME/sumo"
export OMNETPP_IMAGE_PATH="$HOME/veins/images:$HOME/omnetpp/images"

mkdir -p "$RESULTS_DIR"

# --- Cleanup Function ---
cleanup() {
    echo -e "${YELLOW}Cleaning up...${NC}"
    [ -n "$OMNET_PID" ] && kill $OMNET_PID 2>/dev/null || true
    [ -n "$NS3_PID" ] && kill $NS3_PID 2>/dev/null || true
    pkill -f "v2x_leader" 2>/dev/null || true
    pkill -f "ndn-v2x" 2>/dev/null || true
    pkill -f "sumo" 2>/dev/null || true
    lsof -ti:9998 | xargs kill -9 2>/dev/null || true
    lsof -ti:9999 | xargs kill -9 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- Step 1: Kill Old Processes ---
echo -e "${YELLOW}Step 1: Cleaning up old processes...${NC}"
lsof -ti:9998 | xargs kill -9 2>/dev/null || true
lsof -ti:9999 | xargs kill -9 2>/dev/null || true
pkill -f "v2x_leader" 2>/dev/null || true
pkill -f "ndn-v2x" 2>/dev/null || true
sleep 2
echo -e "${GREEN}✅ Ports cleared${NC}"

# --- Step 2: Build NS-3 (if needed) ---
echo -e "${YELLOW}Step 2: Preparing NS-3...${NC}"

# Copy source files
rm -rf "$NS3_PATH/scratch/ndn-v2x"
mkdir -p "$NS3_PATH/scratch/ndn-v2x"
cp "$PROJECT_ROOT/src/"*.cc "$NS3_PATH/scratch/ndn-v2x/" 2>/dev/null || true
cp "$PROJECT_ROOT/src/headers/"*.h "$NS3_PATH/scratch/ndn-v2x/" 2>/dev/null || true
cp "$PROJECT_ROOT/wscript" "$NS3_PATH/scratch/ndn-v2x/" 2>/dev/null || true

# Remove stale *.cc / *.h at scratch/ root. NS-3 builds each scratch/*.cc as its own
# program (expects main()); leftover copies from older workflows break the build.
find "$NS3_PATH/scratch" -maxdepth 1 -type f \( -name '*.cc' -o -name '*.h' \) -delete 2>/dev/null || true

# Build NS-3 (must succeed — do not hide failures)
cd "$NS3_PATH"
echo "Building NS-3 (this may take a minute on first run)..."
if ! ./waf build; then
    echo -e "${RED}❌ NS-3 build failed. Fix compile/link errors above before running co-simulation.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ NS-3 ready${NC}"

# --- Step 3: Start OMNeT++ ---
echo -e "${YELLOW}Step 3: Starting OMNeT++...${NC}"
cd "$OMNET_DIR"

# Check if executable exists
if [ ! -f "./v2x_leader" ]; then
    echo -e "${YELLOW}Building OMNeT++...${NC}"
    make -j$(nproc)
fi

# Start OMNeT++ in background with simulation-time limit override.
# Optional wall-clock timeout can be enabled via --wall-timeout.
OMNET_CMD=(./v2x_leader
    -n .:$HOME/veins/src/veins:
    -u Cmdenv
    -c General
    --sim-time-limit="${DURATION}s")

if [ -n "$WALL_TIMEOUT" ]; then
    timeout --signal=INT --kill-after=30s "${WALL_TIMEOUT}s" "${OMNET_CMD[@]}" \
        > "$RESULTS_DIR/omnet_${RUN_TAG}_live.log" 2>&1 &
else
    "${OMNET_CMD[@]}" > "$RESULTS_DIR/omnet_${RUN_TAG}_live.log" 2>&1 &
fi
OMNET_PID=$!
echo -e "${GREEN}✅ OMNeT++ started (PID: $OMNET_PID)${NC}"

# --- Step 4: Wait for OMNeT++ to be Ready ---
echo -e "${YELLOW}Step 4: Waiting for OMNeT++ to initialize...${NC}"
MAX_WAIT=30
for i in $(seq 1 $MAX_WAIT); do
    # Check if OMNeT++ is still running
    if ! ps -p $OMNET_PID > /dev/null 2>&1; then
        echo -e "${RED}❌ OMNeT++ died during initialization!${NC}"
        tail -30 "$RESULTS_DIR/omnet_${RUN_TAG}_live.log"
        exit 1
    fi
    
    # Check if port is listening
    if lsof -i:9998 > /dev/null 2>&1; then
        echo -e "${GREEN}✅ OMNeT++ listening on port 9998${NC}"
        break
    fi
    
    echo -n "."
    sleep 1
done
echo ""

# --- Step 5: Start NS-3 ---
echo -e "${YELLOW}Step 5: Starting NS-3 client...${NC}"
cd "$NS3_PATH"

# Set results directory for NS-3 to stream logs directly
export V2X_RESULTS_DIR="$RESULTS_DIR/"

# Run NS-3 executable directly (faster than waf --run)
# Pass numerology and seed as command-line arguments
./build/scratch/ndn-v2x/ndn-v2x --numerology=$NUMEROLOGY --seed=$SEED > "$RESULTS_DIR/ns3_${RUN_TAG}_live.log" 2> "$RESULTS_DIR/ns3_${RUN_TAG}_stderr.log" &
NS3_PID=$!
echo -e "${GREEN}✅ NS-3 started (PID: $NS3_PID)${NC}"

# Quick check
sleep 3
if ! ps -p $NS3_PID > /dev/null 2>&1; then
    echo -e "${RED}❌ NS-3 failed to start!${NC}"
    tail -20 "$RESULTS_DIR/ns3_${RUN_TAG}_live.log"
    exit 1
fi

# --- Step 6: Monitor Simulation ---
echo ""
echo -e "${BLUE}================================================================${NC}"
echo -e "${GREEN}🚀 Simulation Running!${NC}"
echo -e "${BLUE}================================================================${NC}"
echo "  OMNeT++ PID: $OMNET_PID"
echo "  NS-3 PID: $NS3_PID"
echo "  Simulation Duration Target: ${DURATION}s"
if [ -n "$WALL_TIMEOUT" ]; then
    echo "  Wall-clock Timeout: ${WALL_TIMEOUT}s"
fi
echo ""
echo "  Monitor logs:"
echo "    tail -f $RESULTS_DIR/omnet_${RUN_TAG}_live.log"
echo "    tail -f $RESULTS_DIR/ns3_${RUN_TAG}_live.log"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop early${NC}"
echo ""

# Monitor both processes; fail fast if NS-3 exits unexpectedly
OMNET_STATUS=0
while true; do
    if ! ps -p $OMNET_PID > /dev/null 2>&1; then
        set +e
        wait $OMNET_PID 2>/dev/null
        OMNET_STATUS=$?
        set -e
        break
    fi

    if ! ps -p $NS3_PID > /dev/null 2>&1; then
        set +e
        wait $NS3_PID 2>/dev/null
        NS3_STATUS=$?
        set -e
        echo -e "${RED}❌ NS-3 exited early (code: ${NS3_STATUS:-unknown})${NC}"
        echo "NS-3 log tail:"
        tail -40 "$RESULTS_DIR/ns3_${RUN_TAG}_live.log" 2>/dev/null || true
        echo -e "${YELLOW}Stopping OMNeT++ due to NS-3 failure...${NC}"
        kill $OMNET_PID 2>/dev/null || true
        set +e
        wait $OMNET_PID 2>/dev/null
        set -e
        OMNET_STATUS=1
        break
    fi

    sleep 1
done

# GNU timeout returns 124 when wall timeout is hit
if [ $OMNET_STATUS -eq 124 ] && [ -n "$WALL_TIMEOUT" ]; then
    echo -e "${YELLOW}⚠️ OMNeT++ reached wall-clock timeout (${WALL_TIMEOUT}s) before natural completion${NC}"
elif [ $OMNET_STATUS -eq 0 ]; then
    echo -e "${GREEN}✅ OMNeT++ completed normally${NC}"
else
    echo -e "${YELLOW}⚠️ OMNeT++ exited with code: $OMNET_STATUS${NC}"
    # 139 = SIGSEGV: often happens during teardown after Cmdenv prints "Simulation ended"
    if [ "$OMNET_STATUS" -eq 139 ] && grep -q "Simulation ended at time" "$RESULTS_DIR/omnet_${RUN_TAG}_live.log" 2>/dev/null; then
        echo -e "${YELLOW}   (Log shows simulation reached end time; crash may be Veins/SUMO shutdown — try: gdb --args ./v2x_leader ...)${NC}"
    fi
fi

# Wait for NS-3 to finish
sleep 2
kill $NS3_PID 2>/dev/null || true
set +e
wait $NS3_PID 2>/dev/null
set -e

# --- Step 7: Collect Results ---
echo ""
echo -e "${YELLOW}Collecting results...${NC}"

# Copy NDN trace files with duration suffix
[ -f "$NS3_PATH/app-delays-trace.txt" ] && cp "$NS3_PATH/app-delays-trace.txt" "$RESULTS_DIR/app-delays-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/rate-trace.txt" ] && cp "$NS3_PATH/rate-trace.txt" "$RESULTS_DIR/rate-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/cs-trace.txt" ] && cp "$NS3_PATH/cs-trace.txt" "$RESULTS_DIR/cs-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/ndn-cs-trace.txt" ] && cp "$NS3_PATH/ndn-cs-trace.txt" "$RESULTS_DIR/ndn-cs-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/pit-trace.txt" ] && cp "$NS3_PATH/pit-trace.txt" "$RESULTS_DIR/pit-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/fib-trace.txt" ] && cp "$NS3_PATH/fib-trace.txt" "$RESULTS_DIR/fib-trace_${RUN_TAG}.txt"
[ -f "$NS3_PATH/actions.txt" ] && cp "$NS3_PATH/actions.txt" "$RESULTS_DIR/actions_${RUN_TAG}.txt"
[ -f "$NS3_PATH/arch_a_metrics.json" ] && cp "$NS3_PATH/arch_a_metrics.json" "$RESULTS_DIR/arch_a_metrics_${RUN_TAG}.json"
[ -f "$NS3_PATH/simulation_metrics.json" ] && cp "$NS3_PATH/simulation_metrics.json" "$RESULTS_DIR/simulation_metrics_${RUN_TAG}.json"

# Copy other trace files
cp "$NS3_PATH"/*.json "$RESULTS_DIR/" 2>/dev/null || true

# OMNeT++ messages are streamed directly to results via V2X_RESULTS_DIR
# But copy from NS-3 dir as fallback if file exists there
if [ -f "$NS3_PATH/omnet_messages.jsonl" ]; then
    cp "$NS3_PATH/omnet_messages.jsonl" "$RESULTS_DIR/omnet_messages_${RUN_TAG}.jsonl" 2>/dev/null || true
fi

# Rename/copy other files with duration suffix
[ -f "$NS3_PATH/omnet-data-log.txt" ] && cp "$NS3_PATH/omnet-data-log.txt" "$RESULTS_DIR/omnet_data_${RUN_TAG}.log" 2>/dev/null
[ -f "$RESULTS_DIR/omnet-data-log.txt" ] && mv "$RESULTS_DIR/omnet-data-log.txt" "$RESULTS_DIR/omnet_data_${RUN_TAG}.log" 2>/dev/null
[ -f "$NS3_PATH/data.txt" ] && cp "$NS3_PATH/data.txt" "$RESULTS_DIR/raw_json_${RUN_TAG}.txt" 2>/dev/null
[ -f "$RESULTS_DIR/data.txt" ] && mv "$RESULTS_DIR/data.txt" "$RESULTS_DIR/raw_json_${RUN_TAG}.txt" 2>/dev/null
[ -f "$RESULTS_DIR/omnet_messages.jsonl" ] && mv "$RESULTS_DIR/omnet_messages.jsonl" "$RESULTS_DIR/omnet_messages_${RUN_TAG}.jsonl" 2>/dev/null
[ -f "$NS3_PATH/ns3-to-omnet-log.txt" ] && cp "$NS3_PATH/ns3-to-omnet-log.txt" "$RESULTS_DIR/ns3_to_omnet_${RUN_TAG}.log" 2>/dev/null
[ -f "$RESULTS_DIR/ns3_to_omnet_messages.jsonl" ] && mv "$RESULTS_DIR/ns3_to_omnet_messages.jsonl" "$RESULTS_DIR/ns3_to_omnet_messages_${RUN_TAG}.jsonl" 2>/dev/null

echo -e "${GREEN}✅ Trace files collected${NC}"

# --- Step 8: Extract Metrics to JSON ---
echo ""
echo -e "${YELLOW}Extracting metrics to JSON...${NC}"
if [ -f "$PROJECT_ROOT/extract_metrics.py" ]; then
    python3 "$PROJECT_ROOT/extract_metrics.py" \
        --results-dir "$RESULTS_DIR" \
        --ns3-dir "$NS3_PATH" \
        --output "$RESULTS_DIR/metrics_${RUN_TAG}.json" \
        --ns3-log "$RESULTS_DIR/ns3_${RUN_TAG}_live.log" \
        --omnet-log "$RESULTS_DIR/omnet_${RUN_TAG}_live.log" 2>/dev/null
    echo -e "${GREEN}✅ Metrics JSON: $RESULTS_DIR/metrics_${RUN_TAG}.json${NC}"
else
    echo -e "${YELLOW}⚠️ extract_metrics.py not found${NC}"
fi

# --- Step 9: Parse Accidents and Safety Events ---
echo ""
echo -e "${YELLOW}Parsing accident and safety data...${NC}"
if [ -f "$PROJECT_ROOT/scripts/parse_accidents.py" ]; then
    python3 "$PROJECT_ROOT/scripts/parse_accidents.py" \
        --duration "${RUN_TAG}" \
        --results-dir "$RESULTS_DIR" 2>/dev/null
    echo -e "${GREEN}✅ Accidents JSON: $RESULTS_DIR/accidents_${RUN_TAG}.json${NC}"
else
    echo -e "${YELLOW}⚠️ parse_accidents.py not found${NC}"
fi

# --- Step 10: Extract NDN Packet Traces ---
echo ""
echo -e "${YELLOW}Extracting NDN packet traces...${NC}"
if [ -f "$PROJECT_ROOT/scripts/extract_ndn_packets.py" ]; then
    python3 "$PROJECT_ROOT/scripts/extract_ndn_packets.py" \
        --duration "${RUN_TAG}" \
        --results-dir "$RESULTS_DIR" 2>/dev/null
    echo -e "${GREEN}✅ NDN packets: $RESULTS_DIR/ndn_packets_${RUN_TAG}.json${NC}"
    echo -e "${GREEN}✅ Interests log: $RESULTS_DIR/interests_${RUN_TAG}.log${NC}"
    echo -e "${GREEN}✅ Data packets log: $RESULTS_DIR/data_packets_${RUN_TAG}.log${NC}"
else
    echo -e "${YELLOW}⚠️ extract_ndn_packets.py not found${NC}"
fi

# --- Step 11: Generate Comprehensive Report ---
echo ""
echo -e "${YELLOW}Generating comprehensive report...${NC}"
if [ -f "$PROJECT_ROOT/scripts/generate_report.py" ]; then
    python3 "$PROJECT_ROOT/scripts/generate_report.py" \
        --duration "${RUN_TAG}" \
        --results-dir "$RESULTS_DIR" 2>/dev/null
    echo -e "${GREEN}✅ Report: $RESULTS_DIR/REPORT_${RUN_TAG}.md${NC}"
else
    echo -e "${YELLOW}⚠️ generate_report.py not found${NC}"
fi

# --- Summary ---
echo ""
echo -e "${BLUE}================================================================${NC}"
echo -e "${GREEN}🎉 Simulation Finished!${NC}"
echo -e "${BLUE}================================================================${NC}"
echo ""
echo "📁 Results Directory: $RESULTS_DIR"
echo ""
echo "📊 Generated Files:"
echo "  • metrics_${RUN_TAG}.json          - Core simulation metrics"
echo "  • accidents_${RUN_TAG}.json        - Collision & safety events"
echo "  • ndn_packets_${RUN_TAG}.json      - NDN packet statistics"
echo "  • interests_${RUN_TAG}.log         - Interest packet traces"
echo "  • data_packets_${RUN_TAG}.log      - Data packet traces"
echo "  • REPORT_${RUN_TAG}.md             - Comprehensive report"
echo ""
echo "📈 Quick View Commands:"
echo "  cat $RESULTS_DIR/REPORT_${RUN_TAG}.md"
echo "  python3 visualize_metrics.py $RESULTS_DIR/metrics_${RUN_TAG}.json"
echo ""

# --- Record Artifact Metadata ---
echo -e "${YELLOW}Recording artifact metadata...${NC}"
{
    echo "=== V2X-NDN Simulation Run Metadata ==="
    echo "Timestamp: $(date -Iseconds)"
    echo "Duration: ${DURATION}s"
    if [ -n "$WALL_TIMEOUT" ]; then
        echo "Wall Timeout: ${WALL_TIMEOUT}s"
    else
        echo "Wall Timeout: disabled"
    fi
    echo "Seed: ${SEED}"
    echo "Numerology: ${NUMEROLOGY} (SCS=$((15 * (1 << NUMEROLOGY))) kHz)"
    echo "Commit: $(git rev-parse HEAD 2>/dev/null || echo 'unknown')"
    echo "Branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'unknown')"
    echo "Uncommitted changes: $(git status --porcelain 2>/dev/null | wc -l) files"
    echo "User: $USER"
    echo "Hostname: $(hostname)"
    echo "OMNET_DIR: $OMNET_DIR"
    echo "NS3_PATH: $NS3_PATH"
    echo "Results_Dir: $RESULTS_DIR"
} >> "$RESULTS_DIR/run_metadata.txt"
echo -e "${GREEN}✅ Metadata saved: $RESULTS_DIR/run_metadata.txt${NC}"

exit 0
