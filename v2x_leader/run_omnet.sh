#!/bin/bash

# ============================================================================
# OMNeT++ V2X Simulation Build and Run Script (Combined)
# ============================================================================

set -e

# ============================================================================
# Configuration
# ============================================================================

# Paths should be set as environment variables or adjusted below
VEINS_PATH="${VEINS_PATH:-$HOME/veins}"
OMNET_PATH="${OMNET_PATH:-$HOME/omnet}"
SUMO_HOME="${SUMO_HOME:-$HOME/sumo}" # Default SUMO path

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAKEFILE="$SCRIPT_DIR/Makefile"
EXECUTABLE="$SCRIPT_DIR/v2x_leader"

# Colors for output (from run-omnet)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# Helper Functions
# ============================================================================

log_info() {
    echo -e "${YELLOW}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

log_section() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

# ============================================================================
# Validation
# ============================================================================

log_section "Environment Validation"

if [ ! -d "$VEINS_PATH" ]; then
    log_error "Veins installation not found at: $VEINS_PATH"
    exit 1
fi
log_success "Veins found at: $VEINS_PATH"

if [ ! -d "$OMNET_PATH" ]; then
    log_error "OMNeT++ installation not found at: $OMNET_PATH"
    exit 1
fi
log_success "OMNeT++ found at: $OMNET_PATH"

if [ ! -d "$SUMO_HOME" ]; then
    log_error "SUMO installation not found at: $SUMO_HOME. Please set SUMO_HOME."
    exit 1
fi
log_success "SUMO found at: $SUMO_HOME"

# Validate project structure (from run_simulation)
log_info "Validating project structure..."

REQUIRED_DIRS=("headers" "src" "ned" "messages" "simulations")
for dir in "${REQUIRED_DIRS[@]}"; do
    if [ ! -d "$SCRIPT_DIR/$dir" ]; then
        log_error "Required directory not found: $SCRIPT_DIR/$dir"
        exit 1
    fi
    log_success "$dir/ directory found"
done

# ============================================================================
# Build Phase
# ============================================================================

log_section "Build Phase"
cd "$SCRIPT_DIR"

if [ -f "$MAKEFILE" ]; then
    log_info "Makefile already exists at: $MAKEFILE. Skipping opp_makemake."
else
    log_info "Makefile not found. Generating using opp_makemake..."
    
    # opp_makemake command from run_simulation
    opp_makemake --deep -f \
        -I. \
        -I"$VEINS_PATH/src" \
        -L"$VEINS_PATH/src" \
        -lveins \
        -Iheaders \
        -Isrc \
        -Ined \
        -Imessages \
        -Mmessages
    
    # Check if Makefile was created
    if [ ! -f "$MAKEFILE" ]; then
        log_error "opp_makemake failed to generate Makefile."
        exit 1
    fi
    log_success "Makefile generated successfully"
fi

log_info "Building project using make -j$(nproc)..."
make clean # Added clean build for robustness
make -j$(nproc)

if [ ! -f "$EXECUTABLE" ]; then
    log_error "Build failed: Executable not found at $EXECUTABLE"
    exit 1
fi
log_success "Build completed successfully"

# ============================================================================
# Runtime Setup
# ============================================================================

log_section "Runtime Setup"

# Set image path (from run_simulation)
export OMNETPP_IMAGE_PATH="$VEINS_PATH/images:$OMNET_PATH/images"
log_info "Image path set: $OMNETPP_IMAGE_PATH"

# Kill any existing processes on port 9998 and 9999 (from run-omnet)
echo -e "${YELLOW}🧹 Cleaning up any existing servers on port 9998 and 9999...${NC}"
lsof -ti:9998 | xargs kill -9 2>/dev/null || true
lsof -ti:9999 | xargs kill -9 2>/dev/null || true
sleep 1

# ============================================================================
# Simulation Execution
# ============================================================================

log_section "Launching Simulation"

cd "$SCRIPT_DIR"

# Simulation command using Cmdenv and INET path (Cmdenv from run-omnet)
# The '-n' path is a combination: current dir (.), Veins

log_info "Starting simulation in Cmdenv (command-line mode)..."
log_info "Command: $EXECUTABLE -n .:\"$VEINS_PATH/src/veins\" -u Qtenv -c General"
echo ""

# Save start time (from run-omnet)
START_TIME=$(date +%s)

# Run the simulation and capture output (from run-omnet)

"$EXECUTABLE" -n .:"$VEINS_PATH/src/veins": -u Qtenv -c General 2>&1 | tee omnet_simulation.log



# Capture exit status
EXIT_STATUS=${PIPESTATUS[0]}
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

# ============================================================================
# Clean Exit and Results (Improved structure from run-omnet)
# ============================================================================

echo ""
log_section "Simulation Results"

if [ $EXIT_STATUS -eq 0 ]; then
    log_success "OMNeT++ simulation completed successfully!"
else
    log_error "OMNeT++ simulation failed with exit code: $EXIT_STATUS"
fi

echo -e "${BLUE}Duration: ${YELLOW}${DURATION}s${NC}"
echo ""

# Check for metrics files (from run-omnet)
echo -e "${BLUE}📊 Checking for metrics files...${NC}"

RESULTS_DIR="$SCRIPT_DIR/results"
if [ -d "$RESULTS_DIR" ]; then
    echo -e "${GREEN}Results directory found:${NC}"
    ls -lh "$RESULTS_DIR" | grep -E '\.(csv|json|vec|sca)$' || echo -e "${YELLOW}  No metrics files found${NC}"
fi

# Check simulation log (from run-omnet)
if [ -f "omnet_simulation.log" ]; then
    echo ""
    echo -e "${BLUE}📄 Log file: ${YELLOW}omnet_simulation.log${NC}"
fi

echo ""
echo -e "${BLUE}================================================================${NC}"
echo -e "${GREEN}🎉 OMNeT++ simulation finished!${NC}"
echo -e "${BLUE}================================================================${NC}"
# Cleanup SUMO files
exit $EXIT_STATUS