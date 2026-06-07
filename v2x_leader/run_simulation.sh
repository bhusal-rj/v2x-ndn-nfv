#!/bin/bash

# ============================================================================
# OMNeT++ V2X Simulation Build and Run Script
# ============================================================================
# Project structure:
#   headers/      — All .h files
#   src/          — All .cc files
#   ned/          — All .ned files
#   messages/     — All .msg files
#   simulations/  — SUMO traffic files
# ============================================================================

set -e

# ============================================================================
# Configuration
# ============================================================================

VEINS_PATH="$HOME/veins"
OMNET_PATH="$HOME/omnetpp"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAKEFILE="$SCRIPT_DIR/Makefile"
EXECUTABLE="$SCRIPT_DIR/v2x_leader"

# ============================================================================
# Helper Functions
# ============================================================================

log_info() {
    echo "ℹ️  $1"
}

log_success() {
    echo "✅ $1"
}

log_error() {
    echo "❌ $1"
}

log_section() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "$1"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
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

# Validate project structure
log_info "Validating project structure..."

REQUIRED_DIRS=("headers" "src" "ned" "simulations")
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

if [ -f "$MAKEFILE" ]; then
    log_info "Makefile already exists at: $MAKEFILE"
else
    log_info "Makefile not found. Generating using opp_makemake..."
    log_info "Project structure: headers/, src/, ned/, messages/, simulations/"
    
    cd "$SCRIPT_DIR"
    
    opp_makemake --deep -f \
        -I. \
        -I"$VEINS_PATH/src" \
        -L"$VEINS_PATH/src" \
        -lveins \
        -Iheaders \
        -Isrc \
        -Ined \
    
    log_success "Makefile generated successfully"
fi

log_info "Building project using make -j$(nproc)..."
cd "$SCRIPT_DIR"
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

export OMNETPP_IMAGE_PATH="$VEINS_PATH/images:$OMNET_PATH/images"
log_info "Image path set: $OMNETPP_IMAGE_PATH"

# Kill any existing processes on port 9998 and 9999
echo -e "🧹 Cleaning up any existing servers on port 9998 and 9999..."
lsof -ti:9998 | xargs kill -9 2>/dev/null || true
lsof -ti:9999 | xargs kill -9 2>/dev/null || true
sleep 1
# ============================================================================
# Simulation Execution
# ============================================================================


log_section "Launching Simulation"
log_info "Starting simulation in Qtenv (graphical mode)..."
log_info "Command: $EXECUTABLE -n .:$VEINS_PATH/src/veins -u Qtenv -c General"
log_info "SUMO simulations folder: $SCRIPT_DIR/simulations/"
echo ""

cd "$SCRIPT_DIR"
"$EXECUTABLE" -n .:$"$VEINS_PATH/src/veins" -u Qtenv -c General | tee omnet_simulation.log
EXIT_STATUS=${PIPESTATUS[0]}



log_success "Simulation finished."

exit $EXIT_STATUS
# Alternative: Uncomment below to run in command-line mode (Cmdenv)
# log_info "Starting simulation in Cmdenv (command-line mode)..."
# "$EXECUTABLE" -n .:$"$VEINS_PATH/src/veins" -u Cmdenv -c General

