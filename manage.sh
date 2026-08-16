#!/usr/bin/env bash
# ==============================================================================
#  Quadplane Condor — Master Control Script (manage.sh)
# ==============================================================================
#  Unified command-line management tool for simulation, testing, model setup,
#  mission execution, telemetry dashboard, and packaging.
# ==============================================================================

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"

# Color formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------
print_banner() {
    echo -e "${CYAN}=================================================================${NC}"
    echo -e "${BOLD}${CYAN}  ✈️  QUADPLANE CONDOR — HYBRID VTOL FLIGHT CONTROL PLATFORM  ${NC}"
    echo -e "${CYAN}=================================================================${NC}"
}

print_status() { echo -e "  ${GREEN}✅${NC} $1"; }
print_warn()   { echo -e "  ${YELLOW}⚠️${NC}  $1"; }
print_error()  { echo -e "  ${RED}❌${NC} $1"; }
print_info()   { echo -e "  ${BLUE}ℹ️${NC}  $1"; }

activate_venv() {
    if [ -f "$PROJECT_DIR/venv_linux/bin/activate" ]; then
        source "$PROJECT_DIR/venv_linux/bin/activate"
    elif [ -f "$PROJECT_DIR/venv/bin/activate" ]; then
        source "$PROJECT_DIR/venv/bin/activate"
    fi
}

set_simulation_env() {
    local world="${1:-default}"
    export DISPLAY="${DISPLAY:-:0}"
    export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/models:$PROJECT_DIR/models/worlds:${GZ_SIM_RESOURCE_PATH:-}"
    export PX4_GZ_MODELS="$PROJECT_DIR/models"
    export PX4_GZ_WORLDS="$PROJECT_DIR/models/worlds"
    export PX4_SIM_MODEL=quadplane_condor
    export PX4_GZ_MODEL_NAME=quadplane_condor
    export PX4_HOME_LAT=21.028511
    export PX4_HOME_LON=105.804817
    export PX4_HOME_ALT=0.0
    export PX4_GZ_WORLD="$world"
    export PX4_GZ_FOLLOW_OFFSET_X=-8.0
    export PX4_GZ_FOLLOW_OFFSET_Y=-8.0
    export PX4_GZ_FOLLOW_OFFSET_Z=6.0
}

# ------------------------------------------------------------------------------
# Subcommands
# ------------------------------------------------------------------------------

cmd_help() {
    print_banner
    echo -e "Usage: ${BOLD}./manage.sh [command] [options]${NC}\n"
    echo -e "${BOLD}Available Commands:${NC}"
    echo -e "  ${GREEN}all${NC} [world]         Launch full stack (PX4 SITL + Gazebo + QGC + Dashboard)"
    echo -e "  ${GREEN}sim${NC} [world]         Start standalone PX4 SITL + Gazebo with Condor model"
    echo -e "  ${GREEN}mission${NC}             Run automated 5-stage Hybrid VTOL Mission"
    echo -e "  ${GREEN}geometric${NC}           Run Geometric SE(3) Figure-8 Trajectory Tracker"
    echo -e "  ${GREEN}dashboard${NC}           Start Web Control Center & Telemetry WebSocket server"
    echo -e "  ${GREEN}setup${NC}               Sync/install Quadplane Condor model and airframe to PX4"
    echo -e "  ${GREEN}test${NC}                Run unit test suite (pytest) and code audit (flake8)"
    echo -e "  ${GREEN}clean${NC}               Clean runtime logs, __pycache__, and test caches"
    echo -e "  ${GREEN}stop${NC}                Stop all background simulation and server processes"
    echo -e "  ${GREEN}package${NC}             Bundle project into redistributable .tar.gz archive"
    echo -e "  ${GREEN}help${NC}                Display this help message\n"
    echo -e "${BOLD}Simulation Worlds:${NC} default (standard airfield), figure8 (aerial gates & helipad)\n"
    echo -e "${BOLD}Examples:${NC}"
    echo -e "  ./manage.sh                     # Launch entire simulation system"
    echo -e "  ./manage.sh sim figure8         # Launch SITL with 3D Figure-8 world"
    echo -e "  ./manage.sh mission             # Execute automated VTOL mission"
    echo -e "  ./manage.sh stop                # Stop all background processes"
    echo ""
}

cmd_setup() {
    print_banner
    echo -e "${BOLD}Installing Quadplane Condor Model Assets into PX4-Autopilot...${NC}"
    if [ ! -d "$PX4_DIR" ]; then
        print_error "PX4-Autopilot not found at $PX4_DIR"
        exit 1
    fi

    # 1. Gazebo SDF Model
    local gz_models="$PX4_DIR/Tools/simulation/gz/models"
    mkdir -p "$gz_models"
    rm -rf "$gz_models/quadplane_condor"
    cp -r "$PROJECT_DIR/models/quadplane_condor" "$gz_models/quadplane_condor"
    print_status "Installed Gazebo Model -> $gz_models/quadplane_condor"

    # 2. Airframe Script
    local airframe_src="$PROJECT_DIR/models/airframes/4030_gz_quadplane_condor"
    local airframe_romfs="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
    mkdir -p "$airframe_romfs"
    cp "$airframe_src" "$airframe_romfs/4030_gz_quadplane_condor"
    chmod +x "$airframe_romfs/4030_gz_quadplane_condor"
    print_status "Installed Airframe Config -> $airframe_romfs/4030_gz_quadplane_condor"

    # Live build airframe directory
    local airframe_build="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
    if [ -d "$airframe_build" ]; then
        cp "$airframe_src" "$airframe_build/4030_gz_quadplane_condor"
        chmod +x "$airframe_build/4030_gz_quadplane_condor"
        print_status "Updated Live Build Airframe -> $airframe_build/4030_gz_quadplane_condor"
    fi

    # 3. Worlds
    local gz_worlds="$PX4_DIR/Tools/simulation/gz/worlds"
    mkdir -p "$gz_worlds"
    if [ -f "$PROJECT_DIR/models/worlds/condor_figure8.sdf" ]; then
        cp "$PROJECT_DIR/models/worlds/condor_figure8.sdf" "$gz_worlds/condor_figure8.sdf"
        print_status "Installed Simulation World -> $gz_worlds/condor_figure8.sdf"
    fi

    print_status "Quadplane Condor setup complete!"
}

cmd_stop() {
    echo -e "${YELLOW}Stopping all simulation and background services...${NC}"
    pkill -f "px4" 2>/dev/null && echo "  - Stopped PX4 Autopilot" || true
    pkill -f "gz sim" 2>/dev/null && echo "  - Stopped Gazebo Simulator" || true
    pkill -f "QGroundControl" 2>/dev/null && echo "  - Stopped QGroundControl" || true
    pkill -f "server.py" 2>/dev/null && echo "  - Stopped Dashboard Telemetry Server" || true
    pkill -f "http.server 8080" 2>/dev/null && echo "  - Stopped Web App HTTP Server" || true
    pkill -f "mavsdk_server" 2>/dev/null && echo "  - Stopped MAVSDK Bridge Server" || true
    print_status "All services stopped."
}

cmd_clean() {
    echo -e "${YELLOW}Cleaning runtime logs, compilation cache, and temporary data...${NC}"
    rm -rf "$LOG_DIR"/*.log "$LOG_DIR"/*.csv "$PROJECT_DIR/.pytest_cache"
    find "$PROJECT_DIR" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find "$PROJECT_DIR" -type f -name "*.pyc" -delete 2>/dev/null || true
    touch "$LOG_DIR/.gitkeep" "$PROJECT_DIR/output/.gitkeep" "$PROJECT_DIR/plots/.gitkeep"
    print_status "Cleaned project directory."
}

cmd_sim() {
    local world="${1:-default}"
    if [ "$world" = "figure8" ]; then
        world="condor_figure8"
        activate_venv
        python3 "$PROJECT_DIR/scripts/generate_figure8_world.py" >/dev/null 2>&1 || true
    fi

    print_banner
    echo -e "${BOLD}Starting Standalone PX4 SITL + Gazebo Sim [World: $world]${NC}"
    cmd_setup >/dev/null 2>&1

    pkill -9 -f "px4" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    sleep 1

    set_simulation_env "$world"
    cd "$PX4_DIR"
    make px4_sitl gz_quadplane_condor
}

cmd_all() {
    local world="${1:-default}"
    if [ "$world" = "figure8" ]; then
        world="condor_figure8"
        activate_venv
        python3 "$PROJECT_DIR/scripts/generate_figure8_world.py" >/dev/null 2>&1 || true
    fi

    print_banner
    echo -e "${CYAN}[1/4] Starting PX4 SITL + Gazebo Simulation...${NC}"

    cmd_setup >/dev/null 2>&1
    cmd_stop >/dev/null 2>&1
    sleep 1

    set_simulation_env "$world"

    cd "$PX4_DIR"
    make px4_sitl gz_quadplane_condor > "$LOG_DIR/px4_sitl.log" 2>&1 &
    local px4_pid=$!

    echo "  Waiting for PX4 SITL to initialize..."
    local timeout=120
    local elapsed=0
    while ! grep -q "Ready for takeoff" "$LOG_DIR/px4_sitl.log" 2>/dev/null; do
        sleep 2
        elapsed=$((elapsed + 2))
        if [ $elapsed -ge $timeout ]; then
            print_error "PX4 failed to start within ${timeout}s. Check $LOG_DIR/px4_sitl.log"
            kill $px4_pid 2>/dev/null || true
            exit 1
        fi
        printf "\r  Waiting... (%ds / %ds)" "$elapsed" "$timeout"
    done
    echo ""
    print_status "PX4 SITL + Gazebo running (PID: $px4_pid)"

    # 2. QGroundControl
    echo -e "${CYAN}[2/4] Starting QGroundControl GCS...${NC}"
    local qgc_pid=""
    if [ -x "/home/tu/QGroundControl/extracted/AppRun" ]; then
        /home/tu/QGroundControl/extracted/AppRun > "$LOG_DIR/qgc.log" 2>&1 &
        qgc_pid=$!
        print_status "QGroundControl started (PID: $qgc_pid)"
    elif [ -x "$HOME/QGC/squashfs-root/AppRun" ]; then
        "$HOME/QGC/squashfs-root/AppRun" > "$LOG_DIR/qgc.log" 2>&1 &
        qgc_pid=$!
        print_status "QGroundControl started (PID: $qgc_pid)"
    else
        print_warn "QGroundControl binary not found. Skipping."
    fi

    # 3. Web Dashboard
    echo -e "${CYAN}[3/4] Starting Web Dashboard Server...${NC}"
    activate_venv
    cd "$PROJECT_DIR"

    pkill -f "http.server 8080" 2>/dev/null || true
    python3 -m http.server 8080 --directory "$PROJECT_DIR/src/dashboard/web_dashboard" > "$LOG_DIR/http_server.log" 2>&1 &
    local http_pid=$!
    print_status "Web App HTTP Server on http://127.0.0.1:8080 (PID: $http_pid)"

    python3 src/dashboard/web_dashboard/server.py > "$LOG_DIR/dashboard.log" 2>&1 &
    local dash_pid=$!
    print_status "Telemetry WebSocket Server on ws://localhost:8765 (PID: $dash_pid)"

    # 4. Summary
    sleep 2
    echo ""
    echo -e "${CYAN}=================================================================${NC}"
    echo -e "${GREEN}  All simulation services started successfully!${NC}"
    echo -e "${CYAN}=================================================================${NC}"
    echo "  Services:"
    printf "  %-35s PID: %s\n" "PX4 SITL + Gazebo ($world)" "$px4_pid"
    [ -n "$qgc_pid" ] && printf "  %-35s PID: %s\n" "QGroundControl (GUI)" "$qgc_pid"
    printf "  %-35s PID: %s\n" "Web App (HTTP 8080)" "$http_pid"
    printf "  %-35s PID: %s\n" "Telemetry Stream (WS 8765)" "$dash_pid"
    echo ""
    echo -e "  🚀 Web Control Center: ${BOLD}${GREEN}http://127.0.0.1:8080${NC}"
    echo -e "  To stop all services:  ${YELLOW}./manage.sh stop${NC}"
    echo ""

    wait
}

cmd_mission() {
    print_banner
    echo -e "${BOLD}Executing Automated Quadplane Condor Hybrid VTOL Mission...${NC}"
    activate_venv
    cd "$PROJECT_DIR"
    python3 src/px4_integration/sitl_condor_mission.py
}

cmd_geometric() {
    print_banner
    echo -e "${BOLD}Executing Geometric SE(3) Figure-8 Flight Tracker...${NC}"
    activate_venv
    cd "$PROJECT_DIR"
    python3 src/px4_integration/sitl_condor_geometric.py
}

cmd_dashboard() {
    print_banner
    echo -e "${BOLD}Starting Web Control Center & Telemetry Server...${NC}"
    activate_venv
    cd "$PROJECT_DIR"
    pkill -f "http.server 8080" 2>/dev/null || true
    python3 -m http.server 8080 --directory "$PROJECT_DIR/src/dashboard/web_dashboard" > "$LOG_DIR/http_server.log" 2>&1 &
    local http_pid=$!
    print_status "Web App HTTP Server running on http://127.0.0.1:8080 (PID: $http_pid)"

    python3 src/dashboard/web_dashboard/server.py
}

cmd_test() {
    print_banner
    echo -e "${BOLD}Running Codebase Quality Audit & Unit Test Suite...${NC}"
    activate_venv
    cd "$PROJECT_DIR"

    echo -e "\n${CYAN}1. Flake8 Linting Check:${NC}"
    if ./venv_linux/bin/flake8 src tests scripts; then
        print_status "Flake8: 0 errors (100% PEP 8 compliant)"
    else
        print_error "Flake8 issues detected."
        exit 1
    fi

    echo -e "\n${CYAN}2. PyTest Test Suite:${NC}"
    ./venv_linux/bin/pytest -v
}

cmd_package() {
    print_banner
    local archive_name="px4_condor_project_$(date +%Y%m%d_%H%M%S).tar.gz"
    echo -e "${BOLD}Creating clean distribution package: $archive_name...${NC}"
    cd "$PROJECT_DIR/.."
    tar --exclude='.git' \
        --exclude='__pycache__' \
        --exclude='*.pyc' \
        --exclude='.pytest_cache' \
        --exclude='logs/*.log' \
        --exclude='logs/*.csv' \
        -czvf "$archive_name" px4_project/
    echo ""
    print_status "Package created successfully: $(pwd)/$archive_name"
}

# ------------------------------------------------------------------------------
# Entrypoint Dispatcher
# ------------------------------------------------------------------------------
case "${1:-all}" in
    all|start)
        shift 2>/dev/null || true
        cmd_all "$@"
        ;;
    sim|sitl)
        shift 2>/dev/null || true
        cmd_sim "$@"
        ;;
    mission)
        cmd_mission
        ;;
    geometric|track)
        cmd_geometric
        ;;
    dashboard|web)
        cmd_dashboard
        ;;
    setup|install)
        cmd_setup
        ;;
    test)
        cmd_test
        ;;
    clean)
        cmd_clean
        ;;
    stop|--stop)
        cmd_stop
        ;;
    package|dist)
        cmd_package
        ;;
    help|--help|-h)
        cmd_help
        ;;
    *)
        print_error "Unknown command: $1"
        echo "Run './manage.sh help' for usage instructions."
        exit 1
        ;;
esac
