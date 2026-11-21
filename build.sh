#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# ========================
# Configuration
# ========================

# ========================
# Build Functions
# ========================

setup_inference_runtime() {
    print_header "[Setting up Inference Runtime]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_SCRIPT="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"

    if [ -f "$DOWNLOAD_SCRIPT" ]; then
        print_info "Checking inference libraries..."
        bash "$DOWNLOAD_SCRIPT" || {
            print_error "Failed to setup inference libraries"
            exit 1
        }
        print_success "Inference runtime setup completed!"
    else
        print_warning "Download script not found: $DOWNLOAD_SCRIPT"
    fi
}

setup_mujoco() {
    print_header "[Setting up MuJoCo]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_MUJOCO_SCRIPT="${SCRIPT_DIR}/scripts/download_mujoco.sh"

    if [ -f "$DOWNLOAD_MUJOCO_SCRIPT" ]; then
        print_info "Checking MuJoCo library..."
        bash "$DOWNLOAD_MUJOCO_SCRIPT" || {
            print_error "Failed to setup MuJoCo"
            exit 1
        }
        print_success "MuJoCo setup completed!"
    else
        print_warning "MuJoCo download script not found: $DOWNLOAD_MUJOCO_SCRIPT"
    fi
}

setup_robot_descriptions() {
    print_header "[Setting up Robot Descriptions]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_ROBOT_DESC_SCRIPT="${SCRIPT_DIR}/scripts/download_robot_descriptions.sh"

    if [ -f "$DOWNLOAD_ROBOT_DESC_SCRIPT" ]; then
        print_info "Checking robot description files..."
        bash "$DOWNLOAD_ROBOT_DESC_SCRIPT" || {
            print_error "Failed to setup robot descriptions"
            exit 1
        }
        print_success "Robot descriptions setup completed!"
    else
        print_warning "Robot descriptions download script not found: $DOWNLOAD_ROBOT_DESC_SCRIPT"
    fi
}

run_cmake_build() {
    print_header "[Running CMake Build]"
    print_warning "NOTE: CMake builds common libraries; ATDog hardware deployment requires ROS2."
    print_separator

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "CMake build completed!"
}

run_mujoco_build() {
    print_header "[Running MuJoCo Build]"
    print_info "Building with MuJoCo simulator support..."
    print_separator

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "MuJoCo build completed!"
}

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    print_header "[Using colcon build]"
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Building all packages..."
        colcon build --merge-install --symlink-install
    else
        print_info "Building specific packages: $package_list"
        colcon build --merge-install --symlink-install --packages-select "${packages[@]}"
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    local packages=("$@")

    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    if [ ${#packages[@]} -eq 0 ]; then
        echo "  - All package.xml symlinks in directory src/"
    else
        echo "  - Package.xml symlinks for: ${packages[*]}"
    fi
    echo "  - directory build/"
    echo "  - directory cmake_build/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"

    # Ask for confirmation
    if [ ${#packages[@]} -eq 0 ]; then
        if ! ask_confirmation "Are you sure you want to clean ALL symlinks and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    else
        if ! ask_confirmation "Are you sure you want to clean symlinks for specified packages and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    fi

    # Remove package.xml symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all symlinks"
    else
        print_info "Removing symlinks for specific packages..."
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ]; then
                if [ -L "$package_dir/package.xml" ]; then
                    rm -f "$package_dir/package.xml"
                    print_success "Removed symlink from $package_name"
                else
                    print_warning "No symlink found for $package_name"
                fi
            else
                print_error "Package '$package_name' not found in src directory"
            fi
        done
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ cmake_build/ install/ log/ logs/

    print_success "Clean completed!"
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove symlinks and build artifacts)"
    echo -e "  -m, --cmake      Build common libraries using standalone CMake"
    echo -e "  -mj,--mujoco     Build with MuJoCo simulator support (CMake only)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all symlinks and build artifacts"
    echo -e "  $0 --clean package1   # Clean specific package and build artifacts"
    echo -e "  $0 -m                 # Build common libraries with CMake"
    echo -e "  $0 -mj                # Build with CMake and MuJoCo simulator support"
}

main() {
    local packages=()
    local clean_mode=false
    local cmake_mode=false
    local mujoco_mode=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -m|--cmake) cmake_mode=true; shift ;;
            -mj|--mujoco) cmake_mode=true; mujoco_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle MuJoCo build mode
    if [ "$mujoco_mode" = true ]; then
        setup_inference_runtime
        setup_robot_descriptions
        setup_mujoco
        run_mujoco_build
        exit 0
    fi

    # Handle CMake build mode
    if [ "$cmake_mode" = true ]; then
        setup_inference_runtime
        run_cmake_build
        exit 0
    fi

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    # Handle ROS build
    if [[ "$ROS_DISTRO" != "foxy" && "$ROS_DISTRO" != "humble" ]]; then
        print_error "Source a ROS2 Foxy/Humble setup.bash first."
        print_info "For standalone MuJoCo simulation, use --mujoco."
        exit 1
    fi

    setup_inference_runtime
    setup_robot_descriptions
    run_ros_build "${packages[@]}"
}

main "$@"
