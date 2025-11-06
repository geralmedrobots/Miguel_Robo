#!/bin/bash
# ============================================================================
# INTEGRATION VALIDATION SCRIPT
# Verifica dependências, sintaxe e configuração antes do build
# ============================================================================

set -e  # Exit on any error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "ULTRABOT AGV - Integration Validation"
echo "=========================================="
echo ""

# ============================================================================
# 1. CHECK REQUIRED ROS2 PACKAGES
# ============================================================================
echo "[1/6] Checking ROS2 package dependencies..."

REQUIRED_PACKAGES=(
    "rclcpp"
    "rclcpp_lifecycle"
    "lifecycle_msgs"
    "robot_state_publisher"
    "xacro"
    "robot_localization"
    "nav2_bringup"
    "slam_toolbox"
)

MISSING_PACKAGES=()

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ! ros2 pkg list | grep -q "^${pkg}$"; then
        MISSING_PACKAGES+=("$pkg")
        echo "  ❌ MISSING: $pkg"
    else
        echo "  ✓ Found: $pkg"
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo ""
    echo "❌ ERROR: Missing ${#MISSING_PACKAGES[@]} required package(s)"
    echo "Install with: sudo apt install $(printf 'ros-${ROS_DISTRO}-%s ' "${MISSING_PACKAGES[@]}" | sed 's/_/-/g')"
    exit 1
fi

echo "  ✓ All ROS2 packages found"
echo ""

# ============================================================================
# 2. VALIDATE URDF/XACRO SYNTAX
# ============================================================================
echo "[2/6] Validating URDF/xacro syntax..."

URDF_FILE="$PKG_DIR/urdf/ultrabot.urdf.xacro"

if [ ! -f "$URDF_FILE" ]; then
    echo "  ❌ ERROR: URDF file not found: $URDF_FILE"
    exit 1
fi

# Check xacro can process the file
if ! xacro "$URDF_FILE" > /dev/null 2>&1; then
    echo "  ❌ ERROR: xacro failed to process URDF"
    xacro "$URDF_FILE"  # Show error
    exit 1
fi

# Validate URDF structure with check_urdf (if available)
if command -v check_urdf &> /dev/null; then
    TEMP_URDF=$(mktemp)
    xacro "$URDF_FILE" > "$TEMP_URDF"
    
    if ! check_urdf "$TEMP_URDF" &> /dev/null; then
        echo "  ❌ ERROR: URDF validation failed"
        check_urdf "$TEMP_URDF"
        rm "$TEMP_URDF"
        exit 1
    fi
    rm "$TEMP_URDF"
    echo "  ✓ URDF structure valid (check_urdf passed)"
else
    echo "  ⚠ check_urdf not available (install with: sudo apt install liburdfdom-tools)"
fi

echo "  ✓ URDF/xacro syntax valid"
echo ""

# ============================================================================
# 3. VALIDATE PYTHON LAUNCH FILES
# ============================================================================
echo "[3/6] Validating Python launch files syntax..."

LAUNCH_FILES=(
    "$PKG_DIR/launch/launch.py"
    "$PKG_DIR/launch/launch_with_nav2.py"
    "$PKG_DIR/launch/launch_slam.py"
    "$PKG_DIR/launch/launch_slam_nav.py"
    "$PKG_DIR/launch/launch_cartographer.py"
)

for launch_file in "${LAUNCH_FILES[@]}"; do
    if [ ! -f "$launch_file" ]; then
        echo "  ❌ ERROR: Launch file not found: $launch_file"
        exit 1
    fi
    
    # Check Python syntax
    if ! python3 -m py_compile "$launch_file" 2>&1; then
        echo "  ❌ ERROR: Syntax error in $(basename "$launch_file")"
        exit 1
    fi
    echo "  ✓ $(basename "$launch_file")"
done

echo "  ✓ All launch files valid"
echo ""

# ============================================================================
# 4. VALIDATE YAML CONFIGURATION FILES
# ============================================================================
echo "[4/6] Validating YAML configuration files..."

YAML_FILES=(
    "$PKG_DIR/config/robot_config.yaml"
    "$PKG_DIR/config/safety_params.yaml"
    "$PKG_DIR/config/ekf_params.yaml"
    "$PKG_DIR/config/nav2_params.yaml"
)

for yaml_file in "${YAML_FILES[@]}"; do
    if [ ! -f "$yaml_file" ]; then
        echo "  ❌ ERROR: Config file not found: $yaml_file"
        exit 1
    fi
    
    # Validate YAML syntax with Python
    if ! python3 -c "import yaml; yaml.safe_load(open('$yaml_file'))" 2>&1; then
        echo "  ❌ ERROR: Invalid YAML in $(basename "$yaml_file")"
        exit 1
    fi
    echo "  ✓ $(basename "$yaml_file")"
done

echo "  ✓ All YAML files valid"
echo ""

# ============================================================================
# 5. CHECK CMAKE CONFIGURATION
# ============================================================================
echo "[5/6] Validating CMakeLists.txt..."

CMAKE_FILE="$PKG_DIR/CMakeLists.txt"

# Check critical find_package declarations
REQUIRED_CMAKE_PACKAGES=(
    "rclcpp"
    "rclcpp_lifecycle"
    "lifecycle_msgs"
    "tf2"
    "tf2_ros"
)

for pkg in "${REQUIRED_CMAKE_PACKAGES[@]}"; do
    if ! grep -q "find_package($pkg REQUIRED)" "$CMAKE_FILE"; then
        echo "  ❌ ERROR: Missing find_package($pkg REQUIRED) in CMakeLists.txt"
        exit 1
    fi
done

# Check URDF installation
if ! grep -q "install(DIRECTORY.*urdf" "$CMAKE_FILE"; then
    echo "  ❌ ERROR: URDF installation not configured in CMakeLists.txt"
    exit 1
fi

echo "  ✓ CMakeLists.txt configuration valid"
echo ""

# ============================================================================
# 6. CHECK PACKAGE.XML
# ============================================================================
echo "[6/6] Validating package.xml..."

PACKAGE_XML="$PKG_DIR/package.xml"

# Check critical dependencies
REQUIRED_PKG_DEPS=(
    "rclcpp_lifecycle"
    "lifecycle_msgs"
    "robot_state_publisher"
    "xacro"
    "ament_index_python"
)

for dep in "${REQUIRED_PKG_DEPS[@]}"; do
    if ! grep -q "<.*depend>$dep</.*depend>" "$PACKAGE_XML"; then
        echo "  ❌ ERROR: Missing dependency '$dep' in package.xml"
        exit 1
    fi
done

echo "  ✓ package.xml dependencies valid"
echo ""

# ============================================================================
# SUMMARY
# ============================================================================
echo "=========================================="
echo "✅ ALL VALIDATION CHECKS PASSED"
echo "=========================================="
echo ""
echo "Ready to build with:"
echo "  colcon build --packages-select somanet --symlink-install"
echo ""
echo "After build, test with:"
echo "  ros2 launch somanet launch.py"
echo "  ros2 run tf2_tools view_frames  # Verify TF tree"
echo ""

exit 0
