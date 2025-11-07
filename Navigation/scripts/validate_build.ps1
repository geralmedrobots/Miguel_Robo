#!/usr/bin/env pwsh
# ============================================================================
# ULTRABOT NAVIGATION - BUILD VALIDATION SCRIPT (PowerShell)
# ============================================================================
# Validates integration and build readiness before colcon build
# Usage: .\scripts\validate_build.ps1
# ============================================================================

$ErrorActionPreference = "Continue"
$script:errorCount = 0
$script:warningCount = 0

function Write-Status {
    param([string]$message, [string]$type = "INFO")
    $timestamp = Get-Date -Format "HH:mm:ss"
    switch ($type) {
        "OK"      { Write-Host "[$timestamp] ✓ $message" -ForegroundColor Green }
        "ERROR"   { Write-Host "[$timestamp] ✗ $message" -ForegroundColor Red; $script:errorCount++ }
        "WARNING" { Write-Host "[$timestamp] ⚠ $message" -ForegroundColor Yellow; $script:warningCount++ }
        "INFO"    { Write-Host "[$timestamp] ℹ $message" -ForegroundColor Cyan }
        "SECTION" { Write-Host "`n========================================" -ForegroundColor Magenta
                    Write-Host "  $message" -ForegroundColor Magenta
                    Write-Host "========================================`n" -ForegroundColor Magenta }
    }
}

# ============================================================================
# 1. CHECK FILE EXISTENCE
# ============================================================================
Write-Status "FILE EXISTENCE CHECKS" "SECTION"

$requiredFiles = @(
    "CMakeLists.txt",
    "package.xml",
    "urdf/ultrabot.urdf.xacro",
    "config/nav2_params.yaml",
    "config/ekf_params.yaml",
    "config/sensors_config.yaml",
    "config/robot_config.yaml",
    "config/safety_params.yaml",
    "launch/launch.py",
    "launch/launch_with_nav2.py",
    "launch/launch_sensors.py",
    "include/odometry_calculator.hpp",
    "include/ethercat_driver.hpp",
    "include/certified_params_validator.hpp",
    "src/main.cpp",
    "src/safety_supervisor_node.cpp",
    "src/command_arbitrator_node.cpp",
    "src/command_mux_node.cpp"
)

foreach ($file in $requiredFiles) {
    if (Test-Path $file) {
        Write-Status "Found: $file" "OK"
    } else {
        Write-Status "Missing: $file" "ERROR"
    }
}

# ============================================================================
# 2. VALIDATE CMAKE DEPENDENCIES
# ============================================================================
Write-Status "CMAKE DEPENDENCY VALIDATION" "SECTION"

$cmakeContent = Get-Content "CMakeLists.txt" -Raw

$cmakeDeps = @(
    "ament_cmake",
    "rclcpp",
    "rclcpp_lifecycle",
    "lifecycle_msgs",
    "geometry_msgs",
    "nav_msgs",
    "sensor_msgs",
    "tf2",
    "tf2_ros"
)

foreach ($dep in $cmakeDeps) {
    if ($cmakeContent -match "find_package\($dep") {
        Write-Status "CMake dependency: $dep" "OK"
    } else {
        Write-Status "Missing find_package($dep)" "ERROR"
    }
}

# Check URDF installation
if ($cmakeContent -match "install\(DIRECTORY.*urdf") {
    Write-Status "URDF installation configured" "OK"
} else {
    Write-Status "URDF installation missing in CMakeLists.txt" "WARNING"
}

# ============================================================================
# 3. VALIDATE PACKAGE.XML DEPENDENCIES
# ============================================================================
Write-Status "PACKAGE.XML DEPENDENCY VALIDATION" "SECTION"

$packageContent = Get-Content "package.xml" -Raw

$packageDeps = @(
    "rclcpp",
    "rclcpp_lifecycle",
    "lifecycle_msgs",
    "ament_index_python",
    "robot_state_publisher",
    "xacro",
    "nav2_bringup",
    "robot_localization"
)

foreach ($dep in $packageDeps) {
    if ($packageContent -match "<depend>$dep</depend>|<exec_depend>$dep</exec_depend>") {
        Write-Status "Package dependency: $dep" "OK"
    } else {
        Write-Status "Missing dependency in package.xml: $dep" "ERROR"
    }
}

# ============================================================================
# 4. VALIDATE URDF SYNTAX
# ============================================================================
Write-Status "URDF/XACRO SYNTAX VALIDATION" "SECTION"

$urdfFile = "urdf/ultrabot.urdf.xacro"
if (Test-Path $urdfFile) {
    try {
        [xml]$urdfXml = Get-Content $urdfFile
        Write-Status "URDF XML syntax valid" "OK"
        
        # Check for required tags
        if ($urdfXml.robot) {
            Write-Status "Root <robot> tag found" "OK"
        } else {
            Write-Status "Missing <robot> root tag" "ERROR"
        }
        
        # Check for essential frames
        $urdfContent = Get-Content $urdfFile -Raw
        $requiredFrames = @("base_footprint", "base_link", "scan_front_link", "ouster_link", "imu_link")
        foreach ($frame in $requiredFrames) {
            if ($urdfContent -match $frame) {
                Write-Status "Frame defined: $frame" "OK"
            } else {
                Write-Status "Missing frame: $frame" "WARNING"
            }
        }
    } catch {
        Write-Status "URDF XML parsing error: $($_.Exception.Message)" "ERROR"
    }
} else {
    Write-Status "URDF file not found" "ERROR"
}

# ============================================================================
# 5. VALIDATE PYTHON LAUNCH FILES
# ============================================================================
Write-Status "PYTHON LAUNCH FILE VALIDATION" "SECTION"

$launchFiles = Get-ChildItem -Path "launch" -Filter "*.py"

foreach ($file in $launchFiles) {
    # Basic syntax check (look for common issues)
    $content = Get-Content $file.FullName -Raw
    
    if ($content -match "from launch import LaunchDescription") {
        Write-Status "$($file.Name): LaunchDescription import OK" "OK"
    } else {
        Write-Status "$($file.Name): Missing LaunchDescription import" "WARNING"
    }
    
    if ($content -match "def generate_launch_description") {
        Write-Status "$($file.Name): generate_launch_description() OK" "OK"
    } else {
        Write-Status "$($file.Name): Missing generate_launch_description()" "ERROR"
    }
    
    # Check for potential unbound variable (realsense_pkg_dir issue)
    if ($file.Name -eq "launch_sensors.py") {
        if ($content -match "realsense_pkg_dir.*=.*get_package_share_directory") {
            $lines = $content -split "`n"
            $tryIndex = -1
            $exceptIndex = -1
            
            for ($i = 0; $i -lt $lines.Count; $i++) {
                if ($lines[$i] -match "try:") { $tryIndex = $i }
                if ($lines[$i] -match "except" -and $tryIndex -ge 0) { 
                    $exceptIndex = $i 
                    break
                }
            }
            
            if ($tryIndex -ge 0 -and $exceptIndex -ge 0) {
                $tryBlock = $lines[$tryIndex..$exceptIndex] -join "`n"
                if ($tryBlock -match "realsense_front_left.*=.*IncludeLaunchDescription" -and 
                    $tryBlock -match "realsense_front_right.*=.*IncludeLaunchDescription") {
                    Write-Status "launch_sensors.py: RealSense exception handling OK" "OK"
                } else {
                    Write-Status "launch_sensors.py: RealSense variables not in same try block" "WARNING"
                }
            }
        }
    }
}

# ============================================================================
# 6. VALIDATE YAML CONFIGS
# ============================================================================
Write-Status "YAML CONFIG VALIDATION" "SECTION"

$yamlFiles = Get-ChildItem -Path "config" -Filter "*.yaml"

foreach ($file in $yamlFiles) {
    try {
        # Basic YAML syntax check
        $content = Get-Content $file.FullName -Raw
        
        # Check for tabs (YAML doesn't allow tabs)
        if ($content -match "`t") {
            Write-Status "$($file.Name): Contains TAB characters (use spaces)" "ERROR"
        } else {
            Write-Status "$($file.Name): No TAB characters" "OK"
        }
        
        # Check ROS2 parameter structure
        if ($content -match "ros__parameters:") {
            Write-Status "$($file.Name): ROS2 parameter structure OK" "OK"
        }
        
        # Specific checks for nav2_params.yaml
        if ($file.Name -eq "nav2_params.yaml") {
            if ($content -match "observation_sources:.*scan.*scan_realsense") {
                Write-Status "nav2_params.yaml: Multi-sensor costmap configured" "OK"
            } elseif ($content -match "observation_sources:.*scan") {
                Write-Status "nav2_params.yaml: Single sensor costmap (consider multi-sensor)" "WARNING"
            }
        }
        
        # Check for EKF IMU configuration
        if ($file.Name -eq "ekf_params.yaml") {
            if ($content -match "imu0:") {
                Write-Status "ekf_params.yaml: IMU fusion configured" "OK"
            } else {
                Write-Status "ekf_params.yaml: IMU not configured (wheels only)" "WARNING"
            }
        }
        
    } catch {
        Write-Status "$($file.Name): Validation error: $($_.Exception.Message)" "ERROR"
    }
}

# ============================================================================
# 7. CHECK C++ HEADERS
# ============================================================================
Write-Status "C++ HEADER VALIDATION" "SECTION"

$headers = Get-ChildItem -Path "include" -Filter "*.hpp"
$sources = Get-ChildItem -Path "src" -Filter "*.cpp"

foreach ($header in $headers) {
    $headerName = $header.Name
    $includePattern = "#include `"$headerName`""
    
    $found = $false
    foreach ($source in $sources) {
        $content = Get-Content $source.FullName -Raw
        if ($content -match [regex]::Escape($includePattern)) {
            $found = $true
            break
        }
    }
    
    if ($found) {
        Write-Status "Header $headerName is used" "OK"
    } else {
        Write-Status "Header $headerName may be unused" "INFO"
    }
}

# Check for missing headers in source files
foreach ($source in $sources) {
    $content = Get-Content $source.FullName -Raw
    $includes = [regex]::Matches($content, '#include\s+"([^"]+)"') | ForEach-Object { $_.Groups[1].Value }
    
    foreach ($include in $includes) {
        if ($include -match "\.hpp$") {
            if (Test-Path "include/$include") {
                Write-Status "$($source.Name): include $include found" "OK"
            } else {
                Write-Status "$($source.Name): include $include NOT FOUND" "ERROR"
            }
        }
    }
}

# ============================================================================
# 8. CHECK FOR COMMON ISSUES
# ============================================================================
Write-Status "COMMON INTEGRATION ISSUES CHECK" "SECTION"

# Check behavior tree files referenced in nav2_params.yaml
$nav2Content = Get-Content "config/nav2_params.yaml" -Raw
if ($nav2Content -match '\$\(find-pkg-share somanet\)/config/behavior_trees/(\S+\.xml)') {
    $btFile = $Matches[1]
    if (Test-Path "config/behavior_trees/$btFile") {
        Write-Status "Behavior tree file exists: $btFile" "OK"
    } else {
        Write-Status "Behavior tree file missing: $btFile" "ERROR"
    }
}

# Check for robot_state_publisher in launch files
$launchWithNav2 = Get-Content "launch/launch_with_nav2.py" -Raw
if ($launchWithNav2 -match "robot_state_publisher") {
    Write-Status "robot_state_publisher configured in launch_with_nav2.py" "OK"
} else {
    Write-Status "robot_state_publisher missing in launch_with_nav2.py" "WARNING"
}

# Check for SOEM mock implementation
$mainCpp = Get-Content "src/main.cpp" -Raw
if ($mainCpp -match "#ifdef HAVE_SOEM" -and $mainCpp -match "#else.*mock.*SOEM") {
    Write-Status "SOEM mock implementation available" "OK"
} else {
    Write-Status "SOEM mock implementation missing (builds will fail without SOEM)" "WARNING"
}

# ============================================================================
# 9. FINAL SUMMARY
# ============================================================================
Write-Status "VALIDATION SUMMARY" "SECTION"

Write-Host ""
Write-Host "════════════════════════════════════════" -ForegroundColor Magenta
Write-Host "  VALIDATION RESULTS" -ForegroundColor Magenta
Write-Host "════════════════════════════════════════" -ForegroundColor Magenta
Write-Host ""

if ($script:errorCount -eq 0 -and $script:warningCount -eq 0) {
    Write-Host "  ✓ ALL CHECKS PASSED" -ForegroundColor Green
    Write-Host "  Build should succeed without errors" -ForegroundColor Green
    $exitCode = 0
} elseif ($script:errorCount -eq 0) {
    Write-Host "  ⚠ $script:warningCount WARNING(S) FOUND" -ForegroundColor Yellow
    Write-Host "  Build should succeed, but review warnings" -ForegroundColor Yellow
    $exitCode = 0
} else {
    Write-Host "  ✗ $script:errorCount ERROR(S) FOUND" -ForegroundColor Red
    Write-Host "  ⚠ $script:warningCount WARNING(S) FOUND" -ForegroundColor Yellow
    Write-Host "  Fix errors before building" -ForegroundColor Red
    $exitCode = 1
}

Write-Host ""
Write-Host "════════════════════════════════════════" -ForegroundColor Magenta
Write-Host ""

Write-Host "Next steps:" -ForegroundColor Cyan
if ($exitCode -eq 0) {
    Write-Host "  1. Run: colcon build --packages-select somanet --symlink-install" -ForegroundColor White
    Write-Host "  2. Source: . install/setup.ps1" -ForegroundColor White
    Write-Host "  3. Test: ros2 launch somanet launch.py" -ForegroundColor White
} else {
    Write-Host "  1. Fix the errors listed above" -ForegroundColor White
    Write-Host "  2. Re-run this validation script" -ForegroundColor White
    Write-Host "  3. Then proceed with colcon build" -ForegroundColor White
}
Write-Host ""

exit $exitCode
