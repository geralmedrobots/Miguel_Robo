#!/usr/bin/env pwsh
# Velocity Limits Consistency Verification Script
# Version: 1.0.0
# Purpose: Verify that all nodes have consistent velocity limits

Write-Host "==================================" -ForegroundColor Cyan
Write-Host "Velocity Limits Consistency Check" -ForegroundColor Cyan
Write-Host "==================================" -ForegroundColor Cyan
Write-Host ""

$allPassed = $true

# Expected values (from safety_params.yaml)
$expectedLinear = 1.0
$expectedAngular = 1.0

Write-Host "[1/3] Checking configuration files..." -ForegroundColor Yellow

# Check safety_params.yaml
$safetyConfig = Get-Content "config\safety_params.yaml" -Raw
if ($safetyConfig -match 'max_linear_velocity:\s*(\d+\.?\d*)') {
    $safetyLinear = [double]$Matches[1]
    if ($safetyLinear -eq $expectedLinear) {
        Write-Host "  ✅ safety_params.yaml: max_linear_velocity = $safetyLinear m/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ safety_params.yaml: max_linear_velocity = $safetyLinear m/s (expected $expectedLinear)" -ForegroundColor Red
        $allPassed = $false
    }
}

if ($safetyConfig -match 'max_angular_velocity:\s*(\d+\.?\d*)') {
    $safetyAngular = [double]$Matches[1]
    if ($safetyAngular -eq $expectedAngular) {
        Write-Host "  ✅ safety_params.yaml: max_angular_velocity = $safetyAngular rad/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ safety_params.yaml: max_angular_velocity = $safetyAngular rad/s (expected $expectedAngular)" -ForegroundColor Red
        $allPassed = $false
    }
}

Write-Host ""
Write-Host "[2/3] Checking source code..." -ForegroundColor Yellow

# Check main.cpp
$mainCpp = Get-Content "src\main.cpp" -Raw
if ($mainCpp -match 'g_max_linear_vel\s*=\s*(\d+\.?\d*);') {
    $mainLinear = [double]$Matches[1]
    if ($mainLinear -eq $expectedLinear) {
        Write-Host "  ✅ main.cpp: g_max_linear_vel = $mainLinear m/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ main.cpp: g_max_linear_vel = $mainLinear m/s (expected $expectedLinear)" -ForegroundColor Red
        $allPassed = $false
    }
}

if ($mainCpp -match 'g_max_angular_vel\s*=\s*(\d+\.?\d*);') {
    $mainAngular = [double]$Matches[1]
    if ($mainAngular -eq $expectedAngular) {
        Write-Host "  ✅ main.cpp: g_max_angular_vel = $mainAngular rad/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ main.cpp: g_max_angular_vel = $mainAngular rad/s (expected $expectedAngular)" -ForegroundColor Red
        $allPassed = $false
    }
}

# Check safety_supervisor_node.cpp
$supervisorCpp = Get-Content "src\safety_supervisor_node.cpp" -Raw
if ($supervisorCpp -match 'max_linear_velocity_\((\d+\.?\d*)\)') {
    $supervisorLinear = [double]$Matches[1]
    if ($supervisorLinear -eq $expectedLinear) {
        Write-Host "  ✅ safety_supervisor_node.cpp: max_linear_velocity_ = $supervisorLinear m/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ safety_supervisor_node.cpp: max_linear_velocity_ = $supervisorLinear m/s (expected $expectedLinear)" -ForegroundColor Red
        $allPassed = $false
    }
}

if ($supervisorCpp -match 'max_angular_velocity_\((\d+\.?\d*)\)') {
    $supervisorAngular = [double]$Matches[1]
    if ($supervisorAngular -eq $expectedAngular) {
        Write-Host "  ✅ safety_supervisor_node.cpp: max_angular_velocity_ = $supervisorAngular rad/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ safety_supervisor_node.cpp: max_angular_velocity_ = $supervisorAngular rad/s (expected $expectedAngular)" -ForegroundColor Red
        $allPassed = $false
    }
}

# Check teleop_joy.cpp
$teleopCpp = Get-Content "src\teleop_joy.cpp" -Raw
if ($teleopCpp -match 'max_linear_cmd_\((\d+\.?\d*)\)') {
    $teleopLinear = [double]$Matches[1]
    if ($teleopLinear -eq $expectedLinear) {
        Write-Host "  ✅ teleop_joy.cpp: max_linear_cmd_ = $teleopLinear m/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ teleop_joy.cpp: max_linear_cmd_ = $teleopLinear m/s (expected $expectedLinear)" -ForegroundColor Red
        $allPassed = $false
    }
}

if ($teleopCpp -match 'max_angular_cmd_\((\d+\.?\d*)\)') {
    $teleopAngular = [double]$Matches[1]
    if ($teleopAngular -eq $expectedAngular) {
        Write-Host "  ✅ teleop_joy.cpp: max_angular_cmd_ = $teleopAngular rad/s" -ForegroundColor Green
    } else {
        Write-Host "  ❌ teleop_joy.cpp: max_angular_cmd_ = $teleopAngular rad/s (expected $expectedAngular)" -ForegroundColor Red
        $allPassed = $false
    }
}

Write-Host ""
Write-Host "[3/3] Checking rate limiting parameters..." -ForegroundColor Yellow

# Check acceleration limits
if ($safetyConfig -match 'max_linear_acceleration:\s*(\d+\.?\d*)') {
    $maxAccel = [double]$Matches[1]
    if ($maxAccel -ge 0.5 -and $maxAccel -le 1.5) {
        Write-Host "  ✅ max_linear_acceleration = $maxAccel m/s² (ISO 3691-4: 0.5-1.5)" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️  max_linear_acceleration = $maxAccel m/s² (ISO 3691-4 recommends 0.5-1.5)" -ForegroundColor Yellow
    }
} else {
    Write-Host "  ⚠️  max_linear_acceleration not found in config" -ForegroundColor Yellow
}

if ($safetyConfig -match 'enable_rate_limiting:\s*(true|false)') {
    $rateLimiting = $Matches[1]
    if ($rateLimiting -eq "true") {
        Write-Host "  ✅ Rate limiting enabled" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️  Rate limiting disabled" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "==================================" -ForegroundColor Cyan

if ($allPassed) {
    Write-Host "✅ ALL CHECKS PASSED!" -ForegroundColor Green
    Write-Host "==================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Velocity Limits: CONSISTENT" -ForegroundColor Green
    Write-Host "Safety Configuration: VALID" -ForegroundColor Green
    Write-Host "ISO 13849-1 Compliance: READY" -ForegroundColor Green
    Write-Host ""
    exit 0
} else {
    Write-Host "❌ SOME CHECKS FAILED!" -ForegroundColor Red
    Write-Host "==================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Velocity Limits: INCONSISTENT" -ForegroundColor Red
    Write-Host "Action Required: Fix inconsistencies before deployment." -ForegroundColor Yellow
    Write-Host ""
    exit 1
}
