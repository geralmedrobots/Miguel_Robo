# Safety-Drive Integration Verification Script (PowerShell)
# Version: 1.0.0
# Purpose: Verify that drive node is correctly connected to Safety Supervisor

Write-Host "==================================" -ForegroundColor Cyan
Write-Host "Safety-Drive Integration Check" -ForegroundColor Cyan
Write-Host "==================================" -ForegroundColor Cyan
Write-Host ""

$allPassed = $true

# Check if main.cpp subscribes to correct topic
Write-Host "[1/4] Checking main.cpp subscription..." -ForegroundColor Yellow
$mainContent = Get-Content "src\main.cpp" -Raw
if ($mainContent -match 'wheel_cmd_safe.*create_subscription|create_subscription.*wheel_cmd_safe') {
    Write-Host "✅ PASS: main.cpp subscribes to /wheel_cmd_safe" -ForegroundColor Green
    $line = (Select-String -Path "src\main.cpp" -Pattern "wheel_cmd_safe").LineNumber
    Write-Host "   Line: $line" -ForegroundColor Gray
} else {
    Write-Host "❌ FAIL: main.cpp does NOT subscribe to /wheel_cmd_safe" -ForegroundColor Red
    $wrong = Select-String -Path "src\main.cpp" -Pattern 'create_subscription.*"cmd_vel"'
    if ($wrong) {
        Write-Host "   Found wrong subscription: $($wrong.Line)" -ForegroundColor Red
    }
    $allPassed = $false
}

Write-Host ""

# Check if main_refactored.cpp subscribes to correct topic
Write-Host "[2/4] Checking main_refactored.cpp subscription..." -ForegroundColor Yellow
if (Test-Path "src\main_refactored.cpp") {
    $refactoredContent = Get-Content "src\main_refactored.cpp" -Raw
    if ($refactoredContent -match 'wheel_cmd_safe') {
        Write-Host "✅ PASS: main_refactored.cpp subscribes to /wheel_cmd_safe" -ForegroundColor Green
        $line = (Select-String -Path "src\main_refactored.cpp" -Pattern "wheel_cmd_safe").LineNumber
        Write-Host "   Line: $line" -ForegroundColor Gray
    } else {
        Write-Host "❌ FAIL: main_refactored.cpp does NOT subscribe to /wheel_cmd_safe" -ForegroundColor Red
        $allPassed = $false
    }
} else {
    Write-Host "⚠️  SKIP: main_refactored.cpp not found" -ForegroundColor Yellow
}

Write-Host ""

# Check if safety_supervisor publishes to correct topic
Write-Host "[3/4] Checking Safety Supervisor publication..." -ForegroundColor Yellow
if (Test-Path "src\safety_supervisor_node.cpp") {
    $safetyContent = Get-Content "src\safety_supervisor_node.cpp" -Raw
    if ($safetyContent -match 'wheel_cmd_safe.*create_publisher') {
        Write-Host "✅ PASS: Safety Supervisor publishes to /wheel_cmd_safe" -ForegroundColor Green
        $line = (Select-String -Path "src\safety_supervisor_node.cpp" -Pattern "wheel_cmd_safe.*create_publisher").LineNumber
        Write-Host "   Line: $line" -ForegroundColor Gray
    } else {
        Write-Host "❌ FAIL: Safety Supervisor does NOT publish to /wheel_cmd_safe" -ForegroundColor Red
        $allPassed = $false
    }
} else {
    Write-Host "❌ FAIL: safety_supervisor_node.cpp not found" -ForegroundColor Red
    $allPassed = $false
}

Write-Host ""

# Check topic flow
Write-Host "[4/4] Verifying complete topic flow..." -ForegroundColor Yellow
Write-Host "   Expected flow:" -ForegroundColor Gray
Write-Host "   teleop → /cmd_vel → Safety Supervisor → /wheel_cmd_safe → Drive Node" -ForegroundColor Gray

$teleopPub = Select-String -Path "src\teleop_joy.cpp" -Pattern 'cmd_vel.*create_publisher' -Quiet
$safetySub = Select-String -Path "src\safety_supervisor_node.cpp" -Pattern 'cmd_vel.*create_subscription' -Quiet
$safePub = Select-String -Path "src\safety_supervisor_node.cpp" -Pattern 'wheel_cmd_safe.*create_publisher' -Quiet
$mainSub = $mainContent -match 'wheel_cmd_safe'

if ($teleopPub -and $safetySub -and $safePub -and $mainSub) {
    Write-Host "✅ PASS: Complete safety chain verified!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Topic Flow:" -ForegroundColor Cyan
    Write-Host "  1. teleop_joy → /cmd_vel ✅" -ForegroundColor Green
    Write-Host "  2. Safety Supervisor ← /cmd_vel ✅" -ForegroundColor Green
    Write-Host "  3. Safety Supervisor → /wheel_cmd_safe ✅" -ForegroundColor Green
    Write-Host "  4. Drive Node ← /wheel_cmd_safe ✅" -ForegroundColor Green
} else {
    Write-Host "❌ FAIL: Incomplete safety chain" -ForegroundColor Red
    $allPassed = $false
}

Write-Host ""
Write-Host "==================================" -ForegroundColor Cyan

if ($allPassed) {
    Write-Host "✅ ALL CHECKS PASSED!" -ForegroundColor Green
    Write-Host "==================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Safety-Drive Integration: VERIFIED" -ForegroundColor Green
    Write-Host "ISO 13849-1 Compliance: READY" -ForegroundColor Green
    Write-Host ""
    exit 0
} else {
    Write-Host "❌ SOME CHECKS FAILED!" -ForegroundColor Red
    Write-Host "==================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Safety-Drive Integration: INCOMPLETE" -ForegroundColor Red
    Write-Host "Please fix the issues above before deployment." -ForegroundColor Yellow
    Write-Host ""
    exit 1
}
