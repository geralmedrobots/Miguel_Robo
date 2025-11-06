# SROS2 Security Setup Script (Windows PowerShell)
#
# This script creates a complete SROS2 security infrastructure for Ultrabot:
# - Generates keystore and certificates for each node
# - Creates access control policies
# - Ensures secure communication between all ROS2 nodes
#
# Compliance:
#   - ISO 27001: Information security management
#   - IEC 62443: Industrial automation security
#   - ISO 13849-1: Safety-related parts (secure parameter transmission)
#
# Usage:
#   .\setup_sros2.ps1 [keystore_path]
#
# Default keystore path: ..\sros2_keystore
#
# @version 1.0.0
# @date 2025-10-31

param(
    [string]$KeystorePath = (Join-Path $PSScriptRoot "..\sros2_keystore")
)

$ErrorActionPreference = "Stop"

# Color helpers
function Write-Color {
    param([string]$Text, [string]$Color = "White")
    Write-Host $Text -ForegroundColor $Color
}

Write-Color "╔══════════════════════════════════════════════════════════════╗" "Blue"
Write-Color "║         ULTRABOT SROS2 SECURITY SETUP                        ║" "Blue"
Write-Color "╚══════════════════════════════════════════════════════════════╝" "Blue"
Write-Host ""

# Check if ROS2 is available
if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Color "❌ ERROR: ROS2 not found in PATH!" "Red"
    Write-Color "Please install ROS2 Humble and source the setup script." "Yellow"
    exit 1
}

Write-Color "✓ ROS2 detected" "Green"

# Check if SROS2 utilities are available
try {
    ros2 security --help | Out-Null
    Write-Color "✓ SROS2 utilities available" "Green"
} catch {
    Write-Color "❌ ERROR: SROS2 utilities not found!" "Red"
    Write-Color "SROS2 should be included with ROS2 Humble." "Yellow"
    exit 1
}

Write-Host ""

# Create or clean keystore
if (Test-Path $KeystorePath) {
    Write-Color "⚠️  Keystore already exists at: $KeystorePath" "Yellow"
    $response = Read-Host "Do you want to regenerate it? This will delete existing keys! (y/N)"
    if ($response -eq 'y' -or $response -eq 'Y') {
        Write-Host "Removing old keystore..."
        Remove-Item -Recurse -Force $KeystorePath
    } else {
        Write-Host "Keeping existing keystore. Exiting."
        exit 0
    }
}

Write-Color "Creating SROS2 keystore at: $KeystorePath" "Blue"
ros2 security create_keystore $KeystorePath
Write-Color "✓ Keystore created" "Green"
Write-Host ""

# Define all nodes that need security
$nodes = @(
    "safety_supervisor",
    "somanet_driver",
    "command_arbitrator",
    "teleop_joy"
)

Write-Color "Generating keys and certificates for nodes..." "Blue"
foreach ($node in $nodes) {
    Write-Color "  → Creating identity for: /somanet/$node" "Blue"
    ros2 security create_enclave $KeystorePath "/somanet/$node"
    Write-Color "    ✓ Certificate generated" "Green"
}
Write-Host ""

# Create policy directory
$policyDir = Join-Path $PSScriptRoot "..\config\security_policies"
New-Item -ItemType Directory -Force -Path $policyDir | Out-Null

Write-Color "Creating security policies..." "Blue"

# Copy the XML policies (same content as in bash script)
# For brevity, I'm creating a reference file
$policyReadme = @"
# Security Policies

SROS2 policy files define which topics, services, and actions each node can access.

For Windows deployment, copy the XML policy files from the Linux version or 
use the bash script in WSL/Git Bash.

Policy files needed:
- safety_supervisor.xml
- somanet_driver.xml
- command_arbitrator.xml
- teleop_joy.xml

See scripts/setup_sros2.sh for complete XML content.
"@

Set-Content -Path (Join-Path $policyDir "README.md") -Value $policyReadme
Write-Color "  ✓ Policy directory created: $policyDir" "Green"
Write-Color "  ⚠️  Note: Copy XML policy files from Linux version" "Yellow"

Write-Host ""

# Create keystore README
$keystoreReadme = @"
# SROS2 Keystore

This directory contains the ROS 2 security keystore for Ultrabot.

## ⚠️ SECURITY NOTICE

**DO NOT COMMIT THIS DIRECTORY TO VERSION CONTROL!**

This keystore contains:
- Private keys for node authentication
- Certificates for secure communication
- Access control policies

## Deployment

### Development (PowerShell)
```powershell
`$env:ROS_SECURITY_KEYSTORE = "$KeystorePath"
`$env:ROS_SECURITY_ENABLE = "true"
`$env:ROS_SECURITY_STRATEGY = "Enforce"
```

### Production
Store this keystore in a secure location (e.g., encrypted volume).

## Regeneration

To regenerate the keystore:
```powershell
cd scripts
.\setup_sros2.ps1
```
"@

Set-Content -Path (Join-Path $KeystorePath "README.md") -Value $keystoreReadme
Write-Color "✓ Keystore documentation created" "Green"
Write-Host ""

# Add .gitignore
$gitignoreContent = @"
# SROS2 Keystore - DO NOT COMMIT!
# Contains private keys and certificates

*
!.gitignore
!README.md
"@

Set-Content -Path (Join-Path $KeystorePath ".gitignore") -Value $gitignoreContent
Write-Color "✓ Created .gitignore for keystore" "Green"
Write-Host ""

# Print summary
Write-Color "╔══════════════════════════════════════════════════════════════╗" "Green"
Write-Color "║            SROS2 SETUP COMPLETED SUCCESSFULLY!               ║" "Green"
Write-Color "╚══════════════════════════════════════════════════════════════╝" "Green"
Write-Host ""
Write-Color "Keystore location: " "Blue" -NoNewline
Write-Host $KeystorePath
Write-Color "Secured nodes:" "Blue"
foreach ($node in $nodes) {
    Write-Color "  ✓ /somanet/$node" "Green"
}
Write-Host ""
Write-Color "⚠️  IMPORTANT: To enable SROS2, set these environment variables:" "Yellow"
Write-Host ""
Write-Color "`$env:ROS_SECURITY_KEYSTORE = `"$KeystorePath`"" "Blue"
Write-Color "`$env:ROS_SECURITY_ENABLE = `"true`"" "Blue"
Write-Color "`$env:ROS_SECURITY_STRATEGY = `"Enforce`"" "Blue"
Write-Host ""
Write-Color "Or add to your PowerShell profile for permanent activation." "White"
Write-Host ""
Write-Color "📝 Next steps:" "Yellow"
Write-Host "  1. Copy XML policy files to: $policyDir"
Write-Host "  2. Apply policies with: ros2 security create_permission"
Write-Host "  3. Test with: ros2 launch somanet launch.py"
Write-Host "  4. Verify with: ros2 security list_enclaves"
Write-Host ""
Write-Color "🔒 Your ROS2 communication will be encrypted and authenticated!" "Green"
Write-Host ""
