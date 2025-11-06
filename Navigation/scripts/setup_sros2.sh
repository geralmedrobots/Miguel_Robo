#!/bin/bash
###############################################################################
# SROS2 Security Setup Script
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
#   ./setup_sros2.sh [keystore_path]
#
# Default keystore path: ../sros2_keystore
#
# @version 1.0.0
# @date 2025-10-31
###############################################################################

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default keystore path (relative to script location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYSTORE_PATH="${1:-$SCRIPT_DIR/../sros2_keystore}"

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         ULTRABOT SROS2 SECURITY SETUP                        ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ERROR: ROS2 not sourced!${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS2 $ROS_DISTRO detected"

# Check if SROS2 utilities are available
if ! command -v ros2 security &> /dev/null; then
    echo -e "${RED}❌ ERROR: SROS2 utilities not found!${NC}"
    echo "Install with: sudo apt-get install ros-${ROS_DISTRO}-sros2"
    exit 1
fi

echo -e "${GREEN}✓${NC} SROS2 utilities available"
echo ""

# Create or clean keystore
if [ -d "$KEYSTORE_PATH" ]; then
    echo -e "${YELLOW}⚠️  Keystore already exists at: $KEYSTORE_PATH${NC}"
    read -p "Do you want to regenerate it? This will delete existing keys! (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing old keystore..."
        rm -rf "$KEYSTORE_PATH"
    else
        echo "Keeping existing keystore. Exiting."
        exit 0
    fi
fi

echo -e "${BLUE}Creating SROS2 keystore at: $KEYSTORE_PATH${NC}"
ros2 security create_keystore "$KEYSTORE_PATH"
echo -e "${GREEN}✓${NC} Keystore created"
echo ""

# Define all nodes that need security
NODES=(
    "safety_supervisor"
    "somanet_driver"
    "command_arbitrator"
    "teleop_joy"
)

echo -e "${BLUE}Generating keys and certificates for nodes...${NC}"
for node in "${NODES[@]}"; do
    echo -e "  ${BLUE}→${NC} Creating identity for: /somanet/$node"
    ros2 security create_enclave "$KEYSTORE_PATH" "/somanet/$node"
    echo -e "    ${GREEN}✓${NC} Certificate generated"
done
echo ""

# Create policy directory
POLICY_DIR="$SCRIPT_DIR/../config/security_policies"
mkdir -p "$POLICY_DIR"

echo -e "${BLUE}Creating security policies...${NC}"

# Policy for Safety Supervisor (critical node - strict policies)
cat > "$POLICY_DIR/safety_supervisor.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Security Policy: Safety Supervisor Node
  Compliance: ISO 13849-1 (safety-critical communication)
  
  This node MUST:
  - Subscribe to cmd_vel (from teleop/navigation)
  - Subscribe to odom (for plausibility checks)
  - Subscribe to deadman_status (mandatory for safety)
  - Subscribe to safety/fault_events (driver faults)
  - Publish wheel_cmd_safe (validated commands only)
  - Publish safety_stop (emergency stop signal)
  - Publish /diagnostics (system health)
-->
<policy version="0.2.0" xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <enclave path="/somanet/safety_supervisor">
      <profiles>
        <profile ns="/" node="safety_supervisor">
          
          <!-- Subscriptions (inputs to safety supervisor) -->
          <topics subscribe="ALLOW">
            <topic>cmd_vel</topic>
            <topic>odom</topic>
            <topic>deadman_status</topic>
            <topic>safety/fault_events</topic>
          </topics>
          
          <!-- Publications (outputs from safety supervisor) -->
          <topics publish="ALLOW">
            <topic>wheel_cmd_safe</topic>
            <topic>safety_stop</topic>
            <topic>/diagnostics</topic>
          </topics>
          
          <!-- Services (lifecycle management) -->
          <services reply="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
            <service>~/get_available_states</service>
            <service>~/get_available_transitions</service>
            <service>~/get_transition_graph</service>
          </services>
          
          <services request="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
          </services>
          
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
EOF
echo -e "  ${GREEN}✓${NC} Policy created: safety_supervisor.xml"

# Policy for Somanet Driver (hardware interface)
cat > "$POLICY_DIR/somanet_driver.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Security Policy: Somanet Driver Node
  Compliance: ISO 13849-1 (safety-critical actuation)
  
  This node MUST:
  - Subscribe to wheel_cmd_safe (ONLY from safety_supervisor)
  - Publish odom (robot state for feedback)
  - Publish safety/fault_events (hardware faults)
  - Publish tf (odometry transform)
-->
<policy version="0.2.0" xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <enclave path="/somanet/somanet_driver">
      <profiles>
        <profile ns="/" node="somanet_driver">
          
          <!-- Subscriptions -->
          <topics subscribe="ALLOW">
            <topic>wheel_cmd_safe</topic>
          </topics>
          
          <!-- Publications -->
          <topics publish="ALLOW">
            <topic>odom</topic>
            <topic>safety/fault_events</topic>
            <topic>/tf</topic>
            <topic>/diagnostics</topic>
          </topics>
          
          <!-- Lifecycle services -->
          <services reply="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
            <service>~/get_available_states</service>
            <service>~/get_available_transitions</service>
            <service>~/get_transition_graph</service>
          </services>
          
          <services request="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
          </services>
          
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
EOF
echo -e "  ${GREEN}✓${NC} Policy created: somanet_driver.xml"

# Policy for Command Arbitrator
cat > "$POLICY_DIR/command_arbitrator.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Security Policy: Command Arbitrator Node
  Compliance: ISO 3691-4 (command priority management)
  
  This node arbitrates between multiple command sources
  and publishes unified commands to the safety supervisor.
-->
<policy version="0.2.0" xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <enclave path="/somanet/command_arbitrator">
      <profiles>
        <profile ns="/" node="command_arbitrator">
          
          <!-- Subscriptions (multiple command sources) -->
          <topics subscribe="ALLOW">
            <topic>cmd_vel_teleop</topic>
            <topic>cmd_vel_nav</topic>
            <topic>cmd_vel_safety</topic>
            <topic>emergency_stop</topic>
          </topics>
          
          <!-- Publications -->
          <topics publish="ALLOW">
            <topic>cmd_vel</topic>
            <topic>/diagnostics</topic>
          </topics>
          
          <!-- Lifecycle services -->
          <services reply="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
            <service>~/get_available_states</service>
            <service>~/get_available_transitions</service>
            <service>~/get_transition_graph</service>
          </services>
          
          <services request="ALLOW">
            <service>~/change_state</service>
            <service>~/get_state</service>
          </services>
          
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
EOF
echo -e "  ${GREEN}✓${NC} Policy created: command_arbitrator.xml"

# Policy for Teleop Joy (user input)
cat > "$POLICY_DIR/teleop_joy.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!--
  Security Policy: Teleoperation Joystick Node
  Compliance: ISO 3691-4 (manual control mode)
  
  This node reads joystick input and publishes
  teleoperation commands with deadman button state.
-->
<policy version="0.2.0" xmlns:xi="http://www.w3.org/2001/XInclude">
  <enclaves>
    <enclave path="/somanet/teleop_joy">
      <profiles>
        <profile ns="/" node="teleop_joy">
          
          <!-- Subscriptions -->
          <topics subscribe="ALLOW">
            <topic>joy</topic>
            <topic>safety_stop</topic>
          </topics>
          
          <!-- Publications -->
          <topics publish="ALLOW">
            <topic>cmd_vel</topic>
            <topic>deadman_status</topic>
            <topic>/diagnostics</topic>
          </topics>
          
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
EOF
echo -e "  ${GREEN}✓${NC} Policy created: teleop_joy.xml"

echo ""
echo -e "${BLUE}Applying policies to keystore...${NC}"
for node in "${NODES[@]}"; do
    if [ -f "$POLICY_DIR/${node}.xml" ]; then
        echo -e "  ${BLUE}→${NC} Applying policy for: $node"
        ros2 security create_permission "$KEYSTORE_PATH" "/somanet/$node" "$POLICY_DIR/${node}.xml"
        echo -e "    ${GREEN}✓${NC} Policy applied"
    fi
done
echo ""

# Create README for keystore
cat > "$KEYSTORE_PATH/README.md" << 'EOF'
# SROS2 Keystore

This directory contains the ROS 2 security keystore for Ultrabot.

## ⚠️ SECURITY NOTICE

**DO NOT COMMIT THIS DIRECTORY TO VERSION CONTROL!**

This keystore contains:
- Private keys for node authentication
- Certificates for secure communication
- Access control policies

## Structure

```
sros2_keystore/
├── enclaves/
│   └── somanet/
│       ├── safety_supervisor/
│       │   ├── cert.pem           # Public certificate
│       │   ├── key.pem            # Private key (SECRET!)
│       │   ├── governance.p7s     # Governance document
│       │   └── permissions.p7s    # Access control policy
│       ├── somanet_driver/
│       ├── command_arbitrator/
│       └── teleop_joy/
├── public/
│   ├── ca.cert.pem               # Certificate Authority
│   └── governance.p7s
└── private/
    └── ca.key.pem                # CA private key (SECRET!)
```

## Deployment

### Development
```bash
export ROS_SECURITY_KEYSTORE=$PWD/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

### Production
Store this keystore in a secure location (e.g., encrypted volume).
Use Ansible/Puppet to deploy to robots.

## Regeneration

To regenerate the keystore (e.g., after adding new nodes):
```bash
cd scripts
./setup_sros2.sh
```

## Compliance

- **ISO 27001**: Secure key management
- **IEC 62443**: Industrial network security
- **NIST SP 800-57**: Key management best practices
EOF

echo -e "${GREEN}✓${NC} Keystore documentation created"
echo ""

# Add .gitignore to prevent accidental commits
cat > "$KEYSTORE_PATH/.gitignore" << 'EOF'
# SROS2 Keystore - DO NOT COMMIT!
# Contains private keys and certificates

*
!.gitignore
!README.md
EOF

echo -e "${GREEN}✓${NC} Created .gitignore for keystore"
echo ""

# Print summary
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║            SROS2 SETUP COMPLETED SUCCESSFULLY!               ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Keystore location:${NC} $KEYSTORE_PATH"
echo -e "${BLUE}Secured nodes:${NC}"
for node in "${NODES[@]}"; do
    echo -e "  ${GREEN}✓${NC} /somanet/$node"
done
echo ""
echo -e "${YELLOW}⚠️  IMPORTANT: To enable SROS2, set these environment variables:${NC}"
echo ""
echo -e "${BLUE}export ROS_SECURITY_KEYSTORE=$KEYSTORE_PATH${NC}"
echo -e "${BLUE}export ROS_SECURITY_ENABLE=true${NC}"
echo -e "${BLUE}export ROS_SECURITY_STRATEGY=Enforce${NC}"
echo ""
echo -e "Or add to your ${BLUE}~/.bashrc${NC} for permanent activation."
echo ""
echo -e "${YELLOW}📝 Next steps:${NC}"
echo "  1. Review security policies in: $POLICY_DIR"
echo "  2. Test with: ros2 launch somanet launch.py"
echo "  3. Verify secure communication with: ros2 security list_enclaves"
echo "  4. Monitor with: ros2 topic echo /rosout (check for security errors)"
echo ""
echo -e "${GREEN}🔒 Your ROS2 communication is now encrypted and authenticated!${NC}"
echo ""
