#!/bin/bash
# Install Dependencies for GAP 2: Immutable Parameters
# ISO 13849-1 §5.2.2 Compliance

echo "═══════════════════════════════════════════════════════════════"
echo "  Installing Dependencies for Certified Parameter Validation"
echo "  (ISO 13849-1 §5.2.2 - Immutable Safety Parameters)"
echo "═══════════════════════════════════════════════════════════════"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
  echo "❌ Please run as root (sudo)"
  exit 1
fi

echo ""
echo "📦 Installing OpenSSL development libraries..."
echo "   → Required for SHA-256 cryptographic hashing"
apt-get update
apt-get install -y libssl-dev

if [ $? -eq 0 ]; then
    echo "✅ OpenSSL installed successfully"
else
    echo "❌ Failed to install OpenSSL"
    exit 1
fi

echo ""
echo "📦 Installing yaml-cpp library..."
echo "   → Required for YAML configuration parsing"
apt-get install -y libyaml-cpp-dev

if [ $? -eq 0 ]; then
    echo "✅ yaml-cpp installed successfully"
else
    echo "❌ Failed to install yaml-cpp"
    exit 1
fi

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "✅ All dependencies installed successfully!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "📝 Next Steps:"
echo "   1. Build the package:"
echo "      cd ~/workspace"
echo "      colcon build --packages-select somanet"
echo ""
echo "   2. Generate certified parameters hash:"
echo "      python3 scripts/generate_certification_hash.py \\"
echo "              config/certified_safety_params.yaml"
echo ""
echo "   3. Test the system:"
echo "      ros2 run somanet safety_supervisor_node --autostart"
echo ""
echo "   4. Try to modify a safety parameter (should be rejected):"
echo "      ros2 param set /safety_supervisor max_linear_velocity 10.0"
echo ""
