#!/usr/bin/env python3
"""
Generate SHA-256 certification hash for safety parameters

This script computes the cryptographic hash of certified safety parameters
to ensure they cannot be modified without detection.

Usage:
    python generate_certification_hash.py [config_file]
    
Example:
    python generate_certification_hash.py ../config/certified_safety_params.yaml

Compliance:
    - ISO 13849-1 §5.2.2: Protection of safety parameters
    - IEC 62304 §5.1.1: Configuration management
    
@version 1.0.0
@date 2025-10-28
"""

import hashlib
import hmac
import sys
import os
import re
from datetime import datetime, timedelta

def get_secret_key(config_path):
    """Read HMAC secret key from file."""
    key_path = os.path.join(os.path.dirname(config_path), "cert.key")
    if not os.path.exists(key_path):
        return None
    with open(key_path, 'r', encoding='utf-8') as f:
        return f.read().strip()


def build_canonical(params):
    """Return canonical string representation for parameters."""
    sorted_params = sorted(params.items())
    return "".join(f"{name}={data['value']:.6f};" for name, data in sorted_params)


def compute_params_hash(canonical_str):
    """Compute SHA-256 hash for canonical representation."""
    return hashlib.sha256(canonical_str.encode('utf-8')).hexdigest()


def compute_params_hmac(canonical_str, key):
    """Compute HMAC-SHA256 for canonical representation."""
    mac = hmac.new(key.encode('utf-8'), canonical_str.encode('utf-8'), hashlib.sha256)
    return mac.hexdigest()

def load_config(config_path):
    """Load YAML configuration file (simple parser)"""
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    params = {}
    certification = {}
    current_param = None
    in_cert_section = False
    
    for line in lines:
        # Skip comments and empty lines
        if line.strip().startswith('#') or not line.strip():
            continue
        
        if line.startswith('certification:'):
            in_cert_section = True
            continue

        if in_cert_section:
            hash_match = re.match(r'^  hash:\s+"?([a-fA-F0-9]+)"?', line)
            hmac_match = re.match(r'^  hmac:\s+"?([a-fA-F0-9]+)"?', line)
            date_match = re.match(r'^  date:\s+"([^"]+)"', line)
            cert_id_match = re.match(r'^  certificate_id:\s+"([^"]+)"', line)
            valid_match = re.match(r'^  valid_until:\s+"([^"]+)"', line)
            certified_by_match = re.match(r'^  certified_by:\s+"([^"]+)"', line)
            project_match = re.match(r'^  project_version:\s+"([^"]+)"', line)

            if hash_match:
                certification['hash'] = hash_match.group(1)
                continue
            if hmac_match:
                certification['hmac'] = hmac_match.group(1)
                continue
            if date_match:
                certification['date'] = date_match.group(1)
                continue
            if cert_id_match:
                certification['certificate_id'] = cert_id_match.group(1)
                continue
            if valid_match:
                certification['valid_until'] = valid_match.group(1)
                continue
            if certified_by_match:
                certification['certified_by'] = certified_by_match.group(1)
                continue
            if project_match:
                certification['project_version'] = project_match.group(1)
                continue

        match = re.match(r'^  (\w+):\s*$', line)
        if match:
            current_param = match.group(1)
            params[current_param] = {}
            continue
        
        # Match value field (e.g., "    value: 1.0")
        if current_param:
            value_match = re.match(r'^\s+value:\s+([\d.]+)', line)
            if value_match:
                params[current_param]['value'] = float(value_match.group(1))
                continue
            
            unit_match = re.match(r'^\s+unit:\s+"([^"]+)"', line)
            if unit_match:
                params[current_param]['unit'] = unit_match.group(1)
                continue
            
            std_match = re.match(r'^\s+standard:\s+"([^"]+)"', line)
            if std_match:
                params[current_param]['standard'] = std_match.group(1)
    
    result = {'safety_limits': params}
    if certification:
        result['certification'] = certification
    return result

def save_config(config_path, config):
    """Update hash in YAML file without destroying formatting"""
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    cert = config.get('certification', {})
    
    # Update hash line
    hmac_updated = False
    hash_index = None
    for i, line in enumerate(lines):
        if re.match(r'^\s+hash:\s+', line):
            lines[i] = f'  hash: "{cert["hash"]}"\n'
            hash_index = i
        elif re.match(r'^\s+date:\s+', line):
            lines[i] = f'  date: "{cert["date"]}"\n'
        elif re.match(r'^\s+certificate_id:\s+', line):
            lines[i] = f'  certificate_id: "{cert["certificate_id"]}"\n'
        elif re.match(r'^\s+valid_until:\s+', line):
            lines[i] = f'  valid_until: "{cert["valid_until"]}"\n'
        elif re.match(r'^\s+hmac:\s+', line):
            lines[i] = f'  hmac: "{cert["hmac"]}"\n'
            hmac_updated = True

    if 'hmac' in cert and not hmac_updated:
        insert_at = hash_index + 1 if hash_index is not None else 0
        lines.insert(insert_at, f'  hmac: "{cert["hmac"]}"\n')
    
    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(lines)

def update_certification_hash(config_path, dry_run=False):
    """
    Update certification hash in config file
    
    Args:
        config_path: Path to certified_safety_params.yaml
        dry_run: If True, only print new hash without modifying file
    """
    # Load config
    config = load_config(config_path)
    
    if 'safety_limits' not in config:
        print("❌ ERROR: No 'safety_limits' section found!")
        return False
    
    canonical_str = build_canonical(config['safety_limits'])
    new_hash = compute_params_hash(canonical_str)
    secret = get_secret_key(config_path)
    new_hmac = compute_params_hmac(canonical_str, secret) if secret else None
    
    print("\n" + "="*70)
    print("  CERTIFICATION HASH GENERATOR")
    print("="*70)
    print(f"\nConfig file: {config_path}")
    print(f"Parameters:  {len(config['safety_limits'])} certified")
    print(f"\nNew SHA-256 hash:\n  {new_hash}")
    
    if 'certification' in config:
        old_hash = config['certification'].get('hash', 'N/A')
        print(f"\nOld hash:\n  {old_hash}")
        
        old_hmac = config['certification'].get('hmac') if secret else None
        if old_hash == new_hash and (new_hmac is None or old_hmac == new_hmac):
            print("\n✅ Hash matches - parameters unchanged")
            if new_hmac is not None and old_hmac != new_hmac:
                print(f"   Updating HMAC using secret key file")
            else:
                return True
        else:
            print("\n⚠️  Hash changed - parameters modified!")
    
    if dry_run:
        print("\n[DRY RUN] File not modified")
        return True
    
    # Update certification metadata
    if 'certification' not in config:
        config['certification'] = {}
    
    cert = config['certification']
    old_cert_id = cert.get('certificate_id', 'ULTRABOT-2025-SAFETY-000')
    
    # Increment certificate counter
    parts = old_cert_id.rsplit('-', 1)
    if len(parts) == 2 and parts[1].isdigit():
        counter = int(parts[1]) + 1
        new_cert_id = f"{parts[0]}-{counter:03d}"
    else:
        new_cert_id = "ULTRABOT-2025-SAFETY-001"
    
    # Update fields
    cert['hash'] = new_hash
    cert['date'] = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    cert['certificate_id'] = new_cert_id
    cert['valid_until'] = (datetime.utcnow() + timedelta(days=365)).strftime("%Y-%m-%dT%H:%M:%SZ")
    
    if 'certified_by' not in cert:
        cert['certified_by'] = "Internal Safety Team (Pre-TÜV)"
    if 'project_version' not in cert:
        cert['project_version'] = "3.0.1"
    
    # Optionally compute HMAC if secret provided
    if new_hmac:
        cert['hmac'] = new_hmac
    elif 'hmac' in cert:
        print(f"\n⚠️  WARNING: Secret key file not found - existing HMAC preserved")

    # Save updated config
    save_config(config_path, config)
    
    print("\n✅ Configuration updated:")
    print(f"   Certificate ID: {new_cert_id}")
    print(f"   Date:           {cert['date']}")
    print(f"   Valid until:    {cert['valid_until']}")
    print(f"   Hash:           {new_hash[:16]}...")
    
    print("\n⚠️  IMPORTANT:")
    print("   1. Review changes in git diff")
    print("   2. Get approval from safety officer")
    print("   3. Update audit trail in YAML file")
    print("   4. Commit with: git commit -m 'cert: update safety params [" + new_cert_id + "]'")
    print("   5. Run validation tests")
    
    return True

def print_parameter_details(config_path):
    """Print detailed information about all parameters"""
    config = load_config(config_path)
    
    if 'safety_limits' not in config:
        print("❌ ERROR: No 'safety_limits' section found!")
        return
    
    print("\n" + "="*70)
    print("  CERTIFIED PARAMETERS")
    print("="*70)
    
    for name, data in sorted(config['safety_limits'].items()):
        value = data['value']
        unit = data.get('unit', '')
        standard = data.get('standard', 'N/A')
        
        print(f"\n📌 {name}")
        print(f"   Value:    {value} {unit}")
        print(f"   Standard: {standard}")

def main():
    """Main entry point"""
    if len(sys.argv) < 2:
        # Default config path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'certified_safety_params.yaml')
    else:
        config_path = sys.argv[1]
    
    if not os.path.exists(config_path):
        print(f"❌ ERROR: Config file not found: {config_path}")
        print(f"\nUsage: {sys.argv[0]} [config_file]")
        return 1
    
    # Check for flags
    dry_run = '--dry-run' in sys.argv
    show_details = '--details' in sys.argv
    
    if show_details:
        print_parameter_details(config_path)
        return 0
    
    # Update hash
    success = update_certification_hash(config_path, dry_run=dry_run)
    
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main())
