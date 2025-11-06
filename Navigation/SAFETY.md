# SAFETY DOCUMENTATION - Ultrabot AGV Navigation System

**Version:** 1.0.0  
**Date:** 2025-10-28  
**Classification:** Safety-Critical System  
**Compliance:** ISO 13849-1/-2, ISO 3691-4, IEC 61508, EN 61800-5-2

---

## 📋 Table of Contents

1. [Safety Overview](#safety-overview)
2. [Applicable Standards](#applicable-standards)
3. [Safety Architecture](#safety-architecture)
4. [Safety Functions](#safety-functions)
5. [Risk Assessment](#risk-assessment)
6. [Operating Procedures](#operating-procedures)
7. [Emergency Procedures](#emergency-procedures)
8. [Maintenance and Verification](#maintenance-and-verification)
9. [Audit Trail and Logging](#audit-trail-and-logging)
10. [Cybersecurity](#cybersecurity)

---

## 🛡️ Safety Overview

The Ultrabot AGV navigation system implements multiple layers of safety controls to ensure safe operation in manual, semi-automatic, and automatic modes.

### Safety Integrity Level (SIL)
- **SIL Level:** SIL 1 (IEC 61508)
- **Performance Level:** PL d (ISO 13849-1)
- **Category:** Category 3 (ISO 13849-1)

### Safety Philosophy
The system follows a **defense-in-depth** approach with multiple independent safety layers:

1. **Hardware Layer:** Physical emergency stop (STO - Safe Torque Off)
2. **Software Layer:** Safety Supervisor Node (logical safety monitoring)
3. **Operational Layer:** Deadman button, watchdog timers, plausibility checks

---

## 📚 Applicable Standards

### Primary Safety Standards

| Standard | Title | Relevance |
|----------|-------|-----------|
| **ISO 13849-1:2023** | Safety of machinery - Safety-related parts of control systems | Core safety architecture |
| **ISO 13849-2:2012** | Validation | Safety validation procedures |
| **ISO 3691-4:2023** | Industrial trucks - Safety requirements - Driverless trucks and their systems | AGV-specific requirements |
| **IEC 61508:2010** | Functional safety of electrical/electronic/programmable electronic safety-related systems | Functional safety framework |
| **EN 61800-5-2:2016** | Adjustable speed electrical power drive systems - Safety functions | Motor drive safety |
| **ISO 13850:2015** | Emergency stop function - Principles for design | E-stop implementation |

### Supporting Standards

| Standard | Title | Relevance |
|----------|-------|-----------|
| **ISO 14971:2019** | Medical devices - Application of risk management | Risk assessment methodology |
| **IEC 62304:2006** | Medical device software - Software life cycle processes | Software development lifecycle |
| **IEC 62443** | Industrial communication networks - IT security | Cybersecurity |
| **AI Act (EU 2024)** | Artificial Intelligence Act | AI system compliance |

---

## 🏗️ Safety Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    OPERATOR INTERFACE                        │
│  (Joystick / Keyboard with Deadman Button)                  │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│               TELEOP NODES (teleop_joy / keyboard)           │
│  - Deadman enforcement                                       │
│  - Command validation                                        │
│  - Watchdog monitoring                                       │
│  - Operator logging                                          │
└──────────────────┬──────────────────────────────────────────┘
                   │ /cmd_vel
                   ▼
┌─────────────────────────────────────────────────────────────┐
│           SAFETY SUPERVISOR NODE ⚠️ SAFETY-CRITICAL          │
│  - Redundant command validation                              │
│  - Plausibility checking (cmd vs actual velocity)           │
│  - Watchdog timeout enforcement                              │
│  - Deadman state verification                                │
│  - Emergency stop logic                                      │
└──────────────────┬──────────────────────────────────────────┘
                   │ /wheel_cmd_safe
                   ▼
┌─────────────────────────────────────────────────────────────┐
│                   DRIVE NODE (main.cpp)                      │
│  - EtherCAT motor control                                    │
│  - Torque management                                         │
│  - Hardware interface                                        │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              HARDWARE EMERGENCY STOP (STO)                   │
│  - Independent safety circuit                                │
│  - Direct motor torque cutoff                                │
│  - Complies with ISO 13850 / EN 61800-5-2                   │
└─────────────────────────────────────────────────────────────┘
```

### Redundancy Levels

1. **Physical Emergency Stop:** Independent hardware circuit (STO)
2. **Safety Supervisor:** Dedicated ROS2 node for logical safety
3. **Teleop Nodes:** First-level command validation
4. **Watchdog Timers:** Multiple timeout monitors

---

## ⚙️ Safety Functions

### SF-001: Emergency Stop (E-Stop)

**Description:** Immediate halt of all motion when E-stop button is pressed.

**Implementation:**
- **Hardware:** Physical E-stop button connected to motor drive STO input
- **Response Time:** < 100 ms (ISO 13850 compliant)
- **Recovery:** Manual reset required after E-stop

**Standard:** ISO 13850:2015, EN 61800-5-2

---

### SF-002: Deadman Button (Three-Position Enabling Device)

**Description:** Operator must continuously hold deadman button/key to enable motion.

**Implementation:**
- **Joystick:** R1 button (configurable)
- **Keyboard:** Space bar
- **Logic:** Robot stops immediately when deadman is released

**Standard:** ISO 3691-4 Section 5.2.3

---

### SF-003: Velocity Limiting

**Description:** Software limits on maximum linear and angular velocities.

**Limits:**
- **Linear:** 1.0 m/s (default, configurable)
- **Angular:** 1.0 rad/s (default, configurable)

**Enforcement:**
- Primary: Teleop nodes
- Secondary: Safety Supervisor (redundant)

**Standard:** ISO 3691-4 Section 5.3

---

### SF-004: Watchdog Timer

**Description:** Automatic stop if no commands received within timeout period.

**Parameters:**
- **Timeout:** 0.5 seconds (default)
- **Check Frequency:** 2x timeout frequency
- **Action:** Emergency stop + log event

**Standard:** IEC 61508 (systematic capability)

---

### SF-005: Plausibility Check

**Description:** Continuous comparison of commanded vs actual velocity.

**Implementation:**
- **Threshold:** 0.2 m/s difference (linear)
- **Angular Threshold:** 0.4 rad/s (angular)
- **Action:** Emergency stop if threshold exceeded

**Standard:** ISO 3691-4 Section 5.6.3 (velocity monitoring)

---

### SF-006: Command Validation

**Description:** All commands validated before execution.

**Checks:**
- Velocity within limits
- No non-planar motion (only 2D)
- Deadman state verified
- Message format validation

**Standard:** IEC 61508 (input validation)

---

## 🎯 Risk Assessment

### Risk Matrix (ISO 14971)

| Hazard | Severity | Probability | Risk Level | Mitigation |
|--------|----------|-------------|------------|------------|
| Unintended motion | High | Medium | **High** | SF-002, SF-004, SF-006 |
| Excessive speed | High | Low | **Medium** | SF-003, SF-005 |
| Loss of control | High | Low | **Medium** | SF-001, SF-002, SF-004 |
| Communication failure | Medium | Low | **Low** | SF-004, Diagnostics |
| Software crash | Medium | Low | **Low** | Watchdog, Exception handling |
| Cybersecurity breach | High | Very Low | **Medium** | Access control, SROS2 |

### Residual Risk

After implementation of all safety functions, residual risk is **ACCEPTABLE** per ISO 14971.

---

## 🎮 Operating Procedures

### Manual Mode Operation (ISO 3691-4 Compliant)

#### Pre-Operation Checklist

- [ ] Visual inspection of AGV (wheels, sensors, emergency stop)
- [ ] Verify workspace is clear of obstacles
- [ ] Check battery level (if applicable)
- [ ] Verify emergency stop button is not engaged
- [ ] Test emergency stop button functionality
- [ ] Verify communication with AGV (ROS topics active)

#### Starting Teleoperation

1. **Launch Safety Supervisor:**
   ```bash
   ros2 run navigation safety_supervisor_node --ros-args \
     --params-file config/safety_params.yaml
   ```

2. **Launch Teleoperation:**
   - **Joystick:**
     ```bash
     ros2 run navigation teleop_joy --ros-args \
       --params-file config/safety_params.yaml
     ```
   - **Keyboard:**
     ```bash
     ros2 run navigation teleop_keyboard_safe.py --ros-args \
       --params-file config/safety_params.yaml
     ```

3. **Verify Safety Status:**
   ```bash
   ros2 topic echo /diagnostics
   ```
   - Check that safety_supervisor status is `OK`

4. **Begin Operation:**
   - **Hold deadman button/key continuously**
   - Start with slow, controlled movements
   - Verify robot responds correctly
   - Monitor diagnostics output

#### During Operation

- **ALWAYS** maintain awareness of surroundings
- **NEVER** operate AGV near personnel without proper safeguards
- Monitor diagnostics for warnings/errors
- Be prepared to release deadman or press E-stop immediately
- Maintain line-of-sight with AGV (manual mode requirement)

#### Stopping Operation

1. Release deadman button (robot stops automatically)
2. Verify robot has stopped completely
3. If needed, press emergency stop button
4. Shut down teleop nodes: `Ctrl+C`
5. Shut down safety supervisor
6. Document any incidents in operator log

---

## 🚨 Emergency Procedures

### Emergency Stop Activation

**When to Use:**
- Imminent collision
- Unexpected behavior
- Person enters danger zone
- Equipment malfunction
- Any unsafe condition

**Procedure:**
1. **Press physical E-stop button immediately**
2. Verify robot has stopped
3. Assess situation
4. Do NOT reset E-stop until safe
5. Investigate cause before resuming
6. Document incident

### Emergency Stop Reset

**Prerequisites:**
- Hazard has been removed
- Workspace is clear
- Cause has been identified
- Supervisor approval (if required)

**Procedure:**
1. Verify workspace is safe
2. Rotate E-stop button to reset (clockwise)
3. Check diagnostics: `ros2 topic echo /safety_stop`
4. Verify `safety_stop: false`
5. Resume operation with caution

### Plausibility Check Failure

**Indication:**
- Safety supervisor logs: "Plausibility check failed"
- Robot stops immediately
- `/safety_stop` topic shows `true`

**Procedure:**
1. Do NOT immediately restart
2. Check for:
   - Wheel slippage
   - Mechanical obstruction
   - Encoder failure
   - Motor drive issue
3. Run odometry validation: `ros2 run navigation validate_odometry.py`
4. If validation fails, recalibrate before resuming
5. Document incident

### Watchdog Timeout

**Indication:**
- Logs show "Watchdog timeout"
- Robot stops
- Loss of communication

**Procedure:**
1. Check network connectivity
2. Verify ROS nodes are running: `ros2 node list`
3. Check for high CPU/network load
4. Restart affected nodes if necessary
5. Test communication before resuming

---

## 🔧 Maintenance and Verification

### Daily Checks

- [ ] Emergency stop button test
- [ ] Deadman button test
- [ ] Visual inspection
- [ ] Diagnostic status check

### Weekly Checks

- [ ] Odometry accuracy verification
- [ ] Velocity limit validation
- [ ] Watchdog timer test
- [ ] Log file review

### Monthly Checks

- [ ] Full odometry calibration
- [ ] Safety function validation
- [ ] Firmware/software update check
- [ ] Documentation review

### Annual Checks (Mandatory)

- [ ] **Complete safety audit (ISO 13849-2)**
- [ ] **Risk assessment review (ISO 14971)**
- [ ] **Odometry recalibration with certification**
- [ ] **Configuration version update**
- [ ] **Professional safety inspection**

### Validation Testing

Run validation suite quarterly:

```bash
# Odometry validation
ros2 run navigation validate_odometry.py

# Safety function validation
ros2 run navigation validate_safety_functions.py
```

Results must be exported to CSV and archived with configuration hash.

---

## 📊 Audit Trail and Logging

### Operator Actions

All operator actions are logged to `/operator_log` topic:

**Format:** `TIMESTAMP | OPERATOR_ID | ACTION | LINEAR | ANGULAR`

**Stored Location:** `/var/log/ultrabot/operator_log_YYYYMMDD.csv`

### Safety Events

All safety-critical events logged to `/diagnostics`:

- Emergency stops
- Watchdog timeouts
- Plausibility failures
- Deadman state changes
- Velocity limit violations

### Log Retention

- **Operator logs:** 1 year
- **Safety events:** 5 years (regulatory requirement)
- **Calibration reports:** Indefinite (traceable records)

### Audit Requirements

Per ISO 14971 and IEC 62304:
- Logs must be tamper-proof
- Timestamps must be synchronized
- Configuration hash must match logged events
- Operator ID must be traceable

---

## 🔐 Cybersecurity

### Threat Model (IEC 62443, AI Act)

| Threat | Risk | Mitigation |
|--------|------|------------|
| Unauthorized command injection | High | ROS 2 Security (SROS2), topic authentication |
| Man-in-the-middle attack | Medium | DDS-Security encryption |
| Denial of service | Medium | QoS configuration, rate limiting |
| Configuration tampering | High | Configuration hash verification |

### Security Controls

1. **ROS 2 Security (SROS2):**
   - Enable for production: `enable_sros2: true`
   - Generate security keys for all nodes
   - Enforce topic authentication

2. **Network Isolation:**
   - Dedicated network for AGV communication
   - Firewall rules limiting access
   - No internet connectivity for safety-critical nodes

3. **Access Control:**
   - Operator ID authentication required
   - Role-based access control (RBAC)
   - Session timeout after inactivity

4. **Configuration Integrity:**
   - SHA-256 hash of configuration files
   - Version control for all parameters
   - Read-only configuration in production

5. **Audit Logging:**
   - All security events logged
   - Tamper-evident log storage
   - Regular security audits

### Security Best Practices

- Change default passwords
- Keep software updated
- Regular security audits
- Incident response plan
- Security training for operators

---

## 📝 Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-10-28 | Safety Engineer | Initial safety documentation |

---

## ✅ Approvals

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Safety Engineer | _____________ | _____________ | ________ |
| Technical Lead | _____________ | _____________ | ________ |
| Quality Assurance | _____________ | _____________ | ________ |

---

## 📞 Emergency Contacts

- **Safety Officer:** [Name] - [Phone]
- **Technical Support:** [Name] - [Phone]
- **Emergency Services:** 112 (EU) / 911 (US)

---

**Document Classification:** Safety-Critical  
**Next Review Date:** 2026-10-28  
**Distribution:** Controlled - Safety Personnel Only
