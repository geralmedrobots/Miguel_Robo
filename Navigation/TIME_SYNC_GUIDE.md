# ============================================================================
# TIME SYNCHRONIZATION GUIDE (NTP/Chrony)
# ============================================================================
# Critical for multi-sensor fusion in AGV systems (ISO 3691-4 §5.4.2)
# Ensures accurate timestamp alignment between sensors (LiDAR, IMU, cameras)
# ============================================================================

## **Why Time Synchronization is Critical:**

### **Problem Without Sync:**
- **Sensor Fusion Errors:** EKF/UKF cannot fuse data with misaligned timestamps
- **TF Extrapolation Errors:** `tf2` fails when sensor timestamps are in the future
- **SLAM Drift:** Incremental mapping algorithms (Cartographer, SLAM Toolbox) drift
- **Collision Detection Failures:** Nav2 costmap uses stale sensor data

### **Example Failure Scenario:**
```
[ERROR] [tf2]: Lookup would require extrapolation at time 1234567890.500,
but only time 1234567890.300 is in the buffer (0.200s gap)
```

**Root Cause:** RealSense camera clock is 200ms ahead of system clock → TF lookup fails → Nav2 crashes

---

## **Solution 1: NTP (Network Time Protocol)**

### **A. Install NTP:**
```bash
sudo apt update
sudo apt install ntp ntpdate
```

### **B. Configure NTP Server:**

Edit `/etc/ntp.conf`:
```conf
# Use local NTP server (e.g., router or dedicated time server)
server 192.168.1.1 prefer iburst  # Replace with your NTP server IP

# Fallback to public NTP pool
server 0.ubuntu.pool.ntp.org iburst
server 1.ubuntu.pool.ntp.org iburst

# Allow only local network to query this NTP server
restrict 192.168.1.0 mask 255.255.255.0 nomodify notrap

# Drift file for clock correction persistence
driftfile /var/lib/ntp/ntp.drift

# Logging
logfile /var/log/ntp.log
```

### **C. Start NTP Service:**
```bash
sudo systemctl enable ntp
sudo systemctl start ntp
```

### **D. Verify Synchronization:**
```bash
# Check NTP status
ntpq -p

# Expected output:
#      remote           refid      st t when poll reach   delay   offset  jitter
# ==============================================================================
# *192.168.1.1     .GPS.            1 u   12   64  377    0.234   -0.125   0.089
#                  ^-- Asterisk (*) means currently synced

# Check system clock offset
ntpdate -q 192.168.1.1
# Offset should be < 10ms for sensor fusion
```

### **E. Force Immediate Sync (if offset > 1s):**
```bash
sudo systemctl stop ntp
sudo ntpdate -B 192.168.1.1  # Force sync
sudo systemctl start ntp
```

---

## **Solution 2: Chrony (Recommended for Modern Systems)**

Chrony is **faster** and **more robust** than NTP, especially for:
- Systems with intermittent network (AGVs on WiFi)
- Virtual machines / containers
- Systems that suspend/hibernate

### **A. Install Chrony:**
```bash
sudo apt update
sudo apt install chrony
sudo systemctl stop ntp  # Disable NTP if installed (conflicts)
sudo systemctl disable ntp
```

### **B. Configure Chrony:**

Edit `/etc/chrony/chrony.conf`:
```conf
# Local NTP server (e.g., industrial PLC or router)
server 192.168.1.1 iburst prefer

# Fallback public NTP servers
pool 0.ubuntu.pool.ntp.org iburst maxsources 2
pool 1.ubuntu.pool.ntp.org iburst maxsources 2

# Allow fast clock correction on startup
makestep 1.0 3  # Step if offset > 1s (first 3 updates)

# RTC (Real-Time Clock) sync
rtcsync

# Drift file
driftfile /var/lib/chrony/chrony.drift

# Log directory
logdir /var/log/chrony

# Allow local network to query this server
allow 192.168.1.0/24
```

### **C. Start Chrony:**
```bash
sudo systemctl enable chrony
sudo systemctl start chrony
```

### **D. Verify Synchronization:**
```bash
# Check Chrony status
chronyc tracking

# Expected output:
# Reference ID    : C0A80101 (192.168.1.1)
# Stratum         : 2
# System time     : 0.000012345 seconds fast of NTP time
# Last offset     : -0.000089012 seconds
# RMS offset      : 0.000123456 seconds
#                   ^-- Should be < 0.010 (10ms)

# Check sync sources
chronyc sources

# Expected output:
# MS Name/IP address         Stratum Poll Reach LastRx Last sample               
# ===============================================================================
# ^* 192.168.1.1                   1   6   377    12   -125us[ -234us] +/-  1234us
#    ^-- Asterisk (*) means currently synced, Last sample < 1ms
```

### **E. Force Immediate Sync:**
```bash
sudo chronyc makestep
```

---

## **Recommended Setup for AGV:**

### **Hardware Architecture:**
```
┌─────────────────────────────────────────────────┐
│ AGV ONBOARD COMPUTER (Jetson Orin / x86 PC)    │
│                                                 │
│ ┌─────────────────────────────────────────┐   │
│ │ Chrony Client (sync from Factory NTP)  │   │
│ │ Offset: < 5ms                           │   │
│ └─────────────────────────────────────────┘   │
│              │                                  │
│              ▼ Time sync via Ethernet          │
│ ┌──────────────────────────────────────────┐  │
│ │ ROS2 Nodes (all use system clock)       │  │
│ │ - LiDAR drivers                          │  │
│ │ - RealSense drivers                      │  │
│ │ - EKF sensor fusion                      │  │
│ │ - Nav2 costmap/planner                   │  │
│ └──────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
                    │
                    ▼ WiFi/Ethernet
         ┌──────────────────────┐
         │ Factory NTP Server   │
         │ (Stratum 1/2)        │
         │ 192.168.1.1          │
         └──────────────────────┘
                    │
                    ▼ GPS/PTP
         ┌──────────────────────┐
         │ GPS Reference Clock  │
         │ (Stratum 0)          │
         └──────────────────────┘
```

### **1. Factory NTP Server (Dedicated Time Server):**
- **Hardware:** Raspberry Pi + GPS HAT (£50-100)
- **Software:** Chrony + gpsd
- **Accuracy:** < 1ms (GPS-disciplined)
- **Benefit:** All AGVs + factory systems sync to same reference

**Setup Raspberry Pi GPS NTP Server:**
```bash
# Install GPS daemon
sudo apt install gpsd gpsd-clients chrony

# Configure GPS (USB GPS receiver on /dev/ttyUSB0)
sudo nano /etc/default/gpsd
# Set: DEVICES="/dev/ttyUSB0"

# Configure Chrony to use GPS
sudo nano /etc/chrony/chrony.conf
# Add:
refclock SHM 0 refid GPS precision 1e-3 offset 0.0
allow 192.168.1.0/24  # Allow local network

sudo systemctl restart gpsd chrony
```

### **2. AGV Onboard (Client):**
```bash
# Configure Chrony to use factory NTP server
sudo nano /etc/chrony/chrony.conf
# Set:
server 192.168.1.1 iburst prefer minpoll 4 maxpoll 6

sudo systemctl restart chrony
```

---

## **Monitoring Time Sync in ROS2:**

### **A. Check System Clock Offset:**
```bash
# Add to AGV startup script
watch -n 5 'chronyc tracking | grep "System time"'

# If offset > 10ms → Trigger fault in safety_supervisor
```

### **B. ROS2 Diagnostic Publisher:**

Create `time_sync_monitor_node.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import subprocess

class TimeSyncMonitor(Node):
    def __init__(self):
        super().__init__('time_sync_monitor')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(5.0, self.check_time_sync)  # Every 5s
    
    def check_time_sync(self):
        try:
            # Get Chrony tracking info
            result = subprocess.run(['chronyc', 'tracking'], 
                                    capture_output=True, text=True)
            
            # Parse "System time" line (e.g., "0.000012345 seconds fast")
            for line in result.stdout.split('\n'):
                if 'System time' in line:
                    offset_str = line.split(':')[1].strip().split()[0]
                    offset_sec = float(offset_str)
                    offset_ms = abs(offset_sec) * 1000
                    
                    # Create diagnostic message
                    diag = DiagnosticArray()
                    diag.header.stamp = self.get_clock().now().to_msg()
                    
                    status = DiagnosticStatus()
                    status.name = "Time Synchronization"
                    status.hardware_id = "chrony"
                    
                    if offset_ms < 10.0:  # < 10ms → OK
                        status.level = DiagnosticStatus.OK
                        status.message = f"Time sync OK ({offset_ms:.2f} ms)"
                    elif offset_ms < 50.0:  # 10-50ms → WARN
                        status.level = DiagnosticStatus.WARN
                        status.message = f"Time sync degraded ({offset_ms:.2f} ms)"
                    else:  # > 50ms → ERROR
                        status.level = DiagnosticStatus.ERROR
                        status.message = f"Time sync FAILED ({offset_ms:.2f} ms)"
                    
                    status.values = [
                        KeyValue(key="offset_ms", value=f"{offset_ms:.3f}"),
                        KeyValue(key="threshold_warn_ms", value="10.0"),
                        KeyValue(key="threshold_error_ms", value="50.0")
                    ]
                    
                    diag.status.append(status)
                    self.pub.publish(diag)
                    return
        except Exception as e:
            self.get_logger().error(f"Failed to check time sync: {e}")

def main():
    rclpy.init()
    node = TimeSyncMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add to launch file:**
```python
time_sync_monitor_node = Node(
    package='somanet',
    executable='time_sync_monitor_node.py',
    name='time_sync_monitor',
    output='screen'
)
```

---

## **Troubleshooting:**

### **Problem: Clock offset > 100ms despite NTP/Chrony running**
**Cause:** Firewall blocking NTP port (UDP 123)
**Solution:**
```bash
sudo ufw allow 123/udp  # Allow NTP
sudo systemctl restart chrony
```

### **Problem: RealSense timestamps in the future**
**Cause:** RealSense uses internal clock (not synced to system)
**Solution:** Use `timestamp_method: system_time` in RealSense launch:
```python
'unite_imu_method': '2',  # Use system timestamp, not camera clock
```

### **Problem: Ouster LiDAR timestamps drift**
**Cause:** Ouster uses PTP (Precision Time Protocol) instead of system clock
**Solution:** Enable PTP on Linux:
```bash
sudo apt install linuxptp
sudo ptp4l -i eth0 -m  # Replace eth0 with Ouster interface
```

Or configure Ouster to use system time:
```yaml
timestamp_mode: "TIME_FROM_ROS_TIME"  # Instead of PTP
```

---

## **Testing Time Sync:**

```bash
# 1. Check all sensor timestamps are within 50ms of system time
ros2 topic echo /scan_front --field header.stamp &
ros2 topic echo /camera_front_left/depth/image_raw --field header.stamp &
ros2 topic echo /imu/data --field header.stamp &

# Compare to system time:
date +%s.%N

# 2. Check TF tree for extrapolation errors
ros2 run tf2_ros tf2_echo base_link scan_front_link
# Should NOT show "Lookup would require extrapolation" errors

# 3. Monitor diagnostic aggregator
ros2 topic echo /diagnostics_agg | grep -A5 "Time Synchronization"
```

---

## **Production Checklist:**

- [ ] NTP/Chrony installed and configured
- [ ] Clock offset < 10ms verified with `chronyc tracking`
- [ ] All sensor drivers using system clock (not internal clocks)
- [ ] Time sync monitor node running and publishing to `/diagnostics`
- [ ] Safety supervisor monitors time sync (fault if offset > 50ms)
- [ ] Factory-wide NTP server deployed (optional but recommended)
- [ ] Automatic time sync check in startup script (`scripts/check_time_sync.sh`)
- [ ] Documentation updated with NTP server IP and credentials

---

**Status:** ✅ Time synchronization configured for production AGV deployment
