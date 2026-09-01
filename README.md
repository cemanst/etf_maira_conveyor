# ETF Maira Conveyor

This repository contains a simple ROS 2 + OPC UA test client for controlling the conveyor through an ATV320 drive.

The current driver is intentionally minimal:
- it connects to the PLC using OPC UA with security mode `none`
- it accepts a conveyor target speed in meters per second
- it converts the requested speed to the PLC frequency value using the known ATV mapping
- it sends the standard ATV command sequence: `7 -> 15` to start, and `6` to stop

## Speed conversion

The relationship is:

```
speed_mps = 0.005 * real_frequency_hz
```

The PLC frequency value is scaled by 10 because the PLC uses 0.1 Hz units.

As a result, the driver converts the ROS parameter in m/s to the PLC value with:

```
plc_frequency = round((speed_mps / 0.005) * 10)
```

Example:
- requested speed = 0.5 m/s
- real frequency = 100 Hz
- PLC value written = 1000

## Quickstart

### 1. Build the workspace

```bash
cd /home/etf_robot/etf_maira_conveyor
source /opt/ros/lyrical/setup.bash
colcon build --packages-select conveyor_driver
source install/setup.bash
```

### 2. Start the conveyor node

```bash
python3 /home/etf_robot/etf_maira_conveyor/scripts_and_tests/altivar_test.py
```

Make sure the node prints:

```text
OPC UA client connected successfully (security none).
```

### 3. Set the target speed in m/s

```bash
ros2 param set /conveyor_driver conveyor_speed_mps 0.5
```

### 4. Start the conveyor

```bash
ros2 service call /conveyor_driver/start std_srvs/srv/Trigger
```

### 5. Stop the conveyor

```bash
ros2 service call /conveyor_driver/stop std_srvs/srv/Trigger
```

## Notes

- The driver is designed for a no-security OPC UA PLC connection.
- The main path is the ATV command word sequence:
  - start: `7` then `15`
  - stop: `6`
- The frequency is configured in m/s via ROS parameter, not directly in Hz.

## Basic troubleshooting

If the PLC rejects writes or the session saturates:

1. make sure no stale Python OPC UA client is still running
2. confirm the PLC is configured for no-security OPC UA
3. confirm the correct variables exist in the namespace
4. verify the ATV is in the proper remote/communication mode
