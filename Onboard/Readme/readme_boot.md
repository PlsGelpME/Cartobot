# Boot Manager - System Initialization

**File**: `boot.py`  
**Location**: Bot Side  
**Purpose**: Hardware initialization and system preparation

## 🎯 Overview

First-execution module that initializes all hardware components, establishes network connectivity, and prepares the system for autonomous operation. Runs automatically on ESP32 boot.

## 🔧 Key Responsibilities

- **Hardware Setup**: Initializes all sensors and actuators
- **Network Connection**: Establishes WiFi to central computer
- **System Validation**: Verifies all components are operational
- **Global State**: Creates shared hardware object registry

## 🚀 Core Features

### Hardware Initialization
```python
# Pin configuration - centralized hardware mapping
LED_PIN = 2              # Status indicator
SERVO_PIN = 13           # Ultrasonic scanner rotation  
STEPPER_STEP_PIN = 12    # Locomotion control
STEPPER_DIR_PIN = 14     # Movement direction
ULTRASONIC_TRIGGER_PIN = 5   # Distance measurement
ULTRASONIC_ECHO_PIN = 18     # Echo reception
```

### Component Management
- **Modular Setup**: Individual initialization functions for each component
- **Error Handling**: Graceful degradation if hardware fails
- **Status Tracking**: Comprehensive component health monitoring

### Global Registry
```python
# Centralized hardware access for all modules
GLOBALS = {
    'objects': { ... },      # Hardware object references
    'pins': { ... },         # GPIO pin mapping
    'config': { ... },       # System parameters
    'system_ready': bool     # Overall status flag
}
```

## ⚙️ Initialization Sequence

1. **LED Setup** - Visual status feedback
2. **Servo Motor** - Ultrasonic scanner positioning
3. **Stepper Motor** - Bot locomotion system
4. **Ultrasonic Sensor** - Environment scanning
5. **WiFi Connection** - Central computer communication
6. **System Validation** - Component health check

## 🔌 Hardware Interfaces

### Servo Control
- **PWM Frequency**: 50Hz standard servo protocol
- **Angle Range**: 0-180 degrees for full scanning
- **Positioning Delay**: 300ms stabilization time

### Stepper Motor
- **Step Control**: Precision movement via STEP pin
- **Direction Control**: Forward/backward via DIR pin
- **Safety Defaults**: Initialized to stop position

### Ultrasonic Sensor
- **Trigger Management**: Controlled pulse generation
- **Echo Reading**: Time-of-flight distance calculation
- **Error Filtering**: Valid range and timeout handling

## 📡 Network Configuration

```python
# Required configuration (update before deployment)
ssid = 'YOUR_WIFI_SSID'
password = 'YOUR_WIFI_PASSWORD'
server_ip = '192.168.1.100'  # Central computer IP
```

## 💡 Status Indicators

### LED Feedback Patterns
- **Single Slow Blink**: Waiting for start signal
- **Double Blink**: WiFi connected successfully
- **Triple Blink**: System fully ready
- **Rapid Blink**: Error state - check components

### System Readiness Checks
- **Hardware Validation**: All components initialized
- **Network Verification**: Stable WiFi connection
- **Configuration Load**: All parameters properly set

## ⚡ Quick Start

```python
# Access initialized hardware from any module
servo = GLOBALS['objects']['servo']
stepper = GLOBALS['objects']['stepper_step']
sensor_trigger = GLOBALS['objects']['ultrasonic_trigger']

# Check system status
if GLOBALS['system_ready']:
    # Begin autonomous operation
```

## 🛡️ Safety Features

- **Pin Protection**: Valid range checking for all GPIO operations
- **Component Isolation**: Failed hardware doesn't crash system
- **Connection Timeouts**: Prevent infinite waiting states
- **Resource Cleanup**: Proper hardware state on exit

The foundation layer that transforms raw hardware into a coordinated, network-ready autonomous system ready for main application control.