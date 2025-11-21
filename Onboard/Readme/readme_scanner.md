# Area Scanner - Environmental Perception

**File**: `area_scanner.py`  
**Location**: Bot Side  
**Purpose**: 180-degree environmental scanning using servo-mounted ultrasonic sensor

## 🎯 Overview

Hardware control module that performs systematic environment scanning by combining servo positioning with ultrasonic distance measurement. Creates detailed 180-degree profiles of surrounding obstacles.

## 🔧 Key Responsibilities

- **Servo Control**: Precise angular positioning for scanning
- **Distance Measurement**: Ultrasonic time-of-flight calculations
- **Scan Coordination**: Systematic 10-degree increment sweeps
- **Data Collection**: Structured environment profiling

## 🚀 Core Features

### Servo Positioning System
```python
def set_servo_angle(self, angle):
    # Converts 0-180° to 2500-7500μs PWM signal
    pulse_width = 2500 + (angle / 180) * 5000  # Standard servo range
    self.servo.duty_ns(pulse_width * 1000)     # Nanosecond precision
    time.sleep_ms(300)  # Stabilization delay
```

### Ultrasonic Distance Measurement
```python
def get_distance(self):
    # HC-SR04 time-of-flight calculation
    trigger_pulse(10μs) → measure_echo_duration → calculate_distance
    distance_cm = (duration * 0.0343) / 2  # Speed of sound adjustment
```

## 📊 Scanning Specifications

### Coverage Pattern
- **Angular Range**: 0° to 180° (full front hemisphere)
- **Resolution**: 10° increments (19 measurement points)
- **Scan Time**: ~6 seconds total (300ms per position + measurement)

### Sensor Performance
- **Measurement Range**: 2cm to 400cm (valid readings)
- **Accuracy**: ~1cm resolution under ideal conditions
- **Error Handling**: Timeout detection and invalid reading filtering

## 🔄 Scan Workflow

```
1. Position servo at 0°
2. Trigger ultrasonic measurement
3. Record (angle, distance) pair
4. Move to next 10° position
5. Repeat until 180° coverage
6. Return servo to 90° center
```

## 📋 Data Output

```python
# Typical scan results format
scan_data = [
    (0, 45),   # 0° angle, 45cm distance
    (10, 50),  # 10° angle, 50cm distance  
    (20, 55),  # ... continues to 180°
    ...
    (180, 45)  # Final position
]
```

## ⚡ Quick Start

```python
# Initialize scanner with hardware pins
scanner = AreaScanner(
    servo_pin=13,        # Servo control pin
    trigger_pin=5,       # Ultrasonic trigger
    echo_pin=18          # Ultrasonic echo
)

# Perform complete environment scan
scan_results = scanner.scan_area()

# Access individual measurements
for angle, distance in scan_results:
    print(f"Obstacle at {angle}°: {distance}cm away")
```

## 🛡️ Safety & Reliability

### Error Prevention
- **Angle Limits**: Hardware-safe 0-180° range enforcement
- **Measurement Validation**: Filtering of out-of-range readings
- **Timeout Protection**: Prevents infinite echo waiting
- **Servo Stabilization**: Adequate movement settling time

### Performance Optimization
- **Efficient Sequencing**: Overlapped movement and measurement
- **Hardware Protection**: Proper signal timing and voltage levels
- **Resource Management**: Clean PWM and GPIO control

## 🔧 Hardware Integration

### Servo Requirements
- **PWM Frequency**: 50Hz standard servo protocol
- **Voltage**: 5V operation with adequate current
- **Mechanical Range**: 180-degree rotational capability

### Ultrasonic Sensor
- **HC-SR04 Compatibility**: Standard 5V trigger/echo
- **Measurement Timing**: Precise microsecond-level control
- **Environmental Factors**: Compensates for temperature/sound speed

## 📈 Use Cases

- **Environment Mapping**: Create 180° obstacle profiles
- **Navigation Planning**: Identify clear paths and obstacles
- **Object Detection**: Locate and range environmental features
- **Space Analysis**: Measure room dimensions and layouts

The environmental perception system that transforms raw sensor hardware into structured, actionable spatial data for autonomous navigation decisions.