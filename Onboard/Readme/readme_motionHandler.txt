# Movement Controller - Precise Locomotion

**File**: `movement_controller.py`  
**Location**: Bot Side  
**Purpose**: Stepper motor control for accurate bot movement and rotation

## 🎯 Overview

Motor control module that translates high-level navigation commands into precise stepper motor movements. Handles both linear travel and rotational maneuvers with configurable parameters.

## 🔧 Key Responsibilities

- **Step Generation**: Precise pulse control for stepper motors
- **Movement Calculation**: Converts distances to motor steps
- **Rotation Handling**: Implements point turns and angled movement
- **Speed Control**: Adjustable movement velocity

## 🚀 Core Features

### Step Control System
```python
def move_forward(self, distance_cm):
    # Direction setting based on distance sign
    self.dir_pin.value(1 if distance_cm > 0 else 0)
    
    # Step generation with precise timing
    for _ in range(steps):
        self.step_pin.value(1)
        time.sleep_ms(1)
        self.step_pin.value(0)
        time.sleep_ms(self.step_delay_ms)
```

### Movement Calculations
```python
def calculate_steps_for_distance(self, distance_cm):
    # Converts linear distance to motor steps
    steps_per_cm = self.steps_per_revolution / self.wheel_circumference
    return int(abs(distance_cm) * steps_per_cm)
```

## 📊 Movement Specifications

### Physical Parameters
- **Steps per Revolution**: 200 (standard stepper)
- **Wheel Diameter**: 6.5cm (configurable)
- **Wheel Base**: 15.0cm (turning radius)
- **Step Delay**: 2ms (speed control)

### Performance Characteristics
- **Linear Resolution**: ~0.1mm per step
- **Rotation Precision**: ~1.8° per step
- **Speed Range**: Configurable via step delay
- **Movement Modes**: Forward, backward, point turns, angled moves

## 🔄 Movement Types

### Linear Movement
```
move_forward(50)   # Move 50cm forward
move_forward(-30)  # Move 30cm backward
```

### Rotational Movement
```
rotate(90)    # 90° clockwise turn
rotate(-45)   # 45° counterclockwise turn
```

### Combined Movement
```
move_with_angle(30, 45)  # Move 30cm at 45° angle
```

## ⚡ Quick Start

```python
# Initialize movement controller
mover = MovementController(
    step_pin=12,        # Stepper STEP pin
    dir_pin=14,         # Stepper DIRECTION pin
    steps_per_revolution=200,
    wheel_diameter_cm=6.5,
    wheel_base_cm=15.0
)

# Execute movements
mover.move_forward(50)     # Travel 50cm forward
mover.rotate(90)           # Turn 90° right
mover.move_with_angle(30, 45)  # Move 30cm at 45° angle
```

## 🧮 Mathematical Foundation

### Distance Calculation
```
steps = (distance_cm × steps_per_revolution) / (π × wheel_diameter_cm)
```

### Rotation Calculation
```
wheel_travel = (rotation_angle_rad × wheel_base_cm) / 2
```

## 🛡️ Safety Features

### Movement Validation
- **Parameter Checking**: Valid distance and angle ranges
- **Step Counting**: Precise movement execution
- **Direction Control**: Clear forward/backward signaling

### Hardware Protection
- **Initial State**: Motors start in stopped position
- **Signal Timing**: Proper pulse widths for driver compatibility
- **Current Management**: Prevents motor stalling through speed control

## ⚙️ Configuration Options

### Performance Tuning
```python
# Adjust for different hardware
self.step_delay_ms = 2    # Lower = faster, Higher = slower
self.steps_per_revolution = 200  # Motor-specific
self.wheel_diameter_cm = 6.5    # Physical measurement
```

### Advanced Features
```python
# Precise speed-controlled movement
mover.precise_move(30, 45, speed_factor=0.5)  # Half speed
```

## 🔧 Hardware Integration

### Stepper Driver Requirements
- **STEP Input**: Rising edge triggered
- **DIRECTION Input**: High/Low for forward/backward
- **Voltage Levels**: 3.3V compatible (ESP32 GPIO)

### Motor Specifications
- **4-phase Stepper**: Standard 28BYJ-48 or similar
- **Driver Board**: ULN2003 or compatible
- **Power Requirements**: Separate 5V motor supply recommended

## 📈 Use Cases

- **Precision Navigation**: Exact position control for mapping
- **Obstacle Avoidance**: Controlled maneuvering in tight spaces
- **Path Following**: Accurate execution of planned routes
- **Exploration**: Systematic environment coverage

The locomotion engine that transforms digital navigation commands into precise physical movement with millimeter-level accuracy and predictable behavior.