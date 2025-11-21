# Main Controller - Bot Operation Orchestrator

**File**: `main.py`  
**Location**: Bot Side  
**Purpose**: Central workflow controller for autonomous bot operation

## 🎯 Overview

Primary control module that orchestrates the complete autonomous operation cycle. Manages the scan→send→receive→move workflow and handles all system states from initialization to continuous operation.

## 🔧 Key Responsibilities

- **Workflow Management**: Coordinates all bot modules in proper sequence
- **State Machine**: Implements operational states and transitions
- **Error Handling**: Comprehensive failure recovery and safety systems
- **Central Computer Coordination**: Manages communication lifecycle

## 🚀 Core Features

### Module Integration
```python
# Centralized module management
self.area_scanner = AreaScanner(...)        # Environment scanning
self.movement_controller = MovementController(...)  # Locomotion
self.communication_handler = CommunicationHandler(...)  # Networking
```

### Operational States
- **Initialization**: Hardware and module setup
- **Waiting**: Awaiting central computer start signal
- **Active**: Continuous autonomous operation
- **Error**: Safe failure states with visual indicators

## 🔄 Core Workflow

### Autonomous Operation Cycle
```
1. SCAN → 2. SEND → 3. RECEIVE → 4. MOVE
   ↓                              ↑
   └─────────── REPEAT ───────────┘
```

**Step Details:**
1. **Scan**: 180° environment sweep with ultrasonic sensor
2. **Send**: Transmit sensor data to central computer
3. **Receive**: Get movement instructions from path planning
4. **Move**: Execute precise locomotion commands

## ⚡ State Management

### Startup Sequence
```python
# Phased initialization with validation
1. System Readiness Check
2. Module Initialization  
3. Network Connectivity
4. Status Reporting
5. Start Signal Waiting
6. Main Operation Loop
```

### Error States & Recovery
- **Hardware Failure**: Rapid LED blinking, system halt
- **Network Loss**: Continuous retry with exponential backoff
- **Communication Timeout**: Reset and reinitialize connection
- **Module Failure**: Isolated degradation with continued operation

## 📡 Central Computer Protocol

### Startup Handshake
```
Bot Boot → System Status → Wait for Start → Begin Cycle
```

### Data Exchange
- **Outgoing**: Sensor scans as (angle, distance) pairs
- **Incoming**: Movement instructions (distance_cm, angle_degrees)
- **Acknowledgment**: Movement completion reports

## 💡 Visual Status System

### LED Communication
- **Slow Single Blink**: Waiting for start signal
- **Rapid Blink**: System error - requires intervention
- **Double Blink**: Successful operation state
- **Continuous**: Normal autonomous operation

## ⚡ Quick Start

```python
# Minimal operation setup
bot = MobileBotMain()
bot.run()  # Begins complete autonomous lifecycle

# The system automatically:
# - Initializes all hardware
# - Connects to central computer  
# - Waits for start authorization
# - Begins continuous scan-move cycles
```

## 🛡️ Safety Systems

### Movement Validation
- **Parameter Checking**: Valid distance and angle ranges
- **Execution Monitoring**: Movement completion verification
- **Failure Recovery**: Retry logic with attempt limiting

### Communication Reliability
- **Timeout Handling**: Prevents infinite waiting states
- **Data Integrity**: Checksum verification on all transmissions
- **Reconnection Logic**: Automatic recovery from network issues

## 📊 Performance Features

- **Cycle Monitoring**: Success/failure tracking and statistics
- **Resource Management**: Efficient module lifecycle control
- **Graceful Degradation**: Continues operation with reduced capabilities
- **Clean Shutdown**: Proper resource release on termination

The brain of the mobile bot that transforms individual hardware capabilities into coordinated, intelligent autonomous behavior through precise workflow orchestration.