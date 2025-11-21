# Main System - Central Controller

**File**: `main_system.py`  
**Location**: Server Side  
**Purpose**: Central orchestration of autonomous navigation system

## 🎯 Overview

The main system controller that integrates all server-side modules and coordinates the complete autonomous navigation workflow. Acts as the brain of the operation, managing bot communications, SLAM processing, and path planning.

## 🔧 Key Responsibilities

- **System Integration**: Coordinates all server modules (SLAM, visualization, communication)
- **Bot Management**: Tracks connected bots and assigns active control
- **Autonomous Operation**: Implements the main scan→plan→move cycle
- **Command Interface**: Provides real-time system control and monitoring
- **Safety Enforcement**: Ensures bots only move within known areas

## 🚀 Core Features

### Bot Coordination
```python
# Manages multiple bot connections
self.connected_bots = {}  # Active bot tracking
self.active_bot = None    # Currently controlled bot
```

### Autonomous Workflow
1. **Receive sensor data** from bots
2. **Update SLAM maps** with new environment data
3. **Generate exploration goals** using frontier detection
4. **Plan safe paths** with RRT* algorithm
5. **Send movement instructions** back to bots

### Safety Systems
- Unknown area prevention
- Connection monitoring with timeouts
- Movement validation and limits
- Emergency stop capability

## 📊 Command Interface

```bash
ANS> status    # System health and statistics
ANS> bots      # Connected bot information  
ANS> save_map  # Export current environment map
ANS> stop      # Graceful system shutdown
```

## 🔄 Data Flow

```
Bot Sensor Data → SLAM Update → Goal Generation → Path Planning → Movement Commands
```

## ⚡ Quick Start

```python
# Initialize and start system
system = AutonomousNavigationSystem(host='0.0.0.0', port=5000)
system.start_system()  # Begins autonomous operation
```

## 🛡️ Safety Features

- **Connection Monitoring**: Automatic bot disconnection detection
- **Boundary Enforcement**: Prevents movement into unexplored areas
- **Error Handling**: Comprehensive exception management
- **Graceful Shutdown**: Clean termination of all processes

The central command center that transforms raw sensor data into intelligent autonomous navigation.