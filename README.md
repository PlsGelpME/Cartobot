# Autonomous Mobile Bot Navigation System

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue.svg)](https://python.org)
[![MicroPython](https://img.shields.io/badge/MicroPython-1.19%2B-orange.svg)](https://micropython.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32%2BWindows-lightgrey.svg)]()

A comprehensive autonomous navigation system featuring a mobile bot that explores environments using SLAM (Simultaneous Localization and Mapping) and RRT* path planning. The system ensures safe navigation by preventing movement into unknown areas.

## 🚀 System Overview

This project implements a complete autonomous navigation stack with two main components:

- **Mobile Bot (ESP32)**: Hardware platform that performs environment scanning and movement execution
- **Central Computer (Windows)**: Server system that processes sensor data, runs SLAM algorithms, and plans safe paths

### Key Features

- **Real-time Environment Mapping**: Uses ultrasonic sensors to build 2D maps of surroundings
- **Safe Navigation**: Implements RRT* path planning with strict unknown-area avoidance
- **Autonomous Exploration**: Frontier-based exploration for efficient environment coverage
- **Live Visualization**: Real-time topology mapping and bot position tracking
- **Multi-module Architecture**: Clean separation of concerns with well-defined interfaces

## 🏗️ System Architecture

```
┌─────────────────┐    WiFi/TCP-IP    ┌──────────────────┐
│   Mobile Bot    │◄────────────────►│ Central Computer │
│  (ESP32/MCU)    │                   │    (Windows)     │
└─────────────────┘                   └──────────────────┘
         │                                      │
┌────────┴────────┐                   ┌─────────┴─────────┐
│ Hardware Modules│                   │  Software Modules │
│ • Ultrasonic    │                   │ • SLAM Navigator  │
│ • Servo Motor   │                   │ • RRT* Planner    │
│ • Stepper Motor │                   │ • Topology Viz    │
│ • WiFi          │                   │ • Server Comms    │
└─────────────────┘                   └───────────────────┘
```

## 📁 Project Structure

### Mobile Bot (ESP32) Modules

```
bot_side/
├── boot.py              # Hardware initialization & system boot
├── main.py              # Main controller & workflow orchestration
├── area_scanner.py      # Ultrasonic sensor scanning module
├── movement_controller.py # Stepper motor control for locomotion
└── communication_handler.py # WiFi communication with central computer
```

### Central Computer (Windows) Modules

```
server_side/
├── main_system.py       # Main autonomous navigation system
├── server_comms.py      # Bot communication handler
├── slam_navigator.py    # SLAM & RRT* path planning
└── topology_visualizer.py # Environment visualization
```

## 🛠️ Hardware Requirements

### Mobile Bot Components
- **Microcontroller**: ESP32 development board
- **Sensors**: HC-SR04 Ultrasonic distance sensor
- **Actuators**: 
  - SG90 Servo motor (for sensor scanning)
  - 28BYJ-48 Stepper motor with ULN2003 driver (for locomotion)
- **Power**: 5V power supply for motors, 3.3V for ESP32
- **Connectivity**: WiFi for communication

### Central Computer
- **OS**: Windows 10/11
- **Python**: 3.7 or higher
- **Dependencies**: NumPy, Matplotlib, SciPy

## ⚙️ Installation & Setup

### 1. Mobile Bot Setup

```bash
# Upload MicroPython files to ESP32
ampy --port COM3 put boot.py
ampy --port COM3 put main.py
ampy --port COM3 put area_scanner.py
ampy --port COM3 put movement_controller.py
ampy --port COM3 put communication_handler.py
```

**Hardware Connections:**
```python
# GPIO Pin Configuration (update in boot.py)
LED_PIN = 2           # Status LED
SERVO_PIN = 13        # Servo motor control
STEPPER_STEP_PIN = 12 # Stepper motor step
STEPPER_DIR_PIN = 14  # Stepper motor direction
ULTRASONIC_TRIGGER_PIN = 5   # Sensor trigger
ULTRASONIC_ECHO_PIN = 18     # Sensor echo
```

**WiFi Configuration:**
Update credentials in `boot.py` and `communication_handler.py`:
```python
ssid = 'YOUR_WIFI_SSID'
password = 'YOUR_WIFI_PASSWORD'
server_ip = '192.168.1.100'  # Central computer IP
```

### 2. Central Computer Setup

```bash
# Clone the repository
git clone https://github.com/yourusername/autonomous-mobile-bot.git
cd autonomous-mobile-bot

# Install Python dependencies
pip install numpy matplotlib scipy

# Start the central server
python server_side/main_system.py
```

## 🎯 Usage

### Starting the System

1. **Power the Mobile Bot**: The bot will automatically:
   - Initialize hardware components
   - Connect to WiFi
   - Send system status to central computer
   - Wait for start signal

2. **Start Central Server**:
   ```bash
   python server_side/main_system.py
   ```

3. **Begin Autonomous Operation**:
   - Use the command interface to monitor system status
   - The system will automatically begin scanning and navigation

### Command Interface

The central server provides an interactive command interface:

```bash
ANS> status    # Show system status
ANS> bots      # List connected bots
ANS> save_map  # Save current map as image
ANS> help      # Show available commands
ANS> stop      # Gracefully shutdown system
```

## 🔧 Configuration

### Bot Configuration (boot.py)
```python
GLOBALS['config'] = {
    'movement': {
        'steps_per_revolution': 200,
        'wheel_diameter_cm': 6.5,
        'wheel_base_cm': 15.0,
        'step_delay_ms': 2
    },
    'scanning': {
        'scan_angle_min': 0,
        'scan_angle_max': 180,
        'scan_step_angle': 10,
        'servo_speed_delay': 0.3
    }
}
```

### Navigation Parameters (main_system.py)
```python
self.exploration_goal_distance = 1.0  # meters
self.safety_margin = 0.3              # meters
```

## 🧠 Algorithm Details

### Hector SLAM Implementation
- **Occupancy Grid Mapping**: Probabilistic grid-based environment representation
- **Scan Matching**: Simplified pose estimation using sensor data
- **Ray Casting**: Bresenham's algorithm for efficient free-space marking

### RRT* Path Planning
- **Safety First**: Strict avoidance of unknown areas
- **Optimal Paths**: Asymptotically optimal through tree rewiring
- **Real-time Planning**: Efficient sampling-based approach

### Frontier-Based Exploration
- **Autonomous Discovery**: Automatically identifies exploration targets
- **Known Area Constraint**: Only moves within mapped regions
- **Efficient Coverage**: Maximizes new area discovery per movement

## 📊 Output & Visualization

The system provides multiple forms of feedback:

1. **Real-time Topology Maps**: Live updating environment visualization
2. **System Logs**: Comprehensive logging for debugging and monitoring
3. **Statistics**: Mapping coverage, path efficiency, and system performance
4. **Saved Maps**: High-resolution images of explored environments

## 🐛 Troubleshooting

### Common Issues

1. **WiFi Connection Failed**
   - Verify SSID and password in both `boot.py` and `communication_handler.py`
   - Check central computer firewall settings

2. **Sensor Reading Errors**
   - Verify ultrasonic sensor wiring (Trigger, Echo, VCC, GND)
   - Check for obstructions in sensor path

3. **Motor Control Issues**
   - Verify stepper motor driver connections
   - Check power supply adequacy for motors

4. **Communication Timeouts**
   - Ensure both devices are on same network
   - Verify central server IP address configuration

### Debugging Tips

- Enable detailed logging in both bot and server
- Use the status LED patterns for hardware diagnostics
- Check serial monitor for ESP32 debug output
- Use the command interface for real-time system monitoring

## 📈 Performance Metrics

- **Mapping Accuracy**: Sub-5cm resolution occupancy grids
- **Path Planning**: 1000 iterations for optimal path finding
- **Communication**: Reliable TCP with checksum verification
- **Scan Speed**: 18-point 180° scan in ~6 seconds

## 🔮 Future Enhancements

- [ ] Multi-bot coordination and swarm intelligence
- [ ] Advanced SLAM algorithms (ORB-SLAM, Cartographer)
- [ ] 3D environment mapping with additional sensors
- [ ] Web-based dashboard for remote monitoring
- [ ] Machine learning for improved path planning

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 🙏 Acknowledgments

- Hector SLAM algorithm by Stefan Kohlbrecher et al.
- RRT* path planning algorithm by Sertac Karaman and Emilio Frazzoli
- MicroPython community for embedded systems support
- Matplotlib and NumPy communities for visualization and computation tools

---

**Note**: This project is for educational and research purposes. Always ensure safe operation when deploying autonomous systems in real-world environments.

For questions and support, please open an issue on GitHub or contact the development team.