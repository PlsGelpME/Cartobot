# Communication Handler - Network Bridge

**File**: `communication_handler.py`  
**Location**: Bot Side  
**Purpose**: Manages all wireless communication with central computer

## 🎯 Overview

Reliable TCP communication module that handles data transmission to the central computer and instruction reception for autonomous operation. Implements robust protocol with error handling and verification.

## 🔧 Key Responsibilities

- **Data Serialization**: Converts sensor data to transmission format
- **TCP Communication**: Manages socket connections to central server
- **Protocol Implementation**: Ensures reliable message delivery
- **Instruction Processing**: Validates and parses movement commands

## 🚀 Core Features

### Connection Management
```python
# Automatic WiFi setup and maintenance
self.setup_wifi(ssid, password)  # Handles connection & authentication
self.server_ip = server_ip       # Central computer address
self.server_port = server_port   # Communication endpoint
```

### Data Protocol
**Transmission Format:**
```
[4-byte length][JSON data] → Send → Wait for [4-byte checksum][ACK]
```

**Message Structure:**
```python
{
    'timestamp': time.time(),
    'sensor_type': 'ultrasonic_scanner', 
    'data_points': [(angle, distance), ...],
    'data_count': len(points)
}
```

## 📨 Communication Flow

### Data Transmission
```
1. Serialize sensor data to JSON
2. Calculate data length prefix
3. Send length + data over TCP
4. Wait for checksum + ACK response
5. Verify data integrity
```

### Instruction Reception  
```
1. Send instruction request signal
2. Receive instruction length prefix
3. Download complete instruction data
4. Parse and validate JSON structure
5. Send acknowledgment
```

## 🔒 Validation Systems

### Data Integrity
- **Checksum Verification**: 32-bit validation of transmitted data
- **Length Prefixing**: Prevents incomplete message reception
- **ACK Confirmation**: Ensures central computer received data

### Instruction Safety
```python
def validate_instruction(self, instruction_data):
    # Ensures movement commands are safe and valid
    required_fields = ['distance_cm', 'angle_degrees']
    distance_limits = (-1000, 1000)    # cm safety range
    angle_limits = (-360, 360)         # degree rotation limits
```

## ⚡ Quick Start

```python
# Initialize communication
comm = CommunicationHandler(
    ssid='network_ssid',
    password='network_password', 
    server_ip='192.168.1.100',
    server_port=5000
)

# Send sensor data
result = comm.send_data(scan_data)
if result['status'] == 'success':
    # Get next movement instruction
    instruction = comm.get_instruction()
```

## 🛡️ Error Handling

### Connection Issues
- **WiFi Failures**: Automatic reconnection attempts
- **Server Timeouts**: 10-second send / 15-second receive timeouts
- **Socket Errors**: Graceful cleanup and retry logic

### Data Problems
- **JSON Errors**: Malformed data detection and rejection
- **Protocol Violations**: Invalid message structure handling
- **Verification Failures**: Checksum and ACK validation

## 📊 Performance Features

- **Efficient Serialization**: Minimal overhead data packaging
- **Streaming Support**: Chunked data transfer for large scans
- **Connection Pooling**: Reusable socket management
- **Bandwidth Optimization**: Compact JSON structure

## 🔄 Integration Points

- **Area Scanner**: Receives raw (angle, distance) data for transmission
- **Movement Controller**: Executes validated movement instructions
- **Main Controller**: Provides high-level communication coordination

The reliable network bridge that ensures seamless, verified data exchange between the mobile bot's sensors and the central computer's intelligence systems.