# Server Communications Handler

**File**: `server_comms.py`  
**Location**: Server Side  
**Purpose**: TCP communication server for mobile bot connectivity

## 🎯 Overview

Robust network communication module that manages all connections with mobile bots. Implements reliable TCP messaging with callback system for real-time data processing.

## 🔧 Key Responsibilities

- **Multi-client TCP Server**: Handles multiple simultaneous bot connections
- **Message Protocol**: Implements length-prefixed JSON messaging
- **Callback System**: Event-driven architecture for data processing
- **Connection Management**: Automatic bot tracking and cleanup

## 🚀 Core Features

### Connection Handling
```python
# Manages bot connections with full lifecycle
self.connected_bots = {
    'bot_id': {
        'socket': client_socket,
        'address': client_address,
        'last_activity': datetime.now()
    }
}
```

### Message Protocol
- **4-byte length prefix** for reliable message reception
- **JSON data serialization** for structured communication
- **Checksum verification** for data integrity
- **ACK confirmation** for reliable delivery

### Callback System
```python
# Register handlers for different message types
server_comms.register_callback('sensor_data', process_sensor_data)
server_comms.register_callback('system_status', handle_bot_status)
```

## 📨 Supported Message Types

- `system_status` - Bot initialization and health reports
- `sensor_data` - Environment scan data from ultrasonic sensors
- `movement_complete` - Bot movement execution acknowledgments

## 🔄 Communication Flow

```
Bot Connects → Authentication → Data Reception → Callback Trigger → Response Send
```

## ⚡ Quick Start

```python
# Initialize server
server = ServerComms(host='0.0.0.0', port=5000)

# Register data handlers
server.register_callback('sensor_data', handle_scan_data)

# Start listening for bots
server.start_server()
```

## 📊 Statistics & Monitoring

- **Connection tracking**: Active bots and uptime
- **Message counters**: Sent/received statistics
- **Performance metrics**: Response times and error rates

## 🛡️ Reliability Features

- **Timeout handling**: 30-second operation timeouts
- **Graceful degradation**: Continues operation during single-bot failures
- **Resource cleanup**: Automatic socket and connection management
- **Error recovery**: Reconnection support for transient failures

The communication backbone that ensures reliable, real-time data exchange between the central system and mobile bot fleet.