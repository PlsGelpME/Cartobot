# communication_handler.py - Mobile Bot Communication Handler
# This module handles all communication with the central computer
# Manages data serialization, transmission, and instruction reception

import json
import socket
import network
import time
import errno
from machine import Pin

class CommunicationHandler:
    """
    Communication Handler for central computer interaction
    Manages WiFi connection, data transmission, and instruction reception
    Implements reliable TCP communication with acknowledgment protocol
    """
    
    def __init__(self, ssid, password, server_ip, server_port):
        """
        Initialize communication handler with network parameters
        
        Args:
            ssid: WiFi network name
            password: WiFi network password  
            server_ip: Central computer IP address
            server_port: Central computer port number
        """
        # Store connection parameters
        self.server_ip = server_ip
        self.server_port = server_port
        
        # Initialize WiFi interface
        self.wlan = network.WLAN(network.STA_IF)
        
        # Establish WiFi connection during initialization
        self.setup_wifi(ssid, password)
        
    def setup_wifi(self, ssid, password):
        """
        Connect to WiFi network with timeout and error handling
        
        Args:
            ssid: WiFi network name
            password: WiFi network password
        """
        # Activate WiFi interface
        self.wlan.active(True)
        
        # Only attempt connection if not already connected
        if not self.wlan.isconnected():
            print('Connecting to WiFi...')
            self.wlan.connect(ssid, password)
            
            # Wait for connection with timeout protection
            # Prevents infinite waiting if connection fails
            timeout = 20
            while not self.wlan.isconnected():
                timeout -= 1
                if timeout == 0:
                    # Connection failed - raise exception for error handling
                    raise Exception('WiFi connection failed')
                time.sleep(1)  # Wait 1 second between connection checks
                
        # Connection successful - display network configuration
        print('Network config:', self.wlan.ifconfig())
        
    def serialize_sensor_data(self, data, status_type):
        """
        Convert sensor data to JSON format for transmission
        Creates structured packet with metadata for central computer
        
        Args:
            scan_data: List of (angle, distance) tuples from AreaScanner
            
        Returns:
            str: JSON formatted string ready for transmission
        """
        # Convert raw scan data to list of dictionaries for better serialization
        # Each data point contains angle and distance in structured format
        
        if status_type == "system_status":
            return json.dumps(data)
        
        serializable_data = [{'angle': angle, 'distance_cm': distance} 
                           for angle, distance in data]
        
        # Create comprehensive data packet with metadata
        packet = {
            'type': status_type,
            'timestamp': time.time(),           # Unix timestamp for data synchronization
            'sensor_type': 'ultrasonic_scanner', # Identifies data source
            'data_points': serializable_data,   # Actual sensor measurements
            'data_count': len(serializable_data) # Number of data points for verification
        }
        
        # Convert to JSON string for transmission
        return json.dumps(packet)
        
    def send_data(self, sensor_data, status_type):
        """
        Send sensor data to central computer and receive acknowledgment
        Implements reliable transmission with checksum verification
        
        Args:
            sensor_data: List of (angle, distance) tuples from AreaScanner
            
        Returns:
            dict: Transmission status with success/failure information
        """
        # Serialize the sensor data for transmission
        json_data = self.serialize_sensor_data(sensor_data,status_type)
        
        # Create TCP socket for communication
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)  # 10 second timeout for transmission operations
        
        try:
            # Connect to central computer server
            server_addr = (self.server_ip, self.server_port)
            sock.connect(server_addr)
            
            # Convert JSON data to bytes for transmission
            data_bytes = json_data.encode('utf-8')
            data_length = len(data_bytes)
            header = data_length.to_bytes(4,'big')
            print(header,data_bytes)
            
            # Send data length first for protocol compliance
            # Central computer uses this to know how much data to expect
            sock.sendall(header)
            # Send actual sensor data
            bytes_sent = sock.sendall(data_bytes)
            
            # Wait for acknowledgment with checksum verification
            # Expects 8 bytes: 4-byte checksum + 3-byte "ACK" + 1 padding
            ack_response = sock.recv(8)
            
            # Validate acknowledgment length
            if len(ack_response) != 8:
                return {'status': 'error', 'reason': 'invalid_ack_length'}
                
            # Extract checksum and ACK code from response
            received_checksum = int.from_bytes(ack_response[:4], 'big')  # First 4 bytes = checksum
            ack_code = ack_response[4:7]  # Next 3 bytes = ACK code
            
            # Calculate local checksum for verification
            calculated_checksum = self.calculate_checksum(data_bytes)
            
            # Verify acknowledgment code
            if ack_code != b'ACK':
                return {'status': 'error', 'reason': 'acknowledgment_failed'}
                
            # Verify data integrity with checksum
            if received_checksum != calculated_checksum:
                return {'status': 'error', 'reason': 'checksum_mismatch'}
                
            # Transmission successful
            return {'status': 'success', 'bytes_sent': data_length}
                    
        except OSError as e:
            if e.args[0] == errno.ETIMEDOUT:
                # Handle network timeout
                return {'status': 'error', 'reason': 'communication_timeout'}
            else:            
                # Handle network-related errors
                return {'status': 'error', 'reason': f'network_error: {str(e)}'}
        except Exception as e:
            # Handle unexpected errors
            return {'status': 'error', 'reason': f'unexpected_error: {str(e)}'}
        finally:
            # Always close socket to free resources
            sock.close()
            
    def get_instruction(self):
        """
        Connect to server and receive movement instruction as distance-angle pair
        Implements request-response pattern for command reception
        
        Returns:
            dict: Movement instruction with status and data
        """
        # Create TCP socket for instruction reception
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(15.0)  # 15 second timeout for instruction waiting
        
        try:
            # Connect to central computer
            server_addr = (self.server_ip, self.server_port)
            sock.connect(server_addr)
            
            # Send instruction request signal to central computer
            # Informs server that bot is ready to receive commands
            request_signal = b'REQUEST_INSTRUCTION'
            sock.sendall(request_signal)
            
            # Receive instruction data length first (4 bytes)
            # This tells us how much data to expect
            length_data = sock.recv(4)
            if len(length_data) != 4:
                return {'status': 'error', 'reason': 'invalid_length_header'}
                
            instruction_length = int.from_bytes(length_data, 'big')
            
            # Receive the actual instruction data
            instruction_bytes = b''
            while len(instruction_bytes) < instruction_length:
                # Receive data in chunks to handle large instructions
                chunk_size = min(1024, instruction_length - len(instruction_bytes))
                chunk = sock.recv(chunk_size)
                if not chunk:
                    break  # No more data available
                instruction_bytes += chunk
                
            # Verify we received complete instruction
            if len(instruction_bytes) != instruction_length:
                return {'status': 'error', 'reason': 'incomplete_instruction_data'}
                
            # Parse JSON instruction data
            instruction_str = instruction_bytes.decode('utf-8')
            instruction_data = json.loads(instruction_str)
            
            print(instruction_data)
            # Validate instruction structure and content
            # Instead of validating as movement instruction, handle different types
            response_type = instruction_data.get('type', 'unknown')
            
            if response_type == 'movement_instruction':
                # Validate as movement command
                if not self.validate_instruction(instruction_data):
                    return {'status': 'error', 'reason': 'invalid_instruction_format'}
                distance = instruction_data.get('distance_cm', 0)
                angle = instruction_data.get('angle_degrees', 0)
                return {'status': 'success', 'type': 'movement', 'distance': distance, 'angle': angle}
            
            elif response_type == 'trial_response':
                # Handle trial response
                message = instruction_data.get('message', '')
                return {'status': 'success', 'type': 'trial_response', 'message': message}
            
            else:
                return {'status': 'error', 'reason': f'unknown_response_type: {response_type}'}   
            # Send acknowledgment to central computer
            sock.sendall(b'ACK')
            
            # Extract distance and angle pair from instruction
            distance = instruction_data.get('distance_cm', 0)
            angle = instruction_data.get('angle_degrees', 0)
            
            # Return successful instruction with movement data
            return {'status': 'success', 'distance': distance, 'angle': angle}
                
        except OSError as e:
            if e.args[0] == errno.ETIMEDOUT:
                # Handle instruction reception timeout
                return {'status': 'error', 'reason': 'instruction_timeout'}
            else:
                # Handle network errors
                return {'status': 'error', 'reason': f'network_error: {str(e)}'}      
        except json.JSONDecodeError:
            # Handle malformed JSON data
            return {'status': 'error', 'reason': 'invalid_json_instruction'}
        except Exception as e:
            # Handle unexpected errors
            return {'status': 'error', 'reason': f'unexpected_error: {str(e)}'}
        finally:
            # Always close socket
            sock.close()
            
    def validate_instruction(self, instruction_data):
        """
        Validate the structure and content of received movement instruction
        Ensures instruction is safe and properly formatted
        
        Args:
            instruction_data: Dictionary containing movement instruction
            
        Returns:
            bool: True if instruction is valid, False otherwise
        """
        # Required fields for any movement instruction
        required_fields = ['distance_cm', 'angle_degrees']
        
        # Check if all required fields are present
        if not all(field in instruction_data for field in required_fields):
            return False
            
        # Validate data types and ranges for safety
        try:
            # Extract and convert distance and angle values
            distance = float(instruction_data['distance_cm'])
            angle = float(instruction_data['angle_degrees'])
            
            # Apply safety limits to prevent extreme movements
            # Distance limits: -1000cm to 1000cm (reasonable movement range)
            if distance < -1000 or distance > 1000:
                return False
            # Angle limits: -360° to 360° (full circle range)
            if angle < -360 or angle > 360:
                return False
                
            # Instruction passed all validation checks
            return True
            
        except (ValueError, TypeError):
            # Handle conversion errors (non-numeric values)
            return False
            
    def calculate_checksum(self, data_bytes):
        """
        Calculate simple 32-bit checksum for data verification
        Used to ensure data integrity during transmission
        
        Args:
            data_bytes: Bytes data to calculate checksum for
            
        Returns:
            int: 32-bit checksum value
        """
        checksum = 0
        for byte in data_bytes:
            # Add each byte to checksum and maintain 32-bit boundary
            checksum = (checksum + byte) & 0xFFFFFFFF  # 32-bit checksum
        return checksum

# Usage example and test function
# Demonstrates how to use the CommunicationHandler class
def main():
    """
    Example usage of CommunicationHandler for testing
    """
    # Create communication handler instance
    # Replace with actual network credentials and server details
    comm = CommunicationHandler(
        ssid='OnePlus 9R', 
        password='kesav115', 
        server_ip='10.47.198.63', 
        server_port=5000
    )
    
    # Example sensor data (simulating AreaScanner output)
    trial_data = [(0, 45), (10, 50), (20, 55), (30, 60), (40, 65),
                   (50, 70), (60, 75), (70, 80), (80, 85), (90, 90),
                   (100, 85), (110, 80), (120, 75), (130, 70), (140, 65),
                   (150, 60), (160, 55), (170, 50), (180, 45)]
    
    # Send sensor data to central computer
    print("Sending sensor data...")
    send_result = comm.send_data(trial_data,"trial run")
    
    # Check transmission result
    if send_result['status'] == 'success':
        print('Data sent successfully')
        
        # Request movement instruction from central computer
        print("Requesting movement instruction...")
        instruction = comm.get_instruction()
        
        if instruction['status'] == 'success':
            print(f"Instruction received: Move {instruction['distance']}cm at angle {instruction['angle']}°")
        else:
            print(f"Instruction failed: {instruction['reason']}")
    else:
        print(f"Data transmission failed: {send_result['reason']}")

# Conditional execution for testing
if __name__ == "__main__":
    main()