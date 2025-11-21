# server_comms.py - Central Server Communication Handler
# This module handles all network communication with mobile bots
# Manages TCP connections, data reception, and instruction transmission

import socket
import json
import threading
import time
import logging
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Callable

class ServerComms:
    """
    Central Server Communication Handler
    Manages multiple bot connections, data processing, and instruction routing
    Implements reliable TCP communication with callback system
    """
    
    def __init__(self, host: str = '0.0.0.0', port: int = 5000):
        """
        Initialize the communication server with network parameters
        
        Args:
            host: Server host address (0.0.0.0 for all interfaces)
            port: Server port number for bot connections
        """
        # Network configuration
        self.host = host
        self.port = port
        
        # Server socket - will be initialized when server starts
        self.server_socket = None
        
        # Server control flag
        self.running = False
        
        # Connected bots tracking dictionary
        # Format: {bot_id: {socket, address, connected_time, last_activity}}
        self.connected_bots = {}
        
        # Callback system for different data types
        # Format: {data_type: [callback_function1, callback_function2, ...]}
        self.data_callbacks = {}
        
        # Communication statistics for monitoring and debugging
        self.stats = {
            'total_messages_received': 0,  # Count of all messages received from bots
            'total_messages_sent': 0,      # Count of all messages sent to bots
            'total_bots_connected': 0,     # Total bots connected since startup
            'start_time': datetime.now()   # Server startup timestamp
        }
        
        # Setup logging for communication operations
        self.setup_logging()
        
    def setup_logging(self):
        """
        Configure logging for the communication module
        Provides detailed logging for debugging and monitoring
        """
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('server_comms.log'),  # Persistent log file
                logging.StreamHandler()  # Console output
            ]
        )
        self.logger = logging.getLogger('ServerComms')
        
    def start_server(self):
        """
        Start the communication server to listen for bot connections
        Initializes TCP socket and begins accepting connections
        
        Returns:
            bool: True if server started successfully, False otherwise
        """
        try:
            # Create TCP socket for server operations
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow port reuse
            self.server_socket.bind((self.host, self.port))  # Bind to specified host and port
            self.server_socket.listen(5)  # Allow up to 5 pending connections
            self.server_socket.settimeout(1.0)  # 1 second timeout for graceful shutdown
            
            # Set server running flag
            self.running = True
            
            # Log server startup
            self.logger.info(f"ServerComms started on {self.host}:{self.port}")
            
            # Start connection handler in separate thread
            connection_thread = threading.Thread(target=self._connection_handler)
            connection_thread.daemon = True  # Thread will exit when main thread exits
            connection_thread.start()
            
            return True  # Server started successfully
            
        except Exception as e:
            self.logger.error(f"Failed to start ServerComms: {e}")
            return False  # Server startup failed
            
    def _connection_handler(self):
        """
        Handle incoming bot connections in main server thread
        Accepts new connections and spawns handler threads for each bot
        """
        while self.running:
            try:
                # Accept incoming connection (blocks until connection or timeout)
                client_socket, client_address = self.server_socket.accept()
                
                # Create unique bot identifier from client address
                bot_id = f"{client_address[0]}:{client_address[1]}"
                
                # Log new connection
                self.logger.info(f"New bot connection: {bot_id}")
                
                # Update connection statistics
                self.stats['total_bots_connected'] += 1
                
                # Store bot connection information
                self.connected_bots[bot_id] = {
                    'socket': client_socket,      # Bot socket for communication
                    'address': client_address,    # Bot network address
                    'connected_time': datetime.now(),  # Connection timestamp
                    'last_activity': datetime.now()    # Last communication timestamp
                }
                
                # Start dedicated communication thread for this bot
                bot_thread = threading.Thread(
                    target=self._bot_communication_handler,
                    args=(client_socket, bot_id)
                )
                bot_thread.daemon = True  # Thread exits when main thread exits
                bot_thread.start()
                
            except socket.timeout:
                # Timeout is expected - allows checking running flag periodically
                continue
            except Exception as e:
                if self.running:
                    self.logger.error(f"Connection handler error: {e}")
                    
    def _bot_communication_handler(self, client_socket: socket.socket, bot_id: str):
        """
        Handle communication with a single mobile bot
        Manages data reception and processing for individual bot connections
        
        Args:
            client_socket: Socket object for bot communication
            bot_id: Unique identifier for the connected bot
        """
        try:
            # Set socket timeout for operations (30 seconds)
            client_socket.settimeout(30.0)
            
            # Main communication loop for this bot
            while self.running and bot_id in self.connected_bots:
                # Receive and process data from bot
                data = self._receive_data(client_socket, bot_id)
                if data is None:
                    break  # No data or connection error - exit loop
                    
                # Update last activity timestamp for this bot
                self.connected_bots[bot_id]['last_activity'] = datetime.now()
                
                # Update message statistics
                self.stats['total_messages_received'] += 1
                
                # Process received data and trigger callbacks
                self._process_received_data(bot_id, data)
                
        except socket.timeout:
            self.logger.warning(f"Bot {bot_id} communication timeout")
        except Exception as e:
            self.logger.error(f"Error handling bot {bot_id}: {e}")
        finally:
            # Always clean up bot connection resources
            self._cleanup_bot_connection(bot_id, client_socket)
            
    def _receive_data(self, client_socket: socket.socket, bot_id: str) -> Optional[Dict]:
        """
        Receive and parse data from mobile bot
        Implements protocol: [4-byte length][json data]
        
        Args:
            client_socket: Socket for receiving data
            bot_id: Identifier of the sending bot
            
        Returns:
            Optional[Dict]: Parsed JSON data, or None if reception failed
        """
        try:
            # First receive data length (4 bytes, big-endian)
            length_data = client_socket.recv(4)
            if len(length_data) != 4:
                return None  # Invalid length header
                
            # Convert bytes to integer data length
            data_length = int.from_bytes(length_data, 'big')
            
            # Receive actual data in chunks until complete
            received_data = b''
            while len(received_data) < data_length:
                # Calculate chunk size to avoid over-reading
                chunk_size = min(4096, data_length - len(received_data))
                chunk = client_socket.recv(chunk_size)
                if not chunk:
                    break  # No more data available
                received_data += chunk
                
            # Verify complete data reception
            if len(received_data) != data_length:
                return None  # Incomplete data
                
            # Parse JSON data from received bytes
            json_data = received_data.decode('utf-8')
            parsed_data = json.loads(json_data)
            
            # Send acknowledgment with checksum for data integrity
            checksum = self._calculate_checksum(received_data)
            ack_packet = checksum.to_bytes(4, 'big') + b'ACK'  # 4-byte checksum + ACK
            client_socket.sendall(ack_packet)
            
            # Log successful data reception
            self.logger.debug(f"Received data from {bot_id}: {parsed_data.get('type', 'unknown')}")
            return parsed_data
            
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error from {bot_id}: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Data reception error from {bot_id}: {e}")
            return None
            
    def _process_received_data(self, bot_id: str, data: Dict):
        """
        Process received data and trigger registered callbacks
        Routes data to appropriate handlers based on data type
        
        Args:
            bot_id: Identifier of the sending bot
            data: Parsed data dictionary from the bot
        """
        # Extract data type for callback routing
        data_type = data.get('type', 'unknown')
        
        # Trigger registered callbacks for this data type
        if data_type in self.data_callbacks:
            for callback in self.data_callbacks[data_type]:
                try:
                    # Execute callback with bot_id and data
                    callback(bot_id, data)
                except Exception as e:
                    self.logger.error(f"Callback error for {data_type}: {e}")
        else:
            self.logger.warning(f"No callbacks registered for data type: {data_type}")
            
    def send_instruction(self, bot_id: str, instruction: Dict) -> bool:
        """
        Send movement instruction to specific bot
        Implements reliable instruction transmission
        
        Args:
            bot_id: Unique identifier of the target bot
            instruction: Movement instruction dictionary
            
        Returns:
            bool: True if instruction sent successfully, False otherwise
        """
        # Check if bot is connected
        if bot_id not in self.connected_bots:
            self.logger.error(f"Bot {bot_id} not connected")
            return False
            
        try:
            # Get bot socket for communication
            client_socket = self.connected_bots[bot_id]['socket']
            
            # Convert instruction to JSON and then to bytes
            instruction_json = json.dumps(instruction)
            data_bytes = instruction_json.encode('utf-8')
            data_length = len(data_bytes)
            
            # Send instruction with length prefix
            client_socket.sendall(data_length.to_bytes(4, 'big'))  # Send length first
            client_socket.sendall(data_bytes)  # Send actual instruction data
            
            # Update statistics and activity timestamp
            self.stats['total_messages_sent'] += 1
            self.connected_bots[bot_id]['last_activity'] = datetime.now()
            
            # Log successful instruction transmission
            self.logger.info(f"Sent instruction to {bot_id}: {instruction}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send instruction to {bot_id}: {e}")
            self._cleanup_bot_connection(bot_id, client_socket)
            return False
            
    def send_to_all_bots(self, instruction: Dict) -> Dict[str, bool]:
        """
        Send instruction to all connected bots
        Useful for broadcast commands or synchronized operations
        
        Args:
            instruction: Instruction to send to all bots
            
        Returns:
            Dict: Results for each bot {bot_id: success_status}
        """
        results = {}
        # Send to each connected bot and record results
        for bot_id in list(self.connected_bots.keys()):
            results[bot_id] = self.send_instruction(bot_id, instruction)
        return results
        
    def register_callback(self, data_type: str, callback: Callable):
        """
        Register callback function for specific data type
        Enables modular processing of different message types
        
        Args:
            data_type: Type of data to handle (e.g., 'sensor_data', 'system_status')
            callback: Function to call when data of this type is received
                     Signature: callback(bot_id: str, data: Dict)
        """
        # Initialize callback list for new data types
        if data_type not in self.data_callbacks:
            self.data_callbacks[data_type] = []
        
        # Add callback to the list for this data type
        self.data_callbacks[data_type].append(callback)
        self.logger.info(f"Registered callback for data type: {data_type}")
        
    def unregister_callback(self, data_type: str, callback: Callable):
        """
        Unregister callback function for specific data type
        Allows dynamic management of data handlers
        
        Args:
            data_type: Type of data to remove handler for
            callback: Callback function to remove
        """
        if data_type in self.data_callbacks and callback in self.data_callbacks[data_type]:
            self.data_callbacks[data_type].remove(callback)
            self.logger.info(f"Unregistered callback for data type: {data_type}")
            
    def get_connected_bots(self) -> List[str]:
        """
        Get list of currently connected bot IDs
        Useful for monitoring and management
        
        Returns:
            List[str]: List of connected bot identifiers
        """
        return list(self.connected_bots.keys())
        
    def get_bot_info(self, bot_id: str) -> Optional[Dict]:
        """
        Get information about specific connected bot
        
        Args:
            bot_id: Unique identifier of the bot
            
        Returns:
            Optional[Dict]: Bot information dictionary, or None if not found
        """
        return self.connected_bots.get(bot_id)
        
    def get_communication_stats(self) -> Dict:
        """
        Get comprehensive communication statistics
        Provides insights into server performance and usage
        
        Returns:
            Dict: Communication statistics and server information
        """
        # Create statistics copy with current values
        stats = self.stats.copy()
        stats['current_connections'] = len(self.connected_bots)  # Active connections
        stats['uptime'] = str(datetime.now() - stats['start_time']).split('.')[0]  # Server uptime
        return stats
        
    def _calculate_checksum(self, data: bytes) -> int:
        """
        Calculate simple 32-bit checksum for data verification
        Used to ensure data integrity during transmission
        
        Args:
            data: Bytes data to calculate checksum for
            
        Returns:
            int: 32-bit checksum value
        """
        checksum = 0
        for byte in data:
            # Add each byte to checksum and maintain 32-bit boundary
            checksum = (checksum + byte) & 0xFFFFFFFF  # 32-bit checksum
        return checksum
        
    def _cleanup_bot_connection(self, bot_id: str, client_socket: socket.socket):
        """
        Clean up bot connection resources
        Removes bot from tracking and closes socket
        
        Args:
            bot_id: Identifier of the bot to clean up
            client_socket: Socket to close
        """
        try:
            client_socket.close()  # Close socket connection
        except:
            pass  # Ignore errors during socket closure
            
        # Remove bot from connected bots tracking
        if bot_id in self.connected_bots:
            del self.connected_bots[bot_id]
            self.logger.info(f"Bot {bot_id} disconnected")
            
    def stop_server(self):
        """
        Stop the communication server gracefully
        Closes all connections and releases resources
        """
        self.running = False  # Stop server operations
        self.logger.info("Stopping ServerComms...")
        
        # Close all bot connections
        for bot_id, bot_info in list(self.connected_bots.items()):
            try:
                bot_info['socket'].close()  # Close each bot socket
            except:
                pass  # Ignore errors during closure
            del self.connected_bots[bot_id]  # Remove from tracking
            
        # Close server socket
        if self.server_socket:
            self.server_socket.close()
            
        self.logger.info("ServerComms stopped successfully")

# Example usage and test functions
def main():
    """
    Example usage of ServerComms module
    Demonstrates basic server operation and callback handling
    """
    
    def handle_system_status(bot_id: str, data: Dict):
        """
        Example callback for system status messages
        """
        print(f"System status from {bot_id}: {data.get('message')}")
        
    def handle_sensor_data(bot_id: str, data: Dict):
        """
        Example callback for sensor data messages
        """
        data_points = data.get('data_points', [])
        print(f"Sensor data from {bot_id}: {len(data_points)} points")
        
        # Example: In real usage, this would trigger path planning
        # and send movement instructions back to the bot
        
    # Create and start server
    server_comms = ServerComms(host='0.0.0.0', port=5000)
    
    # Register callback functions for different data types
    server_comms.register_callback('system_status', handle_system_status)
    server_comms.register_callback('sensor_data', handle_sensor_data)
    
    # Start server
    if server_comms.start_server():
        print("ServerComms started successfully. Press Ctrl+C to stop.")
        
        try:
            # Keep server running
            while server_comms.running:
                time.sleep(1)
                
                # Display statistics every 10 seconds
                if int(time.time()) % 10 == 0:
                    stats = server_comms.get_communication_stats()
                    print(f"Active bots: {stats['current_connections']}, "
                          f"Messages: {stats['total_messages_received']} received, "
                          f"{stats['total_messages_sent']} sent")
                          
        except KeyboardInterrupt:
            print("\nStopping server...")
        finally:
            server_comms.stop_server()
    else:
        print("Failed to start ServerComms")

# Conditional execution for testing
if __name__ == "__main__":
    main()