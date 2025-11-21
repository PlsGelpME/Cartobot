# main.py - Mobile Bot Main Controller
# This module orchestrates the complete operational workflow of the mobile bot
# It coordinates all other modules and implements the main control loop

import time
import json
from boot import GLOBALS, get_system_status, blink_led
from area_scanner import AreaScanner
from movement_controller import MovementController
from communication_handler import CommunicationHandler

class MobileBotMain:
    """
    Main controller class for mobile bot operations
    Manages the complete workflow: scanning, communication, and movement
    """
    
    def __init__(self):
        """
        Initialize the main controller and system state
        Sets up module references and operational flags
        """
        # Get comprehensive system status from boot initialization
        self.system_status = get_system_status()
        
        # Module references - will be initialized later
        self.area_scanner = None        # Handles environment scanning
        self.movement_controller = None # Handles bot locomotion
        self.communication_handler = None # Handles central computer communication
        
        # Operational state flags
        self.running = False            # Main loop control flag
        self.start_signal_received = False # Central computer authorization flag
        
    def initialize_modules(self):
        """
        Initialize all operational modules using hardware from boot.py
        Returns True if all modules initialize successfully
        """
        try:
            # Initialize Area Scanner with hardware objects from boot.py
            # Uses servo for scanning and ultrasonic for distance measurement
            self.area_scanner = AreaScanner(
                servo_pin=GLOBALS['pins']['servo'],
                trigger_pin=GLOBALS['pins']['ultrasonic_trigger'],
                echo_pin=GLOBALS['pins']['ultrasonic_echo']
            )
            print("Area Scanner initialized")
            
            # Initialize Movement Controller for bot locomotion
            # Uses stepper motors for precise movement control
            self.movement_controller = MovementController(
                step_pin=GLOBALS['pins']['stepper_step'],
                dir_pin=GLOBALS['pins']['stepper_dir'],
                steps_per_revolution=GLOBALS['config']['movement']['steps_per_revolution'],
                wheel_diameter_cm=GLOBALS['config']['movement']['wheel_diameter_cm'],
                wheel_base_cm=GLOBALS['config']['movement']['wheel_base_cm']
            )
            print("Movement Controller initialized")
            
            # Initialize Communication Handler for central computer interaction
            # Manages data transmission and instruction reception
            server_ip = GLOBALS['config']['communication']['server_ip']
            server_port = GLOBALS['config']['communication']['server_port']
            
            self.communication_handler = CommunicationHandler(
                ssid='YOUR_WIFI_SSID',  # Should match boot.py credentials
                password='YOUR_WIFI_PASSWORD',
                server_ip=server_ip,
                server_port=server_port
            )
            print("Communication Handler initialized")
            
            # All modules initialized successfully
            return True
            
        except Exception as e:
            # Module initialization failed - log error
            print(f"Module initialization failed: {e}")
            return False
    
    def send_system_status(self):
        """
        Send initial system status to central computer
        Informs central system about bot readiness and capabilities
        """
        # Create status packet with system information
        status_packet = {
            'type': 'system_status',
            'timestamp': time.time(),
            'status': self.system_status,
            'message': 'Mobile Bot Ready - Waiting for Start Signal'
        }
        
        # Convert to JSON format for transmission
        # Using dummy sensor data structure since we're only sending status
        dummy_sensor_data = []  # Empty data, just sending status
        json_data = self.communication_handler.serialize_sensor_data(dummy_sensor_data)
        
        # Override with our status packet for proper formatting
        status_json = json.dumps(status_packet)
        
        # Create temporary socket connection to send status
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)  # 10 second timeout for status transmission
            
            # Connect to central computer
            sock.connect((GLOBALS['config']['communication']['server_ip'], 
                         GLOBALS['config']['communication']['server_port']))
            
            # Send status data with length prefix for protocol compliance
            data_bytes = status_json.encode('utf-8')
            data_length = len(data_bytes)
            sock.sendall(data_length.to_bytes(4, 'big'))  # Send data length first
            sock.sendall(data_bytes)  # Send actual status data
            
            # Wait for acknowledgment from central computer
            ack = sock.recv(3)  # Expect 3-byte ACK response
            sock.close()  # Close connection after status transmission
            
            if ack == b'ACK':
                print("System status sent successfully")
                return True
            else:
                print("Failed to get ACK for status")
                return False
                
        except Exception as e:
            # Status transmission failed
            print(f"Failed to send system status: {e}")
            return False
    
    def wait_for_start_signal(self):
        """
        Wait for green flag/start signal from central system
        Implements polling with timeout and visual feedback
        """
        print("Waiting for start signal from central system...")
        print("Send 'START' command from central computer to begin operation")
        
        # Provide visual feedback while waiting (slow blink)
        blink_led(1, 0.5)  # Slow blink indicates waiting state
        
        # Polling parameters - prevent infinite waiting
        max_attempts = 300  # 5 minutes at 1 second intervals (300 attempts)
        attempt = 0
        
        # Poll for start signal until received or timeout
        while attempt < max_attempts and not self.start_signal_received:
            try:
                # Attempt to get instruction from central computer
                instruction = self.communication_handler.get_instruction()
                
                if instruction['status'] == 'success':
                    # Check if this is a start signal
                    if self.is_start_signal(instruction):
                        self.start_signal_received = True
                        print("START SIGNAL RECEIVED! Beginning operation...")
                        blink_led(5, 0.1)  # Rapid success blink
                        break
                    else:
                        # Received instruction but not a start signal
                        print(f"Received non-start instruction: {instruction}")
                
                # Wait before next polling attempt
                time.sleep(1)  # 1 second between checks
                attempt += 1
                
                # Periodic visual feedback to show system is alive
                if attempt % 10 == 0:  # Every 10 seconds
                    blink_led(1, 0.1)  # Quick blink
                    
            except Exception as e:
                # Error during polling - log and continue
                print(f"Error waiting for start signal: {e}")
                time.sleep(2)  # Longer wait on error
                attempt += 1
        
        # Return whether start signal was received
        return self.start_signal_received
    
    def is_start_signal(self, instruction):
        """
        Check if the received instruction is a valid start signal
        Supports multiple start signal formats for flexibility
        
        Args:
            instruction: Dictionary containing instruction data
            
        Returns:
            bool: True if instruction is a start signal
        """
        # Check multiple possible start signal formats:
        # 1. Specific start command
        if instruction.get('command') == 'start':
            return True
        # 2. Green flag equivalent signal
        if instruction.get('signal') == 'green_flag':
            return True
        # 3. First movement instruction also counts as start signal
        if instruction.get('distance') is not None and instruction.get('angle') is not None:
            return True
        # 4. Explicit start message
        if instruction.get('message') == 'START':
            return True
            
        # Not a start signal
        return False
    
    def execute_work_cycle(self):
        """
        Execute one complete work cycle: Scan -> Send -> Receive Instruction -> Move
        This is the core operational loop of the mobile bot
        
        Returns:
            bool: True if cycle completed successfully
        """
        print("\n--- Starting Work Cycle ---")
        
        try:
            # STEP 1: SCAN - Get environment data using AreaScanner
            print("Step 1: Scanning environment...")
            scan_data = self.area_scanner.scan_area()  # Perform 180-degree scan
            print(f"Scan completed: {len(scan_data)} data points")
            
            # STEP 2: SEND - Transmit sensor data to central system
            print("Step 2: Sending data to central system...")
            send_result = self.communication_handler.send_data(scan_data)
            
            # Check if data transmission was successful
            if send_result['status'] != 'success':
                print(f"Data transmission failed: {send_result['reason']}")
                return False  # Cycle failed
            print("Data sent successfully")
            
            # STEP 3: RECEIVE INSTRUCTION - Get movement commands from central computer
            print("Step 3: Waiting for movement instruction...")
            instruction = self.communication_handler.get_instruction()
            
            # Validate received instruction
            if instruction['status'] != 'success':
                print(f"Instruction reception failed: {instruction['reason']}")
                return False  # Cycle failed
            print(f"Instruction received: Distance={instruction['distance']}cm, Angle={instruction['angle']}°")
            
            # STEP 4: MOVE - Execute movement using MovementController
            print("Step 4: Executing movement...")
            self.movement_controller.move_with_angle(
                instruction['distance'], 
                instruction['angle']
            )
            print("Movement completed")
            
            # STEP 5: Send movement completion acknowledgment to central computer
            completion_ack = {
                'type': 'movement_complete',
                'timestamp': time.time(),
                'distance_executed': instruction['distance'],
                'angle_executed': instruction['angle'],
                'status': 'success'
            }
            
            # Note: In full implementation, this would use a proper acknowledgment mechanism
            # For now, we log the completion locally
            print("Movement completion acknowledged locally")
            
            print("--- Work Cycle Completed ---")
            return True  # Cycle completed successfully
            
        except Exception as e:
            # Handle any errors during work cycle execution
            print(f"Work cycle error: {e}")
            return False  # Cycle failed
    
    def run(self):
        """
        Main execution loop - orchestrates the complete bot operation
        Implements the state machine and error handling
        """
        print("=== MOBILE BOT MAIN CONTROLLER STARTING ===")
        
        # PHASE 1: System Readiness Check
        # Verify that boot initialization was successful
        if not self.system_status['system_ready']:
            print("SYSTEM NOT READY - Check boot initialization")
            print("Failed components:", self.system_status['hardware']['error_components'])
            
            # Enter error state with rapid LED blinking
            while True:
                blink_led(1, 0.1)  # Rapid blink indicates system failure
                time.sleep(0.1)
            return  # Exit main - system cannot operate
        
        print("System ready - Initializing modules...")
        
        # PHASE 2: Module Initialization
        # Initialize all operational modules
        if not self.initialize_modules():
            print("MODULE INITIALIZATION FAILED")
            # Enter module failure state with double blink pattern
            while True:
                blink_led(2, 0.1)  # Double blink indicates module failure
                time.sleep(0.2)
            return  # Exit main - modules cannot be initialized
        
        # PHASE 3: Network Connectivity Check
        # Verify WiFi connection is active
        if not GLOBALS['network_status']['connected']:
            print("WIFI NOT CONNECTED - Entering error mode")
            # Enter network error state with rapid infinite blink
            while True:
                blink_led(1, 0.1)  # Rapid blink indicates network failure
                time.sleep(0.1)
            return  # Exit main - no network connection
        
        print("WiFi connected - Sending system status...")
        
        # PHASE 4: Initial Status Reporting
        # Send system status to central computer
        if not self.send_system_status():
            print("Failed to send initial status")
            # Continue anyway - central computer might still connect
        
        # PHASE 5: Start Signal Waiting
        # Wait for authorization from central computer to begin operation
        if not self.wait_for_start_signal():
            print("Start signal timeout or not received")
            print("Entering standby mode")
            # Enter standby state with slow blinking
            while True:
                blink_led(1, 1.0)  # Slow blink in standby mode
                time.sleep(1)
            return  # Exit main - no start signal received
        
        # PHASE 6: MAIN WORK LOOP
        # Continuous operation: Scan -> Send -> Receive -> Move
        print("=== ENTERING MAIN WORK LOOP ===")
        self.running = True
        cycle_count = 0
        consecutive_failures = 0
        max_consecutive_failures = 5
        
        while self.running:
            cycle_count += 1
            print(f"\n=== Work Cycle #{cycle_count} ===")
            
            # Execute one complete work cycle
            success = self.execute_work_cycle()
            
            if not success:
                # Work cycle failed - implement failure handling
                consecutive_failures += 1
                print(f"Work cycle #{cycle_count} failed, retrying...")
                time.sleep(2)  # Brief pause before retry
                
                # Check if too many consecutive failures have occurred
                if consecutive_failures >= max_consecutive_failures:
                    print("Too many consecutive failures, entering error state")
                    break  # Exit main loop
            else:
                # Work cycle successful - reset failure counter
                consecutive_failures = 0
            
            # Brief pause between cycles to prevent overwhelming the system
            time.sleep(1)
        
        # PHASE 7: Shutdown
        # Main loop ended - perform cleanup
        print("=== MAIN WORK LOOP ENDED ===")
        
    def stop(self):
        """
        Gracefully stop the main control loop
        Can be called for emergency shutdown or normal termination
        """
        self.running = False
        print("Main controller stopping...")

# Application entry point
# Creates and runs the main controller instance
if __name__ == "__main__":
    # Create main controller instance
    bot = MobileBotMain()
    
    try:
        # Start the main control loop
        bot.run()
    except KeyboardInterrupt:
        # Handle user interrupt (Ctrl+C) gracefully
        print("\nKeyboard interrupt received")
        bot.stop()
    except Exception as e:
        # Handle unexpected errors
        print(f"Unexpected error: {e}")
        # Enter error state with rapid blinking
        while True:
            blink_led(1, 0.1)
            time.sleep(0.1)