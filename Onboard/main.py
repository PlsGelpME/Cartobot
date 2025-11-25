# main.py - ESP8266 Main Controller (Sequential)
# Sequential implementation for autonomous bot operation

import time
import json
from boot import GLOBALS, get_system_status, blink_led
from area_scanner import AreaScanner
from movement_controller import MovementController
from communication_handler import CommunicationHandler

# =============================================================================
# MODULE INITIALIZATION
# =============================================================================

print("=== ESP8266 MOBILE BOT MAIN CONTROLLER STARTING ===")

# Check system readiness from boot initialization
system_status = get_system_status()
if not system_status['system_ready']:
    print("SYSTEM NOT READY - Check boot initialization")
    print("Failed components:", system_status['hardware']['error_components'])
    
    # Enter error state with rapid LED blinking
    while True:
        blink_led(1, 0.1)
        time.sleep(0.1)

print("System ready - Initializing modules...")

# Initialize Area Scanner
area_scanner = AreaScanner(
    servo_pin=GLOBALS['pins']['servo'],
    trigger_pin=GLOBALS['pins']['ultrasonic_trigger'],
    echo_pin=GLOBALS['pins']['ultrasonic_echo']
)
print("Area Scanner initialized")

# Initialize Movement Controller
movement_controller = MovementController(
    step_pin=GLOBALS['pins']['stepper_step'],
    dir_pin=GLOBALS['pins']['stepper_dir'],
    steps_per_revolution=GLOBALS['config']['movement']['steps_per_revolution'],
    wheel_diameter_cm=GLOBALS['config']['movement']['wheel_diameter_cm'],
    wheel_base_cm=GLOBALS['config']['movement']['wheel_base_cm']
)
print("Movement Controller initialized")

# Initialize Communication Handler (handles WiFi connection)
server_ip = GLOBALS['config']['communication']['server_ip']
server_port = GLOBALS['config']['communication']['server_port']

communication_handler = CommunicationHandler(
    ssid='OnePlus 9R',
    password='kesav115',
    server_ip=server_ip,
    server_port=server_port
)
print("Communication Handler initialized")

# =============================================================================
# INITIAL SYSTEM SETUP
# =============================================================================

print("WiFi connection established by CommunicationHandler")

# Send initial system status to central computer
print("Sending system status to central computer...")
system_status_packet = {
    'type': 'system_status',
    'timestamp': time.time(),
    'status': system_status,
    'message': 'ESP8266 Bot Ready - Waiting for Start Signal'
}

# Use dummy sensor data structure for status transmission
status_result = communication_handler.send_data(system_status_packet,'system_status')
if status_result['status'] != 'success':
    print("Failed to send initial status")

# =============================================================================
# WAIT FOR START SIGNAL
# =============================================================================

print("Waiting for start signal from central system...")
print("Send 'START' command from central computer to begin operation")

start_signal_received = False
blink_led(1, 0.5)  # Slow blink while waiting

max_attempts = 300  # 5 minutes at 1 second intervals
attempt = 0

while attempt < max_attempts and not start_signal_received:
    try:
        # Try to get instruction from central computer
        instruction = communication_handler.get_instruction()
        
        if instruction['status'] == 'success':
            # Check if this is a start signal
            if (instruction.get('command') == 'start' or 
                instruction.get('signal') == 'green_flag' or
                instruction.get('message') == 'START' or
                (instruction.get('distance') is not None and instruction.get('angle') is not None)):
                
                start_signal_received = True
                print("START SIGNAL RECEIVED! Beginning operation...")
                blink_led(5, 0.1)  # Rapid success blink
                break
            else:
                print(f"Received non-start instruction: {instruction}")
        
        time.sleep(1)
        attempt += 1
        
        # Blink every 10 seconds to show we're alive
        if attempt % 10 == 0:
            blink_led(1, 0.1)
            
    except Exception as e:
        print(f"Error waiting for start signal: {e}")
        time.sleep(2)
        attempt += 1

if not start_signal_received:
    print("Start signal timeout or not received")
    print("Entering standby mode")
    while True:
        blink_led(1, 1.0)
        time.sleep(1)

# =============================================================================
# MAIN AUTONOMOUS OPERATION LOOP
# =============================================================================

print("=== ENTERING MAIN WORK LOOP ===")

running = True
cycle_count = 0
consecutive_failures = 0
max_consecutive_failures = 5

while running:
    cycle_count += 1
    print(f"\n=== Work Cycle #{cycle_count} ===")
    
    try:
        # STEP 1: SCAN - Get environment data
        print("Step 1: Scanning environment...")
        scan_data = area_scanner.scan_area()
        print(f"Scan completed: {len(scan_data)} data points")
        
        # STEP 2: SEND - Transmit data to central system
        print("Step 2: Sending data to central system...")
        send_result = communication_handler.send_data(scan_data)
        
        if send_result['status'] != 'success':
            print(f"Data transmission failed: {send_result['reason']}")
            consecutive_failures += 1
            print(f"Work cycle #{cycle_count} failed, retrying...")
            time.sleep(2)
            
            if consecutive_failures >= max_consecutive_failures:
                print("Too many consecutive failures, entering error state")
                break
            continue
        
        print("Data sent successfully")
        
        # STEP 3: RECEIVE INSTRUCTION - Get movement commands
        print("Step 3: Waiting for movement instruction...")
        instruction = communication_handler.get_instruction()
        
        if instruction['status'] != 'success':
            print(f"Instruction reception failed: {instruction['reason']}")
            consecutive_failures += 1
            print(f"Work cycle #{cycle_count} failed, retrying...")
            time.sleep(2)
            
            if consecutive_failures >= max_consecutive_failures:
                print("Too many consecutive failures, entering error state")
                break
            continue
        
        print(f"Instruction received: Distance={instruction['distance']}cm, Angle={instruction['angle']}°")
        
        # STEP 4: MOVE - Execute movement
        print("Step 4: Executing movement...")
        movement_controller.move_with_angle(
            instruction['distance'], 
            instruction['angle']
        )
        print("Movement completed")
        
        # Reset failure counter on successful cycle
        consecutive_failures = 0
        print("--- Work Cycle Completed ---")
        
    except Exception as e:
        print(f"Work cycle error: {e}")
        consecutive_failures += 1
        print(f"Work cycle #{cycle_count} failed, retrying...")
        time.sleep(2)
        
        if consecutive_failures >= max_consecutive_failures:
            print("Too many consecutive failures, entering error state")
            break
    
    # Brief pause between cycles
    time.sleep(1)

print("=== MAIN WORK LOOP ENDED ===")

# If we exit the main loop, enter error state
print("System halted - Entering error state")
while True:
    blink_led(2, 0.2)
    time.sleep(0.5)