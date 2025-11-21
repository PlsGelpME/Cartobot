# boot.py - Mobile Bot Initialization and System Boot
# This module initializes all hardware components and establishes system globals
# It runs automatically on boot and prepares the system for main.py

import network
import machine
import time
import json
from machine import Pin, PWM

# =============================================================================
# PIN DEFINITIONS - Centralized pin configuration for all hardware components
# =============================================================================

# Status LED for visual feedback and system status indication
LED_PIN = 2

# Servo Motor for ultrasonic scanner - controls scanning direction
SERVO_PIN = 13

# Stepper Motor for locomotion - controls bot movement
STEPPER_STEP_PIN = 12    # Step pin controls motor steps
STEPPER_DIR_PIN = 14     # Direction pin controls movement direction

# Ultrasonic Sensor (HC-SR04) for distance measurement
ULTRASONIC_TRIGGER_PIN = 5   # Trigger pin sends ultrasound pulses
ULTRASONIC_ECHO_PIN = 18     # Echo pin receives reflected signals

# =============================================================================
# GLOBAL VARIABLES AND SYSTEM STATUS - Accessible from all modules via main.py
# =============================================================================

# Global dictionary containing system status, hardware objects, and configuration
# This serves as the central data store for the entire application
GLOBALS = {
    # Tracks hardware initialization status and errors
    'hardware_status': {},
    
    # Stores network connection information and status
    'network_status': {},
    
    # Overall system readiness flag - True when all components are initialized
    'system_ready': False,
    
    # Detailed status of each hardware component
    'components': {},
    
    # System configuration parameters for all modules
    'config': {},
    
    # Pin number references for all hardware components
    'pins': {
        'led_status': LED_PIN,
        'servo': SERVO_PIN,
        'stepper_step': STEPPER_STEP_PIN,
        'stepper_dir': STEPPER_DIR_PIN,
        'ultrasonic_trigger': ULTRASONIC_TRIGGER_PIN,
        'ultrasonic_echo': ULTRASONIC_ECHO_PIN
    },
    
    # Initialized hardware objects - populated during initialization
    'objects': {
        'led': None,                # Status LED object
        'servo': None,              # Servo motor PWM object
        'stepper_step': None,       # Stepper step pin object
        'stepper_dir': None,        # Stepper direction pin object
        'ultrasonic_trigger': None, # Ultrasonic trigger pin object
        'ultrasonic_echo': None     # Ultrasonic echo pin object
    }
}

# =============================================================================
# HARDWARE INITIALIZATION FUNCTIONS - Component-specific setup routines
# =============================================================================

def initialize_led():
    """
    Initialize the status LED for visual system feedback
    The LED provides visual indication of system status and errors
    """
    try:
        # Create LED object on specified pin, set as output
        led = Pin(LED_PIN, Pin.OUT)
        led.value(0)  # Turn off LED initially for clean start
        
        # Store LED object in global dictionary for system-wide access
        GLOBALS['objects']['led'] = led
        
        # Update component status to indicate successful initialization
        GLOBALS['components']['led'] = {'initialized': True, 'pin': LED_PIN}
        
        # Log successful initialization
        print("Status LED initialized on pin", LED_PIN)
        return led
        
    except Exception as e:
        # Handle initialization failure and update status accordingly
        GLOBALS['components']['led'] = {'initialized': False, 'error': str(e)}
        print("LED init error:", e)
        return None

def blink_led(times=1, delay=0.2):
    """
    Provide visual feedback using onboard LED
    Different blink patterns indicate different system states
    
    Args:
        times: Number of blink cycles
        delay: Delay between on/off states in seconds
    """
    # Get LED object from globals
    led = GLOBALS['objects']['led']
    if not led:
        return  # Skip if LED not initialized
        
    # Execute blink pattern
    for _ in range(times):
        led.value(1)        # Turn LED on
        time.sleep(delay)   # Maintain on state
        led.value(0)        # Turn LED off
        time.sleep(delay)   # Maintain off state

def connect_wifi():
    """
    Establish WiFi connection to central computer
    Implements connection timeout and error handling
    """
    # Initialize WiFi interface in station mode
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # WiFi credentials - MUST BE UPDATED for your network
    ssid = 'YOUR_WIFI_SSID'
    password = 'YOUR_WIFI_PASSWORD'
    
    # Initialize network status in globals
    GLOBALS['network_status']['ssid'] = ssid
    GLOBALS['network_status']['connected'] = False
    GLOBALS['network_status']['ip_address'] = None
    
    # Only attempt connection if not already connected
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(ssid, password)
        
        # Wait for connection with timeout protection
        timeout = 20
        while not wlan.isconnected():
            timeout -= 1
            if timeout == 0:
                # Connection failed - update status and provide visual feedback
                print('WiFi connection failed')
                GLOBALS['network_status']['error'] = 'Connection timeout'
                blink_led(5, 0.1)  # Rapid blink indicates connection failure
                return None
            time.sleep(1)
    
    # Connection successful - extract and store network information
    network_info = wlan.ifconfig()
    GLOBALS['network_status']['connected'] = True
    GLOBALS['network_status']['ip_address'] = network_info[0]
    GLOBALS['network_status']['subnet_mask'] = network_info[1]
    GLOBALS['network_status']['gateway'] = network_info[2]
    GLOBALS['network_status']['dns'] = network_info[3]
    
    # Log success and provide visual confirmation
    print('WiFi connected:', network_info)
    blink_led(2)  # Double blink indicates successful connection
    return wlan

def initialize_servo():
    """
    Initialize servo motor for ultrasonic scanner assembly
    Configures PWM for precise angle control
    """
    try:
        # Create PWM object on servo pin for precise position control
        servo = PWM(Pin(SERVO_PIN))
        servo.freq(50)  # Standard 50Hz frequency for servo motors
        
        # Store servo object in globals for system access
        GLOBALS['objects']['servo'] = servo
        
        # Update component status with initialization details
        GLOBALS['components']['servo'] = {
            'initialized': True,
            'pin': SERVO_PIN,
            'frequency': 50  # Store frequency for reference
        }
        
        print("Servo initialized on pin", SERVO_PIN)
        return servo
        
    except Exception as e:
        # Handle servo initialization failure
        GLOBALS['components']['servo'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Servo init error:", e)
        return None

def initialize_stepper():
    """
    Initialize stepper motor control pins for bot locomotion
    Sets up step and direction pins for motor control
    """
    try:
        # Initialize step pin - controls individual motor steps
        step_pin = Pin(STEPPER_STEP_PIN, Pin.OUT)
        
        # Initialize direction pin - controls movement direction
        dir_pin = Pin(STEPPER_DIR_PIN, Pin.OUT)
        
        # Set initial states to prevent unexpected movement
        step_pin.value(0)  # Low state - no stepping
        dir_pin.value(0)   # Default direction
        
        # Store pin objects in globals for motor control
        GLOBALS['objects']['stepper_step'] = step_pin
        GLOBALS['objects']['stepper_dir'] = dir_pin
        
        # Update stepper component status
        GLOBALS['components']['stepper'] = {
            'initialized': True,
            'step_pin': STEPPER_STEP_PIN,
            'dir_pin': STEPPER_DIR_PIN
        }
        
        print("Stepper motors initialized on pins", STEPPER_STEP_PIN, "and", STEPPER_DIR_PIN)
        return step_pin, dir_pin
        
    except Exception as e:
        # Handle stepper initialization failure
        GLOBALS['components']['stepper'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Stepper init error:", e)
        return None, None

def initialize_ultrasonic():
    """
    Initialize ultrasonic distance sensor (HC-SR04)
    Configures trigger and echo pins for distance measurement
    """
    try:
        # Initialize trigger pin - output for sending ultrasound pulses
        trigger = Pin(ULTRASONIC_TRIGGER_PIN, Pin.OUT)
        
        # Initialize echo pin - input for receiving reflected signals
        echo = Pin(ULTRASONIC_ECHO_PIN, Pin.IN)
        
        # Initialize trigger to low state for clean start
        trigger.value(0)
        
        # Store sensor objects in globals for distance measurement
        GLOBALS['objects']['ultrasonic_trigger'] = trigger
        GLOBALS['objects']['ultrasonic_echo'] = echo
        
        # Update ultrasonic component status
        GLOBALS['components']['ultrasonic'] = {
            'initialized': True,
            'trigger_pin': ULTRASONIC_TRIGGER_PIN,
            'echo_pin': ULTRASONIC_ECHO_PIN
        }
        
        print("Ultrasonic sensor initialized on pins", ULTRASONIC_TRIGGER_PIN, "and", ULTRASONIC_ECHO_PIN)
        return trigger, echo
        
    except Exception as e:
        # Handle ultrasonic sensor initialization failure
        GLOBALS['components']['ultrasonic'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Ultrasonic init error:", e)
        return None, None

# =============================================================================
# SYSTEM VALIDATION FUNCTIONS - Verify hardware and network functionality
# =============================================================================

def check_hardware():
    """
    Verify all hardware components are properly initialized
    Updates global hardware status and identifies failed components
    """
    hardware_ok = True
    error_components = []
    
    # Check each component's initialization status
    for component, status in GLOBALS['components'].items():
        if not status.get('initialized', False):
            hardware_ok = False
            error_components.append(component)
    
    # Update global hardware status with comprehensive information
    GLOBALS['hardware_status']['all_ok'] = hardware_ok
    GLOBALS['hardware_status']['error_components'] = error_components
    GLOBALS['hardware_status']['total_components'] = len(GLOBALS['components'])
    GLOBALS['hardware_status']['working_components'] = len(GLOBALS['components']) - len(error_components)
    
    return hardware_ok

def check_network():
    """
    Verify network connectivity status
    Updates global network status flag
    """
    network_ok = GLOBALS['network_status'].get('connected', False)
    GLOBALS['hardware_status']['network_ok'] = network_ok
    return network_ok

def initialize_config():
    """
    Initialize global configuration parameters for all system modules
    Centralized configuration management for easy tuning
    """
    GLOBALS['config'] = {
        'movement': {
            'steps_per_revolution': 200,  # Stepper motor resolution
            'wheel_diameter_cm': 6.5,     # Bot wheel diameter for distance calculation
            'wheel_base_cm': 15.0,        # Distance between wheels for turning
            'step_delay_ms': 2            # Delay between steps for speed control
        },
        'scanning': {
            'scan_angle_min': 0,          # Minimum servo angle for scanning
            'scan_angle_max': 180,        # Maximum servo angle for scanning
            'scan_step_angle': 10,        # Angle increment between scans
            'servo_speed_delay': 0.3      # Delay for servo to reach position
        },
        'communication': {
            'server_ip': '192.168.1.100', # Central computer IP address
            'server_port': 5000,          # Communication port
            'timeout': 10                 # Network timeout in seconds
        }
    }

def get_system_status():
    """
    Generate comprehensive system status report
    Used by main.py to determine system readiness
    """
    # Calculate overall system readiness
    system_ready = (GLOBALS['hardware_status'].get('all_ok', False) and 
                   GLOBALS['hardware_status'].get('network_ok', False))
    
    GLOBALS['system_ready'] = system_ready
    
    # Compile detailed status report
    status_report = {
        'system_ready': system_ready,
        'hardware': GLOBALS['hardware_status'],
        'network': GLOBALS['network_status'],
        'components_initialized': {},
        'config': GLOBALS['config'],
        'pins': GLOBALS['pins'],
        'objects_available': {name: obj is not None for name, obj in GLOBALS['objects'].items()}
    }
    
    # Add detailed component initialization status
    for component, details in GLOBALS['components'].items():
        status_report['components_initialized'][component] = details.get('initialized', False)
    
    return status_report

def print_system_status():
    """
    Display comprehensive system status to console
    Provides human-readable system initialization summary
    """
    status = get_system_status()
    print("\n" + "="*50)
    print("MOBILE BOT SYSTEM STATUS")
    print("="*50)
    print(f"System Ready: {'YES' if status['system_ready'] else 'NO'}")
    print(f"Hardware Status: {status['hardware']['working_components']}/{status['hardware']['total_components']} components OK")
    
    # Display failed components if any
    if status['hardware']['error_components']:
        print(f"Failed Components: {', '.join(status['hardware']['error_components'])}")
    
    # Display network status
    print(f"Network: {'CONNECTED' if status['network']['connected'] else 'DISCONNECTED'}")
    if status['network']['connected']:
        print(f"IP Address: {status['network']['ip_address']}")
    
    # Display individual component status
    print("Component Status:")
    for component, initialized in status['components_initialized'].items():
        print(f"  {component}: {'OK' if initialized else 'FAILED'}")
    
    # Display pin configuration for reference
    print("Pin Configuration:")
    for name, pin in status['pins'].items():
        print(f"  {name}: GPIO{pin}")
    
    print("="*50)

# =============================================================================
# MAIN INITIALIZATION SEQUENCE - Executed automatically on boot
# =============================================================================

print("Initializing Mobile Bot System...")

# Step 1: Initialize system configuration parameters
initialize_config()

# Step 2: Initialize hardware components in logical sequence
# LED first for status feedback, then sensors, then actuators
initialize_led()
initialize_servo()
initialize_stepper()
initialize_ultrasonic()

# Step 3: Establish network connection
wlan = connect_wifi()

# Step 4: Validate all systems
hardware_ok = check_hardware()
network_ok = check_network()

# Step 5: Determine overall system readiness
GLOBALS['system_ready'] = hardware_ok and network_ok

# Step 6: Display comprehensive status report
print_system_status()

# Step 7: Provide visual status indication
if GLOBALS['system_ready']:
    print("All systems ready! Starting main application...")
    blink_led(3)  # Triple blink indicates full system readiness
else:
    print("SYSTEM INITIALIZATION FAILED - Check components and network")
    blink_led(10, 0.1)  # Rapid blink indicates system failure

# Step 8: Cleanup and handoff to main application
GLOBALS['objects']['led'].value(0)  # Ensure LED is off before main.py takes over

# Final boot completion message
print("Boot sequence completed. Global objects available in GLOBALS dictionary")
print("Control transferring to main.py...")

# boot.py execution completes here, main.py will be executed next
# All hardware objects and system status are available via GLOBALS dictionary