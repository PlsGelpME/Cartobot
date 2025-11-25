# boot.py - ESP8266 Hardware Initialization
# This module initializes all hardware components for ESP8266
# Runs automatically on boot and prepares hardware for main.py

import machine
import time
from machine import Pin, PWM

# =============================================================================
# PIN DEFINITIONS - ESP8266 Compatible Pin Configuration
# =============================================================================

# ESP8266 GPIO pins (D1, D2, etc. correspond to GPIO 5, 4, etc.)
LED_PIN = 2              # Built-in LED (GPIO2 - D4 on NodeMCU)

# Servo Motor for ultrasonic scanner
SERVO_PIN = 14           # GPIO14 (D5 on NodeMCU)

# Stepper Motor for locomotion
STEPPER_STEP_PIN = 12    # GPIO12 (D6 on NodeMCU)
STEPPER_DIR_PIN = 13     # GPIO13 (D7 on NodeMCU)

# Ultrasonic Sensor (HC-SR04)
ULTRASONIC_TRIGGER_PIN = 4   # GPIO4 (D2 on NodeMCU)
ULTRASONIC_ECHO_PIN = 5      # GPIO5 (D1 on NodeMCU)

# =============================================================================
# GLOBAL VARIABLES AND SYSTEM STATUS
# =============================================================================

# Global dictionary for system-wide access
GLOBALS = {
    'hardware_status': {},
    'components': {},
    'config': {},
    'pins': {
        'led_status': LED_PIN,
        'servo': SERVO_PIN,
        'stepper_step': STEPPER_STEP_PIN,
        'stepper_dir': STEPPER_DIR_PIN,
        'ultrasonic_trigger': ULTRASONIC_TRIGGER_PIN,
        'ultrasonic_echo': ULTRASONIC_ECHO_PIN
    },
    'objects': {
        'led': None,
        'servo': None,
        'stepper_step': None,
        'stepper_dir': None,
        'ultrasonic_trigger': None,
        'ultrasonic_echo': None
    }
}

# =============================================================================
# HARDWARE INITIALIZATION FUNCTIONS
# =============================================================================

def initialize_led():
    """Initialize the status LED for ESP8266"""
    try:
        # ESP8266 built-in LED is active LOW
        led = Pin(LED_PIN, Pin.OUT)
        led.value(1)  # Turn off initially (active low)
        
        GLOBALS['objects']['led'] = led
        GLOBALS['components']['led'] = {'initialized': True, 'pin': LED_PIN}
        print("Status LED initialized on pin", LED_PIN)
        return led
    except Exception as e:
        GLOBALS['components']['led'] = {'initialized': False, 'error': str(e)}
        print("LED init error:", e)
        return None

def blink_led(times=1, delay=0.2):
    """Visual feedback using ESP8266 built-in LED (active low)"""
    led = GLOBALS['objects']['led']
    if not led:
        return
        
    for _ in range(times):
        led.value(0)  # Turn on (active low)
        time.sleep(delay)
        led.value(1)  # Turn off
        time.sleep(delay)

def initialize_servo():
    """Initialize servo motor for ESP8266"""
    try:
        # ESP8266 PWM on specified pin
        servo = PWM(Pin(SERVO_PIN))
        servo.freq(50)  # 50Hz for servos
        
        GLOBALS['objects']['servo'] = servo
        GLOBALS['components']['servo'] = {
            'initialized': True,
            'pin': SERVO_PIN,
            'frequency': 50
        }
        print("Servo initialized on pin", SERVO_PIN)
        return servo
    except Exception as e:
        GLOBALS['components']['servo'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Servo init error:", e)
        return None

def initialize_stepper():
    """Initialize stepper motor control pins for ESP8266"""
    try:
        # ESP8266 GPIO pins for stepper control
        step_pin = Pin(STEPPER_STEP_PIN, Pin.OUT)
        dir_pin = Pin(STEPPER_DIR_PIN, Pin.OUT)
        
        # Set initial states
        step_pin.value(0)
        dir_pin.value(0)
        
        GLOBALS['objects']['stepper_step'] = step_pin
        GLOBALS['objects']['stepper_dir'] = dir_pin
        GLOBALS['components']['stepper'] = {
            'initialized': True,
            'step_pin': STEPPER_STEP_PIN,
            'dir_pin': STEPPER_DIR_PIN
        }
        print("Stepper motors initialized on pins", STEPPER_STEP_PIN, "and", STEPPER_DIR_PIN)
        return step_pin, dir_pin
    except Exception as e:
        GLOBALS['components']['stepper'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Stepper init error:", e)
        return None, None

def initialize_ultrasonic():
    """Initialize ultrasonic sensor for ESP8266"""
    try:
        trigger = Pin(ULTRASONIC_TRIGGER_PIN, Pin.OUT)
        echo = Pin(ULTRASONIC_ECHO_PIN, Pin.IN)
        
        # Initialize trigger to low
        trigger.value(0)
        
        GLOBALS['objects']['ultrasonic_trigger'] = trigger
        GLOBALS['objects']['ultrasonic_echo'] = echo
        GLOBALS['components']['ultrasonic'] = {
            'initialized': True,
            'trigger_pin': ULTRASONIC_TRIGGER_PIN,
            'echo_pin': ULTRASONIC_ECHO_PIN
        }
        print("Ultrasonic sensor initialized on pins", ULTRASONIC_TRIGGER_PIN, "and", ULTRASONIC_ECHO_PIN)
        return trigger, echo
    except Exception as e:
        GLOBALS['components']['ultrasonic'] = {
            'initialized': False,
            'error': str(e)
        }
        print("Ultrasonic init error:", e)
        return None, None

def check_hardware():
    """Verify all hardware components are properly initialized"""
    hardware_ok = True
    error_components = []
    
    for component, status in GLOBALS['components'].items():
        if not status.get('initialized', False):
            hardware_ok = False
            error_components.append(component)
    
    GLOBALS['hardware_status']['all_ok'] = hardware_ok
    GLOBALS['hardware_status']['error_components'] = error_components
    GLOBALS['hardware_status']['total_components'] = len(GLOBALS['components'])
    GLOBALS['hardware_status']['working_components'] = len(GLOBALS['components']) - len(error_components)
    
    return hardware_ok

def initialize_config():
    """Initialize configuration parameters for ESP8266"""
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
        },
        'communication': {
            'server_ip': '10.47.198.63',
            'server_port': 5000,
            'timeout': 10
        }
    }

def get_system_status():
    """Return comprehensive system status"""
    hardware_ok = GLOBALS['hardware_status'].get('all_ok', False)
    GLOBALS['system_ready'] = hardware_ok
    
    status_report = {
        'system_ready': hardware_ok,
        'hardware': GLOBALS['hardware_status'],
        'components_initialized': {},
        'config': GLOBALS['config'],
        'pins': GLOBALS['pins'],
        'objects_available': {name: obj is not None for name, obj in GLOBALS['objects'].items()}
    }
    
    for component, details in GLOBALS['components'].items():
        status_report['components_initialized'][component] = details.get('initialized', False)
    
    return status_report

def print_system_status():
    """Display system status to console"""
    status = get_system_status()
    print("\n" + "="*50)
    print("ESP8266 MOBILE BOT SYSTEM STATUS")
    print("="*50)
    print(f"System Ready: {'YES' if status['system_ready'] else 'NO'}")
    print(f"Hardware Status: {status['hardware']['working_components']}/{status['hardware']['total_components']} components OK")
    
    if status['hardware']['error_components']:
        print(f"Failed Components: {', '.join(status['hardware']['error_components'])}")
    
    print("Component Status:")
    for component, initialized in status['components_initialized'].items():
        print(f"  {component}: {'OK' if initialized else 'FAILED'}")
    
    print("Pin Configuration:")
    for name, pin in status['pins'].items():
        print(f"  {name}: GPIO{pin}")
    
    print("="*50)

# =============================================================================
# MAIN INITIALIZATION SEQUENCE
# =============================================================================

print("Initializing ESP8266 Mobile Bot System...")

# Initialize configuration
initialize_config()

# Initialize hardware components
initialize_led()
initialize_servo()
initialize_stepper()
initialize_ultrasonic()

# Validate all systems
hardware_ok = check_hardware()

# Display comprehensive status report
print_system_status()

# Provide visual status indication
if GLOBALS['system_ready']:
    print("All systems ready! Starting main application...")
    blink_led(3)  # Triple blink indicates full system readiness
else:
    print("SYSTEM INITIALIZATION FAILED - Check components")
    blink_led(10, 0.1)  # Rapid blink indicates system failure

# Ensure LED is off before main.py takes over
GLOBALS['objects']['led'].value(1)  # Turn off (active low)

print("Boot sequence completed. Control transferring to main.py...")