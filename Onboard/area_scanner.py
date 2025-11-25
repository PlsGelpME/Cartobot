# area_scanner.py - Ultrasonic Sensor Based Area Scanner
# This module controls the servo-mounted ultrasonic sensor to scan the environment
# and collect distance measurements at 10-degree intervals across 180 degrees

from machine import Pin, PWM
import time

class AreaScanner:
    """
    Area Scanner class for environmental mapping
    Combines servo motor control with ultrasonic distance measurement
    to create a 180-degree scan of the surroundings
    """
    
    def __init__(self, servo_pin, trigger_pin, echo_pin):
        """
        Initialize the Area Scanner with hardware pins and parameters
        
        Args:
            servo_pin: GPIO pin connected to servo motor control
            trigger_pin: GPIO pin connected to ultrasonic sensor trigger
            echo_pin: GPIO pin connected to ultrasonic sensor echo
        """
        # Servo motor setup for directional control
        # Servo rotates the ultrasonic sensor to different angles
        self.servo = PWM(Pin(servo_pin))
        self.servo.freq(50)  # Standard 50Hz frequency for servo motors
        
        # HC-SR04 Ultrasonic sensor setup for distance measurement
        # Trigger pin sends ultrasonic pulses, echo pin receives reflections
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        
        # Servo angle limits in microseconds for PWM control
        # These values correspond to 0-180 degree range for most servos
        self.MIN_ANGLE = 2500  # 0 degrees position in microseconds
        self.MAX_ANGLE = 7500  # 180 degrees position in microseconds
        
    def set_servo_angle(self, angle):
        """
        Set servo to specific angle with safety limits and stabilization delay
        
        Args:
            angle: Desired angle in degrees (0-180)
        """
        # Apply safety limits to prevent servo damage
        if angle < 0:
            angle = 0
        if angle > 180:
            angle = 180
            
        # Convert angle to PWM duty cycle in microseconds
        # Linear mapping: 0° = 2500μs, 180° = 7500μs
        pulse_width = int(self.MIN_ANGLE + (angle / 180) * (self.MAX_ANGLE - self.MIN_ANGLE))
        
        # Set PWM duty cycle in nanoseconds (convert microseconds to nanoseconds)
        self.servo.duty_ns(pulse_width * 1000)
        
        # Allow time for servo to reach the target position
        # 300ms provides stable positioning for most servos
        time.sleep_ms(300)
        
    def get_distance(self):
        """
        Get distance measurement from ultrasonic sensor in centimeters
        Uses time-of-flight calculation for accurate distance measurement
        
        Returns:
            float: Distance in centimeters, or -1 for measurement error
        """
        # Send trigger pulse to initiate distance measurement
        self.trigger.value(0)       # Ensure trigger starts low
        time.sleep_us(2)         # Brief 2μs pause
        self.trigger.value(1)      # Send 10μs high pulse
        time.sleep_us(10)
        self.trigger.value(0)       # Return to low state
        
        # Wait for echo pin to go high (start of measurement)
        # Implement timeout to prevent infinite waiting
        timeout = 100000  # Timeout in microseconds (approximately 17 meters range)
        while self.echo.value() == 0:
            if timeout <= 0:
                return -1  # Timeout error - no echo received
            timeout -= 1
            time.sleep_us(1)  # Wait 1μs between checks
            
        # Record start time when echo goes high
        start = time.ticks_us()
        
        # Wait for echo pin to go low (end of measurement)
        # Implement timeout for echo duration
        timeout = 100000
        while self.echo.value() == 1:
            if timeout <= 0:
                return -1  # Timeout error - echo too long
            timeout -= 1
            time.sleep_us(1)
            
        # Record end time when echo goes low
        end = time.ticks_us()
        
        # Calculate distance using time-of-flight
        # Speed of sound = 343 m/s = 0.0343 cm/μs
        # Divide by 2 because sound travels to object and back
        duration = time.ticks_diff(end, start)
        distance = (duration * 0.0343) / 2
        
        return distance
        
    def scan_area(self):
        """
        Perform complete 180-degree environmental scan
        Moves servo in 10-degree increments and collects distance measurements
        
        Returns:
            list: 18 pairs of (angle, distance) representing the environment
        """
        # Initialize list to store scan results
        scan_results = []
        
        # Perform scan from 0 to 180 degrees in 10-degree increments
        # Total of 19 positions (0, 10, 20, ..., 180) but we use 18 for symmetry
        for angle in range(0, 181, 10):
            # Move servo to current scanning angle
            self.set_servo_angle(angle)
            
            # Get distance measurement at current angle
            distance = self.get_distance()
            
            # Store angle and distance pair
            # Format: (angle_in_degrees, distance_in_centimeters)
            scan_results.append((angle, distance))
            
        # Return servo to center position (90 degrees) after scan completion
        # This provides a known starting position for next scan
        self.set_servo_angle(90)
        
        # Return complete scan data
        return scan_results

# Usage example and test function
# This demonstrates how to use the AreaScanner class

# Conditional execution for testing
# Only runs when this file is executed directly, not when imported
if __name__ == "__main__":
    """
    Demonstration of AreaScanner functionality
    Used for testing and verification during development
    """
    # Create scanner instance with specific GPIO pins
    # Adjust pin numbers based on your actual hardware connections
    scanner = AreaScanner(servo_pin=15, trigger_pin=2, echo_pin=3)
    
    # Perform environmental scan
    print("Starting environmental scan...")
    results = scanner.scan_area()
    
    # Display scan results
    print("Scan completed. Results:")
    for angle, distance in results:
        print(f"Angle: {angle}°, Distance: {distance}cm")
    
    # Example of accessing specific data points
    if results:
        print(f"\nFirst measurement: {results[0]}")
        print(f"Center measurement: {results[9]}")  # Approximately 90 degrees
        print(f"Last measurement: {results[-1]}")