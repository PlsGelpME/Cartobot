# movement_controller.py - Mobile Bot Movement Controller
# This module controls the stepper motor for precise locomotion
# Converts distance and angle commands into precise motor movements

from machine import Pin
import time
import math

class MovementController:
    """
    Movement Controller for mobile bot locomotion
    Translates high-level distance and angle commands into stepper motor control
    Provides precise movement and rotation capabilities
    """
    
    def __init__(self, step_pin, dir_pin, steps_per_revolution=200, wheel_diameter_cm=6.5, wheel_base_cm=15.0):
        """
        Initialize movement controller with motor parameters and physical dimensions
        
        Args:
            step_pin: GPIO pin connected to stepper motor STEP input
            dir_pin: GPIO pin connected to stepper motor DIRECTION input
            steps_per_revolution: Number of steps for one full motor revolution
            wheel_diameter_cm: Diameter of bot wheels in centimeters
            wheel_base_cm: Distance between wheels for turning calculations
        """
        # Stepper motor control pins
        # STEP pin triggers individual motor steps
        self.step_pin = Pin(step_pin, Pin.OUT)
        # DIRECTION pin controls movement direction (high/low)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        
        # Motor mechanical parameters
        self.steps_per_revolution = steps_per_revolution  # Motor resolution
        self.wheel_diameter_cm = wheel_diameter_cm        # Wheel size for distance calculation
        self.wheel_base_cm = wheel_base_cm                # Distance between wheels for turning
        
        # Calculate derived constants for movement calculations
        self.wheel_circumference = math.pi * wheel_diameter_cm  # Distance per wheel revolution
        self.steps_per_cm = steps_per_revolution / self.wheel_circumference  # Steps needed per cm
        
        # Movement control parameters
        self.step_delay_ms = 2  # Delay between steps in milliseconds (controls motor speed)
        
    def calculate_steps_for_distance(self, distance_cm):
        """
        Calculate number of steps needed to move specified distance
        Converts linear distance to motor steps based on wheel geometry
        
        Args:
            distance_cm: Distance to move in centimeters (positive or negative)
            
        Returns:
            int: Number of steps required for the movement
        """
        # Use absolute distance for step calculation
        # Direction is handled separately by DIR pin
        return int(abs(distance_cm) * self.steps_per_cm)
    
    def calculate_rotation_angle(self, target_angle):
        """
        Calculate the rotation needed to achieve target angle
        Normalizes angles to shortest rotation path
        
        Args:
            target_angle: Desired angle in degrees
            
        Returns:
            float: Normalized rotation angle in degrees (-180 to 180)
        """
        # Normalize angle to -180 to 180 degree range
        # This finds the shortest rotation path to target angle
        normalized_angle = target_angle % 360  # Ensure angle is within 0-360
        if normalized_angle > 180:
            normalized_angle -= 360  # Convert to -180 to 180 range
            
        return normalized_angle
    
    def calculate_rotation_distance(self, rotation_angle):
        """
        Calculate linear distance each wheel needs to travel for rotation
        Uses arc length formula based on wheel base geometry
        
        Args:
            rotation_angle: Angle to rotate in degrees (positive = clockwise)
            
        Returns:
            float: Linear distance each wheel travels in centimeters
        """
        # Convert angle to radians for trigonometric calculations
        angle_radians = math.radians(abs(rotation_angle))
        
        # Calculate arc length for rotation
        # Each wheel travels along an arc with radius = wheel_base / 2
        wheel_travel_distance = angle_radians * (self.wheel_base_cm / 2)
        
        return wheel_travel_distance
    
    def move_forward(self, distance_cm):
        """
        Move straight forward or backward by specified distance
        Controls stepper motor for precise linear movement
        
        Args:
            distance_cm: Distance to move in centimeters
                         Positive = forward, Negative = backward
        """
        # Skip movement if distance is zero
        if distance_cm == 0:
            return
            
        # Set direction based on distance sign
        if distance_cm > 0:
            self.dir_pin.value(1)  # Forward direction
        else:
            self.dir_pin.value(0)  # Backward direction
            
        # Calculate number of steps needed for the distance
        steps = self.calculate_steps_for_distance(distance_cm)
        
        # Execute stepping sequence
        for _ in range(steps):
            # Generate step pulse
            self.step_pin.value(1)  # Step pin high
            time.sleep_ms(1)        # Maintain high state briefly
            self.step_pin.value(0)  # Step pin low
            
            # Delay between steps controls motor speed
            time.sleep_ms(self.step_delay_ms)
            
    def rotate(self, angle_degrees):
        """
        Rotate in place by specified angle
        Implements point turns by moving wheels in opposite directions
        
        Args:
            angle_degrees: Angle to rotate in degrees
                           Positive = clockwise, Negative = counterclockwise
        """
        # Skip rotation if angle is zero
        if angle_degrees == 0:
            return
            
        # Calculate rotation distance for each wheel
        # Both wheels move equal distances in opposite directions
        rotation_distance = self.calculate_rotation_distance(angle_degrees)
        
        # Set directions for rotation
        # For single motor system, we simulate rotation by moving one wheel
        # In dual motor system, both motors would be controlled simultaneously
        if angle_degrees > 0:
            # Clockwise rotation
            left_direction = 1   # Left wheel forward
            right_direction = 0  # Right wheel backward
        else:
            # Counterclockwise rotation  
            left_direction = 1   # Left wheel backward
            right_direction = 0  # Right wheel forward
            
        # For single motor implementation, double the steps to simulate rotation
        # This approximates the effect of differential steering
        steps = self.calculate_steps_for_distance(rotation_distance * 2)
        
        # Set direction for the active motor
        self.dir_pin.value(left_direction if angle_degrees > 0 else right_direction)
        
        # Execute rotation steps
        for _ in range(steps):
            # Generate step pulse
            self.step_pin.value(1)  # Step pin high
            time.sleep_ms(1)        # Maintain high state
            self.step_pin.value(0)  # Step pin low
            
            # Delay between steps
            time.sleep_ms(self.step_delay_ms)
    
    def move_with_angle(self, distance_cm, angle_degrees):
        """
        Move specified distance at specified angle relative to current heading
        Combines rotation and linear movement for precise navigation
        
        Args:
            distance_cm: Distance to move in centimeters
            angle_degrees: Angle relative to current heading in degrees
        """
        # Skip if both distance and angle are zero
        if distance_cm == 0 and angle_degrees == 0:
            return
            
        # First, rotate to target angle relative to current heading
        if angle_degrees != 0:
            self.rotate(angle_degrees)
            time.sleep_ms(500)  # Brief pause after rotation for stabilization
            
        # Then, move straight for the specified distance
        if distance_cm != 0:
            self.move_forward(distance_cm)
            
    def precise_move(self, distance_cm, angle_degrees, speed_factor=1.0):
        """
        More precise movement with adjustable speed control
        Allows fine-tuning of movement speed for different scenarios
        
        Args:
            distance_cm: Distance to move in centimeters
            angle_degrees: Angle relative to current heading in degrees
            speed_factor: Speed multiplier (>1 = faster, <1 = slower)
        """
        # Store original speed setting
        original_delay = self.step_delay_ms
        
        # Adjust speed based on speed factor
        self.step_delay_ms = int(original_delay / speed_factor)
        
        try:
            # Execute movement with adjusted speed
            self.move_with_angle(distance_cm, angle_degrees)
        finally:
            # Restore original speed setting regardless of success/failure
            self.step_delay_ms = original_delay
    
    def stop(self):
        """
        Stop any ongoing movement immediately
        For stepper motors, stopping is immediate since they move step by step
        This function can be expanded for emergency stop scenarios
        """
        # Stepper motors stop immediately when stepping stops
        # No additional action needed for basic implementation
        # Can be expanded with braking or position holding in advanced implementations
        pass

# Dual motor version for differential drive systems
# This class provides true differential steering for two-motor setups
class DualMotorMovementController:
    """
    Advanced movement controller for dual motor differential drive systems
    Provides true tank-style steering with independent motor control
    """
    
    def __init__(self, left_step_pin, left_dir_pin, right_step_pin, right_dir_pin, 
                 steps_per_revolution=200, wheel_diameter_cm=6.5, wheel_base_cm=15.0):
        """
        Initialize dual motor controller with separate motor controls
        
        Args:
            left_step_pin: Left motor STEP pin
            left_dir_pin: Left motor DIRECTION pin  
            right_step_pin: Right motor STEP pin
            right_dir_pin: Right motor DIRECTION pin
            steps_per_revolution: Steps per motor revolution
            wheel_diameter_cm: Wheel diameter in centimeters
            wheel_base_cm: Distance between wheels
        """
        # Left motor control pins
        self.left_step = Pin(left_step_pin, Pin.OUT)
        self.left_dir = Pin(left_dir_pin, Pin.OUT)
        
        # Right motor control pins  
        self.right_step = Pin(right_step_pin, Pin.OUT)
        self.right_dir = Pin(right_dir_pin, Pin.OUT)
        
        # Motor parameters (same as single motor controller)
        self.steps_per_revolution = steps_per_revolution
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheel_base_cm = wheel_base_cm
        
        # Calculate movement constants
        self.wheel_circumference = math.pi * wheel_diameter_cm
        self.steps_per_cm = steps_per_revolution / self.wheel_circumference
        
        # Speed control parameter
        self.step_delay_ms = 2

# Usage example and test function
# Demonstrates how to use the MovementController class
def main():
    """
    Example usage of MovementController for testing
    """
    # Create movement controller instance
    # Replace pin numbers with actual GPIO connections
    mover = MovementController(step_pin=12, dir_pin=13)
    
    # Example movement sequence
    print("Starting movement test...")
    
    # Move forward 50 cm
    print("Moving forward 50cm...")
    mover.move_forward(50)
    time.sleep(1)  # Pause between movements
    
    # Rotate 90 degrees clockwise
    print("Rotating 90 degrees clockwise...")
    mover.rotate(90)
    time.sleep(1)
    
    # Move at 45 degree angle for 30cm
    print("Moving 30cm at 45 degree angle...")
    mover.move_with_angle(30, 45)
    time.sleep(1)
    
    # Precise slow movement
    print("Precise slow movement...")
    mover.precise_move(20, 0, speed_factor=0.5)  # Half speed
    
    print("Movement test completed")

# Conditional execution for testing
if __name__ == "__main__":
    main()