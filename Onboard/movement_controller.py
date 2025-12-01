# movement_controller.py - With Parameterized Pin Configuration
from machine import Pin
import time
import math

class MovementController:
    """
    Differential Drive Movement Controller 
    Optimized for: 4cm wheels, 10.5cm wheel base, 28BYJ-48 steppers
    """
    
    def __init__(self, left_pins, right_pins, steps_per_revolution=2048):
        """
        Initialize with your specific pin configuration
        
        Args:
            left_pins: [IN1, IN2, IN3, IN4] for left motor
            right_pins: [IN1, IN2, IN3, IN4] for right motor
            steps_per_revolution: 2048 for 28BYJ-48
        """
        print(" Initializing Movement Controller")
        print(f"Left Motor Pins: {left_pins}")
        print(f"Right Motor Pins: {right_pins}")
        
        # ULN2003 8-step sequence for smooth operation
        self.step_sequence = [
            [1, 0, 0, 1],
            [1, 0, 0, 0], 
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1]
        ]
        
        # Your specific robot geometry
        self.WHEEL_DIAMETER = 4.0      # cm
        self.WHEEL_BASE = 10.5         # cm
        self.STEPS_PER_REV = steps_per_revolution
        
        # Calculate precise movement constants
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER  # 12.57 cm
        self.STEPS_PER_CM = self.STEPS_PER_REV / self.WHEEL_CIRCUMFERENCE  # 163 steps/cm
        
        print(f" Movement Calibration:")
        print(f"- Wheel diameter: {self.WHEEL_DIAMETER}cm")
        print(f"- Wheel base: {self.WHEEL_BASE}cm") 
        print(f"- Steps per cm: {self.STEPS_PER_CM:.1f}")
        
        # Initialize motor coils with provided pins
        self.left_coils = [Pin(pin, Pin.OUT) for pin in left_pins]
        self.right_coils = [Pin(pin, Pin.OUT) for pin in right_pins]
        
        # Movement control
        self.step_delay_ms = 3  # Slightly slower for stability
        
        # Step counters
        self.left_step_pos = 0
        self.right_step_pos = 0
        
        # Direction correction (will be determined by test)
        self.left_direction_correction = 1
        self.right_direction_correction = 1
        
        # Release motors initially
        self._release_motors()
        print(" Movement Controller Ready")
    
    def _release_motors(self):
        """Turn off all coils to prevent overheating"""
        for coil in self.left_coils + self.right_coils:
            coil.value(0)
    
    def _step_motor(self, coils, step_pos, direction):
        """
        Step a single motor and return new position
        """
        if direction == 1:
            new_pos = (step_pos + 1) % 8
        else:
            new_pos = (step_pos - 1) % 8
            
        # Apply the step pattern
        pattern = self.step_sequence[new_pos]
        for i in range(4):
            coils[i].value(pattern[i])
            
        return new_pos
    
    def move_forward(self, distance_cm):
        """
        Move straight forward/backward
        
        Args:
            distance_cm: Positive=forward, Negative=backward
        """
        if distance_cm == 0:
            return
            
        # Calculate exact steps needed
        steps = int(abs(distance_cm) * self.STEPS_PER_CM)
        direction = 1 if distance_cm > 0 else -1
        
        print(f" Moving {'FORWARD' if distance_cm > 0 else 'BACKWARD'} {abs(distance_cm)}cm → {steps} steps")
        
        # Apply direction corrections
        left_dir = direction * self.left_direction_correction
        right_dir = direction * self.right_direction_correction
        
        # Execute movement
        for i in range(steps):
            self.left_step_pos = self._step_motor(self.left_coils, self.left_step_pos, left_dir)
            self.right_step_pos = self._step_motor(self.right_coils, self.right_step_pos, right_dir)
            time.sleep_ms(self.step_delay_ms)
        
        self._release_motors()
        print("Movement completed")
    
    def rotate(self, angle_degrees):
        """
        Rotate in place by specified angle
        
        Args:
            angle_degrees: Positive=clockwise, Negative=counterclockwise
        """
        if angle_degrees == 0:
            return
            
        # Calculate rotation geometry
        rotation_circumference = math.pi * self.WHEEL_BASE  # 32.99 cm
        wheel_travel_distance = (abs(angle_degrees) / 360.0) * rotation_circumference
        
        # Calculate steps for rotation
        steps = int(wheel_travel_distance * self.STEPS_PER_CM)
        
        print(f" Rotating {'CLOCKWISE' if angle_degrees > 0 else 'COUNTERCLOCKWISE'} {abs(angle_degrees)}° → {steps} steps")
        
        # Set directions for rotation
        if angle_degrees > 0:
            # Clockwise: left forward, right backward
            left_dir = 1 * self.left_direction_correction
            right_dir = -1 * self.right_direction_correction
        else:
            # Counterclockwise: left backward, right forward
            left_dir = -1 * self.left_direction_correction  
            right_dir = 1 * self.right_direction_correction
        
        # Execute rotation
        for i in range(steps):
            self.left_step_pos = self._step_motor(self.left_coils, self.left_step_pos, left_dir)
            self.right_step_pos = self._step_motor(self.right_coils, self.right_step_pos, right_dir)
            time.sleep_ms(self.step_delay_ms)
        
        self._release_motors()
        print(" Rotation completed")
    
    def move_with_angle(self, distance_cm, angle_degrees):
        """
        Combined move: rotate then move straight
        """
        if distance_cm == 0 and angle_degrees == 0:
            return
            
        # First rotate to target angle
        if angle_degrees != 0:
            print(f" Phase 1: Rotating to {angle_degrees}°")
            self.rotate(angle_degrees)
            time.sleep_ms(500)
        
        # Then move straight
        if distance_cm != 0:
            print(f" Phase 2: Moving {distance_cm}cm")
            self.move_forward(distance_cm)
    
    def test_motor_directions(self):
        """
        Diagnostic test to determine motor directions
        Run this first to see which way your bot moves!
        """
        print("\n MOTOR DIRECTION TEST")
        print("======================")
        
        # Test forward movement
        print("Testing forward movement...")
        self.move_forward(5)
        
        print("\n OBSERVATION GUIDE:")
        print(" MOVED STRAIGHT: Motors are correct!")
        print(" TURNED LEFT: Right motor reversed → run: mover.fix_motor_direction(1, -1)")
        print("TURNED RIGHT: Left motor reversed → run: mover.fix_motor_direction(-1, 1)")
    
    def fix_motor_direction(self, left_correction=1, right_correction=1):
        """
        Fix motor directions if they're mounted opposite
        
        Args:
            left_correction: 1=normal, -1=reversed
            right_correction: 1=normal, -1=reversed
        """
        self.left_direction_correction = left_correction
        self.right_direction_correction = right_correction
        print(f"️ Direction correction applied: Left={left_correction}, Right={right_correction}")
    
    def set_speed(self, speed_factor):
        """
        Adjust movement speed
        """
        base_delay = 3  # ms
        self.step_delay_ms = max(1, int(base_delay / speed_factor))
        print(f" Speed set to {speed_factor}x → delay: {self.step_delay_ms}ms")

# Test function with your specific pins
def main():
    """Test movement with your exact pin configuration"""
    
    # YOUR PIN CONFIGURATION
    left_pins = [19, 18, 5, 17]    # Left motor [IN1, IN2, IN3, IN4]
    right_pins = [15, 2, 4, 16]  # Right motor [IN1, IN2, IN3, IN4]
    
    # Create movement controller
    mover = MovementController(left_pins, right_pins)
    
    print("\n TESTING YOUR BOT MOVEMENTS")
    print("============================")
    
    # First, run the direction test
    mover.test_motor_directions()
    
    # If it didn't move straight, uncomment and run the appropriate fix:
    # mover.fix_motor_direction(1, -1)  # If it turned LEFT
    # mover.fix_motor_direction(-1, 1)  # If it turned RIGHT
    
    # Then test the movements
    print("\n Testing rotation...")
    mover.rotate(90)
    time.sleep(1)
    
    print("\n Testing forward movement...")
    mover.move_forward(10)
    time.sleep(1)
    
    print("\n All tests completed!")

if __name__ == "__main__":
    main()