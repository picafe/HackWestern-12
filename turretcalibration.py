#!/usr/bin/env python3
"""
Turret Calibration Tool
Helps determine optimal stepper parameters and camera FOV
"""

import board
import digitalio
import time
import atexit


class TurretCalibrator:
    """Interactive calibration for turret system"""
    
    def __init__(self):
        # GPIO Setup
        self.stepX = digitalio.DigitalInOut(board.D23)
        self.dirX = digitalio.DigitalInOut(board.D24)
        self.stepY = digitalio.DigitalInOut(board.D22)
        self.dirY = digitalio.DigitalInOut(board.D25)
        self.lzr = digitalio.DigitalInOut(board.D27)
        
        # Set all pins as outputs
        self.stepX.direction = digitalio.Direction.OUTPUT
        self.dirX.direction = digitalio.Direction.OUTPUT
        self.stepY.direction = digitalio.Direction.OUTPUT
        self.dirY.direction = digitalio.Direction.OUTPUT
        self.lzr.direction = digitalio.Direction.OUTPUT
        
        # Initial states
        self.dirX.value = True
        self.dirY.value = False
        self.lzr.value = True  # Laser on for calibration
        
        self.step_delay = 0.001
        
        atexit.register(self.cleanup)
        
    def cleanup(self):
        """Release GPIO"""
        try:
            self.lzr.value = False
            self.stepX.deinit()
            self.dirX.deinit()
            self.stepY.deinit()
            self.dirY.deinit()
            self.lzr.deinit()
            print("\nGPIO cleaned up")
        except:
            pass
            
    def move_steps(self, axis, steps, direction):
        """Move specified axis by number of steps"""
        if axis == 'X':
            step_pin = self.stepX
            dir_pin = self.dirX
        else:
            step_pin = self.stepY
            dir_pin = self.dirY
            
        dir_pin.value = direction
        
        for _ in range(steps):
            step_pin.value = True
            time.sleep(self.step_delay)
            step_pin.value = False
            time.sleep(self.step_delay)
            
    def calibrate_steps_per_degree(self):
        """Determine steps per degree for each axis"""
        print("\n" + "="*50)
        print("STEP 1: Calibrate Steps Per Degree")
        print("="*50)
        
        results = {}
        
        for axis in ['X', 'Y']:
            print(f"\n--- Calibrating {axis} Axis ---")
            print("1. Position a target at a known distance")
            print("2. Mark the laser's starting position")
            print("3. We'll move the turret a set number of steps")
            print("4. Measure the angular displacement\n")
            
            input("Press Enter when ready...")
            
            # Move 1000 steps
            test_steps = 1000
            print(f"\nMoving {test_steps} steps...")
            self.move_steps(axis, test_steps, True)
            
            print("\nMeasure the angular displacement:")
            print("- Use a protractor, or")
            print("- Calculate: angle = atan(displacement / distance) * 180 / pi")
            
            while True:
                try:
                    angle = float(input(f"\nEnter angular displacement in degrees: "))
                    steps_per_degree = test_steps / angle
                    results[axis] = steps_per_degree
                    print(f"✓ {axis} axis: {steps_per_degree:.2f} steps/degree")
                    break
                except ValueError:
                    print("Invalid input. Please enter a number.")
            
            # Return to start
            print(f"Returning to start position...")
            self.move_steps(axis, test_steps, False)
            time.sleep(1)
        
        return results
        
    def test_range(self, axis, steps_per_degree):
        """Test the full range of motion"""
        print(f"\n--- Testing {axis} Axis Range ---")
        print("Watch the turret movement and note any limits\n")
        
        # Test in both directions
        for direction in [True, False]:
            dir_name = "positive" if direction else "negative"
            print(f"Testing {dir_name} direction...")
            
            steps = int(90 * steps_per_degree)  # Try 90 degrees
            self.move_steps(axis, steps, direction)
            time.sleep(1)
            
            # Return to center
            self.move_steps(axis, steps, not direction)
            time.sleep(1)
            
    def calibrate_fov(self):
        """Guidance for measuring camera FOV"""
        print("\n" + "="*50)
        print("STEP 2: Measure Camera Field of View")
        print("="*50)
        print("\nMethod 1 - Using Known Objects:")
        print("1. Place objects of known width at a known distance")
        print("2. Count how many fit across the camera frame")
        print("3. FOV = 2 * atan((object_width * count) / (2 * distance)) * 180/pi")
        
        print("\nMethod 2 - Using Manufacturer Specs:")
        print("- Check camera datasheet for FOV")
        print("- Typical webcams: 60-80° horizontal")
        
        print("\nMethod 3 - Turret Sweep:")
        print("1. Aim laser at left edge of camera view")
        print("2. Sweep to right edge while counting steps")
        print("3. FOV = steps / steps_per_degree")
        
        input("\nPress Enter to continue...")
        
    def test_tracking_speed(self, steps_per_degree_x):
        """Test different tracking speeds"""
        print("\n" + "="*50)
        print("STEP 3: Test Tracking Speed")
        print("="*50)
        
        delays = [0.0005, 0.001, 0.002, 0.005]
        
        for delay in delays:
            self.step_delay = delay
            print(f"\nTesting delay: {delay*1000:.2f}ms")
            
            # Quick back-and-forth
            steps = int(30 * steps_per_degree_x)
            
            start = time.time()
            self.move_steps('X', steps, True)
            self.move_steps('X', steps, False)
            elapsed = time.time() - start
            
            print(f"Time for 60° sweep: {elapsed:.2f}s")
            print("Observe smoothness and listen for motor strain")
            
            input("Press Enter for next speed...")
            
    def run_full_calibration(self):
        """Run complete calibration sequence"""
        print("\n" + "="*50)
        print("TURRET CALIBRATION WIZARD")
        print("="*50)
        print("\nThis will help you determine:")
        print("- Steps per degree for each axis")
        print("- Maximum safe range of motion")
        print("- Camera field of view")
        print("- Optimal tracking speed")
        
        input("\nPress Enter to start calibration...")
        
        # Step 1: Steps per degree
        steps_per_degree = self.calibrate_steps_per_degree()
        
        # Step 2: Test range
        print("\n" + "="*50)
        print("Testing Range of Motion")
        print("="*50)
        
        for axis in ['X', 'Y']:
            self.test_range(axis, steps_per_degree[axis])
        
        # Step 3: FOV
        self.calibrate_fov()
        
        # Step 4: Speed
        self.test_tracking_speed(steps_per_degree['X'])
        
        # Summary
        print("\n" + "="*50)
        print("CALIBRATION RESULTS")
        print("="*50)
        print(f"\nSteps per degree:")
        print(f"  X axis: {steps_per_degree['X']:.2f}")
        print(f"  Y axis: {steps_per_degree['Y']:.2f}")
        
        print("\nAdd these values to your tracking script:")
        print(f"""
self.stepper = StepperController(
    steps_per_degree_x={steps_per_degree['X']:.2f},
    steps_per_degree_y={steps_per_degree['Y']:.2f},
    max_speed=0.001,  # Adjust based on speed test
    smoothing=5
)
""")
        
    def manual_control(self):
        """Manual joystick-style control for testing"""
        print("\n" + "="*50)
        print("MANUAL CONTROL MODE")
        print("="*50)
        print("\nControls:")
        print("  w/s - Y axis")
        print("  a/d - X axis")
        print("  l   - Toggle laser")
        print("  q   - Quit")
        print("="*50)
        
        try:
            import sys
            import tty
            import termios
            
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            
            try:
                tty.setraw(sys.stdin.fileno())
                
                while True:
                    char = sys.stdin.read(1)
                    
                    if char == 'q':
                        break
                    elif char == 'w':
                        self.move_steps('Y', 50, True)
                    elif char == 's':
                        self.move_steps('Y', 50, False)
                    elif char == 'a':
                        self.move_steps('X', 50, False)
                    elif char == 'd':
                        self.move_steps('X', 50, True)
                    elif char == 'l':
                        self.lzr.value = not self.lzr.value
                        
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
        except ImportError:
            print("Manual control requires termios (Linux only)")
            print("Use full calibration instead")


def main():
    import sys
    
    calibrator = TurretCalibrator()
    
    print("Turret Calibration Tool")
    print("\nSelect mode:")
    print("1. Full calibration wizard")
    print("2. Manual control mode")
    
    choice = input("\nEnter choice (1 or 2): ")
    
    if choice == "1":
        calibrator.run_full_calibration()
    elif choice == "2":
        calibrator.manual_control()
    else:
        print("Invalid choice")
        
    calibrator.cleanup()


if __name__ == "__main__":
    main()