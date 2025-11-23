#!/usr/bin/env python3
"""
Dual Camera Living Object Detection System with Turret Tracking
Uses RGB camera + IR camera for detection and stepper motors with DRV8825 drivers to track living objects
"""

import cv2
import numpy as np
from threading import Thread, Lock
import time
import argparse
import board
import digitalio
import atexit
from collections import deque


class StepperController:
    """Controls X/Y stepper motors for turret positioning"""
    
    def __init__(self, steps_per_degree_x=5.55, steps_per_degree_y=5.55, 
                 max_speed=0.0005, smoothing=5):
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
        self.lzr.value = False  # Laser off initially
        
        # Positioning parameters
        self.steps_per_degree_x = steps_per_degree_x
        self.steps_per_degree_y = steps_per_degree_y
        self.step_delay = max_speed  # Seconds between steps
        
        # Current position (in steps from center)
        self.current_x = 0
        self.current_y = 0
        
        # Target smoothing
        self.target_buffer_x = deque(maxlen=smoothing)
        self.target_buffer_y = deque(maxlen=smoothing)
        
        # Limits (in degrees from center)
        self.max_x_angle = 90
        self.max_y_angle = 45
        
        # Thread control
        self.stopped = False
        self.target_x = 0
        self.target_y = 0
        self.lock = Lock()
        self.tracking_active = False
        
        # Register cleanup
        atexit.register(self.cleanup)
        
        print("Stepper controller initialized")
        
    def cleanup(self):
        """Release GPIO pins on exit"""
        try:
            self.lzr.value = False
            self.stepX.deinit()
            self.dirX.deinit()
            self.stepY.deinit()
            self.dirY.deinit()
            self.lzr.deinit()
            print("GPIO pins released")
        except:
            pass
            
    def set_laser(self, state):
        """Turn laser on/off"""
        self.lzr.value = state
        
    def move_motor(self, step_pin, dir_pin, steps, direction):
        """Move a single motor by specified steps"""
        if steps == 0:
            return
            
        # Set direction
        dir_pin.value = direction
        
        # Step the motor
        for _ in range(abs(steps)):
            step_pin.value = True
            time.sleep(self.step_delay)
            step_pin.value = False
            time.sleep(self.step_delay)
            
    def set_target(self, x_angle, y_angle, enable_tracking=True):
        """Set target position in degrees from center"""
        with self.lock:
            # Clamp to limits
            x_angle = max(-self.max_x_angle, min(self.max_x_angle, x_angle))
            y_angle = max(-self.max_y_angle, min(self.max_y_angle, y_angle))
            
            # Add to smoothing buffer
            self.target_buffer_x.append(x_angle)
            self.target_buffer_y.append(y_angle)
            
            # Use smoothed target
            self.target_x = sum(self.target_buffer_x) / len(self.target_buffer_x)
            self.target_y = sum(self.target_buffer_y) / len(self.target_buffer_y)
            self.tracking_active = enable_tracking
            
    def get_position(self):
        """Get current position in degrees"""
        with self.lock:
            x_deg = self.current_x / self.steps_per_degree_x
            y_deg = self.current_y / self.steps_per_degree_y
            return x_deg, y_deg
            
    def update_position(self):
        """Move towards target position (called in loop)"""
        with self.lock:
            if not self.tracking_active:
                return
                
            # Convert target angles to steps
            target_steps_x = int(self.target_x * self.steps_per_degree_x)
            target_steps_y = int(self.target_y * self.steps_per_degree_y)
            
            # Calculate steps needed
            delta_x = target_steps_x - self.current_x
            delta_y = target_steps_y - self.current_y
            
        # Move if needed (outside lock to avoid blocking)
        if abs(delta_x) > 0:
            direction = delta_x > 0
            steps = min(abs(delta_x), 10)  # Limit steps per update for smoother motion
            self.move_motor(self.stepX, self.dirX, steps, direction)
            with self.lock:
                self.current_x += steps if direction else -steps
                
        if abs(delta_y) > 0:
            direction = delta_y > 0
            steps = min(abs(delta_y), 10)
            self.move_motor(self.stepY, self.dirY, steps, direction)
            with self.lock:
                self.current_y += steps if direction else -steps
                
    def center(self):
        """Return turret to center position"""
        print("Centering turret...")
        self.set_target(0, 0, True)
        
        # Wait for centering to complete
        while True:
            x, y = self.get_position()
            if abs(x) < 0.5 and abs(y) < 0.5:
                break
            self.update_position()
            time.sleep(0.01)
            
        print("Turret centered")


class CameraStream:
    """Threaded camera capture for better performance"""
    def __init__(self, src, name, width=None, height=None):
        self.stream = cv2.VideoCapture(src)
        self.name = name
        
        if width and height:
            self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.lock = Lock()
        
    def start(self):
        Thread(target=self.update, daemon=True).start()
        return self
        
    def update(self):
        while not self.stopped:
            grabbed, frame = self.stream.read()
            with self.lock:
                self.grabbed = grabbed
                self.frame = frame
                
    def read(self):
        with self.lock:
            return self.grabbed, self.frame.copy() if self.frame is not None else None
            
    def stop(self):
        self.stopped = True
        self.stream.release()


class TurretTracker:
    """Integrates detection with turret tracking"""
    
    def __init__(self, rgb_idx=0, ir_idx=1, process_width=640, 
                 fov_horizontal=60, fov_vertical=45):
        # Camera setup
        self.rgb_stream = CameraStream(rgb_idx, "RGB", 1920, 1080).start()
        self.ir_stream = CameraStream(ir_idx, "IR", 256, 192).start()
        
        # Stepper controller
        self.stepper = StepperController(
            steps_per_degree_x=5.55,  # Adjust based on your stepper motor
            steps_per_degree_y=5.55,
            max_speed=0.0005,
            smoothing=5
        )
        
        # Processing parameters
        self.process_width = process_width
        self.min_area = 500
        self.detection_threshold = 0.3
        
        # Field of view (degrees)
        self.fov_h = fov_horizontal
        self.fov_v = fov_vertical
        
        # Background subtractors
        self.rgb_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=50, detectShadows=True
        )
        self.ir_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=300, varThreshold=30, detectShadows=False
        )
        
        # Tracking state
        self.last_detection_time = 0
        self.tracking_timeout = 2.0  # Seconds before laser turns off
        
        # Warm-up
        time.sleep(1.0)
        self.stepper.center()
        
    def preprocess_frame(self, frame, target_width):
        """Resize and prepare frame for processing"""
        if frame is None:
            return None
        h, w = frame.shape[:2]
        aspect = h / w
        target_height = int(target_width * aspect)
        resized = cv2.resize(frame, (target_width, target_height))
        return resized
        
    def detect_objects(self, frame, bg_subtractor, blur_size=5):
        """Detect objects using background subtraction"""
        if frame is None:
            return [], None
            
        fg_mask = bg_subtractor.apply(frame)
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (blur_size, blur_size))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                detections.append((x, y, w, h, area))
                
        return detections, fg_mask
        
    def normalize_detections(self, detections, frame_shape):
        """Convert detections to normalized coordinates (0-1)"""
        h, w = frame_shape[:2]
        normalized = []
        for x, y, bw, bh, area in detections:
            norm_x = x / w
            norm_y = y / h
            norm_w = bw / w
            norm_h = bh / h
            normalized.append((norm_x, norm_y, norm_w, norm_h, area))
        return normalized
        
    def calculate_iou(self, box1, box2):
        """Calculate Intersection over Union"""
        x1, y1, w1, h1 = box1[:4]
        x2, y2, w2, h2 = box2[:4]
        
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
            
        intersection = (x_right - x_left) * (y_bottom - y_top)
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0
        
    def cross_validate_detections(self, rgb_detections, ir_detections):
        """Find detections in both cameras"""
        validated = []
        
        for rgb_det in rgb_detections:
            for ir_det in ir_detections:
                iou = self.calculate_iou(rgb_det, ir_det)
                if iou > self.detection_threshold:
                    validated.append({
                        'box': rgb_det[:4],
                        'area': rgb_det[4],
                        'confidence': iou
                    })
                    break
                    
        return validated
        
    def calculate_target_angles(self, box, frame_shape):
        """Convert bounding box center to turret angles"""
        x, y, w, h = box
        h_frame, w_frame = frame_shape[:2]
        
        # Calculate centroid
        center_x = x + w / 2
        center_y = y + h / 2
        
        # Normalize to -0.5 to 0.5 (center is 0)
        norm_x = (center_x / w_frame) - 0.5
        norm_y = (center_y / h_frame) - 0.5
        
        # Convert to angles
        angle_x = norm_x * self.fov_h
        angle_y = -norm_y * self.fov_v  # Invert Y (screen coords vs real world)
        
        return angle_x, angle_y
        
    def select_primary_target(self, validated):
        """Select which target to track (largest by area)"""
        if not validated:
            return None
            
        # Sort by area and return largest
        sorted_targets = sorted(validated, key=lambda x: x['area'], reverse=True)
        return sorted_targets[0]
        
    def denormalize_box(self, norm_box, frame_shape):
        """Convert normalized box to pixel coordinates"""
        h, w = frame_shape[:2]
        x, y, bw, bh = norm_box
        return int(x * w), int(y * h), int(bw * w), int(bh * h)
        
    def draw_detections(self, frame, detections, color=(0, 255, 0)):
        """Draw bounding boxes"""
        output = frame.copy()
        for det in detections:
            x, y, w, h = det[:4]
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
        return output
        
    def draw_crosshair(self, frame, box, color=(0, 255, 0)):
        """Draw tracking crosshair"""
        x, y, w, h = box
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        
        cv2.drawMarker(frame, (center_x, center_y), color, 
                      cv2.MARKER_CROSS, 20, 2)
        cv2.circle(frame, (center_x, center_y), 10, color, 2)
        
        return frame
        
    def run(self, display=True):
        """Main tracking loop"""
        print("Starting turret tracking system...")
        print("Press 'q' to quit, 'r' to reset background, 'c' to center turret")
        
        try:
            while True:
                loop_start = time.time()
                
                # Read frames
                rgb_ok, rgb_frame = self.rgb_stream.read()
                ir_ok, ir_frame = self.ir_stream.read()
                
                if not rgb_ok or not ir_ok:
                    print("Error reading frames")
                    break
                
                # Preprocess
                rgb_processed = self.preprocess_frame(rgb_frame, self.process_width)
                ir_processed = self.preprocess_frame(ir_frame, self.process_width)
                
                # Detect objects
                rgb_detections, _ = self.detect_objects(rgb_processed, self.rgb_bg_subtractor)
                ir_detections, _ = self.detect_objects(ir_processed, self.ir_bg_subtractor)
                
                # Normalize and cross-validate
                rgb_norm = self.normalize_detections(rgb_detections, rgb_processed.shape)
                ir_norm = self.normalize_detections(ir_detections, ir_processed.shape)
                validated = self.cross_validate_detections(rgb_norm, ir_norm)
                
                # Select and track target
                target = self.select_primary_target(validated)
                
                if target:
                    # Calculate angles to target
                    angle_x, angle_y = self.calculate_target_angles(
                        target['box'], rgb_processed.shape
                    )
                    
                    # Update turret position
                    self.stepper.set_target(angle_x, angle_y, enable_tracking=True)
                    self.stepper.set_laser(True)
                    self.last_detection_time = time.time()
                    
                else:
                    # No target - check timeout
                    if time.time() - self.last_detection_time > self.tracking_timeout:
                        self.stepper.set_laser(False)
                        self.stepper.set_target(0, 0, enable_tracking=False)
                
                # Update stepper position
                self.stepper.update_position()
                
                # Visualization
                if display:
                    rgb_vis = self.draw_detections(rgb_processed, rgb_detections, (255, 0, 0))
                    
                    if target:
                        # Denormalize target box for display
                        target_box = self.denormalize_box(target['box'], rgb_processed.shape)
                        rgb_vis = cv2.rectangle(
                            rgb_vis, 
                            (target_box[0], target_box[1]),
                            (target_box[0] + target_box[2], target_box[1] + target_box[3]),
                            (0, 255, 0), 3
                        )
                        rgb_vis = self.draw_crosshair(rgb_vis, target_box, (0, 255, 0))
                    
                    # Draw info
                    pos_x, pos_y = self.stepper.get_position()
                    info = [
                        f"Targets: {len(validated)}",
                        f"Position: X:{pos_x:.1f}° Y:{pos_y:.1f}°",
                        f"Laser: {'ON' if self.stepper.lzr.value else 'OFF'}",
                        f"FPS: {1/(time.time()-loop_start):.1f}"
                    ]
                    
                    y_offset = 30
                    for line in info:
                        cv2.putText(rgb_vis, line, (10, y_offset),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        y_offset += 25
                    
                    cv2.imshow('Turret Tracker', rgb_vis)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        self.rgb_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                            history=500, varThreshold=50, detectShadows=True
                        )
                        self.ir_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                            history=300, varThreshold=30, detectShadows=False
                        )
                        print("Background models reset")
                    elif key == ord('c'):
                        self.stepper.center()
                
        finally:
            self.cleanup()
            cv2.destroyAllWindows()
            
    def cleanup(self):
        """Release all resources"""
        print("Shutting down...")
        self.stepper.set_laser(False)
        self.stepper.center()
        self.rgb_stream.stop()
        self.ir_stream.stop()
        self.stepper.cleanup()


def main():
    parser = argparse.ArgumentParser(description='Turret Tracking System')
    parser.add_argument('--rgb-camera', type=int, default=0)
    parser.add_argument('--ir-camera', type=int, default=1)
    parser.add_argument('--process-width', type=int, default=640)
    parser.add_argument('--fov-horizontal', type=float, default=60.0,
                       help='Camera horizontal field of view in degrees')
    parser.add_argument('--fov-vertical', type=float, default=45.0,
                       help='Camera vertical field of view in degrees')
    parser.add_argument('--no-display', action='store_true')
    
    args = parser.parse_args()
    
    tracker = TurretTracker(
        rgb_idx=args.rgb_camera,
        ir_idx=args.ir_camera,
        process_width=args.process_width,
        fov_horizontal=args.fov_horizontal,
        fov_vertical=args.fov_vertical
    )
    
    tracker.run(display=not args.no_display)


if __name__ == "__main__":
    main()