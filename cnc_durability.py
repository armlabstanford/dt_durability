#!/usr/bin/env python3

import serial
import time
import re
import os
import glob
import simpleaudio as sa
from pynput import keyboard
import numpy as np
import platform
import cv2
import csv
import threading
from skimage.metrics import structural_similarity as ssim

import serial.tools.list_ports
import NetFT

##############################################################################
# CONFIGURATION
##############################################################################
# Camera Settings
CAMERA_DEVICE_ID = 4       # /dev/video4 (Change according to your environment)
EXPOSURE_VALUE = 50        # Manual Exposure Value
WB_TEMP = 6500             # White Balance Temperature
FRAME_WIDTH = 1600
FRAME_HEIGHT = 1200

# ATI Sensor Settings
SENSOR_IP = "192.168.2.1"  # ATI Sensor IP
COUNTS_PER_FORCE = 1000000.0
COUNTS_PER_TORQUE = 1000000.0
##############################################################################


class ArduinoController:
    """Detect Arduino (skip GRBL/CNC ports) and send step-motor commands."""
    _KNOWN_VIDS = {0x2341, 0x1A86, 0x10C4, 0x1B4F}
    _CNC_TAGS = ("grbl", "cnc")
    _PREFERRED_PATH = "/dev/serial/by-id/"

    def __init__(self, baudrate: int = 115200, timeout: float = 1):
        self.port = self._find_port()
        if not self.port:
            self.serial_port = None
            print("(Rotation unavailable — no Arduino detected.)")
            return
        self.serial_port = serial.Serial(self.port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)
        print(f"Arduino connected on {self.port}")

    def _find_port(self):
        # Prefer using by-id path for more reliable detection
        if os.path.exists(self._PREFERRED_PATH):
            for fname in os.listdir(self._PREFERRED_PATH):
                path = os.path.join(self._PREFERRED_PATH, fname)
                if "arduino" in fname.lower() and os.path.exists(path):
                    return os.path.realpath(path)

        # Fallback: Use VID/manufacturer/descriptions
        for p in serial.tools.list_ports.comports():
            desc = p.description.lower()
            manu = (p.manufacturer or "").lower()
            if any(tag in desc for tag in self._CNC_TAGS):
                continue
            if p.vid in self._KNOWN_VIDS or "arduino" in desc or "arduino" in manu:
                return p.device

        return None

    def send_step(self, direction: str, deg: float, rpm: int = 120):
        if not self.serial_port:
            return
        cmd = f"{direction.upper()} {deg} {rpm}\n"
        self.serial_port.write(cmd.encode())
        print(f"[Arduino] → {direction.upper()} {deg}°")

    def close(self):
        if self.serial_port:
            self.serial_port.close()


class DurabilityTester:
    """Handles camera and force sensor for durability testing"""
    
    def __init__(self):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.save_root = os.path.join(self.base_dir, 'durability_test')
        
        # State flags
        self.active = False
        self.initial_image = None
        self.csv_file = None
        self.csv_writer = None
        self.trial_dir = None
        
        # Initialize components (lazy loading)
        self.sensor = None
        self.cap = None

    def initialize(self):
        """Initialize sensor and camera for testing"""
        if self.active:
            print("Durability tester already initialized.")
            return
            
        print("\n[Durability Test] Initializing...")
        
        # Initialize ATI Sensor
        self.init_sensor()
        
        # Initialize Camera
        self.init_camera()
        
        # Set data save path
        self.trial_dir = self.get_next_trial_dir()
        self.csv_path = os.path.join(self.trial_dir, 'data_log.csv')
        print(f"[Info] Data will be saved to: {self.trial_dir}")
        
        # Initialize CSV file
        self.init_csv()
        
        # Capture Initial Image
        ret, frame = self.cap.read()
        if ret:
            self.initial_image = frame.copy()
            cv2.imwrite(os.path.join(self.trial_dir, "initial_image.png"), self.initial_image)
            print("[Info] Initial reference image captured.")
        else:
            print("[Error] Could not capture initial image.")
            return False
            
        self.active = True
        print("[Durability Test] Ready!\n")
        return True

    def get_next_trial_dir(self):
        """Creates a new trial folder (trial_1, trial_2, ...)"""
        if not os.path.exists(self.save_root):
            os.makedirs(self.save_root)
        
        i = 1
        while True:
            trial_dir = os.path.join(self.save_root, f"trial_{i}")
            if not os.path.exists(trial_dir):
                os.makedirs(trial_dir)
                return trial_dir
            i += 1

    def init_csv(self):
        """Initialize CSV file with headers"""
        headers = [
            'timestamp', 'image_filename', 
            'fx', 'fy', 'fz', 'tx', 'ty', 'tz',
            'mse', 'psnr', 'ssim', 'l1_error'
        ]
        
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(headers)
        self.csv_file.flush()

    def init_sensor(self):
        """Initialize ATI Force/Torque Sensor"""
        print(f"Connecting to ATI Sensor at {SENSOR_IP}...")
        try:
            self.sensor = NetFT.Sensor(SENSOR_IP)
            self.sensor.tare()
            print("ATI Sensor connected and tared.")
            self.sensor.startStreaming()
        except Exception as e:
            print(f"Error connecting to sensor: {e}")
            self.sensor = None

    def init_camera(self):
        """Initialize Camera with v4l2-ctl settings"""
        print(f"Opening camera /dev/video{CAMERA_DEVICE_ID}...")
        self.cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
        
        # Set Resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Manual settings using v4l2-ctl command
        self.set_manual_exposure(CAMERA_DEVICE_ID, EXPOSURE_VALUE)
        self.set_manual_wb(CAMERA_DEVICE_ID, WB_TEMP)
        
        # Wait briefly for camera stabilization and clear buffer
        time.sleep(2)
        for _ in range(10):
            self.cap.read()
            
        print("Camera initialized with manual settings.")

    def set_manual_exposure(self, video_id, exposure_time):
        """Set manual exposure using v4l2-ctl"""
        commands = [
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=3",  # Auto mode (to reset)
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=1",  # Manual mode
            f"v4l2-ctl --device /dev/video{video_id} -c exposure_time_absolute={exposure_time}"
        ]
        for cmd in commands:
            os.system(cmd)

    def set_manual_wb(self, video_id, wb_temp):
        """Set manual white balance using v4l2-ctl"""
        commands = [
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=1",  # Toggle to ensure reset
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_temperature={wb_temp}"
        ]
        for cmd in commands:
            os.system(cmd)

    def compute_metrics(self, img1, img2):
        """Compute MSE, PSNR, SSIM, L1 between two images"""
        g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # MSE
        diff = (g1.astype(np.float32) - g2.astype(np.float32)) ** 2
        mse = np.mean(diff)
        
        # PSNR
        if mse == 0:
            psnr = 100.0
        else:
            psnr = 10 * np.log10(255.0**2 / mse)
            
        # L1 Error
        l1_error = np.mean(np.abs(g1.astype(np.float32) - g2.astype(np.float32)))
        
        # SSIM
        score, _ = ssim(g1, g2, full=True)
        
        return mse, psnr, score, l1_error

    def get_force_reading(self):
        """Get scaled force reading from sensor"""
        if not self.sensor:
            return [0.0]*6
            
        raw_ft = self.sensor.measurement()
        if raw_ft:
            fx = raw_ft[0] / COUNTS_PER_FORCE
            fy = raw_ft[1] / COUNTS_PER_FORCE
            fz = raw_ft[2] / COUNTS_PER_FORCE
            tx = raw_ft[3] / COUNTS_PER_TORQUE
            ty = raw_ft[4] / COUNTS_PER_TORQUE
            tz = raw_ft[5] / COUNTS_PER_TORQUE
            return [fx, fy, fz, tx, ty, tz]
        return [0.0]*6

    def save_data_point(self):
        """Save current image and force data with metrics"""
        if not self.active:
            print("[Warning] Durability tester not initialized. Use 'dt_start' first.")
            return
            
        ret, frame = self.cap.read()
        if not ret:
            print("[Error] Could not capture image.")
            return
            
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # Compute Metrics against initial image
        mse, psnr, ssim_val, l1 = self.compute_metrics(frame, self.initial_image)
        
        # Get force reading
        ft = self.get_force_reading()
        
        # Save Image
        img_filename = f"image_{timestamp}.png"
        img_path = os.path.join(self.trial_dir, img_filename)
        cv2.imwrite(img_path, frame)
        
        # Save to CSV
        row = [
            timestamp, img_filename,
            f"{ft[0]:.4f}", f"{ft[1]:.4f}", f"{ft[2]:.4f}", 
            f"{ft[3]:.4f}", f"{ft[4]:.4f}", f"{ft[5]:.4f}",
            f"{mse:.4f}", f"{psnr:.4f}", f"{ssim_val:.4f}", f"{l1:.4f}"
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        
        print(f"[Saved] {img_filename} | Fz: {ft[2]:.2f}N | MSE: {mse:.2f} | SSIM: {ssim_val:.4f}")

    def tare_sensor(self):
        """Re-tare the sensor"""
        if not self.sensor:
            print("[Warning] No sensor connected.")
            return
            
        print("\n[User] Re-taring sensor...")
        self.sensor.stopStreaming()
        time.sleep(0.1)
        self.sensor.tare()
        self.sensor.startStreaming()
        print("Sensor Zeroed.")

    def shutdown(self):
        """Clean shutdown of durability tester"""
        if not self.active:
            return
            
        print("\n[Durability Test] Shutting down...")
        if self.sensor:
            self.sensor.stopStreaming()
        if self.cap:
            self.cap.release()
        if self.csv_file:
            self.csv_file.close()
        self.active = False
        print("[Durability Test] Stopped.")


class CNCController:
    def __init__(self, baudrate=115200, timeout=1):
        print("Initializing CNC Controller...")
        self.HOME_FILE = "home_position_aticenter.txt"
        
        # Detect OS and set appropriate port pattern
        system = platform.system()
        
        if system == "Darwin":  # macOS
            available_ports = glob.glob("/dev/cu.usb*") + glob.glob("/dev/tty.usb*")
            available_ports = [p for p in available_ports if "wlan" not in p.lower() and "bluetooth" not in p.lower()]
        elif system == "Linux":  # Ubuntu/Linux
            available_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        else:  # Windows or other
            available_ports = []
            for p in serial.tools.list_ports.comports():
                available_ports.append(p.device)

        if not available_ports:
            raise Exception("No serial ports found!")

        # Try connecting to each available port
        for port in available_ports:
            try:
                print(f"Trying {port}...")
                self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                print(f"Connected successfully to {port}")
                break
            except serial.SerialException:
                print(f"Failed to connect to {port}")
        else:
            raise Exception("Could not connect to any available serial port.")

    def wait_for_idle(self):
        """Wait for CNC to finish current movement"""
        while True:
            time.sleep(0.1)
            self.serial_port.write(b'?')
            response = self.serial_port.readline().decode('utf-8').strip()
            if response.startswith("<Idle") or "Idle" in response:
                break

    def unlock_cnc(self):
        """Unlock the CNC machine to allow manual control."""
        self.serial_port.write(b'$X\n')
        time.sleep(1)
        command = f'$10=1\n'
        self.serial_port.write(b"$10=0\n")
        time.sleep(0.1)
        self.serial_port.write(command.encode())
        print('CNC machine unlocked')

    def view_grbl_settings(self):
        """View the current Grbl settings."""
        self.serial_port.write(b'$$\n')
        time.sleep(0.5)
        response = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8').strip()
        print(f'Grbl settings:\n{response}')

    def reset_grbl_settings(self):
        """Reset Grbl settings to default values."""
        self.serial_port.write(b'$RST=$\n')
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        if 'ok' in response:
            print('Grbl settings reset to default')
        else:
            print('Failed to reset Grbl settings')

    def move_to_machine_home(self):
        """Perform homing operation on the CNC machine."""
        self.serial_port.write(b'$H\n')
        time.sleep(2)
        self.xcoord = 0.0
        self.ycoord = 0.0
        self.zcoord = 0.0
        print('CNC machine homed')
    
    def get_current_position_work_home(self):
        """Query CNC for current position relative to work home (G92)"""
        while True:
            self.serial_port.write(b'?\n')
            time.sleep(0.1)
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")
            
            match = re.search(r"WPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                print(f"Current position (work home): X={x}, Y={y}, Z={z}")
                return x, y, z
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)
       
    def get_current_position_mechanical_home(self):
        """Query CNC for current position relative to mechanical home"""
        while True:
            self.serial_port.write(b'?\n')
            time.sleep(0.1)
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")
            
            match = re.search(r"MPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z = float(match.group(3))
                print(f"Current position (mechanical home): X={x}, Y={y}, Z={z}")
                return x, y, z
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)

    def get_feed_rate(self):
        """Get the current feed rate."""
        print(f'Current feed rate: {self.feed_rate} mm/min')
        return self.feed_rate
    
    def save_home_position(self, x, y, z):
        """Save the current home position to a file."""
        with open(self.HOME_FILE, "w") as f:
            f.write(f"{x},{y},{z}")
        print(f"Home position saved: X={x}, Y={y}, Z={z}")

    def load_home_position(self):
        """Load the saved home position from a file"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, self.HOME_FILE)
        print("file_path", file_path)
        if os.path.exists(file_path):
            try:
                with open(file_path, "r") as f:
                    data = f.read().strip()
                    x, y, z = map(float, data.split(","))
                    print(f"Loaded and set saved home position: X={x}, Y={y}, Z={z}")
                    return x, y, z
            except Exception as e:
                print(f"Error loading home position: {e}")
                print("Skipping home position load.")
        else:
            print("No saved home position found. Skipping home setup.")

    def set_curPos_home(self):
        """Set the current position as the work home (0,0,0)."""
        self.serial_port.write(b'G92 X0 Y0 Z0\n')
        time.sleep(0.1)
        print("Current position set as home (0,0,0)")

    def relative_move(self, x, y, z):
        """Send a relative movement command to the CNC machine."""
        self.serial_port.write(f'G91 X{x} Y{y} Z{z}\n'.encode())
        time.sleep(0.1)

    def absolute_move(self, x, y, z):
        """Send an absolute movement command to the CNC machine."""
        self.serial_port.write(f'G90 G01 X{x} Y{y} Z{z} F{self.feed_rate}\n'.encode())
        time.sleep(0.1)
        print(f'Absolute Move: G90 X{x} Y{y} Z{z} F{self.feed_rate}')

    def query_status(self):
        """Query the current status of the CNC machine."""
        self.serial_port.write(b'?')
        cnc_curr_status = self.serial_port.readline().decode('utf-8')
        print(f'CNC status: {cnc_curr_status}')
        return cnc_curr_status

    def set_feed_rate(self, feed_rate):
        """Set the feed rate for CNC movement in mm/min."""
        self.feed_rate = feed_rate
        time.sleep(0.1)
        self.serial_port.write(f"F{feed_rate}\n".encode())
        print(f"Feed rate set to: {feed_rate} mm/min")
        time.sleep(0.1)

    def set_incremental_mode(self):
        """Set CNC to relative (incremental) positioning mode."""
        self.serial_port.write(b'G91\n')
        time.sleep(0.1)
        print("Incremental mode set.")
    
    def generate_sweep_path(self):
        """Generate dome sweep path coordinates."""
        coordinates = []
        
        x_start = 135.0
        x_end = 177.0
        x_peak = 156.0
        y = 0.0
        z_bottom = -27.0
        z_peak = -20.0
        z_transit = -17.0
        x_step = 2.0
        
        coordinates.append([x_start, y, z_transit])
        
        num_steps = int((x_end - x_start) / x_step) + 1
        
        for i in range(num_steps):
            x = x_start + (i * x_step)
            dist_from_peak = abs(x - x_peak)
            max_dist = max(x_peak - x_start, x_end - x_peak)
            z_dome = z_peak + (z_bottom - z_peak) * (dist_from_peak / max_dist) ** 2
            
            coordinates.append([x, y, z_dome])
            
            if i < num_steps - 1:
                coordinates.append([x, y, z_transit])
        
        coordinates.append([x_end, y, z_transit])
        
        return coordinates
    
    def sweep_dome(self, durability_tester=None):
        """Execute the dome sweep pattern with optional durability testing."""
        print("\n=== Starting Dome Sweep ===")
        print("Generating sweep path...")
        
        coordinates = self.generate_sweep_path()
        
        print(f"Generated {len(coordinates)} waypoints")
        print(f"First point: X={coordinates[0][0]}, Y={coordinates[0][1]}, Z={coordinates[0][2]}")
        print(f"Last point: X={coordinates[-1][0]}, Y={coordinates[-1][1]}, Z={coordinates[-1][2]}")
        
        self.serial_port.write(b'G90\n')
        time.sleep(0.1)
        self.set_feed_rate(200)
        
        total_points = len(coordinates)
        for idx, coord in enumerate(coordinates):
            x, y, z = coord
            print(f"[{idx+1}/{total_points}] Moving to X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            self.absolute_move(x, y, z)
            
            self.wait_for_idle()
            
            # Save data if durability tester is active
            if durability_tester and durability_tester.active:
                durability_tester.save_data_point()
            
            time.sleep(0.5)
        
        print("\n=== Sweep Complete ===")
        print("Returning to home position...")
        self.absolute_move(0, 0, 0)
        self.wait_for_idle()
        time.sleep(1)
        print("Done!")
    
    def repeat_test(self, durability_tester=None):
        """
        Repeat test: Move to X=156, Y=0, then alternate between Z=-15 and Z=-21 for 1000 cycles.
        Now waits for each movement to complete before sending the next command.
        """
        print("\n=== Starting Repeat Test ===")
        
        x = 156.0
        y = 0.0
        z_top = -15.0
        z_bottom = -21.0
        repetitions = 1000
        
        # Set absolute mode and feed rate
        self.serial_port.write(b'G90\n')
        time.sleep(0.1)
        self.set_feed_rate(200)
        
        # Move to starting position (X=156, Y=0, Z=-15)
        print(f"Moving to starting position: X={x}, Y={y}, Z={z_top}")
        self.absolute_move(x, y, z_top)
        self.wait_for_idle()
        
        print(f"Starting {repetitions} cycles of Z movement...")
        
        # Alternate between z_top and z_bottom
        for i in range(repetitions):
            # Move down to z_bottom
            self.absolute_move(x, y, z_bottom)
            self.wait_for_idle()
            
            # Save data if durability tester is active
            if durability_tester and durability_tester.active:
                durability_tester.save_data_point()
            
            # Move up to z_top
            self.absolute_move(x, y, z_top)
            self.wait_for_idle()
            
            # Print progress every 50 cycles
            if (i + 1) % 50 == 0:
                print(f"Progress: {i + 1}/{repetitions} cycles completed")
        
        print("\n=== Repeat Test Complete ===")
        print("Returning to home position...")
        self.absolute_move(0, 0, 0)
        self.wait_for_idle()
        time.sleep(1)
        print("Done!")
    
    def jog_mode(self, rotation_control=None):
        self.set_incremental_mode()
        self.set_feed_rate(200)
        lin_steps = [0.1, 0.5, 1, 2, 5]
        lin_i = 2
        rot_steps = [0.5, 1, 2, 5]
        rot_i = 1
        rot_rpm = 120
        mode = 1
        print("Jog → 1=translate 2=rotate  arrows X/Y  [ ] Z  ,/. lin‑step  ↑/↓ rot‑step Esc=q")

        def on_press(key):
            nonlocal lin_i, rot_i, mode
            try:
                ch = key.char.lower()
            except AttributeError:
                ch = None

            # Mode switching
            if ch == '1':
                mode = 1
                print("→ Translation mode")
                return
            if ch == '2':
                if not (rotation_control and rotation_control.serial_port):
                    print("(Rotation unavailable)")
                    return
                mode = 2
                print("→ Rotation mode")
                return

            # Quit
            if key in (keyboard.Key.esc,) or ch == 'q':
                print("Exiting jog")
                return False
        
            # Translation controls
            if mode == 1:
                if key == keyboard.Key.right:
                    amt = lin_steps[lin_i]
                    self.relative_move(amt, 0, 0)
                    print(f"X +{amt}")
                elif key == keyboard.Key.left:
                    amt = lin_steps[lin_i]
                    self.relative_move(-amt, 0, 0)
                    print(f"X -{amt}")
                elif key == keyboard.Key.up:
                    amt = lin_steps[lin_i]
                    self.relative_move(0, amt, 0)
                    print(f"Y +{amt}")
                elif key == keyboard.Key.down:
                    amt = lin_steps[lin_i]
                    self.relative_move(0, -amt, 0)
                    print(f"Y -{amt}")
                elif ch == '[':
                    amt = lin_steps[lin_i]
                    self.relative_move(0, 0, -amt)
                    print(f"Z -{amt}")
                elif ch == ']':
                    amt = lin_steps[lin_i]
                    self.relative_move(0, 0, amt)
                    print(f"Z +{amt}")
                elif ch == ',':
                    lin_i = max(0, lin_i - 1)
                    print(f"Step {lin_steps[lin_i]} mm")
                elif ch == '.':
                    lin_i = min(len(lin_steps) - 1, lin_i + 1)
                    print(f"Step {lin_steps[lin_i]} mm")

            # Rotation controls
            elif mode == 2:
                if key == keyboard.Key.left:
                    deg = rot_steps[rot_i]
                    rotation_control.send_step("CCW", deg, rot_rpm)
                    print(f"CCW {deg}°")
                elif key == keyboard.Key.right:
                    deg = rot_steps[rot_i]
                    rotation_control.send_step("CW", deg, rot_rpm)
                    print(f"CW {deg}°")
                elif key == keyboard.Key.up:
                    rot_i = min(len(rot_steps) - 1, rot_i + 1)
                    print(f"Rot step {rot_steps[rot_i]}°")
                elif key == keyboard.Key.down:
                    rot_i = max(0, rot_i - 1)
                    print(f"Rot step {rot_steps[rot_i]}°")

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def test_mode(self, l, segments, wait_t, z_offset):
        """Sweep a square centered at the current work-home origin."""
        self.set_incremental_mode()
        vel = 200
        self.set_feed_rate(vel)

        dz = float(z_offset)
        z = 0.0
        if abs(dz) > 1e-9:
            self.relative_move(0, 0, dz)
            time.sleep(wait_t)
            z += dz

        step_xy = l / (segments - 1) if segments > 1 else 0.0

        x = 0.0
        y = 0.0
        self.relative_move(-l/2, l/2, 0)
        time.sleep(wait_t)
        x += -l/2
        y += l/2
        print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        counter = 0
        total_moves = (segments - 1) * segments
        for row in range(segments):
            dir_sign = 1 if row % 2 == 0 else -1

            for col in range(segments - 1):
                dx = dir_sign * step_xy
                self.relative_move(dx, 0, 0)
                time.sleep(wait_t)
                x += dx
                counter += 1
                print(f"Progress: {counter / total_moves * 100:.2f}%  Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

            if row < segments - 1:
                dy = -step_xy
                self.relative_move(0, dy, 0)
                time.sleep(wait_t)
                y += dy
                print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        self.relative_move(-x, -y, 0)
        print("Test-mode square complete.")
    
    def play_sound(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            wav_path = os.path.join(script_dir, "truck_sound.wav")
            wave_obj = sa.WaveObject.from_wave_file(wav_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()
            print("Played truck_sound.wav")
        except Exception as e:
            print(f"Error playing sound: {e}")

    def test_rotation(self, rotation_control=None, step_size=1, step_count=5, wait_time=4):
        """Test rotation of the CNC machine."""
        total_steps = step_count * 4
        count = 0
        if not rotation_control or not rotation_control.serial_port:
            print("Rotation control unavailable.")
            return

        for i in range(step_count):
            rotation_control.send_step("CW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        for i in range(step_count * 2):
            rotation_control.send_step("CCW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        for i in range(step_count):
            rotation_control.send_step("CW", step_size, 60)
            count += 1
            print(f"Progress: {count / total_steps * 100:.2f}%")
            time.sleep(wait_time)

        time.sleep(1)


def main():
    # Create controller instances
    controller = CNCController()
    controller_rotation = ArduinoController()
    durability_tester = DurabilityTester()

    # Unlock CNC (clear any alarms)
    controller.unlock_cnc()
    controller.set_feed_rate(500)
    
    # Skip homing - set current position as home instead
    print("\n=== Setting current position as home (0,0,0) ===")
    controller.set_curPos_home()
    
    # Display current coordinate system
    print("\n=== Current Coordinate System ===")
    print("Work Home: X=0, Y=0, Z=0 (set at current position)")
    print("All movements will be relative to this point.")
    print("=====================================\n")
    
    # Wait for CNC to be ready
    time.sleep(1)
    print("CNC ready!\n")

    while True:
        print("\n--- Commands ---")
        print("CNC: s=status, u=unlock, mh=machine_home, h=work_home, sethome, gp=get_pos")
        print("     a=abs_move, r=rel_move, j=jog, swp=sweep, rpt=repeat_test")
        print("Durability: dt_start=init_test, dt_save=save_data, dt_tare=tare_sensor, dt_stop=shutdown")
        print("Other: p=play_sound, test=rotation_test, q=quit")
        print("----------------")
        
        command = input("\nEnter command: ").strip().lower()
        
        if command == 's':
            controller.query_status()
        
        elif command == 'u':
            controller.unlock_cnc()

        elif command == 'mh':
            controller.move_to_machine_home()
        
        elif command == 'h':
            controller.set_feed_rate(1000)
            controller.absolute_move(0, 0, 0)
        
        elif command == 'sethome':
            controller.set_curPos_home()

        elif command == 'p':
            controller.play_sound()
        
        elif command == 'gp':
            controller.get_current_position_mechanical_home()
        
        elif command == 'swp':
            controller.sweep_dome(durability_tester)
        
        elif command == 'rpt':
            controller.repeat_test(durability_tester)
        
        elif command == 'a':
            try:
                coordinates = input("Enter the target position (x,y,z): ").strip().split(',')
                if len(coordinates) == 3:
                    x = float(coordinates[0])
                    y = float(coordinates[1])
                    z = float(coordinates[2])
                    controller.absolute_move(x, y, z)
                else:
                    print("Invalid input. Please enter coordinates in the format x,y,z.")
            except ValueError:
                print("Invalid input. Please enter numeric values for x, y, and z.")
        
        elif command == 'r':
            try:
                distances = input("Enter the distance to move (x,y,z): ").strip().split(',')
                if len(distances) == 3:
                    x = float(distances[0])
                    y = float(distances[1])
                    z = float(distances[2])
                    controller.relative_move(x, y, z)
                else:
                    print("Invalid input. Please enter distances in the format x,y,z.")
            except ValueError:
                print("Invalid input. Please enter numeric values for x, y, and z.")
        
        elif command == 'j':
            controller.jog_mode(rotation_control=controller_rotation)
        
        elif command == "test":
            controller.test_rotation(controller_rotation, step_size=2, step_count=5, wait_time=4)
            controller.play_sound()
        
        elif command.startswith("test "):
            try:
                _, argstr = command.split(" ", 1)
                l, seg, wt, z = [a.strip() for a in argstr.split(",")]
                controller.test_mode(float(l), int(seg), float(wt), z)
            except Exception as e:
                print(f"Error: {e}")
                print("Usage: test l,segments,wait_time,z")
        
        elif command == 'dt_start':
            durability_tester.initialize()
        
        elif command == 'dt_save':
            durability_tester.save_data_point()
        
        elif command == 'dt_tare':
            durability_tester.tare_sensor()
        
        elif command == 'dt_stop':
            durability_tester.shutdown()
        
        elif command == 'q':
            print("Exiting program.")
            durability_tester.shutdown()
            break
        
        else:
            print("Invalid command. Try again.")


if __name__ == '__main__':
    main()