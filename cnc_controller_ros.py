#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import re
import os
import serial
import glob
import simpleaudio as sa
from pynput import keyboard
import threading
import serial.tools.list_ports
import subprocess
import signal
from datetime import datetime 

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

class CNCController:
    def __init__(self, baudrate=115200, timeout=1):
        print("Initializing CNC Controller...")
        self.HOME_FILE = "home_position_aticenter.txt"
        self.feed_rate = 500  # Initialize feed_rate

        # Get list of all available USB serial ports
        available_ports = glob.glob("/dev/ttyUSB*")

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
        """Get current position relative to work home (WPos)."""
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
        """Get current position relative to mechanical home (MPos)."""
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
        """Load the saved home position from a file."""
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
        """Set the current position as the work home and save it."""
        pos = self.get_current_position_mechanical_home()
        if pos:
            x, y, z = pos
            self.serial_port.write(b'G92 X0 Y0 Z0\n')
            time.sleep(0.1)
            return x, y, z
        else:
            print("Error: Could not retrieve position to set home.")

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

    def test_mode(self, l, segments, wait_t, z_offset, z_start=0.0, position_log_file=None):

        """Sweep a square centered at the current work-home origin."""
        import csv
        import threading

        self.set_incremental_mode()
        vel = 200  # Feed rate in mm/min
        self.set_feed_rate(vel)

        # Setup position logging if requested
        position_csv_writer = None
        position_csv_file = None
        start_time = None
        logging_active = True
        log_lock = threading.Lock()

        # Shared position state for continuous logging
        current_position = {'x': 0.0, 'y': 0.0, 'z': z_start, 'moving': False}
        position_lock = threading.Lock()

        if position_log_file:
            position_csv_file = open(position_log_file, 'w', newline='')
            position_csv_writer = csv.writer(position_csv_file)
            position_csv_writer.writerow(['Timestamp', 'Elapsed_Time_s', 'X_mm', 'Y_mm', 'Z_mm', 'Moving'])
            start_time = time.time()
            print(f"Position logging enabled: {position_log_file}")

            def continuous_logger():
                """Log position continuously at 50 Hz."""
                sample_interval = 0.02  # 50 Hz
                while logging_active:
                    with position_lock:
                        x = current_position['x']
                        y = current_position['y']
                        z = current_position['z']
                        moving = current_position['moving']

                    with log_lock:
                        timestamp = datetime.now().isoformat()
                        elapsed = time.time() - start_time
                        position_csv_writer.writerow([timestamp, elapsed, x, y, z, 1 if moving else 0])
                        position_csv_file.flush()

                    time.sleep(sample_interval)

            # Start continuous logging thread
            logger_thread = threading.Thread(target=continuous_logger)
            logger_thread.daemon = True
            logger_thread.start()

        def update_position(x, y, z, moving=False):
            """Update the current position state."""
            if position_log_file:
                with position_lock:
                    current_position['x'] = x
                    current_position['y'] = y
                    current_position['z'] = z
                    current_position['moving'] = moving

        def interpolate_motion(x_start, y_start, z_start, dx, dy, dz, feed_rate_mm_per_min):
            """
            Interpolate position during CNC motion for smooth position tracking.

            Args:
                x_start, y_start, z_start: Starting position
                dx, dy, dz: Movement deltas
                feed_rate_mm_per_min: Feed rate in mm/min
            """
            # Calculate distance and duration
            distance = (dx**2 + dy**2 + dz**2)**0.5
            if distance < 1e-6:
                return  # No movement

            # Calculate movement duration
            duration = (distance / feed_rate_mm_per_min) * 60  # Convert mm/min to seconds

            # Update position at high frequency during motion
            sample_interval = 0.02  # 50 Hz
            num_samples = int(duration / sample_interval)

            update_position(x_start, y_start, z_start, moving=True)

            # Update interpolated positions
            for i in range(1, num_samples + 1):
                t_frac = (i * sample_interval) / duration
                if t_frac > 1.0:
                    break

                x_interp = x_start + dx * t_frac
                y_interp = y_start + dy * t_frac
                z_interp = z_start + dz * t_frac

                update_position(x_interp, y_interp, z_interp, moving=True)
                time.sleep(sample_interval)

        dz = float(z_offset)
        z = z_start  # Start with the current z position
        if abs(dz) > 1e-9:
            self.relative_move(0, 0, dz)
            if position_log_file:
                interpolate_motion(0, 0, z, 0, 0, dz, vel)
            else:
                time.sleep(wait_t)
            z += dz  # Update z after the move
            update_position(0, 0, z, moving=False)
            time.sleep(wait_t)

        step_xy = 2 * l / (segments - 1) if segments > 1 else 0.0

        x = 0.0
        y = 0.0

        # Initial move to start position
        self.relative_move(-l, l, 0)
        if position_log_file:
            interpolate_motion(x, y, z, -l, l, 0, vel)
        else:
            time.sleep(5 * wait_t)
        x += -l
        y += l
        print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        update_position(x, y, z, moving=False)
        time.sleep(5 * wait_t)

        counter = 0
        total_moves = (segments - 1) * segments
        for row in range(segments):
            dir_sign = 1 if row % 2 == 0 else -1

            for _col in range(segments - 1):
                dx = dir_sign * step_xy
                self.relative_move(dx, 0, 0)

                # Update position during motion
                if position_log_file:
                    interpolate_motion(x, y, z, dx, 0, 0, vel)
                else:
                    time.sleep(wait_t)

                x += dx
                counter += 1
                print(f"Progress: {counter / total_moves * 100:.2f}%  Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
                update_position(x, y, z, moving=False)
                time.sleep(wait_t)

            if row < segments - 1:
                dy = -step_xy
                self.relative_move(0, dy, 0)

                # Update position during motion
                if position_log_file:
                    interpolate_motion(x, y, z, 0, dy, 0, vel)
                else:
                    time.sleep(wait_t)

                y += dy
                print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
                update_position(x, y, z, moving=False)
                time.sleep(wait_t)

        time.sleep(4*wait_t)
        self.relative_move(-x, -y, 0)
        print("Test-mode square complete.")

        # Close position log file
        if position_csv_file:
            logging_active = False
            position_csv_file.close()
            print(f"Position log saved: {position_log_file}")
    
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

    # def test_rotation(self, rotation_control=None, step_size=1, step_count=5, wait_time=4):
    #     """Test rotation of the CNC machine."""
    #     total_steps = step_count * 4
    #     count = 0
    #     if not rotation_control or not rotation_control.serial_port:
    #         print("Rotation control unavailable.")
    #         return

    #     for i in range(step_count):        
    #         rotation_control.send_step("CW", step_size, 60) 
    #         count += 1
    #         print(f"Progress: {count / total_steps * 100:.2f}%")
    #         time.sleep(wait_time)

    #     for i in range(step_count * 2):        
    #         rotation_control.send_step("CCW", step_size, 60)
    #         count += 1
    #         print(f"Progress: {count / total_steps * 100:.2f}%")
    #         time.sleep(wait_time)

    #     for i in range(step_count):        
    #         rotation_control.send_step("CW", step_size, 60)
    #         count += 1
    #         print(f"Progress: {count / total_steps * 100:.2f}%")
    #         time.sleep(wait_time)


    


    def jog_mode(self, rotation_control=None):
        """Interactive jogging mode with keyboard controls."""
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

class CNCControllerNode(Node):
    def __init__(self):
        super().__init__('cnc_controller_node')
        
        # Initialize process tracking for bash scripts
        self.current_bash_process = None
        
        # Initialize CNC controllers
        try:
            self.cnc_controller = CNCController()
            self.rotation_controller = ArduinoController()


            
            # Initial setup
            self.cnc_controller.unlock_cnc()
            self.cnc_controller.set_feed_rate(500)
            self.cnc_controller.move_to_machine_home()

            self.rotation_controller.send_step("CCW", -5) # to make roation motor stall
            self.rotation_controller.send_step("CCW", +5) # to make roation motor stall
            
            # Wait for machine to be idle
            while True:
                status = self.cnc_controller.query_status()
                if status.startswith("<Idle"):
                    break
                time.sleep(0.1)
                
            self.get_logger().info('CNC Controller initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CNC Controller: {e}')
            return
        
        # Create subscriber for command messages
        self.subscription = self.create_subscription(
            String,
            'cnc_commands',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('CNC Controller Node started. Listening for commands on /cnc_commands topic')

    def command_callback(self, msg):
        """Process incoming command messages."""
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {command}')
        
        try:
            self.process_command(command)
        except Exception as e:
            self.get_logger().error(f'Error processing command "{command}": {e}')

    def process_command(self, command):
        """Process individual commands."""
        if command == 's':
            self.cnc_controller.query_status()
            
        elif command == 'u':
            self.cnc_controller.unlock_cnc()
            
        elif command == 'mh':
            self.cnc_controller.move_to_machine_home()
            
        elif command == 'h':
            result = self.cnc_controller.load_home_position()
            if result:
                x, y, z = result
                self.cnc_controller.set_feed_rate(1000)
                self.cnc_controller.absolute_move(x, y, z)
            
        elif command == 'sethome':
            x, y, z = self.cnc_controller.get_current_position_mechanical_home()
            self.cnc_controller.save_home_position(x, y, z)
            self.cnc_controller.set_curPos_home()
            
        elif command == 'p':
            self.cnc_controller.play_sound()
            
        elif command == 'gp':
            self.cnc_controller.get_current_position_mechanical_home()
            
        elif command.startswith('a '):
            # Absolute move: format "a x,y,z"
            try:
                coords_str = command[2:].strip()
                coordinates = coords_str.split(',')
                if len(coordinates) == 3:
                    x, y, z = map(float, coordinates)
                    self.cnc_controller.absolute_move(x, y, z)
                else:
                    self.get_logger().error("Invalid absolute move format. Use: a x,y,z")
            except ValueError:
                self.get_logger().error("Invalid coordinates for absolute move")
                
        # elif command.startswith('r '):
        #     # Relative move: format "r x,y,z"
        #     try:
        #         coords_str = command[2:].strip()
        #         distances = coords_str.split(',')
        #         if len(distances) == 3:
        #             x, y, z = map(float, distances)
        #             self.cnc_controller.relative_move(x, y, z)
        #         else:
        #             self.get_logger().error("Invalid relative move format. Use: r x,y,z")
        #     except ValueError:
        #         self.get_logger().error("Invalid distances for relative move")
                
        elif command == 'j':
            # Run jog mode in a separate thread to avoid blocking
            jog_thread = threading.Thread(
                target=self.cnc_controller.jog_mode, 
                args=(self.rotation_controller,)
            )
            jog_thread.daemon = True
            jog_thread.start()
            
        elif command == 'test':
            # Legacy test command without files
            self.cnc_controller.test_mode(
                l=4,           # 4mm square
                segments=10,    # 10x10 grid
                wait_t=2,       # 2 second wait between moves
                z_offset=0      # no Z offset
            )
            self.cnc_controller.play_sound()
            
        # ★★★ MODIFIED LOGIC TO HANDLE ALL 'test' COMMANDS ★★★
        elif command.startswith('test '):
            parts = command.split()
            # parts will be ['test', 'a.csv', 'b.csv', 'seg=50'] for example

            if len(parts) < 3:
                self.get_logger().error("Invalid test format. Use: test <evm_file> <ati_file> [seg=N]")
                return

            evm_file = parts[1]
            ati_file = parts[2]
            
            # Default segment value
            seg_value = 10 
            
            # Check for the optional seg parameter
            if len(parts) == 4 and parts[3].startswith('seg='):
                try:
                    # Extract the number after "seg="
                    seg_value = int(parts[3][4:])
                except ValueError:
                    self.get_logger().error(f"Invalid segment value in '{parts[3]}'. Using default {seg_value}.")
            
            # Call the main test function with the parsed parameters
            self.run_inductance_test_with_cnc(evm_file, ati_file, seg=seg_value)

        elif command.startswith("ztest "):
            parts = command.split()
            if len(parts) < 3:
                self.get_logger().error("Usage: ztest <evm_file> <ati_file> [seg=N] [layers=M] [dz=Z]")
                return

            evm_file = parts[1]
            ati_file = parts[2]

            # Defaults
            xy_range = 4
            xy_seg = 10
            z_range = 0.5
            z_seg = 6
            th_range = 0
            th_seg = 1

            # Parse optional arguments
            for part in parts[3:]:
                if part.startswith("xyseg="):
                    try:
                        xy_seg = int(part[6:])
                    except ValueError:
                        self.get_logger().error(f"Invalid xy_seg value: {part}")
                elif part.startswith("zseg="):
                    try:
                        z_seg = int(part[5:])
                    except ValueError:
                        self.get_logger().error(f"Invalid z_seg value: {part}")
                elif part.startswith("zrange="):
                    try:
                        z_range = float(part[7:])
                    except ValueError:
                        self.get_logger().error(f"Invalid z_range value: {part}")
                elif part.startswith("xyrange="):
                    try:
                        xy_range = float(part[8:])
                    except ValueError:
                        self.get_logger().error(f"Invalid xy_range value: {part}")
                elif part.startswith("thrange="):
                    try:
                        th_range = float(part[8:])
                    except ValueError:
                        self.get_logger().error(f"Invalid th_range value: {part}")
                elif part.startswith("thseg="):
                    try:
                        th_seg = int(part[6:])
                    except ValueError:
                        self.get_logger().error(f"Invalid th_seg value: {part}")

            self.run_depth_sweep_test(evm_file, ati_file, xy_range=xy_range, xy_seg=xy_seg, z_range=z_range, z_seg=z_seg, th_range=th_range, th_seg=th_seg)

                
        elif command == 'kill_test':
            # Command to manually kill the running bash process
            self.kill_bash_process()
                
        elif command.startswith("rtest "):
            # print("Running rotation test...")
            parts = command.split()
            if len(parts) < 3:
                self.get_logger().error("Usage: rtest <evm_file> <ati_file> [thrange=X] [thseg=Y] [wait=Z] [rpm=R]")
                return

            evm_file = parts[1]
            ati_file = parts[2]

            # defaults
            th_range = 3.0
            th_seg = 1.0
            wait_time = 4.0
            rpm = 60

            # optional args
            for part in parts[3:]:
                if part.startswith("thrange="):
                    try:
                        th_range = float(part[len("thrange="):])
                    except ValueError:
                        self.get_logger().error(f"Invalid thrange value: {part}")
                elif part.startswith("thseg="):
                    try:
                        th_seg = float(part[len("thseg="):])
                    except ValueError:
                        self.get_logger().error(f"Invalid thseg value: {part}")
                elif part.startswith("wait="):
                    try:
                        wait_time = float(part[len("wait="):])
                    except ValueError:
                        self.get_logger().error(f"Invalid wait value: {part}")
                elif part.startswith("rpm="):
                    try:
                        rpm = int(part[len("rpm="):])
                    except ValueError:
                        self.get_logger().error(f"Invalid rpm value: {part}")

            self.test_rotation(
                evm_file, ati_file,
                th_range=th_range,
                th_seg=th_seg,
                wait_time=wait_time,
                rpm=rpm
            )

            
        elif command.startswith('feed '):
            # Set feed rate: format "feed 500"
            try:
                feed_rate = int(command[5:].strip())
                self.cnc_controller.set_feed_rate(feed_rate)
            except ValueError:
                self.get_logger().error("Invalid feed rate value")



    def run_depth_sweep_test(self, evm_file, ati_file, xy_range=2, xy_seg=10, z_range=0.5, z_seg=6, th_range=0, th_seg=1):
        def sweep_sequence():
            try:
                # Strip file extensions
                base_evm, ext_evm = os.path.splitext(evm_file)
                base_ati, ext_ati = os.path.splitext(ati_file)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")


                if z_seg > 1:
                    dz = z_range / (z_seg - 1)
                else:
                    dz = 0.0


                # Calculate rotation angles
                if th_seg <= 1:
                    r_angles = [0.0]
                else:
                    r_angles = [round(-th_range + (2 * th_range * i) / (th_seg - 1), 3) for i in range(th_seg)]

                prev_angle = 0.0

                current_z_offset = 0.0

                for angle in r_angles:
                    self.get_logger().info(f"🔁 Rotating to {angle:.2f}°")
                    if self.rotation_controller and self.rotation_controller.serial_port:
                        
                        delta_angle = angle - prev_angle
                        direction = "CCW" if delta_angle >= 0 else "CW"
                        self.rotation_controller.send_step(direction, abs(delta_angle), rpm=60)
                        prev_angle = angle
                        
                        time.sleep(2)
                    else:
                        self.get_logger().warn("No rotation controller connected")

                    # Create folder for current rotation angle
                    # angle_folder = f"{timestamp}_{base_evm}_r{int(angle)}_xyseg{xy_seg}"

                    # script_dir = os.path.expanduser("~/TI_EVM_logger")
                    # output_dir = os.path.join(script_dir, angle_folder)
                    # os.makedirs(output_dir, exist_ok=True)

                    # Create folder for current rotation angle and its subfolders
                    
                    base_folder_name = f"{timestamp}_{base_evm}_r{int(angle)}_xyseg{xy_seg}"

                    script_dir = os.path.expanduser("~/TI_EVM_logger")
                    parent_output_dir = os.path.join(script_dir, "logs", "data", base_folder_name)

                    evm_output_dir = os.path.join(parent_output_dir, "evm")
                    ati_output_dir = os.path.join(parent_output_dir, "ati")

                    os.makedirs(evm_output_dir, exist_ok=True)
                    os.makedirs(ati_output_dir, exist_ok=True)

                    for i in range(1, z_seg + 1):
                        z_offset = (i - 1) * dz

                        evm_filename = f"{timestamp}_{base_evm}_r{int(angle)}_xyseg{xy_seg}_z{i}{ext_evm}"
                        ati_filename = f"{timestamp}_{base_ati}_r{int(angle)}_xyseg{xy_seg}_z{i}{ext_ati}"

                        evm_layer_path = os.path.join(evm_output_dir, evm_filename)
                        ati_layer_path = os.path.join(ati_output_dir, ati_filename)

                        self.get_logger().info(f"[R={angle:.2f}° | Z Layer {i}] Starting scan at Z={z_offset:.3f} mm")
                        self.run_inductance_test_with_cnc_sync(evm_layer_path, ati_layer_path, xy_seg, xy_range, z_start=current_z_offset)

                        # Wait for logging + motion to complete
                        while self.current_bash_process is not None:
                            time.sleep(0.5)

                        # Move down in Z between layers (except after last)
                        if i < z_seg:
                            self.get_logger().info(f"Moving up by dz = {dz:.3f} mm")
                            self.cnc_controller.relative_move(0, 0, dz)
                            current_z_offset += dz
                            time.sleep(3)

                    # Return to top Z position after each full rotation sweep
                    if z_seg > 1:
                        self.get_logger().info(f"Resetting Z to bottom after R={angle:.2f}°")
                        self.cnc_controller.relative_move(0, 0, -dz * (z_seg - 1))
                        current_z_offset = 0.0
                        time.sleep(2)

                self.cnc_controller.play_sound()
                self.get_logger().info("✅ Full rotation-Z sweep test completed.")

                # Reset rotation back to 0°
                if self.rotation_controller and self.rotation_controller.serial_port:
                    if abs(prev_angle) > 1e-3:
                        direction = "CW" if prev_angle > 0 else "CCW"
                        self.get_logger().info(f"🔄 Resetting rotation back to 0° from {prev_angle:.2f}°")
                        self.rotation_controller.send_step(direction, abs(prev_angle), rpm=60)
                        time.sleep(2)
                    else:
                        self.get_logger().info("🎯 Already at 0°, no rotation reset needed.")


            except Exception as e:
                self.get_logger().error(f"❌ Error during depth sweep: {e}")
            finally:
                self.current_bash_process = None

        thread = threading.Thread(target=sweep_sequence)
        thread.daemon = True
        thread.start()



    def run_inductance_test_with_cnc_sync(self, evm_file, ati_file, xy_seg=10, xy_range=4, z_start=0.0):
        try:
            final_evm_filename = evm_file
            final_ati_filename = ati_file

            # Create position log file path in the same directory as evm_file
            # Extract directory and base filename
            evm_dir = os.path.dirname(evm_file)
            evm_basename = os.path.basename(evm_file)
            evm_name_noext = os.path.splitext(evm_basename)[0]

            # Create position log path (replace evm folder with cnc_position folder)
            parent_dir = os.path.dirname(evm_dir)  # Go up one level from evm folder
            position_dir = os.path.join(parent_dir, "cnc_position")
            os.makedirs(position_dir, exist_ok=True)

            position_log_file = os.path.join(position_dir, f"{evm_name_noext}_position.csv")

            self.get_logger().info("Starting CNC test...")
            self.get_logger().info(f"Grid size set to: {xy_seg}x{xy_seg}")
            self.get_logger().info(f"Starting inductance test script with files: {final_evm_filename}, {final_ati_filename}")
            self.get_logger().info(f"Position log file: {position_log_file}")

            # script_cmd = f"source /opt/ros/foxy/setup.bash && cd ~/TI_EVM_logger/ && ./run_inductance_test.sh -c 2 -e {final_evm_filename} -a {final_ati_filename}"

            script_cmd = (
                f"source /opt/ros/foxy/setup.bash && cd ~/TI_EVM_logger/ && "
                f"./run_inductance_test.sh -c 2 -e '{final_evm_filename}' -a '{final_ati_filename}'"
            )

            self.current_bash_process = subprocess.Popen(
                script_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid,
                executable='/bin/bash',
                bufsize=1,
                universal_newlines=True
            )

            self.get_logger().info(f"Inductance test script started with PID: {self.current_bash_process.pid}")

            stdout_thread = threading.Thread(target=self._read_stdout, args=(self.current_bash_process,))
            stderr_thread = threading.Thread(target=self._read_stderr, args=(self.current_bash_process,))
            stdout_thread.daemon = True
            stderr_thread.daemon = True
            stdout_thread.start()
            stderr_thread.start()

            time.sleep(2)  # Warmup
            self.cnc_controller.test_mode(
                l=xy_range,
                segments=xy_seg,
                wait_t=2,
                z_offset=0,
                z_start=z_start,
                position_log_file=position_log_file
            )

            self.kill_bash_process()
            self.cnc_controller.play_sound()
            self.get_logger().info("CNC test completed successfully")

        except Exception as e:
            self.get_logger().error(f"Error in test sequence: {e}")
        finally:
            self.current_bash_process = None


                
    def run_inductance_test_with_cnc(self, evm_file, ati_file, seg=10):
        """
        Run CNC test rotation followed by inductance test script.
        
        Args:
            evm_file (str): The base filename for the EVM data CSV.
            ati_file (str): The base filename for the ATI data CSV.
            seg (int): The number of segments for the CNC test grid (e.g., 10 for a 10x10 grid). 
                    Defaults to 10.
        """
        def test_sequence():
            try:
                # --- ★ RESTORED: Create a timestamp for unique filenames ★ ---
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

                # --- ★ MODIFIED: Combine timestamp and segment info into filenames ★ ---
                base_evm, ext_evm = os.path.splitext(evm_file)
                base_ati, ext_ati = os.path.splitext(ati_file)
                
                # Final filename format: 20250629_213000_evm_data_seg10.csv
                final_evm_filename = f"{timestamp}_{base_evm}_seg{seg}{ext_evm}"
                final_ati_filename = f"{timestamp}_{base_ati}_seg{seg}{ext_ati}"
                # ----------------------------------------------------------------------

                self.get_logger().info("Starting CNC test...")
                self.get_logger().info(f"Grid size set to: {seg}x{seg}")
                self.get_logger().info(f"Starting inductance test script with files: {final_evm_filename}, {final_ati_filename}")
                
                # Use the newly constructed final filenames in the shell command
                script_cmd = f"source /opt/ros/foxy/setup.bash && cd ~/TI_EVM_logger/ && ./run_inductance_test.sh -c 2 -e {final_evm_filename} -a {final_ati_filename}"

                # Start the bash process
                self.current_bash_process = subprocess.Popen(
                    script_cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    preexec_fn=os.setsid,
                    executable='/bin/bash',
                    bufsize=1,
                    universal_newlines=True
                )
                
                self.get_logger().info(f"Inductance test script started with PID: {self.current_bash_process.pid}")
                
                # Start threads to read stdout and stderr from the script
                stdout_thread = threading.Thread(target=self._read_stdout, args=(self.current_bash_process,))
                stderr_thread = threading.Thread(target=self._read_stderr, args=(self.current_bash_process,))
                
                stdout_thread.daemon = True
                stderr_thread.daemon = True
                
                stdout_thread.start()
                stderr_thread.start()
                
                # Give the logging script a moment to initialize
                time.sleep(2)
                
                # Use the 'seg' parameter in the CNC controller call
                self.cnc_controller.test_mode(
                    l=4,           # 4mm square
                    segments=seg,   # Use the seg parameter for grid size
                    wait_t=2,       # 2 second wait
                    z_offset=0      # no Z offset
                )

                self.kill_bash_process()
                self.cnc_controller.play_sound()
                self.get_logger().info("CNC test completed successfully")
                
            except Exception as e:
                self.get_logger().error(f"Error in test sequence: {e}")
            finally:
                self.current_bash_process = None
        
        # Run the entire test sequence in a separate thread to avoid blocking the main process
        test_thread = threading.Thread(target=test_sequence)
        test_thread.daemon = True
        test_thread.start()

    def _read_stdout(self, process):
        """Read and log stdout from subprocess."""
        try:
            for line in iter(process.stdout.readline, ''):
                if line:
                    # self.get_logger().info(f"Script stdout: {line.strip()}")
                    pass
                if process.poll() is not None:
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading stdout: {e}")

    def _read_stderr(self, process):
        """Read and log stderr from subprocess."""
        try:
            for line in iter(process.stderr.readline, ''):
                if line:
                    self.get_logger().error(f"Script stderr: {line.strip()}")
                if process.poll() is not None:
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading stderr: {e}")

    def kill_bash_process(self):
        """Properly terminate the bash process and its children."""
        if self.current_bash_process and self.current_bash_process.poll() is None:
            try:
                self.get_logger().info("Terminating bash process...")
                # Send SIGTERM to the process group (not SIGKILL)
                os.killpg(os.getpgid(self.current_bash_process.pid), signal.SIGTERM)
                
                # Wait for graceful shutdown
                try:
                    self.current_bash_process.wait(timeout=10)
                    self.get_logger().info("Bash process terminated gracefully")
                except subprocess.TimeoutExpired:
                    # If it doesn't terminate gracefully, force kill
                    self.get_logger().warning("Process didn't terminate gracefully, force killing...")
                    os.killpg(os.getpgid(self.current_bash_process.pid), signal.SIGKILL)
                    self.get_logger().warning("Bash process force killed")
                    
            except ProcessLookupError:
                # Process already terminated
                self.get_logger().info("Process already terminated")
            except Exception as e:
                self.get_logger().error(f"Error terminating bash process: {e}")
        else:
            self.get_logger().info("No active bash process to terminate")

def main(args=None):
    rclpy.init(args=args)
    
    node = CNCControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if hasattr(node, 'cnc_controller') and hasattr(node.cnc_controller, 'serial_port'):
            node.cnc_controller.serial_port.close()
        if hasattr(node, 'rotation_controller'):
            node.rotation_controller.close()
        
        # Kill any running bash process
        if hasattr(node, 'current_bash_process'):
            node.kill_bash_process()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()