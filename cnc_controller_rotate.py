#!/usr/bin/env python3

import serial
import time
import re
import os
import glob
import simpleaudio as sa
from pynput import keyboard        # NEW – replaces the old `keyboard` module


import serial.tools.list_ports  # Import for listing serial ports

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
        self.HOME_FILE = "home_position_aticenter.txt"  # File to store home position

        # Get list of all available USB serial ports (macOS uses /dev/cu.* and /dev/tty.*)
        available_ports = glob.glob("/dev/cu.usb*") + glob.glob("/dev/tty.usb*")

        # Filter out non-hardware ports
        available_ports = [p for p in available_ports if "wlan" not in p.lower() and "bluetooth" not in p.lower()]

        if not available_ports:
            raise Exception("No serial ports found!")

        # Try connecting to each available port
        for port in available_ports:
            try:
                print(f"Trying {port}...")
                self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                print(f"Connected successfully to {port}")
                break  # Exit the loop if successful
            except serial.SerialException:
                print(f"Failed to connect to {port}")

        else:
            # If no ports worked, raise an exception
            raise Exception("Could not connect to any available serial port.")

    def unlock_cnc(self):
        """Unlock the CNC machine to allow manual control."""
        self.serial_port.write(b'$X\n')
        time.sleep(1)

        command = f'$10=1\n'  # Format the GRBL command / shows Mpos
        self.serial_port.write(b"$10=0\n")  # Disable override lock
        time.sleep(0.1)
        
        self.serial_port.write(command.encode())  # Send the command
        print('CNC machine unlocked')

    def view_grbl_settings(self):
        """View the current Grbl settings."""
        self.serial_port.write(b'$$\n')  # Send the settings query command
        time.sleep(0.5)  # Wait for the CNC machine to respond
        response = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8').strip()
        print(f'Grbl settings:\n{response}')

    def reset_grbl_settings(self):
        """Reset Grbl settings to default values."""
        self.serial_port.write(b'$RST=$\n')  # Send the reset command
        time.sleep(0.5)  # Wait for the CNC machine to respond
        response = self.serial_port.readline().decode('utf-8').strip()
        if 'ok' in response:
            print('Grbl settings reset to default')
        else:
            print('Failed to reset Grbl settings')

    def set_pulloff_distance(self, distance):
        """Set homing pull-off distance in mm for all axes."""
        self.serial_port.write(f'$27={distance}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        print(f'Pull-off distance set to {distance} mm')
        print(f'Response: {response}')

    def set_axis_pulloff_distance(self, axis, distance):
        """
        Set homing pull-off distance for a specific axis (GRBL 1.1+).
        axis: 'x', 'y', or 'z'
        distance: pulloff distance in mm
        """
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        if axis.lower() not in axis_map:
            print("Invalid axis. Use 'x', 'y', or 'z'")
            return
        
        axis_num = axis_map[axis.lower()]
        self.serial_port.write(f'$27.{axis_num}={distance}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        print(f'{axis.upper()}-axis pull-off distance set to {distance} mm')
        print(f'Response: {response}')

    def set_homing_direction(self, x_invert=False, y_invert=False, z_invert=False):
        """
        Set homing direction for each axis using $23 (homing dir invert mask).
        
        GRBL $23 setting uses a bitmask:
        - Bit 0 (value 1): Invert X homing direction
        - Bit 1 (value 2): Invert Y homing direction  
        - Bit 2 (value 4): Invert Z homing direction
        
        Parameters:
            x_invert: True to invert X (home in positive direction), False for negative
            y_invert: True to invert Y (home in positive direction), False for negative
            z_invert: True to invert Z (home in positive direction), False for negative
        
        Example: To make X home positive, Y negative, Z negative:
            set_homing_direction(x_invert=True, y_invert=False, z_invert=False)
        """
        mask = 0
        if x_invert:
            mask |= 1  # Set bit 0
        if y_invert:
            mask |= 2  # Set bit 1
        if z_invert:
            mask |= 4  # Set bit 2
        
        self.serial_port.write(f'$23={mask}\n'.encode())
        time.sleep(0.5)
        response = self.serial_port.readline().decode('utf-8').strip()
        
        directions = []
        directions.append(f"X: {'positive' if x_invert else 'negative'}")
        directions.append(f"Y: {'positive' if y_invert else 'negative'}")
        directions.append(f"Z: {'positive' if z_invert else 'negative'}")
        
        print(f'Homing directions set - {", ".join(directions)}')
        print(f'Response: {response}')

    def move_to_machine_home(self):
        """Perform homing operation on the CNC machine, with X offset to avoid limit switch."""
        print("Starting homing sequence...")
        self.serial_port.write(b'$H\n')
        time.sleep(4)  # Give it time to complete homing
        
        # After homing, move X away from the limit switch by 2mm
        # This prevents X from sitting right on the limit switch at "home"
        print("Moving X to 2mm offset from limit switch...")
        self.serial_port.write(b'G90\n')  # Absolute mode
        time.sleep(0.1)
        self.serial_port.write(b'G0 X2\n')  # Move X to 2mm from machine home
        time.sleep(1)
        
        # Now reset the work coordinate system so this position (2mm from limit) becomes our new "home" (0,0,0)
        self.serial_port.write(b'G92 X0 Y0 Z0\n')  # Set current position as work coordinate 0,0,0
        time.sleep(0.1)
        
        self.xcoord = 0.0
        self.ycoord = 0.0
        self.zcoord = 0.0
        print('CNC machine homed successfully!')
        print('X is 2mm away from limit switch (set as X=0 in work coordinates)')
    
    def get_current_position_work_home(self):
        """
        Continuously query the CNC machine for its current position relative to the work home (G92)
        until a valid position (WPos) is obtained.

        Returns:
            tuple: A tuple containing the current X, Y, and Z coordinates relative to the work home.
        """
        while True:
            # Send a status query to the CNC machine
            self.serial_port.write(b'?\n')  # Ensure newline for GRBL
            time.sleep(0.1)  # Wait for the response

            # Read the response from the CNC machine
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")  # Debugging output

            # Use regex to extract the current position from the response
            match = re.search(r"WPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))  # Extract X coordinate
                y = float(match.group(2))  # Extract Y coordinate
                z = float(match.group(3))  # Extract Z coordinate
                print(f"Current position (work home): X={x}, Y={y}, Z={z}")
                return x, y, z  # Exit loop when a valid position is found
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)  # Small delay before retrying
       
    def get_current_position_mechanical_home(self):
        """
        Continuously query the CNC machine for its current position relative to the mechanical home (machine coordinates)
        until a valid position (MPos) is obtained.

        Returns:
            tuple: A tuple containing the current X, Y, and Z coordinates relative to the mechanical home.
        """
        while True:
            # Send a status query to the CNC machine
            self.serial_port.write(b'?\n')  # Ensure newline for GRBL
            time.sleep(0.1)  # Wait for the response

            # Read the response from the CNC machine
            response = self.serial_port.readline().decode('utf-8').strip()
            print(f"Raw CNC response: {response}")  # Debugging output

            # Use regex to extract the current position from the response
            match = re.search(r"MPos:([-\d.]+),([-\d.]+),([-\d.]+)", response)
            if match:
                x = float(match.group(1))  # Extract X coordinate
                y = float(match.group(2))  # Extract Y coordinate
                z = float(match.group(3))  # Extract Z coordinate
                print(f"Current position (mechanical home): X={x}, Y={y}, Z={z}")
                return x, y, z  # Exit loop when a valid position is found
            else:
                print("Warning: Unable to parse current position. Retrying...")
                time.sleep(0.1)  # Small delay before retrying


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
        """Load the saved home position from a file and set it as the work home, if available."""

        script_dir = os.path.dirname(os.path.abspath(__file__))  # Folder where the script is
        file_path = os.path.join(script_dir, self.HOME_FILE)
        print("file_path", file_path)
        if os.path.exists(file_path):  # Check if file exists
            try:
                with open(file_path, "r") as f:
                    data = f.read().strip()
                    x, y, z = map(float, data.split(","))
                    # self.serial_port.write(f'G92 X{x} Y{y} Z{z}\n'.encode())
                    print(f"Loaded and set saved home position: X={x}, Y={y}, Z={z}")
                    return x,y,z
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
            # self.save_home_position(x, y, z)  # Save home position
            return x,y,z
        else:
            print("Error: Could not retrieve position to set home.")
    def relative_move(self, x, y, z):
        """Send a relative movement command to the CNC machine."""
        self.serial_port.write(f'G91 X{x} Y{y} Z{z}\n'.encode())
        time.sleep(0.1)
        # print(f'Relative Move: G91 X{x} Y{y} Z{z}')

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

    # def set_feed_rate(self, feed_rate):
    #     """Set the feed rate for CNC movement."""
    #     self.feed_rate = feed_rate
    #     self.serial_port.write(f'G21 F{feed_rate}\n'.encode())  # Set feed rate in mm/min
    #     print(f'Feed rate set to: {feed_rate} mm/min')
    #     time.sleep(0.1)

    def set_feed_rate(self, feed_rate):
        """Set the feed rate for CNC movement in mm/min."""
        self.feed_rate = feed_rate
        # self.serial_port.write("G21\n".encode())  # Ensure millimeter mode is set
        time.sleep(0.1)  # Short delay for CNC to process
        
        self.serial_port.write(f"F{feed_rate}\n".encode())  # Set the feed rate
        print(f"Feed rate set to: {feed_rate} mm/min")
        time.sleep(0.1)  # Short delay

    def set_incremental_mode(self):
        """Set CNC to relative (incremental) positioning mode."""
        self.serial_port.write(b'G91\n')
        time.sleep(0.1)
        print("Incremental mode set.")

    def get_feed_rate(self):
        """Get the current feed rate."""
        print(f'Current feed rate: {self.feed_rate} mm/min')  # Print the current feed rate
        return self.feed_rate  # Return the feed rate value
    
    def jog_mode(self, rotation_control=None):
            self.set_incremental_mode(); self.set_feed_rate(200)
            lin_steps = [0.1,0.5,1,2,5]; lin_i=2
            rot_steps=[0.5,1,2,5]; rot_i=1; rot_rpm=120
            mode=1
            print("Jog → 1=translate 2=rotate  arrows X/Y  [ ] Z  ,/. lin‑step  ↑/↓ rot‑step Esc=q")

            def on_press(key):
                nonlocal lin_i, rot_i, mode
                try: ch = key.char.lower()
                except AttributeError: ch = None

                # Mode switching
                if ch=='1':
                    mode=1; print("→ Translation mode"); return
                if ch=='2':
                    if not (rotation_control and rotation_control.serial_port):
                        print("(Rotation unavailable)"); return
                    mode=2; print("→ Rotation mode"); return

                # Quit
                if key in (keyboard.Key.esc,) or ch=='q':
                    print("Exiting jog"); return False
            
                # Translation controls
                if mode==1:
                    if key==keyboard.Key.right:
                        amt=lin_steps[lin_i]; self.relative_move(amt,0,0); print(f"X +{amt}")
                    elif key==keyboard.Key.left:
                        amt=lin_steps[lin_i]; self.relative_move(-amt,0,0); print(f"X -{amt}")
                    elif key==keyboard.Key.up:
                        amt=lin_steps[lin_i]; self.relative_move(0,amt,0); print(f"Y +{amt}")
                    elif key==keyboard.Key.down:
                        amt=lin_steps[lin_i]; self.relative_move(0,-amt,0); print(f"Y -{amt}")
                    elif ch=='[':
                        amt=lin_steps[lin_i]; self.relative_move(0,0,-amt); print(f"Z -{amt}")
                    elif ch==']':
                        amt=lin_steps[lin_i]; self.relative_move(0,0, amt); print(f"Z +{amt}")
                    elif ch==',':
                        lin_i=max(0,lin_i-1); print(f"Step {lin_steps[lin_i]} mm")
                    elif ch=='.':
                        lin_i=min(len(lin_steps)-1,lin_i+1); print(f"Step {lin_steps[lin_i]} mm")

                # Rotation controls
                elif mode==2:
                    if key==keyboard.Key.left:
                        deg=rot_steps[rot_i]; rotation_control.send_step("CCW",deg,rot_rpm); print(f"CCW {deg}°")
                    elif key==keyboard.Key.right:
                        deg=rot_steps[rot_i]; rotation_control.send_step("CW",deg,rot_rpm); print(f"CW {deg}°")
                    elif key==keyboard.Key.up:
                        rot_i=min(len(rot_steps)-1,rot_i+1); print(f"Rot step {rot_steps[rot_i]}°")
                    elif key==keyboard.Key.down:
                        rot_i=max(0,rot_i-1); print(f"Rot step {rot_steps[rot_i]}°")

            with keyboard.Listener(on_press=on_press) as listener:
                listener.join()

    # def jog_mode(self):
    #     """
    #     Interactive jogging that works under an unprivileged user
    #     (arrow keys = X/Y, [ ] = Z, ,/. step size, Esc quits).
    #     """
    #     self.set_incremental_mode()
    #     vel = 200            # mm / min
    #     self.set_feed_rate(vel)

    #     print("Jog mode ⟿  arrows: X/Y   [ ]: Z   , .: step   Esc: quit")

    #     steps = [0.1, 0.5, 1, 2, 5]
    #     step_i = 2           # default 1 mm

    #     def on_press(key):
    #         nonlocal step_i
    #         try:
    #             k = key.char
    #         except AttributeError:
    #             k = None

    #         # ── X / Y axes ────────────────────────────────────────────
    #         if key == keyboard.Key.right:
    #             self.relative_move( steps[step_i], 0, 0)
    #         elif key == keyboard.Key.left:
    #             self.relative_move(-steps[step_i], 0, 0)
    #         elif key == keyboard.Key.up:
    #             self.relative_move(0,  steps[step_i], 0)
    #         elif key == keyboard.Key.down:
    #             self.relative_move(0, -steps[step_i], 0)

    #         # ── Z axis ───────────────────────────────────────────────
    #         elif k == '[':
    #             self.relative_move(0, 0, -steps[step_i])
    #         elif k == ']':
    #             self.relative_move(0, 0,  steps[step_i])

    #         # ── step size control ───────────────────────────────────
    #         elif k == ',':
    #             step_i = max(0, step_i - 1)
    #             print(f"Step → {steps[step_i]} mm")
    #         elif k == '.':
    #             step_i = min(len(steps) - 1, step_i + 1)
    #             print(f"Step → {steps[step_i]} mm")

    #         # ── quit ────────────────────────────────────────────────
    #         elif key == keyboard.Key.esc or k in ('q', 'Q'):
    #             print("Exiting jog mode.")
    #             return False        # stop the listener

    #     # Start listening (blocks until Esc / q pressed)
    #     with keyboard.Listener(on_press=on_press) as listener:
    #         listener.join()

    def test_mode(self, l, segments, wait_t, z_offset):
        """
        Sweep a square centered at the current work-home origin.
        Start at (-l/2, +l/2). After each move, update and print calculated (x,y,z) position.
        """
        # 0. Switch controller to incremental mode
        self.set_incremental_mode()
        vel = 200  # Movement speed
        self.set_feed_rate(vel)

        # 1. Apply Z offset if needed
        dz = float(z_offset)
        z = 0.0  # initialize z
        if abs(dz) > 1e-9:
            self.relative_move(0, 0, dz)
            time.sleep(wait_t)
            z += dz

        # 2. Pre-compute step size
        step_xy = l / (segments - 1) if segments > 1 else 0.0

        # 3. Move to starting point (-l/2, l/2)
        x = 0.0
        y = 0.0
        self.relative_move(-l/2, l/2, 0)
        time.sleep(wait_t)
        x += -l/2
        y += l/2
        print(f"Current Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

        # 4. Raster scan
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

        # self.absolute_move(0,0,0)  # Move back to work home
        # Total relative offset from center = x, y, z (including raster scan and starting offset)
        self.relative_move(-x, -y, 0)

        # Then undo just the starting corner offset
        # self.relative_move(l/2, -l/2, 0)
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
        

    def test_rotation(self, rotation_control=None, step_size = 1, step_count = 5, wait_time=4):
        """
        Test rotation of the CNC machine.
        """

        total_steps = step_count * 4  # CW + CCW + CW
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
        # Example: Rotate 90 degrees counter-clockwise at 120 RPM

def main():
    # Create an instance of the CNC Controller
    controller = CNCController()
    controller_rotation = ArduinoController()

    # Unlock CNC and perform homing
    controller.unlock_cnc()
    controller.set_feed_rate(500)
    
    # Configure homing directions
    # Set X to home in POSITIVE direction (where your limit switch is)
    # Keep Y and Z in their default directions
    # Change x_invert to False if your X limit is in the negative direction
    controller.set_homing_direction(x_invert=True, y_invert=False, z_invert=False)
    
    # Configure X-axis pulloff distance to 1mm
    # This keeps Y and Z at their current GRBL-configured values
    controller.set_axis_pulloff_distance('x', 1.0)
    
    controller.move_to_machine_home()
    
    # Wait for CNC to be ready (FIXED SECTION)
    max_attempts = 20
    attempts = 0
    while attempts < max_attempts:
        status = controller.query_status()
        if status.startswith("<Idle") or "Idle" in status:
            break
        attempts += 1
        time.sleep(0.5)

    if attempts >= max_attempts:
        print("Warning: CNC may not be idle, but continuing anyway...")
    else:
        print("CNC ready!")
  

    while True:
        command = input("Enter command (s=status, mh=move to machine home, h=go to work home, sethome=set work home, gmp=get mechanical home position, a=absolute move, r=relative move, j=jog mode, q=quit): ").strip().lower()
        
        if command == 's':  # Query status
            controller.query_status()
        elif command == 'u':  # Move to machine home
            controller.unlock_cnc()

        elif command == 'mh':  # Move to machine home
            controller.move_to_machine_home()
        
        elif command == 'h':  # Go to work home (absolute move to 0,0,0)
            x,y,z = controller.load_home_position()
            controller.set_feed_rate(1000)
            controller.absolute_move(x,y,z)
        
        elif command == 'sethome':  # Alias for SH (set work home)
            x,y,z=controller.get_current_position_mechanical_home()
            controller.save_home_position(x,y,z)
            controller.set_curPos_home()

        elif command == 'p':
            controller.play_sound()  # Play sound
        
        elif command == 'gp':  # Get current position relative to mechanical home
            controller.get_current_position_mechanical_home()
        
        elif command == 'a':  # Absolute move
            try:
                # Prompt the user for X, Y, Z coordinates
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
        
        elif command == 'r':  # Relative move
            try:
                # Prompt the user for X, Y, Z distances
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
        
        elif command == 'j':  # Jog mode
            controller.jog_mode(rotation_control=controller_rotation)
            
        # elif command == 'test':  # Jog mode
        #     controller.set_feed_rate(50)
        #     controller.relative_move(-5,0,0)
        #     controller.relative_move(10,0,0)
        #     controller.relative_move(-5,0,0)

        elif command == "test":
            # default:  l=2 mm, segments=100, wait=1 s, z=0 mm offset
            # controller.test_mode(4, 10, 4, 0)
            controller.test_rotation(controller_rotation, step_size=2, step_count=5, wait_time=4)     
            controller.play_sound()  # Play sound after test completion

            try:
                _, argstr = command.split(" ", 1)
                l, seg, wt, z = [a.strip() for a in argstr.split(",")]
                controller.test_mode(float(l), int(seg), float(wt), z)
            except Exception:
                print("Usage: test  l,segments,wait_time,z")
        
        elif command == 'rotate test':
            # Test rotation of the CNC machine
            controller.test_rotation(controller_rotation, step_size=2, step_count=5, wait_time=4)
                    
        elif command == 'q':  # Quit
            print("Exiting program.")
            break
        #     # Speed control
        # elif command == '+':
        #     vel = controller.get_feed_rate()
        #     vel = max(vel + 100, 1000)  # Decrease speed (minimum 20 mm/min)
        #     print(f"Current velocity: {vel} mm/min")
        #     time.sleep(0.2)  # Debounce delay
        # elif command == '-':
        #     vel = controller.get_feed_rate()
        #     vel = min(vel - 100, 200)  # Increase speed (maximum 200 mm/min)
        #     print(f"Current velocity: {vel} mm/min")
        #     time.sleep(0.2)  # Debounce delay
        
        else:
            print("Invalid command. Try again.")


if __name__ == '__main__':
    main()