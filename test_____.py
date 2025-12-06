import os
import sys
import time
import cv2
import numpy as np
import csv
import threading
from pynput import keyboard
from skimage.metrics import structural_similarity as ssim

import NetFT

##############################################################################
# pip install scikit-image opencv-python pynput NetFT

# ==============================================================================
# CONFIGURATION
# ==============================================================================
CAMERA_DEVICE_ID = 6       # /dev/video4 (Change according to your environment)
SENSOR_IP = "192.168.2.1"  # ATI Sensor IP

# ATI Sensor Scaling Factors (Must check values from ATI web interface)
COUNTS_PER_FORCE = 1000000.0
COUNTS_PER_TORQUE = 1000000.0

# Camera Settings (Based on user code)
EXPOSURE_VALUE = 25        # Manual Exposure Value
WB_TEMP = 6500             # White Balance Temperature
FRAME_WIDTH = 1600
FRAME_HEIGHT = 1200
# ==============================================================================

class DurabilityTester:
    def __init__(self):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.save_root = os.path.join(self.base_dir, 'durability_test')
        
        # State flags
        self.running = True
        self.tare_requested = False
        self.save_requested = False
        
        # Initialize ATI Sensor
        self.init_sensor()
        
        # Initialize Camera
        self.init_camera()
        
        # Set data save path
        self.trial_dir = self.get_next_trial_dir()
        self.csv_path = os.path.join(self.trial_dir, 'data_log.csv')
        print(f"\n[Info] Data will be saved to: {self.trial_dir}")
        
        # Variables for initial image storage
        self.initial_image = None
        self.csv_file = None
        self.csv_writer = None
        
        # Initialize CSV file
        self.init_csv()

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
        
        # Open file in write mode
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
            sys.exit(1)

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
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=3", # Auto mode (to reset)
            f"v4l2-ctl --device /dev/video{video_id} -c auto_exposure=1", # Manual mode
            f"v4l2-ctl --device /dev/video{video_id} -c exposure_time_absolute={exposure_time}"
        ]
        for cmd in commands:
            os.system(cmd)

    def set_manual_wb(self, video_id, wb_temp):
        """Set manual white balance using v4l2-ctl"""
        commands = [
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=1", # Toggle to ensure reset
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_automatic=0",
            f"v4l2-ctl --device /dev/video{video_id} -c white_balance_temperature={wb_temp}"
        ]
        for cmd in commands:
            os.system(cmd)

    def compute_metrics(self, img1, img2):
        """Compute MSE, PSNR, SSIM, L1 between two images"""
        # Convert to grayscale for metrics (simplifies calculation)
        g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        g2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # 1. MSE (Mean Squared Error)
        diff = (g1.astype(np.float32) - g2.astype(np.float32)) ** 2
        mse = np.mean(diff)
        
        # 2. PSNR (Peak Signal-to-Noise Ratio)
        if mse == 0:
            psnr = 100.0
        else:
            psnr = 10 * np.log10(255.0**2 / mse)
            
        # 3. L1 Error (Mean Absolute Error)
        l1_error = np.mean(np.abs(g1.astype(np.float32) - g2.astype(np.float32)))
        
        # 4. SSIM (Structural Similarity Index)
        score, _ = ssim(g1, g2, full=True)
        
        return mse, psnr, score, l1_error

    def get_force_reading(self):
        """Get scaled force reading from sensor"""
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

    def on_press(self, key):
        """Keyboard listener callback"""
        try:
            if hasattr(key, 'char'):
                if key.char == 's':
                    self.save_requested = True
                elif key.char == 'r':
                    self.tare_requested = True
                elif key.char == 'q':
                    self.running = False
        except AttributeError:
            if key == keyboard.Key.esc:
                self.running = False

    def run(self):
        # Start keyboard listener
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        
        print("\n" + "="*60)
        print("Durability Test Started")
        print("Controls:")
        print("  [s] : Save current data (Image + Force + Diff metrics)")
        print("  [r] : Re-tare sensor")
        print("  [q] : Quit")
        print("="*60 + "\n")

        # Capture Initial Image
        ret, frame = self.cap.read()
        if ret:
            self.initial_image = frame.copy()
            cv2.imwrite(os.path.join(self.trial_dir, "initial_image.png"), self.initial_image)
            print("Initial reference image captured.")
        else:
            print("Error: Could not capture initial image.")
            return

        try:
            while self.running:
                # 1. Read Camera
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # 2. Read Sensor
                ft = self.get_force_reading()

                # 3. Handle Tare Request
                if self.tare_requested:
                    print("\n[User] Re-taring sensor...")
                    self.sensor.stopStreaming()
                    time.sleep(0.1)
                    self.sensor.tare()
                    self.sensor.startStreaming()
                    self.tare_requested = False
                    print("Sensor Zeroed.")

                # 4. Handle Save Request
                if self.save_requested:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    
                    # Compute Metrics against initial image
                    mse, psnr, ssim_val, l1 = self.compute_metrics(frame, self.initial_image)
                    
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
                    
                    print(f"Saved: {img_filename} | Force(Z): {ft[2]:.2f}N | MSE: {mse:.2f}")
                    self.save_requested = False

                # 5. Display Information
                # Create visualization text
                disp_img = frame.copy()
                # Resize for display if too big
                disp_img = cv2.resize(disp_img, (800, 600))
                
                info_text = f"Fz: {ft[2]:.2f} N"
                cv2.putText(disp_img, info_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                            1, (0, 255, 0), 2)
                
                cv2.imshow("Durability Test Monitor", disp_img)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False

        finally:
            print("\nShutting down...")
            self.sensor.stopStreaming()
            self.cap.release()
            cv2.destroyAllWindows()
            if self.csv_file:
                self.csv_file.close()
            listener.stop()
            print("Done.")

if __name__ == "__main__":
    tester = DurabilityTester()
    tester.run()