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
CAMERA_DEVICE_ID = 6       # /dev/video4 (Change according to your environment)
EXPOSURE_VALUE = 30        # Manual Exposure Value/s
WB_TEMP = 6500             # White Balance Temperature
FRAME_WIDTH = 1600
FRAME_HEIGHT = 1200

# ATI Sensor Settings
SENSOR_IP = "192.168.2.1"  # ATI Sensor IP
COUNTS_PER_FORCE = 1000000.0
COUNTS_PER_TORQUE = 1000000.0

TOTAL_TOUCHES = 5
FZ_MAX_THRESHOLD = -3.7
FZ_MIN_THRESHOLD = -0.001
Z_STEP = 0.1
Z_SLEEP_TIME = 0.01

# NAS_MOUNT_PATH = '/mnt/armlabnas'
NAS_MOUNT_PATH = '~'
DURABILITY_DIRECTORY = 'ankush'
##############################################################################


class DurabilityTester:
    """Handles camera and force sensor for durability testing"""
    
    def __init__(self):
        # self.base_dir = f"{NAS_MOUNT_PATH}/{DURABILITY_DIRECTORY}"
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

        # Thresholds
        # self.Fz_threshold

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
        # self.csv_path = os.path.join(f"{NAS_MOUNT_PATH}/ankush/durability_test/")
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

        # Flush camera buffer to get the most recent frame
        for _ in range(5):
            self.cap.read()

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
    
    def run_durability_test(self,  cnc_controller):
        self.initialize()
        zero_force = self.get_force_reading()
        print(zero_force)
        time.sleep(2.0)
        for i in range(TOTAL_TOUCHES):
            print("Moving down")
            num_down_steps = 0
            while self.get_force_reading()[2] > FZ_MAX_THRESHOLD:
                cnc_controller.relative_move(0,0,-Z_STEP)
                time.sleep(Z_SLEEP_TIME)
                num_down_steps+=1
            # take photo and save PRESSED
            self.save_data_point()
            time.sleep(2.0)

            print(f"Moving up: {num_down_steps*Z_STEP}")
            cnc_controller.relative_move(0,0, num_down_steps*Z_STEP)
            time.sleep(2.0)
            # while self.get_force_reading()[2] < FZ_MIN_THRESHOLD:
            #     cnc_controller.relative_move(0,0,Z_STEP)
            #     time.sleep(Z_SLEEP_TIME)
            # take photo and save UNPRESSED
            self.save_data_point()
        
        print("Ending Test")
        # cnc_controller.relative_move(0,0, 20)

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