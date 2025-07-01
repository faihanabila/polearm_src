import cv2
import time
import threading
import csv
import os
from datetime import datetime
from pyzbar import pyzbar
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from picamera2 import Picamera2
from rclpy.executors import MultiThreadedExecutor

import tkinter as tk
import customtkinter as ctk
from PIL import Image, ImageTk

seen_barcodes = set()
barcode_locks = {}
LOCK_TIMEOUT = 2.0
camera_frames = {
    "Cam1-C270": None,
    "Cam2-Brio": None,
    "Cam3-PiCam3": None
}

# === CSV Logging Setup ===
now = datetime.now()
CSV_PATH = f"/home/pi/Polearm/src/barcode_detection/results/{now.strftime('%y%m%d')}.csv"
os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
with open(CSV_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "nomor seri", "type", "camera name", "time (s)", "fps"])

class BarcodeDetector:
    def __init__(self, logger):
        self.logger = logger

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.convertScaleAbs(gray, alpha=1.5, beta=10)
        binary = cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, 11, 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        barcodes = pyzbar.decode(binary)
        return [(barcode.data.decode("utf-8"), barcode.type) for barcode in barcodes]

class CameraThread(threading.Thread):
    def __init__(self, name, capture_func, publisher, logger):
        super().__init__(daemon=True)
        self.name = name
        self.capture_func = capture_func
        self.publisher = publisher
        self.logger = logger
        self.running = True
        self.detector = BarcodeDetector(logger)

    def run(self):
        while self.running:
            start = time.time()
            frame = self.capture_func()
            if frame is None:
                time.sleep(0.05)
                continue
            if len(frame.shape) == 2:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame_bgr = frame

            frame_display = cv2.resize(frame_bgr, (640, 480))
            detections = self.detector.process_frame(frame_display.copy())

            for data, btype in detections:
                key = (data, btype)
                now_sec = time.time()
                if key in seen_barcodes and (now_sec - barcode_locks.get(data, 0)) < LOCK_TIMEOUT:
                    continue

                fps = 1 / (time.time() - start + 1e-6)
                duration = round(time.time() - start, 2)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                with open(CSV_PATH, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, data, btype, self.name, f"{duration:.2f} s", f"{fps:.2f}"])

                self.publisher.publish(String(data=f"[{self.name}] {data} ({btype}) FPS: {fps:.2f}"))
                print(f"[{self.name}] Barcode: {data} ({btype}) | FPS: {fps:.2f}")
                seen_barcodes.add(key)
                barcode_locks[data] = now_sec

            camera_frames[self.name] = frame_display

    def stop(self):
        self.running = False

class ThreeCameraBarcodeNode(Node):
    def __init__(self):
        super().__init__('three_camera_barcode_node')

        self.declare_parameter('camera1_device', '/dev/video7')
        self.declare_parameter('camera2_device', '/dev/video2')

        cam1 = self.get_parameter('camera1_device').value
        cam2 = self.get_parameter('camera2_device').value

        self.cap1 = cv2.VideoCapture(cam1, cv2.CAP_V4L2)
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap1.set(cv2.CAP_PROP_FPS, 30)
        self.cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self.cap2 = cv2.VideoCapture(cam2, cv2.CAP_V4L2)
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap2.set(cv2.CAP_PROP_FPS, 30)
        self.cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameDurationLimits": (33333, 33333)}
        )
        self.picam2.configure(config)
        self.picam2.set_controls({
            "Contrast": 1.0,
            "Sharpness": 1.0,
            "Saturation": 0.0,
            "Brightness": 0.5
        })
        self.picam2.start()
        time.sleep(1)

        self.publisher_1 = self.create_publisher(String, 'barcode_data_cam1', 10)
        self.publisher_2 = self.create_publisher(String, 'barcode_data_cam2', 10)
        self.publisher_3 = self.create_publisher(String, 'barcode_data_cam3', 10)

        self.threads = [
            CameraThread("Cam1-C270", self.capture_cam1, self.publisher_1, self.get_logger()),
            CameraThread("Cam2-Brio", self.capture_cam2, self.publisher_2, self.get_logger()),
            CameraThread("Cam3-PiCam3", self.capture_picam3, self.publisher_3, self.get_logger()),
        ]

        for t in self.threads:
            t.start()

    def capture_cam1(self):
        ret, frame = self.cap1.read()
        return frame if ret else None

    def capture_cam2(self):
        ret, frame = self.cap2.read()
        return frame if ret else None

    def capture_picam3(self):
        return self.picam2.capture_array()

    def destroy_node(self):
        for t in self.threads:
            t.stop()
        for t in self.threads:
            t.join()
        if self.cap1.isOpened():
            self.cap1.release()
        if self.cap2.isOpened():
            self.cap2.release()
        self.picam2.stop()
        super().destroy_node()

class BarcodeGUI(ctk.CTk):
    def __init__(self, node):
        super().__init__()
        self.title("Barcode Monitor")
        self.geometry("1300x700")
        self.node = node
        self.camera_labels = {}
        self.create_widgets()
        self.bind("<KeyPress>", self.key_pressed)
        self.after(30, self.update_gui)

    def create_widgets(self):
        frame = ctk.CTkFrame(self)
        frame.pack(fill="both", expand=True, padx=10, pady=10)

        for name in camera_frames:
            lbl = ctk.CTkLabel(frame, text=name, width=400, height=300)
            lbl.pack(side="left", padx=10, pady=10)
            self.camera_labels[name] = lbl

    def key_pressed(self, event):
        if event.char.lower() == 'r':
            seen_barcodes.clear()
            print("Barcode reset (seen_barcodes cleared)")

    def update_gui(self):
        for name, label in self.camera_labels.items():
            frame = camera_frames.get(name)
            if frame is not None:
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(img)
                img = img.resize((400, 300))
                img_tk = ImageTk.PhotoImage(image=img)
                label.img_tk = img_tk
                label.configure(image=img_tk)
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.after(30, self.update_gui)

    def on_closing(self):
        self.node.destroy_node()
        self.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ThreeCameraBarcodeNode()
    print(">>> GUI Starting...")
    try:
        gui = BarcodeGUI(node)
        gui.protocol("WM_DELETE_WINDOW", gui.on_closing)
        gui.mainloop()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()