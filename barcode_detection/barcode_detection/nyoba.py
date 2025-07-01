import cv2
import csv
import os
from datetime import datetime
from pyzbar import pyzbar
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Global set to store all barcodes saved to CSV (persist during runtime)
seen_barcodes_global = set()

# Dictionary to store barcode detection timestamps
barcode_detection_times = {}

class BarcodeDetector:
    def __init__(self, logger):
        self.logger = logger

    def process_frame(self, frame):
        img = frame.copy()
        barcodes = pyzbar.decode(img)
        new_detections = []

        if not barcodes:
            cv2.putText(img, "Tidak ada barcode terdeteksi", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        for barcode in barcodes:
            data = barcode.data.decode("utf-8")
            btype = barcode.type
            key = (data, btype)

            if key not in seen_barcodes_global:
                text = f"{data} ({btype})"
                color = (0, 0, 255)  # Red for new detection
                new_detections.append((data, btype))
                self.logger.info(f"[NEW] {text}")
                barcode_detection_times[key] = time.time()  # Store the detection time
            else:
                # Check how much time has passed since the first detection
                time_diff = time.time() - barcode_detection_times[key]
                if time_diff > 5:  # If more than 1 second has passed since the first detection
                    text = "Barcode sudah terdeteksi"
                    color = (0, 255, 0)  # Green for already seen
                    self.logger.info(f"[ALREADY DETECTED] {data}")
                else:
                    # If less than 1 second, don't mark it as already detected
                    text = f"{data} ({btype})"
                    color = (0, 0, 255)  # Red for new detection
                    new_detections.append((data, btype))

            self._annotate_frame(img, barcode, text, color)

        return img, new_detections

    def _annotate_frame(self, img, barcode, text, color):
        (x, y, w, h) = barcode.rect
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


class WebcamBarcodeNode(Node):
    def __init__(self):
        super().__init__('webcam_barcode_node')
        self.get_logger().info("Memulai node barcode webcam...")

        # Open video sources
        self.cap1 = cv2.VideoCapture(2)
        self.cap2 = cv2.VideoCapture(4)

        if not self.cap1.isOpened() or not self.cap2.isOpened():
            self.get_logger().error("ERROR: Tidak dapat membuka salah satu kamera!")
            return

        self.detector = BarcodeDetector(self.get_logger())

        self.publisher_1 = self.create_publisher(String, 'barcode_data_cam1', 10)
        self.publisher_2 = self.create_publisher(String, 'barcode_data_cam2', 10)

        # Setup CSV output directory and file name
        results_dir = '/home/faiha/polearm/src/barcode_detection/barcode_detection/results'
        os.makedirs(results_dir, exist_ok=True)

        today_str = datetime.now().strftime("%Y%m%d")  # Format: YYYYMMDD
        self.csv_filename = os.path.join(results_dir, f"{today_str}.csv")

        self.csv_file = open(self.csv_filename, mode='w', newline='', encoding='utf-8')  # Open file in write mode ('w')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header only if file is empty
        self.csv_writer.writerow(['Timestamp', 'Data', 'Type', 'Camera', 'Time (seconds)'])

        self.start_time = None

    def run(self):
        while rclpy.ok():
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()

            if not ret1 or not ret2:
                self.get_logger().error("ERROR: Gagal membaca frame dari kamera!")
                break

            # Start the timer when the camera starts
            if self.start_time is None:
                self.start_time = time.time()

            # Process frame from Camera 1
            annotated1, detected1 = self.detector.process_frame(frame1)
            cv2.imshow("Barcode Scanner Cam 1", annotated1)
            for data, btype in detected1:
                if (data, btype) not in seen_barcodes_global:
                    self.publisher_1.publish(String(data=data))
                    self.save_to_csv(data, btype, "Cam1")

            # Process frame from Camera 2
            annotated2, detected2 = self.detector.process_frame(frame2)
            cv2.imshow("Barcode Scanner Cam 2", annotated2)
            for data, btype in detected2:
                if (data, btype) not in seen_barcodes_global:
                    self.publisher_2.publish(String(data=data))
                    self.save_to_csv(data, btype, "Cam2")

            # Handle key press for resetting barcode history (press 'r')
            key = cv2.waitKey(1)
            if key == ord('r'):
                seen_barcodes_global.clear()
                barcode_detection_times.clear()  # Clear barcode detection times as well
                self.get_logger().info("Barcode history reset.")

            # Exit condition (press 'q' to quit)
            if key & 0xFF == ord('q'):
                break

        self.cleanup()

    def save_to_csv(self, data, btype, camera):
        # Calculate the time spent for detection
        detection_time = round(time.time() - self.start_time, 2)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Timestamp format
        row = [timestamp, data, btype, camera, detection_time]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        seen_barcodes_global.add((data, btype))  # Mark the barcode as seen
        self.get_logger().info(f"[CSV] {row}")

    def cleanup(self):
        self.cap1.release()
        self.cap2.release()
        cv2.destroyAllWindows()
        self.csv_file.close()
        self.get_logger().info("Barcode scanner ditutup.")


def main(args=None):
    rclpy.init(args=args)
    node = WebcamBarcodeNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()