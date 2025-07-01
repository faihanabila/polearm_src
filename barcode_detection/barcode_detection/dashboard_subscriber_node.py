import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import datetime
import csv
import random
import time
from PIL import Image, ImageTk

class BarcodeHMI:
    def __init__(self, master):
        self.master = master
        master.title("Barcode Detection HMI Dashboard")
        master.geometry("1200x800")
        master.resizable(True, True)

        # --- Define Colors FIRST ---
        self.bg_color = "#E0F2F7" # Light Sky Blue (very light blue)
        self.frame_bg_color = "#FFFFFF" # White for inner frames
        self.primary_blue = "#2196F3" # Material Design Blue 500 (buttons, headings)
        self.dark_blue = "#1976D2" # Material Design Blue 700 (accents, active states)
        self.text_color = "#333333" # Dark grey for general text
        self.secondary_text_color = "#616161" # Lighter grey for less important text
        self.success_green = "#4CAF50" # Material Design Green (active, detection)
        self.danger_red = "#F44336" # Material Design Red (disconnected)
        self.warning_orange = "#FF9800" # Material Design Orange (for a "scanning" state)
        self.accent_blue = "#4682B4" # SteelBlue - medium blue (added back for Treeview selection)

        master.configure(bg=self.bg_color)

        # --- Configure Styles (Blue & White Theme) ---
        self.style = ttk.Style()
        self.style.theme_use('clam')

        # General Label Style
        self.style.configure("TLabel", background=self.frame_bg_color, foreground=self.text_color, font=("Arial", 10))
        # Frame Style
        self.style.configure("TFrame", background=self.frame_bg_color, relief="flat")
        self.style.configure("TLabelframe", background=self.frame_bg_color, foreground=self.dark_blue, font=("Arial", 12, "bold"))
        self.style.configure("TLabelframe.Label", background=self.frame_bg_color, foreground=self.dark_blue)

        # Button Style
        self.style.configure("TButton",
                             background=self.primary_blue,
                             foreground="white",
                             font=("Arial", 10, "bold"),
                             padding=[15, 8],
                             relief="flat")
        self.style.map("TButton",
                       background=[('active', self.dark_blue), ('pressed', self.dark_blue)],
                       foreground=[('active', 'white'), ('pressed', 'white')])

        # Treeview Style
        self.style.configure("Treeview.Heading",
                             font=("Arial", 11, "bold"),
                             background=self.primary_blue,
                             foreground="white",
                             relief="flat")
        self.style.configure("Treeview",
                             background="white",
                             foreground=self.text_color,
                             rowheight=28,
                             fieldbackground="white")
        self.style.map("Treeview",
                       background=[('selected', self.accent_blue)])

        # --- Variables to hold data and states ---
        self.is_detection_active = False
        self.camera_statuses = {
            "Camera 1": {"status": "Disconnected", "indicator_widget": None, "scan_count": tk.IntVar(value=0),
                         "fps_value": tk.StringVar(value="0.0"), "fps_frame_count": 0, "fps_start_time": time.time()},
            "Camera 2": {"status": "Disconnected", "indicator_widget": None, "scan_count": tk.IntVar(value=0),
                         "fps_value": tk.StringVar(value="0.0"), "fps_frame_count": 0, "fps_start_time": time.time()},
            "Camera 3": {"status": "Disconnected", "indicator_widget": None, "scan_count": tk.IntVar(value=0),
                         "fps_value": tk.StringVar(value="0.0"), "fps_frame_count": 0, "fps_start_time": time.time()}
        }
        self.total_barcodes_detected = tk.IntVar(value=0)
        self.last_detected_barcode = tk.StringVar(value="N/A")
        self.last_detection_timestamp = tk.StringVar(value="N/A")
        self.last_detection_duration = tk.StringVar(value="N/A")
        self.active_detection_camera = tk.StringVar(value="None")
        self.system_status_msg = tk.StringVar(value="System Ready. Click 'Start Detection' to begin.")

        self.barcode_log_data = []
        self.last_simulated_detection_time = 0

        # --- Build the UI ---
        self._create_widgets()
        self._update_camera_status_indicators()

    def _create_widgets(self):
        # Master Grid Configuration
        self.master.grid_rowconfigure(0, weight=0)
        self.master.grid_rowconfigure(1, weight=0)
        self.master.grid_rowconfigure(2, weight=1)
        self.master.grid_columnconfigure(0, weight=1)

        # --- Header Frame (Top Section: Camera Status & General Info) ---
        header_frame = ttk.Frame(self.master, padding="15", style="TFrame", relief="solid", borderwidth=1)
        header_frame.grid(row=0, column=0, sticky="ew", padx=15, pady=10)
        header_frame.grid_columnconfigure(0, weight=0) # For Logo
        header_frame.grid_columnconfigure(1, weight=1) # For Title (expands)
        header_frame.grid_columnconfigure(2, weight=0) # For Camera status block (fixed size)
        header_frame.grid_columnconfigure(3, weight=0) # For Active detector block (fixed size)


        # --- Add the Logo ---
        try:
            img = Image.open("polman_logo.png")
            original_width, original_height = img.size
            new_height = 50 # You can change this value to make the logo smaller/larger
            new_width = int(original_width * (new_height / original_height))
            img = img.resize((new_width, new_height), Image.LANCZOS)

            self.polman_logo = ImageTk.PhotoImage(img)

            logo_label = ttk.Label(header_frame, image=self.polman_logo, background=self.frame_bg_color)
            logo_label.grid(row=0, column=0, padx=(0, 15), pady=5, sticky="w")
        except FileNotFoundError:
            print("Error: polman_logo.png not found. Please ensure the image is in the same directory as the script.")
            ttk.Label(header_frame, text="[Logo Missing]", font=("Arial", 10, "italic"), foreground=self.danger_red, background=self.frame_bg_color).grid(row=0, column=0, padx=(0, 15), pady=5, sticky="w")
        except Exception as e:
            print(f"Error loading logo: {e}")
            ttk.Label(header_frame, text="[Logo Error]", font=("Arial", 10, "italic"), foreground=self.danger_red, background=self.frame_bg_color).grid(row=0, column=0, padx=(0, 15), pady=5, sticky="w")


        # App Title (Moved to column 1)
        ttk.Label(header_frame, text="Polearm Barcode Detection", font=("Arial", 15, "bold"),
                  foreground=self.primary_blue, background=self.frame_bg_color).grid(row=0, column=1, padx=10, pady=5, sticky="w")

        # Camera Status Block (Moved to column 2)
        cam_status_block_frame = ttk.Frame(header_frame, style="TFrame")
        cam_status_block_frame.grid(row=0, column=2, padx=10, pady=5, sticky="e")
        ttk.Label(cam_status_block_frame, text="Camera Status:", font=("Arial", 11, "bold")).pack(side="left", padx=(0, 10))

        # Create individual camera status indicators
        for i, cam_name in enumerate(self.camera_statuses.keys()):
            cam_frame = ttk.Frame(cam_status_block_frame, style="TFrame")
            cam_frame.pack(side="left", padx=10, pady=0)

            indicator_canvas = tk.Canvas(cam_frame, width=18, height=18, bg=self.frame_bg_color, highlightthickness=0)
            indicator_canvas.pack(side="left", padx=(0, 5))
            indicator_oval = indicator_canvas.create_oval(3, 3, 15, 15, fill=self.danger_red, outline=self.text_color, width=1)
            self.camera_statuses[cam_name]["indicator_widget"] = indicator_oval
            self.camera_statuses[cam_name]["canvas_widget"] = indicator_canvas

            ttk.Label(cam_frame, text=cam_name, font=("Arial", 10, "bold")).pack(side="left")

        # Active Detection Camera (Moved to column 3)
        active_detector_frame = ttk.Frame(header_frame, style="TFrame")
        active_detector_frame.grid(row=0, column=3, padx=10, pady=5, sticky="e")
        ttk.Label(active_detector_frame, text="Active Detector: ", font=("Arial", 11, "bold")).pack(side="left")
        ttk.Label(active_detector_frame, textvariable=self.active_detection_camera,
                  font=("Arial", 11, "bold"), foreground=self.primary_blue).pack(side="left")


        # --- Main Content Frame (Metrics, Last Barcode, Controls) ---
        main_content_frame = ttk.Frame(self.master, padding="15", style="TFrame", relief="solid", borderwidth=1)
        main_content_frame.grid(row=1, column=0, sticky="ew", padx=15, pady=10)
        main_content_frame.grid_columnconfigure(0, weight=1)
        main_content_frame.grid_columnconfigure(1, weight=2)

        # --- Metrics Frame (Left side of main content) ---
        metrics_frame = ttk.LabelFrame(main_content_frame, text="Overall Metrics", padding="15", style="TLabelframe")
        metrics_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        metrics_frame.grid_columnconfigure(0, weight=1)
        metrics_frame.grid_columnconfigure(1, weight=1)

        # Total Barcodes Detected
        ttk.Label(metrics_frame, text="Total Barcodes:", font=("Arial", 11, "bold")).grid(row=0, column=0, sticky="w", pady=5)
        ttk.Label(metrics_frame, textvariable=self.total_barcodes_detected, font=("Arial", 18, "bold"), foreground=self.dark_blue).grid(row=0, column=1, sticky="w", pady=5)

        # Barcodes per Camera
        ttk.Label(metrics_frame, text="Barcodes / Camera:", font=("Arial", 11, "bold")).grid(row=1, column=0, sticky="w", pady=5, columnspan=2)
        current_row_for_metrics = 2
        for i, cam_name in enumerate(self.camera_statuses.keys()):
            ttk.Label(metrics_frame, text=f"{cam_name}:", foreground=self.secondary_text_color).grid(row=current_row_for_metrics + i, column=0, sticky="w", padx=10)
            ttk.Label(metrics_frame, textvariable=self.camera_statuses[cam_name]["scan_count"], font=("Arial", 10, "bold")).grid(row=current_row_for_metrics + i, column=1, sticky="w")
        
        # FPS Display INSIDE Overall Metrics box, per camera
        current_row_for_metrics += len(self.camera_statuses) 
        ttk.Label(metrics_frame, text="Current FPS:", font=("Arial", 11, "bold")).grid(row=current_row_for_metrics, column=0, sticky="w", pady=(10,5), columnspan=2)
        current_row_for_metrics += 1 

        for i, cam_name in enumerate(self.camera_statuses.keys()):
            ttk.Label(metrics_frame, text=f"{cam_name}:", foreground=self.secondary_text_color).grid(row=current_row_for_metrics + i, column=0, sticky="w", padx=10)
            ttk.Label(metrics_frame, textvariable=self.camera_statuses[cam_name]["fps_value"], font=("Arial", 10, "bold"), foreground=self.dark_blue).grid(row=current_row_for_metrics + i, column=1, sticky="w")


        # --- Last Detected Barcode & Controls Frame (Right side of main content) ---
        barcode_controls_frame = ttk.Frame(main_content_frame, style="TFrame")
        barcode_controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        barcode_controls_frame.grid_rowconfigure(0, weight=1)
        barcode_controls_frame.grid_rowconfigure(1, weight=0)
        barcode_controls_frame.grid_rowconfigure(2, weight=0)
        barcode_controls_frame.grid_columnconfigure(0, weight=1)

        self.last_barcode_display_frame = ttk.LabelFrame(barcode_controls_frame, text="Last Detected Barcode", padding="15", style="TLabelframe")
        self.last_barcode_display_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 10))
        self.last_barcode_display_frame.grid_columnconfigure(0, weight=1)
        self.last_barcode_display_label = ttk.Label(self.last_barcode_display_frame, textvariable=self.last_detected_barcode,
                                                 font=("Arial", 40, "bold"), foreground=self.primary_blue, background=self.frame_bg_color)
        self.last_barcode_display_label.grid(row=0, column=0, pady=20, sticky="nsew")


        # Detection Details
        details_frame = ttk.Frame(barcode_controls_frame, style="TFrame", padding="10")
        details_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        details_frame.grid_columnconfigure(1, weight=1)
        details_frame.grid_columnconfigure(3, weight=1)

        ttk.Label(details_frame, text="Timestamp:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5)
        ttk.Label(details_frame, textvariable=self.last_detection_timestamp).grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(details_frame, text="Duration (s):", font=("Arial", 10, "bold")).grid(row=0, column=2, sticky="w", padx=20)
        ttk.Label(details_frame, textvariable=self.last_detection_duration).grid(row=0, column=3, sticky="w", padx=5)


        # Control Buttons
        button_frame = ttk.Frame(barcode_controls_frame, style="TFrame", padding="10")
        button_frame.grid(row=2, column=0, sticky="ew")
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
        button_frame.grid_columnconfigure(2, weight=1)
        button_frame.grid_columnconfigure(3, weight=1)
        button_frame.grid_columnconfigure(4, weight=1)

        ttk.Button(button_frame, text="Start Detection", command=self._start_detection).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(button_frame, text="Stop Detection", command=self._stop_detection).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ttk.Button(button_frame, text="Reset Data", command=self._reset_data).grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        ttk.Button(button_frame, text="Save Log (CSV)", command=self._save_log_to_csv).grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        ttk.Button(button_frame, text="Help", command=self._show_help).grid(row=0, column=4, padx=5, pady=5, sticky="ew")


        # --- Results Table / Data Logger Frame (Bottom Section) ---
        table_frame = ttk.LabelFrame(self.master, text="Barcode Detection History", padding="15", style="TLabelframe")
        table_frame.grid(row=2, column=0, sticky="nsew", padx=15, pady=10)
        table_frame.grid_rowconfigure(0, weight=1)
        table_frame.grid_columnconfigure(0, weight=1)

        columns = ("barcode_series", "timestamp", "detection_duration", "camera_source", "fps_at_detection")
        self.barcode_table = ttk.Treeview(table_frame, columns=columns, show="headings")

        self.barcode_table.heading("barcode_series", text="Barcode Series", anchor="center")
        self.barcode_table.heading("timestamp", text="Timestamp", anchor="center")
        self.barcode_table.heading("detection_duration", text="Duration (s)", anchor="center")
        self.barcode_table.heading("camera_source", text="Camera Source", anchor="center")
        self.barcode_table.heading("fps_at_detection", text="FPS at Detection", anchor="center")

        # Set column widths - allowing them to grow
        self.barcode_table.column("barcode_series", width=200, minwidth=150, stretch=tk.YES)
        self.barcode_table.column("timestamp", width=180, minwidth=150, stretch=tk.YES)
        self.barcode_table.column("detection_duration", width=100, minwidth=80, stretch=tk.YES)
        self.barcode_table.column("camera_source", width=100, minwidth=80, stretch=tk.YES)
        self.barcode_table.column("fps_at_detection", width=100, minwidth=80, stretch=tk.YES)

        self.barcode_table.grid(row=0, column=0, sticky="nsew")

        # Scrollbar for the table
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.barcode_table.yview)
        self.barcode_table.configure(yscrollcommand=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky="ns")

        # --- System Status Bar ---
        status_bar = ttk.Label(self.master, textvariable=self.system_status_msg, relief="groove", anchor="w",
                               font=("Arial", 10), background="white", foreground=self.text_color, padding=[10, 5])
        status_bar.grid(row=3, column=0, sticky="ew", padx=15, pady=(0, 15))


    def _update_camera_status_indicators(self):
        """Updates the color of the camera status indicators."""
        for cam_name, cam_data in self.camera_statuses.items():
            color = self.danger_red # Disconnected
            if cam_data["status"] == "Active":
                color = self.success_green
            elif cam_data["status"] == "Connected":
                color = self.warning_orange # Changed to orange for connected but not active
            elif cam_data["status"] == "Scanning": # Used during active scan if no barcode detected
                 color = self.warning_orange

            cam_data["canvas_widget"].itemconfig(cam_data["indicator_widget"], fill=color)

    def _show_help(self):
        """Displays a helpful message for new users."""
        help_text = (
            "Welcome to the Barcode Detection Dashboard!\n\n"
            "**How to Use:**\n"
            "1. Click 'Start Detection' to begin scanning for barcodes from your cameras.\n"
            "2. The 'Last Detected Barcode' area will show the most recent barcode found.\n"
            "3. 'Camera Status' indicators show if cameras are connected/active.\n"
            "4. The 'Barcode Detection History' table logs all successful scans.\n"
            "5. Click 'Stop Detection' to pause scanning.\n"
            "6. 'Reset Data' clears the history table.\n"
            "7. 'Save Log (CSV)' exports all logged data to a CSV file for analysis.\n\n"
            "**Camera Status Meanings:**\n"
            "  • Red: Disconnected\n"
            "  • Orange: Connected (Idle/Scanning but not active detector)\n"
            "  • Green: Active (This camera just detected a barcode)\n\n"
            "**FPS (Frames Per Second):**\n"
            "  • Indicates the rate at which each camera is processing frames or detection attempts."
        )
        messagebox.showinfo("Help - Using the Dashboard", help_text)


    # --- Control Functions ---
    def _start_detection(self):
        if not self.is_detection_active:
            self.is_detection_active = True
            self.system_status_msg.set("Detection started. Waiting for barcodes...")
            print("Detection Started")
            self._simulate_camera_connection() # Simulate cameras connecting
            # Reset FPS counters for all cameras when starting detection
            for cam_data in self.camera_statuses.values():
                cam_data["fps_frame_count"] = 0
                cam_data["fps_start_time"] = time.time()
                cam_data["fps_value"].set("0.0") # Reset FPS display to 0.0
            self.last_simulated_detection_time = time.time() # Initialize for rate calc (for overall simulation loop)
            self.master.after(500, self._simulate_barcode_detection) # Start simulation loop
        else:
            self.system_status_msg.set("Detection is already active.")
            messagebox.showinfo("Info", "Detection is already active.")

    def _stop_detection(self):
        if self.is_detection_active:
            self.is_detection_active = False
            self.system_status_msg.set("Detection stopped.")
            self.active_detection_camera.set("None")
            self._simulate_camera_disconnection() # Simulate cameras disconnecting
            # Reset all camera FPS displays to 0.0 or N/A
            for cam_data in self.camera_statuses.values():
                cam_data["fps_value"].set("0.0")
            print("Detection Stopped")
        else:
            self.system_status_msg.set("Detection is not active.")
            messagebox.showinfo("Info", "Detection is not active.")

    def _reset_data(self):
        if messagebox.askyesno("Confirm Reset", "Are you sure you want to clear all logged barcode data and reset counts?"):
            self.barcode_log_data.clear()
            for item in self.barcode_table.get_children():
                self.barcode_table.delete(item)

            self.total_barcodes_detected.set(0)
            for cam_name in self.camera_statuses.keys():
                self.camera_statuses[cam_name]["scan_count"].set(0)
                # Reset FPS for each camera too
                self.camera_statuses[cam_name]["fps_value"].set("0.0")
                self.camera_statuses[cam_name]["fps_frame_count"] = 0
                self.camera_statuses[cam_name]["fps_start_time"] = time.time()

            self.last_detected_barcode.set("N/A")
            self.last_detection_timestamp.set("N/A")
            self.last_detection_duration.set("N/A")
            self.active_detection_camera.set("None")
            self.system_status_msg.set("All logged data and counts cleared.")
            self.last_barcode_display_label.config(background=self.frame_bg_color) # Reset barcode display bg
            print("Data Reset")

    def _save_log_to_csv(self):
        if not self.barcode_log_data:
            messagebox.showinfo("Info", "No data to save. Start detection and capture some barcodes first.")
            return

        current_time_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"barcode_log_{current_time_str}.csv"

        try:
            fieldnames = ["Barcode Series", "Timestamp", "Detection Duration (s)", "Camera Source", "FPS at Detection"]
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.barcode_log_data)
            messagebox.showinfo("Save Successful", f"Data saved to {filename}")
            self.system_status_msg.set(f"Log saved successfully to {filename}.")
            print(f"Log saved to {filename}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save data: {e}\nCheck permissions or close the file if open.")
            self.system_status_msg.set(f"Error saving log: {e}")
            print(f"Error saving log: {e}")

    # --- Simulation Functions for Demonstration ---
    def _simulate_camera_connection(self):
        for cam_name in self.camera_statuses.keys():
            self.camera_statuses[cam_name]["status"] = "Connected"
        self._update_camera_status_indicators()

    def _simulate_camera_disconnection(self):
        for cam_name in self.camera_statuses.keys():
            self.camera_statuses[cam_name]["status"] = "Disconnected"
        self._update_camera_status_indicators()

    def _calculate_fps(self, camera_name):
        cam_data = self.camera_statuses[camera_name]
        cam_data["fps_frame_count"] += 1
        elapsed_time = time.time() - cam_data["fps_start_time"]

        if elapsed_time > 0.5: # Update FPS every 0.5 seconds
            current_fps = cam_data["fps_frame_count"] / elapsed_time
            cam_data["fps_value"].set(f"{current_fps:.1f}")
            cam_data["fps_frame_count"] = 0
            cam_data["fps_start_time"] = time.time()

    def _simulate_barcode_detection(self):
        if not self.is_detection_active:
            return

        # Update FPS for all connected cameras in every simulation cycle
        for cam_name, cam_data in self.camera_statuses.items():
            if cam_data["status"] != "Disconnected":
                self._calculate_fps(cam_name)

        current_time = time.time()
        # Simulate a detection success with 70% probability
        if random.random() < 0.7:
            detected_barcode = f"PDB-{random.randint(1000000, 9999999)}"
            detection_timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # --- Perubahan di SINI: Mengubah rentang durasi simulasi ---
            detection_duration_seconds = round(random.uniform(0.1, 2.5), 2) # Sekarang bisa 0.1 hingga 2.5 detik
            
            detected_camera = random.choice(list(self.camera_statuses.keys()))

            current_detected_camera_fps = self.camera_statuses[detected_camera]["fps_value"].get()

            # Update camera statuses based on detection
            for cam_name in self.camera_statuses.keys():
                if cam_name == detected_camera:
                    self.camera_statuses[cam_name]["status"] = "Active"
                else:
                    self.camera_statuses[cam_name]["status"] = "Connected"
            self._update_camera_status_indicators()

            # Update GUI variables
            self.last_detected_barcode.set(detected_barcode)
            self.last_detection_timestamp.set(detection_timestamp)
            self.last_detection_duration.set(f"{detection_duration_seconds:.2f}") 
            
            self.active_detection_camera.set(f"{detected_camera}")
            self.system_status_msg.set(f"SUCCESS: Barcode '{detected_barcode}' detected by {detected_camera}!")
            self.total_barcodes_detected.set(self.total_barcodes_detected.get() + 1)
            self.camera_statuses[detected_camera]["scan_count"].set(self.camera_statuses[detected_camera]["scan_count"].get() + 1)

            # Visual feedback for detection
            self.last_barcode_display_label.config(background=self.success_green, foreground="white")
            self.master.after(200, lambda: self.last_barcode_display_label.config(background=self.frame_bg_color, foreground=self.primary_blue))

            # Log to table
            log_entry = {
                "Barcode Series": detected_barcode,
                "Timestamp": detection_timestamp,
                "Detection Duration (s)": detection_duration_seconds,
                "Camera Source": detected_camera,
                "FPS at Detection": current_detected_camera_fps
            }
            self.barcode_log_data.insert(0, log_entry)
            self.barcode_table.insert("", "0", values=(detected_barcode, detection_timestamp, f"{detection_duration_seconds:.2f}", detected_camera, current_detected_camera_fps))

        else:
            self.system_status_msg.set("Scanning... No barcode detected in this cycle.")
            for cam_name, cam_data in self.camera_statuses.items():
                 if cam_data["status"] == "Active":
                     cam_data["status"] = "Connected"
                 elif cam_data["status"] == "Disconnected":
                     pass
                 else:
                     cam_data["status"] = "Scanning"
            self._update_camera_status_indicators()
            self.last_barcode_display_label.config(background=self.frame_bg_color, foreground=self.primary_blue)
            self.active_detection_camera.set("None")

        self.master.after(500, self._simulate_barcode_detection)


# --- Main Application Loop ---
if __name__ == "__main__":
    root = tk.Tk()
    app = BarcodeHMI(root)
    root.mainloop()