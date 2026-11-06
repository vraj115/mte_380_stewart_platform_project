import cv2
import numpy as np
import json
import math
import os
import serial
import time
from datetime import datetime


class StewartPlatformCalibrator:
    """
    Interactive calibration tool for a 3-servo Stewart platform balancing
    an orange ping-pong ball.

    Calibration sequence:
        1. Detect the white circular top plate.
        2. Sample the orange ball color in HSV.
        3. Manually calibrate servo min / neutral / max angles.
        4. Save all results to stewart_config.json.
    """

    def __init__(self):
        # Physical platform parameters
        # CHANGE THIS to the measured diameter of your top plate.
        self.TOP_PLATE_DIAMETER_M = 0.30

        # Camera configuration
        self.CAM_INDEX = 1
        self.FRAME_W = 640
        self.FRAME_H = 480

        # Calibration state
        self.phase = "plate"
        self.current_frame = None

        # Plate geometry calibration
        self.detected_circle = None
        self.locked_circle = None
        self.manual_points = []
        self.pixel_to_meter_ratio = None

        # Orange-ball HSV calibration
        self.hsv_samples = []
        self.lower_hsv = None
        self.upper_hsv = None

        # Servo / Arduino configuration
        self.ser = None
        self.servo_port = "COM5"   # CHANGE THIS to your Arduino COM port
        self.baud_rate = 9600

        # Must match the safe range expected by the Arduino sketch
        self.ABS_MIN_ANGLE = 0
        self.ABS_MAX_ANGLE = 60

        # Starting values
        self.servo_angles = [15, 15, 15]
        self.servo_neutral = [15, 15, 15]
        self.servo_min = [0, 0, 0]
        self.servo_max = [30, 30, 30]
        self.selected_servo = 0

        # Save config beside this Python file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(script_dir, "stewart_config.json")

    def connect_serial(self):
        """Open the serial connection to the Arduino."""
        try:
            self.ser = serial.Serial(self.servo_port, self.baud_rate, timeout=1)
            time.sleep(2)
            print(f"[SERIAL] Connected on {self.servo_port}")
            self.send_servo_angles()
            return True
        except Exception as e:
            print(f"[SERIAL] Connection failed: {e}")
            print("[SERIAL] Vision calibration can still continue.")
            self.ser = None
            return False

    def send_servo_angles(self):
        """Send servo angles as: servo1,servo2,servo3\\n"""
        if self.ser is None:
            return

        command = (
            f"{self.servo_angles[0]},"
            f"{self.servo_angles[1]},"
            f"{self.servo_angles[2]}\n"
        )

        try:
            self.ser.write(command.encode("utf-8"))
        except Exception as e:
            print(f"[SERIAL] Send failed: {e}")

    def detect_white_plate(self, frame):
        """Detect the white circular top plate."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)

        _, mask = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_circle = None
        best_score = -1.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1500:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * math.pi * area / (perimeter ** 2)
            if circularity < 0.65:
                continue

            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius < 20:
                continue

            score = area * circularity
            if score > best_score:
                best_score = score
                best_circle = (
                    int(round(x)),
                    int(round(y)),
                    int(round(radius))
                )

        return best_circle, mask

    def lock_plate_circle(self, x, y, radius):
        """Accept plate geometry and compute meters per pixel."""
        if radius <= 0:
            print("[PLATE] Invalid radius.")
            return

        self.locked_circle = (int(x), int(y), int(radius))
        diameter_px = 2.0 * radius
        self.pixel_to_meter_ratio = self.TOP_PLATE_DIAMETER_M / diameter_px

        print(f"[PLATE] Center = ({x}, {y}) px, radius = {radius} px")
        print(f"[PLATE] Scale = {self.pixel_to_meter_ratio:.8f} m/pixel")

        self.phase = "ball_color"
        print("[INFO] Plate geometry complete.")
        print("[INFO] Click the orange ball 10-20 times, then press 'c'.")

    def sample_ball_color(self, x, y):
        """Sample a 5x5 HSV region around the click point."""
        if self.current_frame is None:
            return

        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)

        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px = x + dx
                py = y + dy

                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px].astype(np.float64))

        self.update_hsv_bounds()
        print(f"[BALL COLOR] {len(self.hsv_samples)} HSV samples collected")

    def update_hsv_bounds(self):
        """Compute adaptive HSV limits from collected orange-ball samples."""
        if not self.hsv_samples:
            return

        samples = np.array(self.hsv_samples)

        h_range = np.max(samples[:, 0]) - np.min(samples[:, 0])
        s_range = np.max(samples[:, 1]) - np.min(samples[:, 1])
        v_range = np.max(samples[:, 2]) - np.min(samples[:, 2])

        h_margin = max(5.0, h_range * 0.10)
        s_margin = max(10.0, s_range * 0.15)
        v_margin = max(10.0, v_range * 0.15)

        self.lower_hsv = [
            int(max(0, np.min(samples[:, 0]) - h_margin)),
            int(max(0, np.min(samples[:, 1]) - s_margin)),
            int(max(0, np.min(samples[:, 2]) - v_margin)),
        ]

        self.upper_hsv = [
            int(min(179, np.max(samples[:, 0]) + h_margin)),
            int(min(255, np.max(samples[:, 1]) + s_margin)),
            int(min(255, np.max(samples[:, 2]) + v_margin)),
        ]

    def detect_ball_preview(self, frame):
        """Preview orange-ball detection during color calibration."""
        if self.lower_hsv is None or self.upper_hsv is None:
            return None, None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # Restrict detection to inside the calibrated platform.
        if self.locked_circle is not None:
            cx, cy, r = self.locked_circle
            platform_mask = np.zeros_like(mask)
            cv2.circle(platform_mask, (cx, cy), r, 255, -1)
            mask = cv2.bitwise_and(mask, platform_mask)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None, mask

        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)

        if radius < 5 or radius > 100:
            return None, None, mask

        return (int(round(x)), int(round(y))), radius, mask

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.phase == "manual_plate":
            self.manual_points.append((x, y))

            if len(self.manual_points) == 1:
                print(f"[MANUAL PLATE] Center selected: ({x}, {y})")
                print("[MANUAL PLATE] Now click one point on the rim.")

            elif len(self.manual_points) == 2:
                center = self.manual_points[0]
                rim = self.manual_points[1]

                radius = int(round(math.sqrt(
                    (rim[0] - center[0]) ** 2
                    + (rim[1] - center[1]) ** 2
                )))

                self.lock_plate_circle(center[0], center[1], radius)

        elif self.phase == "ball_color":
            self.sample_ball_color(x, y)

    def save_config(self):
        """Save all calibration data to stewart_config.json."""
        config = {
            "timestamp": datetime.now().isoformat(),
            "camera": {
                "index": int(self.CAM_INDEX),
                "frame_width": int(self.FRAME_W),
                "frame_height": int(self.FRAME_H)
            },
            "platform": {
                "top_plate_diameter_m": float(self.TOP_PLATE_DIAMETER_M),
                "center_px": [
                    int(self.locked_circle[0]),
                    int(self.locked_circle[1])
                ] if self.locked_circle else None,
                "radius_px": int(self.locked_circle[2]) if self.locked_circle else None,
                "pixel_to_meter_ratio": float(
                    self.pixel_to_meter_ratio
                ) if self.pixel_to_meter_ratio else None
            },
            "ball_detection": {
                "lower_hsv": self.lower_hsv,
                "upper_hsv": self.upper_hsv
            },
            "servos": {
                "port": str(self.servo_port),
                "baud_rate": int(self.baud_rate),
                "current_angles_deg": [int(a) for a in self.servo_angles],
                "neutral_angles_deg": [int(a) for a in self.servo_neutral],
                "min_angles_deg": [int(a) for a in self.servo_min],
                "max_angles_deg": [int(a) for a in self.servo_max]
            }
        }

        with open(self.config_path, "w") as file:
            json.dump(config, file, indent=2)

        print(f"[SAVE] Configuration saved to:\n       {self.config_path}")

    def draw_overlay(self, frame, plate_mask=None):
        overlay = frame.copy()

        phase_text = {
            "plate": "Auto-detect white plate: G=accept, M=manual",
            "manual_plate": "Click plate center, then click plate rim",
            "ball_color": "Click orange ball 10-20 times, then press C",
            "servo": "1/2/3 select | A/D move | N neutral | J min | K max | S save",
            "complete": "Calibration complete"
        }

        cv2.putText(
            overlay, f"Phase: {self.phase}", (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
        )
        cv2.putText(
            overlay, phase_text[self.phase], (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1
        )

        if self.detected_circle is not None:
            x, y, r = self.detected_circle
            cv2.circle(overlay, (x, y), r, (0, 255, 255), 2)
            cv2.circle(overlay, (x, y), 4, (0, 255, 255), -1)

        if self.locked_circle is not None:
            x, y, r = self.locked_circle
            cv2.circle(overlay, (x, y), r, (0, 255, 0), 2)
            cv2.circle(overlay, (x, y), 4, (0, 255, 0), -1)

            # Cartesian axes: +x right, +y up.
            cv2.arrowedLine(
                overlay, (x, y), (x + 60, y),
                (255, 255, 255), 2, tipLength=0.15
            )
            cv2.putText(
                overlay, "+x", (x + 65, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )

            cv2.arrowedLine(
                overlay, (x, y), (x, y - 60),
                (255, 255, 255), 2, tipLength=0.15
            )
            cv2.putText(
                overlay, "+y", (x + 5, y - 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )

        for i, point in enumerate(self.manual_points):
            cv2.circle(overlay, point, 5, (255, 0, 0), -1)
            cv2.putText(
                overlay, f"P{i + 1}", (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1
            )

        if self.phase in ("ball_color", "servo", "complete"):
            if self.lower_hsv is not None:
                cv2.putText(
                    overlay, f"HSV lower: {self.lower_hsv}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
                )
                cv2.putText(
                    overlay, f"HSV upper: {self.upper_hsv}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
                )

                center, radius, _ = self.detect_ball_preview(frame)
                if center is not None:
                    cv2.circle(
                        overlay, center, int(radius), (0, 165, 255), 2
                    )
                    cv2.circle(overlay, center, 3, (0, 165, 255), -1)

        if self.phase in ("servo", "complete"):
            start_y = 135

            for i in range(3):
                color = (
                    (0, 255, 255)
                    if i == self.selected_servo
                    else (255, 255, 255)
                )

                text = (
                    f"S{i + 1}: current={self.servo_angles[i]} "
                    f"min={self.servo_min[i]} "
                    f"neutral={self.servo_neutral[i]} "
                    f"max={self.servo_max[i]}"
                )

                cv2.putText(
                    overlay, text, (10, start_y + 25 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
                )

        if plate_mask is not None:
            preview = cv2.resize(plate_mask, (160, 120))
            preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)

            h, w = overlay.shape[:2]
            overlay[10:130, w - 170:w - 10] = preview
            cv2.rectangle(
                overlay, (w - 170, 10), (w - 10, 130),
                (255, 255, 255), 1
            )

        return overlay

    def run(self):
        self.cap = cv2.VideoCapture(self.CAM_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera.")

        window_name = "Stewart Platform Calibration"

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        self.connect_serial()

        print("[INFO] Stewart Platform Calibration")
        print("")
        print("PHASE 1 - WHITE PLATE")
        print("  G : accept automatically detected circle")
        print("  M : manually select plate center and rim")
        print("")
        print("PHASE 2 - ORANGE BALL")
        print("  Click the orange ball 10-20 times")
        print("  C : finish ball-color calibration")
        print("")
        print("PHASE 3 - SERVOS")
        print("  1 / 2 / 3 : select servo")
        print("  A / D     : decrease / increase selected angle")
        print("  N         : save current angle as neutral")
        print("  J         : save current angle as minimum")
        print("  K         : save current angle as maximum")
        print("  R         : return all servos to saved neutral")
        print("  S         : save stewart_config.json and exit")
        print("")
        print("Q : quit without saving")

        while True:
            ret, frame = self.cap.read()

            if not ret:
                continue

            frame = cv2.resize(frame, (self.FRAME_W, self.FRAME_H))
            self.current_frame = frame
            plate_mask = None

            if self.phase == "plate":
                circle, plate_mask = self.detect_white_plate(frame)
                self.detected_circle = circle

            display = self.draw_overlay(frame, plate_mask)
            cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Calibration cancelled.")
                break

            elif key == ord("g") and self.phase == "plate":
                if self.detected_circle is None:
                    print("[PLATE] No valid circle detected.")
                else:
                    x, y, r = self.detected_circle
                    self.lock_plate_circle(x, y, r)

            elif key == ord("m") and self.phase == "plate":
                self.phase = "manual_plate"
                self.manual_points = []
                print(
                    "[MANUAL PLATE] Click the platform center, "
                    "then one rim point."
                )

            elif key == ord("c") and self.phase == "ball_color":
                if not self.hsv_samples:
                    print("[BALL COLOR] No samples collected yet.")
                else:
                    print(
                        f"[BALL COLOR] Final HSV range: "
                        f"{self.lower_hsv} to {self.upper_hsv}"
                    )
                    self.phase = "servo"
                    print("[INFO] Ball color complete.")
                    print("[INFO] Begin servo calibration.")

            elif self.phase == "servo":
                if key == ord("1"):
                    self.selected_servo = 0
                    print("[SERVO] Selected Servo 1")

                elif key == ord("2"):
                    self.selected_servo = 1
                    print("[SERVO] Selected Servo 2")

                elif key == ord("3"):
                    self.selected_servo = 2
                    print("[SERVO] Selected Servo 3")

                elif key == ord("a"):
                    i = self.selected_servo
                    self.servo_angles[i] = max(
                        self.ABS_MIN_ANGLE,
                        self.servo_angles[i] - 1
                    )
                    self.send_servo_angles()

                elif key == ord("d"):
                    i = self.selected_servo
                    self.servo_angles[i] = min(
                        self.ABS_MAX_ANGLE,
                        self.servo_angles[i] + 1
                    )
                    self.send_servo_angles()

                elif key == ord("n"):
                    i = self.selected_servo
                    self.servo_neutral[i] = self.servo_angles[i]
                    print(
                        f"[SERVO {i + 1}] "
                        f"Neutral = {self.servo_neutral[i]} deg"
                    )

                elif key == ord("j"):
                    i = self.selected_servo
                    self.servo_min[i] = self.servo_angles[i]
                    print(
                        f"[SERVO {i + 1}] "
                        f"Minimum = {self.servo_min[i]} deg"
                    )

                elif key == ord("k"):
                    i = self.selected_servo
                    self.servo_max[i] = self.servo_angles[i]
                    print(
                        f"[SERVO {i + 1}] "
                        f"Maximum = {self.servo_max[i]} deg"
                    )

                elif key == ord("r"):
                    self.servo_angles = self.servo_neutral.copy()
                    self.send_servo_angles()
                    print("[SERVO] Returned all servos to neutral.")

                elif key == ord("s"):
                    self.save_config()
                    self.phase = "complete"
                    break

        self.cap.release()
        cv2.destroyAllWindows()

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    calibrator = StewartPlatformCalibrator()
    calibrator.run()
