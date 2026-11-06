import cv2
import numpy as np
import json
import os


class BallDetector:
    """
    Detect an orange ping-pong ball on the calibrated Stewart platform.

    Coordinate convention:
        origin: center of circular platform
        +x: right
        -x: left
        +y: up
        -y: down

    Positions are returned in meters.
    """

    def __init__(self, config_file="stewart_config.json"):
        # Resolve config path relative to this Python file.
        if os.path.isabs(config_file):
            self.config_path = config_file
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.config_path = os.path.join(script_dir, config_file)

        if not os.path.exists(self.config_path):
            raise FileNotFoundError(
                f"Calibration file not found:\n"
                f"{self.config_path}\n"
                f"Run stewart_cal.py first."
            )

        with open(self.config_path, "r") as file:
            self.config = json.load(file)

        # Camera configuration
        camera = self.config["camera"]

        self.camera_index = int(camera["index"])
        self.frame_width = int(camera["frame_width"])
        self.frame_height = int(camera["frame_height"])

        # Platform geometry
        platform = self.config["platform"]

        center_px = platform.get("center_px")
        radius_px = platform.get("radius_px")
        scale = platform.get("pixel_to_meter_ratio")

        if center_px is None or radius_px is None or scale is None:
            raise ValueError(
                "Platform geometry is missing from stewart_config.json."
            )

        self.platform_center = (
            int(center_px[0]),
            int(center_px[1])
        )

        self.platform_radius_px = int(radius_px)
        self.pixel_to_meter_ratio = float(scale)

        # Orange-ball HSV calibration
        ball = self.config["ball_detection"]

        lower = ball.get("lower_hsv")
        upper = ball.get("upper_hsv")

        if lower is None or upper is None:
            raise ValueError(
                "Ball HSV calibration is missing from stewart_config.json."
            )

        self.lower_hsv = np.array(lower, dtype=np.uint8)
        self.upper_hsv = np.array(upper, dtype=np.uint8)

        print(f"[BALL DETECTOR] Loaded config:\n  {self.config_path}")
        print(
            f"[BALL DETECTOR] Platform center: "
            f"{self.platform_center} px"
        )
        print(
            f"[BALL DETECTOR] Scale: "
            f"{self.pixel_to_meter_ratio:.8f} m/pixel"
        )
        print(
            f"[BALL DETECTOR] HSV: "
            f"{self.lower_hsv.tolist()} to "
            f"{self.upper_hsv.tolist()}"
        )

    def create_ball_mask(self, frame):
        """Create an orange mask restricted to the circular platform."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_mask = cv2.inRange(
            hsv,
            self.lower_hsv,
            self.upper_hsv
        )

        platform_mask = np.zeros(
            color_mask.shape,
            dtype=np.uint8
        )

        cv2.circle(
            platform_mask,
            self.platform_center,
            self.platform_radius_px,
            255,
            -1
        )

        mask = cv2.bitwise_and(
            color_mask,
            platform_mask
        )

        mask = cv2.erode(
            mask,
            None,
            iterations=2
        )

        mask = cv2.dilate(
            mask,
            None,
            iterations=2
        )

        return mask

    def detect_ball(self, frame):
        """
        Detect the orange ball.

        Returns:
            found: bool
            center_px: (x, y) image coordinates or None
            radius_px: float or None
            x_m: Cartesian x position in meters
            y_m: Cartesian y position in meters

        Cartesian convention:
            +x -> right
            +y -> up
        """
        mask = self.create_ball_mask(frame)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return False, None, None, 0.0, 0.0

        largest = max(contours, key=cv2.contourArea)

        (x, y), radius = cv2.minEnclosingCircle(largest)

        if radius < 5 or radius > 100:
            return False, None, None, 0.0, 0.0

        center_px = (
            int(round(x)),
            int(round(y))
        )

        platform_x, platform_y = self.platform_center

        # OpenCV: +x right, +y down
        # Cartesian: +x right, +y up
        x_offset_px = x - platform_x
        y_offset_px = platform_y - y

        x_m = x_offset_px * self.pixel_to_meter_ratio
        y_m = y_offset_px * self.pixel_to_meter_ratio

        return (
            True,
            center_px,
            radius,
            float(x_m),
            float(y_m)
        )

    def draw_detection(self, frame):
        """Draw platform, Cartesian axes, and ball location."""
        overlay = frame.copy()

        cx, cy = self.platform_center

        # Platform boundary
        cv2.circle(
            overlay,
            self.platform_center,
            self.platform_radius_px,
            (255, 255, 255),
            2
        )

        # Origin
        cv2.circle(
            overlay,
            self.platform_center,
            4,
            (255, 255, 255),
            -1
        )

        # +x axis
        cv2.arrowedLine(
            overlay,
            (cx, cy),
            (
                cx + int(self.platform_radius_px * 0.45),
                cy
            ),
            (255, 255, 255),
            2,
            tipLength=0.12
        )

        cv2.putText(
            overlay,
            "+x",
            (
                cx + int(self.platform_radius_px * 0.48),
                cy + 5
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        # +y axis points upward on screen
        cv2.arrowedLine(
            overlay,
            (cx, cy),
            (
                cx,
                cy - int(self.platform_radius_px * 0.45)
            ),
            (255, 255, 255),
            2,
            tipLength=0.12
        )

        cv2.putText(
            overlay,
            "+y",
            (
                cx + 5,
                cy - int(self.platform_radius_px * 0.48)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        found, center, radius, x_m, y_m = self.detect_ball(frame)

        if found:
            cv2.circle(
                overlay,
                center,
                int(radius),
                (0, 255, 0),
                2
            )

            cv2.circle(
                overlay,
                center,
                3,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                overlay,
                f"x = {x_m:+.4f} m",
                (
                    center[0] + 15,
                    center[1] - 20
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

            cv2.putText(
                overlay,
                f"y = {y_m:+.4f} m",
                (
                    center[0] + 15,
                    center[1]
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

        else:
            cv2.putText(
                overlay,
                "Ball not detected",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

        return overlay, found, x_m, y_m


def main():
    detector = BallDetector("stewart_config.json")

    cap = cv2.VideoCapture(
        detector.camera_index,
        cv2.CAP_DSHOW
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        detector.frame_width
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        detector.frame_height
    )

    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1
    )

    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    print("")
    print("Stewart Platform Ball Detection Test")
    print("Coordinate system:")
    print("  +x = right")
    print("  +y = up")
    print("  origin = platform center")
    print("")
    print("Press Q to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        frame = cv2.resize(
            frame,
            (
                detector.frame_width,
                detector.frame_height
            )
        )

        display, found, x_m, y_m = detector.draw_detection(frame)

        if found:
            print(
                f"Ball position: "
                f"x={x_m:+.4f} m, "
                f"y={y_m:+.4f} m"
            )

        cv2.imshow(
            "Stewart Platform Ball Detection",
            display
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
