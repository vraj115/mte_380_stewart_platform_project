"""Inverse kinematics for a three-actuator, 3-DOF ball-balancing platform.

Based on the *two-link, vertical-plane* actuator model in the supplied
"Stewart Platform Kinematics" slides (especially slides 13-20, 22-34, 35-46).

Assumptions
-----------
* Three radial actuator planes, separated by 120 degrees, as in the slides.
* Each motor drives a rigid lower link in its vertical plane. A rigid upper
  link connects the elbow to a ball joint on the moving platform.
* Three platform joints form an equilateral triangle at a known joint radius.
* The moving top is rigid. Roll, pitch, and center height are commanded.
  Its horizontal translation and small induced yaw are solved so that all
  three joints remain in their respective actuator planes.
* The camera Cartesian axes and the physical mechanism's x/y axes have
  been aligned. A different mounting orientation requires adaptation.

All lengths are meters. All *public* angles are degrees. No motor command is
sent by this module. Unreachable or out-of-limit poses raise errors instead
of silently clipping angles.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


class KinematicsError(ValueError):
    """Invalid geometry, unreachable pose, or unsafe servo output."""


def _rot_x(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rot_y(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rot_z(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _positive(name: str, value: Any) -> float:
    if value is None:
        raise KinematicsError(
            f"Missing '{name}' in kinematics_config.json. "
            "Measure the actual mechanism before using this model."
        )
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise KinematicsError(f"'{name}' must be a number in metres.") from exc
    if not math.isfinite(number) or number <= 0:
        raise KinematicsError(f"'{name}' must be finite and > 0.")
    return number


def _triple(name: str, value: Any, positive: bool = False) -> tuple:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise KinematicsError(f"'{name}' must contain exactly three values.")
    result = []
    for i, x in enumerate(value):
        if positive:
            result.append(_positive(f"{name}[{i}]", x))
        else:
            try:
                v = float(x)
            except (TypeError, ValueError) as exc:
                raise KinematicsError(f"Invalid {name}[{i}]: {x!r}") from exc
            if not math.isfinite(v):
                raise KinematicsError(f"Non-finite {name}[{i}].")
            result.append(v)
    return tuple(result)


class StewartKinematics:
    """Convert desired roll/pitch/height to three calibrated servo angles."""

    # Slide axes: actuator 1 is along +y, actuator 2 at -30 degrees,
    # actuator 3 at -150 degrees in the x-y plane.
    ACTUATOR_AZIMUTHS_DEG = (90.0, -30.0, -150.0)

    def __init__(self, geometry_config: dict, servo_config: dict):
        geometry = geometry_config.get("geometry", {})
        actuators = geometry_config.get("actuators", {})
        servos = servo_config.get("servos", {})

        self.base_radii = _triple(
            "base_motor_radii_m", geometry.get("base_motor_radii_m"), True
        )
        self.joint_radius = _positive(
            "platform_joint_radius_m", geometry.get("platform_joint_radius_m")
        )
        self.lower_lengths = _triple(
            "lower_link_lengths_m", geometry.get("lower_link_lengths_m"), True
        )
        self.upper_lengths = _triple(
            "upper_link_lengths_m", geometry.get("upper_link_lengths_m"), True
        )
        self.home_height = _positive(
            "home_height_m", geometry.get("home_height_m")
        )
        self.max_tilt = _positive(
            "max_tilt_deg", geometry.get("max_tilt_deg", 3.0)
        )
        self.max_height_delta = _positive(
            "max_height_delta_m", geometry.get("max_height_delta_m", 0.005)
        )
        # Optional common z offset from the platform center to joint layer.
        self.joint_z_offset = float(geometry.get("joint_z_offset_m", 0.0))
        if not math.isfinite(self.joint_z_offset):
            raise KinematicsError("joint_z_offset_m must be finite.")

        self.elbow_branches = tuple(actuators.get("elbow_branches", [-1, -1, -1]))
        self.directions = tuple(actuators.get("servo_directions", [1, 1, 1]))
        if len(self.elbow_branches) != 3 or any(v not in (-1, 1) for v in self.elbow_branches):
            raise KinematicsError("elbow_branches must be three values of -1 or +1.")
        if len(self.directions) != 3 or any(v not in (-1, 1) for v in self.directions):
            raise KinematicsError("servo_directions must be three values of -1 or +1.")

        self.neutral_servo = _triple(
            "neutral_angles_deg", servos.get("neutral_angles_deg")
        )
        self.minimum_servo = _triple(
            "min_angles_deg", servos.get("min_angles_deg")
        )
        self.maximum_servo = _triple(
            "max_angles_deg", servos.get("max_angles_deg")
        )
        for i in range(3):
            if not (self.minimum_servo[i] <= self.neutral_servo[i] <= self.maximum_servo[i]):
                raise KinematicsError(
                    f"Servo {i + 1} neutral must lie between its calibrated limits."
                )

        azimuths = np.deg2rad(self.ACTUATOR_AZIMUTHS_DEG)
        self.u = np.column_stack((np.cos(azimuths), np.sin(azimuths)))
        # Tangent direction perpendicular to each radial actuator plane.
        self.tangent = np.column_stack((-self.u[:, 1], self.u[:, 0]))

        # Define the home *mechanical* angles, so a zero-tilt pose maps exactly
        # to each independently calibrated servo neutral (e.g., [15, 15, 15]).
        home_joints, _, _ = self.platform_joints(0.0, 0.0, self.home_height)
        self.home_motor_angles = self._motor_angles(home_joints)

    @classmethod
    def from_files(
        cls,
        geometry_path: str | Path | None = None,
        calibration_path: str | Path | None = None,
    ) -> "StewartKinematics":
        folder = Path(__file__).resolve().parent
        geometry_path = Path(geometry_path) if geometry_path else folder / "kinematics_config.json"
        calibration_path = Path(calibration_path) if calibration_path else folder / "stewart_config.json"
        with geometry_path.open("r", encoding="utf-8") as file:
            geometry_config = json.load(file)
        with calibration_path.open("r", encoding="utf-8") as file:
            servo_config = json.load(file)
        return cls(geometry_config, servo_config)

    def platform_joints(
        self, roll_deg: float, pitch_deg: float, height_m: float | None = None
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Return three world-space attachment points, platform center, yaw.

        Roll is positive about +x, pitch positive about +y, using the right-
        hand rule. Translation in x/y and passive yaw are solved such that the
        three attachment points stay within the radial actuator planes.

        Returns:
            joints_xyz: (3,3) world-space points, in metres
            center_xyz: (3,) position of moving plate center
            induced_yaw_deg: orientation around world z, in degrees
        """
        roll, pitch = float(roll_deg), float(pitch_deg)
        height = self.home_height if height_m is None else float(height_m)
        if not all(math.isfinite(v) for v in (roll, pitch, height)):
            raise KinematicsError("Roll, pitch, and height must be finite.")
        if math.hypot(roll, pitch) > self.max_tilt + 1e-12:
            raise KinematicsError(
                f"Combined roll/pitch magnitude exceeds {self.max_tilt:g} deg "
                "safety setting."
            )
        if abs(height - self.home_height) > self.max_height_delta + 1e-12:
            raise KinematicsError(
                f"Height differs from home by more than {self.max_height_delta:g} m."
            )

        tilt_rotation = _rot_y(pitch) @ _rot_x(roll)
        a, b = tilt_rotation[0, 0], tilt_rotation[0, 1]
        c, d = tilt_rotation[1, 0], tilt_rotation[1, 1]

        # For symmetric three-actuator radial planes, yaw making the summed
        # lateral error zero is given by the polar rotation of the 2x2 matrix.
        # This is a *passive* yaw, not a fourth commanded degree of freedom.
        yaw = math.atan2(b - c, a + d)
        rotation = _rot_z(yaw) @ tilt_rotation

        # Local platform joints are at radii along each actuator direction.
        local = np.column_stack((
            self.joint_radius * self.u,
            np.full(3, self.joint_z_offset),
        ))
        rotated = (rotation @ local.T).T

        # Find common x,y center translation that places all three rigid-plate
        # joints in their respective radial planes, tangent_i · joint_xy = 0.
        shift_xy, _, rank, _ = np.linalg.lstsq(
            self.tangent,
            -np.sum(self.tangent * rotated[:, :2], axis=1),
            rcond=None,
        )
        if rank < 2:
            raise KinematicsError("Actuator-plane geometry is singular.")

        center = np.array([shift_xy[0], shift_xy[1], height])
        joints = rotated + center
        out_of_plane = np.sum(self.tangent * joints[:, :2], axis=1)
        if np.max(np.abs(out_of_plane)) > 1e-8:
            raise KinematicsError(
                "Requested pose cannot satisfy planar actuator constraints."
            )
        return joints, center, math.degrees(yaw)

    def _motor_angles(self, joints: np.ndarray) -> np.ndarray:
        """Solve each slide's two-link radial/vertical IK in radians.

        In local actuator coordinates:
          r = base_radius_i + L1*cos(theta_motor) - L2*cos(theta_upper)
          z =                 L1*sin(theta_motor) + L2*sin(theta_upper)

        Eliminating theta_upper with sin^2 + cos^2 = 1 gives
          X*cos(theta_motor) + Z*sin(theta_motor)
            = (X^2 + Z^2 + L1^2 - L2^2) / (2*L1)
        where X = r - base_radius_i, Z = z.
        """
        motor_angles = []
        for i in range(3):
            r = float(np.dot(self.u[i], joints[i, :2]))
            z = float(joints[i, 2])
            X = r - self.base_radii[i]
            Z = z
            lower = self.lower_lengths[i]
            upper = self.upper_lengths[i]
            reach = math.hypot(X, Z)
            if reach < 1e-12:
                raise KinematicsError(f"Actuator {i + 1}: singular target at motor pivot.")

            cosine_argument = (
                (X * X + Z * Z + lower * lower - upper * upper)
                / (2.0 * lower * reach)
            )
            if cosine_argument < -1.0 - 1e-9 or cosine_argument > 1.0 + 1e-9:
                raise KinematicsError(
                    f"Actuator {i + 1}: unreachable joint position "
                    f"(radial={r:.5f} m, z={z:.5f} m)."
                )
            cosine_argument = float(np.clip(cosine_argument, -1.0, 1.0))
            alpha = math.atan2(Z, X)
            beta = math.acos(cosine_argument)
            theta = alpha + self.elbow_branches[i] * beta

            # Verify target-to-elbow rod length, for numerical sanity.
            elbow_r = self.base_radii[i] + lower * math.cos(theta)
            elbow_z = lower * math.sin(theta)
            rod_length = math.hypot(r - elbow_r, z - elbow_z)
            if abs(rod_length - upper) > 1e-8:
                raise KinematicsError(f"Actuator {i + 1}: IK consistency failure.")
            motor_angles.append(theta)

        return np.asarray(motor_angles, dtype=float)

    def calculate_servo_angles(
        self, roll_deg: float, pitch_deg: float, height_m: float | None = None
    ) -> tuple[float, float, float]:
        """Return commanded servo angles in *degrees*; does not move hardware.

        A solved motor angle theta is mapped using the calibrated neutral:
          servo = neutral + direction * (theta - theta_home).
        """
        joints, _, _ = self.platform_joints(roll_deg, pitch_deg, height_m)
        motors = self._motor_angles(joints)
        offsets_deg = np.rad2deg(motors - self.home_motor_angles)
        angles = np.asarray(self.neutral_servo) + np.asarray(self.directions) * offsets_deg

        for i, angle in enumerate(angles):
            if not (self.minimum_servo[i] - 1e-9 <= angle <= self.maximum_servo[i] + 1e-9):
                raise KinematicsError(
                    f"Servo {i + 1}: calculated {angle:.2f} deg outside "
                    f"calibrated [{self.minimum_servo[i]:g}, "
                    f"{self.maximum_servo[i]:g}] deg. Command rejected."
                )
        return tuple(float(v) for v in angles)

    def format_serial_command(self, angles_deg: tuple[float, float, float]) -> bytes:
        """Format values for the existing Arduino comma-separated line protocol.

        This prepares bytes only. It never opens a serial port. The current
        Arduino sketch accepts integer degree triplets, e.g. b'15,16,14\\n'.
        """
        if len(angles_deg) != 3:
            raise KinematicsError("Expected exactly three servo angles.")
        values = tuple(int(round(float(a))) for a in angles_deg)
        for i, v in enumerate(values):
            if not self.minimum_servo[i] <= v <= self.maximum_servo[i]:
                raise KinematicsError(f"Rounded servo {i + 1} angle out of bounds.")
        return (",".join(map(str, values)) + "\n").encode("ascii")


def _synthetic_self_test() -> None:
    """Run math-only tests with FICTIONAL dimensions, never for hardware."""
    print("SYNTHETIC SELF-TEST ONLY. DO NOT USE THESE VALUES FOR HARDWARE.")
    fake_geometry = {
        "geometry": {
            "base_motor_radii_m": [0.090] * 3,
            "platform_joint_radius_m": 0.070,
            "lower_link_lengths_m": [0.035] * 3,
            "upper_link_lengths_m": [0.090] * 3,
            "home_height_m": 0.095,
            "max_tilt_deg": 3.0,
            "max_height_delta_m": 0.005,
        },
        "actuators": {"elbow_branches": [-1] * 3, "servo_directions": [1] * 3},
    }
    fake_cal = {"servos": {
        "neutral_angles_deg": [15] * 3,
        "min_angles_deg": [0] * 3,
        "max_angles_deg": [30] * 3,
    }}
    k = StewartKinematics(fake_geometry, fake_cal)
    home = k.calculate_servo_angles(0, 0)
    assert np.allclose(home, [15] * 3, atol=1e-9), home
    print("Home servo angles:", tuple(round(x, 3) for x in home))
    for roll, pitch in [(1, 0), (0, 1), (1, -1), (-2, 2)]:
        joints, center, yaw = k.platform_joints(roll, pitch)
        targets = k.calculate_servo_angles(roll, pitch)
        assert np.max(np.abs(np.sum(k.tangent * joints[:, :2], axis=1))) < 1e-10
        assert np.allclose(
            np.linalg.norm(joints[:, None, :] - joints[None, :, :], axis=2)[np.triu_indices(3, 1)],
            math.sqrt(3) * k.joint_radius,
            atol=1e-10,
        )
        print(
            f"Roll {roll:+g}, pitch {pitch:+g} deg -> "
            f"servos {tuple(round(v, 3) for v in targets)}; "
            f"induced yaw {yaw:+.5f} deg"
        )
    try:
        k.calculate_servo_angles(4, 0)
    except KinematicsError:
        print("Unsafe tilt correctly rejected.")
    else:
        raise AssertionError("Safety tilt test failed")
    print("All synthetic tests passed.")


def main() -> None:
    parser = argparse.ArgumentParser(description="3-servo platform inverse kinematics (no hardware I/O)")
    parser.add_argument("--roll", type=float, default=0.0, help="Desired roll in degrees")
    parser.add_argument("--pitch", type=float, default=0.0, help="Desired pitch in degrees")
    parser.add_argument("--height", type=float, default=None, help="Optional center height in meters")
    parser.add_argument("--self-test", action="store_true", help="Run synthetic math-only tests")
    args = parser.parse_args()
    if args.self_test:
        _synthetic_self_test()
        return
    try:
        solver = StewartKinematics.from_files()
        angles = solver.calculate_servo_angles(args.roll, args.pitch, args.height)
        joints, center, yaw = solver.platform_joints(args.roll, args.pitch, args.height)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(f"Requested roll={args.roll:+.3f} deg, pitch={args.pitch:+.3f} deg")
    print(f"Solved plate center (m): {np.round(center, 6).tolist()}")
    print(f"Passive yaw: {yaw:+.6f} deg")
    print("Servo 1, 2, 3 (deg):", tuple(round(v, 3) for v in angles))
    print("Arduino serial bytes:", repr(solver.format_serial_command(angles)))
    print("NO motor commands have been sent.")


if __name__ == "__main__":
    main()
