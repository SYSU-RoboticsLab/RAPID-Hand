import os
import json
import numpy as np
from datetime import datetime
from .dynamixel_client import DynamixelClient


class RapidDriver:
    """
    A control interface for a Dynamixel-based hand or manipulator.

    Responsibilities:
    - Initialize servo parameters from a JSON config (e.g., PID gains, current limit).
    - Read joint states (position, velocity, current).
    - Convert RAPID pose vectors to servo commands using joint mapping.
    - Save/restore configuration for reuse and tuning.
    """

    def __init__(self, config_path=None, port="/dev/ttyUSB0", init_set=False):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "rapid_full.json")
        self.config_path = config_path
        self.config = self._load_config(config_path)

        # Initial values
        self.jointmap = self.config["init"]["jointmap"]
        self.kP = np.array(self.config["init"]["kP"], dtype=np.uint16)
        self.kI = np.array(self.config["init"]["kI"], dtype=np.uint16)
        self.kD = np.array(self.config["init"]["kD"], dtype=np.uint16)
        self.curr_lim = np.array(self.config["init"]["curr_lim"], dtype=np.uint16)
        self.init_pos = np.array(self.config["init"]["init_pos"], dtype=np.float32)

        self.prev_pos = self.curr_pos = self.init_pos.copy()
        self.motors = list(range(len(self.init_pos)))

        try:
            self.dxl_client = DynamixelClient(self.motors, port, baudrate=4000000)
            self.dxl_client.connect()
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Dynamixel on {port}: {e}")

        # Motor configuration
        self._apply_initial_motor_config()

        if init_set:
            self.dxl_client.write_desired_pos(self.motors, self.init_pos)

    def _apply_initial_motor_config(self):
        """Apply torque settings, PID gains, and current limits."""
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, self.kP, 84, 2)
        self.dxl_client.sync_write(self.motors, self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, self.kD, 80, 2)
        self.dxl_client.sync_write(self.motors, self.curr_lim, 102, 2)

    def _load_config(self, file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Config file not found: {file_path}")
        with open(file_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def save_config(self, save_path=None):
        """Save the current PID gains and state to a JSON file."""
        if save_path is None:
            os.makedirs("args", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"args/rapid_{timestamp}.json"

        config_to_save = {
            "init": {
                "kP": self.kP.tolist(),
                "kI": self.kI.tolist(),
                "kD": self.kD.tolist(),
                "curr_lim": self.curr_lim.tolist(),
                "pos": self.curr_pos.tolist(),
            }
        }

        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(config_to_save, f, indent=4)
        print(f"[✓] Configuration saved to {save_path}")

    def set_servo(self, pose: np.ndarray):
        """Send raw joint command (in radians) to all motors."""
        self.dxl_client.write_desired_pos(self.motors, pose)

    def set_pos(self, pose: np.ndarray):
        """Map RAPID pose (e.g., 20 DOF) into servo positions and send to hardware."""
        assert len(pose) == len(self.jointmap), "Invalid pose vector length."

        self.prev_pos = self.curr_pos.copy()
        temp = np.zeros_like(self.init_pos)

        for i, mappings in enumerate(self.jointmap):
            for motor_idx, weight in mappings:
                temp[motor_idx] += pose[i] * weight

        self.curr_pos = self.init_pos + temp
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def read_pos_vel_cur(self):
        """Returns a RAPID pose vector (backmapped) and servo states."""
        pos, vel, cur = self.dxl_client.read_pos_vel_cur()
        delta = pos - self.init_pos
        pose = self._backproject_pose(delta)
        return pose, vel, cur

    def read_pos(self):
        delta = self.dxl_client.read_pos() - self.init_pos
        return self._backproject_pose(delta)

    def read_vel(self):
        return self.dxl_client.read_vel()

    def read_cur(self):
        return self.dxl_client.read_cur()

    def _backproject_pose(self, delta: np.ndarray) -> np.ndarray:
        """Convert joint angles (rad) to abstracted RAPID pose space."""
        pose = np.zeros(len(self.jointmap))
        for i, mappings in enumerate(self.jointmap):
            for motor_idx, weight in mappings:
                pose[i] += delta[motor_idx] / weight / len(mappings)
        return pose

    def setkP(self, data):
        self._update_gain("kP", 84, data)

    def setkI(self, data):
        self._update_gain("kI", 82, data)

    def setkD(self, data):
        self._update_gain("kD", 80, data)

    def set_curr_lim(self, data):
        self._update_gain("curr_lim", 102, data)

    def _update_gain(self, name: str, address: int, data):
        """Update a specific control parameter (PID or current limit)."""
        setattr(self, name, np.array(data, dtype=np.uint16))
        self.dxl_client.sync_write(self.motors, getattr(self, name), address, 2)
