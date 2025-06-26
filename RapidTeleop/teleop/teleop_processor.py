import os
import numpy as np
from scipy.spatial.transform import Rotation as R

from .hand_pose_stream import ImageInput, VisionProInput
from dex_retargeting.retargeting_config import RetargetingConfig
from control import PinocchioMotionControl
from .retargeting_method.retargeting_utils import build_retarget_tool
from .retargeting_method.hand_pose_adapter import PhalangeStretchTransform


class TeleopProcessor:
    # Predefined finger bone structures (5 fingers)
    finger_lines = [
        np.array([[0, 2], [2, 3], [3, 4]]),
        np.array([[0, 5], [5, 6], [6, 7], [7, 8]]),
        np.array([[0, 9], [9, 10], [10, 11], [11, 12]]),
        np.array([[0, 13], [13, 14], [14, 15], [15, 16]]),
        np.array([[0, 17], [17, 18], [18, 19], [19, 20]]),
    ]

    left_offset = np.zeros(3)
    right_offset = np.zeros(3)

    def __init__(self, inputstream_config, arm_config, retarget_config):
        # Initialize input stream
        stream_type = inputstream_config["input_stream_type"]
        if stream_type == "image":
            self.InputStream = ImageInput(
                input_type=inputstream_config["input_type"],
                settings=inputstream_config["settings"]
            )
        elif stream_type == "visionpro":
            self.InputStream = VisionProInput(avp_ip=inputstream_config["avp_ip"])
        else:
            raise ValueError(f"Unsupported input stream type: {stream_type}")

        # Initialize arm control
        self.left_arm_rotation = R.from_matrix(np.array(arm_config["rotation_matrix"]))
        self.right_arm_rotation = R.from_matrix(np.array(arm_config["transform_matrix"]))
        self.arm_control = PinocchioMotionControl(
            robot_config_path="", robot_config_dict=arm_config
        )
        self.arm_joint_names = self.arm_control.get_joint_names()
        initial_qpos = np.array([
            arm_config["init_qpos"][name] for name in self.arm_joint_names
        ])
        self.arm_control.set_current_qpos(initial_qpos)
        ee_pose = self.arm_control.compute_ee_pose(initial_qpos)
        self.set_arm_ee_position(ee_pose[:3])

        # Initialize hand retargeting configuration
        self.retarget_type = retarget_config["retargeting"]["type"]
        self.finger_lines = self.finger_lines[:retarget_config["transform"]["finger_num"]]
        self.std_to_robot_matrix = np.array(retarget_config["transform"]["standard2robot"])

        # Optional transformation for phalange stretch
        if retarget_config["transform"]["type"] == "PhalangeStretch":
            hand_trans = PhalangeStretchTransform(fingers_line=self.finger_lines)

            if stream_type in ["visionpro"]:
                hand_data = self.InputStream.read_data(
                    user_name=retarget_config["transform"]["user"]
                )[5]["hand_keypoints"]
            elif stream_type == "image":
                hand_data = self.InputStream.read_img_data(
                    user_name=retarget_config["transform"]["user"]
                )
            else:
                raise ValueError("Unsupported input stream type for transformation.")

            hand_data = np.array(hand_data) @ self.std_to_robot_matrix.T

            hand_trans.update_parameters(
                robot_link_pos_dict=retarget_config["transform"]["robot_link_pos_dict"],
                human_robot_link_map=retarget_config["transform"]["human_robot_link_map"],
                human_hand_points=hand_data,
                robot_base_link=retarget_config["transform"]["robot_base_link"],
            )
            self.hand_trans = hand_trans
        else:
            self.hand_trans = None

        # Build retargeting tool
        hand_urdf_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "assets/robots/hands")
        )
        self.right_retargeting = build_retarget_tool(retarget_config, hand_urdf_dir)
        right_indices = self.right_retargeting.optimizer.target_link_human_indices
        self.origin_indices = right_indices[0, :]
        self.task_indices = right_indices[1, :]
        self.hand_joint_names = self.right_retargeting.joint_names

    def step(self):
        """Calculate one control cycle of joint positions."""
        arm_qpos = self.calculate_arm_qpos()
        hand_qpos = self.calculate_hand_qpos()
        return np.concatenate([arm_qpos, hand_qpos])

    def calculate_arm_qpos(self):
        """Compute robot arm joint positions based on wrist pose."""
        target_pose = self.get_wrist_pose()
        self.arm_control.step(target_pose)
        return self.arm_control.get_current_qpos()

    def calculate_hand_qpos(self):
        """Compute robot hand joint positions based on hand keypoints."""
        keypoints = self.get_hand_keypoints()
        ref_vector_original = keypoints[self.task_indices] - keypoints[self.origin_indices]

        if self.hand_trans:
            transformed = self.hand_trans.apply_transformation(keypoints)
            ref_vector = transformed[self.task_indices] - transformed[self.origin_indices]
        else:
            ref_vector = ref_vector_original

        if self.retarget_type == "vectorada":
            num_constraints = self.right_retargeting.optimizer.num_constrain
            ref_vector[-num_constraints:] = ref_vector_original[-num_constraints:]

        return self.right_retargeting.retarget(ref_vector)

    def set_arm_ee_position(self, ee_position):
        """Initialize the end-effector offset for transformation."""
        wrist_pose = self.InputStream.get_wrist_pose()
        self.right_offset = ee_position - wrist_pose[:3]

    def get_wrist_pose(self):
        """Transform human wrist pose to robot frame."""
        human_pose = self.InputStream.get_wrist_pose()
        position = human_pose[:3] + self.right_offset
        human_quat = R.from_quat(human_pose[3:])
        robot_quat = self.left_arm_rotation * human_quat * self.right_arm_rotation
        return np.concatenate([position, robot_quat.as_quat()])

    def get_hand_keypoints(self):
        """Get transformed 3D hand keypoints."""
        keypoints = self.InputStream.get_hand_keypoints()
        return keypoints @ self.std_to_robot_matrix.T

    def get_robot_joint_names(self):
        return self.arm_joint_names + self.hand_joint_names

    def get_arm_joint_names(self):
        return self.arm_joint_names

    def get_hand_joint_names(self):
        return self.hand_joint_names
