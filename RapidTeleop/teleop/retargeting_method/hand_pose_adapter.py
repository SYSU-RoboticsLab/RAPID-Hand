import numpy as np
from typing import Dict, List


class PhalangeStretchTransform:
    """
    A class to scale human finger bone lengths to match the robot's link lengths
    and apply transformations to align human hand keypoints with the robot hand model.
    """

    def __init__(self, fingers_line: List[np.ndarray], init_scale: float = 1.3):
        self.reset_scaling_ratios(fingers_line, init_scale)

        # Unique keypoint indices used in all fingers
        self.finger_keypoint_indices = np.unique(
            np.hstack([np.unique(line) for line in fingers_line])
        )

        # Per-finger alignment offset vectors
        self.finger_alignment_offsets = np.zeros((5, 3))

        self.robot_link_pos_dict = None
        self.human_robot_link_map = None

        # Index of metacarpal joints in each finger
        self.metacarpal_joint_indices = [line[0][1] for line in fingers_line]

    def update_parameters(
        self,
        robot_link_pos_dict: Dict[str, np.ndarray],
        human_robot_link_map: Dict[int, str],
        human_hand_keypoints: np.ndarray,
        robot_base_link: str,
    ):
        """
        Updates the transformation parameters by computing scaling ratios and alignment offsets.
        """
        if np.max(self.finger_keypoint_indices) >= len(human_hand_keypoints):
            raise ValueError("Human keypoint index out of bounds.")

        self.robot_link_pos_dict = robot_link_pos_dict
        self.human_robot_link_map = human_robot_link_map

        # Ensure all required human indices are present in mapping
        if not all(index in human_robot_link_map for index in self.finger_keypoint_indices):
            raise ValueError("Missing keypoints in human-to-robot link map.")

        # Ensure all mapped robot links exist
        mapped_robot_links = [
            human_robot_link_map[idx] for idx in self.finger_keypoint_indices
        ]
        if not all(link in robot_link_pos_dict for link in mapped_robot_links):
            raise ValueError("Mapped robot link not found in robot_link_pos_dict.")

        # Compute segment-wise scaling ratios
        for finger_idx, finger_segments in enumerate(self.scaled_finger_lines):
            for segment_idx, (start_idx, end_idx, _) in enumerate(finger_segments):
                start_idx, end_idx = int(start_idx), int(end_idx)
                robot_vec = (
                    robot_link_pos_dict[human_robot_link_map[end_idx]] -
                    robot_link_pos_dict[human_robot_link_map[start_idx]]
                )
                human_vec = (
                    human_hand_keypoints[end_idx] - human_hand_keypoints[start_idx]
                )
                scale = np.linalg.norm(robot_vec) / np.linalg.norm(human_vec)
                self.scaled_finger_lines[finger_idx][segment_idx, 2] = scale

        # Apply transformation to calculate alignment offsets
        transformed_keypoints = self.apply_transformation(human_hand_keypoints.copy())

        for i, metacarpal_idx in enumerate(self.metacarpal_joint_indices):
            robot_vec = (
                robot_link_pos_dict[human_robot_link_map[metacarpal_idx]] -
                robot_link_pos_dict[robot_base_link]
            )
            human_vec = transformed_keypoints[metacarpal_idx]
            self.finger_alignment_offsets[i] = robot_vec - human_vec

    def reset_scaling_ratios(self, fingers_line: List[np.ndarray], init_scale: float = 1.3):
        """
        Initializes the finger segment scaling ratios to a default value.
        Each segment has format: [start_idx, end_idx, scale].
        """
        self.scaled_finger_lines = [
            np.hstack((line, np.full((line.shape[0], 1), init_scale)))
            for line in fingers_line
        ]

    def apply_transformation(self, source_keypoints: np.ndarray) -> np.ndarray:
        """
        Applies the stretch transformation to human keypoints using the
        precomputed scaling ratios and alignment vectors.
        """
        if np.max(self.finger_keypoint_indices) >= len(source_keypoints):
            raise ValueError("Keypoint index exceeds input keypoint array size.")

        keypoints = source_keypoints.copy()

        for finger_id, finger_segments in enumerate(self.scaled_finger_lines):
            scaled_vectors = (
                keypoints[finger_segments[:, 1].astype(int)] -
                keypoints[finger_segments[:, 0].astype(int)]
            ) * finger_segments[:, 2, np.newaxis]

            # Apply cumulative translation to each point along the finger
            for j, target_idx in enumerate(finger_segments[:, 1].astype(int)):
                offset = np.sum(scaled_vectors[:j + 1], axis=0)
                keypoints[target_idx] = offset + self.finger_alignment_offsets[finger_id]

        return keypoints
