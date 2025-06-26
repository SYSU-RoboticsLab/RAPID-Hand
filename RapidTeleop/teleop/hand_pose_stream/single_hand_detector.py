import numpy as np
import mediapipe as mp
from mediapipe.framework import formats
from mediapipe.framework.formats import landmark_pb2
from mediapipe.python.solutions.drawing_utils import DrawingSpec
from mediapipe.python.solutions.hands import HandLandmark
from mediapipe.python.solutions import hands_connections


# Coordinate transformation from camera/operator to MANO for right hand
OPERATOR_TO_MANO_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
OPERATOR_TO_MANO_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])


class SingleHandDetector:
    def __init__(
        self,
        hand_type: str = "Right",
        min_detection_confidence: float = 0.8,
        min_tracking_confidence: float = 0.8,
        selfie: bool = False,
    ):
        """
        Initializes a single hand detector using MediaPipe.

        Args:
            hand_type: 'Right' or 'Left'.
            min_detection_confidence: Minimum confidence for detection.
            min_tracking_confidence: Minimum confidence for tracking.
            selfie: If True, does not flip hand type.
        """
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.selfie = selfie
        self.operator2mano = (
            OPERATOR_TO_MANO_RIGHT if hand_type == "Right" else OPERATOR_TO_MANO_LEFT
        )
        hand_flip = {"Right": "Left", "Left": "Right"}
        self.detected_hand_type = hand_type if selfie else hand_flip[hand_type]

    @staticmethod
    def draw_skeleton_on_image(image, keypoint_2d: landmark_pb2.NormalizedLandmarkList, style: str = "white"):
        """
        Draws hand landmarks and connections on an image.

        Args:
            image: Input image.
            keypoint_2d: 2D hand landmarks.
            style: 'default' or 'white'.
        """
        if style == "default":
            mp.solutions.drawing_utils.draw_landmarks(
                image,
                keypoint_2d,
                mp.solutions.hands.HAND_CONNECTIONS,
                mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                mp.solutions.drawing_styles.get_default_hand_connections_style(),
            )
        elif style == "white":
            landmark_style = {
                landmark: DrawingSpec(color=(255, 48, 48), circle_radius=4, thickness=-1)
                for landmark in HandLandmark
            }
            connection_style = {
                pair: DrawingSpec(thickness=2) for pair in hands_connections.HAND_CONNECTIONS
            }
            mp.solutions.drawing_utils.draw_landmarks(
                image,
                keypoint_2d,
                mp.solutions.hands.HAND_CONNECTIONS,
                landmark_style,
                connection_style,
            )
        return image

    def detect(self, rgb_image: np.ndarray):
        """
        Detects hand landmarks from an RGB image.

        Returns:
            num_hands: number of detected hands (max 1)
            joint_pos: np.ndarray(21,3) in MANO frame
            keypoint_2d: 2D landmarks (MediaPipe format)
            wrist_rotation: rotation matrix from world to wrist frame
        """
        results = self.hand_detector.process(rgb_image)
        if not results.multi_hand_landmarks:
            return 0, None, None, None

        for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
            label = results.multi_handedness[i].classification[0].label
            if label == self.detected_hand_type:
                keypoint_3d = results.multi_hand_world_landmarks[i]
                keypoint_2d = results.multi_hand_landmarks[i]
                keypoint_3d_array = self.parse_keypoint_3d(keypoint_3d)
                keypoint_3d_array -= keypoint_3d_array[0]  # Normalize to wrist
                wrist_rot = self.estimate_frame_from_hand_points(keypoint_3d_array)
                joint_pos = keypoint_3d_array @ wrist_rot @ self.operator2mano
                return 1, joint_pos, keypoint_2d, wrist_rot

        return 0, None, None, None

    @staticmethod
    def parse_keypoint_3d(keypoint_3d: formats.landmark_pb2.LandmarkList) -> np.ndarray:
        """
        Converts MediaPipe 3D landmark list to NumPy array (21, 3).
        """
        return np.array([[lm.x, lm.y, lm.z] for lm in keypoint_3d.landmark])

    @staticmethod
    def parse_keypoint_2d(keypoint_2d: landmark_pb2.NormalizedLandmarkList, img_size: tuple) -> np.ndarray:
        """
        Converts 2D normalized landmarks to pixel coordinates.

        Args:
            keypoint_2d: Normalized landmark list (x,y in [0,1]).
            img_size: (height, width)

        Returns:
            NumPy array of shape (21, 2)
        """
        h, w = img_size
        return np.array([[lm.x * w, lm.y * h] for lm in keypoint_2d.landmark])

    @staticmethod
    def estimate_frame_from_hand_points(keypoints: np.ndarray) -> np.ndarray:
        """
        Estimate wrist rotation matrix from 3 hand points:
            wrist (0), index base (5), middle base (9)

        Returns:
            3x3 rotation matrix
        """
        assert keypoints.shape == (21, 3)

        wrist = keypoints[0]
        index_base = keypoints[5]
        middle_base = keypoints[9]

        # X axis: wrist → middle
        x = wrist - middle_base

        # Estimate palm normal from 3 points
        points = keypoints[[0, 5, 9]]
        centered = points - points.mean(axis=0)
        _, _, vh = np.linalg.svd(centered)
        normal = vh[2]

        # Gram-Schmidt orthogonalization
        x -= np.dot(x, normal) * normal
        x /= np.linalg.norm(x)

        z = np.cross(x, normal)

        if np.dot(z, index_base - middle_base) < 0:
            normal *= -1
            z *= -1

        return np.stack([x, normal, z], axis=1)
