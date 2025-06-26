import sapien
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer
import numpy as np
from pathlib import Path


class SimSapien:
    """
    A simple simulation environment based on SAPIEN, with URDF loading and rendering support.
    """

    def __init__(self, urdf_path: str):
        """
        Initialize the SAPIEN simulation with scene, lighting, and viewer setup.

        Args:
            urdf_path (str): Path to the robot URDF file.
        """
        # Setup SAPIEN renderer
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")

        # Create scene
        self.scene = sapien.Scene()

        # Add ground with default material
        ground_mat = sapien.render.RenderMaterial()
        ground_mat.base_color = [0.12, 0.08, 0.06, 1.0]
        ground_mat.metallic = 0.0
        ground_mat.roughness = 0.9
        ground_mat.specular = 0.8
        self.scene.add_ground(-0.2, render_material=ground_mat, render_half_size=[1000, 1000])

        # Lighting setup
        self.scene.add_directional_light(np.array([1, 1, -1]), np.array([5, 5, 5]))
        self.scene.add_point_light(np.array([2, 2, 2]), np.array([5, 5, 5]), shadow=False)
        self.scene.add_point_light(np.array([2, -2, 2]), np.array([5, 5, 5]), shadow=False)
        self.scene.set_environment_map(
            create_dome_envmap(sky_color=[0.3, 0.3, 0.3], ground_color=[0.3, 0.3, 0.3])
        )
        self.scene.add_area_light_for_ray_tracing(
            sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]),
            np.array([1, 1, 1]),
            5, 5
        )

        # Setup simulation camera
        self.cam_width = 1280
        self.cam_height = 720
        self.fovy = 1.0
        self.near = 0.1
        self.far = 10.0

        self.camera = self.scene.add_camera(
            name="Camera",
            width=self.cam_width,
            height=self.cam_height,
            fovy=self.fovy,
            near=self.near,
            far=self.far,
        )

        self.camera.set_local_pose(sapien.Pose([0, -0.5, 0], [-0.70, 0, 0, -0.75]))

        # Setup viewer
        self.viewer = Viewer()
        self.viewer.set_scene(self.scene)
        self.viewer.control_window.show_origin_frame = False
        self.viewer.control_window.move_speed = 0.01
        self.viewer.control_window.toggle_camera_lines(False)
        self.viewer.set_camera_pose(self.camera.get_local_pose())

        # Load robot from URDF
        self.loader = self.scene.create_urdf_loader()
        self.loader.load_multiple_collisions_from_file = True
        self.loader.scale = 1.4

        filepath = Path(urdf_path)
        self.robot = self.loader.load(str(filepath.resolve()))
        self.robot.set_pose(sapien.Pose([0, 0, -0.15]))

        self.sapien_joint_names = [joint.get_name() for joint in self.robot.get_active_joints()]

    def step(self, qpos: np.ndarray):
        """
        Update the robot joint positions and render the scene.

        Args:
            qpos (np.ndarray): Array of joint positions.

        Returns:
            tuple: (left_image, right_image), currently returns (None, None)
        """
        self.robot.set_qpos(qpos)

        # Render updates
        self.scene.update_render()
        for _ in range(2):
            self.viewer.render()
