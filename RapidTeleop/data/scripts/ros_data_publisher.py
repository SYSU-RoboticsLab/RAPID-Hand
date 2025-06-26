import rospy
from sensor_msgs.msg import JointState, Image, PointCloud2, PointField
from std_msgs.msg import Header, UInt8MultiArray, MultiArrayLayout, MultiArrayDimension
from cv_bridge import CvBridge
import numpy as np
from threading import Thread, Lock
import cv2


class RobotDataPublisher:
    def __init__(
        self,
        rate_hz=25,
        init_node: bool = True,
        joint_topic: str = "/robot2/joint_states",
        rgb_image_topic: str = "/camera_image",
        tactile_topic: str = "/tactile_data",
        tactile_image_topic: str = "/tactile_image",
        tactile_point_cloud_topic: str = "/tactile_point_cloud",
    ):
        if init_node:
            rospy.init_node("robot_data_publisher")

        # Publishers
        self.joint_pub = rospy.Publisher(joint_topic, JointState, queue_size=10)

        self.image_pub = rospy.Publisher(rgb_image_topic, Image, queue_size=10)
        self.tactile_pub = rospy.Publisher(
            tactile_topic, UInt8MultiArray, queue_size=10
        )
        self.tactile_image_pub = rospy.Publisher(
            tactile_image_topic, Image, queue_size=10
        )
        self.point_cloud_pub = rospy.Publisher(
            tactile_point_cloud_topic, PointCloud2, queue_size=10
        )

        self.bridge = CvBridge()

        # Data and locks
        self.joint_angles = []
        self.joint_names = []
        self.image = None
        self.tactile_data = np.array([])
        self.tactile_data_3D = np.array([])
        self.lock = Lock()

        # Threading and rate
        self.rate_hz = rate_hz
        self.thread = None
        self.running = False

        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        self.tactile_msg_layout = MultiArrayLayout()
        self.tactile_msg_layout.dim = [
            MultiArrayDimension(label="dim0", size=5, stride=5 * 12 * 8),
            MultiArrayDimension(label="dim1", size=12, stride=12 * 8),
            MultiArrayDimension(label="dim2", size=8, stride=8),
        ]

    def update_data(
        self, joint_angles, joint_names, image, tactile_data, tactile_data_3D
    ):
        """Update the data to be published."""
        with self.lock:
            self.joint_angles = joint_angles
            self.joint_names = joint_names
            self.image = image
            self.tactile_data = tactile_data
            self.tactile_data_3D = tactile_data_3D

    def publish_joint_angles(self):
        """Publish joint angles as JointState message."""
        with self.lock:
            if len(self.joint_angles) and len(self.joint_names):
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_angles
                self.joint_pub.publish(joint_state_msg)
            else:
                return

    def publish_image(self):
        """Publish image as ROS Image message."""
        with self.lock:
            if self.image is None:
                return
            ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="rgb8")
        self.image_pub.publish(ros_image)

    def publish_tactile_data(self):
        """Publish tactile data as UInt8MultiArray."""
        with self.lock:
            if self.tactile_data.size == 0:
                return
            tactile_msg = UInt8MultiArray()
            tactile_msg.data = self.tactile_data.flatten().tolist()
            tactile_msg.layout = self.tactile_msg_layout
        self.tactile_pub.publish(tactile_msg)

    def publish_tactile_image(self):
        """Publish tactile data as an image."""
        with self.lock:
            if self.tactile_data.size == 0:
                return

            # Generate an image from the tactile data
            tactile_data_max_value = 140
            tactile_images = []
            separator_column = np.linspace(
                0, tactile_data_max_value, self.tactile_data.shape[1], dtype=np.uint8
            )
            separator_column = cv2.applyColorMap(separator_column, cv2.COLORMAP_JET)

            for i in range(self.tactile_data.shape[0]):
                normalized_data = cv2.normalize(
                    self.tactile_data[i],
                    None,
                    0,
                    tactile_data_max_value,
                    cv2.NORM_MINMAX,
                ).astype(np.uint8)

                # Apply color map to create a heatmap
                tactile_image = cv2.applyColorMap(normalized_data, cv2.COLORMAP_JET)

                # Insert the gradient column between tactile images
                if tactile_images:
                    tactile_images.append(separator_column)

                tactile_images.append(tactile_image)

            # Combine all tactile sensor images horizontally
            combined_image = cv2.hconcat(tactile_images)

            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(combined_image, encoding="bgr8")
        self.tactile_image_pub.publish(ros_image)

    def publish_tactile_point_cloud(self):
        """Publish tactile_data_3D as a PointCloud2 message."""
        with self.lock:
            if self.tactile_data_3D.size == 0:
                return

            # Create the header for the PointCloud2 message
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "robot2/BASE"  # Adjust to your coordinate frame

            # Extract positions and values
            positions = self.tactile_data_3D[:, :, :, :3].reshape(
                -1, 3
            )  # Flatten (x, y, z)
            values = self.tactile_data_3D[:, :, :, 3].reshape(-1)  # Flatten values

            # Normalize values and map to RGB
            normalized_values = np.clip(values / 200.0, 0, 1)  # Normalize to [0, 1]
            colormap = cv2.applyColorMap(
                (normalized_values * 255).astype(np.uint8), cv2.COLORMAP_JET
            )
            rgb_values = (
                (colormap[:, 0, 2].astype(np.uint32) << 16)  # Red
                | (colormap[:, 0, 1].astype(np.uint32) << 8)  # Green
                | (colormap[:, 0, 0].astype(np.uint32))  # Blue
            )

            # Combine positions and RGB
            points = np.zeros((positions.shape[0], 4), dtype=np.float32)
            points[:, :3] = positions
            points[:, 3] = rgb_values.view(np.float32)  # Pack RGB as float32

            # Convert to PointCloud2
            fields = (
                self.point_fields
            )  # Precomputed PointField list during initialization
            point_cloud_msg = PointCloud2(
                header=header,
                height=1,
                width=points.shape[0],
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=16,
                row_step=16 * points.shape[0],
                data=points.tobytes(),
            )

            self.point_cloud_pub.publish(point_cloud_msg)

    def start(self):
        """Start the publishing thread."""
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            self.thread = Thread(target=self.run)
            self.thread.start()

    def stop(self):
        """Stop the publishing thread."""
        self.running = False
        if self.thread is not None:
            self.thread.join()

    def run(self):
        """Main loop to publish data periodically."""
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.running:
            self.publish_joint_angles()
            self.publish_image()
            self.publish_tactile_data()
            self.publish_tactile_image()
            self.publish_tactile_point_cloud()  # Publish tactile point cloud
            rate.sleep()
