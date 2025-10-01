from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import supervision as sv
import tf2_geometry_msgs
from tf2_ros import buffer
from tf2_ros import TransformListener
from ultralytics import YOLO


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.declare_params()
        self.get_params()

        self.tf_buffer = buffer.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.model = YOLO(self.model_path)

        self.rgb_subcriber = Subscriber(
            self,
            Image,
            self.rgb_topic
        )

        self.depth_subcriber = Subscriber(
            self,
            Image,
            self.depth_topic
        )

        self.camera_info_subcriber = Subscriber(
            self,
            CameraInfo,
            self.camera_info
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            fs=[self.rgb_subcriber, self.depth_subcriber, self.camera_info_subcriber],
            queue_size=self.queue_size,
            slop=self.sync_slop  # Allow 100ms difference between timestamps
        )

        self.synchronizer.registerCallback(self.synchronized_callback)

        self.image_publisher = self.create_publisher(Image, 'detection/image', 10)

        self.get_logger().info("Detection node initialized with synchronized RGB and Depth images.")

    def declare_params(self):
        """
        Declare ROS2 node parameters for camera configuration, synchronization,
        visualization, and object detection.
        """
        # TF config
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')

        # Camera config
        self.declare_parameter('rgb_topic', 'camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', 'camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info', 'camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('depth_scale', 1.0)

        # Message synchronization config
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)

        # OpenCV config
        self.declare_parameter('visualize_rgb_image', True)
        self.declare_parameter('visualize_depth_image', False)
        self.declare_parameter('visualize_center_object', True)
        self.declare_parameter('center_object_color', [0, 255, 0])
        self.declare_parameter('visualize_center_image', True)
        self.declare_parameter('center_image_color', [255, 0, 0])

        # Detection config
        self.declare_parameter('model_path', 'mobi_yolo/yolo_detection/weights/apple.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('iou', 0.7)
        self.declare_parameter('publish_poses', True)
        self.declare_parameter('sphere_shape', False)

    def get_params(self):
        """
        Retrieve and store all declared ROS2 parameters.
        """
        # TF config
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Camera config
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info = self.get_parameter('camera_info').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value

        # Message synchronization config
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        # OpenCV config
        self.visualize_rgb_image = self.get_parameter(
            'visualize_rgb_image').get_parameter_value().bool_value
        self.visualize_depth_image = self.get_parameter(
            'visualize_depth_image').get_parameter_value().bool_value
        self.visualize_center_object = self.get_parameter(
            'visualize_center_object').get_parameter_value().bool_value
        self.center_object_color = self.get_parameter(
            'center_object_color').get_parameter_value().integer_array_value
        self.visualize_center_image = self.get_parameter(
            'visualize_center_image').get_parameter_value().bool_value
        self.center_image_color = self.get_parameter(
            'center_image_color').get_parameter_value().integer_array_value

        # Detection config
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().double_value
        self.iou = self.get_parameter('iou').get_parameter_value().double_value
        self.publish_poses = self.get_parameter('publish_poses').get_parameter_value().bool_value
        self.sphere_shape = self.get_parameter('sphere_shape').get_parameter_value().bool_value

    def synchronized_callback(self, rgb_msg, depth_msg, camera_info_msg):
        """
        Main callback function for image processing and object detection.
        """
        try:
            # Get intrinsic parameters from topic
            self.header = camera_info_msg.header
            self.width = camera_info_msg.width
            self.height = camera_info_msg.height
            self.fx = camera_info_msg.k[0]
            self.fy = camera_info_msg.k[4]
            self.cx = camera_info_msg.k[2]
            self.cy = camera_info_msg.k[5]

            # Convert ROS Image messages to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )

            rgb_image, poses = self.get_detections(rgb_image, depth_image / self.depth_scale)

            if self.publish_poses:
                self.publish_detections(rgb_image, poses)

            if self.visualize_depth_image or self.visualize_rgb_image:
                self.visualize_images(rgb_image, depth_image)

        except Exception as e:
            self.get_logger().error(f'Error processing synchronized images: {str(e)}')

    def get_detections(self, rgb_image, depth_image):
        """
        Perform object detection on RGB image.

        Returns:
            tuple: Annotated RGB image + list of detected object poses
        """
        results = self.model.predict(
            source=rgb_image,
            conf=self.conf,
            save=False,
            imgsz=(self.width, self.height)
        )[0]

        detections = sv.Detections.from_ultralytics(results)
        rgb_image = self.bounding_box_annotator.annotate(scene=rgb_image, detections=detections)

        pose = []

        for bbox in detections.xyxy:
            pose.append(self.get_position(depth_image, bbox))

            text_position = (int(bbox[0]), int(bbox[1]) - 10)
            x, y, z = pose[-1]
            x_center = int((bbox[0] + bbox[2]) / 2)
            y_center = int((bbox[1] + bbox[3]) / 2)

            # For reference, mark the center of the bounding box
            if self.visualize_center_object:
                rgb_image = cv2.circle(
                    img=rgb_image,
                    center=(x_center, y_center),
                    radius=5,
                    color=self.center_image_color,
                    thickness=-1,
                )

            rgb_image = cv2.putText(
                color=(0, 255, 0),
                img=rgb_image,
                text=f'X:{x:.3f} Y:{y:.3f} Z:{z:.3f} m',
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                org=text_position,
                thickness=1,
            )

        return rgb_image, pose

    def get_position(self, depth_image, bbox):
        """
        Estimate the 3D position of an object from its bounding box and depth information.
        Reference: https://www.mdpi.com/2073-4395/13/7/1816

        Returns:
            list: [X, Y, Z] (m) in base frame coordinates
        """
        # Get the depth value at the center and adjacents points of the bounding box
        x_center = int((bbox[0] + bbox[2]) / 2)
        y_center = int((bbox[1] + bbox[3]) / 2)
        points = [
            (x_center, y_center),
            (x_center - 1, y_center),
            (x_center + 1, y_center),
            (x_center, y_center - 1),
            (x_center, y_center + 1),
        ]

        # Depth mean value from the points
        depths = [depth_image[y, x] for x, y in points if depth_image[y, x] > 0]
        z0 = sum(depths) / len(depths)

        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]

        # Estimate sphere radius if treating object as spherical (e.g., apples)
        r = (z0 * w / (2 * self.fx)) if w >= h else (z0 * h / (2 * self.fy))

        X = z0 * (x_center - self.cx) / self.fx
        Y = z0 * (y_center - self.cy) / self.fy
        Z = z0 + r if self.sphere_shape else z0

        X, Y, Z = self.transform_to_base([Z, -X, -Y])

        return [X, Y, Z]

    def transform_to_base(self, position):
        """
        Transform 3D position from camera frame to robot base frame using TF2.

        Returns:
            list: 3D position [x, y, z] in base frame (meters)
        """

        try:
            x, y, z = [pose for pose in position]

            point_stamped = PointStamped()
            point_stamped.header = self.header
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = z

            pose_transformed = self.tf_buffer.transform(
                point_stamped,
                self.base_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            point_transformed = pose_transformed.point

            return [point_transformed.x, point_transformed.y, point_transformed.z]

        except Exception as e:
            self.get_logger().error(f'TF transform error: {str(e)}')
            return position

    def publish_detections(self, rgb_image, poses):
        """
        Publish the annotated RGB image with detection results.
        """
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        self.image_publisher.publish(rgb_msg)

    def visualize_images(self, rgb_image, depth_image):
        """
        Display RGB and depth images using OpenCV windows.
        """
        if self.visualize_rgb_image:
            # For reference, mark the center of the image
            rgb_image = cv2.circle(
                img=rgb_image,
                center=(int(self.width / 2), int(self.height / 2)),
                radius=5,
                color=self.center_image_color,
                thickness=-1,
            )
            cv2.imshow('RGB', rgb_image)
        if self.visualize_depth_image:
            depth_image = cv2.normalize(
                depth_image,
                None,
                0,
                255,
                cv2.NORM_MINMAX,
                cv2.CV_8U
            )

            cv2.imshow('Depth', depth_image)

        if cv2.waitKey(1) == 113:  # q key
            self.get_logger().info('Destroying all windows...')
            cv2.destroyAllWindows()
            self.get_logger().info('All windows destroyed. Closing node.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
