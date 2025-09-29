#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2
import cv_bridge
import tf_transformations
import time
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from pick_and_place.msg import DetectedObjectsStamped, DetectedObject

class VisionObjectDetector(Node):
    def __init__(self):
        super().__init__('vision_object_detector')
        
        self.color_ranges = {"blue": [np.array([110,50,50]), np.array([130,255,255])],
                             "green": [np.array([36, 25, 25]), np.array([70, 255,255])],
                             "red": [np.array([0, 100, 100]), np.array([10, 255, 255])],
                             "black": [np.array([0,0,0]), np.array([180, 255, 40])]}
        self.block_contour_area_threshold = 200
        self.blocks_on_workbench = []
        
        self.bridge = cv_bridge.CvBridge()
        
        # temporary subscriber for getting messages
        self.cam_info = self.wait_for_message('/camera/color/camera_info', CameraInfo)
        self.pin_cam = PinholeCameraModel()
        self.pin_cam.fromCameraInfo(self.cam_info)

        color_image = self.wait_for_message('/camera/color/image_raw', Image)
        self.image_height, self.image_width, _ = self.bridge.imgmsg_to_cv2(color_image, 'bgr8').shape

        self.depth_image = self.bridge.imgmsg_to_cv2(self.wait_for_message('/camera/depth/image_raw', Image), '32FC1')

        self.T_c_w, self.T_w_c = self.get_camera_homogeneous_tranforms()
        self.workbench_depth = self.get_workbench_depth()
        
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.detected_objects_pub = self.create_publisher(DetectedObjectsStamped, '/object_detection', 10)
        self.get_logger().info("Vision Object Detector node has started.")

    def wait_for_message(self, topic, msg_type, timeout=10.0):
        """Generic method to wait for a single message on a topic."""
        self.get_logger().info(f"Waiting for message on topic: {topic}")
        future = rclpy.Future()
        
        def cb(msg):
            future.set_result(msg)
            self.destroy_subscription(sub)

        sub = self.create_subscription(msg_type, topic, cb, 1)
      
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
             if future.done():
                 return future.result()
             executor.spin_once(timeout_sec=0.1)
        raise RuntimeError(f"Timeout waiting for message on topic {topic}")


    def get_camera_homogeneous_tranforms(self):
        camera_origin = self.get_model_position_from_gazebo("kinect")
        Rot_c_w = np.array([[0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        Transl_c_w = tf_transformations.translation_matrix(camera_origin)
        T_c_w = np.dot(Transl_c_w, Rot_c_w)
        T_w_c = tf_transformations.inverse_matrix(T_c_w)
        return T_c_w, T_w_c

    def get_model_position_from_gazebo(self, model_name: str):
        client = self.create_client(GetModelState, '/gazebo/get_model_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/get_model_state not available, Waiting...')
        
        req = GetModelState.Request()
        req.model_name = model_name
        req.relative_entity_name = "world"
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            pos = future.result().pose.position
            return pos.x, pos.y, pos.z
        else:
            self.get_logger().error(f"Failed to get model state for {model_name}")
            return None, None, None

    def get_workbench_depth(self) -> float:
        image = self.bridge.imgmsg_to_cv2(self.wait_for_message('/camera/color/image_raw', Image), 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(hsv, "black")
        
        indices = np.where(mask == 255)
        if len(indices[0]) > 0:
            u, v = indices[1][0], indices[0][0] #u is x which is (width), v is y which is (height)
            return self.get_pixel_depth(u, v)
        return 0.0

    def get_mask(self, hsv, color):
        return cv2.inRange(hsv, self.color_ranges[color][0], self.color_ranges[color][1])

    def get_pixel_depth(self, u, v):
        u, v = int(u), int(v)
        if u >= self.image_width: u = self.image_width - 1
        if v >= self.image_height: v = self.image_height - 1
        return self.depth_image[v, u]

    def image_callback(self, msg: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        self.blocks_on_workbench = []
        for color in ["red", "green", "blue"]:
            mask = self.get_mask(hsv, color)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.block_contour_area_threshold:
                    M = cv2.moments(contour)
                    cx = int(M['m10'] / (M['m00'] + 1e-6))
                    cy = int(M['m01'] / (M['m00'] + 1e-6))
                    _, _, w, h = cv2.boundingRect(contour)
                    depth = self.get_pixel_depth(cx, cy)
                    height = self.workbench_depth - depth
                    self.blocks_on_workbench.append(((cx, cy, w, h), height, color))
                    
                    cv2.rectangle(image, (cx-int(w/2),cy-int(h/2)), (cx+int(w/2),cy+int(h/2)), (255,255,255), 1)
                    cv2.putText(image, f"{color} ({cx}, {cy})", (cx-45, cy+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Camera View", image)
        cv2.waitKey(1)
        self.publish_detected_objects()
    
    def publish_detected_objects(self) -> None:
        msg = DetectedObjectsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for box_props, height, color in self.blocks_on_workbench:
            cx, cy, w, h = box_props
            x_world, y_world, _ = self.get_3d_point_from_pixel(cx, cy)
            
            detected_obj = DetectedObject()
            detected_obj.x_world = x_world
            detected_obj.y_world = y_world
            detected_obj.height = height
            detected_obj.color = color
            msg.detected_objects.append(detected_obj)
            
        self.detected_objects_pub.publish(msg)

    def get_3d_point_from_pixel(self, u, v):
        depth = self.get_pixel_depth(u, v)
        height = self.workbench_depth - depth
        ray = self.pin_cam.projectPixelTo3dRay((u, v))
        
        x_cam = (ray[0] / ray[2]) * (depth + height / 2)
        y_cam = (ray[1] / ray[2]) * (depth + height / 2)
        z_cam = depth + height/2
        
        point_cam = np.array([x_cam, y_cam, z_cam, 1])
        point_world = np.dot(self.T_c_w, point_cam)
        return point_world[0], point_world[1], point_world[2]

def main(args=None):
    rclpy.init(args=args)
    node = VisionObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
