#!/usr/bin/env python3
import rclpy
import numpy as np
import random
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import moveit_commander
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

# from franka_interface import GripperInterface
# from panda_robot import PandaArm

from pick_and_place.msg import DetectedObjectsStamped, DetectedObject

# Dummy Classes for Demonstration if and only if Mrinal's ROS2 Interfaces are Not Available
class PandaArm:
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Dummy PandaArm Initialized.")
    def move_to_neutral(self):
        self.node.get_logger().info("Moving to neutral.")
        time.sleep(1)
    def move_to_cartesian_pose(self, pos, ori):
        self.node.get_logger().info(f"Moving to pose: pos={pos}, ori={ori}")
        time.sleep(1)

class GripperInterface:
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Dummy GripperInterface initialized.")
    def open(self):
        self.node.get_logger().info("Gripper opening.")
        time.sleep(0.5)
    def grasp(self, width, force, speed, epsilon_inner, epsilon_outer):
        self.node.get_logger().info(f"Gripper grasping with width={width}.")
        time.sleep(0.5)
        return True


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.red_bin = (-0.5, -0.25)
        self.green_bin = (-0.5, 0.0)
        self.blue_bin = (-0.5, 0.25)

        self.workbench_height = 0.2
        self.x_offset = 0.01
        self.z_offset = 0.105
        self.z_pre_pick_offset = 0.2
        self.z_pre_place_offset = 0.2

        self.objects_on_workbench = []
        self.subscription = self.create_subscription(
            DetectedObjectsStamped,
            '/object_detection',
            self.update_objects_callback,
            10)
        
        moveit_commander.roscpp_initialize([])
        
        self.panda = PandaArm(self)                                                              
        self.gripper_interface = GripperInterface(self)    
        self.scene_interface = moveit_commander.PlanningSceneInterface()
        self.add_collision_objects()


    def update_objects_callback(self, msg: DetectedObjectsStamped) -> None:
        self.objects_on_workbench = msg.detected_objects
        #self.get_logger().info(f"Updated objects: {len(self.objects_on_workbench)} found.")

    def select_random_object(self) -> DetectedObject:
        return random.choice(self.objects_on_workbench)
    
    def are_objects_on_workbench(self) -> bool:
        return len(self.objects_on_workbench) > 0

    def move_object(self, object: DetectedObject) -> None:
        x, y, color = object.x_world, object.y_world, object.color
        z = object.height + self.workbench_height 
        self.get_logger().info(f"\nSelected Object: {color}    (x,y) = ({x:.3f}, {y:.3f})\n")

        self.pick(x=x, y=y, z=z)
        bin_pos = self.select_bin(color)
        self.place(x=bin_pos[0], y=bin_pos[1], z=0.5)

    def select_bin(self, color: str):
        if color == "red": return self.red_bin
        if color == "green": return self.green_bin
        if color == "blue": return self.blue_bin
        self.get_logger().info('Object color not recognized. Placing in green bin.')
        return self.green_bin

    def pick(self, x, y, z, roll=0, pitch=np.pi, yaw=0, object_width=0.025):
        pre_pick_position = np.array([x + self.x_offset, y, z + self.z_offset + self.z_pre_pick_offset]) 
        pick_position = np.array([x + self.x_offset, y, z + self.z_offset]) 
        pick_orientation = quaternion_from_euler(roll, pitch, yaw)

        self.panda.move_to_cartesian_pose(pos=pre_pick_position, ori=pick_orientation)
        self.gripper_interface.open()
        self.panda.move_to_cartesian_pose(pos=pick_position, ori=pick_orientation)
        
        result = self.gripper_interface.grasp(width=object_width, force=5, speed=None, epsilon_inner=0.005, epsilon_outer=0.005)
        if not result:
            self.get_logger().warn("Grasping did not succeed.")

        self.panda.move_to_cartesian_pose(pos=pre_pick_position, ori=pick_orientation)
    
    def place(self, x, y, z, roll=0, pitch=np.pi, yaw=0):
        pre_place_position = np.array([x, y, z + self.z_pre_place_offset])
        place_position = np.array([x, y, z])
        place_orientation = quaternion_from_euler(roll, pitch, yaw)

        self.panda.move_to_cartesian_pose(pos=pre_place_position, ori=place_orientation)
        self.panda.move_to_cartesian_pose(pos=place_position, ori=place_orientation)
        self.gripper_interface.open()
        self.panda.move_to_cartesian_pose(pos=pre_place_position, ori=place_orientation)

    def add_collision_objects(self) -> None:
        workbench_pose = PoseStamped()
        workbench_pose.header.frame_id = "world"
        workbench_pose.pose.position.x = 0.7
        workbench_pose.pose.position.y = 0.0
        workbench_pose.pose.position.z = 0.1
        self.scene_interface.add_box("workbench", workbench_pose, size=(1.0, 3.0, 0.2))

        bin_bench_pose = PoseStamped()
        bin_bench_pose.header.frame_id = "world"
        bin_bench_pose.pose.position.x = -0.55
        bin_bench_pose.pose.position.y = 0.0
        bin_bench_pose.pose.position.z = 0.1 
        self.scene_interface.add_box("binbench", bin_bench_pose, size=(0.4, 1.5, 0.2))

        self.wait_for_objects()

    def wait_for_objects(self, timeout: float = 5.0) -> None:
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            known_objects = self.scene_interface.get_known_object_names()
            if "workbench" in known_objects and "binbench" in known_objects:
                self.get_logger().info("Collision objects are visible in the planning scene.")
                return
            time.sleep(0.1)
        self.get_logger().warn("Collision objects not yet visible in the planning scene.")
