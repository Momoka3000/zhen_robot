import json
import math
import os
import re
from enum import Enum

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32, String


class MissionStep(Enum):
    IDLE = "IDLE"
    NAVIGATING_TO_CELL = "NAVIGATING_TO_CELL"
    ALIGNING_CELL = "ALIGNING_CELL"
    PICKING = "PICKING"
    NAVIGATING_TO_SHELF = "NAVIGATING_TO_SHELF"
    ALIGNING_SHELF = "ALIGNING_SHELF"
    PLACING = "PLACING"
    NAVIGATING_TO_CHARGER = "NAVIGATING_TO_CHARGER"
    CHARGING = "CHARGING"


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("locations_file", "")
        self.declare_parameter("default_cargo_cell", "1")
        self.declare_parameter("default_shelf", "1")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_charge_when_idle", True)

        self.locations = self.load_locations()
        self.map_frame = self.locations.get("map_frame", "map")
        self.current_task = None
        self.pending_target = None
        self.state = MissionStep.IDLE

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_cmd_pub = self.create_publisher(String, "/arm_command", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(Float32, "target_offset", self.vision_feedback_callback, 10)
        self.create_subscription(String, "logistics_task", self.logistics_task_callback, 10)

        self.get_logger().info(
            "Logistics task manager started. Cargo cells 1-4 and shelves 1-4 "
            "are recognized from the location config."
        )

        if self.get_parameter("auto_start").value:
            cargo_cell = str(self.get_parameter("default_cargo_cell").value)
            shelf = str(self.get_parameter("default_shelf").value)
            self.start_task(cargo_cell, shelf)
        elif self.get_parameter("auto_charge_when_idle").value:
            self.create_timer(3.0, self.charge_when_idle_once)

    def load_locations(self):
        configured_path = self.get_parameter("locations_file").value
        if configured_path:
            locations_path = configured_path
        else:
            locations_path = os.path.join(
                get_package_share_directory("amr"),
                "config",
                "logistics_locations.json",
            )

        with open(locations_path, "r", encoding="utf-8") as file:
            locations = json.load(file)

        self.validate_locations(locations)
        self.get_logger().info(f"Loaded logistics location config: {locations_path}")
        return locations

    def validate_locations(self, locations):
        cargo_cells = locations.get("cargo_cells", {})
        shelves = locations.get("shelves", {})
        missing_cells = [str(i) for i in range(1, 5) if str(i) not in cargo_cells]
        missing_shelves = [str(i) for i in range(1, 5) if str(i) not in shelves]

        if missing_cells or missing_shelves:
            raise ValueError(
                "Incomplete location config. Missing "
                f"cargo cells: {missing_cells or 'none'}, "
                f"shelves: {missing_shelves or 'none'}"
            )
        if "charging_station" not in locations:
            raise ValueError("Incomplete location config. Missing charging_station.")

    def logistics_task_callback(self, msg):
        task = self.parse_task(msg.data)
        if task is None:
            self.get_logger().error(
                "Invalid task. Use: cell:1 shelf:2, or JSON: "
                '{"cargo_cell":"1","shelf":"2"}'
            )
            return

        self.start_task(task["cargo_cell"], task["shelf"])

    def parse_task(self, task_text):
        task_text = task_text.strip()
        if not task_text:
            return None

        try:
            data = json.loads(task_text)
            cargo_cell = data.get("cargo_cell") or data.get("cell")
            shelf = data.get("shelf")
            if cargo_cell and shelf:
                return {"cargo_cell": str(cargo_cell), "shelf": str(shelf)}
        except json.JSONDecodeError:
            pass

        cell_match = re.search(r"(?:cargo_cell|cell)\s*[:=]\s*(\d+)", task_text)
        shelf_match = re.search(r"(?:shelf)\s*[:=]\s*(\d+)", task_text)
        if cell_match and shelf_match:
            return {"cargo_cell": cell_match.group(1), "shelf": shelf_match.group(1)}

        numbers = re.findall(r"\d+", task_text)
        if len(numbers) >= 2:
            return {"cargo_cell": numbers[0], "shelf": numbers[1]}

        return None

    def start_task(self, cargo_cell_id, shelf_id):
        if cargo_cell_id not in self.locations["cargo_cells"]:
            self.get_logger().error(f"Unknown cargo cell id: {cargo_cell_id}")
            return
        if shelf_id not in self.locations["shelves"]:
            self.get_logger().error(f"Unknown shelf id: {shelf_id}")
            return

        self.current_task = {"cargo_cell": cargo_cell_id, "shelf": shelf_id}
        pickup_pose = self.locations["cargo_cells"][cargo_cell_id]["pickup_pose"]
        self.get_logger().info(
            f"Task accepted: pick from cargo cell {cargo_cell_id}, "
            f"deliver to shelf {shelf_id}."
        )
        self.navigate_to(pickup_pose, MissionStep.NAVIGATING_TO_CELL)

    def charge_when_idle_once(self):
        if self.state == MissionStep.IDLE:
            self.go_to_charger()

    def go_to_charger(self):
        dock_pose = self.locations["charging_station"]["dock_pose"]
        self.current_task = None
        self.get_logger().info("No active task. Navigating to charging station.")
        self.navigate_to(dock_pose, MissionStep.NAVIGATING_TO_CHARGER)

    def navigate_to(self, pose_config, state):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 navigate_to_pose action server is unavailable.")
            self.state = MissionStep.IDLE
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pose_config["x"])
        goal_msg.pose.pose.position.y = float(pose_config["y"])
        goal_msg.pose.pose.orientation.z = math.sin(float(pose_config.get("yaw", 0.0)) / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(float(pose_config.get("yaw", 0.0)) / 2.0)

        self.state = state
        self.pending_target = state
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            f"Navigating: {state.value} -> x={pose_config['x']}, y={pose_config['y']}"
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            self.state = MissionStep.IDLE
            return

        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result()
        if result.status != 4:
            self.get_logger().error(f"Navigation failed with status: {result.status}")
            self.state = MissionStep.IDLE
            return

        if self.pending_target == MissionStep.NAVIGATING_TO_CELL:
            self.get_logger().info("Arrived near cargo cell. Waiting for vision alignment.")
            self.state = MissionStep.ALIGNING_CELL
        elif self.pending_target == MissionStep.NAVIGATING_TO_SHELF:
            self.get_logger().info("Arrived near shelf. Waiting for vision alignment.")
            self.state = MissionStep.ALIGNING_SHELF
        elif self.pending_target == MissionStep.NAVIGATING_TO_CHARGER:
            self.get_logger().info("Arrived at charging station. Charging.")
            self.state = MissionStep.CHARGING

    def vision_feedback_callback(self, msg):
        if self.state not in (MissionStep.ALIGNING_CELL, MissionStep.ALIGNING_SHELF):
            return

        self.align_robot(msg.data)

    def align_robot(self, offset):
        twist = Twist()
        if abs(offset) > 15.0:
            twist.angular.z = -offset / 250.0
            twist.linear.x = 0.02
            self.cmd_vel_pub.publish(twist)
            return

        self.cmd_vel_pub.publish(twist)

        if self.state == MissionStep.ALIGNING_CELL:
            self.state = MissionStep.PICKING
            self.send_arm_command("PICK")
            shelf_id = self.current_task["shelf"]
            dropoff_pose = self.locations["shelves"][shelf_id]["dropoff_pose"]
            self.get_logger().info("Pick complete. Navigating to target shelf.")
            self.navigate_to(dropoff_pose, MissionStep.NAVIGATING_TO_SHELF)
        elif self.state == MissionStep.ALIGNING_SHELF:
            self.state = MissionStep.PLACING
            self.send_arm_command("PLACE")
            self.get_logger().info("Place complete. Returning to charging station.")
            if self.get_parameter("auto_charge_when_idle").value:
                self.go_to_charger()
            else:
                self.state = MissionStep.IDLE

    def send_arm_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.arm_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
