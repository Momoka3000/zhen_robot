import heapq
import json
import math
import os
import re
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
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


@dataclass(order=True)
class QueuedTask:
    sort_key: tuple
    task_id: int = field(compare=False)
    cargo_cell: str = field(compare=False)
    shelf: str = field(compare=False)
    priority: int = field(compare=False)


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.declare_parameter("locations_file", "")
        self.declare_parameter("auto_charge_when_idle", True)
        self.declare_parameter("min_battery_to_start", 30.0)
        self.declare_parameter("battery_resume_level", 80.0)
        self.declare_parameter("initial_battery_percent", 100.0)
        self.declare_parameter("simulated_charge_rate_percent_per_sec", 1.0)

        self.locations = self.load_locations()
        self.map_frame = self.locations.get("map_frame", "map")
        self.current_task = None
        self.pending_target = None
        self.state = MissionStep.IDLE
        self.task_queue = []
        self.task_sequence = 0
        self.battery_percent = float(self.get_parameter("initial_battery_percent").value)
        self.last_feedback_log_time = None
        self.last_nav_unavailable_log_time = None

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_cmd_pub = self.create_publisher(String, "/arm_command", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(Float32, "target_offset", self.vision_feedback_callback, 10)
        self.create_subscription(String, "logistics_task", self.logistics_task_callback, 10)
        self.create_subscription(Float32, "battery_percent", self.battery_callback, 10)
        self.create_timer(1.0, self.scheduler_tick)

        self.get_logger().info(
            "Task manager started. Send one task as 'cell:1 shelf:2 priority:5', "
            'JSON {"cargo_cell":"1","shelf":"2","priority":5}, or a JSON list.'
        )

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

    def battery_callback(self, msg):
        self.battery_percent = max(0.0, min(100.0, float(msg.data)))

    def logistics_task_callback(self, msg):
        tasks = self.parse_tasks(msg.data)
        if not tasks:
            self.get_logger().error(
                "Invalid task. Use 'cell:1 shelf:2 priority:5', "
                'JSON {"cargo_cell":"1","shelf":"2","priority":5}, or a JSON list.'
            )
            return

        for task in tasks:
            self.enqueue_task(task["cargo_cell"], task["shelf"], task["priority"])
        self.try_start_next_task()

    def parse_tasks(self, task_text):
        task_text = task_text.strip()
        if not task_text:
            return []

        try:
            data = json.loads(task_text)
            if isinstance(data, list):
                return [self.normalize_task(item) for item in data if self.normalize_task(item)]
            normalized = self.normalize_task(data)
            return [normalized] if normalized else []
        except json.JSONDecodeError:
            pass

        cell_match = re.search(r"(?:cargo_cell|cell)\s*[:=]\s*(\d+)", task_text)
        shelf_match = re.search(r"(?:shelf)\s*[:=]\s*(\d+)", task_text)
        priority_match = re.search(r"(?:priority|prio|p)\s*[:=]\s*(-?\d+)", task_text)
        if cell_match and shelf_match:
            return [{
                "cargo_cell": cell_match.group(1),
                "shelf": shelf_match.group(1),
                "priority": int(priority_match.group(1)) if priority_match else 0,
            }]

        numbers = re.findall(r"-?\d+", task_text)
        if len(numbers) >= 2:
            return [{
                "cargo_cell": numbers[0],
                "shelf": numbers[1],
                "priority": int(numbers[2]) if len(numbers) >= 3 else 0,
            }]

        return []

    def normalize_task(self, data):
        if not isinstance(data, dict):
            return None

        cargo_cell = data.get("cargo_cell") or data.get("cell")
        shelf = data.get("shelf")
        priority = int(data.get("priority", 0))
        if cargo_cell is None or shelf is None:
            return None
        return {"cargo_cell": str(cargo_cell), "shelf": str(shelf), "priority": priority}

    def enqueue_task(self, cargo_cell_id, shelf_id, priority):
        if cargo_cell_id not in self.locations["cargo_cells"]:
            self.get_logger().error(f"Unknown cargo cell id: {cargo_cell_id}")
            return
        if shelf_id not in self.locations["shelves"]:
            self.get_logger().error(f"Unknown shelf id: {shelf_id}")
            return

        self.task_sequence += 1
        task = QueuedTask(
            sort_key=(-int(priority), self.task_sequence),
            task_id=self.task_sequence,
            cargo_cell=cargo_cell_id,
            shelf=shelf_id,
            priority=int(priority),
        )
        heapq.heappush(self.task_queue, task)
        self.get_logger().info(
            f"Queued task #{task.task_id}: cell {cargo_cell_id} -> shelf {shelf_id}, "
            f"priority={priority}. Pending={len(self.task_queue)}"
        )

    def scheduler_tick(self):
        if self.state == MissionStep.CHARGING:
            self.simulate_charging()
            self.try_start_next_task()
            return

        if self.state == MissionStep.IDLE:
            if self.task_queue:
                self.try_start_next_task()
            elif self.get_parameter("auto_charge_when_idle").value:
                self.go_to_charger("No active task.")

    def simulate_charging(self):
        rate = float(self.get_parameter("simulated_charge_rate_percent_per_sec").value)
        if rate <= 0.0:
            return
        self.battery_percent = min(100.0, self.battery_percent + rate)

    def has_enough_battery_to_start(self):
        return self.battery_percent >= float(self.get_parameter("min_battery_to_start").value)

    def has_enough_battery_to_resume(self):
        return self.battery_percent >= float(self.get_parameter("battery_resume_level").value)

    def try_start_next_task(self):
        if self.state not in (MissionStep.IDLE, MissionStep.CHARGING):
            return
        if not self.task_queue:
            return

        if self.state == MissionStep.CHARGING and not self.has_enough_battery_to_resume():
            return
        if self.state == MissionStep.IDLE and not self.has_enough_battery_to_start():
            self.go_to_charger("Battery too low before starting next task.")
            return

        if not self.nav_server_ready():
            return

        next_task = heapq.heappop(self.task_queue)
        self.start_task(next_task)

    def start_task(self, task):
        self.current_task = {
            "task_id": task.task_id,
            "cargo_cell": task.cargo_cell,
            "shelf": task.shelf,
            "priority": task.priority,
        }
        pickup_pose = self.locations["cargo_cells"][task.cargo_cell]["pickup_pose"]
        self.get_logger().info(
            f"Starting task #{task.task_id}: cell {task.cargo_cell} -> shelf {task.shelf}, "
            f"priority={task.priority}, battery={self.battery_percent:.1f}%."
        )
        self.navigate_to(pickup_pose, MissionStep.NAVIGATING_TO_CELL)

    def go_to_charger(self, reason):
        if self.state in (MissionStep.NAVIGATING_TO_CHARGER, MissionStep.CHARGING):
            return
        if not self.nav_server_ready():
            return
        dock_pose = self.locations["charging_station"]["dock_pose"]
        self.current_task = None
        self.get_logger().info(f"{reason} Navigating to charging station.")
        self.navigate_to(dock_pose, MissionStep.NAVIGATING_TO_CHARGER)

    def nav_server_ready(self):
        if self.nav_client.wait_for_server(timeout_sec=1.0):
            return True

        now = self.get_clock().now()
        if self.last_nav_unavailable_log_time is None:
            should_log = True
        else:
            elapsed = (now - self.last_nav_unavailable_log_time).nanoseconds / 1e9
            should_log = elapsed >= 5.0

        if should_log:
            self.last_nav_unavailable_log_time = now
            self.get_logger().warn(
                "Waiting for Nav2 navigate_to_pose action server. "
                "Pending tasks will stay queued."
            )
        return False

    def navigate_to(self, pose_config, state):
        if not self.nav_server_ready():
            self.state = MissionStep.IDLE
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        # A zero timestamp asks Nav2/TF to use the latest available transform.
        # This is more tolerant after manual teleop localization in simulation.
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0
        goal_msg.pose.pose.position.x = float(pose_config["x"])
        goal_msg.pose.pose.position.y = float(pose_config["y"])
        goal_msg.pose.pose.orientation.z = math.sin(float(pose_config.get("yaw", 0.0)) / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(float(pose_config.get("yaw", 0.0)) / 2.0)

        self.state = state
        self.pending_target = state
        self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback,
        ).add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            f"Navigating: {state.value} -> x={pose_config['x']}, y={pose_config['y']}"
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected.")
            self.state = MissionStep.IDLE
            return

        self.get_logger().info("Navigation goal accepted by Nav2.")
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        now = self.get_clock().now()
        if self.last_feedback_log_time is not None:
            elapsed = (now - self.last_feedback_log_time).nanoseconds / 1e9
            if elapsed < 2.0:
                return

        self.last_feedback_log_time = now
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Navigation feedback: "
            f"distance_remaining={feedback.distance_remaining:.2f}, "
            f"navigation_time={feedback.navigation_time.sec}s"
        )

    def navigation_result_callback(self, future):
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Navigation failed with status: {result.status} "
                f"({self.goal_status_name(result.status)})"
            )
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

    def goal_status_name(self, status):
        names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return names.get(status, "UNRECOGNIZED")

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
            finished_task = self.current_task
            self.current_task = None
            self.get_logger().info(
                f"Task #{finished_task['task_id']} complete. Checking queue and battery."
            )
            self.state = MissionStep.IDLE
            self.try_start_next_task()

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
