import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class BatterySimulator(Node):
    def __init__(self):
        super().__init__("battery_simulator")

        self.declare_parameter("initial_battery_percent", 100.0)
        self.declare_parameter("task_battery_cost_percent", 36.0)
        self.declare_parameter("simulated_charge_rate_percent_per_sec", 8.0)
        self.declare_parameter("publish_rate_hz", 1.0)

        self.battery_percent = float(self.get_parameter("initial_battery_percent").value)
        self.task_battery_cost = float(self.get_parameter("task_battery_cost_percent").value)
        self.charge_rate = float(self.get_parameter("simulated_charge_rate_percent_per_sec").value)
        publish_rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.is_charging = False

        self.battery_pub = self.create_publisher(Float32, "battery_percent", 10)
        self.create_subscription(String, "task_manager_state", self.state_callback, 10)
        self.create_subscription(String, "battery_event", self.event_callback, 10)
        self.create_timer(1.0 / publish_rate, self.publish_battery)

        self.get_logger().info(
            f"Battery simulator started: initial={self.battery_percent:.1f}%, "
            f"task_cost={self.task_battery_cost:.1f}%, charge_rate={self.charge_rate:.1f}%/s."
        )

    def state_callback(self, msg):
        self.is_charging = msg.data == "CHARGING"

    def event_callback(self, msg):
        if msg.data != "TASK_COMPLETED":
            return

        old_battery = self.battery_percent
        self.battery_percent = max(0.0, self.battery_percent - self.task_battery_cost)
        self.get_logger().info(
            f"Task battery use: {old_battery:.1f}% -> {self.battery_percent:.1f}%."
        )
        self.publish_battery()

    def publish_battery(self):
        if self.is_charging and self.charge_rate > 0.0:
            self.battery_percent = min(100.0, self.battery_percent + self.charge_rate)

        msg = Float32()
        msg.data = float(self.battery_percent)
        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
