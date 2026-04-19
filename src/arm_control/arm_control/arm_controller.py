import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 订阅来自 main_logic 的任务指令
        self.command_sub = self.create_subscription(
            String,
            '/arm_command',
            self.listener_callback,
            10)
            
        # 发布给 Gazebo 控制器 (对应 URDF 中的 ros2_control 配置)
        # 假设你使用的是 Position Controller，控制旋转和平移两个关节
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10)
            
        self.get_logger().info("---------------------------------------")
        self.get_logger().info("ARM CONTROL: 机械臂执行器已就绪，等待 PICK 指令")
        self.get_logger().info("---------------------------------------")

    def listener_callback(self, msg):
        command = msg.data.upper()
        if command == "PICK":
            self.execute_pick_sequence()
        elif command == "HOME":
            self.move_joints(0.0, 0.0)

    def move_joints(self, rotation, lift):
        """发送关节目标位置：[旋转弧度, 平移距离]"""
        msg = Float64MultiArray()
        msg.data = [float(rotation), float(lift)]
        self.joint_pub.publish(msg)

    def execute_pick_sequence(self):
        """执行抓取动作流"""
        self.get_logger().info("收到指令：开始抓取...")

        # 1. 旋转到正面 (对准)
        self.get_logger().info("Step 1: 旋转对准目标")
        self.move_joints(0.0, 0.0)
        time.sleep(1.5)

        # 2. 伸出手臂 (下降)
        self.get_logger().info("Step 2: 伸出手臂进行抓取")
        self.move_to_position = 0.12 # 对应 URDF 中 limit upper 的值
        self.move_joints(0.0, self.move_to_position)
        time.sleep(2.0)

        # 3. 抬起手臂 (搬运)
        self.get_logger().info("Step 3: 抓取成功，抬起货物")
        self.move_joints(0.0, 0.02)
        self.get_logger().info("抓取序列完成，准备运送。")

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()