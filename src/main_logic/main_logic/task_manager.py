import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
import time

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # 1. 发布器：控制底盘移动和机械臂指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)

        # 2. 订阅器：监听视觉识别的偏移量
        self.target_sub = self.create_subscription(
            Float32, 
            'target_offset', 
            self.vision_feedback_callback, 
            10)

        # 3. 初始化状态机
        # 状态列表: IDLE(闲置), NAVIGATING(导航中), ALIGNING(对准中), PICKING(抓取中)
        self.state = "NAVIGATING" 
        self.get_logger().info("--- 大脑逻辑已启动：当前状态 = 自主导航避障 ---")

    def vision_feedback_callback(self, msg):
        """处理视觉反馈并决定下一步行动"""
        offset = msg.data

        # 逻辑：如果我们在导航时发现了目标且偏移量进入视野
        if self.state == "NAVIGATING":
            self.get_logger().warn("检测到目标货物！切断自主导航，进入视觉精准对位状态。")
            self.state = "ALIGNING"

        if self.state == "ALIGNING":
            self.align_robot(offset)

    def align_robot(self, offset):
        """通过视觉偏移量微调机器人位置"""
        twist = Twist()
        
        # 简单的比例控制 (P控制)
        # 如果目标在左边，向左转；在右边，向右转
        if abs(offset) > 15.0:  # 允许15像素的误差区间
            twist.angular.z = -offset / 250.0  # 转向增益
            twist.linear.x = 0.02              # 缓慢靠近
            self.cmd_vel_pub.publish(twist)
        else:
            # 已经对准中心
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info("对准成功！发送 PICK 指令给机械臂。")
            self.send_arm_command("PICK")
            self.state = "PICKING"

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