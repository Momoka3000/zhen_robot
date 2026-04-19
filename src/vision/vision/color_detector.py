import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        # 1. 订阅 Gazebo 摄像头的原始图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # 2. 发布目标偏移量 (正值代表目标在右，负值代表在左)
        self.publisher_ = self.create_publisher(Float32, 'target_offset', 10)
        
        # 3. 初始化工具
        self.bridge = CvBridge()
        self.get_logger().info("---------------------------------------")
        self.get_logger().info("VISION: 视觉识别节点已启动，正在监测绿色目标...")
        self.get_logger().info("---------------------------------------")

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 获取图像尺寸 (480, 640)
        height, width, _ = cv_image.shape
        center_x = width // 2

        # 1. 颜色空间转换：从 BGR 转为 HSV (HSV 对光照更鲁棒)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. 定义绿色目标的 HSV 范围 (毕设建议使用绿色，因为对比度高)
        lower_green = np.array([35, 43, 46])
        upper_green = np.array([77, 255, 255])

        # 3. 创建掩膜 (Mask)
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # 4. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找到面积最大的轮廓 (即最靠近货物的物体)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 只有面积大于一定阈值才处理 (过滤噪点)
            if cv2.contourArea(largest_contour) > 500:
                # 计算重心 (Centroid)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 计算偏移量：当前中心 - 画面中心
                    offset = float(cx - center_x)
                    
                    # 发布偏移量供 main_logic 使用
                    msg_offset = Float32()
                    msg_offset.data = offset
                    self.publisher_.publish(msg_offset)

                    # 在画面上绘制识别结果 (用于调试)
                    cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Offset: {offset}", (cx, cy-20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # 5. 显示窗口 (实时监控)
        cv2.imshow("AMR Camera View", cv_image)
        # cv2.imshow("Mask", mask) # 取消注释可查看二值化效果
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()