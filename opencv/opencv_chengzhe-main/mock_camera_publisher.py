#!/usr/bin/env python3
"""
ROS 2 模擬攝像頭發佈者 - 發佈測試影像
在沒有真實攝像頭時用於測試
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
from ros_image_codec import cv_frame_to_image_msg


class MockCameraPublisher(Node):
    """模擬攝像頭發佈者"""
    
    def __init__(self):
        super().__init__('mock_camera_publisher')
        
        # 宣告參數
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_topic', '/camera/image_raw')
        
        # 取得參數
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        
        # 創建發佈者
        self.publisher_ = self.create_publisher(
            Image,
            self.publish_topic,
            10
        )
        
        # 計時器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        # 幀計數器
        self.frame_count = 0
        self.animation_offset = 0
        
        self.get_logger().info(
            f"Mock Camera 已啟動\n"
            f"  Topic: {self.publish_topic}\n"
            f"  Resolution: {self.frame_width}x{self.frame_height}\n"
            f"  FPS: {self.fps}"
        )
    
    def create_test_frame(self):
        """生成一個測試幀"""
        # 建立黑色背景
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        frame[:, :] = [30, 30, 30]
        
        # 添加標題
        cv2.putText(
            frame, 
            "Mock Camera Stream", 
            (self.frame_width // 2 - 120, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 
            1.0, 
            (0, 255, 0), 
            2
        )
        
        # 繪製中心十字
        center_x, center_y = self.frame_width // 2, self.frame_height // 2
        cv2.line(frame, (center_x - 50, center_y), (center_x + 50, center_y), (255, 0, 0), 2)
        cv2.line(frame, (center_x, center_y - 50), (center_x, center_y + 50), (255, 0, 0), 2)
        
        # 繪製動畫圓形
        radius = int(50 + 20 * np.sin(self.animation_offset / 10.0))
        cv2.circle(frame, (center_x, center_y), radius, (0, 255, 255), 2)
        
        # 繪製移動的方塊
        x_offset = int(100 * np.sin(self.animation_offset / 20.0))
        y_offset = int(100 * np.cos(self.animation_offset / 20.0))
        
        rect_x1 = center_x + x_offset - 30
        rect_y1 = center_y + y_offset - 30
        rect_x2 = center_x + x_offset + 30
        rect_y2 = center_y + y_offset + 30
        
        cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 2)
        
        # 添加幀信息
        info_text = f"Frame: {self.frame_count} | FPS: {self.fps}"
        cv2.putText(
            frame, 
            info_text, 
            (10, self.frame_height - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (200, 200, 200), 
            1
        )
        
        # 添加時間戳
        timestamp_text = f"T: {time.time():.2f}"
        cv2.putText(
            frame, 
            timestamp_text, 
            (self.frame_width - 150, self.frame_height - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (200, 200, 200), 
            1
        )
        
        self.animation_offset += 1
        
        return frame
    
    def publish_frame(self):
        """發佈測試幀"""
        # 生成幀
        frame = self.create_test_frame()
        
        # 轉換為 ROS 2 Image 消息
        msg = cv_frame_to_image_msg(
            frame=frame,
            stamp=self.get_clock().now().to_msg(),
            frame_id="camera_frame",
            encoding="bgr8",
        )
        
        # 發佈
        self.publisher_.publish(msg)
        
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = MockCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
