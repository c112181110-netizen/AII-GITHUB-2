#!/usr/bin/env python3
"""
攝像頭測試模組 - 用於驗證 OpenCV 和相機功能
"""

import sys
import cv2
import numpy as np
from pathlib import Path


def test_opencv_version():
    """測試 OpenCV 版本"""
    print("\n📷 OpenCV 測試套件")
    print("=" * 50)
    print(f"✅ OpenCV 版本: {cv2.__version__}")
    print(f"✅ NumPy 版本: {np.__version__}")
    return True


def test_available_cameras():
    """掃描可用的攝像頭設備"""
    print("\n🔍 掃描可用攝像頭...")
    print("-" * 50)
    
    available_cameras = []
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            
            print(f"✅ 攝像頭 {i}: {width}x{height} @ {fps} FPS")
            available_cameras.append({
                'id': i,
                'width': width,
                'height': height,
                'fps': fps
            })
            cap.release()
    
    if not available_cameras:
        print("⚠️  未找到連接的攝像頭")
        return []
    
    return available_cameras


def create_test_video():
    """創建一個測試視頻幀"""
    print("\n🎬 生成測試視頻幀...")
    print("-" * 50)
    
    # 創建一個 800x600 的測試畫面
    width, height = 800, 600
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # 填充背景
    frame[:, :] = [30, 30, 30]
    
    # 繪製 OpenCV 標誌
    cv2.putText(frame, "OpenCV Camera Test", (width//2 - 150, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
    
    # 繪製中心十字
    cv2.line(frame, (width//2 - 50, height//2), (width//2 + 50, height//2), 
             (255, 0, 0), 2)
    cv2.line(frame, (width//2, height//2 - 50), (width//2, height//2 + 50), 
             (255, 0, 0), 2)
    
    # 繪製三個彩色方塊
    cv2.rectangle(frame, (100, 200), (250, 350), (0, 0, 255), -1)  # 紅色
    cv2.circle(frame, (400, 250), 75, (0, 255, 0), -1)  # 綠色
    cv2.rectangle(frame, (550, 200), (700, 350), (255, 0, 0), -1)  # 藍色
    
    # 繪製座標資訊
    info_text = f"Resolution: {width}x{height} | Time: {cv2.getTickCount()}"
    cv2.putText(frame, info_text, (20, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    print(f"✅ 測試幀已生成: {width}x{height}")
    return frame


def display_test_frame():
    """顯示測試幀"""
    print("\n🖼️  顯示測試視窗...")
    print("-" * 50)
    
    try:
        frame = create_test_video()
        
        window_name = "OpenCV Camera Test (Press Q to close)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 600)
        cv2.imshow(window_name, frame)
        
        print("✅ 視窗已打開，按 'Q' 關閉...")
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                break
        
        cv2.destroyAllWindows()
        print("✅ 視窗已關閉")
        return True
        
    except Exception as e:
        print(f"❌ 顯示視窗失敗: {e}")
        return False


def test_real_camera(camera_id=0):
    """測試真實攝像頭"""
    print(f"\n📹 測試攝像頭 {camera_id}...")
    print("-" * 50)
    
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"❌ 無法打開攝像頭 {camera_id}")
        return False
    
    # 設定分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✅ 攝像頭已打開: {width}x{height} @ {fps} FPS")
    print("按 'Q' 停止錄製...")
    
    try:
        window_name = f"Camera {camera_id} (Press Q to stop)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)
        
        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 無法讀取幀")
                break
            
            frame_count += 1
            
            # 添加幀計數器
            cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow(window_name, frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                break
        
        cv2.destroyAllWindows()
        cap.release()
        
        print(f"✅ 已捕獲 {frame_count} 幀")
        return True
        
    except Exception as e:
        print(f"❌ 攝像頭測試失敗: {e}")
        cap.release()
        cv2.destroyAllWindows()
        return False


def ros2_camera_simulation():
    """模擬 ROS 2 攝像頭消息發佈"""
    print("\n🤖 ROS 2 攝像頭模擬...")
    print("-" * 50)
    
    try:
        import rclpy
        from sensor_msgs.msg import Image
        from ros_image_codec import cv_frame_to_image_msg
        
        print("✅ ROS 2 模組已加載")
        
        # 創建一個簡單的 ROS 2 節點
        rclpy.init()
        node = rclpy.create_node('camera_test_node')
        
        ## 創建發佈者
        publisher = node.create_publisher(Image, '/camera/image_raw', 10)
        
        # 生成測試幀
        frame = create_test_video()
        ros_image = cv_frame_to_image_msg(
            frame=frame,
            stamp=node.get_clock().now().to_msg(),
            frame_id="camera_frame",
            encoding="bgr8",
        )
        
        # 發佈消息
        publisher.publish(ros_image)
        node.get_logger().info("✅ 測試影像已發佈到 /camera/image_raw")
        
        rclpy.shutdown()
        return True
        
    except ImportError:
        print("⚠️  ROS 2 模組不可用，跳過")
        return False
    except Exception as e:
        print(f"❌ ROS 2 模擬失敗: {e}")
        return False


def main():
    """主測試程序"""
    print("\n" + "=" * 50)
    print("🎥 攝像頭功能測試套件")
    print("=" * 50)
    
    # 1. 測試版本
    test_opencv_version()
    
    # 2. 掃描攝像頭
    cameras = test_available_cameras()
    
    # 3. 顯示測試幀
    print("\n選擇測試模式:")
    print("1. 顯示測試幀")
    print("2. 測試真實攝像頭 (如果可用)")
    print("3. ROS 2 模擬")
    print("4. 全部測試")
    print("5. 退出")
    
    try:
        choice = input("\n請選擇 (1-5): ").strip()
        
        if choice == "1":
            display_test_frame()
        elif choice == "2":
            if cameras:
                test_real_camera(cameras[0]['id'])
            else:
                print("❌ 沒有可用的攝像頭")
        elif choice == "3":
            ros2_camera_simulation()
        elif choice == "4":
            display_test_frame()
            if cameras:
                test_real_camera(cameras[0]['id'])
            ros2_camera_simulation()
        elif choice == "5":
            print("✅ 測試結束")
        else:
            print("❌ 無效選擇")
            
    except KeyboardInterrupt:
        print("\n⏹️  測試已停止")
    
    print("\n" + "=" * 50)
    print("測試完成！")
    print("=" * 50 + "\n")


if __name__ == "__main__":
    main()
