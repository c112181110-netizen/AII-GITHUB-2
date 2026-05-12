# OpenCV -> ROS IK -> CM530 環境建置說明

本文件說明如何把整套系統建起來：

```text
OpenCV Camera -> ROS 2 -> PhantomX MoveIt IK -> CM530 -> AX-12A
```

目前 CM530 通訊基準是 `ros/TEST07.py`，整合主控是 `ros/opencv_to_cm530_node.py`。

## 1. 系統角色

```text
Windows / Camera PC
  - 執行 OpenCV camera sender
  - 把影像與座標送到 WSL / ROS

WSL Ubuntu / ROS PC
  - 執行 ROS 2 Jazzy
  - 執行 OpenCV bridge
  - 執行 PhantomX MoveIt move_group
  - 執行 opencv_to_cm530_node.py

CM530 / AX-12A
  - CM530 燒錄 15 cm530 test 韌體
  - 只接收 AX position 整數
```

## 2. Windows 端需求

安裝：

```text
Python 3.10+
OpenCV Python
NumPy
RoboPlus / CM530 燒錄工具
```

Windows Python 套件：

```powershell
python -m pip install opencv-python numpy pyserial
```

CM530 serial 設定：

```text
Port      : COM4
Baudrate  : 57600
Data bits : 8
Parity    : none
Stop bits : 1
Line end  : LF, "\n"
```

重要：同一時間只能有一個程式使用 `COM4`。要跑 ROS / Python 前，請關掉 RoboPlus Terminal。

## 3. WSL / Ubuntu ROS 2 環境

建議環境：

```text
Ubuntu 24.04 / WSL2
ROS 2 Jazzy
MoveIt 2
Python 3
```

安裝常用套件：

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-numpy \
  ros-jazzy-ros-base \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-moveit \
  ros-jazzy-moveit-msgs
```

載入 ROS：

```bash
source /opt/ros/jazzy/setup.bash
```

建議加入 `~/.bashrc`：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 4. 建置 OpenCV ROS 介面

進入 OpenCV 專案：

```bash
cd /mnt/c/Users/a0975/Desktop/整合資料夾/opencv/opencv_chengzhe-main
```

建置 service interface：

```bash
colcon build --packages-select opencv_ros2_bridge_interfaces
source install/setup.bash
```

確認 service type 可用：

```bash
ros2 interface show opencv_ros2_bridge_interfaces/srv/GetObjectPoint
```

## 5. 建置 PhantomX / MoveIt

進入 PhantomX 專案：

```bash
cd /mnt/c/Users/a0975/Desktop/整合資料夾/phantomx_pincher-ros2
```

建置：

```bash
colcon build
source install/setup.bash
```

確認 MoveIt IK 設定存在：

```bash
cat phantomx_pincher_moveit_config/config/kinematics.yaml
```

應看到：

```text
kinematics_solver: lma_kinematics_plugin/LMAKinematicsPlugin
```

## 6. 啟動 OpenCV Bridge

WSL terminal A：

```bash
cd /mnt/c/Users/a0975/Desktop/整合資料夾/opencv/opencv_chengzhe-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash run.bash win-bridge
```

Windows terminal：

```powershell
cd "C:\Users\a0975\Desktop\整合資料夾\opencv\opencv_chengzhe-main"
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview
```

測試座標 topic：

```bash
ros2 topic echo /camera/object_point
```

在 Windows preview 視窗按滑鼠左鍵，應該看到 `PointStamped`。

## 7. 啟動 PhantomX MoveIt

WSL terminal B：

```bash
cd /mnt/c/Users/a0975/Desktop/整合資料夾/phantomx_pincher-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py
```

確認 `/compute_ik` 存在：

```bash
ros2 service list | grep compute_ik
```

應看到：

```text
/compute_ik
```

## 8. 測試 CM530 通訊

Windows PowerShell：

```powershell
cd "C:\Users\a0975\Desktop\整合資料夾"
python ros\TEST07.py --port COM4 --baud 57600 --self-test
```

如果要允許小幅動作測試：

```powershell
python ros\TEST07.py --port COM4 --baud 57600 --self-test --move
```

預期會看到：

```text
TX -> PING
RX <- PONG
TX -> AX,2000
RX <- ERR,RANGE
```

如果 COM4 打不開，請確認 RoboPlus Terminal 已關閉。

## 9. 測試 ROS 主控 Dry Run

WSL terminal C 或 Windows 已有 ROS2 Python 的 terminal：

```bash
cd /mnt/c/Users/a0975/Desktop/整合資料夾
source /opt/ros/jazzy/setup.bash
source opencv/opencv_chengzhe-main/install/setup.bash
source phantomx_pincher-ros2/install/setup.bash
python ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05
```

預期輸出包含：

```text
IK rad j1..j4=[...]
AX=[...]
TX -> AX,...
```

Dry run 不會開啟 COM4，也不會讓手臂動。

## 10. 從 OpenCV Service 拉一次座標

先確定 OpenCV bridge 已經收到至少一個座標。

```bash
python ros/opencv_to_cm530_node.py --dry-run --once
```

這會呼叫：

```text
/camera/get_object_point
```

然後做：

```text
OpenCV point -> MoveIt IK -> AX position
```

## 11. 正式連動手臂

確認以下都已完成：

```text
1. OpenCV /camera/object_point 有座標
2. PhantomX MoveIt /compute_ik 存在
3. TEST07.py --self-test 通過
4. RoboPlus Terminal 已關閉，COM4 未被占用
```

執行連續模式：

```bash
python ros/opencv_to_cm530_node.py --move --continuous --port COM4 --baud 57600
```

預設送給 CM530：

```text
AX,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
```

如果要使用規格書 trajectory 格式：

```bash
python ros/opencv_to_cm530_node.py --move --continuous --send-mode trajectory --trajectory-dt-ms 300
```

會送：

```text
BEGIN,<traj_id>,4,1
PT,0,300,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
END,<traj_id>
```

## 12. AX 校正參數

預設轉換：

```text
AX center       : 512
AX range        : 0..1023
units per rad   : 1023 / 300deg
joint direction : 1,1,1,1
offset rad      : 0,0,0,0
```

校正範例：

```bash
python ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05 \
  --ax-directions "1,-1,1,-1" \
  --ax-offsets-rad "0,0,0,0" \
  --ax-min "100,100,100,100" \
  --ax-max "900,900,900,900"
```

如果某顆馬達方向相反，調整 `--ax-directions`。

如果 home 不是 512，調整 `--ax-centers`。

## 13. 常見問題

### `No module named rclpy`

沒有載入 ROS：

```bash
source /opt/ros/jazzy/setup.bash
```

### 找不到 `opencv_ros2_bridge_interfaces`

OpenCV service interface 沒建置或沒 source：

```bash
cd opencv/opencv_chengzhe-main
colcon build --packages-select opencv_ros2_bridge_interfaces
source install/setup.bash
```

### `/compute_ik` 不存在

MoveIt move_group 沒啟動：

```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py
```

### IK failed

常見原因：

```text
1. 目標點超出手臂工作範圍
2. frame_id 不對，沒有 TF
3. 姿態 quat 不適合目前手臂
```

可先用安全測試點：

```bash
python ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05
```

### COM4 被占用

關閉：

```text
RoboPlus Terminal
舊的 PowerShell 測試程式
舊的 ROS/ Python serial node
```

### 手臂不應該動卻動了

`opencv_to_cm530_node.py` 預設不允許實際動作。只有加上 `--move` 才會非 dry-run 控制 CM530。

## 14. 建議測試順序

```text
1. python ros\TEST07.py --self-test
2. python ros\TEST07.py --self-test --move
3. 啟動 OpenCV bridge，確認 /camera/object_point
4. 啟動 PhantomX MoveIt，確認 /compute_ik
5. python ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05
6. python ros/opencv_to_cm530_node.py --dry-run --once
7. python ros/opencv_to_cm530_node.py --move --continuous
```

## 15. 整合 Docker Jazzy 環境

如果本機沒有 WSL Ubuntu / ROS2，也可以先用整合 Docker 跑：

```text
OpenCV bridge + OpenCV service interface + PhantomX MoveIt + ROS 主控 dry-run
```

建置 image：

```powershell
cd "C:\Users\a0975\Desktop\整合資料夾"
docker build -f Dockerfile.integrated -t arm_integrated:jazzy .
```

檢查容器內環境：

```powershell
docker run --rm arm_integrated:jazzy bash /run_integrated.bash doctor
```

啟動整合 dry-run：

```powershell
docker run --rm -it --name arm_integrated -p 5001:5001 arm_integrated:jazzy bash /run_integrated.bash all-dry-run
```

再開另一個 Windows PowerShell 啟動相機 sender：

```powershell
cd "C:\Users\a0975\Desktop\整合資料夾\opencv\opencv_chengzhe-main"
python windows_camera_ros_sender.py --host 127.0.0.1 --port 5001 --preview
```

在 preview 視窗點一下畫面，容器應該會印出：

```text
Target point ...
IK rad j1..j4=[...]
AX=[...]
TX -> AX,...
```

這個模式使用 `--dry-run`，不會開啟 COM4，也不會讓手臂動。

### Docker 與 CM530 COM4 注意事項

Docker Desktop 的 Linux container 通常不能直接看到 Windows 的 `COM4`。所以整合 Docker 的第一目標是驗證：

```text
OpenCV 座標 -> ROS topic/service -> MoveIt IK -> AX position
```

實際 CM530 動作仍建議用能看見 `COM4` 的 Windows / WSL serial 環境執行：

```powershell
python ros\TEST07.py --self-test --move
```

或在已具備 ROS2 + MoveIt + COM4 的環境執行：

```bash
python ros/opencv_to_cm530_node.py --move --continuous --port COM4
```
