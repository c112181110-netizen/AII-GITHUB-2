# OpenCV -> ROS IK -> CM530 整合狀態 README

最後整理日期：2026-05-12

本專案目標是把 OpenCV 偵測座標、ROS/MoveIt IK 計算、CM530/AX-12A 通訊串成一條流程：

```text
OpenCV Camera -> ROS 主控節點 -> MoveIt IK -> AX position -> CM530 -> 機械手臂
```

目前實作重點放在 `ros` 與 `docker` 兩個資料夾內，並保留 `ros/TEST07.py` 的 CM530 通訊邏輯不改。

## 目前已完成

- 已新增 `ros/opencv_to_cm530_node.py`
  - 可接 `/camera/object_point`
  - 可呼叫 `/camera/get_object_point`
  - 可呼叫 MoveIt `/compute_ik`
  - 可把 4 個 joint rad 轉成 AX position
  - 可 dry-run 印出準備送給 CM530 的 `AX,j1,j2,j3,j4`
  - 實際送出時仍使用 `TEST07.py` 的 `CM530Bridge`

- 已新增 `ros/ik_to_ax.py`
  - rad -> AX position
  - 預設中心值 `512`
  - 支援 offset、direction、min/max clamp
  - 輸出固定 clamp 到 `0..1023`

- 已整合 `TEST06.py` 可用部分
  - `TEST06.py` 的 OpenCV 座標轉 robot target/clamp 概念已加入 `opencv_to_cm530_node.py`
  - 使用參數：`--vision-map test06`
  - Docker dry-run 目前預設使用 test06 mapping

- 已新增整合 Docker
  - `Dockerfile.integrated`
  - `docker/integrated_entrypoint.sh`
  - `docker/run_integrated.bash`
  - `docker/integrated_move_group.launch.py`

- 已完成過的測試
  - `TEST07.py --self-test` 曾測到 CM530 `PING -> PONG`
  - `COM4` 曾確認存在
  - camera `cv2.VideoCapture(0)` 曾可開啟
  - OpenCV bridge 曾發布 `/camera/image_raw`、`/camera/object_point`
  - Docker doctor 曾通過 ROS/OpenCV/MoveIt 依賴檢查
  - MoveIt 最小啟動曾成功顯示 `You can start planning now!`
  - dry-run 測試點成功：

```text
test point: 0.075 0.0 0.05
IK rad: [-0.0000, -0.2331, 1.9787, 1.3959]
AX: [512, 466, 899, 785]
TX: AX,512,466,899,785
```

## 目前未完成

- 尚未完成整條實體流程實測：

```text
OpenCV 實際座標 -> ROS IK -> CM530 實際送出 -> 手臂動作
```

- Docker 內目前只建議 dry-run
  - Docker Desktop Linux container 通常不能直接使用 Windows 的 `COM4`
  - 實際控制 CM530 仍建議在能看見 serial port 的 Windows/WSL/實機 ROS 環境執行

- OpenCV 座標到機械手座標的精準校正尚未完成
  - 目前只有沿用 `TEST06.py` 的簡易 clamp mapping
  - 還沒做完整 camera calibration、hand-eye calibration 或 TF tree

- IK 的可達範圍尚未完整掃描
  - 正前方小點可算成功
  - 側向 `y` 偏移時曾出現 IK failed
  - 目前 Docker dry-run 先把側向 `y` 壓成 0，降低測試風險

- AX position 實機校正尚未完成
  - 目前 rad -> AX 是理論轉換
  - 每顆 AX-12A 的方向、中心點、offset、上下限仍需要依照實機微調

- 吸盤/ESP32 尚未整合
  - 本階段只做手臂座標與 CM530 arm motion
  - 系統架構圖中的吸、放服務還沒接進主控流程

## 已知問題

- 本機 ROS 是 Galactic，但整合 Docker 使用 Jazzy
  - Docker 是獨立 Jazzy 環境
  - 不會直接使用本機 Galactic 套件
  - 如果未來要在本機 Galactic 直接跑，需要另外處理 MoveIt、介面與套件版本差異

- PhantomX 原本 MoveIt launch 不完全適合 Jazzy Docker
  - 原 launch 會誤啟動 `ros2_control_node`
  - 原 OMPL 參數格式在 Jazzy 會報錯
  - 已用 `docker/integrated_move_group.launch.py` 建立最小 MoveIt 啟動檔處理

- Jazzy Docker 內沒有 LMA IK plugin
  - 原設定參考 `LMAKinematicsPlugin`
  - Docker 內改用 `kdl_kinematics_plugin/KDLKinematicsPlugin`

- `docker exec` 直接跑 Python 可能找不到 `rclpy`
  - 需要透過 `/integrated_entrypoint.sh` 載入 ROS 環境

正確範例：

```powershell
docker exec arm_moveit_test /integrated_entrypoint.sh python3 /workspace/ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05 --no-tf
```

- Docker Desktop 曾出現 Linux engine 斷線

```text
failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine
```

處理方式：

```text
重新啟動 Docker Desktop，確認 Linux containers engine 已啟動。
```

- OpenCV 偵測座標曾出現過大值

```text
OpenCV point: 0.9 -1.5 0.0
```

這種值直接丟 IK 通常不可達，所以目前需要 `--vision-map test06` 先轉成安全範圍。

## 最小測試流程

先確認 Docker Desktop 已啟動。

```powershell
docker build -f Dockerfile.integrated -t arm_integrated:jazzy .
```

```powershell
docker run --rm arm_integrated:jazzy bash /run_integrated.bash doctor
```

只開 MoveIt 測 IK：

```powershell
docker run -d --rm --name arm_moveit_test arm_integrated:jazzy bash /run_integrated.bash moveit
```

跑 dry-run 測試點：

```powershell
docker exec arm_moveit_test /integrated_entrypoint.sh python3 /workspace/ros/opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05 --no-tf --target-frame phantomx_pincher_arm_base_link --service-timeout-sec 10
```

預期會看到類似：

```text
IK rad j1..j4=[...] -> AX=[...]
TX -> AX,...
```

## OpenCV 整合 dry-run

啟動整合容器：

```powershell
docker run --rm -it --name arm_integrated -p 5001:5001 arm_integrated:jazzy bash /run_integrated.bash all-dry-run
```

Windows 端 camera sender 需要連到 `127.0.0.1:5001`。若已有 sender 視窗或 Python process 在跑，要注意不要重複開太多。

## 實機 CM530 測試注意

先測 CM530 通訊：

```powershell
python ros\TEST07.py --port COM4 --baud 57600 --timeout 2 --self-test
```

再跑主控節點時，若不是 dry-run，必須加 `--move`：

```powershell
python ros\opencv_to_cm530_node.py --move --once --port COM4 --baud 57600
```

目前不建議直接實機跑連續模式，應先用 dry-run 對照 AX position 是否合理。

## 下一步建議

1. 重新啟動 Docker Desktop，確認 Docker API 正常。
2. 重跑 `doctor` 與最小 IK dry-run。
3. 用 `--vision-map test06` 測 OpenCV service/topic 是否能穩定轉成可達點。
4. 掃描手臂可達範圍，建立安全 `x/y/z` limits。
5. 實機校正 AX center、direction、offset、min/max。
6. 確認單點動作安全後，再開啟 `--move` 做 CM530 實機測試。
7. 最後再接吸盤/ESP32 的吸、放服務。
