# OpenCV to CM530 ROS Controller

This controller keeps CM530 serial communication in `TEST07.py` unchanged.

## Prerequisites

Start the PhantomX MoveIt stack first so `/compute_ik` is available, and source both ROS 2 and the PhantomX/OpenCV workspaces.

The OpenCV bridge should publish `geometry_msgs/PointStamped` on:

```text
/camera/object_point
```

Best first setup: publish points in frame `phantomx_pincher_arm_base_link`. If not, provide TF into that frame.

## Dry Run

Calculate IK and AX positions without opening COM4:

```powershell
python ros\opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05
```

Pull one point from the OpenCV service:

```powershell
python ros\opencv_to_cm530_node.py --dry-run --once
```

## Move The Arm

Run continuous OpenCV topic mode and allow motion:

```powershell
python ros\opencv_to_cm530_node.py --move --continuous --port COM4 --baud 57600
```

Use trajectory protocol instead of direct `AX,...`:

```powershell
python ros\opencv_to_cm530_node.py --move --continuous --send-mode trajectory
```

## Calibration Knobs

AX conversion defaults to center `512`, range `0..1023`, and `1023 / 300deg` units per radian.

Per-joint calibration examples:

```powershell
python ros\opencv_to_cm530_node.py --dry-run --test-point 0.075 0.0 0.05 `
  --ax-directions "1,-1,1,-1" `
  --ax-offsets-rad "0,0,0,0" `
  --ax-min "100,100,100,100" `
  --ax-max "900,900,900,900"
```
