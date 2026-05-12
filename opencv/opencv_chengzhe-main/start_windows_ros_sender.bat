@echo off
setlocal
cd /d %~dp0
python windows_camera_ros_sender.py %*
if errorlevel 1 (
  echo.
  echo [windows-sender] sender exited with error
)
