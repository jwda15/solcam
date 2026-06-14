@echo off
rem SolCam UI preview (tkinter). Double-click = UI window only. No ROS/MediaPipe.
rem repo root = two levels up from this .bat (scripts\desktop).
cd /d "%~dp0..\.."
where pyw >nul 2>nul && ( start "" pyw "ros2_gesture_node\tools\ui_preview_tk.py" %* & exit /b )
where pythonw >nul 2>nul && ( start "" pythonw "ros2_gesture_node\tools\ui_preview_tk.py" %* & exit /b )
start "" py "ros2_gesture_node\tools\ui_preview_tk.py" %*
