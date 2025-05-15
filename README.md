# GokHAS Interface

This Python & Qt6-based GUI simplifies interaction with ROS Noetic systems.  
Through a visual interface, you can manage **joint** and **effector** controls in manual or autonomous modes, perform calibration steps, and view real-time camera feed.

## Features

- **Real-Time Video**  
  Subscribes to the `/image_raw` ROS topic and displays the incoming frames.  
- **Manual Joint & Effector Control**  
  - Adjustable **Position** sliders (–135° to +135°) and **Power** sliders (0% to 100%) for each joint  
  - “MANUAL” mode shows a question-mark icon in the top-right corner of the video; click it for help pop-up  
- **Autonomous / Shutdown Mode**  
  - Main toggle button (“ACTIVATED/DEACTIVATED”) enables or disables all control elements  
- **Calibration**  
  - Dedicated calibration buttons for `Joint1`, `Joint2`, and `Effector`  
- **Easy Setup & Launch**  
  Spin up the GUI with a single ROS launch command.

## Installation

1. Ensure Python 3 and ROS Noetic are installed.  
2. Install required Python packages:  
   ```bash
   pip install PyQt6 rospkg
   ```  
3. Build the ROS package:  
   ```bash
   cd ~/bitirme_ws
   catkin build
   source devel/setup.bash
   ```

## Usage

Launch the interface and connect to all necessary ROS topics with:

```bash
roslaunch gokhas_interface interface.launch
```

- In **MANUAL** mode, click the question-mark icon on the video to view help instructions.  
- In **DEACTIVATED** mode, only the shutdown button remains active; all other controls are locked.

## Project Structure

```
gokhas_interface/
├── launch/
│   └── interface.launch       # ROS launch file
├── scripts/
│   ├── launch_interface.py    # GUI startup script
│   └── camera_publisher.py    # Test video publisher
├── src/
│   ├── main.py
│   ├── ui/
│   │   └── main_window.py     # Qt GUI code
│   └── ros/
│       └── ros_bridge.py      # ROS–Python bridge
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

## Contributing

Feel free to open issues or pull requests for new features, improvements, or bug fixes.

## License

MIT © 2025