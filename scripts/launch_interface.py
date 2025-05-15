#!/usr/bin/env python3

import sys
import os
import rospy
from PyQt6.QtWidgets import QApplication

# src dizinini PYTHONPATH'e ekle
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(SCRIPT_DIR, 'src')
sys.path.insert(0, SRC_DIR)

from ui.main_window import MainWindow

def main():
    rospy.init_node('qt_ros_interface')
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    timer = app.processEvents
    while not rospy.is_shutdown():
        timer()
        rospy.sleep(0.01)
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()