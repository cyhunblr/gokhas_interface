#!/usr/bin/env python3
# -- coding: UTF-8 --

import sys
import os
import signal
import rospy
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from ui.main_window import MainWindow

# Global flag to prevent multiple shutdowns
shutdown_initiated = False

def signal_handler(sig, frame): 
    """SIGINT/SIGTERM signal handler"""
    global shutdown_initiated
    if shutdown_initiated:
        return
    shutdown_initiated = True
    
    print("Shutdown signal received! Clean shutdown started...")
    try:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Shutdown initiated!")
    except:
        pass
    
    # Force quit Qt application
    app = QApplication.instance()
    if app:
        app.quit()
    sys.exit(0)

def main():
    global shutdown_initiated
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)    
    signal.signal(signal.SIGTERM, signal_handler)   
    
    # Start ROS node
    rospy.init_node('qt_interface_node', anonymous=True) 
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    # Safe shutdown handler - prevent infinite loop
    def safe_shutdown_handler():
        global shutdown_initiated
        if shutdown_initiated:
            return
        shutdown_initiated = True
        
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("Qt application closing!") 
        except:
            pass
    
    # This connection only works when Qt window is closed
    app.aboutToQuit.connect(safe_shutdown_handler)  
    
    try:
        rospy.loginfo("Qt ROS Interface started!")  
        exit_code = app.exec()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()