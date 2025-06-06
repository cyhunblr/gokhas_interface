#!/usr/bin/env python3
# -- coding: UTF-8 --

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt

class ROSBridge:
    def __init__(self, main_window):
        self.main_window = main_window
        self.bridge = CvBridge()
        
        # ROS node zaten başlatılmış olmalı, sadece kontrol et
        try:
            rospy.get_name()  # Node başlatılmış mı kontrol et
        except:
            rospy.logerr("ROS node başlatılmamış! Önce rospy.init_node() çağırın.")
            return
            
        # Görüntü konusuna abone ol
        self.image_sub = rospy.Subscriber('/zedm/zed_node/left/yolo_image', Image, 
                                         self.image_callback, queue_size=1)
        rospy.loginfo("ROS Bridge başlatıldı. YOLOv8 konusuna abone olundu.")

    def image_callback(self, msg):
        try:
            # ROS görüntüsünü OpenCV formatına dönüştür
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # OpenCV görüntüsünü Qt formatına dönüştür
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(cv_image.data, w, h, bytes_per_line, 
                            QImage.Format.Format_RGB888).rgbSwapped()
            
            # Qt görüntüsünü pixmap'e dönüştür
            pixmap = QPixmap.fromImage(qt_image)
            
            # main_window'da image_label var mı kontrol et
            if hasattr(self.main_window, 'image_label') and self.main_window.image_label:
                # Görüntüyü etiket boyutuna ölçeklendir
                scaled_pixmap = pixmap.scaled(
                    self.main_window.image_label.width(),
                    self.main_window.image_label.height(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation
                )
                
                # Ana pencereye görüntüyü güncelleme gönder
                if hasattr(self.main_window, 'update_image'):
                    self.main_window.update_image(scaled_pixmap)
                else:
                    rospy.logwarn("MainWindow'da update_image metodu bulunamadı!")
            
        except Exception as e:
            rospy.logerr(f"Görüntü dönüştürme hatası: {e}")

    def shutdown(self):
        """ROS bridge'i temiz şekilde kapat"""
        if hasattr(self, 'image_sub'):
            self.image_sub.unregister()
        rospy.loginfo("ROS Bridge kapatıldı.")