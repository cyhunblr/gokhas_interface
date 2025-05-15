import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt

class ROSBridge:
    def __init__(self, main_window):
        self.main_window = main_window
        self.bridge = CvBridge()
        
        # ROS düğümünü başlat
        if not rospy.get_node_uri():
            rospy.init_node('qt_interface_node', anonymous=True)
            
        # Görüntü konusuna abone ol
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, 
                                         self.image_callback, queue_size=1)
        rospy.loginfo("ROS Bridge başlatıldı. /camera/image_raw konusuna abone olundu.")
    
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
            
            # Görüntüyü etiket boyutuna ölçeklendir
            scaled_pixmap = pixmap.scaled(self.main_window.image_label.width(),
                                      self.main_window.image_label.height(),
                                      Qt.AspectRatioMode.KeepAspectRatio)
            
            # Ana pencereye görüntüyü güncelleme gönder
            self.main_window.update_image(scaled_pixmap)
            
        except Exception as e:
            rospy.logerr(f"Görüntü dönüştürme hatası: {e}")