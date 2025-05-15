#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def generate_test_image(counter):
    """Test görüntüsü oluştur"""
    img = np.zeros((480, 640, 3), np.uint8)
    
    # Bir merkez nokta ve çember çiz
    center_x = int(320 + 100 * np.sin(counter / 30))
    center_y = int(240 + 80 * np.cos(counter / 20))
    
    # Arka plan rengi
    color_b = int(127 + 127 * np.sin(counter / 25))
    color_g = int(127 + 127 * np.sin(counter / 40))
    color_r = int(127 + 127 * np.sin(counter / 55))
    
    # Basit bir arkaplan dolgusu
    cv2.rectangle(img, (0, 0), (640, 480), (color_b, color_g, color_r), -1)
    
    # Çember çiz
    cv2.circle(img, (center_x, center_y), 50, (0, 255, 255), 5)
    
    # Metin ekle
    cv2.putText(img, f"ROS Image: {counter}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                1, (255, 255, 255), 2, cv2.LINE_AA)
    
    return img

if __name__ == '__main__':
    rospy.init_node('camera_simulator')
    
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=2)
    bridge = CvBridge()
    rate = rospy.Rate(30)  # 30Hz
    
    counter = 0
    
    rospy.loginfo("Kamera simülatörü başlatıldı. /camera/image_raw konusuna yayın yapılıyor.")
    
    while not rospy.is_shutdown():
        img = generate_test_image(counter)
        msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        pub.publish(msg)
        counter += 1
        rate.sleep()