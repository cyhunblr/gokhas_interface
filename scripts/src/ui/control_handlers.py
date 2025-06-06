#!/usr/bin/env python3
# -- coding: UTF-8 --

import rospy
from PyQt6.QtWidgets import QMessageBox

class ControlHandlers:
    """Arayüz buton fonksiyonlarını yöneten sınıf"""
    
    def __init__(self, parent=None):
        self.parent = parent
        
    # --- JOINT KONTROL FONKSİYONLARI ---
    def joint_button_clicked(self, joint_name):
        """Joint butonları için fonksiyon"""
        print(f"Joint butonu basıldı: {joint_name}")
        rospy.loginfo(f"Joint butonu basıldı: {joint_name}")
        
        # TODO: ROS topic'ine joint komutları gönder
        # Example:
        # joint_command = JointCommand()
        # joint_command.joint_name = joint_name
        # self.joint_pub.publish(joint_command)

    def position_slider_changed(self, joint_name, value):
        """Position slider değişimi için fonksiyon"""
        print(f"{joint_name} Position slider değeri: {value}")
        rospy.loginfo(f"{joint_name} Position slider değeri: {value}")
        
        # TODO: ROS topic'ine position komutları gönder
        # Example:
        # position_command = PositionCommand()
        # position_command.joint_name = joint_name
        # position_command.position = value
        # self.position_pub.publish(position_command)

    def power_slider_changed(self, joint_name, value):
        """Power slider değişimi için fonksiyon"""
        print(f"{joint_name} Power slider değeri: {value}")
        rospy.loginfo(f"{joint_name} Power slider değeri: {value}")
        
        # TODO: ROS topic'ine power komutları gönder
        # Example:
        # power_command = PowerCommand()
        # power_command.joint_name = joint_name
        # power_command.power = value
        # self.power_pub.publish(power_command)

    # --- KALİBRASYON FONKSİYONLARI ---
    def calibration_button_clicked(self, calibration_name):
        """Calibration butonları için fonksiyon"""
        print(f"Calibration butonu basıldı: {calibration_name}")
        rospy.loginfo(f"Calibration butonu basıldı: {calibration_name}")
        
        # TODO: ROS service çağırarak kalibrasyon başlat
        # Example:
        # try:
        #     calibration_service = rospy.ServiceProxy('/calibration_service', CalibrateJoint)
        #     response = calibration_service(joint_name=calibration_name)
        #     if response.success:
        #         rospy.loginfo(f"{calibration_name} kalibrasyonu başarılı")
        #     else:
        #         rospy.logwarn(f"{calibration_name} kalibrasyonu başarısız")
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Kalibrasyon servisi hatası: {e}")

    # --- AKTIVASYON VE MOD FONKSİYONLARI ---
    def activation_button_clicked(self, status):
        """Activation toggle butonu için fonksiyon"""
        print(f"Activation durumu değişti: {status}")
        rospy.loginfo(f"Activation durumu değişti: {status}")
        
        # TODO: ROS topic'ine sistem aktivasyon durumu gönder
        # Example:
        # activation_msg = Bool()
        # activation_msg.data = (status == "ACTIVATED")
        # self.activation_pub.publish(activation_msg)

    def mode_button_clicked(self, mode):
        """Mode toggle butonu için fonksiyon"""
        print(f"Mode değişti: {mode}")
        rospy.loginfo(f"Mode değişti: {mode}")
        
        # TODO: ROS topic'ine mod bilgisi gönder
        # Example:
        # mode_msg = String()
        # mode_msg.data = mode
        # self.mode_pub.publish(mode_msg)

    # --- YARDIM VE GÖRÜNTÜ FONKSİYONLARI ---
    def show_manual_help(self):
        """Manuel mod yardım mesajı"""
        print("Help butonu basıldı")
        if self.parent:
            QMessageBox.information(
                self.parent,
                "Manual Mode Help",
                "Bu modda joint ve effector kontrollerini elle ayarlayabilirsiniz.\n"
                "• Butonları tıklayın.\n"
                "• Slider'ları sürükleyin.\n"
                "• Değerler anında güncellenir."
            )

    def update_image(self, pixmap):
        """ROS'tan gelen görüntüyü güncelle"""
        try:
            if self.parent and hasattr(self.parent, 'image_label'):
                self.parent.image_label.setPixmap(pixmap)
        except Exception as e:
            rospy.logerr(f"Görüntü güncelleme hatası: {e}")

    # --- UYGULAMA KONTROL FONKSİYONLARI ---
    def close_app(self):
        """Uygulamayı kapat"""
        if self.parent and hasattr(self.parent, 'shutdown_initiated'):
            if self.parent.shutdown_initiated:
                return
            self.parent.shutdown_initiated = True
            
            print("Kapat butonu basıldı - Uygulamayı kapatıyor...")
            rospy.loginfo("Kapat butonu basıldı - Uygulamayı kapatıyor...")
            
            # ROS bridge'i kapat
            if hasattr(self.parent, 'ros_bridge'):
                self.parent.ros_bridge.shutdown()
            
            # ROS'u kapat (sadece bir kez)
            try:
                if not rospy.is_shutdown():
                    rospy.signal_shutdown("Kullanıcı arayüzden çıkış yaptı.")
            except Exception as e:
                print(f"ROS shutdown hatası (normal): {e}")
            
            # Qt uygulamasını direkt kapat
            if self.parent:
                self.parent.close()

    # --- TEMA FONKSİYONLARI ---
    def change_theme(self, theme_name):
        """Tema değiştir"""
        if not self.parent:
            return
            
        try:
            # Resource manager'dan tema al
            from ui.resources.resource import resource_manager
            new_style = resource_manager.set_theme(theme_name)
            
            if new_style:
                # Temayı uygula
                self.parent.setStyleSheet(new_style)
                
                # Tema buton stillerini güncelle
                active_style = """
                    QPushButton {
                        background-color: #0078d4;
                        color: white;
                        border: 2px solid #005a9e;
                        border-radius: 17px;
                        font-size: 16px;
                    }
                """
                
                normal_style = """
                    QPushButton {
                        background-color: #f0f0f0;
                        border: 1px solid #cccccc;
                        border-radius: 17px;
                        font-size: 16px;
                    }
                    QPushButton:hover {
                        background-color: #e0e0e0;
                    }
                """
                
                # Buton durumlarını güncelle
                if hasattr(self.parent, 'light_theme_btn') and hasattr(self.parent, 'dark_theme_btn'):
                    if theme_name == "light_theme":
                        self.parent.light_theme_btn.setStyleSheet(active_style)
                        self.parent.dark_theme_btn.setStyleSheet(normal_style)
                    else:
                        self.parent.dark_theme_btn.setStyleSheet(active_style)
                        self.parent.light_theme_btn.setStyleSheet(normal_style)
                
                rospy.loginfo(f"Tema '{theme_name}' uygulandı")
            else:
                rospy.logwarn(f"Tema '{theme_name}' bulunamadı")
                
        except Exception as e:
            rospy.logerr(f"Tema değiştirme hatası: {e}")