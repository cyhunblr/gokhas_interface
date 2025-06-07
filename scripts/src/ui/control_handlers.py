#!/usr/bin/env python3
# -- coding: UTF-8 --

import rospy
from PyQt6.QtWidgets import QMessageBox, QPushButton  # QPushButton import'u eklendi
from PyQt6.QtCore import QTimer

class ControlHandlers:
    """Class that manages interface button functions"""
    
    def __init__(self, parent=None):
        self.parent = parent
        self.pulse_timers = {}  # Her buton için ayrı timer dictionary'si
        self.calibration_in_progress = False  # Kalibrasyon durumu kontrolü
        self.current_calibrating_button = None  # Şu anda kalibre olan buton
        
    # --- JOINT CONTROL FUNCTIONS ---
    def joint_button_clicked(self, joint_name):
        """Function for joint control buttons"""
        print(f"Joint button pressed: {joint_name}")
        rospy.loginfo(f"Joint button pressed: {joint_name}")
        
        # TODO: Send joint control command to ROS topic
        # Example:
        # joint_msg = JointCommand()
        # joint_msg.joint_name = joint_name
        # self.joint_pub.publish(joint_msg)

    def position_slider_changed(self, joint_name, value):
        """Function for position slider changes"""
        print(f"Position slider changed for {joint_name}: {value}")
        rospy.loginfo(f"Position slider changed for {joint_name}: {value}")
        
        # TODO: Send position command to ROS topic
        # Example:
        # pos_msg = JointPosition()
        # pos_msg.joint_name = joint_name
        # pos_msg.position = value
        # self.position_pub.publish(pos_msg)

    def power_slider_changed(self, joint_name, value):
        """Function for power slider changes"""
        print(f"Power slider changed for {joint_name}: {value}")
        rospy.loginfo(f"Power slider changed for {joint_name}: {value}")
        
        # TODO: Send power command to ROS topic
        # Example:
        # power_msg = JointPower()
        # power_msg.joint_name = joint_name
        # power_msg.power = value
        # self.power_pub.publish(power_msg)

    # --- CALIBRATION FUNCTIONS ---
    def calibration_button_clicked(self, calibration_name):
        """Function for calibration buttons"""
        print(f"Calibration button pressed: {calibration_name}")
        rospy.loginfo(f"Calibration button pressed: {calibration_name}")
        
        # TODO: Send calibration command to ROS service
        # Example:
        # try:
        #     calibration_service = rospy.ServiceProxy('calibrate_joint', CalibrateJoint)
        #     response = calibration_service(joint_name=calibration_name)
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Calibration service call failed: {e}")

    def handle_calibration_click(self, btn, all_buttons):
        """Handle calibration button click with visual feedback"""
        # Eğer kalibrasyon devam ediyorsa, yeni kalibrasyon başlatma
        if self.calibration_in_progress:
            rospy.logwarn(f"Calibration already in progress for {self.current_calibrating_button.text() if self.current_calibrating_button else 'unknown'}. Please wait...")
            print(f"Calibration blocked: {btn.text()} - Another calibration in progress")
            return
        
        # Kalibrasyon başlat
        self.calibration_in_progress = True
        self.current_calibrating_button = btn
        
        # İlk olarak handler'ı çağır
        self.calibration_button_clicked(btn.text())
        
        # UI güncellemesi - TÜM butonları deaktif et (processing olan dahil)
        for b in all_buttons:
            if b == btn:
                b.setObjectName("calibration-processing")
                b.setEnabled(False)  # Processing butonunu da deaktif et
            else:
                b.setObjectName("calibration-disabled")
                b.setEnabled(False)  # Diğer butonları da deaktif et
    
        # Parent'a style uygulama sinyali gönder
        if self.parent:
            self.parent.apply_styles()
            
            # Pulse animasyonu başlat
            self._start_pulse_animation(btn)
            
            # 5 saniye sonra tamamla
            QTimer.singleShot(5000, lambda: self.finish_calibration(btn, all_buttons))

    def _start_pulse_animation(self, btn):
        """Start pulse animation for calibration button"""
        # Varolan timer'ı durdur
        btn_id = id(btn)
        if btn_id in self.pulse_timers:
            self.pulse_timers[btn_id].stop()
            del self.pulse_timers[btn_id]
        
        # Yeni timer oluştur
        timer = QTimer()
        pulse_state = [True]  # List kullanarak mutable yapıyoruz
        
        def toggle_opacity():
            if hasattr(btn, 'setStyleSheet') and btn.objectName() == "calibration-processing":
                if pulse_state[0]:
                    # Opak
                    btn.setStyleSheet("""
                        QPushButton#calibration-processing {
                            background-color: #ffc107 !important;
                            color: #212529 !important;
                            border: 2px solid #e0a800;
                            border-radius: 8px;
                            padding: 8px 16px;
                            font-weight: bold;
                            font-size: 12px;
                        }
                    """)
                else:
                    # Şeffaf
                    btn.setStyleSheet("""
                        QPushButton#calibration-processing {
                            background-color: rgba(255, 193, 7, 0.6) !important;
                            color: #212529 !important;
                            border: 2px solid #e0a800;
                            border-radius: 8px;
                            padding: 8px 16px;
                            font-weight: bold;
                            font-size: 12px;
                        }
                    """)
                pulse_state[0] = not pulse_state[0]
        
        timer.timeout.connect(toggle_opacity)
        timer.start(250)  # 250ms'de bir değiş
        self.pulse_timers[btn_id] = timer

    def finish_calibration(self, btn, all_buttons):
        """Mark calibration as completed"""
        # Pulse animasyonunu durdur
        btn_id = id(btn)
        if btn_id in self.pulse_timers:
            try:
                self.pulse_timers[btn_id].stop()
                del self.pulse_timers[btn_id]
            except Exception as e:
                print(f"Timer stop error: {e}")
    
        # Style'ı temizle ve completed durumuna geçir
        if hasattr(btn, 'setStyleSheet'):
            btn.setStyleSheet("")  # Inline style'ı temizle
            btn.setObjectName("calibration-completed")
            btn.setEnabled(True)  # Tamamlanan butonu tekrar aktif et
        
            # Diğer butonları tekrar aktif et ve default duruma getir
            for b in all_buttons:
                if b != btn:
                    b.setObjectName("calibration-default")
                    b.setEnabled(True)  # Butonları tekrar aktif et
        
        # Kalibrasyon durumunu sıfırla
        self.calibration_in_progress = False
        self.current_calibrating_button = None
        
        if self.parent:
            self.parent.apply_styles()

    # --- ACTIVATION AND MODE FUNCTIONS ---
    def activation_button_clicked(self, status):
        """Function for activation toggle button"""
        print(f"Activation status changed: {status}")
        rospy.loginfo(f"Activation status changed: {status}")
        
        # TODO: Send system activation status to ROS topic
        # Example:
        # activation_msg = Bool()
        # activation_msg.data = (status == "ACTIVATED")
        # self.activation_pub.publish(activation_msg)

    def mode_button_clicked(self, mode):
        """Function for mode toggle button"""
        print(f"Mode changed: {mode}")
        rospy.loginfo(f"Mode changed: {mode}")
        
        # TODO: Send mode information to ROS topic
        # Example:
        # mode_msg = String()
        # mode_msg.data = mode
        # self.mode_pub.publish(mode_msg)

    def handle_toggle(self, btn, checked, on_text, off_text, handler):
        """Handle toggle button state changes"""
        btn.setText(on_text if checked else off_text)
        btn.setObjectName("toggle-active" if checked else "toggle-inactive")
        handler(btn.text())
        if self.parent:
            self.parent.apply_styles()

    # --- HELP AND IMAGE FUNCTIONS ---
    def show_manual_help(self):
        """Manual mode help message"""
        print("Help button pressed")
        if self.parent:
            QMessageBox.information(
                self.parent,
                "Manual Mode Help",
                "In this mode, you can manually adjust joint and effector controls.\n"
                "• Click buttons.\n"
                "• Drag sliders.\n"
                "• Values are updated instantly."
            )

    # --- THEME AND SYSTEM FUNCTIONS ---
    def change_theme(self, theme_name):
        """Function for theme change"""
        print(f"Theme changed: {theme_name}")
        rospy.loginfo(f"Theme changed: {theme_name}")
        
        # Apply theme through resource manager
        if self.parent:
            from ui.resources.resource import resource_manager
            resource_manager.set_theme(theme_name)

    def handle_theme_change(self, theme):
        """Handle theme change with style application and button state update"""
        self.change_theme(theme)
        
        # Tema butonlarının stilini güncelle
        self._update_theme_button_styles(theme)
        
        if self.parent:
            self.parent.apply_styles()

    def _update_theme_button_styles(self, active_theme):
        """Update theme button styles to show which one is active"""
        if not self.parent:
            return
            
        # Tüm tema butonlarını bul ve stilini güncelle
        theme_buttons = self.parent.findChildren(QPushButton)
        
        for btn in theme_buttons:
            # Sadece tema butonlarını kontrol et
            if btn.objectName() in ["theme-button-normal", "theme-button-active"]:
                # Button'un hangi temaya ait olduğunu tooltip'ten anla
                is_light_btn = "Açık Tema" in btn.toolTip()
                is_dark_btn = "Koyu Tema" in btn.toolTip()
                
                # Aktif tema ile eşleşen butonu mavi yap
                if (is_light_btn and active_theme == "light_theme") or \
                   (is_dark_btn and active_theme == "dark_theme"):
                    btn.setObjectName("theme-button-active")
                else:
                    btn.setObjectName("theme-button-normal")

    def close_app(self):
        """Close application"""
        if self.parent and hasattr(self.parent, 'shutdown_initiated'):
            if self.parent.shutdown_initiated:
                return
            self.parent.shutdown_initiated = True
            
            print("Close button pressed - Closing application...")
            rospy.loginfo("Close button pressed - Closing application...")
            
            # Close ROS bridge - güvenli kontrol ekle
            if hasattr(self.parent, 'ros_bridge') and hasattr(self.parent.ros_bridge, 'shutdown'):
                try:
                    self.parent.ros_bridge.shutdown()
                except Exception as e:
                    rospy.logwarn(f"ROS bridge shutdown hatası: {e}")
            
            # Close ROS (only once)
            try:
                if not rospy.is_shutdown():
                    rospy.signal_shutdown("User exited from interface.")
            except Exception as e:
                print(f"ROS shutdown error (normal): {e}")
            
            # Close Qt application directly
            if self.parent:
                self.parent.close()