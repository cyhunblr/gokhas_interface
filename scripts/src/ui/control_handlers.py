#!/usr/bin/env python3
# -- coding: UTF-8 --

import rospy
from PyQt6.QtWidgets import QMessageBox, QPushButton, QSlider, QFrame, QLabel
from PyQt6.QtCore import QTimer

class ControlHandlers:
    """Class that manages interface button functions"""
    
    def __init__(self, parent=None):
        self.parent = parent
        self.main_window = parent  # main_window referansını ekle
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

    def handle_joint_click(self, button):
        """Joint butonuna tıklandığında çağrılır - toggle sistemi."""
        joint_name = button.text()
        
        # Butonun mevcut durumunu kontrol et
        if button.objectName() == "joint-button-active":
            # Aktif ise deaktif yap (kırmızı)
            button.setObjectName("joint-button-inactive")
            print(f"Joint deactivated: {joint_name}")
            self.main_window.add_log_message(f"Joint deactivated: {joint_name}")
            # Slider'ları deaktif et
            self._set_joint_sliders_enabled(joint_name, False)
            
            # Hiç aktif joint yoksa kalibrasyon butonlarını tekrar aktif et
            if not self._get_active_joint():
                self._set_calibration_buttons_enabled(True)
        else:
            # Deaktif ise aktif yap (yeşil)
            button.setObjectName("joint-button-active")
            print(f"Joint activated: {joint_name}")
            self.main_window.add_log_message(f"Joint activated: {joint_name}")
            # Slider'ları aktif et
            self._set_joint_sliders_enabled(joint_name, True)
            # Aktif olduğunda mevcut değerleri log'a yazdır
            self._log_combined_values(joint_name)
        
            # Joint aktif olunca kalibrasyon butonlarını deaktif et
            self._set_calibration_buttons_enabled(False)
    
        # Stilleri yenile
        self.main_window.apply_styles()

    def _set_joint_sliders_enabled(self, joint_name, enabled):
        """Belirtilen joint'in slider'larını ve üçgen butonlarını aktif/deaktif yapar."""
        # Object name'e göre slider'ları bul
        position_slider = self.main_window.findChild(QSlider, f"position-slider-{joint_name}")
        power_slider = self.main_window.findChild(QSlider, f"power-slider-{joint_name}")
        
        if position_slider:
            position_slider.setEnabled(enabled)
            print(f"Position slider for {joint_name}: {'enabled' if enabled else 'disabled'}")
            
            # Position slider'ın parent group'ündaki arrow butonları bul ve aktif et
            parent_group = position_slider.parent()
            if parent_group:
                arrow_buttons = parent_group.findChildren(QPushButton)
                for btn in arrow_buttons:
                    if btn.objectName() == "arrow-button":
                        btn.setEnabled(enabled)
        
        if power_slider:
            power_slider.setEnabled(enabled)
            print(f"Power slider for {joint_name}: {'enabled' if enabled else 'disabled'}")
            
            # Power slider'ın parent group'ündaki arrow butonları bul ve aktif et
            parent_group = power_slider.parent()
            if parent_group:
                arrow_buttons = parent_group.findChildren(QPushButton)
                for btn in arrow_buttons:
                    if btn.objectName() == "arrow-button":
                        btn.setEnabled(enabled)

    def position_slider_changed(self, joint_name, value):
        """Position slider değiştiğinde çağrılır."""
        print(f"Position changed for {joint_name}: {value}")
        # Sadece aktif joint'ler için log'a yazdır - birlikte yazdırmak için
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)

    def power_slider_changed(self, joint_name, value):
        """Power slider değiştiğinde çağrılır."""
        print(f"Power changed for {joint_name}: {value}")
        # Sadece aktif joint'ler için log'a yazdır - birlikte yazdırmak için
        if self._is_joint_active(joint_name):
            self._log_combined_values(joint_name)

    def _is_joint_active(self, joint_name):
        """Belirtilen joint'in aktif olup olmadığını kontrol eder."""
        for widget in self.main_window.findChildren(QPushButton):
            if (widget.objectName() == "joint-button-active" and 
                widget.text() == joint_name):
                return True
        return False

    def _log_combined_values(self, joint_name):
        """Position ve power değerlerini birlikte log'a yazdır."""
        position_value, power_value = self._get_joint_values(joint_name)
        self.main_window.add_log_message(f"{joint_name} - Position: {position_value}°, Power: {power_value}%")

    def _get_joint_values(self, joint_name):
        """Belirtilen joint'in position ve power değerlerini döndürür."""
        position_value = 0
        power_value = 0
        
        # Tüm control group'ları bul
        control_groups = self.main_window.findChildren(QFrame)
        
        for group in control_groups:
            if group.objectName() == "control-group":
                # Bu grup'un hangi joint'e ait olduğunu kontrol et
                row_widget = group.parent()
                if row_widget:
                    # Aynı row'daki joint button'ı bul
                    joint_buttons = []
                    if hasattr(row_widget, 'findChildren'):
                        joint_buttons = [btn for btn in row_widget.findChildren(QPushButton) 
                                       if btn.objectName() in ["joint-button", "joint-button-active", "joint-button-inactive"]]
                    
                    # Joint button'ın text'i ile eşleşen row'u bul
                    for btn in joint_buttons:
                        if btn.text() == joint_name:
                            # Bu doğru row, slider'ları bul
                            position_sliders = []
                            power_sliders = []
                            
                            # Row'daki tüm control group'larda slider'ları bul
                            for child_group in row_widget.findChildren(QFrame):
                                if child_group.objectName() == "control-group":
                                    sliders = child_group.findChildren(QSlider)
                                    labels = child_group.findChildren(QLabel)
                                    
                                    # Position grup mu yoksa Power grup mu kontrol et
                                    for label in labels:
                                        if "Position:" in label.text() and sliders:
                                            position_sliders.extend(sliders)
                                        elif "Power:" in label.text() and sliders:
                                            power_sliders.extend(sliders)
                            
                            # Değerleri al
                            if position_sliders:
                                position_value = position_sliders[0].value()
                            if power_sliders:
                                power_value = power_sliders[0].value()
                            
                            break
        
        return position_value, power_value

    def _log_joint_values(self, joint_name):
        """Joint aktif edildiğinde mevcut değerleri log'a yazdır."""
        self._log_combined_values(joint_name)

    def _get_active_joint(self):
        """Aktif (yeşil) joint'in adını döndürür."""
        for widget in self.main_window.findChildren(QPushButton):
            if widget.objectName() == "joint-button-active":
                return widget.text()
        return None

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
        
        # Kalibrasyon başlarken joint butonlarını deaktif et
        self._set_joint_buttons_enabled(False)
        
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

        # Parent'a style uygula sinyali gönder
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
    
        # Kalibrasyon tamamlandığında joint butonlarını tekrar aktif et
        self._set_joint_buttons_enabled(True)
        
        # Joint butonlarının metin rengini normal yap
        joint_buttons = self.main_window.findChildren(QPushButton)
        for joint_btn in joint_buttons:
            if joint_btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                joint_btn.setStyleSheet("")  # Gri rengi temizle, normal renge döndür
    
        # Kalibrasyon durumunu sıfırla
        self.calibration_in_progress = False
        self.current_calibrating_button = None
        
        if self.parent:
            self.parent.apply_styles()

    def _set_calibration_buttons_enabled(self, enabled):
        """Kalibrasyon butonlarını aktif/deaktif eder."""
        if not self.main_window:
            return
    
        calibration_buttons = self.main_window.findChildren(QPushButton)
        for btn in calibration_buttons:
            if btn.objectName() in ["calibration-default", "calibration-completed", "calibration-disabled"]:
                btn.setEnabled(enabled)
                if enabled:
                    # Aktif yaparken önceki duruma göre object name'i ayarla
                    if btn.objectName() == "calibration-disabled":
                        btn.setObjectName("calibration-default")
                    # Normal renk
                    btn.setStyleSheet("")
                else:
                    # Deaktif yaparken disabled yap
                    btn.setObjectName("calibration-disabled")
                    # Metin rengini gri yap
                    btn.setStyleSheet("color: #888888 !important;")

        # Stilleri yenile
        if self.parent:
            self.parent.apply_styles()

    def _set_joint_buttons_enabled(self, enabled):
        """Joint butonlarını aktif/deaktif eder."""
        if not self.main_window:
            return
    
        joint_buttons = self.main_window.findChildren(QPushButton)
        for btn in joint_buttons:
            if btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                btn.setEnabled(enabled)
                if not enabled:
                    # Deaktif yaparken aktif olanları da deaktif yap
                    if btn.objectName() == "joint-button-active":
                        btn.setObjectName("joint-button-inactive")
                        joint_name = btn.text()
                        self._set_joint_sliders_enabled(joint_name, False)
                # Metin rengini gri yap
                btn.setStyleSheet("color: #888888 !important;")
            else:
                # Normal renk
                btn.setStyleSheet("")

        # Stilleri yenile
        if self.parent:
            self.parent.apply_styles()

    def handle_toggle(self, btn, checked, on_text, off_text, handler):
        """Handle toggle button state changes"""
        if checked:
            btn.setText(on_text)
            btn.setObjectName("toggle-active")
            handler(on_text)
        else:
            btn.setText(off_text)
            btn.setObjectName("toggle-inactive")
            handler(off_text)
        
        # Stilleri yenile
        if self.parent:
            self.parent.apply_styles()

    # --- ACTIVATION AND MODE FUNCTIONS ---
    def activation_button_clicked(self, status):
        """Function for activation toggle button"""
        print(f"Activation status changed: {status}")
        rospy.loginfo(f"Activation status changed: {status}")
        
        # Sistem aktivasyon durumuna göre kontrolleri aktif/deaktif et
        self._set_system_controls_enabled(status == "ACTIVATED")
        
        # TODO: Send system activation status to ROS topic
        # Example:
        # activation_msg = Bool()
        # activation_msg.data = (status == "ACTIVATED")
        # self.activation_pub.publish(activation_msg)

    def mode_button_clicked(self, mode):
        """Function for mode toggle button"""
        print(f"Mode changed: {mode}")
        rospy.loginfo(f"Mode changed: {mode}")
        
        # Manuel kontrolleri aktif/deaktif et
        self._set_manual_controls_enabled(mode == "MANUAL")
        
        # TODO: Send mode information to ROS topic
        # Example:
        # mode_msg = String()
        # mode_msg.data = mode
        # self.mode_pub.publish(mode_msg)

    def _set_system_controls_enabled(self, enabled):
        """Sistem aktivasyon durumuna göre tüm kontrolleri aktif/deaktif eder."""
        if not self.main_window:
            return

        # Joint butonlarını aktif/deaktif et
        joint_buttons = self.main_window.findChildren(QPushButton)
        for btn in joint_buttons:
            if btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                btn.setEnabled(enabled)
                if not enabled:
                    # Deaktif yaparken aktif olanları da deaktif yap
                    if btn.objectName() == "joint-button-active":
                        btn.setObjectName("joint-button-inactive")
                        joint_name = btn.text()
                        self._set_joint_sliders_enabled(joint_name, False)
                # Buton metnini gri yap
                btn.setStyleSheet("color: #888888 !important;" if not enabled else "")
            else:
                # Aktif yaparken normal renk
                btn.setStyleSheet("")

        # Kalibrasyon butonlarını aktif/deaktif et
        calibration_buttons = self.main_window.findChildren(QPushButton)
        for btn in calibration_buttons:
            if btn.objectName() in ["calibration-default", "calibration-completed", "calibration-disabled"]:
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setObjectName("calibration-disabled")
                    # Buton metnini gri yap
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    # Aktif yaparken önceki duruma göre ayarla
                    if btn.objectName() == "calibration-disabled":
                        btn.setObjectName("calibration-default")
                    # Normal renk
                    btn.setStyleSheet("")

        # Mode butonunu aktif/deaktif et (MANUAL/AUTONOMOUS)
        toggle_buttons = self.main_window.findChildren(QPushButton)
        for btn in toggle_buttons:
            if btn.objectName() in ["toggle-active", "toggle-inactive"]:
                # Mode butonunu bul (MANUAL/AUTONOMOUS olan)
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    btn.setEnabled(enabled)
                    if not enabled:
                        btn.setStyleSheet("color: #888888 !important;")
                    else:
                        btn.setStyleSheet("")

        # Tüm slider'ları deaktif et
        sliders = self.main_window.findChildren(QSlider)
        for slider in sliders:
            if not enabled:
                slider.setEnabled(False)

        # Arrow butonlarını deaktif et
        arrow_buttons = self.main_window.findChildren(QPushButton)
        for btn in arrow_buttons:
            if btn.objectName() == "arrow-button":
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    btn.setStyleSheet("")

        # Help butonunu aktif/deaktif et
        help_buttons = self.main_window.findChildren(QPushButton)
        for btn in help_buttons:
            if "help" in btn.objectName().lower() or "?" in btn.text():
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    btn.setStyleSheet("")

        # Border toggle butonunu aktif/deaktif et
        border_buttons = self.main_window.findChildren(QPushButton)
        for btn in border_buttons:
            if btn.objectName() in ["border-button-normal", "border-button-active"]:
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    btn.setStyleSheet("")

        # Başlık renklerini güncelle
        title_labels = self.main_window.findChildren(QLabel)
        for label in title_labels:
            if ("MANUEL JOINT and EFFECTOR CONTROL" in label.text() or 
                "CALIBRATION" in label.text()):
                if enabled:
                    # Normal renk
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
                else:
                    # Gri renk (disabled)
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none; color: #888888;")

        # Log mesajı
        system_status = "enabled" if enabled else "disabled"
        self.main_window.add_log_message(f"System controls {system_status}")
        
        # Stilleri yenile
        self.main_window.apply_styles()

    def _set_manual_controls_enabled(self, enabled):
        """Manuel kontrolleri ve kalibrasyon butonlarını aktif/deaktif eder."""
        if not self.main_window:
            return

        # Başlık metinlerine dokunma - sadece renkleri değiştir
        title_labels = self.main_window.findChildren(QLabel)
        for label in title_labels:
            if "MANUEL JOINT and EFFECTOR CONTROL" in label.text():
                if enabled:
                    # Normal renk
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
                else:
                    # Gri renk (disabled)
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none; color: #888888;")
            elif "CALIBRATION" in label.text():
                if enabled:
                    # Normal renk
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
                else:
                    # Gri renk (disabled)
                    label.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none; color: #888888;")
        
        # Tüm joint butonlarını aktif/deaktif et VE metin rengini değiştir
        joint_buttons = self.main_window.findChildren(QPushButton)
        for btn in joint_buttons:
            if btn.objectName() in ["joint-button-active", "joint-button-inactive"]:
                btn.setEnabled(enabled)
                if not enabled:
                    # Autonomous modda: buton deaktif + gri metin
                    if btn.objectName() == "joint-button-active":
                        btn.setObjectName("joint-button-inactive")
                        joint_name = btn.text()
                        self._set_joint_sliders_enabled(joint_name, False)
                        self.main_window.add_log_message(f"Joint deactivated (Autonomous mode): {joint_name}")
                    # Buton metnini gri yap
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    # Manuel modda: normal renk
                    btn.setStyleSheet("")  # Inline style'ı temizle
        
        # Kalibrasyon butonlarını aktif/deaktif et VE metin rengini değiştir
        calibration_buttons = self.main_window.findChildren(QPushButton)
        for btn in calibration_buttons:
            if btn.objectName() in ["calibration-default", "calibration-completed", "calibration-disabled"]:
                btn.setEnabled(enabled)
                if not enabled:
                    btn.setObjectName("calibration-disabled")
                    # Kalibrasyon buton metnini gri yap
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    # Manuel moda dönerken completed olanları saklayın, diğerlerini default yapın
                    if btn.objectName() == "calibration-disabled":
                        btn.setObjectName("calibration-default")
                    # Normal renk
                    btn.setStyleSheet("")  # Inline style'ı temizle
        
        # Tüm slider'ları ve arrow butonlarını deaktif et
        sliders = self.main_window.findChildren(QSlider)
        for slider in sliders:
            slider.setEnabled(False)
        
        arrow_buttons = self.main_window.findChildren(QPushButton)
        for btn in arrow_buttons:
            if btn.objectName() == "arrow-button":
                btn.setEnabled(False)
                if not enabled:
                    # Arrow butonlarının da metnini gri yap
                    btn.setStyleSheet("color: #888888 !important;")
                else:
                    btn.setStyleSheet("")

        # Log mesajı
        mode_status = "enabled" if enabled else "disabled"
        self.main_window.add_log_message(f"Manual controls {mode_status}")
        
        # Stilleri yenile
        self.main_window.apply_styles()

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

    def handle_border_toggle(self, btn):
        """Handle border toggle button click"""
        # Parent'tan border durumunu değiştir
        if self.parent:
            self.parent.show_borders = not self.parent.show_borders
            self.parent.update_borders()
            
            # Buton stilini güncelle
            if self.parent.show_borders:
                btn.setObjectName("border-button-active")  # Kırmızı (açık)
                rospy.loginfo("Debug borders enabled")
                print("Debug borders açık")
            else:
                btn.setObjectName("border-button-normal")   # Transparan (kapalı)
                rospy.loginfo("Debug borders disabled")
                print("Debug borders kapalı")
        
            # Stili tekrar uygula
            self.parent.apply_styles()
        else:
            rospy.logwarn("Parent window not found for border toggle")