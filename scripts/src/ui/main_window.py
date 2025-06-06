#!/usr/bin/env python3
# -- coding: UTF-8 --

from PyQt6.QtWidgets import (
    QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QWidget, QLabel, QFrame, QCheckBox, QApplication,
    QSizePolicy, QSlider, QToolButton, QMessageBox, QStyle
)
from PyQt6.QtCore import Qt, QSize, QTimer
from PyQt6.QtGui import QPixmap, QImage, QIcon

from ui.resources.resource import resource_manager
from ui.control_handlers import ControlHandlers

import rospy
import sys
import os

from ros.ros_bridge import ROSBridge

# Özel widget sınıfı ekle
class HeightSyncWidget(QWidget):
    def __init__(self, reference_widget, parent=None):
        super().__init__(parent)
        self.reference_widget = reference_widget

    def showEvent(self, event):
        super().showEvent(event)
        if self.reference_widget:
            self.updateHeight()
            
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.updateHeight()
        
    def updateHeight(self):
        if self.reference_widget and self.reference_widget.height() > 0:
            self.setFixedHeight(self.reference_widget.height())

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GokHAS Central Control Interface")
        self.setGeometry(100, 100, 1200, 800)
        
        # Shutdown flag ekle
        self.shutdown_initiated = False
        
        # Control handlers'ı başlat
        self.control_handlers = ControlHandlers(self)
        
        # UI bileşenlerini oluştur
        self.init_ui()
        
        # ROS bridge'i başlat
        self.ros_bridge = ROSBridge(self)
        
    def init_ui(self):     
        """UI bileşenlerini başlat - TAM ARAYÜZ"""
        # Ana widget ve grid layout (2 sütun, 2 satır)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        grid = QGridLayout(central_widget)
        grid.setColumnStretch(0, 5)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 5)
        grid.setRowStretch(1, 2)

        # 1. sütun 1. satır: Görüntü
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("background-color: black; border: 2px solid red;")
        self.image_label.setMinimumHeight(350)
        self.image_label.setText("Kamera görüntüsü bekleniyor...")
        grid.addWidget(self.image_label, 0, 0)

        # 1. sütun 2. satır: 2'ye bölünmüş boş alan
        bottom_row_widget = QWidget()
        bottom_row_widget.setStyleSheet("border: 2px solid red;")
        bottom_row_layout = QHBoxLayout(bottom_row_widget)
        left_bottom_col1 = QFrame()
        left_bottom_col1.setFrameShape(QFrame.Shape.StyledPanel)
        left_bottom_col1.setStyleSheet("border: 2px solid red;")
        left_bottom_col2 = QFrame()
        left_bottom_col2.setFrameShape(QFrame.Shape.StyledPanel)
        left_bottom_col2.setStyleSheet("border: 2px solid red;")
        bottom_row_layout.addWidget(left_bottom_col1)
        bottom_row_layout.addWidget(left_bottom_col2)
        grid.addWidget(bottom_row_widget, 1, 0)

        # --- KONTROL PANELLERİ ---
        # left_bottom_col1 için:
        left_col1_vlayout = QVBoxLayout(left_bottom_col1)
        left_col1_vlayout.setSpacing(15)
        left_col1_vlayout.setContentsMargins(10, 10, 10, 10)

        # left_bottom_col2 için:
        left_col2_vlayout = QVBoxLayout(left_bottom_col2)
        left_col2_vlayout.setSpacing(15)
        left_col2_vlayout.setContentsMargins(10, 10, 10, 10)

        # Başlıklar
        control_label = QLabel("MANUEL JOINT and EFFECTOR CONTROL")
        control_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        control_label.setStyleSheet("font-size: 18px; font-weight: bold;")  # Bu kaldırılacak
        left_col1_vlayout.addWidget(control_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        calibration_label = QLabel("CALIBRATION")
        calibration_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        calibration_label.setStyleSheet("font-size: 18px; font-weight: bold;")  # Bu kaldırılacak
        left_col2_vlayout.addWidget(calibration_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # --- JOINT KONTROL SATIRLARI (GÜNCELLENECEK) ---
        def make_joint_row(name):
            # Buton
            btn = QPushButton(name)
            btn.setMinimumSize(80, 40)
            btn.setMaximumWidth(80)
            # ESKİ STİL KALDIRILACAK:
            # btn.setStyleSheet("""...""")
            # YENİ STİL:
            btn.setObjectName("joint-button")
            btn.clicked.connect(lambda: self.control_handlers.joint_button_clicked(name))

            # Position label
            position_label = QLabel("Position")
            position_label.setFixedWidth(60)
            position_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            position_label.setObjectName("joint-label")  # ESKİ: setStyleSheet("font-weight: bold;")
            
            # Power label
            power_label = QLabel("Power")
            power_label.setFixedWidth(50)
            power_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            power_label.setObjectName("joint-label")  # ESKİ: setStyleSheet("font-weight: bold;")
            
            position_slider = QSlider(Qt.Orientation.Horizontal)
            position_slider.setMinimum(-135)
            position_slider.setMaximum(135)
            position_slider.setValue(0)
            position_slider.setFixedWidth(100)
            position_value = QLabel(str(position_slider.value()))
            position_value.setFixedWidth(40)
            position_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            position_slider.valueChanged.connect(lambda val: position_value.setText(str(val)))
            # Position slider fonksiyonu ekle
            position_slider.valueChanged.connect(lambda val: self.control_handlers.position_slider_changed(name, val))

            # Power bar
            power_slider = QSlider(Qt.Orientation.Horizontal)
            power_slider.setMinimum(0)
            power_slider.setMaximum(100)
            power_slider.setValue(0)
            power_slider.setFixedWidth(90)
            power_value = QLabel(str(power_slider.value()))
            power_value.setFixedWidth(40)
            power_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            power_slider.valueChanged.connect(lambda val: power_value.setText(str(val)))
            # Power slider fonksiyonu ekle
            power_slider.valueChanged.connect(lambda val: self.control_handlers.power_slider_changed(name, val))

            # Satırı oluştur
            row = QHBoxLayout()
            row.setSpacing(10)
            row.addWidget(btn, alignment=Qt.AlignmentFlag.AlignVCenter)
            row.addWidget(position_label)
            row.addWidget(position_slider)
            row.addWidget(position_value)
            row.addWidget(power_label)
            row.addWidget(power_slider)
            row.addWidget(power_value)
            return row

        # Joint kontrollerini ekle
        left_col1_vlayout.addLayout(make_joint_row("Joint1"))
        left_col1_vlayout.addLayout(make_joint_row("Joint2"))
        left_col1_vlayout.addLayout(make_joint_row("Effector"))

        # Calibration butonları (GÜNCELLENECEK)
        left_col2_layout = QVBoxLayout()
        calib_buttons = []
        
        for name in ["Joint1", "Joint2", "Effector"]:
            btn = QPushButton(name)
            btn.setMinimumSize(120, 40)
            btn.setMaximumWidth(120)
            btn.setObjectName("calibration-default")  # ESKİ: setStyleSheet(col2_default_style)
            left_col2_layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignHCenter)
            calib_buttons.append(btn)

        left_col2_layout.setSpacing(20)
        left_col2_vlayout.addLayout(left_col2_layout)

        # Calibration buton fonksiyonları (GÜNCELLENECEK)
        def activate_button_temporarily(btn):
            self.control_handlers.calibration_button_clicked(btn.text())
            # Önce tüm butonları eski haline döndür
            for b in calib_buttons:
                b.setObjectName("calibration-default")
                self.apply_styles()  # Stilleri yeniden uygula
            # Tıklanan butonu sarı yap
            btn.setObjectName("calibration-processing")
            self.apply_styles()
            # 5 saniye sonra yeşil yap
            QTimer.singleShot(5000, lambda: [
                btn.setObjectName("calibration-completed"),
                self.apply_styles()
            ])

        # Butonlara tıklama bağlantısı
        for btn in calib_buttons:
            btn.clicked.connect(lambda checked, b=btn: activate_button_temporarily(b))

        # --- SAĞ SÜTUN ---
        # 2. sütun: Dikeyde 3'e böl
        # 1. Switchler (Toggle butonlar)
        switch_widget = QWidget()
        switch_widget.setStyleSheet("border: 2px solid red;")
        switch_layout = QVBoxLayout(switch_widget)
        
        # Toggle butonları (GÜNCELLENECEK)
        self.button1 = QPushButton("DEACTIVATED")
        self.button2 = QPushButton("MANUAL")
        self.button1.setCheckable(True)
        self.button2.setCheckable(True)
        
        # ESKİ STİLLER KALDIRILACAK
        self.button1.setObjectName("toggle-inactive")
        self.button2.setObjectName("toggle-inactive")

        # Toggle fonksiyonları (GÜNCELLENECEK)
        def toggle_button1_style(button):
            if button.isChecked():
                button.setText("ACTIVATED")
                button.setObjectName("toggle-active")
                self.control_handlers.activation_button_clicked("ACTIVATED")
            else:
                button.setText("DEACTIVATED")
                button.setObjectName("toggle-inactive")
                self.control_handlers.activation_button_clicked("DEACTIVATED")
            self.apply_styles()

        def toggle_button2_style(button):
            if button.isChecked():
                button.setText("AUTOMATIC")
                button.setObjectName("toggle-active")
                self.control_handlers.mode_button_clicked("AUTOMATIC")
            else:
                button.setText("MANUAL")
                button.setObjectName("toggle-inactive")
                self.control_handlers.mode_button_clicked("MANUAL")
            self.apply_styles()

        # Toggle sinyallerine bağla
        self.button1.toggled.connect(lambda checked: toggle_button1_style(self.button1))
        self.button2.toggled.connect(lambda checked: toggle_button2_style(self.button2))
        
        switch_layout.addWidget(self.button1)
        switch_layout.addWidget(self.button2)

        # 2. Boş kutu
        empty_box = QFrame()
        empty_box.setFrameShape(QFrame.Shape.StyledPanel)
        empty_box.setStyleSheet("border: 2px solid red;")

        # 3. Kapatma butonu kutusu (GÜNCELLENECEK)
        button_row_widget = HeightSyncWidget(bottom_row_widget)
        button_row_widget.setStyleSheet("border: 2px solid red;")
        button_row_layout = QVBoxLayout(button_row_widget)  # HBoxLayout'tan VBoxLayout'a değiştir
    
        # Kapatma butonu (GÜNCELLENECEK)
        self.action_button = QPushButton()
        self.action_button.setIcon(resource_manager.get_icon("closeApp.png"))
        self.action_button.setIconSize(QSize(120, 120))
        self.action_button.setText("")
        self.action_button.setFixedSize(150, 150)
        self.action_button.setObjectName("close-button")  # ESKİ: setStyleSheet("""...""")
        self.action_button.setToolTip("Uygulamayı Kapat")
        self.action_button.clicked.connect(self.control_handlers.close_app)
        button_row_layout.addWidget(self.action_button, alignment=Qt.AlignmentFlag.AlignCenter)

        # === TEMA SEÇİCİSİ KAPATMA BUTONU ALTINA ===
        # Tema label (GÜNCELLENECEK)
        theme_label = QLabel("Theme")
        theme_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        theme_label.setObjectName("theme-label")  # ESKİ: setStyleSheet("font-weight: bold; font-size: 12px; margin: 5px 0;")
        button_row_layout.addWidget(theme_label)
        
        # Tema butonları (GÜNCELLENECEK)
        theme_buttons_layout = QHBoxLayout()
        theme_buttons_layout.setSpacing(5)
        
        # Açık tema butonu
        self.light_theme_btn = QPushButton("☀️")
        self.light_theme_btn.setFixedSize(35, 35)
        self.light_theme_btn.setToolTip("Açık Tema")
        self.light_theme_btn.setObjectName("theme-button-normal")
        self.light_theme_btn.clicked.connect(lambda: self.control_handlers.change_theme("light_theme"))
        
        # Koyu tema butonu  
        self.dark_theme_btn = QPushButton("🌙")
        self.dark_theme_btn.setFixedSize(35, 35)
        self.dark_theme_btn.setToolTip("Koyu Tema")
        self.dark_theme_btn.setObjectName("theme-button-normal")
        self.dark_theme_btn.clicked.connect(lambda: self.control_handlers.change_theme("dark_theme"))

        theme_buttons_layout.addWidget(self.light_theme_btn)
        theme_buttons_layout.addWidget(self.dark_theme_btn)
        button_row_layout.addLayout(theme_buttons_layout)

        # SAĞ SÜTUNU GRİD'E EKLE (BU KISIM EKSİKTİ!)
        right_col_widget = QWidget()
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(0, 0, 0, 0)
        right_col_layout.setSpacing(0)
        right_col_layout.addWidget(switch_widget, 2)
        right_col_layout.addWidget(empty_box, 5)
        right_col_layout.addWidget(button_row_widget, 2)

        # Sağ sütunu grid'e ekle (BU SATIRLAR EKSİKTİ!)
        grid.addWidget(right_col_widget, 0, 1, 2, 1)

        # Help butonu ekle (image_label'in üzerine)
        self.help_btn = QPushButton("?")
        self.help_btn.setParent(self.image_label)
        self.help_btn.setFixedSize(40, 40)
        self.help_btn.setStyleSheet("""
            QPushButton {
                background-color: rgba(0, 120, 212, 0.8);
                color: white;
                border: none;
                border-radius: 20px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: rgba(0, 120, 212, 1.0);
            }
        """)
        self.help_btn.clicked.connect(self.control_handlers.show_manual_help)
        
        # Help butonunu konumlandır
        QTimer.singleShot(0, self._reposition_help_btn)

        # Panel kenarlıkları ObjectName ile ayarla
        self.image_label.setObjectName("panel-border")
        bottom_row_widget.setObjectName("panel-border")
        switch_widget.setObjectName("panel-border")
        empty_box.setObjectName("panel-border")
        button_row_widget.setObjectName("panel-border")
        left_bottom_col1.setObjectName("panel-border")
        left_bottom_col2.setObjectName("panel-border")

        # Varsayılan tema ve stilleri uygula
        self.control_handlers.change_theme("light_theme")
        self.apply_styles()

    def apply_styles(self):
        """Tüm component stillerini uygula"""
        try:
            # Genel temayı uygula
            theme_style = resource_manager.get_current_theme()
            
            # Özel component stillerini ekle
            joint_style = resource_manager.get_joint_button_style()
            calibration_style = resource_manager.get_style("calibration_buttons")
            toggle_style = resource_manager.get_style("toggle_buttons")
            close_style = resource_manager.get_close_button_style()
            theme_button_style = resource_manager.get_style("theme_buttons")
            label_style = resource_manager.get_style("labels")
            
            # Tüm stilleri birleştir
            combined_style = f"""
            {theme_style}
            {joint_style}
            {calibration_style}
            {toggle_style}
            {close_style}
            {theme_button_style}
            {label_style}
            """
            
            # Ana pencereye uygula
            self.setStyleSheet(combined_style)
            
        except Exception as e:
            print(f"Stil uygulama hatası: {e}")

    def _reposition_help_btn(self):
        """Help butonunu doğru konuma taşı"""
        try:
            if hasattr(self, 'help_btn') and hasattr(self, 'image_label'):
                # image_label'in boyutlarını al
                image_rect = self.image_label.geometry()
                
                # Help butonunu sağ üst köşeye yerleştir
                btn_size = 40
                margin = 10
                x = image_rect.width() - btn_size - margin
                y = margin
                
                self.help_btn.setGeometry(x, y, btn_size, btn_size)
        except Exception as e:
            print(f"Help butonu konumlama hatası: {e}")

    def resizeEvent(self, event):
        """Pencere boyutu değiştiğinde help butonunu yeniden konumlandır"""
        super().resizeEvent(event)
        self._reposition_help_btn()
            
    def closeEvent(self, event):
        """Pencere kapatılırken temizlik işlemleri"""
        if self.shutdown_initiated:
            event.accept()
            return
        self.shutdown_initiated = True
        
        print("Pencere kapatılıyor...")
        
        # ROS bridge'i kapat
        if hasattr(self, 'ros_bridge'):
            try:
                self.ros_bridge.shutdown()
            except Exception as e:
                print(f"ROS bridge kapatma hatası: {e}")
        
        # ROS'u kapat (eğer hala aktifse)
        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown("Pencere kapatıldı")
        except Exception as e:
            print(f"ROS shutdown hatası: {e}")
        
        # Olayı kabul et
        event.accept()

    def update_image(self, pixmap):
        """Görüntü güncelleme proxy metodu - control_handlers'a yönlendir"""
        if hasattr(self, 'control_handlers'):
            self.control_handlers.update_image(pixmap)
        else:
            rospy.logwarn("Control handlers bulunamadı!")