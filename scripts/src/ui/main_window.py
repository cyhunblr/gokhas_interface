from PyQt6.QtWidgets import (
    QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QWidget, QLabel, QFrame, QCheckBox, QApplication,
    QSizePolicy, QSlider, QToolButton, QMessageBox, QStyle
)
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QPixmap, QImage, QIcon
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

        # --- BURASI YENİ EKLENECEK KISIM ---
        # left_bottom_col1 için:
        left_col1_vlayout = QVBoxLayout(left_bottom_col1)
        left_col1_vlayout.setSpacing(15)
        left_col1_vlayout.setContentsMargins(10, 10, 10, 10)

        # left_bottom_col2 için:
        left_col2_vlayout = QVBoxLayout(left_bottom_col2)
        left_col2_vlayout.setSpacing(15)
        left_col2_vlayout.setContentsMargins(10, 10, 10, 10)

        # Başlıklar (aynı stil ve hizalama)
        label_style = "font-size: 18px; font-weight: bold;"
        control_label = QLabel("MANUEL JOINT and EFFECTOR CONTROL")
        control_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        control_label.setStyleSheet(label_style)
        left_col1_vlayout.addWidget(control_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        calibration_label = QLabel("CALIBRATION")
        calibration_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        calibration_label.setStyleSheet(label_style)
        left_col2_vlayout.addWidget(calibration_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # --- JOINT KONTROL SATIRLARI ---
        def make_joint_row(name):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.setMinimumSize(120, 40)
            btn.setMaximumWidth(120)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #cccccc;
                    color: black;
                    border: 1px solid #888888;
                    border-radius: 5px;
                    padding: 6px;
                    font-weight: bold;
                }
                QPushButton:checked {
                    background-color: #55bb55;
                    color: white;
                }
            """)

            # Position bar
            position_label = QLabel("Position")
            position_label.setFixedWidth(65)
            position_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            position_label.setStyleSheet("font-weight: bold;")
            position_slider = QSlider(Qt.Orientation.Horizontal)
            position_slider.setMinimum(-135)
            position_slider.setMaximum(135)
            position_slider.setValue(0)
            position_slider.setFixedWidth(120)
            position_value = QLabel(str(position_slider.value()))
            position_value.setFixedWidth(40)
            position_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            position_slider.valueChanged.connect(lambda val: position_value.setText(str(val)))

            # Power bar
            power_label = QLabel("Power")
            power_label.setFixedWidth(50)
            power_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            power_label.setStyleSheet("font-weight: bold;")
            power_slider = QSlider(Qt.Orientation.Horizontal)
            power_slider.setMinimum(0)
            power_slider.setMaximum(100)
            power_slider.setValue(0)
            power_slider.setFixedWidth(90)
            power_value = QLabel(str(power_slider.value()))
            power_value.setFixedWidth(40)
            power_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            power_slider.valueChanged.connect(lambda val: power_value.setText(str(val)))

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

        # Joint1
        left_col1_vlayout.addLayout(make_joint_row("Joint1"))
        # Joint2
        left_col1_vlayout.addLayout(make_joint_row("Joint2"))
        # Effector
        left_col1_vlayout.addLayout(make_joint_row("Effector"))

        # Calibration kısmı için butonlar (aynı genişlik ve margin)
        left_col2_layout = QVBoxLayout()
        calib_buttons = []
        for name in ["Joint1", "Joint2", "Effector"]:
            btn = QPushButton(name)
            btn.setMinimumSize(120, 40)
            btn.setMaximumWidth(120)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #cccccc;
                    color: black;
                    border: 1px solid #888888;
                    border-radius: 5px;
                    padding: 6px;
                    font-weight: bold;
                }
            """)
            left_col2_layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignHCenter)
            calib_buttons.append(btn)
        left_col2_layout.setSpacing(20)
        left_col2_vlayout.addLayout(left_col2_layout)

        # left_bottom_col2 butonları için stiller
        col2_default_style = """
            QPushButton {
                background-color: #cccccc;
                color: black;
                border: 1px solid #888888;
                border-radius: 5px;
                padding: 6px;
                font-weight: bold;
            }
        """
        col2_yellow_style = """
            QPushButton {
                background-color: #ffe066;
                color: black;
                border: 1px solid #bba100;
                border-radius: 5px;
                padding: 6px;
                font-weight: bold;
            }
        """
        col2_active_style = """
            QPushButton {
                background-color: #55bb55;
                color: white;
                border: 1px solid #339933;
                border-radius: 5px;
                padding: 6px;
                font-weight: bold;
            }
        """

        from PyQt6.QtCore import QTimer

        # Hepsi için ortak bir fonksiyon
        def activate_button_temporarily(btn):
            # Önce tüm butonları eski haline döndür
            for b in calib_buttons:
                b.setStyleSheet(col2_default_style)
            # Tıklanan butonu sarı yap
            btn.setStyleSheet(col2_yellow_style)
            # 5 saniye sonra yeşil yap
            QTimer.singleShot(5000, lambda: btn.setStyleSheet(col2_active_style))

        # Butonlara tıklma bağlantısı
        for btn in calib_buttons:
            btn.clicked.connect(lambda checked, b=btn: activate_button_temporarily(b))

        # 2. sütun: Dikeyde 3'e böl
        # 1. row: Switchler -> Butonlar olarak değiştirildi
        switch_widget = QWidget()
        switch_widget.setStyleSheet("border: 2px solid red;")
        switch_layout = QVBoxLayout(switch_widget)
        
        # Switch'leri butonlara dönüştürelim
        self.button1 = QPushButton("DEACTIVATED")
        self.button2 = QPushButton("MANUAL")  # Değişti: DEACTIVATED -> MANUAL
        
        # Butonları toggle button haline getir
        self.button1.setCheckable(True)
        self.button2.setCheckable(True)
        
        # Butonların default stilini kırmızı yap
        button_inactive_style = """
            QPushButton {
                background-color: #ff5555;
                color: white;
                border: 1px solid #c03030;
                border-radius: 5px;
                padding: 8px;
                font-weight: bold;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #ff7777;
            }
            QPushButton:pressed {
                background-color: #cc3333;
            }
        """
        
        # Butonların aktif stilini yeşil yap
        button_active_style = """
            QPushButton {
                background-color: #55bb55;
                color: white;
                border: 1px solid #30a030;
                border-radius: 5px;
                padding: 8px;
                font-weight: bold;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #77cc77;
            }
            QPushButton:pressed {
                background-color: #339933;
            }
        """
        
        # Başlangıçta inaktif stil uygula
        self.button1.setStyleSheet(button_inactive_style)
        self.button2.setStyleSheet(button_inactive_style)
        
        # Toggle olduğunda stil ve metin değişikliği - button1
        def toggle_button1_style(button):
            if button.isChecked():
                button.setText("ACTIVATED")
                button.setStyleSheet(button_active_style)
            else:
                button.setText("DEACTIVATED")
                button.setStyleSheet(button_inactive_style)
        
        # Toggle olduğunda stil ve metin değişikliği - button2
        def toggle_button2_style(button):
            if button.isChecked():
                button.setText("AUTONOMOUS")  # Değişti: ACTIVATED -> AUTONOMOUS
                button.setStyleSheet(button_active_style)
            else:
                button.setText("MANUAL")  # Değişti: DEACTIVATED -> MANUAL
                button.setStyleSheet(button_inactive_style)
        
        # Toggle sinyallerine bağla - her buton için ayrı fonksiyon
        self.button1.toggled.connect(lambda checked: toggle_button1_style(self.button1))
        self.button2.toggled.connect(lambda checked: toggle_button2_style(self.button2))
        
        # Butonları kutu içinde sığdır ve dikey olarak büyüt
        switch_layout.addWidget(self.button1, 1)  # 1 birim stretch
        switch_layout.addWidget(self.button2, 1)  # 1 birim stretch
        
        # Butonların minimum yüksekliklerini ayarla
        self.button1.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.button2.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        # Butonlar arasına boşluk ekle
        switch_layout.setSpacing(10)
        
        # Butonların etrafında biraz boşluk bırak
        switch_layout.setContentsMargins(10, 10, 10, 10)

        # 2. row: Boş kutu
        empty_box = QFrame()
        empty_box.setFrameShape(QFrame.Shape.StyledPanel)
        empty_box.setStyleSheet("border: 2px solid red;")

        # 3. row: Kapatma butonu kutusu (1. sütun 2. row ile hizalı)
        # Yeni HeightSyncWidget sınıfını kullanıyoruz
        button_row_widget = HeightSyncWidget(bottom_row_widget)
        button_row_widget.setStyleSheet("border: 2px solid red;")
        button_row_layout = QHBoxLayout(button_row_widget)
        self.action_button = QPushButton()
        self.action_button.setText("")
        assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
        icon_path = os.path.join(assets_dir, "closeApp.png")
        if os.path.exists(icon_path):
            self.action_button.setIcon(QIcon(icon_path))
            self.action_button.setIconSize(QSize(150, 150))
        else:
            print(f"İkon bulunamadı: {icon_path}")
        self.action_button.setFixedSize(75, 75)
        self.action_button.setStyleSheet("""
            QPushButton {
                border: none;
                background: transparent;
                padding: 0px;
            }
            QPushButton:pressed {
                background: #dddddd;
            }
        """)
        self.action_button.setToolTip("Kapat")
        self.action_button.clicked.connect(self.close_app)
        button_row_layout.addWidget(self.action_button, alignment=Qt.AlignmentFlag.AlignCenter)

        # 2. sütunu QVBoxLayout ile oluşturup grid'e ekle
        right_col_widget = QWidget()
        right_col_layout = QVBoxLayout(right_col_widget)
        right_col_layout.setContentsMargins(0, 0, 0, 0)
        right_col_layout.setSpacing(0)
        right_col_layout.addWidget(switch_widget, 2)   # 1. row (küçüldü)
        right_col_layout.addWidget(empty_box, 5)       # 2. row (büyütüldü)
        right_col_layout.addWidget(button_row_widget, 2) # 3. row (buton kutusu)

        # Sorunlu lambda-tabanlı resizeEvent özelleştirmesini kaldırdık
        # ve yerine özel HeightSyncWidget sınıfımızı kullandık

        grid.addWidget(right_col_widget, 0, 1, 2, 1)  # 2 satırı kapsayacak şekilde ekle

        # ROS bağlantısı
        try:
            self.ros_bridge = ROSBridge(self)
        except Exception as e:
            rospy.logerr(f"ROS Bridge başlatılamadı: {e}")

        # (1) right_col_layout zaten tanımlı: image_label oraya eklenmiş
        # -> şimdi soru ikonu ekleniyor
        self.help_btn = QToolButton()
        self.help_btn.setIcon(self.style()
            .standardIcon(QStyle.StandardPixmap.SP_MessageBoxQuestion))
        self.help_btn.setAutoRaise(True)
        self.help_btn.clicked.connect(self.show_manual_help)
        self.help_btn.hide()  # başta gizli
        
        # Sağ üst köşeye yerleştir
        # help_btn'i image_label'in çocuğu yap ve hemen listen
        self.help_btn.setParent(self.image_label)
        self.help_btn.move(
            self.image_label.width() - self.help_btn.width() - 10,
            10
        )

         # (2) Manuel moda geçişte göster/gizle
        # isChecked==False iken (MANUAL moddayken) göster
        self.button2.toggled.connect(lambda on: self.help_btn.setVisible(not on))
        self.help_btn.setVisible(not self.button2.isChecked())

        # pencere tamamen oluştuktan sonra 0ms gecikmeyle doğru konuma taşı
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(0, self._reposition_help_btn)

    def update_image(self, pixmap):
        self.image_label.setPixmap(pixmap)

    def close_app(self):
        print("Kullanıcı arayüzden çıkış yaptı.")
        try:
            rospy.signal_shutdown("Kullanıcı arayüzden çıkış yaptı.")
        except Exception:
            pass
        QApplication.instance().quit()

    def show_manual_help(self):
        QMessageBox.information(
            self,
            "Manual Mode Help",
            "Bu modda joint ve effector kontrollerini elle ayarlayabilirsiniz.\n"
            "• Butonları tıklayın.\n"
            "• Slider’ları sürükleyin.\n"
            "• Değerler anında güncellenir."
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # image_label her yeniden boyutlanınca help_btn'i yeniden konumlandır
        self._reposition_help_btn()

    def _reposition_help_btn(self):
        if hasattr(self, 'help_btn') and hasattr(self, 'image_label'):
            x = self.image_label.width() - self.help_btn.width() - 10
            y = 10
            self.help_btn.move(x, y)