#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ana pencere modülü:
GokHAS Merkezi Kontrol Arayüzü'nü başlatır ve yönetir.
"""
import sys
import rospy

# PyQt6 bileşenleri ve düzen yöneticileri
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QFrame,
    QPushButton, QSlider, QGridLayout, QHBoxLayout, QVBoxLayout,
    QLayout, QTextEdit
)
from PyQt6.QtCore import Qt, QSize, QTimer
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QSizePolicy, QStyle


# Uygulama içi kaynak ve kontrol işleyicileri
from ui.resources.resource import resource_manager
from ui.control_handlers import ControlHandlers
from ros.ros_bridge import ROSBridge


class HeightSyncWidget(QWidget):
    """
    Referans widget'ın yüksekliği kadar sabit yükseklik ayarlayan özel widget.
    """
    def __init__(self, reference_widget, parent=None):
        super().__init__(parent)
        # Yüksekliğini eşleyeceğimiz widget
        self.reference_widget = reference_widget

    def showEvent(self, event):
        super().showEvent(event)
        # Görünür olduğunda yüksekliği güncelle
        self.updateHeight()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Her yeniden boyutlama sonrası referans yükseklik ile eşitle
        self.updateHeight()

    def updateHeight(self):
        """
        Referans widget mevcut yükseklik > 0 ise,
        bu widget'ın yüksekliğini referansın yüksekliğine sabitle.
        """
        if self.reference_widget and self.reference_widget.height() > 0:
            self.setFixedHeight(self.reference_widget.height())


class MainWindow(QMainWindow):
    """
    Ana uygulama penceresi:
    - UI bileşenlerini oluştur
    - Tema ve stil yönetimi
    - ROS entegrasyonu
    """
    def __init__(self):
        super().__init__()
        # Kapama işlemi birden fazla tetiklenmesin
        self.shutdown_initiated = False

        # Pencere başlığı, boyutlandırma
        self.setWindowTitle("GokHAS Central Control Interface")
        self.setGeometry(100, 100, 1200, 800)

        # Kontroller için handler ve ROS köprüsü
        self.control_handlers = ControlHandlers(self)
        self.ros_bridge = ROSBridge(self)

        # UI oluşturma ve stil uygulama
        self._create_ui()
        self.control_handlers.change_theme("light_theme")
        self.apply_styles()

    def _create_ui(self):
        """
        Ana UI düzenini hazırlar:
        Sol: Kamera + kontrol panelleri
        Sağ: Toggle, boş panel, kapatma/tema bölümü
        """
        # Pencere ikonunu ayarla
        self._set_window_icon()

        # Central widget ve grid layout
        central = QWidget()
        self.setCentralWidget(central)
        grid = QGridLayout(central)
        grid.setColumnStretch(0, 5)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 5)
        grid.setRowStretch(1, 2)

        # ----- Sol Üst: Kamera Görüntüsü -----
        self.image_label = QLabel("Kamera görüntüsü bekleniyor...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumHeight(350)
        self.image_label.setStyleSheet("background-color: black; border: 2px solid red;")
        grid.addWidget(self.image_label, 0, 0)

        # ----- Sol Alt: Joint Kontrol ve Kalibrasyon -----
        bottom_widget = QWidget()
        bottom_widget.setStyleSheet("border: 2px solid red;")
        bottom_layout = QHBoxLayout(bottom_widget)
        # Kontrol ve kalibrasyon panelleri
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.Shape.StyledPanel)
        left_panel.setStyleSheet("border: 2px solid red;")
        right_panel = QFrame()
        right_panel.setFrameShape(QFrame.Shape.StyledPanel)
        right_panel.setStyleSheet("border: 2px solid red;")
        bottom_layout.addWidget(left_panel)
        bottom_layout.addWidget(right_panel)
        grid.addWidget(bottom_widget, 1, 0)

        # Joint kontrol panelini inşa et
        self._create_control_panel(left_panel, ["Joint1", "Joint2", "Effector"])
        # Kalibrasyon panelini inşa et
        self._create_calibration_panel(right_panel, ["Joint1", "Joint2", "Effector"])

        # ----- Sağ Yan Menü -----
        sidebar = QWidget()
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(2, 2, 2, 2)
        sidebar_layout.setSpacing(0)
        sidebar_layout.addWidget(self._create_toggle_section(), stretch=2)
        sidebar_layout.addWidget(self._create_empty_section(), stretch=5)
        sidebar_layout.addWidget(self._create_action_section(bottom_widget), stretch=2)
        grid.addWidget(sidebar, 0, 1, 2, 1)

        # Yardım butonu
        self._add_help_button()

    def _set_window_icon(self):
        """
        Logo varsa kullan, yoksa sistem bilgisayar ikonunu ata.
        """
        icon = resource_manager.get_image("logo.png")
        if not icon.isNull():
            self.setWindowIcon(QIcon(icon))
        else:
            style = self.style()
            if style:
                self.setWindowIcon(style.standardIcon(QStyle.StandardPixmap.SP_ComputerIcon))
            else:
                self.setWindowIcon(QIcon()) 

    def _create_control_panel(self, parent, names):
        """
        'names' listesi kadar joint control satırı oluşturur.
        Her satırda buton, position slider, power slider bulunur.
        """
        layout = QVBoxLayout(parent)
        layout.setSpacing(15)
        # Panel başlığı
        title = QLabel("MANUEL JOINT and EFFECTOR CONTROL")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0;")
        layout.addWidget(title)
        # Her isim için satır ekle
        for name in names:
            layout.addLayout(self._make_joint_row(name))

    def _make_joint_row(self, name):
        """
        Tek bir joint satırı oluşturur:
        - Hareket butonu (name)
        - Pozisyon slider + değer etiketi
        - Güç slider + değer etiketi
        """
        # Buton
        btn = QPushButton(name)
        btn.setObjectName("joint-button")
        btn.setFixedSize(80, 40)
        btn.clicked.connect(lambda _, n=name: self.control_handlers.joint_button_clicked(n))

        # Sliderlar
        pos_slider, pos_value = self._make_slider(-135, 135, name, "position")
        power_slider, power_value = self._make_slider(0, 100, name, "power")

        # Satırı oluştur
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(btn, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Pozisyon ve güç etiketleri ile sliderları ekle
        for label_text, slider, value in (("Position", pos_slider, pos_value),
                                          ("Power", power_slider, power_value)):
            lbl = QLabel(label_text)
            lbl.setObjectName("joint-label")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFixedWidth(60 if label_text == "Position" else 50)
            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(value)

        return row

    def _make_slider(self, min_val, max_val, name, attr):
        """
        Ortak slider üretici:
        - Aralık: min_val, max_val
        - change eventi ile hem etiket güncelleme hem de handler çağrısı
        """
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(0)
        slider.setFixedWidth(100 if attr == "position" else 90)

        value_label = QLabel("0")
        value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_label.setFixedWidth(40)

        # Etiket güncellemesi
        slider.valueChanged.connect(lambda val, lbl=value_label: lbl.setText(str(val)))
        # Handler bağlıyoruz
        if attr == "position":
            slider.valueChanged.connect(lambda val, n=name: self.control_handlers.position_slider_changed(n, val))
        else:
            slider.valueChanged.connect(lambda val, n=name: self.control_handlers.power_slider_changed(n, val))

        return slider, value_label

    def _create_calibration_panel(self, parent, names):
        """
        Kalibrasyon butonları panelini oluşturur.
        Butona tıklandığında control_handlers'taki handle_calibration_click metodunu çağırır.
        """
        layout = QVBoxLayout(parent)
        layout.setSpacing(15)
        title = QLabel("CALIBRATION")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0;")
        layout.addWidget(title)

        btns = []
        calib_layout = QVBoxLayout()
        calib_layout.setSpacing(20)
        for name in names:
            btn = QPushButton(name)
            btn.setObjectName("calibration-default")
            btn.setFixedSize(120, 40)
            # control_handlers'taki handle_calibration_click metodunu çağır
            btn.clicked.connect(lambda _, b=btn, btns=btns: self.control_handlers.handle_calibration_click(b, btns))
            calib_layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            btns.append(btn)
        layout.addLayout(calib_layout)

    def _create_toggle_section(self):
        """
        Sağ üstte iki toggle button:
        - Activation (DEACT/ACT)
        - Mode (MANUAL/AUTOMATIC)
        """
        widget = QWidget()
        widget.setStyleSheet("border: 2px solid red;")
        widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        layout = QVBoxLayout(widget)
        layout.setSpacing(5)

        # STATUS başlığı
        title = QLabel("STATUS")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin:0; padding:0;")
        layout.addWidget(title)

        # Toggle butonları
        entries = [
            ("DEACTIVATED", "ACTIVATED", self.control_handlers.activation_button_clicked),
            ("MANUAL",      "AUTONOMOUS", self.control_handlers.mode_button_clicked),
        ]
        for off_text, on_text, handler in entries:
            btn = QPushButton(off_text)
            btn.setCheckable(True)
            btn.setObjectName("toggle-inactive")
            btn.toggled.connect(
                lambda checked, b=btn, on=on_text, off=off_text, h=handler: 
                self.control_handlers.handle_toggle(b, checked, on, off, h)
            )
            layout.addWidget(btn)
        layout.addSpacing(5) 
        return widget

    def _create_empty_section(self):
        """Sağ ortadaki LOG panel."""
        empty = QFrame()
        empty.setFrameShape(QFrame.Shape.StyledPanel)
        empty.setStyleSheet("border: 2px solid red;")
        
        # Layout ekle
        layout = QVBoxLayout(empty)
        layout.setSpacing(5)
        
        # LOG başlığı ekle
        title = QLabel("LOG")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0;")
        layout.addWidget(title)
        
        # Küçük aralık
        layout.addSpacing(10)
        
        # Terminal benzeri metin alanı
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 10px;
                border: 1px solid #333;
                padding: 5px;
            }
        """)
        layout.addWidget(self.log_display)
        
        return empty

    def _create_action_section(self, reference):
        """
        Sağ alt: Kapatma butonu ve tema seçim butonları.
        """
        widget = HeightSyncWidget(reference)
        widget.setStyleSheet("border: 2px solid red;")
        layout = QVBoxLayout(widget)

        # Kapatma butonu
        btn = QPushButton()
        btn.setObjectName("close-button")
        btn.setIcon(resource_manager.get_icon("closeApp.png"))
        btn.setIconSize(QSize(120, 120))
        btn.setFixedSize(150, 150)
        btn.setToolTip("Uygulamayı Kapat")
        btn.clicked.connect(self.control_handlers.close_app)
        layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        # Tema başlığı ve butonları
        theme_lbl = QLabel("Theme")
        theme_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        theme_lbl.setObjectName("theme-label")
        layout.addWidget(theme_lbl)

        theme_layout = QHBoxLayout()
        for sym, theme in [("☀️", "light_theme"), ("🌙", "dark_theme")]:
            tbtn = QPushButton(sym)
            tbtn.setFixedSize(35, 35)
            tbtn.setToolTip("Açık Tema" if theme == "light_theme" else "Koyu Tema")
            
            # Initial tema light_theme olduğu için light butonunu aktif yap
            if theme == "light_theme":
                tbtn.setObjectName("theme-button-active")  # Light butonu başlangıçta mavi
            else:
                tbtn.setObjectName("theme-button-normal")   # Dark butonu başlangıçta normal
                
            tbtn.clicked.connect(lambda _, th=theme: self.control_handlers.handle_theme_change(th))
            theme_layout.addWidget(tbtn)
        layout.addLayout(theme_layout)

        return widget

    def _add_help_button(self):
        """Kamera görüntüsünün üzerine '?' butonu ekler."""
        self.help_btn = QPushButton("?", parent=self.image_label)
        self.help_btn.setObjectName("help-button")
        self.help_btn.setFixedSize(40, 40)
        self.help_btn.clicked.connect(self.control_handlers.show_manual_help)
        QTimer.singleShot(0, self._reposition_help_btn)

    def _reposition_help_btn(self):
        """Help butonunu image_label içinde sağ üst köşeye taşır."""
        rect = self.image_label.geometry()
        margin, size = 10, 40
        self.help_btn.setGeometry(rect.width() - size - margin, margin, size, size)

    def resizeEvent(self, event):
        """Pencere her yeniden boyutlandığında help butonunu güncelle."""
        super().resizeEvent(event)
        self._reposition_help_btn()

    def closeEvent(self, event):
        """
        Pencere kapatılırken ROS köprüsünü ve rospy'yu kapat.
        """
        if self.shutdown_initiated:
            event.accept()
            return
        self.shutdown_initiated = True
        self.ros_bridge.shutdown()
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Pencere kapatıldı")
        event.accept()

    def apply_styles(self):
        """
        Tüm widget'lara tema ve stil tanımlarını uygular.
        """
        try:
            styles = "\n".join([
                resource_manager.get_current_theme(),
                resource_manager.get_joint_button_style(),
                resource_manager.get_style("calibration_buttons"),
                resource_manager.get_style("toggle_buttons"),
                resource_manager.get_close_button_style(),
                resource_manager.get_style("theme_buttons"),
                resource_manager.get_style("labels"),
            ])
            self.setStyleSheet(styles)
        except Exception as e:
            print(f"Stil uygulama hatası: {e}")

    def add_log_message(self, message):
        """LOG paneline yeni mesaj ekler."""
        if hasattr(self, 'log_display'):
            from datetime import datetime
            timestamp = datetime.now().strftime("%H:%M:%S")
            formatted_message = f"[{timestamp}] {message}"
            self.log_display.append(formatted_message)
            # En son mesajı görmek için scroll'u en alta kaydır
            scrollbar = self.log_display.verticalScrollBar()
            if scrollbar is not None:
                scrollbar.setValue(scrollbar.maximum())
    
    def update_image(self, pixmap):
        """Kamera görüntüsünü günceller."""
        if hasattr(self, 'image_label'):
            self.image_label.setPixmap(pixmap)

def main():
    """
    Uygulamayı başlatır: QApplication oluştur, ana pencereyi göster, event loop.
    """
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
