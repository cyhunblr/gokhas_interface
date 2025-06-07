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
        self.show_borders = False

        # Pencere başlığı, boyutlandırma
        self.setWindowTitle("GokHAS Central Control Interface")
        self.setGeometry(100, 100, 1200, 800)

        # Kontroller için handler
        self.control_handlers = ControlHandlers(self)

        # UI oluşturma
        self._create_ui()

        #ROS köprüsü
        self.ros_bridge = ROSBridge(self)

        #stil uygulama
        self.control_handlers.change_theme("light_theme")
        self.apply_styles()

    def _create_ui(self):
        """
        Ana UI düzenini hazırlar:
        Sol: Kamera + kontrol panelleri
        Sağ: Toggle, log paneli, kapatma/tema/border bölümü
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

        border = "2px solid red" if self.show_borders else "none"

        # ----- Sol Üst: Kamera Görüntüsü -----
        self.image_label = QLabel("Kamera görüntüsü bekleniyor...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setScaledContents(True)
        self.image_label.setStyleSheet(f"background-color: black; border: {border};")
        self.image_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        grid.addWidget(self.image_label, 0, 0)

        # ----- Sol Alt: Joint Kontrol ve Kalibrasyon -----
        self.bottom_widget = QWidget()
        self.bottom_widget.setStyleSheet(f"border: {border};")
        self.bottom_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        bottom_layout = QHBoxLayout(self.bottom_widget)

        self.left_panel = QFrame()
        self.left_panel.setFrameShape(QFrame.Shape.StyledPanel)
        self.left_panel.setStyleSheet(f"border: {border};")
        self.left_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.right_panel = QFrame()
        self.right_panel.setFrameShape(QFrame.Shape.StyledPanel)
        self.right_panel.setStyleSheet(f"border: {border};")
        self.right_panel.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.right_panel.setFixedWidth(120)

        bottom_layout.addWidget(self.left_panel, stretch=3)
        bottom_layout.addWidget(self.right_panel, stretch=0)

        grid.addWidget(self.bottom_widget, 1, 0)

        # Panel içerikleri
        self._create_control_panel(self.left_panel, ["Joint1", "Joint2", "Effector"])
        self._create_calibration_panel(self.right_panel, ["Joint1", "Joint2", "Effector"])

        # ----- Sağ Yan Menü -----
        sidebar = QWidget()
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(2, 2, 2, 2)
        sidebar_layout.setSpacing(0)

        self.sidebar_toggle = self._create_toggle_section()
        self.sidebar_log = self._create_empty_section()
        self.sidebar_action = self._create_action_section(self.bottom_widget)

        self.sidebar_toggle.setStyleSheet(f"border: {border};")
        self.sidebar_log.setStyleSheet(f"border: {border};")
        self.sidebar_action.setStyleSheet(f"border: {border};")

        sidebar_layout.addWidget(self.sidebar_toggle, stretch=2)
        sidebar_layout.addWidget(self.sidebar_log, stretch=5)
        sidebar_layout.addWidget(self.sidebar_action, stretch=2)

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
        layout.setSpacing(25)
        layout.setContentsMargins(5, 5, 5, 5)        
        # Panel başlığı
        title = QLabel("MANUEL JOINT and EFFECTOR CONTROL")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
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
        layout.setSpacing(10)
        layout.setContentsMargins(5, 5, 5, 5)
        title = QLabel("CALIBRATION")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0;")
        layout.addWidget(title)

        btns = []
        calib_layout = QVBoxLayout()
        calib_layout.setSpacing(10)
        for name in names:
            btn = QPushButton(name)
            btn.setObjectName("calibration-default")
            btn.setFixedSize(100, 35)
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
        theme_lbl.setStyleSheet("font-size: 12px; margin: 0; padding: 0;")
        layout.addWidget(theme_lbl)

        theme_layout = QHBoxLayout()

        # Tema butonları
        for sym, theme in [("☀️", "light_theme"), ("🌙", "dark_theme")]:
            tbtn = QPushButton(sym)
            tbtn.setFixedSize(35, 35)
            tbtn.setToolTip("Açık Tema" if theme == "light_theme" else "Koyu Tema")
            tbtn.setObjectName("theme-button-active" if theme == "light_theme" else "theme-button-normal")
            tbtn.clicked.connect(lambda _, th=theme: self.control_handlers.handle_theme_change(th))
            theme_layout.addWidget(tbtn)

        # Border Toggle butonu
        border_btn = QPushButton("B")
        border_btn.setFixedSize(35, 35)
        border_btn.setToolTip("Debug Borderlarını Aç/Kapat")
        border_btn.setObjectName("border-button-normal")  # Başlangıçta kapalı
        border_btn.clicked.connect(lambda: self.control_handlers.handle_border_toggle(border_btn))
        theme_layout.addWidget(border_btn)

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
        if self.image_label and self.help_btn:
            size = self.help_btn.size()
            margin = 10
            label_width = self.image_label.width()
            self.help_btn.move(label_width - size.width() - margin, margin)

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
    
    def update_borders(self):
        """Tüm bileşenlerin kenarlık stilini günceller, diğer stilleri bozmadan."""
        border = "2px solid red" if self.show_borders else "none"

        # Kamera etiketi: Arka planı korunmalı
        self.image_label.setStyleSheet(
            f"background-color: black; border: {border};"
        )

        # Diğer widget'lar: sadece border içeriyor
        other_widgets = [
            self.left_panel,
            self.right_panel,
            self.bottom_widget,
            self.sidebar_toggle,
            self.sidebar_log,
            self.sidebar_action,
        ]
        for widget in other_widgets:
            widget.setStyleSheet(f"border: {border};")



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
