# GokHAS Interface

Python ve Qt6 temelli bu GUI, ROS Noetic sistemleriyle etkileşimi kolaylaştırır.  
Görsel bir arayüz üzerinden **joint** ve **effector** kontrollerini manuel veya otonom modda yönetebilir, kalibrasyon adımlarını gerçekleştirebilir ve gerçek zamanlı kameradan gelen görüntüyü izleyebilirsiniz.

## Özellikler

- **Gerçek Zamanlı Görüntü**  
  `/image_raw` ROS konusundan gelen görüntüyü alır ve ekranda gösterir.  
- **Manuel Joint & Effector Kontrol**  
  - Her joint için **Position** (-135…+135°) ve **Power** (0…100%) kaydırıcıları  
  - “MANUAL” moddayken sağ üst köşede soru işareti ikonu ile yardım popup’ı  
- **Otonom/Kapatma Modu**  
  - “ACTIVATED/DEACTIVATED” anahtar butonuyla tüm kontrol elemanlarını etkinleştirme/devre dışı bırakma  
- **Kalibrasyon**  
  - `Joint1`, `Joint2` ve `Effector` için ayrıntılı kalibrasyon butonları  
- **Kolay Kurulum ve Çalıştırma**  
  ROS ortamında tek komutla arayüzü ayağa kaldırın.

## Kurulum

1. Python 3 ve ROS Noetic kurulu olmalı  
2. Gerekli Python paketlerini yükleyin  
   ```
   pip install PyQt6 rospkg
   ```  
3. ROS paketi derleyin  
   ```
   catkin build
   source devel/setup.bash
   ```

## Kullanım

Aşağıdaki komut arayüzü başlatır ve tüm ROS topic’lerine bağlanır:

```
roslaunch gokhas_interface interface.launch
```

- **MANUAL** moddayken ekranda beliren soru işareti ikonuna tıklayarak yardım metnini görebilirsiniz.  
- **DEACTIVATED** modunda sadece kapatma butonu dışındaki tüm kontroller kilitlenir.

## Proje Yapısı

```
gokhas_interface/
├── launch/
│   └── interface.launch       # ROS launch dosyası
├── scripts/
│   ├── launch_interface.py    # Arayüzü başlatan script
│   └── camera_publisher.py    # Test için kamera yayıncısı
├── src/
│   ├── main.py
│   ├── ui/
│   │   └── main_window.py     # Qt arayüz kodları
│   └── ros/
│       └── ros_bridge.py      # ROS-Python köprü adaptörü
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

## Katkıda Bulunma

Yeni özellik önerileri ve hata bildirimleri için lütfen pull request oluşturun veya issue açın.

## Lisans

MIT © 2025