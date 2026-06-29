# Çok Robotlu Depo Ortamında Trafik Yönetimi

ROS 2 (Humble) ve Open-RMF tabanlı depo simülasyonunda üç farklı trafik yönetim algoritmasını karşılaştıran mezuniyet projesi.

## Algoritmalar

| Mod | Yaklaşım | Açıklama |
|-----|----------|----------|
| **Open-RMF** | Reaktif | Varsayılan müzakere tabanlı CBS benzeri trafik yönetimi |
| **DKR** | Proaktif | Merkezi kaynak kilitleme + deadlock tespiti |
| **İDKR** | Gelişmiş Proaktif | Kontrol noktası yönetimi + Res1 mekanizması + çatışma sınıflandırma |

## Sistem Mimarisi

![Sistem Mimarisi](images/architecture.png)

## Simülasyon

Gazebo Harmonic üzerinde `warehouse_starter` haritasında 4 robot ile çalışır. RViz 2 ile gerçek zamanlı görselleştirme yapılır.

![Gazebo ve RViz](images/simulation.png)

## Dashboard (PyQt6)

Simülasyonları başlatma, metrik toplama ve algoritma karşılaştırma arayüzü.

**Simülasyon sekmesi** — algoritma ve senaryo seçimi:

![Simülasyon Sekmesi](images/dashboard_simulation.png)

**Karşılaştırma sekmesi** — verim, enerji, bekleme süresi gibi metriklerin karşılaştırılması:

![Karşılaştırma](images/dashboard_full.png)

## Proje Yapısı

```
src/
├── dkr_controller/        # DKR: Kaynak kilitleme + deadlock tespiti
├── idkr_controller/       # İDKR: CP yönetimi + Res1 + çatışma sınıflandırma
├── metric_logger/         # Deney metriklerini toplayan ROS 2 paketi
├── reservation_viz/       # RViz2 kaynak rezervasyon görselleştirme
└── rmf_demos/             # Open-RMF demo paketleri (warehouse_starter)
dashboard/                 # PyQt6 gösterge paneli
images/                    # Ekran görüntüleri
```

## Gereksinimler

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- Open-RMF paketleri
- Python 3.10+

## Kurulum

```bash
# Workspace oluştur ve kaynak kodu klonla
mkdir -p ~/rmf_ws/src && cd ~/rmf_ws
git clone <repo-url> src/graduation_project

# ROS 2 bağımlılıklarını yükle
rosdep install --from-paths src --ignore-src -r -y

# Derle
colcon build --symlink-install

# Dashboard bağımlılıkları
pip install -r src/graduation_project/dashboard/requirements.txt
```

## Çalıştırma

```bash
# Simülasyonu başlat (Open-RMF modu)
source install/setup.bash
ros2 launch rmf_demos warehouse_starter.launch.xml

# DKR modunda çalıştır
ros2 launch dkr_controller dkr_standalone.launch.py

# İDKR modunda çalıştır
ros2 launch idkr_controller idkr_standalone.launch.py

# Dashboard
cd dashboard && python main.py
```

## Senaryolar

| Senaryo | Görev Sayısı | Açıklama |
|---------|-------------|----------|
| Normal | 10 | Standart teslimat görevleri |
| Dar Koridor | 12 | Dar geçitlerde yoğun trafik |
| Yoğun Trafik | 16 | Yüksek görev yükü |

## UML Sınıf Diyagramı

![UML Class](images/uml_class.png)

## Lisans

Bu proje bir üniversite mezuniyet projesidir.
