# Çalıştırma Rehberi

## Ön Hazırlık (Build)

```bash
cd ~/Desktop/graduation_project
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## MOD 1 — RMF Baseline (Gazebo ile)

RMF'in kendi trafik yönetimi aktif. Robotlar Gazebo'da simüle edilir.

### Terminal 1 — RMF + Gazebo

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch rmf_demos_gz warehouse_starter.launch.xml
```

Bu komut şunları başlatır:
- Gazebo Harmonic (warehouse_starter.world)
- rmf_traffic_schedule (RMF trafik yönetimi)
- rmf_traffic_blockade
- rmf_fleet_adapter + fleet_manager
- rmf_task_dispatcher
- RViz görselleştirme
- /clock bridge (sim time)

### Terminal 2 — Metric Logger + Task Dispatcher

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch metric_logger metric_logger.launch.py \
    traffic_mode:=rmf \
    scenario_id:=normal \
    run_id:=1 \
    expected_tasks:=10 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml
```

### Çıktı

Deney bitince: `~/Desktop/graduation_project/results/rmf/normal/run_001.json`

---

## MOD 2 — DKR (Gazebo ile)

DKR trafik yönetimi aktif. RMF traffic_schedule **kalır** (fleet adapter'ın path planning
ve task bidding yapabilmesi için gerekli), ancak `rmf_traffic_blockade` **kaldırıldı**.
Gerçek trafik enforcement'ı DKR resource reservation ile yapılır.

Karşılaştırma mantığı:
- RMF modu: schedule + negotiation + blockade (tam RMF stack)
- DKR modu: schedule (sadece planlama), enforcement = DKR kaynak rezervasyonu

### Terminal 1 — RMF (blockade yok) + Gazebo

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch rmf_demos_gz warehouse_starter_dkr.launch.xml
```

### Terminal 2 — DKR Controller + Metric Logger + Task Dispatcher

```bash
cd ~/Desktop/graduation_project && source install/setup.bash
ros2 launch dkr_controller dkr_controller.launch.py \
    scenario_id:=normal \
    run_id:=1 \
    expected_tasks:=10 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml

ros2 launch dkr_controller dkr_controller.launch.py \
    scenario_id:=bottlenck \
    run_id:=1 \
    expected_tasks:=12 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_bottleneck.yaml

ros2 launch dkr_controller dkr_controller.launch.py \
    scenario_id:=high_density \
    run_id:=1 \
    expected_tasks:=16 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_high_density.yaml
```

### Çıktı

Deney bitince: `~/Desktop/graduation_project/results/dkr/normal/run_001.json`

---

## Senaryolar

| Senaryo | Dosya | expected_tasks | Açıklama |
|---------|-------|----------------|----------|
| normal | `scenario_normal.yaml` | 10 | Geniş koridor, aralıklı görevler |
| bottleneck | `scenario_bottleneck.yaml` | 12 | Dar koridor, karşılıklı trafik |
| high_density | `scenario_high_density.yaml` | 16 | Sürekli görev akışı, yoğun trafik |

### Bottleneck senaryosu örneği (DKR):

```bash
# Terminal 1
ros2 launch rmf_demos_gz warehouse_starter_dkr.launch.xml

# Terminal 2
ros2 launch dkr_controller dkr_controller.launch.py \
    scenario_id:=bottleneck \
    run_id:=1 \
    expected_tasks:=12 \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_bottleneck.yaml
```

---

## Tekrarlı Deneyler (10 run)

Her senaryo × mod için 10 tekrar:

```bash
# Örnek: DKR + normal × 10 tekrar
for i in $(seq 1 10); do
  echo "=== Run $i ==="

  # Terminal 1'de warehouse_starter_dkr.launch.xml çalışıyor olmalı

  ros2 launch dkr_controller dkr_controller.launch.py \
      scenario_id:=normal \
      run_id:=$i \
      expected_tasks:=10 \
      scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml

  sleep 5  # Gazebo'nun resetlenmesi için
done
```

---

## Headless (GUI'siz) Çalıştırma

Grafik ortamı gerektirmeyen batch deneyleri:

```bash
ros2 launch rmf_demos_gz warehouse_starter.launch.xml headless:=true
```

---

## Sonuçları Kontrol Etme

```bash
# Son çalışma sonuçlarını gör
cat results/rmf/normal/run_001.json | python3 -m json.tool

# Tüm sonuçları listele
find results/ -name "*.json" | sort
```

---

## DKR'nin Gerçekten Çalıştığını Doğrulama

Terminal 2 loglarında şunları görmelisiniz:
- `[dkr_traffic_manager] New path: ... waypoints, ... resources`
- `dkr_grant_count` > 0 olan `results/dkr/normal/run_001.json`

Şunlar DKR'nin **çalışmadığını** gösterir:
- `dkr_grant_count: 0` ve sadece RMF `Conflict #N detected` mesajları
- DKR hiç `New path` loglamıyorsa → Terminal 1'i **yeniden** `warehouse_starter_dkr.launch.xml` ile başlatın (topic remap gerekli)

---

## Notlar

- Gazebo sim time kullanır (`use_sim_time:=true`). Tüm node'lar /clock'a bağlıdır.
- Gazebo olmadan hızlı test için `mock_robot_state_publisher.py` kullanılabilir (bkz. aşağı).
- DKR modunda `rmf_traffic_schedule` kalır (fleet adapter path planning için), `rmf_traffic_blockade` kaldırılır — DKR kendi enforcement'ını yapar.
- Robot sayısı `warehouseRobot_config.yaml`'da 4 olarak tanımlı (senaryolar 3-4 robot kullanır).

### Mock Robot ile Hızlı Test (Gazebo'suz)

Gazebo başlatmadan, çok daha hızlı çalışan test modu:

```bash
# Terminal 1 — RMF (Gazebo'suz)
ros2 launch rmf_demos warehouse_starter.launch.xml use_sim_time:=false

# Terminal 2 — Mock robot simülasyonu
python3 mock_robot_state_publisher.py --ros-args -p speed:=5.0 -p num_robots:=3

# Terminal 3 — Metric logger
ros2 launch metric_logger metric_logger.launch.py \
    traffic_mode:=rmf scenario_id:=normal run_id:=1 expected_tasks:=10 \
    use_sim_time:=false \
    scenario_file:=$(ros2 pkg prefix metric_logger)/share/metric_logger/config/scenarios/scenario_normal.yaml
```
