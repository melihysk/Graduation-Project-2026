"""
Tahmini enerji tüketimi hesabı — sabit güç modeli.

E = Σ(t_hareket × P_hareket + t_bekleme × P_idle)

Güç değerleri warehouseRobot_config.yaml'dan alınır:
  - mechanical_system.mass: 20 kg
  - ambient_system.power: 20 W (idle güç)
  - Hareket gücü: basitleştirilmiş model (v × m × g × μ + P_ambient)
"""


# Default power values (from warehouseRobot_config.yaml)
P_MOVING_W = 60.0    # Hareket halindeki güç tüketimi (W)
P_IDLE_W = 20.0      # Bekleme/idle güç tüketimi (W)
P_WAITING_W = 20.0   # Aktif bekleme (motorlar aktif ama hareket yok)


class EnergyEstimator:

    def __init__(self, logger, p_moving=P_MOVING_W, p_idle=P_IDLE_W, p_waiting=P_WAITING_W):
        self._logger = logger
        self.p_moving = p_moving
        self.p_idle = p_idle
        self.p_waiting = p_waiting

    def estimate(self, robot_metrics: dict) -> dict:
        """
        robot_metrics: RobotTracker.get_metrics()['per_robot'] dict.
        Returns energy estimates per robot and total.
        """
        total_energy_wh = 0.0
        per_robot_energy = {}

        for robot_name, data in robot_metrics.items():
            moving_sec = data.get("moving_time_sec", 0.0)
            waiting_sec = data.get("waiting_time_sec", 0.0)
            idle_sec = data.get("idle_time_sec", 0.0)
            charging_sec = data.get("charging_time_sec", 0.0)

            energy_j = (
                moving_sec * self.p_moving
                + waiting_sec * self.p_waiting
                + idle_sec * self.p_idle
                + charging_sec * self.p_idle
            )
            energy_wh = energy_j / 3600.0

            per_robot_energy[robot_name] = round(energy_wh, 4)
            total_energy_wh += energy_wh

        return {
            "total_energy_wh": round(total_energy_wh, 4),
            "per_robot_energy_wh": per_robot_energy,
            "power_model": {
                "P_moving_W": self.p_moving,
                "P_idle_W": self.p_idle,
                "P_waiting_W": self.p_waiting,
            },
        }
