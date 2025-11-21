# New Scripts/overwatch/overwatch_manager.py
"""
OverwatchManager: continuous UAV overwatch for UGV
- Tracks UGV progress toward a GPS-like target
- Computes deviation from path
- Produces BODY_ENU error (x,y) so a controller can output (v,w)
- Safety: arrival radius, mission timeout, loss-of-sight handling
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import math
import time

from overwatch_controller import OverwatchController, OverwatchControllerConfig

@dataclass
class SimpleLocation:
    lat: float
    lon: float
    alt: float = 0.0

@dataclass
class UGVState:
    latitude: float
    longitude: float
    heading: float
    timestamp: float
    distance_to_target: float
    deviation_from_path: float

@dataclass
class OverwatchConfig:
    # Deviation thresholds
    max_deviation_meters: float = 1.5
    correction_threshold_meters: float = 0.8

    # Target arrival
    arrival_radius_meters: float = 1.5

    # Monitoring
    update_rate_hz: float = 2.0

    # Safety
    max_mission_time_seconds: float = 420.0
    loss_of_sight_timeout_seconds: float = 3.0

    # Velocity controller config
    velocity_controller_config: Optional[OverwatchControllerConfig] = None

class OverwatchManager:
    def __init__(self, target_location: SimpleLocation, config: Optional[OverwatchConfig] = None):
        self.target = target_location
        self.config = config or OverwatchConfig()
        vel_cfg = self.config.velocity_controller_config or OverwatchControllerConfig()
        self.velocity_controller = OverwatchController(vel_cfg)
        self.mission_start_time: Optional[float] = None
        self.last_ugv_update: Optional[float] = None
        self.ugv_state: Optional[UGVState] = None
        self.total_corrections = 0
        self.max_deviation_seen = 0.0

    # --- Geodesy helpers (small-distance approximations) ---
    def _distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R = 6371000.0
        phi1 = math.radians(lat1); phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlmb = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
        return 2*R*math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def _bearing_deg(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        phi1 = math.radians(lat1); phi2 = math.radians(lat2)
        dlmb = math.radians(lon2 - lon1)
        y = math.sin(dlmb) * math.cos(phi2)
        x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlmb)
        brg = math.degrees(math.atan2(y, x))
        return (brg + 360) % 360

    def start_mission(self):
        self.mission_start_time = time.time()
        print("\n================ OVERWATCH: MISSION STARTED ================\n"
              f"Target: ({self.target.lat:.8f}, {self.target.lon:.8f})\n"
              f"Max mission time: {self.config.max_mission_time_seconds:.0f}s\n"
              f"Deviation threshold: {self.config.max_deviation_meters:.2f}m\n")

    def calculate_deviation(self, ugv_lat: float, ugv_lon: float, start_lat: float, start_lon: float) -> float:
        ds_ugv = self._distance_m(start_lat, start_lon, ugv_lat, ugv_lon)
        ds_tgt = self._distance_m(start_lat, start_lon, self.target.lat, self.target.lon)
        du_tgt = self._distance_m(ugv_lat, ugv_lon, self.target.lat, self.target.lon)
        if du_tgt < self.config.arrival_radius_meters or ds_ugv == 0 or ds_tgt == 0:
            return 0.0
        # Law of cosines → perpendicular deviation
        try:
            cos_a = ((ds_ugv**2 + ds_tgt**2 - du_tgt**2) / (2*ds_ugv*ds_tgt))
            cos_a = max(-1.0, min(1.0, cos_a))
            sin_a = math.sqrt(1 - cos_a**2)
            return abs(ds_ugv * sin_a)
        except (ValueError, ZeroDivisionError):
            return 0.0

    def update_ugv_state(self, ugv_lat: float, ugv_lon: float, ugv_heading: float,
                         start_lat: float, start_lon: float, confidence: float = 1.0) -> UGVState:
        now = time.time()
        dist_to_target = self._distance_m(ugv_lat, ugv_lon, self.target.lat, self.target.lon)
        deviation = self.calculate_deviation(ugv_lat, ugv_lon, start_lat, start_lon)
        state = UGVState(
            latitude=ugv_lat, longitude=ugv_lon, heading=ugv_heading,
            timestamp=now, distance_to_target=dist_to_target, deviation_from_path=deviation
        )
        self.ugv_state = state
        self.last_ugv_update = now
        self.max_deviation_seen = max(self.max_deviation_seen, deviation)
        return state

    def needs_correction(self) -> Tuple[bool, str]:
        if not self.ugv_state:
            return False, "no_state"
        if self.ugv_state.deviation_from_path > self.config.correction_threshold_meters:
            return True, f"deviation_{self.ugv_state.deviation_from_path:.2f}m"
        return False, "on_track"

    def correction_vector_body_enu(self, ugv_lat: float, ugv_lon: float, ugv_heading_deg: float) -> Tuple[float, float]:
        bearing_to_target = self._bearing_deg(ugv_lat, ugv_lon, self.target.lat, self.target.lon)
        distance_to_target = self._distance_m(ugv_lat, ugv_lon, self.target.lat, self.target.lon)
        relative_angle = bearing_to_target - ugv_heading_deg
        while relative_angle > 180: relative_angle -= 360
        while relative_angle < -180: relative_angle += 360
        x = distance_to_target * math.cos(math.radians(relative_angle))  # forward
        y = distance_to_target * math.sin(math.radians(relative_angle))  # right
        return x, y

    def generate_velocity_command(self, x: float, y: float, confidence: float = 1.0) -> Dict:
        v, w, dbg = self.velocity_controller.compute(x, y, conf=confidence)
        return {"type": "nav", "cmd": "vel", "v": round(v, 3), "w": round(w, 3),
                "timestamp": time.time(), "debug": dbg}

    def has_reached_target(self) -> bool:
        return bool(self.ugv_state and self.ugv_state.distance_to_target <= self.config.arrival_radius_meters)

    def is_mission_timeout(self) -> bool:
        return bool(self.mission_start_time and (time.time() - self.mission_start_time) > self.config.max_mission_time_seconds)

    def is_loss_of_sight(self) -> bool:
        return bool(self.last_ugv_update and (time.time() - self.last_ugv_update) > self.config.loss_of_sight_timeout_seconds)

    def print_status(self):
        if not self.ugv_state:
            print("[OVERWATCH] Waiting for UGV position…")
            return
        elapsed = time.time() - (self.mission_start_time or time.time())
        needs_corr, reason = self.needs_correction()
        status = "CORRECTING" if needs_corr else "ON TRACK"
        s = self.ugv_state
        print(f"[OVERWATCH] {status} | Dist {s.distance_to_target:.2f} m | Dev {s.deviation_from_path:.2f} m | "
              f"Head {s.heading:.0f}° | t+{elapsed:.0f}s | Reason: {reason}")

    def mission_summary(self) -> Dict:
        elapsed = time.time() - (self.mission_start_time or time.time())
        return {
            "mission_time_seconds": elapsed,
            "reached_target": self.has_reached_target(),
            "total_corrections": self.total_corrections,
            "max_deviation_meters": self.max_deviation_seen,
            "final_distance_to_target": (self.ugv_state.distance_to_target if self.ugv_state else None)
        }
