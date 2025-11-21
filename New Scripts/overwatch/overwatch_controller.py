# New Scripts/overwatch/overwatch_controller.py
"""
Continuous overwatch controller:
- Input: relative target (x,y) in BODY_ENU from UAV -> UGV (meters)
- Output: differential-drive style command (v, w)
    v: forward (+) / reverse (-) linear velocity suggestion
    w: turn right (+) / left (-) angular rate suggestion

Includes:
- deadbands to avoid jitter near zero
- proportional gains with saturation
- simple exponential smoothing on outputs
- confidence gating (drop to zero below threshold)
- loss-of-sight timeout safety hold
"""

from dataclasses import dataclass, field
import time

@dataclass
class OverwatchControllerConfig:
    # Geometry thresholds (meters)
    forward_deadband: float = 0.10
    turn_deadband: float = 0.10

    # Controller gains
    k_v: float = 0.30     # m/s per meter of x
    k_w: float = 0.80     # rad/s per meter of y

    # Saturation limits
    v_max: float = 0.40   # m/s
    w_max: float = 1.20   # rad/s

    # Smoothing (exponential, 0=off, 1=very sluggish)
    alpha_v: float = 0.25
    alpha_w: float = 0.25

    # Confidence gating & safety
    conf_min: float = 0.55       # below this -> hold
    los_timeout_s: float = 1.0   # if no update for this long -> hold

@dataclass
class OverwatchControllerState:
    v_smoothed: float = 0.0
    w_smoothed: float = 0.0
    last_update_ts: float = field(default_factory=lambda: 0.0)

class OverwatchController:
    def __init__(self, cfg: OverwatchControllerConfig | None = None):
        self.cfg = cfg or OverwatchControllerConfig()
        self.state = OverwatchControllerState()

    def _apply_deadband(self, value: float, db: float) -> float:
        if abs(value) < db:
            return 0.0
        return value

    def _clip(self, value: float, lim: float) -> float:
        if value > lim:  return lim
        if value < -lim: return -lim
        return value

    def _smooth(self, prev: float, new: float, alpha: float) -> float:
        return (1.0 - alpha) * new + alpha * prev

    def compute(self, x: float, y: float, *, conf: float | None = None, now_s: float | None = None):
        """
        x: forward (+ ahead, - behind), meters
        y: right (+ to right, - to left), meters
        conf: optional confidence in [0..1]
        Returns: (v, w, debug)
        """
        cfg = self.cfg
        st  = self.state
        tnow = now_s if now_s is not None else time.time()

        # Safety: confidence gating
        if conf is not None and conf < cfg.conf_min:
            v_raw = 0.0
            w_raw = 0.0
            reason = "hold_low_conf"
        else:
            # Deadbanded errors
            x_db = self._apply_deadband(x, cfg.forward_deadband)
            y_db = self._apply_deadband(y, cfg.turn_deadband)

            # Proportional control
            v_raw = self._clip(cfg.k_v * x_db, cfg.v_max)
            w_raw = self._clip(cfg.k_w * y_db, cfg.w_max)
            reason = "normal"

        # Loss-of-sight timeout: if too long since last good update, hold
        if st.last_update_ts and (tnow - st.last_update_ts) > cfg.los_timeout_s:
            v_raw = 0.0
            w_raw = 0.0
            reason = "hold_timeout"

        # Smooth outputs
        v = self._smooth(st.v_smoothed, v_raw, cfg.alpha_v)
        w = self._smooth(st.w_smoothed, w_raw, cfg.alpha_w)

        # Update state
        st.v_smoothed = v
        st.w_smoothed = w
        st.last_update_ts = tnow

        dbg = {
            "reason": reason,
            "inputs": {"x": x, "y": y, "conf": conf},
            "raw": {"v_raw": v_raw, "w_raw": w_raw},
            "smoothed": {"v": v, "w": w},
            "ts": tnow
        }
        return v, w, dbg
