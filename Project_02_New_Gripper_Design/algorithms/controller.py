"""Hysteresis-aware grey-box force controller.

The controller is hardware agnostic: call ``update`` at a fixed rate and send
``torque_command`` to the motor.  Inverse maps are optional; a conservative
feedback-only fallback is used when a branch has no map.
"""
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import numpy as np


class Branch(str, Enum):
    RELEASE = "release"; LOADING = "loading"; HOLD = "hold"
    PEAK_LAG = "peak_lag"; UNLOADING = "unloading"; LOWER_TURN = "lower_turn"


@dataclass
class ControllerConfig:
    kp: float = 0.035
    ki: float = 0.008
    kd: float = 0.001
    torque_min: float = 0.0
    torque_max: float = 0.8
    hard_force_max: float = 25.0
    rate_deadband: float = 0.03
    turn_rate: float = 0.08
    turn_hold_s: float = 0.20
    integral_limit: float = 8.0
    learn_gain: float = 0.002
    learn_limit: float = 0.08


class HysteresisController:
    def __init__(self, maps=None, config=None):
        self.cfg = config or ControllerConfig(); self.maps = maps or {}
        self.branch = Branch.RELEASE; self.upper_turn = 0.0; self.lower_turn = 0.0
        self.integral = 0.0; self.prev_error = 0.0; self.prev_force = 0.0
        self.turn_elapsed = 0.0; self.residual = {}

    @classmethod
    def from_npz(cls, path, config=None):
        d = np.load(Path(path), allow_pickle=False)
        required = ("force_grid", "rate_grid", "torque_grid")
        if not all(k in d for k in required): raise ValueError("invalid inverse map")
        base = {k: np.asarray(d[k], dtype=float) for k in required}
        return cls({"loading": base}, config)

    def _lookup(self, branch, force, rate):
        m = self.maps.get(branch) or self.maps.get("loading")
        if not m: return 0.0
        fg, rg, tg = m["force_grid"], m["rate_grid"], m["torque_grid"]
        i = int(np.clip(np.searchsorted(fg, force), 1, len(fg)-1)); i -= force-fg[i-1] < fg[i]-force
        j = int(np.clip(np.searchsorted(rg, rate), 1, len(rg)-1)); j -= rate-rg[j-1] < rg[j]-rate
        return float(tg[i, j])

    def update(self, force_ref, force_rate_ref, force_measured, dt, enabled=True):
        c = self.cfg; f = float(force_measured); dt = max(float(dt), 1e-5)
        if not np.isfinite(f) or f < -1.0 or f > c.hard_force_max:
            self.branch = Branch.RELEASE; self.integral = 0.0
            return {"torque_command": 0.0, "branch": self.branch.value, "safety_limited": True}
        rate = (f - self.prev_force) / dt; self.prev_force = f
        if not enabled or force_ref <= 0:
            self.branch = Branch.RELEASE; self.integral = 0.0; return {"torque_command": 0.0, "branch": self.branch.value, "safety_limited": False}
        if force_rate_ref > c.rate_deadband: desired = Branch.LOADING
        elif force_rate_ref < -c.rate_deadband: desired = Branch.UNLOADING
        else: desired = Branch.HOLD
        if self.branch == Branch.LOADING and desired == Branch.UNLOADING:
            self.branch = Branch.PEAK_LAG; self.upper_turn = max(self.upper_turn, f); self.turn_elapsed = 0.0
        elif self.branch in (Branch.UNLOADING, Branch.LOWER_TURN) and desired == Branch.LOADING:
            self.branch = Branch.LOWER_TURN; self.lower_turn = f; self.turn_elapsed = 0.0
        elif self.branch in (Branch.PEAK_LAG, Branch.LOWER_TURN):
            self.turn_elapsed += dt
            if self.turn_elapsed >= c.turn_hold_s: self.branch = desired
        else: self.branch = desired
        if self.branch == Branch.LOADING: self.upper_turn = max(self.upper_turn, f)
        if self.branch == Branch.UNLOADING: self.lower_turn = min(self.lower_turn or f, f)
        e = float(force_ref) - f; self.integral = np.clip(self.integral + e*dt, -c.integral_limit, c.integral_limit)
        de = (e - self.prev_error) / dt; self.prev_error = e
        ff = self._lookup(self.branch.value, np.clip(force_ref, 0, c.hard_force_max), force_rate_ref)
        key = (self.branch.value, int(np.clip(force_ref, 0, 100)*10))
        correction = self.residual.get(key, 0.0); fb = c.kp*e + c.ki*self.integral + c.kd*de
        tau = float(np.clip(ff + fb + correction, c.torque_min, c.torque_max))
        if abs(e) < 0.5 and self.branch in (Branch.LOADING, Branch.UNLOADING, Branch.HOLD):
            self.residual[key] = float(np.clip(correction + c.learn_gain*e, -c.learn_limit, c.learn_limit))
        return {"torque_command": tau, "branch": self.branch.value, "tau_ff": ff, "tau_fb": fb,
                "tau_learning": correction, "force_rate": rate, "upper_turn": self.upper_turn,
                "lower_turn": self.lower_turn, "safety_limited": False}
