"""varış zamanını seyir hızıyla düzenler"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

NANOSECONDS_PER_SECOND = 1_000_000_000  # saniyedeki nanosaniye sayısı
MIN_REMAINING_TIME_S = 15.0  # oran tabanlı kontrolün en kısa kalan süresi
MIN_COMMAND_STEP_MPS = 0.1  # yeni hız komutu için gereken en küçük fark


class ControlAction(Enum):
    HOLD = "sabit"
    SPEED_UP = "hizlan"
    SLOW_DOWN = "yavasla"


@dataclass(frozen=True)
class SpeedCommand:
    """tek bir hız kontrolü sonucunu tutar"""

    action: ControlAction
    airspeed_mps: float
    timing_error_s: float
    required_speed_mps: float
    deadband_active: bool
    saturated: bool
    rate_limited: bool
    changed: bool


class ArrivalController:
    """taahhüt edilen varış anına göre seyir hızını düzenler"""

    def __init__(
        self,
        nominal_airspeed_mps: float,
        min_airspeed_mps: float,
        max_airspeed_mps: float,
        rate_limit_mps_per_s: float,
        deadband_s: float,
    ) -> None:
        """hız sınırlarını ve kontrolün başlangıç değerlerini hazırlar"""
        if not min_airspeed_mps < max_airspeed_mps:
            raise ValueError("min_airspeed_mps, max_airspeed_mps'ten kucuk olmali")
        self._min_airspeed_mps = min_airspeed_mps
        self._max_airspeed_mps = max_airspeed_mps
        self._rate_limit_mps_per_s = rate_limit_mps_per_s
        self._deadband_s = deadband_s
        self._commanded_mps = _clamp(nominal_airspeed_mps, min_airspeed_mps, max_airspeed_mps)
        self._last_sent_mps = self._commanded_mps

    @property
    def commanded_airspeed_mps(self) -> float:
        """son hesaplanan hedef hava hızını döner"""
        return self._commanded_mps

    def update(
        self,
        eta_s: float,
        remaining_distance_m: float,
        planned_arrival_monotonic_ns: int,
        now_monotonic_ns: int,
        dt_s: float,
        forced_airspeed_mps: Optional[float] = None,
    ) -> Optional[SpeedCommand]:
        """yeni hız komutunu hesaplar uygulanamıyorsa none döndürür"""
        if planned_arrival_monotonic_ns <= 0:
            return None

        remaining_time_s = (
            planned_arrival_monotonic_ns - now_monotonic_ns
        ) / NANOSECONDS_PER_SECOND
        # son saniyelerde yalnızca zorlanmış hız komutu uygulanır
        if remaining_time_s < MIN_REMAINING_TIME_S and forced_airspeed_mps is None:
            return None

        timing_error_s = eta_s - remaining_time_s
        if forced_airspeed_mps is None and abs(timing_error_s) <= self._deadband_s:
            return SpeedCommand(
                action=ControlAction.HOLD,
                airspeed_mps=self._commanded_mps,
                timing_error_s=timing_error_s,
                required_speed_mps=self._commanded_mps,
                deadband_active=True,
                saturated=False,
                rate_limited=False,
                changed=False,
            )

        if eta_s <= 0.0 and forced_airspeed_mps is None:
            return None

        required_mps = (
            forced_airspeed_mps
            if forced_airspeed_mps is not None
            else self._commanded_mps * (eta_s / remaining_time_s)
        )
        target_mps = _clamp(required_mps, self._min_airspeed_mps, self._max_airspeed_mps)
        saturated = not math.isclose(target_mps, required_mps, rel_tol=1e-9)

        delta_mps = target_mps - self._commanded_mps
        max_delta_mps = self._rate_limit_mps_per_s * max(dt_s, 0.0)
        rate_limited = abs(delta_mps) > max_delta_mps
        if rate_limited:
            delta_mps = math.copysign(max_delta_mps, delta_mps)

        self._commanded_mps = _clamp(
            self._commanded_mps + delta_mps, self._min_airspeed_mps, self._max_airspeed_mps
        )
        changed = abs(self._commanded_mps - self._last_sent_mps) >= MIN_COMMAND_STEP_MPS
        if changed:
            self._last_sent_mps = self._commanded_mps

        if delta_mps > 0.0:
            action = ControlAction.SPEED_UP
        elif delta_mps < 0.0:
            action = ControlAction.SLOW_DOWN
        else:
            action = ControlAction.HOLD

        return SpeedCommand(
            action=action,
            airspeed_mps=self._commanded_mps,
            timing_error_s=timing_error_s,
            required_speed_mps=required_mps,
            deadband_active=False,
            saturated=saturated,
            rate_limited=rate_limited,
            changed=changed,
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    """değeri verilen alt ve üst sınırlar içinde tutar"""
    return max(lower, min(upper, value))
