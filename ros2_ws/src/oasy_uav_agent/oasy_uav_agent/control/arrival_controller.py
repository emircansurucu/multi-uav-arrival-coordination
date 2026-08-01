"""Varis zamanini seyir hiziyla duzenleyen kontrolcu.

Kontrol yasasi: yeni komut = mevcut komut * (ETA / kalan sure).

Kalan mesafeyi kalan sureye bolmek yeterli degildir; o hesap yer hizi
uretir ve komut edilen hava hizi ile olusan yer hizi arasindaki farki
(donus kayiplari, ruzgar, irtifada TAS/EAS farki) gormez. ETA zaten mevcut
ilerleme hizindan turedigi icin ETA/kalan sure orani dogrudan gereken
olceklemeyi verir ve bu farklari kendiliginden kapsar.

Uc koruma zorunlu:
  - deadband: kucuk hatalarda mudahale edilmez, yoksa kontrolcu surekli
    hiz degistirip salinim uretir.
  - saturation: komut otopilotun guvenli hava hizi araligi disina cikamaz.
  - rate limit: ani hiz sicramalari engellenir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

NANOSECONDS_PER_SECOND = 1_000_000_000
# Varisa cok az kaldiginda gerekli hiz sonsuza gider; bu esigin altinda
# komut degistirilmez.
MIN_REMAINING_TIME_S = 15.0
# Komut, en son gonderilen degerden bu kadar uzaklastiginda yeniden
# gonderilir. Karsilastirma bir onceki tick'e gore yapilirsa rate limit
# adimi (0.5 m/s^2 * 0.05 s = 0.025 m/s) bu esigin altinda kaldigi icin
# hicbir komut gonderilmez.
MIN_COMMAND_STEP_MPS = 0.1


class ControlAction(Enum):
    HOLD = "sabit"
    SPEED_UP = "hizlan"
    SLOW_DOWN = "yavasla"


@dataclass(frozen=True)
class SpeedCommand:
    """Tek bir kontrol adiminin sonucu ve hangi sinirlayicilardan gectigi."""

    action: ControlAction
    airspeed_mps: float
    timing_error_s: float
    required_speed_mps: float
    deadband_active: bool
    saturated: bool
    rate_limited: bool
    changed: bool


class ArrivalController:
    """Taahhut edilen varis anina gore seyir hizini duzenler."""

    def __init__(
        self,
        nominal_airspeed_mps: float,
        min_airspeed_mps: float,
        max_airspeed_mps: float,
        rate_limit_mps_per_s: float,
        deadband_s: float,
    ) -> None:
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
        return self._commanded_mps

    def update(
        self,
        eta_s: float,
        remaining_distance_m: float,
        planned_arrival_monotonic_ns: int,
        now_monotonic_ns: int,
        dt_s: float,
    ) -> Optional[SpeedCommand]:
        """Yeni hiz komutunu hesaplar. Kontrol uygulanamiyorsa None doner."""
        if planned_arrival_monotonic_ns <= 0:
            return None

        remaining_time_s = (
            planned_arrival_monotonic_ns - now_monotonic_ns
        ) / NANOSECONDS_PER_SECOND
        if remaining_time_s < MIN_REMAINING_TIME_S:
            return None

        timing_error_s = eta_s - remaining_time_s
        if abs(timing_error_s) <= self._deadband_s:
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

        if eta_s <= 0.0:
            return None

        required_mps = self._commanded_mps * (eta_s / remaining_time_s)
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

        return SpeedCommand(
            action=ControlAction.SPEED_UP if timing_error_s > 0 else ControlAction.SLOW_DOWN,
            airspeed_mps=self._commanded_mps,
            timing_error_s=timing_error_s,
            required_speed_mps=required_mps,
            deadband_active=False,
            saturated=saturated,
            rate_limited=rate_limited,
            changed=changed,
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
