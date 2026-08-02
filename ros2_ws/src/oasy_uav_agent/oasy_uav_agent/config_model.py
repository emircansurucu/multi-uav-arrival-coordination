"""Arac konfigurasyonunun YAML'dan okunmasi ve dogrulanmasi."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

from .estimation.geodesy import LatLon

# Vaka dokumani rota noktalarinin kabul yaricapini en fazla 400 m ile
# sinirliyor; konfigurasyon bu siniri asamaz.
MAX_WP_ACCEPT_RADIUS_M = 400.0


@dataclass(frozen=True)
class VehicleConfig:
    """Tek bir hava aracinin calisma parametreleri."""

    vehicle_id: int
    vehicle_domain_id: int
    coordination_domain_id: int
    mavlink_address: str
    home: LatLon
    route: Tuple[LatLon, ...]
    cruise_alt_msl_m: float
    takeoff_alt_msl_m: float
    wp_accept_radius_m: float
    nominal_cruise_speed_mps: float
    min_airspeed_mps: float
    max_airspeed_mps: float
    airspeed_rate_limit_mps2: float
    timing_deadband_s: float
    s_maneuver_enabled: bool
    loiter_enabled: bool
    status_publish_hz: float
    peer_stale_after_s: float
    peer_lost_after_s: float

    @property
    def target(self) -> LatLon:
        """Ortak hedef, rotanin son noktasidir."""
        return self.route[-1]


def load_vehicle_config(path: Path) -> VehicleConfig:
    """YAML dosyasini okur ve zorunlu alanlari dogrular."""
    with open(path, "r", encoding="utf-8") as handle:
        raw: Dict[str, Any] = yaml.safe_load(handle)

    route = tuple(_to_latlon(point) for point in raw["route"])
    if len(route) < 2:
        raise ValueError(f"{path}: rota en az iki nokta icermeli")

    config = VehicleConfig(
        vehicle_id=int(raw["vehicle_id"]),
        vehicle_domain_id=int(raw["vehicle_domain_id"]),
        coordination_domain_id=int(raw["coordination_domain_id"]),
        mavlink_address=str(raw["mavlink_address"]),
        home=_to_latlon(raw["home"]),
        route=route,
        cruise_alt_msl_m=float(raw["cruise_alt_msl_m"]),
        takeoff_alt_msl_m=float(raw["takeoff_alt_msl_m"]),
        wp_accept_radius_m=float(raw["wp_accept_radius_m"]),
        nominal_cruise_speed_mps=float(raw["nominal_cruise_speed_mps"]),
        min_airspeed_mps=float(raw["min_airspeed_mps"]),
        max_airspeed_mps=float(raw["max_airspeed_mps"]),
        airspeed_rate_limit_mps2=float(raw["airspeed_rate_limit_mps2"]),
        timing_deadband_s=float(raw["timing_deadband_s"]),
        s_maneuver_enabled=bool(raw["s_maneuver_enabled"]),
        loiter_enabled=bool(raw["loiter_enabled"]),
        status_publish_hz=float(raw["status_publish_hz"]),
        peer_stale_after_s=float(raw["peer_stale_after_s"]),
        peer_lost_after_s=float(raw["peer_lost_after_s"]),
    )

    if not 1 <= config.vehicle_id <= 3:
        raise ValueError(f"{path}: vehicle_id 1-3 araliginda olmali")
    if config.vehicle_domain_id == config.coordination_domain_id:
        raise ValueError(f"{path}: arac ve koordinasyon domain'leri ayni olamaz")
    if config.peer_stale_after_s >= config.peer_lost_after_s:
        raise ValueError(f"{path}: peer_stale_after_s, peer_lost_after_s'ten kucuk olmali")
    if not 0.0 < config.wp_accept_radius_m <= MAX_WP_ACCEPT_RADIUS_M:
        raise ValueError(
            f"{path}: wp_accept_radius_m 0 ile {MAX_WP_ACCEPT_RADIUS_M:.0f} m arasinda olmali"
        )
    if config.takeoff_alt_msl_m > config.cruise_alt_msl_m:
        raise ValueError(f"{path}: takeoff_alt_msl_m, cruise_alt_msl_m'yi asamaz")
    if config.nominal_cruise_speed_mps <= 0.0:
        raise ValueError(f"{path}: nominal_cruise_speed_mps pozitif olmali")
    if not config.min_airspeed_mps < config.max_airspeed_mps:
        raise ValueError(f"{path}: min_airspeed_mps, max_airspeed_mps'ten kucuk olmali")
    if not config.min_airspeed_mps <= config.nominal_cruise_speed_mps <= config.max_airspeed_mps:
        raise ValueError(f"{path}: nominal_cruise_speed_mps hiz sinirlari disinda")

    return config


def _to_latlon(raw: Dict[str, Any]) -> LatLon:
    return LatLon(float(raw["lat"]), float(raw["lon"]))
