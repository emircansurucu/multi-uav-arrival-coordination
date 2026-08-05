"""diğer araçlardan rüzgâr kaynağı seçimini sınar"""
from types import SimpleNamespace

import pytest

SECOND_NS = 1_000_000_000  # saniyedeki nanosaniye sayısı


@pytest.fixture
def peer_api():
    """ros çalışma alanı yoksa testi atlar"""
    messages = pytest.importorskip(
        "oasy_interfaces.msg", reason="ROS calisma alani kurulmamis"
    )
    from oasy_uav_agent.coordination.peer_manager import PeerManager

    def make_status(
        vehicle_id, wind_valid=True, wind_speed=8.0, wind_dir_deg=0.0, **fields
    ):
        """istenen alanlarla bir test araç durumu oluşturur"""
        status = messages.VehicleStatus()
        status.vehicle_id = vehicle_id
        status.wind_valid = wind_valid
        status.wind_speed = wind_speed
        status.wind_dir_deg = wind_dir_deg
        for name, value in fields.items():
            setattr(status, name, value)
        return status

    def make_peers(stale_after_s=2.0, lost_after_s=5.0):
        """verilen zaman aşımı sınırlarıyla araç yöneticisi oluşturur"""
        return PeerManager(
            own_vehicle_id=9, stale_after_s=stale_after_s, lost_after_s=lost_after_s
        )

    return SimpleNamespace(status=make_status, peers=make_peers)


def test_ruzgar_kaynagi_gelis_sirasindan_bagimsiz(peer_api):
    """mesaj sırasından bağımsız aynı aracın seçilmesini sınar"""
    once_ha1 = peer_api.peers()
    once_ha1.update(peer_api.status(1, wind_speed=8.0), SECOND_NS)
    once_ha1.update(peer_api.status(2, wind_speed=6.0), 2 * SECOND_NS)

    once_ha2 = peer_api.peers()
    once_ha2.update(peer_api.status(2, wind_speed=6.0), SECOND_NS)
    once_ha2.update(peer_api.status(1, wind_speed=8.0), 2 * SECOND_NS)

    now_ns = 2 * SECOND_NS
    assert once_ha1.settled_wind(now_ns) == once_ha2.settled_wind(now_ns)
    # en küçük araç kimliği seçilmeli
    assert once_ha1.settled_wind(now_ns) == (8.0, 0.0)


def test_oturmamis_ruzgar_kaynak_olarak_secilmez(peer_api):
    """geçersiz rüzgâr bildiren aracın kaynak seçilmemesini sınar"""
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_valid=False), SECOND_NS)
    peers.update(peer_api.status(2, wind_speed=6.0), SECOND_NS)

    assert peers.settled_wind(SECOND_NS) == (6.0, 0.0)


def test_kayip_peer_ruzgar_kaynagi_olamaz(peer_api):
    """kayıp sayılan aracın rüzgâr kaynağı olmamasını sınar"""
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_speed=8.0), SECOND_NS)
    peers.update(peer_api.status(2, wind_speed=6.0), 10 * SECOND_NS)

    # dokuz saniyelik ha1 mesajı kayıp sayılmalı
    assert peers.settled_wind(10 * SECOND_NS) == (6.0, 0.0)


def test_ruzgar_bilen_peer_yoksa_none(peer_api):
    """geçerli rüzgâr bildiren araç yokken none dönmesini sınar"""
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_valid=False), SECOND_NS)

    assert peers.settled_wind(SECOND_NS) is None



