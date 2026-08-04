"""Peer ruzgar kaynagi seciminin deterministik oldugunun testleri.

VehicleStatus mesajina ihtiyac duyulur. Import modul seviyesinde yapilamaz:
launch_testing eklentisi toplama sirasinda her modulu import ettigi icin
modul seviyesindeki bir atlama tum oturumun toplanmasini durdurur.
"""
from types import SimpleNamespace

import pytest

SECOND_NS = 1_000_000_000


@pytest.fixture
def peer_api():
    """Calisma alani kurulu degilse testi atlar, kuruluysa fabrikalari verir."""
    messages = pytest.importorskip(
        "oasy_interfaces.msg", reason="ROS calisma alani kurulmamis"
    )
    from oasy_uav_agent.coordination.peer_manager import PeerManager

    def make_status(
        vehicle_id, wind_valid=True, wind_speed=8.0, wind_dir_deg=0.0, **fields
    ):
        status = messages.VehicleStatus()
        status.vehicle_id = vehicle_id
        status.wind_valid = wind_valid
        status.wind_speed = wind_speed
        status.wind_dir_deg = wind_dir_deg
        for name, value in fields.items():
            setattr(status, name, value)
        return status

    def make_peers(stale_after_s=2.0, lost_after_s=5.0):
        return PeerManager(
            own_vehicle_id=9, stale_after_s=stale_after_s, lost_after_s=lost_after_s
        )

    return SimpleNamespace(status=make_status, peers=make_peers)


def test_ruzgar_kaynagi_gelis_sirasindan_bagimsiz(peer_api):
    """Mesajlar hangi sirayla gelirse gelsin ayni peer secilmeli.

    En taze mesaji secmek, iki arac ayni anda havadayken agentin saniyede
    birkac kez tahminler arasinda ziplamasina yol aciyordu.
    """
    once_ha1 = peer_api.peers()
    once_ha1.update(peer_api.status(1, wind_speed=8.0), SECOND_NS)
    once_ha1.update(peer_api.status(2, wind_speed=6.0), 2 * SECOND_NS)

    once_ha2 = peer_api.peers()
    once_ha2.update(peer_api.status(2, wind_speed=6.0), SECOND_NS)
    once_ha2.update(peer_api.status(1, wind_speed=8.0), 2 * SECOND_NS)

    now_ns = 2 * SECOND_NS
    assert once_ha1.settled_wind(now_ns) == once_ha2.settled_wind(now_ns)
    # Sabit sira en kucuk id demektir.
    assert once_ha1.settled_wind(now_ns) == (8.0, 0.0)


def test_oturmamis_ruzgar_kaynak_olarak_secilmez(peer_api):
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_valid=False), SECOND_NS)
    peers.update(peer_api.status(2, wind_speed=6.0), SECOND_NS)

    assert peers.settled_wind(SECOND_NS) == (6.0, 0.0)


def test_kayip_peer_ruzgar_kaynagi_olamaz(peer_api):
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_speed=8.0), SECOND_NS)
    peers.update(peer_api.status(2, wind_speed=6.0), 10 * SECOND_NS)

    # HA-1'in son mesajinin uzerinden 9 s gecti, kayip sayilir.
    assert peers.settled_wind(10 * SECOND_NS) == (6.0, 0.0)


def test_ruzgar_bilen_peer_yoksa_none(peer_api):
    peers = peer_api.peers()
    peers.update(peer_api.status(1, wind_valid=False), SECOND_NS)

    assert peers.settled_wind(SECOND_NS) is None





