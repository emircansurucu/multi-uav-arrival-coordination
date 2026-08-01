"""Birim testleri ROS calisma alani kurulmadan da calisabilsin diye
paket kaynak dizinini import yoluna ekler."""
import sys
from pathlib import Path

PACKAGE_SRC = Path(__file__).resolve().parents[2] / "ros2_ws/src/oasy_uav_agent"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
