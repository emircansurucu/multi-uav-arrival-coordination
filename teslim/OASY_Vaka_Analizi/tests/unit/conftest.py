"""birim testleri için paket kaynak dizinini içe aktarma yoluna ekler"""
import sys
from pathlib import Path

PACKAGE_SRC = Path(__file__).resolve().parents[2] / "ros2_ws/src/oasy_uav_agent"  # paket kaynak dizini
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
