"""Installierten Team-Einstieg ohne Arm oder Browser prüfen."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "kinova/simulation"))
import sim

db, world = sim.aufbauen(.06, zeitraffer=20)
try:
    tags = db.tag_erkennen({})["tags"]
    assert tags
    db.wuerfel_scannen({"bei_m": [.37, .15]})
    assert db.wuerfel_greifen({"bestaetigt": True})["gegriffen"]
    assert db.wuerfel_ablegen({"bestaetigt": True, "x_m": .28, "y_m": .07})["abgelegt"]
    print("Kinova: Tags, Greifen und Ablegen bestanden.")
finally:
    world.stop()
    db.arm.stop()
