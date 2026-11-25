"""Verbindung prüfen: Zustand, Pose und Gelenke lesen. Bewegt nichts."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinova_team import Roboter

r = Roboter()
print("Server:", r.basis_url)
z = r.zustand()
print("Armzustand:", z["armzustand"], "| Simulation" if z.get("simulation") else "| echter Arm")
print("Pose (mm, Grad):", r.pose())
print("Gelenke (Grad):", r.gelenke())
print("Greiferweite:", r.greifer_weite_mm(), "mm")
netz = z["netz"]
print("Fahren freigegeben:", netz["freigabe"], "| Fahrrecht hat:", (netz["fahrer"] or {}).get("team", "niemand"))
