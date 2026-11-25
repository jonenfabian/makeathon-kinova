"""Würfel mit Tag 201 suchen, greifen und an einen festen Platz legen (Aufgabe 1 mit den fertigen Bausteinen)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinova_team import Abgelehnt, Roboter

TAG = 201
ZIEL_MM = (280.0, 70.0)  # Ablageplatz (Mitte des Würfels); vorher prüfen, dass er frei ist

with Roboter() as r:
    r.home()
    gefunden = r.tags(id=TAG)
    if not gefunden:
        sys.exit(f"Tag {TAG} ist nicht im Bild. Würfel hinlegen oder Kamera anders stellen.")
    t = gefunden[0]
    print(f"Tag {TAG} bei x {t['x_mm']:.0f}, y {t['y_mm']:.0f} mm")
    w = r.wuerfel_vermessen(bei_x_mm=t["x_mm"], bei_y_mm=t["y_mm"])
    print("Würfel:", w)
    if not w["greifen_moeglich"]:
        sys.exit("Greifen nicht erlaubt: " + "; ".join(w["pruefung"]))
    try:
        r.greifen()
        r.ablegen(*ZIEL_MM, gier_deg=0)
    except Abgelehnt as fehler:
        print("abgebrochen:", fehler)
    r.home()
