"""Eigener Turm mit den Grundbausteinen, ohne r.stapeln(): Würfel anfahren und vermessen, greifen, auf den Turm legen.
Danach von oben wieder abbauen, immer den obersten Würfel.

In der Simulation legt das Programm die Würfel vorher selbst hin. Am echten Arm TURM_MM und WUERFEL_MM an die Lage
eurer Würfel anpassen; auf etwa 5 cm genau reicht, das Vermessen findet die genaue Mitte.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinova_team import Abgelehnt, Roboter

TURM_MM = (420, -10)  # hier steht der unterste Würfel
WUERFEL_MM = [(330, 120), (330, -120)]  # diese kommen obendrauf
ABLAGE_MM = [(330, 120), (330, -120)]  # Plätze beim Abbauen, frei und mindestens 100 mm auseinander

with Roboter() as r:
    if r.zustand().get("simulation"):
        r.sim_szene(wuerfel=[{"id": 200 + i, "x_mm": x, "y_mm": y} for i, (x, y) in enumerate([TURM_MM, *WUERFEL_MM])])
    try:
        for x, y in WUERFEL_MM:
            turm = r.wuerfel_vermessen(*TURM_MM, hinfahren=True)  # Oberseite und Drehung des Turms
            print(f"Turm: {turm['stapel']} Würfel, Oberseite {turm['oben_z_mm']} mm")
            w = r.wuerfel_vermessen(x, y, hinfahren=True)
            if not w["greifen_moeglich"]:
                sys.exit(f"Würfel bei ({x}, {y}) lässt sich nicht greifen: {w['pruefung']}")
            r.greifen()
            r.ablegen(turm["x_mm"], turm["y_mm"], gier_deg=turm["gier_deg"], unten_z_mm=turm["oben_z_mm"])
        turm = r.wuerfel_vermessen(*TURM_MM, hinfahren=True)
        print(f"Turm fertig: {turm['stapel']} Würfel übereinander")

        for x, y in ABLAGE_MM:  # abbauen
            turm = r.wuerfel_vermessen(*TURM_MM, hinfahren=True)
            if turm["stapel"] < 2:
                break
            r.greifen()  # greift den obersten Würfel
            r.ablegen(x, y)  # ohne unten_z_mm: auf den Tisch
        r.home()
    except Abgelehnt as fehler:
        print("abgebrochen:", fehler)
        if r.haelt_wuerfel():
            print("Ein Würfel hängt im Greifer: r.anmelden(), dann r.ablegen(x, y) oder r.abstellen().")
