"""Farb- und Tiefenbild speichern und ansehen. Bewegt nichts.

Schreibt farbe.png, tiefe.png (Rohwerte in mm, 16 Bit, sieht fast schwarz aus) und tiefe_farbig.png (nah rot,
fern blau, schwarz = keine Messung). Dazu ein Beispiel, wie man einen Bildpunkt in Basiskoordinaten umrechnet.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinova_team import Roboter

r = Roboter()
print("Pose:", r.pose())
farbe, tiefe = r.bild("farbe"), r.bild("tiefe")
print("gespeichert:", farbe.speichern("farbe"), tiefe.speichern("tiefe"))

# Bildmitte des Tiefenbilds in Basiskoordinaten: Abstand aus dem Bild, Lage über die Kamerapose
h, b = tiefe.bild.shape
abstand_mm = float(tiefe.bild[h // 2, b // 2])
if abstand_mm > 0:
    print("Bildmitte liegt bei (x, y, z) mm:", tiefe.pixel_zu_basis(b / 2, h / 2, abstand_mm))
else:
    print("in der Bildmitte keine Tiefenmessung")
