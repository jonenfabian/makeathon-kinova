"""Home anfahren, ein Quadrat über dem Tisch abfahren, Greifer öffnen und schließen."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinova_team import Abgelehnt, Roboter

with Roboter() as r:
    r.home()
    start = r.pose()
    print("Start:", start)
    x, y = start["x_mm"], start["y_mm"]
    for dx, dy in ((50, 0), (50, 50), (0, 50), (0, 0)):
        r.fahre_zu(x + dx, y + dy, start["z_mm"])
        print("jetzt bei", r.pose()["x_mm"], r.pose()["y_mm"])
    r.greifer(60)
    r.schliessen()
    r.oeffnen()
    try:
        r.fahre_zu(z_mm=-50)  # unter der Tischplatte: der Server lehnt ab, der Arm bleibt stehen
    except Abgelehnt as fehler:
        print("abgelehnt, wie erwartet:", fehler)
    r.home()
