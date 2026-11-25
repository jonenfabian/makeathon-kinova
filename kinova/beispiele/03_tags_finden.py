"""Aus der Home-Pose alle AprilTags suchen, ihre Lage ausgeben und ein Bild mit Markierungen speichern."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
from kinova_team import Roboter

with Roboter() as r:
    r.home()
    tags = r.tags()
    for t in tags:
        print(f"Tag {t['id']}: x {t['x_mm']:.0f}  y {t['y_mm']:.0f}  z {t['z_mm']:.0f} mm, gedreht {t['gier_deg']:.0f}°")
    b = r.bild("farbe")
    for t in tags:
        ecken = np.array(t["ecken_px"], dtype=np.int32)
        cv2.polylines(b.bild, [ecken], True, (0, 200, 0), 3)
        cv2.putText(b.bild, str(t["id"]), tuple(int(v) for v in ecken[0]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.imwrite("tags.png", b.bild)
    print("Bild gespeichert: tags.png")
