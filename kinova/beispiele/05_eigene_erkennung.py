"""Ohne die fertigen Bausteine: Tag selbst im Farbbild finden und mit dem Tiefenbild in Basiskoordinaten umrechnen.

Kette: Pixel (u, v) + Abstand → Kamera-Frame → Basis über ``Bild.t_basis_kamera``. Farb- und Tiefenkamera sitzen an
verschiedenen Stellen; deshalb wird der Tag-Mittelpunkt erst über die Farbkamera in Basiskoordinaten gebracht und
dann ins Tiefenbild projiziert, um dort die Höhe nachzumessen.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
from kinova_team import Roboter

KANTE_M = 0.036  # schwarze Kante der Würfel-Tags 200–205

with Roboter() as r:
    r.home()
    farbe, tiefe = r.bild("farbe"), r.bild("tiefe")
    i = farbe.intrinsik
    k = np.array([[i["fx"], 0, i["cx"]], [0, i["fy"], i["cy"]], [0, 0, 1]])
    detektor = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    ecken, ids, _ = detektor.detectMarkers(cv2.cvtColor(farbe.bild, cv2.COLOR_BGR2GRAY))
    if ids is None:
        sys.exit("kein Tag im Bild")
    h = KANTE_M / 2
    objekt = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    for c, tag_id in zip(ecken, ids.flatten()):
        ok, rvec, tvec = cv2.solvePnP(objekt, c.reshape(4, 2), k, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        mitte_basis = farbe.t_basis_kamera @ np.append(tvec.ravel(), 1.0)
        x, y, z = (mitte_basis[:3] * 1000).round(1)
        uv = tiefe.basis_zu_pixel(x, y, z)
        hoehe = "außerhalb des Tiefenbilds"
        if uv is not None and 0 <= uv[0] < tiefe.bild.shape[1] and 0 <= uv[1] < tiefe.bild.shape[0]:
            mm = float(tiefe.bild[int(uv[1]), int(uv[0])])
            if mm > 0:
                hoehe = f"Tiefenbild: z {tiefe.pixel_zu_basis(uv[0], uv[1], mm)[2]:.0f} mm"
        print(f"Tag {tag_id}: Farbbild x {x:.0f} y {y:.0f} z {z:.0f} mm | {hoehe}")
