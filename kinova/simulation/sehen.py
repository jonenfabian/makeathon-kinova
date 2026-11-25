"""Bildauswertung: AprilTags, Kamerakette, Tiefenbild → Punktwolke, Würfel finden, Greifpose.

Kette Kamera → Basis: ``T_basis_kamera = T_basis_werkzeug(gemessen) · W⁻¹ · T_ee_kamera(Nennmaß URDF)``.
Kamera-Frames sind optische Frames (z nach vorn, x rechts, y unten im Bild).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import kinematik as kin
import numpy as np

TAG_KANTE_M = 0.0395  # schwarze Kante des Test-Tags (Fabian, 9.11.2026)
# Würfel-Sticker IDs 200–205: schwarze Kante 36 mm (kinova/README.md, „Würfel und Marker“); alle anderen IDs 39,5 mm
TAG_KANTEN_M = {i: 0.036 for i in range(200, 206)}


# --- Kamera -------------------------------------------------------------------------------------------------------


def kameramatrix(intr: dict) -> tuple[np.ndarray, np.ndarray]:
    k = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], dtype=float)
    return k, np.asarray(intr.get("verzeichnung", [0, 0, 0, 0, 0]), dtype=float)


def basis_kamera(t_basis_werkzeug: np.ndarray, werkzeug: np.ndarray, t_ee_kamera: np.ndarray) -> np.ndarray:
    return t_basis_werkzeug @ kin.invers(werkzeug) @ t_ee_kamera


# --- AprilTags ----------------------------------------------------------------------------------------------------


@dataclass
class Tag:
    id: int
    ecken_px: np.ndarray  # 4×2
    t_kamera_tag: np.ndarray  # 4×4
    reprojektion_px: float

    def als_dict(self, t_basis_kamera: np.ndarray | None = None) -> dict:
        d = {"id": self.id, "ecken_px": self.ecken_px.round(1).tolist(),
             "abstand_m": float(np.linalg.norm(self.t_kamera_tag[:3, 3])), "reprojektion_px": self.reprojektion_px,
             "kamera": kin.Pose.aus_matrix(self.t_kamera_tag).als_dict()}
        if t_basis_kamera is not None:
            t = t_basis_kamera @ self.t_kamera_tag
            d["basis"] = kin.Pose.aus_matrix(t).als_dict()
            d["basis_matrix"] = t.tolist()
        return d


def tags_erkennen(bild_bgr: np.ndarray, intr: dict, kante_m: float | None = None) -> list[Tag]:
    """AprilTag 36h11 mit cv2.aruco (wie pi/followme/detect.py), Lage per solvePnP IPPE_SQUARE.
    Tag-Frame: Ursprung in der Mitte, x nach rechts, y nach oben im Tag, z aus der Ebene heraus.
    kante_m: schwarze Kante für alle Tags; None = je ID aus ``TAG_KANTEN_M``, sonst ``TAG_KANTE_M``."""
    import cv2

    k, dist = kameramatrix(intr)
    aruco = cv2.aruco
    detektor = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11),
                                   aruco.DetectorParameters())
    ecken, ids, _ = detektor.detectMarkers(cv2.cvtColor(bild_bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    tags = []
    for c, i in zip(ecken, ids.flatten()):
        h = (kante_m if kante_m is not None else TAG_KANTEN_M.get(int(i), TAG_KANTE_M)) / 2
        objekt = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)  # Reihenfolge wie aruco
        bildpunkte = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(objekt, bildpunkte, k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        r, _ = cv2.Rodrigues(rvec)
        proj, _ = cv2.projectPoints(objekt, rvec, tvec, k, dist)
        fehler = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - bildpunkte) ** 2, axis=1))))
        tags.append(Tag(int(i), bildpunkte, kin.transform(r, tvec.ravel()), fehler))
    return tags


def senkrecht_ausgerichtet(r: np.ndarray) -> np.ndarray:
    """Gleiche Drehlage um die Senkrechte, aber z exakt entlang Basis-z (Neigung entfernt)."""
    x = r[:, 0].copy()
    x[2] = 0.0
    if np.linalg.norm(x) < 1e-6:  # x zeigt senkrecht: y verwenden
        x = np.cross(r[:, 1], (0.0, 0.0, 1.0))
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def ziel_ueber_tag(t_basis_tag: np.ndarray, t_basis_werkzeug_jetzt: np.ndarray, werkzeug: np.ndarray,
                   abstand_spitze_m: float, greifer_position: float = 1.0, senkrecht: bool = True) -> np.ndarray:
    """Werkzeugpose, bei der die Mitte der Fingerspitzen ``abstand_spitze_m`` über dem Tag-Ursprung steht und Werkzeug-z
    in −z des Tags zeigt. Von den vier Drehlagen um die Tag-Normale wird die nächstgelegene gewählt.
    senkrecht=True (Standard, Fabian 9.11.2026): Tag-Normale durch Basis-z ersetzen. Die Farbkamera schätzt die
    Neigung eines 39,5-mm-Tags mit Kinovas Vorgabe-Intrinsik um ≈ 10° falsch, Position und Höhe stimmen."""
    r_tag = senkrecht_ausgerichtet(t_basis_tag[:3, :3]) if senkrecht else t_basis_tag[:3, :3]
    kandidaten = [r_tag @ kin.rot_z(math.radians(a)) @ kin.rot_x(math.pi) for a in (0, 90, 180, 270)]
    r_ziel = min(kandidaten, key=lambda r: kin.winkel_abstand_deg(r, t_basis_werkzeug_jetzt[:3, :3]))
    p_spitze = t_basis_tag[:3, 3] + abstand_spitze_m * r_tag[:, 2]
    return kin.transform(r_ziel, p_spitze - r_ziel @ kin.fingermitte_im_werkzeug(werkzeug, greifer_position))


# --- Tiefenbild ---------------------------------------------------------------------------------------------------


def punktwolke(tiefe_mm: np.ndarray, intr: dict, schritt: int = 2, max_m: float = 1.5) -> np.ndarray:
    """Tiefenbild (uint16 mm) → N×3 Punkte im optischen Frame der Tiefenkamera (m)."""
    v, u = np.mgrid[0:tiefe_mm.shape[0]:schritt, 0:tiefe_mm.shape[1]:schritt]
    z = tiefe_mm[::schritt, ::schritt].astype(np.float64) / 1000.0
    gueltig = (z > 0.05) & (z < max_m)
    u, v, z = u[gueltig], v[gueltig], z[gueltig]
    x = (u - intr["cx"]) * z / intr["fx"]
    y = (v - intr["cy"]) * z / intr["fy"]
    return np.column_stack([x, y, z])


def in_basis(punkte: np.ndarray, t_basis_kamera: np.ndarray) -> np.ndarray:
    return punkte @ t_basis_kamera[:3, :3].T + t_basis_kamera[:3, 3]


@dataclass
class Wuerfel:
    mitte_m: np.ndarray  # Mittelpunkt (x, y, z) in der Basis
    kanten_m: tuple[float, float]  # Kanten der Oberseite, längere zuerst
    hoehe_m: float
    gier_deg: float  # Richtung der ersten Kante in der Tischebene
    tisch_z_m: float
    punkte_oben: int
    punkte_gesamt: int

    def als_dict(self) -> dict:
        return {"mitte_m": [round(float(v), 4) for v in self.mitte_m],
                "kanten_mm": [round(k * 1000, 1) for k in self.kanten_m], "hoehe_mm": round(self.hoehe_m * 1000, 1),
                "gier_deg": round(self.gier_deg, 1), "tisch_z_mm": round(self.tisch_z_m * 1000, 1),
                "oben_z_mm": round((self.tisch_z_m + self.hoehe_m) * 1000, 1),
                "punkte_oben": self.punkte_oben, "punkte_gesamt": self.punkte_gesamt}


def tisch_hoehe(punkte_basis: np.ndarray, bereich_m: float = 0.04) -> float:
    """Häufigste Höhe (Histogramm 2 mm) der Punkte nahe z = 0 → Tischplatte."""
    z = punkte_basis[:, 2]
    z = z[np.abs(z) < bereich_m]
    if z.size < 50:
        raise ValueError("zu wenige Tischpunkte im Tiefenbild")
    hist, kanten = np.histogram(z, bins=np.arange(-bereich_m, bereich_m + 0.002, 0.002))
    i = int(np.argmax(hist))
    nah = z[(z >= kanten[i] - 0.002) & (z <= kanten[i + 1] + 0.002)]
    return float(np.median(nah))


def objekte_finden(punkte_basis: np.ndarray, roi=((-0.70, 0.70), (-0.70, 0.70)), min_hoehe_m: float = 0.04,
                   max_hoehe_m: float = 0.15, zelle_m: float = 0.005) -> list[Wuerfel]:
    """Alle zusammenhängenden Objekte zwischen ``min_hoehe_m`` und ``max_hoehe_m`` über dem Tisch im ``roi``,
    jeweils mit Oberseite (minAreaRect), Höhe und Gierwinkel; größte zuerst. Tag-Klotz (18 mm) und eigene
    Finger/Arm fallen durch die Höhengrenzen heraus."""
    import cv2

    (x0, x1), (y0, y1) = roi
    p = punkte_basis[(punkte_basis[:, 0] > x0) & (punkte_basis[:, 0] < x1) &
                     (punkte_basis[:, 1] > y0) & (punkte_basis[:, 1] < y1)]
    tisch = tisch_hoehe(p)
    hoch = p[(p[:, 2] > tisch + min_hoehe_m) & (p[:, 2] < tisch + max_hoehe_m)]
    if len(hoch) < 30:
        return []
    ix = ((hoch[:, 0] - x0) / zelle_m).astype(int)
    iy = ((hoch[:, 1] - y0) / zelle_m).astype(int)
    raster = np.zeros((int((y1 - y0) / zelle_m) + 1, int((x1 - x0) / zelle_m) + 1), np.uint8)
    raster[iy, ix] = 255
    raster = cv2.morphologyEx(raster, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    _, etiketten, statistik, _ = cv2.connectedComponentsWithStats(raster)
    objekte = []
    for k in 1 + np.argsort(-statistik[1:, cv2.CC_STAT_AREA]):
        obj = hoch[etiketten[iy, ix] == k]
        if len(obj) < 30:
            continue
        oben_z = float(np.percentile(obj[:, 2], 95))
        oben = obj[obj[:, 2] > oben_z - 0.008]
        (cx, cy), (a, b), winkel = cv2.minAreaRect(oben[:, :2].astype(np.float32))
        kanten = (float(a), float(b))
        if kanten[0] < kanten[1]:
            kanten = (kanten[1], kanten[0])
            winkel += 90.0
        hoehe = oben_z - tisch
        objekte.append(Wuerfel(np.array([cx, cy, tisch + hoehe / 2]), kanten, hoehe, ((winkel + 90) % 180) - 90,
                               tisch, len(oben), len(obj)))
    return objekte


STAPEL_MAX = 3  # so viele Würfel übereinander erkennt der Scan noch
STAPEL_TOLERANZ_M = 0.008  # gemessene Höhe gegen Vielfaches der Nennkante (am Arm 59–63 mm bei 60 mm)


def _text(w: Wuerfel) -> str:
    return (f"({w.mitte_m[0] * 1000:.0f}, {w.mitte_m[1] * 1000:.0f}) mm: {w.kanten_m[0] * 1000:.0f}×"
            f"{w.kanten_m[1] * 1000:.0f}×{w.hoehe_m * 1000:.0f} mm")


def wuerfel_masse(kante_m: float) -> dict:
    """Suchbereiche für Würfel der Nennkante ``kante_m``: Kanten ±15 mm, Höhe −20 mm bis drei Würfel übereinander
    +20 mm (Turm: gegriffen wird der oberste). Für 70 mm (Ryobi-Würfel) ergibt das die am Arm am 9.11.2026 erprobten
    55–85 mm Kante."""
    return {"kante_m": (kante_m - 0.015, kante_m + 0.015), "hoehe_m": (kante_m - 0.020, STAPEL_MAX * kante_m + 0.020)}


def wuerfel_finden(punkte_basis: np.ndarray, roi=((-0.70, 0.70), (-0.70, 0.70)), min_hoehe_m: float = 0.04,
                   max_hoehe_m: float = 0.15, kante_m=(0.055, 0.085), hoehe_m=(0.05, 0.09),
                   zelle_m: float = 0.005, bei_m=None, bei_radius_m: float = 0.05) -> Wuerfel:
    """Würfel = Objekt, dessen Oberseite und Höhe zu ``kante_m``/``hoehe_m`` passen. Am Arm (9.11.2026) lag sonst ein
    größeres flaches Objekt am Tischrand vor dem Würfel. Passt keins, nennt die Fehlermeldung die Kandidaten.
    bei_m (x, y): von den passenden Würfeln den, dessen Mitte am nächsten liegt (höchstens ``bei_radius_m`` entfernt),
    z. B. an der Stelle eines erkannten AprilTags."""
    max_hoehe_m = max(max_hoehe_m, hoehe_m[1] + 0.010)  # ein Turm darf höher sein als der Standard
    objekte = objekte_finden(punkte_basis, roi, min_hoehe_m, max_hoehe_m, zelle_m)
    if not objekte:
        raise ValueError(f"kein Objekt zwischen {min_hoehe_m * 1000:.0f} und {max_hoehe_m * 1000:.0f} mm Höhe")
    passend = nur_wuerfel(objekte, kante_m, hoehe_m)
    if passend and bei_m is not None:
        abstand = {id(w): math.hypot(w.mitte_m[0] - bei_m[0], w.mitte_m[1] - bei_m[1]) for w in passend}
        naechster = min(passend, key=lambda w: abstand[id(w)])
        if abstand[id(naechster)] <= bei_radius_m:
            return naechster
        raise ValueError(f"kein Würfel innerhalb {bei_radius_m * 1000:.0f} mm um ({bei_m[0] * 1000:.0f}, "
                         f"{bei_m[1] * 1000:.0f}) mm; nächster: {_text(naechster)}")
    if passend:
        return passend[0]
    raise ValueError("kein Objekt mit Würfelmaßen; Kandidaten: " + "; ".join(_text(w) for w in objekte[:5]))


def nur_wuerfel(objekte: list[Wuerfel], kante_m=(0.055, 0.085), hoehe_m=(0.05, 0.09)) -> list[Wuerfel]:
    """Objekte, deren Oberseite und Höhe zu einem Würfel bzw. Turm passen."""
    return [w for w in objekte if kante_m[0] <= w.kanten_m[1] and w.kanten_m[0] <= kante_m[1]
            and hoehe_m[0] <= w.hoehe_m <= hoehe_m[1]]


def alle_wuerfel(punkte_basis: np.ndarray, kante_m=(0.055, 0.085), hoehe_m=(0.05, 0.09),
                 zelle_m: float = 0.005) -> list[Wuerfel]:
    """Alle Würfel und Türme im Bild (für den Überblick beim Stapeln); leer, wenn keiner da ist."""
    objekte = objekte_finden(punkte_basis, min_hoehe_m=0.04, max_hoehe_m=hoehe_m[1] + 0.010, zelle_m=zelle_m)
    return nur_wuerfel(objekte, kante_m, hoehe_m)


def angeschnittener_wuerfel(objekte: list[Wuerfel], hoehe_m=(0.05, 0.09), min_kante_m: float = 0.03,
                            min_radius_m: float = 0.20) -> Wuerfel | None:
    """Kandidat mit Würfelhöhe, aber nur teilweise sichtbarer Oberseite (am Bildrand). Am Arm sah die Suche den
    Würfel so (z. B. 66 × 52 × 70 mm). Objekte näher als ``min_radius_m`` an der Basisachse werden übergangen (dort
    steht dauerhaft etwas, und greifen ließe es sich ohnehin nicht)."""
    passend = [w for w in objekte if hoehe_m[0] <= w.hoehe_m <= hoehe_m[1] and w.kanten_m[0] >= min_kante_m
               and math.hypot(w.mitte_m[0], w.mitte_m[1]) >= min_radius_m]
    return max(passend, key=lambda w: w.kanten_m[0] * w.kanten_m[1]) if passend else None


def stapel(w: Wuerfel, kante_m: float | None) -> tuple[int, float, float]:
    """(Anzahl Würfel übereinander, Unterseite des obersten, Boden unter dem Stapel). Mit Nennkante aus der Oberseite
    gerechnet, sonst aus der Tischhöhe. Die Oberseite misst das Tiefenbild auf ±0,5 mm, die Tischhöhe springt
    zwischen Tischplatte und gewelltem Kalibrierblatt (−27 / −23 mm, am Arm 11.11.2026)."""
    oben_z = w.tisch_z_m + w.hoehe_m
    if kante_m:
        n = round(w.hoehe_m / kante_m)
        if 1 <= n <= STAPEL_MAX and abs(w.hoehe_m - n * kante_m) <= STAPEL_TOLERANZ_M:
            return n, oben_z - kante_m, oben_z - n * kante_m
    return 1, w.tisch_z_m, w.tisch_z_m


def greifpose_wuerfel(w: Wuerfel, t_basis_werkzeug_jetzt: np.ndarray, werkzeug: np.ndarray,
                      eintauchen_m: float = 0.025, ueber_m: float = 0.10, kante_m: float | None = None) -> dict:
    """Greifen von oben: Werkzeug-z nach unten, Finger (Greifer-x) quer über die kürzere Kante. Fingerspitzen
    ``eintauchen_m`` unter der Oberseite (offener Greifer). Liefert Vorgriff- und Griffpose sowie Greiferweiten.
    kante_m: Nennkante; damit gelten Unterseite und Boden aus ``stapel`` (auch für den obersten Würfel eines Turms)."""
    oben_z = w.tisch_z_m + w.hoehe_m
    anzahl, unten_z, boden_z = stapel(w, kante_m)
    # Greifer-x entlang der Richtung, über die geschlossen wird: senkrecht zur längeren Kante = kürzere Kante
    kandidaten = []
    for zusatz in (90.0, -90.0):
        gier = math.radians(w.gier_deg + zusatz)
        r = kin.rot_z(gier) @ kin.rot_x(math.pi)  # z nach unten, x in der Tischebene
        kandidaten.append(r)
    r_ziel = min(kandidaten, key=lambda r: kin.winkel_abstand_deg(r, t_basis_werkzeug_jetzt[:3, :3]))
    offen = kin.greifer_position_fuer_weite(min(0.085, w.kanten_m[1] + 0.015))
    # Zentrierpunkt der Finger (nicht die Werkzeugachse) über die Würfelmitte: dorthin zieht der Greifer den Würfel
    p_spitze = np.array([w.mitte_m[0], w.mitte_m[1], oben_z - eintauchen_m])
    griff = kin.transform(r_ziel, p_spitze - r_ziel @ kin.fingermitte_im_werkzeug(werkzeug, offen))
    vorgriff = griff.copy()
    vorgriff[2, 3] += ueber_m
    return {"vorgriff": vorgriff, "griff": griff, "greifer_offen": offen,
            "greifer_zu": kin.greifer_position_fuer_weite(max(0.0, w.kanten_m[1] - 0.010)),
            "spitze_z_m": float(p_spitze[2]), "unten_z_m": float(unten_z), "boden_z_m": float(boden_z),
            "stapel": anzahl, "hoehe_m": float(w.hoehe_m),
            "mitte_m": [float(w.mitte_m[0]), float(w.mitte_m[1])], "gier_deg": float(w.gier_deg)}
