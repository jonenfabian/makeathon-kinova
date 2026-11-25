"""Kalibrierung der Farbkamera: Intrinsik und Hand-Auge-Transformation (Werkzeug → Kamera) mit einem ChArUco-Brett.

Anlass (9.11.2026): Das Kameramodul meldet für die Farbkamera nur Kinovas allgemeine Vorgabe-Intrinsik (cy = 238 bei
720 px Höhe); AprilTags lagen damit ≈ 5 mm und ≈ 10° daneben. Die Tiefenkamera trifft mit den Werkswerten auf < 1 mm
und bleibt unverändert.

Brett: 6 × 8 Felder, Quadrat 30 mm, Marker 22 mm, ``DICT_4X4_50`` (Druckvorlage ``tools/kalibrierbrett.py``).
Brett-Frame (OpenCV): Ursprung an der äußeren Ecke des Felds oben links, x entlang der 6 Felder, y entlang der 8,
z in das Brett hinein (liegt das Brett auf dem Tisch, zeigt z nach unten).

Verfahren:
1. Das Brett liegt fest auf dem Tisch. Der Arm fährt mit Gelenkfahrten (Bahnsimulation wie immer) eine Reihe von
   Stellungen an, aus denen die Kamera das Brett aus 0,3–0,5 m Abstand, bis 40° geneigt und um bis zu ±45° gedreht
   sieht. Je Stellung: Bild, gemessene Werkzeugpose (Kinova), Gelenkwinkel.
2. Intrinsik mit ``cv2.calibrateCamera`` (Brettlage je Bild frei, k3 fest auf 0).
3. Hand-Auge als Bündelausgleich: gesucht sind ``T_werkzeug_kamera`` und ``T_basis_brett`` (je 6 Freiheitsgrade), so
   dass alle Brettecken über die gemessene Werkzeugpose ins Bild projiziert möglichst genau auf die erkannten Ecken
   fallen (Levenberg-Marquardt, Intrinsik aus Schritt 2 fest). Das ist die „Roboter-Welt-Hand-Auge“-Formulierung;
   ``cv2.calibrateHandEye`` fehlt in opencv-python-headless 5.0.
4. Gegenproben: Streuung der Brettecken aus den Einzelbildern in der Basis, Höhe der Brettfläche im Tiefenbild,
   Kreuzprüfung an zurückgehaltenen Ansichten (Vorhersage nur aus Armpose und Kalibrierung), Abdeckung des Bildes.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import arm as arm_mod
import cv2
import kinematik as kin
import kollision
import numpy as np
import sehen

QUADRAT_M = 0.030
MARKER_M = 0.022
FELDER = (6, 8)
WOERTERBUCH = cv2.aruco.DICT_4X4_50
MIN_ECKEN = 12  # weniger erkannte Brettecken: Bild verwerfen
MIN_ANSICHTEN = 8
ANSCHLAG_RESERVE_DEG = 15.0
# Stellungen der Kamera relativ zur Brettmitte: (Abstand m, Neigung gegen die Senkrechte °, Richtung in der Basis
# von +x aus °, Drehung um die optische Achse °[, Brettmitte im Bild rechts °, unten °]). Ohne die letzten beiden
# Werte liegt die Brettmitte auf der optischen Achse; mit ihnen schaut die Kamera vom selben Ort bewusst daneben, damit
# das Brett am Bildrand liegt. Mischung aus Abständen, Neigungen, Drehungen und Bildlagen, damit Intrinsik
# (Verzeichnung am Bildrand, Perspektive) und Hand-Auge (Drehungen um verschiedene Achsen) bestimmt sind.
# Starke Neigungen nur seitlich (Richtung 60–120° und 240–300°): Von der Roboterseite (180°) läuft Gelenk 4, von
# außen (0°) bei ≥ 30° Gelenk 6 in den Anschlag; das liegt an der Blickpose, nicht an der Home-Pose oder am Zweig der
# inversen Kinematik (geprüft 9.11.2026 mit Brett bei x 440 mm, ``tools/kalibrierung_planen.py``).
STELLUNGEN = (
    (0.42, 0, 0, 0), (0.34, 0, 0, 30), (0.50, 0, 0, -30),
    (0.40, 20, 0, 0), (0.40, 20, 90, 20), (0.48, 25, 150, -20), (0.40, 20, 270, 0),
    (0.40, 35, 75, 0), (0.40, 40, 105, 15), (0.40, 40, 255, -15), (0.40, 35, 285, 0),
    (0.32, 25, 0, -30), (0.48, 25, 210, 30), (0.48, 25, 90, -15), (0.48, 25, 270, 15),
    (0.36, 30, 60, 30), (0.36, 30, 240, -30), (0.45, 15, 120, 45), (0.45, 15, 300, -45),
    # Bildränder und -ecken (Verzeichnung ist dort am größten; ohne diese bleiben die äußeren Bildviertel leer)
    (0.48, 0, 0, 0, 16, 0), (0.48, 0, 0, 0, -16, 0),
    (0.48, 10, 90, 0, 15, 9), (0.48, 10, 270, 0, -15, 9), (0.48, 10, 90, 0, 15, -9), (0.48, 10, 270, 0, -15, -9),
    (0.44, 20, 60, 15, 14, 8), (0.44, 20, 240, -15, -14, -8), (0.44, 20, 120, -15, -14, 8), (0.44, 20, 300, 15, 14, -8),
    # um ±90° gedreht: Drehungen um die Werkzeugachse für die Hand-Auge-Kette, Brett hochkant im Bild
    (0.42, 0, 0, 90), (0.42, 0, 0, -90), (0.40, 25, 90, 90), (0.40, 25, 270, -90),
)
# Gütegrenzen. Simulation 10.11.2026: Schon 0,2 mm / 0,03° Zufallsfehler der gemeldeten Armpose (ANNAHME, am Arm
# offen) ergeben ≈ 1,0 px Bildfehler und ≈ 1,2 px (bis 1,75 px) in der Gegenprobe bei < 1,5 mm Lagefehler auf dem
# Tisch. 2 px entsprechen in 45 cm Abstand ≈ 0,75 mm.
GUETE_RMS_PX = 2.0  # Bündelausgleich: mittlerer Bildfehler je Ecke
GUETE_STREUUNG_MM = 3.0  # größte Abweichung einer Einzelbild-Brettlage von der gemeinsamen Lösung
GUETE_GEGENPROBE_PX = 3.0  # Bildfehler auf zurückgehaltenen Ansichten (Vorhersage über die gemessene Armpose)
GEGENPROBE_ANZAHL = 4  # Ansichten, die für die Gegenprobe nicht in die Rechnung eingehen
RASTER = (8, 5)  # Abdeckung des Bildes: Spalten × Zeilen
# Am Arm (10.11.2026) war eine von 31 Ansichten verwackelt (5,2 px gegen höchstens 2,1 px bei den übrigen, Streuung
# 11,7 mm): Der Arm schwang nach der Gelenkfahrt noch nach. Deshalb Ruheprüfung bei der Aufnahme und einmaliges
# Auslassen klarer Ausreißer bei der Auswertung.
AUSREISSER_PX = 3.0  # eine Ansicht gilt als Ausreißer über diesem Bildfehler und über dem Dreifachen des Medians
AUSREISSER_ANTEIL = 0.1  # höchstens so viele Ansichten werden ausgelassen
RUHE_PX = 0.5  # Ecken dürfen sich zwischen erstem und letztem Bild einer Aufnahme höchstens so weit bewegen
RUHE_VERSUCHE = 3


def brett(quadrat_m: float = QUADRAT_M) -> cv2.aruco.CharucoBoard:
    """ChArUco-Brett; ``quadrat_m`` ist die nachgemessene Kante (Druckmaßstab), der Marker skaliert mit."""
    return cv2.aruco.CharucoBoard(FELDER, quadrat_m, quadrat_m * MARKER_M / QUADRAT_M,
                                  cv2.aruco.getPredefinedDictionary(WOERTERBUCH))


def brett_mitte(quadrat_m: float = QUADRAT_M) -> np.ndarray:
    return np.array([FELDER[0] * quadrat_m / 2, FELDER[1] * quadrat_m / 2, 0.0])


def objektpunkte(ids, quadrat_m: float = QUADRAT_M) -> np.ndarray:
    return np.asarray(brett(quadrat_m).getChessboardCorners(), dtype=np.float64)[np.asarray(ids, dtype=int).ravel()]


def ecken_finden(bild_bgr: np.ndarray, quadrat_m: float = QUADRAT_M) -> tuple[np.ndarray, np.ndarray] | None:
    """Erkannte Brettecken: (ids, Pixel N×2) oder None bei weniger als ``MIN_ECKEN``."""
    grau = cv2.cvtColor(bild_bgr, cv2.COLOR_BGR2GRAY) if bild_bgr.ndim == 3 else bild_bgr
    ecken, ids, _, _ = cv2.aruco.CharucoDetector(brett(quadrat_m)).detectBoard(grau)
    if ids is None or len(ids) < MIN_ECKEN:
        return None
    return ids.ravel().astype(int), ecken.reshape(-1, 2).astype(np.float64)


def _pnp(obj: np.ndarray, px: np.ndarray, intr: dict) -> np.ndarray:
    k, dist = sehen.kameramatrix(intr)
    ok, rvec, tvec = cv2.solvePnP(obj, px, k, dist)
    if not ok:
        raise ValueError("solvePnP fehlgeschlagen")
    return kin.transform(cv2.Rodrigues(rvec)[0], tvec.ravel())


def _exp(d: np.ndarray) -> np.ndarray:
    return kin.transform(cv2.Rodrigues(np.asarray(d[:3], dtype=np.float64))[0], d[3:])


def _projizieren(obj: np.ndarray, t_kamera_objekt: np.ndarray, intr: dict) -> np.ndarray:
    k, dist = sehen.kameramatrix(intr)
    rvec = cv2.Rodrigues(t_kamera_objekt[:3, :3])[0]
    proj, _ = cv2.projectPoints(obj, rvec, t_kamera_objekt[:3, 3], k, dist)
    return proj.reshape(-1, 2)


@dataclass
class Ansicht:
    t_basis_werkzeug: np.ndarray  # gemessene Werkzeugpose (Kinova)
    gelenke_deg: list
    ids: np.ndarray
    px: np.ndarray
    datei: str | None = None
    info: dict = field(default_factory=dict)

    def als_dict(self) -> dict:
        return {"datei": self.datei, "gelenke_deg": [float(g) for g in self.gelenke_deg],
                "t_basis_werkzeug": self.t_basis_werkzeug.tolist(), "ids": self.ids.tolist(),
                "px": self.px.round(3).tolist(), **self.info}

    @staticmethod
    def aus_dict(d: dict) -> Ansicht:
        info = {k: v for k, v in d.items() if k not in ("t_basis_werkzeug", "gelenke_deg", "ids", "px", "datei")}
        return Ansicht(np.array(d["t_basis_werkzeug"]), d["gelenke_deg"], np.array(d["ids"], dtype=int),
                       np.array(d["px"], dtype=np.float64), d.get("datei"), info)


# --- Auswertung ---------------------------------------------------------------------------------------------------


def intrinsik_schaetzen(ansichten: list[Ansicht], bildgroesse: tuple[int, int],
                        quadrat_m: float = QUADRAT_M) -> tuple[dict, float]:
    """``cv2.calibrateCamera`` über alle Ansichten (k3 = 0). Liefert (Intrinsik wie ``Arm.kamera_intrinsik``, RMS px)."""
    obj = [objektpunkte(a.ids, quadrat_m).astype(np.float32) for a in ansichten]
    img = [a.px.astype(np.float32) for a in ansichten]
    rms, k, dist, _, _ = cv2.calibrateCamera(obj, img, bildgroesse, None, None, flags=cv2.CALIB_FIX_K3)
    d = dist.ravel().tolist() + [0.0] * (5 - dist.size)
    return ({"aufloesung": f"{bildgroesse[0]}x{bildgroesse[1]}", "fx": float(k[0, 0]), "fy": float(k[1, 1]),
             "cx": float(k[0, 2]), "cy": float(k[1, 2]), "verzeichnung": [float(v) for v in d[:5]]}, float(rms))


def hand_auge(ansichten: list[Ansicht], intr: dict, t_werkzeug_kamera_start: np.ndarray,
              quadrat_m: float = QUADRAT_M, iterationen: int = 60) -> dict:
    """Bündelausgleich über ``T_werkzeug_kamera`` und ``T_basis_brett`` (Levenberg-Marquardt, Störung jeweils von
    rechts im eigenen Frame). Startwert für das Brett: Einzelbild-Lage der ersten Ansicht über den Startwert."""
    obj = [objektpunkte(a.ids, quadrat_m) for a in ansichten]
    t_wk = np.array(t_werkzeug_kamera_start, dtype=float)
    t_bb = ansichten[0].t_basis_werkzeug @ t_wk @ _pnp(obj[0], ansichten[0].px, intr)

    def residuen(wk: np.ndarray, bb: np.ndarray) -> np.ndarray:
        return np.concatenate([(_projizieren(o, kin.invers(a.t_basis_werkzeug @ wk) @ bb, intr) - a.px).ravel()
                               for a, o in zip(ansichten, obj)])

    r = residuen(t_wk, t_bb)
    kosten = float(r @ r)
    lam = 1e-3
    h = 1e-6
    for _ in range(iterationen):
        j = np.empty((r.size, 12))
        for i in range(12):
            d = np.zeros(12)
            d[i] = h
            j[:, i] = (residuen(t_wk @ _exp(d[:6]), t_bb @ _exp(d[6:])) - r) / h
        a = j.T @ j
        g = j.T @ r
        while True:
            schritt = -np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), g)
            wk_neu, bb_neu = t_wk @ _exp(schritt[:6]), t_bb @ _exp(schritt[6:])
            r_neu = residuen(wk_neu, bb_neu)
            if float(r_neu @ r_neu) < kosten:
                t_wk, t_bb, r, kosten = wk_neu, bb_neu, r_neu, float(r_neu @ r_neu)
                lam = max(lam / 3, 1e-9)
                break
            lam *= 5
            if lam > 1e6:
                break
        if lam > 1e6 or float(np.max(np.abs(schritt))) < 1e-10:
            break
    je_ansicht, start = [], 0
    for o in obj:
        n = 2 * len(o)
        je_ansicht.append(float(np.sqrt(np.mean(r[start:start + n].reshape(-1, 2) ** 2) * 2)))
        start += n
    return {"t_werkzeug_kamera": t_wk, "t_basis_brett": t_bb,
            "rms_px": float(np.sqrt(np.mean(r.reshape(-1, 2) ** 2) * 2)), "je_ansicht_px": je_ansicht}


def streuung(ansichten: list[Ansicht], intr: dict, t_werkzeug_kamera: np.ndarray, t_basis_brett: np.ndarray,
             quadrat_m: float = QUADRAT_M) -> list[float]:
    """Je Ansicht: größter Abstand (mm) der Brettecken aus der Einzelbild-Lage (PnP über die gemessene Werkzeugpose)
    zu den Ecken der gemeinsamen Lösung. Prüft Hand-Auge und Armgenauigkeit zusammen."""
    alle = np.asarray(brett(quadrat_m).getChessboardCorners(), dtype=np.float64)
    soll = alle @ t_basis_brett[:3, :3].T + t_basis_brett[:3, 3]
    aus = []
    for a in ansichten:
        t = a.t_basis_werkzeug @ t_werkzeug_kamera @ _pnp(objektpunkte(a.ids, quadrat_m), a.px, intr)
        ist = alle @ t[:3, :3].T + t[:3, 3]
        aus.append(float(np.max(np.linalg.norm(ist - soll, axis=1)) * 1000))
    return aus


def brett_im_tiefenbild(punkte_basis: np.ndarray, t_basis_brett: np.ndarray, quadrat_m: float = QUADRAT_M,
                        rand_m: float = 0.015) -> dict | None:
    """Höhe der Brettfläche im Tiefenbild gegen die Brettebene der Farbkalibrierung. ``abstand_mm`` > 0: die
    Tiefenkamera sieht die Fläche höher als die Farbkamera."""
    p = (punkte_basis - t_basis_brett[:3, 3]) @ t_basis_brett[:3, :3]  # im Brett-Frame (z ins Brett)
    nx, ny = FELDER
    innen = ((p[:, 0] > rand_m) & (p[:, 0] < nx * quadrat_m - rand_m) & (p[:, 1] > rand_m)
             & (p[:, 1] < ny * quadrat_m - rand_m) & (np.abs(p[:, 2]) < 0.03))
    if innen.sum() < 200:
        return None
    z = -p[innen, 2] * 1000
    return {"punkte": int(innen.sum()), "abstand_mm": round(float(np.median(z)), 2),
            "streuung_mm": round(float(np.percentile(z, 84) - np.percentile(z, 16)) / 2, 2)}


def abdeckung(ansichten: list[Ansicht], bildgroesse: tuple[int, int]) -> dict:
    """Erkannte Ecken je Bildfeld (``RASTER``, Zeile 0 = oben) über alle Ansichten."""
    spalten, zeilen = RASTER
    raster = np.zeros((zeilen, spalten), dtype=int)
    for a in ansichten:
        s = np.clip((a.px[:, 0] * spalten / bildgroesse[0]).astype(int), 0, spalten - 1)
        z = np.clip((a.px[:, 1] * zeilen / bildgroesse[1]).astype(int), 0, zeilen - 1)
        np.add.at(raster, (z, s), 1)
    return {"raster": raster.tolist(), "leere_felder": int((raster == 0).sum()),
            "felder_unter_10": int((raster < 10).sum())}


def _vorhersage_px(a: Ansicht, intr: dict, t_werkzeug_kamera: np.ndarray, t_basis_brett: np.ndarray,
                   quadrat_m: float) -> np.ndarray:
    return _projizieren(objektpunkte(a.ids, quadrat_m), kin.invers(a.t_basis_werkzeug @ t_werkzeug_kamera)
                        @ t_basis_brett, intr)


def gegenprobe(ansichten: list[Ansicht], bildgroesse: tuple[int, int], werkzeug: np.ndarray,
               t_ee_kamera_vorher: np.ndarray, quadrat_m: float = QUADRAT_M,
               anzahl: int = GEGENPROBE_ANZAHL) -> dict | None:
    """Kreuzprüfung: ``anzahl`` über den Ablauf verteilte Ansichten zurückhalten (nie die erste), mit den übrigen
    Intrinsik und Hand-Auge rechnen und die Ecken der zurückgehaltenen allein aus gemessener Armpose, Hand-Auge und
    Brettlage vorhersagen. Prüft die ganze Kette an Bildern, die nicht in die Rechnung eingingen."""
    n = len(ansichten)
    if anzahl < 1 or n - anzahl < MIN_ANSICHTEN:
        return None
    zurueck = sorted({round(float(v)) for v in np.linspace(1, n - 1, anzahl + 2)[1:-1]})
    rechnen = [a for i, a in enumerate(ansichten) if i not in zurueck]
    intr, _ = intrinsik_schaetzen(rechnen, bildgroesse, quadrat_m)
    he = hand_auge(rechnen, intr, kin.invers(werkzeug) @ t_ee_kamera_vorher, quadrat_m)
    fehler = [np.linalg.norm(_vorhersage_px(ansichten[i], intr, he["t_werkzeug_kamera"], he["t_basis_brett"], quadrat_m)
                             - ansichten[i].px, axis=1) for i in zurueck]
    alle = np.concatenate(fehler)
    return {"ansichten": zurueck, "rms_px": round(float(np.sqrt(np.mean(alle ** 2))), 3),
            "max_px": round(float(alle.max()), 2), "je_ansicht_px": [round(float(np.sqrt(np.mean(f ** 2))), 3)
                                                                     for f in fehler]}


def auswerten(ansichten: list[Ansicht], bildgroesse: tuple[int, int], werkzeug: np.ndarray,
              t_ee_kamera_vorher: np.ndarray, quadrat_m: float = QUADRAT_M) -> dict:
    """Wie ``_auswerten_alle``; klare Ausreißer (Bildfehler über ``AUSREISSER_PX`` und über dem Dreifachen des Medians,
    höchstens ``AUSREISSER_ANTEIL``) werden einmal ausgelassen und unter ``ausgelassen`` genannt."""
    erg = _auswerten_alle(ansichten, bildgroesse, werkzeug, t_ee_kamera_vorher, quadrat_m)
    fehler = np.array(erg["je_ansicht_px"])
    grenze = max(AUSREISSER_PX, 3.0 * float(np.median(fehler)))
    raus = [i for i in np.argsort(-fehler) if fehler[i] > grenze][:int(AUSREISSER_ANTEIL * len(ansichten))]
    if not raus or len(ansichten) - len(raus) < MIN_ANSICHTEN:
        return {**erg, "ausgelassen": []}
    behalten = [a for i, a in enumerate(ansichten) if i not in raus]
    neu = _auswerten_alle(behalten, bildgroesse, werkzeug, t_ee_kamera_vorher, quadrat_m)
    neu["ausgelassen"] = [{"datei": ansichten[i].datei, "fehler_px": float(fehler[i]),
                           "streuung_mm": erg["streuung_mm"][i], **ansichten[i].info} for i in sorted(raus)]
    return neu


def _auswerten_alle(ansichten: list[Ansicht], bildgroesse: tuple[int, int], werkzeug: np.ndarray,
                    t_ee_kamera_vorher: np.ndarray, quadrat_m: float = QUADRAT_M) -> dict:
    """Intrinsik, Hand-Auge, Streuung, Gegenprobe, Abdeckung und Vergleich mit dem bisherigen Wert. ``t_ee_kamera``
    ist im selben Sinn wie ``kinematik.T_EE_FARBE`` (Endeffektor → optischer Frame) und ersetzt diesen. Das Ergebnis
    rechnet mit allen übergebenen Ansichten; die Gegenprobe ist nur eine Prüfung."""
    if len(ansichten) < MIN_ANSICHTEN:
        raise arm_mod.BewegungAbgelehnt(f"nur {len(ansichten)} brauchbare Ansichten (mindestens {MIN_ANSICHTEN})")
    intr, rms_intr = intrinsik_schaetzen(ansichten, bildgroesse, quadrat_m)
    he = hand_auge(ansichten, intr, kin.invers(werkzeug) @ t_ee_kamera_vorher, quadrat_m)
    t_ee = werkzeug @ he["t_werkzeug_kamera"]
    d = kin.invers(t_ee_kamera_vorher) @ t_ee
    streu = streuung(ansichten, intr, he["t_werkzeug_kamera"], he["t_basis_brett"], quadrat_m)
    t_bb = he["t_basis_brett"]
    mitte = t_bb[:3, :3] @ brett_mitte(quadrat_m) + t_bb[:3, 3]
    probe = gegenprobe(ansichten, bildgroesse, werkzeug, t_ee_kamera_vorher, quadrat_m)
    guete = {"rms_px": round(he["rms_px"], 3), "streuung_max_mm": round(max(streu), 2),
             "streuung_median_mm": round(float(np.median(streu)), 2), "ansichten": len(ansichten),
             "gegenprobe_rms_px": None if probe is None else probe["rms_px"]}
    guete["brauchbar"] = bool(he["rms_px"] <= GUETE_RMS_PX and max(streu) <= GUETE_STREUUNG_MM
                              and (probe is None or probe["rms_px"] <= GUETE_GEGENPROBE_PX))
    return {
        "intrinsik": intr, "rms_intrinsik_px": round(rms_intr, 3), "guete": guete,
        "je_ansicht_px": [round(v, 3) for v in he["je_ansicht_px"]], "streuung_mm": [round(v, 2) for v in streu],
        "t_ee_kamera": t_ee.tolist(), "t_basis_brett": t_bb.tolist(),
        "aenderung": {"versatz_mm": (d[:3, 3] * 1000).round(2).tolist(),
                      "betrag_mm": round(float(np.linalg.norm(d[:3, 3])) * 1000, 2),
                      "drehung_deg": round(kin.winkel_abstand_deg(np.eye(3), d[:3, :3]), 3)},
        "brett": {"mitte_mm": (mitte * 1000).round(1).tolist(), "oberflaeche_z_mm": round(float(mitte[2]) * 1000, 2),
                  "neigung_deg": round(math.degrees(math.acos(min(1.0, -float(t_bb[2, 2])))), 3)},
        "quadrat_mm": round(quadrat_m * 1000, 3), "ecken": int(sum(len(a.ids) for a in ansichten)),
        "gegenprobe": probe, "abdeckung": abdeckung(ansichten, bildgroesse),
    }


# --- Stellungen planen --------------------------------------------------------------------------------------------


def kamera_pose(mitte: np.ndarray, x_bezug: np.ndarray, abstand_m: float, neigung_deg: float, richtung_deg: float,
                drehung_deg: float, bild_u_deg: float = 0.0, bild_v_deg: float = 0.0) -> np.ndarray:
    """Kamerapose (optischer Frame) mit Blick auf ``mitte``; x möglichst entlang ``x_bezug``, dann um die optische
    Achse gedreht. ``bild_u_deg``/``bild_v_deg``: vom selben Ort so geschwenkt, dass ``mitte`` um diese Winkel rechts
    bzw. unterhalb der optischen Achse erscheint (≈ fx · tan(Winkel) Pixel)."""
    n, r = math.radians(neigung_deg), math.radians(richtung_deg)
    p = mitte + abstand_m * np.array([math.sin(n) * math.cos(r), math.sin(n) * math.sin(r), math.cos(n)])
    z = (mitte - p) / np.linalg.norm(mitte - p)
    x = x_bezug - (x_bezug @ z) * z
    x /= np.linalg.norm(x)
    r_kamera = (np.column_stack([x, np.cross(z, x), z]) @ kin.rot_z(math.radians(drehung_deg))
                @ kin.rot_x(math.radians(bild_v_deg)) @ kin.rot_y(-math.radians(bild_u_deg)))
    return kin.transform(r_kamera, p)


def stellung_text(stellung) -> str:
    abstand, neigung, richtung, drehung, *bild = stellung
    text = f"{abstand * 100:.0f} cm, {neigung}° Richtung {richtung}°, gedreht {drehung:+d}°"
    if any(bild):
        text += f", Brett im Bild {bild[0]:+d}°/{bild[1]:+d}°"
    return text


def stellungen_planen(t_basis_brett: np.ndarray, gelenke_start, werkzeug: np.ndarray, t_ee_kamera: np.ndarray,
                      raum: kin.Arbeitsraum, greifer_position: float = 0.0, x_bezug: np.ndarray | None = None,
                      quadrat_m: float = QUADRAT_M) -> tuple[list[dict], list[dict]]:
    """Gelenkwinkel für ``STELLUNGEN`` (inverse Kinematik ab ``gelenke_start``), nur erreichbare, kollisionsfreie mit
    ≥ ``ANSCHLAG_RESERVE_DEG`` Reserve; nach kürzesten Gelenkwegen sortiert. Liefert (Plan, verworfene)."""
    mitte = t_basis_brett[:3, :3] @ brett_mitte(quadrat_m) + t_basis_brett[:3, 3]
    if x_bezug is None:
        x_bezug = (kin.vorwaerts(gelenke_start)["end_effector_link"] @ t_ee_kamera)[:3, 0]
    t_werkzeug_kamera = kin.invers(werkzeug) @ t_ee_kamera
    plan, verworfen = [], []
    for stellung in STELLUNGEN:
        text = stellung_text(stellung)
        t_kamera = kamera_pose(mitte, np.asarray(x_bezug, float), *stellung)
        t_werkzeug = t_kamera @ kin.invers(t_werkzeug_kamera)
        q, rest_m, rest_deg = kin.inverse_kinematik(t_werkzeug, gelenke_start, werkzeug)
        grund = None
        if rest_m > 1e-3 or rest_deg > 0.5:
            grund = f"nicht erreichbar (Rest {rest_m * 1000:.0f} mm, {rest_deg:.1f}°)"
        elif min(kin.anschlag_reserven(q).values()) < ANSCHLAG_RESERVE_DEG:
            grund = f"zu nah am Anschlag ({min(kin.anschlag_reserven(q).values()):.0f}°)"
        elif raum.pruefen(t_werkzeug, werkzeug, max(greifer_position, 0.0)):
            grund = "; ".join(raum.pruefen(t_werkzeug, werkzeug, max(greifer_position, 0.0)))
        else:
            befund = kollision.pruefen(q, greifer_position)
            if not befund.frei:
                grund = befund.text()
        eintrag = {"text": text, "gelenke_deg": [round(float(v), 3) for v in q],
                   "pose": kin.Pose.aus_matrix(t_werkzeug).als_dict()}
        (verworfen if grund else plan).append({**eintrag, "grund": grund} if grund else eintrag)
    # Reihenfolge: jeweils die nächste Stellung mit dem kleinsten größten Gelenkweg
    def weg(a, b):
        return max(abs(((x - y) + 180.0) % 360.0 - 180.0) for x, y in zip(a, b))

    sortiert, jetzt, rest = [], list(gelenke_start), list(plan)
    while rest:
        naechste = min(rest, key=lambda e: weg(jetzt, e["gelenke_deg"]))
        rest.remove(naechste)
        sortiert.append(naechste)
        jetzt = naechste["gelenke_deg"]
    return sortiert, verworfen


# --- Konfiguration --------------------------------------------------------------------------------------------------


def laden(pfad: Path) -> dict | None:
    return json.loads(pfad.read_text(encoding="utf-8")) if pfad.exists() else None


def speichern(pfad: Path, ergebnis: dict, ordner: str | None) -> dict:
    daten = {"kalibriert": time.strftime("%Y-%m-%dT%H:%M:%S"), "quelle": ordner,
             "intrinsik": ergebnis["intrinsik"], "t_ee_kamera": ergebnis["t_ee_kamera"],
             "guete": ergebnis["guete"], "quadrat_mm": ergebnis["quadrat_mm"],
             "hinweis": "Farbkamera: Intrinsik und Endeffektor → optischer Frame (ersetzt kinematik.T_EE_FARBE)"}
    pfad.parent.mkdir(parents=True, exist_ok=True)
    pfad.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return daten


# --- Ablauf am Arm --------------------------------------------------------------------------------------------------


class KalibrierAblauf:
    """Aufnahme am Arm über den Würfel-Ablauf (gleiche Sperre, Statusanzeige und STOPP)."""

    def __init__(self, db):
        self.db = db
        self.arm = db.arm
        self.letztes: dict | None = None
        self.ordner: Path | None = None

    def _bild(self) -> tuple[np.ndarray, np.ndarray, list, float | None]:
        """Bild im Stillstand: drei Bilder, Ecken im ersten und letzten vergleichen; bewegen sie sich um mehr als
        ``RUHE_PX``, kurz warten und neu aufnehmen. Liefert auch die gemessene Bewegung (None: kein Vergleich)."""
        for versuch in range(RUHE_VERSUCHE):
            bilder, _ = self.db._bild_und_kamera("farbe", anzahl=3)
            a, b = ecken_finden(bilder[0], self._quadrat_m), ecken_finden(bilder[-1], self._quadrat_m)
            unruhe = None
            if a is not None and b is not None:
                ea, eb = dict(zip(a[0].tolist(), a[1])), dict(zip(b[0].tolist(), b[1]))
                gemeinsam = set(ea) & set(eb)
                if gemeinsam:
                    unruhe = max(float(np.linalg.norm(ea[i] - eb[i])) for i in gemeinsam)
            if unruhe is None or unruhe <= RUHE_PX or versuch == RUHE_VERSUCHE - 1:
                return bilder[-1], self.arm.pose_matrix(), list(self.arm.zustand()["gelenke_deg"]), unruhe
            time.sleep(0.5)
        raise AssertionError("unerreichbar")

    def _ansicht(self, nummer: int) -> Ansicht | None:
        bild, t_bw, gelenke, unruhe = self._bild()
        datei = self.ordner / f"ansicht_{nummer:02d}.png"
        cv2.imwrite(str(datei), bild)
        self._bildgroesse = (bild.shape[1], bild.shape[0])
        f = ecken_finden(bild, self._quadrat_m)
        if f is None:
            return None
        v = Ansicht(t_bw, gelenke, f[0], f[1], datei.name)
        if unruhe is not None:
            v.info["unruhe_px"] = round(unruhe, 2)
        return v

    def api_aufnehmen(self, d: dict) -> dict:
        quadrat_m = float(d.get("quadrat_mm", QUADRAT_M * 1000)) / 1000.0
        if not 0.025 <= quadrat_m <= 0.035:
            raise arm_mod.BewegungAbgelehnt("Quadratkante 25–35 mm erwartet (nachgemessen, Soll 30 mm)")
        return self.db.ablauf._ausfuehren("Farbkamera kalibrieren", self._aufnehmen, quadrat_m)

    def _aufnehmen(self, quadrat_m: float) -> dict:
        a = self.db.ablauf
        self._quadrat_m = quadrat_m
        self.ordner = self.db.messungen / f"kalibrierung_{time.strftime('%Y%m%d_%H%M%S')}"
        self.ordner.mkdir(parents=True, exist_ok=True)
        a._home()
        a._schritt("Brett suchen")
        erste = self._ansicht(0)
        if erste is None:
            raise arm_mod.BewegungAbgelehnt("Kalibrierbrett nicht im Bild (mindestens 12 Ecken nötig)")
        intr = self.db.intrinsik["farbe"]
        t_wk = kin.invers(self.arm.werkzeug) @ self.db.t_ee["farbe"]
        t_bb = erste.t_basis_werkzeug @ t_wk @ _pnp(objektpunkte(erste.ids, quadrat_m), erste.px, intr)
        mitte = t_bb[:3, :3] @ brett_mitte(quadrat_m) + t_bb[:3, 3]
        a.status["verlauf"][-1]["ergebnis"] = (f"{len(erste.ids)} Ecken, Brettmitte ≈ ({mitte[0] * 1000:.0f}, "
                                               f"{mitte[1] * 1000:.0f}, {mitte[2] * 1000:.0f}) mm")
        plan, verworfen = stellungen_planen(t_bb, erste.gelenke_deg, self.arm.werkzeug, self.db.t_ee["farbe"],
                                            self.arm.raum, self.arm.zustand().get("greifer_position") or 0.0,
                                            quadrat_m=quadrat_m)
        ansichten = [erste]
        for i, s in enumerate(plan, start=1):
            a._schritt(f"Ansicht {i}/{len(plan)}: {s['text']}")
            try:
                e = self.arm.gelenke_anfahren(s["gelenke_deg"], f"Kalibrierung {i}/{len(plan)}", pose_pruefen=s["pose"])
            except arm_mod.BewegungAbgelehnt as exc:  # Bahnsimulation: diese Stellung auslassen
                a.status["verlauf"][-1]["ergebnis"] = f"ausgelassen: {exc}"
                continue
            a._ok(e, f"Ansicht {i}")
            v = self._ansicht(i)
            if v is None:
                a.status["verlauf"][-1]["ergebnis"] += "; Brett nicht erkannt"
                continue
            v.info = {"stellung": s["text"], **v.info}
            ansichten.append(v)
            a.status["verlauf"][-1]["ergebnis"] += f"; {len(v.ids)} Ecken"
        a._home()
        a._schritt("Tiefenbild der Brettfläche")
        punkte = a._tiefe_in_basis()
        np.save(self.ordner / "tiefe_punkte_basis.npy", punkte.astype(np.float32))
        roh = {"quadrat_mm": quadrat_m * 1000, "bildgroesse": self._bildgroesse, "werkzeug": self.arm.werkzeug.tolist(),
               "t_ee_kamera_vorher": self.db.t_ee["farbe"].tolist(), "intrinsik_vorher": intr,
               "verworfen": verworfen, "ansichten": [v.als_dict() for v in ansichten]}
        (self.ordner / "ansichten.json").write_text(json.dumps(roh, indent=1, ensure_ascii=False), encoding="utf-8")
        a._schritt("auswerten")
        return self._auswerten(roh, punkte)

    def _auswerten(self, roh: dict, punkte: np.ndarray | None) -> dict:
        quadrat_m = roh["quadrat_mm"] / 1000.0
        ansichten = [Ansicht.aus_dict(v) for v in roh["ansichten"]]
        erg = auswerten(ansichten, tuple(roh["bildgroesse"]), np.array(roh["werkzeug"]),
                        np.array(roh["t_ee_kamera_vorher"]), quadrat_m)
        if punkte is not None:
            erg["tiefe_gegen_farbe"] = brett_im_tiefenbild(punkte, np.array(erg["t_basis_brett"]), quadrat_m)
        erg["ordner"] = str(self.ordner)
        erg["verworfen"] = roh.get("verworfen", [])
        (self.ordner / "ergebnis.json").write_text(json.dumps(erg, indent=1, ensure_ascii=False), encoding="utf-8")
        self.letztes = erg
        return erg

    def api_rechnen(self, d: dict) -> dict:
        """Gespeicherte Aufnahme neu auswerten, z. B. mit nachgemessener Quadratkante (ohne Bewegung)."""
        ordner = Path(d["ordner"]) if d.get("ordner") else self.ordner
        if ordner is None or not (ordner / "ansichten.json").exists():
            raise arm_mod.BewegungAbgelehnt("keine Kalibrieraufnahme vorhanden")
        if ordner.resolve().parent != self.db.messungen.resolve():
            raise arm_mod.BewegungAbgelehnt("Ordner muss unter messungen/ liegen")
        roh = json.loads((ordner / "ansichten.json").read_text(encoding="utf-8"))
        if d.get("quadrat_mm"):
            roh["quadrat_mm"] = float(d["quadrat_mm"])
        self.ordner = ordner
        tiefe = ordner / "tiefe_punkte_basis.npy"
        return self._auswerten(roh, np.load(tiefe).astype(np.float64) if tiefe.exists() else None)

    def api_uebernehmen(self, d: dict) -> dict:
        if self.letztes is None:
            raise arm_mod.BewegungAbgelehnt("erst kalibrieren")
        if not self.letztes["guete"]["brauchbar"] and d.get("trotzdem") is not True:
            raise arm_mod.BewegungAbgelehnt(f"Güte nicht ausreichend: {self.letztes['guete']}")
        daten = speichern(self.db.kamera_konfig_pfad, self.letztes, str(self.ordner))
        self.db.farbkamera_setzen(daten)
        return daten

    def api_werkswerte(self, _d: dict) -> dict:
        """Zurück zu Kinovas Vorgabe-Intrinsik und den URDF-Nennmaßen; die Konfigurationsdatei wird umbenannt."""
        pfad = self.db.kamera_konfig_pfad
        if pfad.exists():
            pfad.rename(pfad.with_name(f"{pfad.stem}_{time.strftime('%Y%m%d_%H%M%S')}.alt.json"))
        self.db.farbkamera_setzen(None)
        return {"farbkamera": "Werkswerte"}
