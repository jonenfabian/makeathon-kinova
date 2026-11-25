"""Kollisionsprüfung mit den Meshes von Kinova Gen3 (7 DoF) und Robotiq 2F-85: Arm gegen sich selbst, Tisch und
bekannte Hindernisse. Grundlage für Bahnsimulationen vor Gelenkfahrten (Wunsch Fabian, 9.11.2026).

Verfahren: Jedes Mesh wird auf ein Punktraster (``RASTER_M``) ausgedünnt und mit der URDF-Kinematik in die Basis
gelegt. Zwei Glieder kollidieren, wenn sich Punkte näher als ``ABSTAND_MIN_M`` kommen. Benachbarte Glieder (über ein
Gelenk verbunden) und Paare, die schon in der geprüften Home-Pose so nah sind (Gehäuse um ein Gelenk), werden nicht
gegeneinander geprüft. Grenzen: Punktabstand ≈ Raster, URDF-Nennmaße (≈ 7 mm gegen Kinovas Rechnung), keine Kabel.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cache
from pathlib import Path

import kinematik as kin
import numpy as np

MESHES = Path(__file__).resolve().parent / "static" / "meshes"
RASTER_M = 0.008
ABSTAND_MIN_M = 0.015  # Punkte zweier Glieder näher als das → Kollision (Rasterweite + Reserve)
TISCH_Z_M = -0.027  # Tischplatte im Basissystem laut Tiefenbild (9.11.2026)
TISCH_ABSTAND_M = 0.010

ARM = ("base_link", "shoulder_link", "half_arm_1_link", "half_arm_2_link", "forearm_link",
       "spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_link")
GREIFER = ("robotiq_base", "left_knuckle", "right_knuckle", "left_finger", "right_finger",
           "left_inner_knuckle", "right_inner_knuckle", "left_finger_tip", "right_finger_tip")
# Greiferteile gelten untereinander und mit Handgelenk/Armband als ein Körper
STARR = set(GREIFER) | {"bracelet_link"}

# Bekannte Hindernisse als Quader (x0, x1, y0, y1, z0, z1) in der Basis: Objekt neben dem Fuß, 59–79 mm hoch, aus den
# Tiefenbildern vom 9.11.2026 (x −12…48, y −184…−82 mm); mit 2 cm Zugabe.
HINDERNISSE = {"Objekt neben dem Fuß": (-0.032, 0.068, -0.204, -0.062, TISCH_Z_M, 0.08)}


def _stl_punkte(pfad: Path) -> np.ndarray:
    d = pfad.read_bytes()
    n = int.from_bytes(d[80:84], "little")
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float)


@cache
def glied_punkte(name: str) -> np.ndarray:
    """Ausgedünnte Oberflächenpunkte eines Glieds im eigenen Frame (Mitte je Rasterzelle)."""
    if name in ARM:
        datei = MESHES / "gen3_7dof" / ("bracelet_with_vision_link.STL" if name == "bracelet_link" else f"{name}.STL")
    else:
        datei = MESHES / "robotiq_2f85" / f"{name}.stl"
    p = _stl_punkte(datei)
    zellen = np.floor(p / RASTER_M).astype(np.int64)
    _, idx, inv = np.unique(zellen, axis=0, return_index=True, return_inverse=True)
    summe = np.zeros((len(idx), 3))
    np.add.at(summe, inv.ravel(), p)
    return summe / np.bincount(inv.ravel())[:, None]


def frames(gelenke_deg, greifer_position: float = 1.0) -> dict[str, np.ndarray]:
    f = kin.vorwaerts(gelenke_deg)
    aus = {n: f[n] for n in ARM}
    aus.update(kin.greifer_frames(f["end_effector_link"], greifer_position))
    return aus


def welt_punkte(gelenke_deg, greifer_position: float = 1.0) -> dict[str, np.ndarray]:
    return {n: glied_punkte(n) @ t[:3, :3].T + t[:3, 3] for n, t in frames(gelenke_deg, greifer_position).items()}


def _min_abstand(a: np.ndarray, b: np.ndarray) -> float:
    """Kleinster Punktabstand; liegen die Hüllquader weiter als ABSTAND_MIN_M auseinander, deren Abstand
    (untere Schranke, spart die Paarrechnung)."""
    luecke = np.maximum(0.0, np.maximum(a.min(0) - b.max(0), b.min(0) - a.max(0)))
    if float(np.linalg.norm(luecke)) > ABSTAND_MIN_M:
        return float(np.linalg.norm(luecke))
    d2 = np.min(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1))
    return float(np.sqrt(d2))


def _benachbart(a: str, b: str) -> bool:
    if a in STARR and b in STARR:
        return True
    if a in ARM and b in ARM and abs(ARM.index(a) - ARM.index(b)) <= 1:
        return True
    return {a, b} <= {"spherical_wrist_2_link", "robotiq_base"}


@dataclass
class Befund:
    frei: bool
    eigen: list = field(default_factory=list)  # (glied_a, glied_b, abstand_m)
    tisch: list = field(default_factory=list)  # (glied, z_min)
    hindernis: list = field(default_factory=list)  # (glied, hindernis)
    min_abstand_m: float = float("inf")
    min_tisch_m: float = float("inf")

    def text(self) -> str:
        if self.frei:
            return (f"frei (kleinster Abstand zwischen Gliedern {self.min_abstand_m * 1000:.0f} mm, "
                    f"zum Tisch {self.min_tisch_m * 1000:.0f} mm)")
        teile = [f"{a}↔{b} {d * 1000:.0f} mm" for a, b, d in self.eigen]
        teile += [f"{g} am Tisch (z {z * 1000:.0f} mm)" for g, z in self.tisch]
        teile += [f"{g} im Hindernis „{h}“" for g, h in self.hindernis]
        return "Kollision: " + "; ".join(teile)


def pruefen(gelenke_deg, greifer_position: float = 1.0, ausnahmen: frozenset = frozenset(),
            eigen: bool = True) -> Befund:
    """Eine Stellung prüfen. ``ausnahmen``: Gliederpaare, die nie gegeneinander geprüft werden. eigen=False: nur Tisch
    und Hindernisse, ohne die (teuren) Abstände der Glieder untereinander."""
    pts = welt_punkte(gelenke_deg, greifer_position)
    befund = Befund(True)
    namen = list(pts) if eigen else []
    for i, a in enumerate(namen):
        for b in namen[i + 1:]:
            if _benachbart(a, b) or frozenset((a, b)) in ausnahmen:
                continue
            d = _min_abstand(pts[a], pts[b])
            befund.min_abstand_m = min(befund.min_abstand_m, d)
            if d < ABSTAND_MIN_M:
                befund.eigen.append((a, b, d))
    for n, p in pts.items():
        if n == "base_link":
            continue
        z = float(p[:, 2].min()) - TISCH_Z_M
        befund.min_tisch_m = min(befund.min_tisch_m, z)
        if z < TISCH_ABSTAND_M:
            befund.tisch.append((n, float(p[:, 2].min())))
        for h, (x0, x1, y0, y1, z0, z1) in HINDERNISSE.items():
            if n in ("base_link", "shoulder_link"):
                continue
            innen = ((p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1) & (p[:, 2] > z0)
                     & (p[:, 2] < z1))
            if innen.any():
                befund.hindernis.append((n, h))
    befund.frei = not (befund.eigen or befund.tisch or befund.hindernis)
    return befund


def gelenk_bahn(von_deg, nach_deg, schritte: int = 60) -> list[list[float]]:
    """Gelenkbahn wie die Gelenkfahrt des Arms: alle Gelenke gleichzeitig linear; endlos drehende Gelenke (1, 3, 5, 7)
    auf dem kürzeren Weg (ANNAHME zum Verhalten von ReachJointAngles, am Arm beobachten)."""
    von = np.asarray(von_deg, float)
    diff = np.asarray(nach_deg, float) - von
    for i in (0, 2, 4, 6):
        diff[i] = (diff[i] + 180.0) % 360.0 - 180.0
    for i in (1, 3, 5):  # begrenzte Gelenke: echte Differenz der vorzeichenbehafteten Winkel
        diff[i] = kin.gelenk_signiert(nach_deg[i]) - kin.gelenk_signiert(von_deg[i])
    return [list(von + diff * s) for s in np.linspace(0.0, 1.0, schritte + 1)]


def bahn_pruefen(von_deg, nach_deg, greifer_position: float = 1.0, schritte: int = 60,
                 ausnahmen: frozenset = frozenset(), bekannt=()) -> tuple[bool, list[tuple[int, Befund]]]:
    """Alle Zwischenstellungen prüfen; liefert (frei, [(index, befund) der Kollisionen]).

    bekannt: Stellungen, die der echte Arm nachweislich einnimmt (die gemessene Startstellung, eine gespeicherte
    Parkposition). Die Parkposition vom 10.11.2026 liegt laut Modell 6,5° hinter dem Anschlag von Gelenk 4 (URDF ±147,3°)
    und mit vier Gliederpaaren unter 15 mm; trotzdem steht der Arm dort. Unterwegs gilt deshalb ein Anschlag oder ein zu
    knappes Gliederpaar als zulässig, wenn es nicht schlimmer ist als in einer bekannten Stellung (Gelenk nicht tiefer
    hinter dem Anschlag, Paar nicht näher). Tisch und Hindernisse bleiben immer verboten."""
    bekannt_reserven = [kin.anschlag_reserven(q) for q in bekannt]
    bekannt_paare: dict[frozenset, float] = {}
    for q in bekannt:
        for a, b, d in pruefen(q, greifer_position, ausnahmen).eigen:
            paar = frozenset((a, b))
            bekannt_paare[paar] = min(d, bekannt_paare.get(paar, d))
    fehler = []
    for i, q in enumerate(gelenk_bahn(von_deg, nach_deg, schritte)):
        b = pruefen(q, greifer_position, ausnahmen)
        # Paar bleibt Kollision, außer eine bekannte Stellung hat es mindestens so knapp (unbekanntes Paar: immer)
        b.eigen = [(x, y, d) for x, y, d in b.eigen
                   if d < bekannt_paare.get(frozenset((x, y)), float("inf")) - 0.001]
        for n, reserve in kin.anschlag_reserven(q).items():
            erlaubt = min([0.0, *(r[n] - 0.1 for r in bekannt_reserven)])
            if reserve < erlaubt:
                b.eigen.append(("Anschlag", f"Gelenk {n} {reserve:.1f}°", 0.0))
        b.frei = not (b.eigen or b.tisch or b.hindernis)
        if not b.frei:
            fehler.append((i, b))
    return not fehler, fehler
