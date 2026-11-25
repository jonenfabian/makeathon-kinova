"""Kinematik des Kinova Gen3 (7 Gelenke) mit Robotiq 2F-85 und Kameramodul.

Quelle der Geometrie: Kinovas Robotermodell ``ros2_kortex/kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro``
(BSD-3-Clause, Kinova inc.), Variante ohne externes Kabel; Kamera-Frames aus dem Zweig ``unless sim_gazebo``
(Nennmaße, keine Kalibrierung). Greifer: ``PickNikRobotics/ros2_robotiq_gripper``,
``robotiq_2f_85_macro.urdf.xacro`` (BSD-3-Clause). Stand der Dateien: 9.11.2026.

Einheiten: Meter und Radiant intern; die Kortex-API liefert Gelenkwinkel und Euler-Winkel in Grad.
Alle Transformationen sind 4×4-Matrizen ``T_a_b`` (Frame b in Frame a ausgedrückt).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# --- Grundbausteine ---------------------------------------------------------------------------------------------


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF-Konvention: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def transform(r: np.ndarray | None = None, p=(0.0, 0.0, 0.0)) -> np.ndarray:
    t = np.eye(4)
    if r is not None:
        t[:3, :3] = r
    t[:3, 3] = p
    return t


def urdf_origin(xyz, rpy) -> np.ndarray:
    return transform(rpy_matrix(*rpy), xyz)


def invers(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    return transform(r.T, -r.T @ p)


# --- Kinova-Euler-Winkel ----------------------------------------------------------------------------------------
# Kortex gibt die Orientierung als theta_x, theta_y, theta_z in Grad an. Konvention (durch Vergleich mit der
# gemessenen Pose bestätigt, tests/test_kinematik.py): feste Achsen x, dann y, dann z, R = Rz(θz)·Ry(θy)·Rx(θx).


def euler_matrix(theta_x_deg: float, theta_y_deg: float, theta_z_deg: float) -> np.ndarray:
    return rpy_matrix(math.radians(theta_x_deg), math.radians(theta_y_deg), math.radians(theta_z_deg))


def matrix_euler(r: np.ndarray) -> tuple[float, float, float]:
    """Umkehrung von euler_matrix; bei θy = ±90° (Singularität) wird θx = 0 gesetzt."""
    sy = -r[2, 0]
    if abs(sy) < 1.0 - 1e-9:
        ty = math.asin(sy)
        tx = math.atan2(r[2, 1], r[2, 2])
        tz = math.atan2(r[1, 0], r[0, 0])
    else:
        ty = math.copysign(math.pi / 2, sy)
        tx = 0.0
        tz = math.atan2(-r[0, 1], r[1, 1])
    return math.degrees(tx), math.degrees(ty), math.degrees(tz)


@dataclass(frozen=True)
class Pose:
    """Kortex-Pose: Position in m, Orientierung als Kinova-Euler-Winkel in Grad."""

    x: float
    y: float
    z: float
    theta_x: float
    theta_y: float
    theta_z: float

    def matrix(self) -> np.ndarray:
        return transform(euler_matrix(self.theta_x, self.theta_y, self.theta_z), (self.x, self.y, self.z))

    @staticmethod
    def aus_matrix(t: np.ndarray) -> Pose:
        tx, ty, tz = matrix_euler(t[:3, :3])
        return Pose(float(t[0, 3]), float(t[1, 3]), float(t[2, 3]), tx, ty, tz)

    def als_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "z": self.z,
                "theta_x": self.theta_x, "theta_y": self.theta_y, "theta_z": self.theta_z}


def winkel_abstand_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """Drehwinkel zwischen zwei Orientierungen in Grad."""
    c = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# --- Arm ---------------------------------------------------------------------------------------------------------
# (Name, Eltern-Link, Kind-Link, xyz, rpy), Achse jeweils z des Kind-Frames (URDF: axis 0 0 1).
GELENKE = (
    ("joint_1", "base_link", "shoulder_link", (0, 0, 0.15643), (3.1416, 2.7629e-18, -4.9305e-36)),
    ("joint_2", "shoulder_link", "half_arm_1_link", (0, 0.005375, -0.12838), (1.5708, 2.1343e-17, -1.1102e-16)),
    ("joint_3", "half_arm_1_link", "half_arm_2_link", (0, -0.21038, -0.006375), (-1.5708, 1.2326e-32, -2.9122e-16)),
    ("joint_4", "half_arm_2_link", "forearm_link", (0, 0.006375, -0.21038), (1.5708, -6.6954e-17, -1.6653e-16)),
    ("joint_5", "forearm_link", "spherical_wrist_1_link", (0, -0.20843, -0.006375), (-1.5708, 2.2204e-16, -6.373e-17)),
    ("joint_6", "spherical_wrist_1_link", "spherical_wrist_2_link", (0, 0.00017505, -0.10593),
     (1.5708, 9.2076e-28, -8.2157e-15)),
    ("joint_7", "spherical_wrist_2_link", "bracelet_link", (0, -0.10593, -0.00017505),
     (-1.5708, -5.5511e-17, 9.6396e-17)),
)
T_BRACELET_EE = urdf_origin((0, 0, -0.061525), (math.pi, 1.09937075168372e-32, 0))
# Kameramodul, reale Hardware (Zweig unless sim_gazebo): optische Achse = z des Endeffektors. Nennmaße.
T_EE_FARBE = urdf_origin((0, 0.05639, -0.00305), (math.pi, math.pi, 0))
# Tiefenkamera: Nennmaß (0,0275 / 0,066) um (−0,9 / +1,0) mm korrigiert. Am Arm (11.11.2026) maß sie einen ruhenden
# 60-mm-Würfel bei vier Drehungen des Handgelenks um (0,9 / −1,0) mm (Werkzeug-x/-y) versetzt, gemittelt aus zwei
# Reihen (gegen die kalibrierte Farbkamera und gegen das Mittel der vier Drehungen). Danach stimmten Tiefenbild und
# Farbkamera bei vier Drehungen auf ±0,6 mm überein (vorher bis 1,4 mm).
T_EE_TIEFE = urdf_origin((0.0275 - 0.0009, 0.066 + 0.0010, -0.00305), (math.pi, math.pi, 0))
# Zentrierpunkt des Greifers: Der Robotiq 2F-85 zieht ein Teil beim Schließen zwischen die Finger, am Arm (11.11.2026,
# 60-mm-Holzwürfel, Farbkamera auf dem Kalibrierblatt) 0,4 mm neben die Werkzeugachse in −x des Werkzeugs. Achse 3 mm
# links, rechts oder genau über der Mitte, mit und ohne Anheben, 0 oder 2 mm über dem Tisch losgelassen: elf Proben,
# −0,2 … −0,8 mm. In der ersten Stunde lagen umgesetzte Würfel 2,1 mm daneben (drei Proben, danach nicht mehr; Ursache
# offen, vielleicht Tape auf den Backen). ANNAHME: gilt für jede Greiferweite. Vor Ort: tools/ablage_pruefen.py.
GREIFER_MITTE_X_M = -0.0004
GLIEDER = ("base_link",) + tuple(g[2] for g in GELENKE)
# Anschläge laut gen3_macro.xacro (joint_2/4/6: ±2,24 / ±2,57 / ±2,09 rad); Gelenke 1, 3, 5, 7 drehen endlos.
GELENK_GRENZEN_DEG = {2: math.degrees(2.24), 4: math.degrees(2.57), 6: math.degrees(2.09)}


def gelenk_signiert(winkel_deg: float) -> float:
    """Kortex meldet 0…360°; Anschläge sind symmetrisch um 0 → in (−180, 180] umrechnen."""
    return ((float(winkel_deg) + 180.0) % 360.0) - 180.0


def anschlag_reserven(gelenke_deg) -> dict[int, float]:
    """Abstand zum Anschlag je begrenztem Gelenk in Grad (negativ = jenseits)."""
    return {n: g - abs(gelenk_signiert(gelenke_deg[n - 1])) for n, g in GELENK_GRENZEN_DEG.items()}

# --- Greifer Robotiq 2F-85 -----------------------------------------------------------------------------------
# robotiq_85_base_link sitzt am end_effector_link (ros2_kortex: origin 0 0 0). Gelenkwinkel 0 = offen, 0,8 rad = zu.
GREIFER_WINKEL_ZU_RAD = 0.8
_KNOECHEL = np.array([0.03060114, 0.0, 0.05490452])
_FINGER = np.array([0.03152616, 0.0, -0.00376347])
_SPITZE = np.array([0.00563134, 0.0, 0.04718515])
_SPITZE_INNEN_X = -0.0253  # Innenfläche des Fingerspitzen-Meshes (left_finger_tip.stl, x min)
_SPITZE_ENDE_Z = 0.0510  # Ende des Fingerspitzen-Meshes in z (left_finger_tip.stl, z max)


def greifer_winkel(position: float) -> float:
    """Kortex-Greiferstellung 0 (offen) … 1 (zu) → Knöchelwinkel. ANNAHME: linear, am Greifer nachmessen."""
    return GREIFER_WINKEL_ZU_RAD * min(1.0, max(0.0, position))


def _spitze_im_greifer(winkel: float) -> np.ndarray:
    """Ursprung der linken Fingerspitze im robotiq_85_base_link (Parallelführung: Spitze bleibt senkrecht)."""
    return _KNOECHEL + rot_y(-winkel) @ (_FINGER + _SPITZE)


def greifer_weite_m(position: float) -> float:
    """Lichter Abstand der Fingerspitzen-Innenflächen (Modell): ≈ 0,085 m offen, 0 zu."""
    return max(0.0, 2.0 * (_spitze_im_greifer(greifer_winkel(position))[0] + _SPITZE_INNEN_X))


def greifer_position_fuer_weite(weite_m: float) -> float:
    """Umkehrung von greifer_weite_m per Bisektion (Weite fällt monoton mit der Position)."""
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mitte = (lo + hi) / 2
        if greifer_weite_m(mitte) > weite_m:
            lo = mitte
        else:
            hi = mitte
    return (lo + hi) / 2


def fingerspitze_unter_flansch_m(position: float) -> float:
    """Abstand Flansch (end_effector_link) → Ende der Fingerspitzen entlang Werkzeug-z (Modell, ANNAHME:
    Greifer sitzt ohne Zwischenplatte am Flansch; mit Lineal prüfen). Offen ≈ 0,149 m, zu ≈ 0,163 m."""
    return float(_spitze_im_greifer(greifer_winkel(position))[2] + _SPITZE_ENDE_Z)


def fingermitte_im_werkzeug(werkzeug: np.ndarray, position: float) -> np.ndarray:
    """Mitte zwischen den Fingerspitzen an ihrem unteren Ende, im Werkzeug-KS: der Punkt, auf den der Greifer ein Teil
    zentriert (``GREIFER_MITTE_X_M`` neben der Achse)."""
    return (invers(werkzeug) @ np.array([GREIFER_MITTE_X_M, 0.0, fingerspitze_unter_flansch_m(position), 1.0]))[:3]


def greifer_frames(t_ee: np.ndarray, position: float) -> dict[str, np.ndarray]:
    """Lage der Robotiq-Glieder (Namen wie die Mesh-Dateien) für den Digital Twin. Achse aller Greifergelenke
    (0, −1, 0), mimic-Faktoren aus dem Robotiq-Modell; Drehung um −y mit Winkel a = rot_y(−a)."""
    w = greifer_winkel(position)

    def gelenk(xyz, winkel):
        return transform(rot_y(-winkel), xyz)

    basis = t_ee
    lk = basis @ gelenk((0.03060114, 0.0, 0.05490452), w)
    rk = basis @ gelenk((-0.03060114, 0.0, 0.05490452), -w)
    lf = lk @ transform(None, (0.03152616, 0.0, -0.00376347))
    rf = rk @ transform(None, (-0.03152616, 0.0, -0.00376347))
    return {
        "robotiq_base": basis, "left_knuckle": lk, "right_knuckle": rk, "left_finger": lf, "right_finger": rf,
        "left_inner_knuckle": basis @ gelenk((0.0127, 0.0, 0.06142), w),
        "right_inner_knuckle": basis @ gelenk((-0.0127, 0.0, 0.06142), -w),
        "left_finger_tip": lf @ gelenk((0.00563134, 0.0, 0.04718515), -w),
        "right_finger_tip": rf @ gelenk((-0.00563134, 0.0, 0.04718515), w),
    }


# --- Vorwärtskinematik -----------------------------------------------------------------------------------------


def vorwaerts(gelenke_deg, werkzeug: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """Lage aller Glieder, Endeffektor, Werkzeugpunkt und Kamera-Frames in der Roboterbasis."""
    if len(gelenke_deg) != len(GELENKE):
        raise ValueError(f"{len(GELENKE)} Gelenkwinkel erwartet, {len(gelenke_deg)} erhalten")
    frames = {"base_link": np.eye(4)}
    t = np.eye(4)
    for (name, _eltern, kind, xyz, rpy), winkel in zip(GELENKE, gelenke_deg):
        t = t @ urdf_origin(xyz, rpy) @ transform(rot_z(math.radians(float(winkel))))
        frames[kind] = t
    ee = t @ T_BRACELET_EE
    frames["end_effector_link"] = ee
    frames["werkzeug"] = ee @ (werkzeug if werkzeug is not None else WERKZEUG_STANDARD)
    frames["kamera_farbe"] = ee @ T_EE_FARBE
    frames["kamera_tiefe"] = ee @ T_EE_TIEFE
    return frames


WERKZEUG_STANDARD = transform(None, (0.0, 0.0, 0.12))  # vom Arm gemeldet (ControlConfig.GetToolConfiguration)


def werkzeug_aus_kortex(tool_transform: dict) -> np.ndarray:
    """ToolConfiguration.tool_transform (x, y, z in m, theta_* in Grad) → 4×4."""
    p = Pose(tool_transform.get("x", 0.0), tool_transform.get("y", 0.0), tool_transform.get("z", 0.0),
             tool_transform.get("theta_x", 0.0), tool_transform.get("theta_y", 0.0), tool_transform.get("theta_z", 0.0))
    return p.matrix()


# --- Inverse Kinematik ----------------------------------------------------------------------------------------


def jacobi(frames: dict[str, np.ndarray]) -> np.ndarray:
    """Geometrische Jacobi-Matrix (6 × 7) des Werkzeugpunkts; Zeilen: Geschwindigkeit, Winkelgeschwindigkeit (Basis).
    Die Achse eines Gelenks ist z seines Kind-Frames (URDF ``axis 0 0 1``)."""
    p = frames["werkzeug"][:3, 3]
    j = np.zeros((6, len(GELENKE)))
    for i, g in enumerate(GELENKE):
        f = frames[g[2]]
        j[:3, i] = np.cross(f[:3, 2], p - f[:3, 3])
        j[3:, i] = f[:3, 2]
    return j


def inverse_kinematik(t_ziel: np.ndarray, q_start_deg, werkzeug: np.ndarray | None = None,
                      iterationen: int = 300, mitte: float = 0.05) -> tuple[np.ndarray, float, float]:
    """Gelenkwinkel (Grad, 0…360) für eine Werkzeugpose, ausgehend von ``q_start_deg`` (gedämpfte kleinste Quadrate).
    Die eine überzählige Achse (7 Gelenke, 6 Bedingungen) zieht die begrenzten Gelenke 2/4/6 Richtung 0°.
    Liefert (Gelenke, Restfehler Position in m, Restfehler Orientierung in Grad). Nennmodell der URDF: die vom Arm
    gemessene Pose weicht ≈ 7 mm/1° ab; für Ziele, die genauer sein müssen, ``ReachPose`` verwenden."""
    q = np.radians(np.asarray(q_start_deg, dtype=float))
    begrenzt = {1: math.radians(GELENK_GRENZEN_DEG[2]), 3: math.radians(GELENK_GRENZEN_DEG[4]),
                5: math.radians(GELENK_GRENZEN_DEG[6])}
    for _ in range(iterationen):
        f = vorwaerts(np.degrees(q), werkzeug)
        t = f["werkzeug"]
        e = np.concatenate([t_ziel[:3, 3] - t[:3, 3],
                            0.5 * sum(np.cross(t[:3, i], t_ziel[:3, i]) for i in range(3))])
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-5:
            break
        j = jacobi(f)
        j_plus = j.T @ np.linalg.inv(j @ j.T + 1e-4 * np.eye(6))
        gradient = np.zeros(len(q))
        for i, grenze in begrenzt.items():
            gradient[i] = (((q[i] + math.pi) % (2 * math.pi)) - math.pi) / grenze ** 2
        dq = j_plus @ e - mitte * (np.eye(len(q)) - j_plus @ j) @ gradient
        groesste = float(np.max(np.abs(dq)))
        if groesste > math.radians(5.0):
            dq *= math.radians(5.0) / groesste
        q += dq
    t = vorwaerts(np.degrees(q), werkzeug)["werkzeug"]
    return (np.degrees(q) % 360.0, float(np.linalg.norm(t_ziel[:3, 3] - t[:3, 3])),
            winkel_abstand_deg(t[:3, :3], t_ziel[:3, :3]))


# --- Schritte ----------------------------------------------------------------------------------------------------


def schritt(t_basis_werkzeug: np.ndarray, d_pos_m, d_rot_deg, bezug: str) -> np.ndarray:
    """Zielpose für einen Schritt. bezug="basis": Verschiebung entlang der Basisachsen, Drehung um Basisachsen
    durch den Werkzeugpunkt. bezug="werkzeug": Verschiebung und Drehung in den Achsen des Werkzeugs."""
    d_r = euler_matrix(*d_rot_deg)
    if bezug == "basis":
        ziel = t_basis_werkzeug.copy()
        ziel[:3, :3] = d_r @ t_basis_werkzeug[:3, :3]
        ziel[:3, 3] = t_basis_werkzeug[:3, 3] + np.asarray(d_pos_m, dtype=float)
        return ziel
    if bezug == "werkzeug":
        return t_basis_werkzeug @ transform(d_r, d_pos_m)
    raise ValueError(f"unbekannter Bezug {bezug!r} (basis oder werkzeug)")


# --- Arbeitsraum -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Arbeitsraum:
    """Grenzen für jede Zielpose (Software, zusätzlich zu den Firmwaregrenzen des Arms)."""

    xy_max_m: float = 0.70  # Tisch: ±0,80 m frei (Fabian, 9.11.2026), 0,10 m Abstand
    z_max_m: float = 0.80
    tisch_z_m: float = 0.0  # ANNAHME: Basisursprung liegt auf der Tischplatte; mit Tiefenbild prüfen
    abstand_tisch_m: float = 0.010  # Mindestabstand Fingerspitzen/Flansch über dem Tisch
    schritt_max_m: float = 0.050
    schritt_max_deg: float = 15.0
    abstand_basis_m: float = 0.20  # kein Werkzeugpunkt näher als das an der Basisachse (unterhalb z_basis)
    z_basis_m: float = 0.30

    def pruefen(self, t_ziel: np.ndarray, werkzeug: np.ndarray, greifer_position: float = 1.0,
                abstand_tisch_m: float | None = None) -> list[str]:
        """Liefert die Gründe, warum die Zielpose verboten ist (leer = erlaubt). Geprüft werden Werkzeugpunkt,
        Flansch und Fingerspitzen; für die Fingerspitzen gilt die längste Stellung (zu), außer es wird eine
        andere Greiferstellung übergeben."""
        gruende = []
        t_ee = t_ziel @ invers(werkzeug)
        spitze = t_ee @ np.array([0.0, 0.0, fingerspitze_unter_flansch_m(max(greifer_position, 0.0)), 1.0])
        tisch_min = self.tisch_z_m + (self.abstand_tisch_m if abstand_tisch_m is None else abstand_tisch_m)
        for name, p in (("Werkzeugpunkt", t_ziel[:3, 3]), ("Flansch", t_ee[:3, 3]), ("Fingerspitzen", spitze[:3])):
            if abs(p[0]) > self.xy_max_m or abs(p[1]) > self.xy_max_m:
                gruende.append(f"{name} außerhalb ±{self.xy_max_m:.2f} m in x/y ({p[0]:.3f}, {p[1]:.3f})")
            if p[2] > self.z_max_m:
                gruende.append(f"{name} über z = {self.z_max_m:.2f} m ({p[2]:.3f})")
            if p[2] < tisch_min:
                gruende.append(f"{name} unter {tisch_min * 1000:.0f} mm über dem Tisch (z = {p[2] * 1000:.0f} mm)")
        r = math.hypot(t_ziel[0, 3], t_ziel[1, 3])
        if t_ziel[2, 3] < self.z_basis_m and r < self.abstand_basis_m:
            gruende.append(f"Werkzeugpunkt zu nah an der Basis (r = {r:.3f} m < {self.abstand_basis_m:.2f} m)")
        return gruende

    def schritt_pruefen(self, d_pos_m, d_rot_deg) -> list[str]:
        gruende = []
        if float(np.linalg.norm(d_pos_m)) > self.schritt_max_m + 1e-9:
            gruende.append(f"Schritt {np.linalg.norm(d_pos_m) * 1000:.0f} mm > {self.schritt_max_m * 1000:.0f} mm")
        if max(abs(float(a)) for a in d_rot_deg) > self.schritt_max_deg + 1e-9:
            gruende.append(f"Drehschritt > {self.schritt_max_deg:.0f}°")
        return gruende
