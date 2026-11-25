"""Vorberechnete Gelenkbahnen durch mehrere Wegpunkte ohne Halt (Fabian 11.11.2026: Stapeln „so schnell wie möglich,
möglichst mit OptiMove“).

Hintergrund:
- Glatte Bahnen durch mehrere Wegpunkte bietet Kinova als ``ExecuteWaypointTrajectory`` mit ``use_optimal_blending``
  erst ab Firmware 2.3.0 (Kortex-Release 2.3.0 vom 4.6.2021: „Joint and cartesian waypoint support“). Der Arm läuft mit
  2.2.0 (Festlegung Fabian, 9.11.2026).
- Firmware 2.2.0 kann ``Base.PlayPreComputedJointTrajectory``. Der Rechner schickt die ganze Bahn als Gelenkwinkel,
  -geschwindigkeiten und -beschleunigungen im Takt von 1 ms. Der erste Punkt (Zeit 0) muss der jetzigen Stellung
  entsprechen, die Steuerung prüft die Bahn vor dem Start (Kortex-Doku ``PreComputedJointTrajectory``).
- Kinova selbst nutzte genau das in ros_kortex 2.2.1 für MoveIt (``pre_computed_joint_trajectory_action_server.cpp``:
  Stetigkeit ``TRAJECTORY_CONTINUITY_MODE_POSITION``, Abstand der Punkte genau 1 ms).

Rechnung:
1. Die Wegpunkte (Werkzeugposen in der Basis) werden geradlinig verbunden und die Ecken mit einer quadratischen
   Bézierkurve gerundet (``rundung_m``). So fährt der Arm durch die Ecke, ohne anzuhalten.
2. Der Pfad wird dicht abgetastet (4 mm bzw. 1,5°). Jede Abtastung wird per inverser Kinematik des Nennmodells umgerechnet.
   Vorher wird die Abweichung des Nennmodells an der gemessenen Startstellung abgezogen.
3. Das Tempo entlang des Pfads wird zeitoptimal unter den Gelenkgrenzen für Tempo und Beschleunigung bestimmt
   (Vorwärts-/Rückwärtsrechnung über ``ṡ²`` wie bei TOPP). Danach wird im 1-ms-Takt abgetastet und jede Probe gegen die
   Grenzen geprüft. Werden sie überschritten, wird die Zeit gedehnt.
4. Ruck begrenzen (Fabian 12.11.2026: „nicht ruckeln, sanft anfahren und abbremsen, aber zügig“): Das zeitoptimale Tempo
   aus 3. beschleunigt sprunghaft (die Beschleunigung springt von 0 auf die Grenze und an jeder Umschaltung um). Ein
   gleitender Mittelwert über ``rampe_s`` (FIR-Filter wie bei CNC-Steuerungen) auf den Fortschritt s(t) entlang des
   Pfads macht daraus ein S-Profil: Die Beschleunigung steigt in ``rampe_s`` gleichmäßig an und fällt ebenso ab. Der Arm
   bleibt dabei genau auf dem geprüften Pfad (gefiltert wird nur, wann er wo ist), Start und Ende stehen exakt still, die
   Bahn dauert ``rampe_s`` länger. Danach werden die Proben wie in 3. gegen die Grenzen geprüft.

Genauigkeit: Das Nennmodell weicht am Arm ≈ 7 mm ab (kinematik.py). Nach der Korrektur an der Startstellung bleibt am
Ziel ein Rest (**ANNAHME** wenige mm, am Arm messen). Bahnen enden deshalb kurz vor einem Ziel, das genau sitzen muss.
Den Rest fährt ``ReachPose`` von der gemessenen Pose aus.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import kinematik as kin
import numpy as np

TAKT_S = 0.001
# Grenzen am Arm (kinova_lesen 9.11.2026): Tempo hart = weich 80 °/s (Gelenke 1–4) und 70 °/s (5–7). Beschleunigung weich
# im Modus ANGULAR_TRAJECTORY 40 °/s² für alle Gelenke (hart 298 bzw. 573 °/s²). Welche Grenze die Firmware für
# vorberechnete Bahnen prüft, ist offen (**ANNAHME**: die weiche, am Arm mit kleinen Bahnen prüfen).
GELENK_V_GRENZE_DEG_S = np.array([80.0, 80.0, 80.0, 80.0, 70.0, 70.0, 70.0])
GELENK_A_GRENZE_DEG_S2 = np.array([40.0] * 7)
GELENK_A_HART_DEG_S2 = np.array([298.0] * 4 + [573.0] * 3)
SCHRITT_M = 0.004
SCHRITT_DEG = 1.5
IK_REST_M = 0.001
IK_REST_DEG = 0.3
SPRUNG_MAX_DEG = 4.0  # zwischen zwei Abtastungen; mehr heißt: IK springt in eine andere Armstellung
GLAETTUNG_PROBEN = 3.0  # Breite (σ) des Gauß-Filters über den Gelenkpfad, in Abtastungen (4 mm)
GLAETTUNG_MAX_M = 0.003  # so weit darf der geglättete Pfad vom kartesischen abweichen (Nennmodell)
# Ruck der Gelenke, der noch sanft wirkt (**ANNAHME**, am Arm mit der Ruckprobe prüfen, README „Sanft fahren“). Daraus die
# Rampe: 2 · a / Ruck, bei a = 36 °/s² also 0,18 s. Höchstens 0,4 s, damit schnelle Bahnen zügig bleiben.
RUCK_GRENZE_DEG_S3 = 400.0
RAMPE_MIN_S = 0.10
RAMPE_MAX_S = 0.40
# Glätte-Kennwerte (``kennwerte``): Proben auf ein 10-ms-Raster, vor jeder Ableitung über 3 Proben gemittelt
MESS_RASTER_S = 0.01
MESS_FENSTER = 3


class BahnFehler(Exception):
    """Die Bahn lässt sich nicht planen (Pose nicht erreichbar, Sprung der inversen Kinematik). Nichts gesendet."""


@dataclass
class Bahn:
    zeit_s: np.ndarray  # (M,) im 1-ms-Takt ab 0
    winkel_deg: np.ndarray  # (M, 7) stetig, ab der Startstellung ohne Sprung um 360°
    tempo_deg_s: np.ndarray  # (M, 7)
    beschl_deg_s2: np.ndarray  # (M, 7)
    posen: list[np.ndarray]  # dichter kartesischer Pfad (Nennmodell, korrigiert), für Prüfungen
    gelenkpfad_deg: np.ndarray  # (N, 7) Gelenkwinkel zu ``posen``, geglättet
    v_grenze: np.ndarray = field(default_factory=lambda: GELENK_V_GRENZE_DEG_S.copy())
    a_grenze: np.ndarray = field(default_factory=lambda: GELENK_A_GRENZE_DEG_S2.copy())
    rampe_s: float = 0.0  # Anfahr- und Bremsrampe der Beschleunigung (Ruckbegrenzung)

    @property
    def dauer_s(self) -> float:
        return float(self.zeit_s[-1])

    @property
    def ruck_max_deg_s3(self) -> float:
        """Größter Ruck (Änderung der Beschleunigung je Sekunde) über alle Gelenke und Proben."""
        return float(np.max(np.abs(np.diff(self.beschl_deg_s2, axis=0)))) / TAKT_S if len(self.zeit_s) > 1 else 0.0

    def auslastung(self) -> tuple[float, float]:
        """Größtes Verhältnis von Tempo bzw. Beschleunigung zur Grenze der Firmware (Tempo) bzw. zur benutzten
        Beschleunigungsgrenze über alle Proben und Gelenke."""
        return (float(np.max(np.abs(self.tempo_deg_s) / GELENK_V_GRENZE_DEG_S)),
                float(np.max(np.abs(self.beschl_deg_s2) / self.a_grenze)))


# --- 1. Kartesischer Pfad mit gerundeten Ecken --------------------------------------------------------------------


def _slerp(r0: np.ndarray, r1: np.ndarray, f: float) -> np.ndarray:
    """Drehung zwischen r0 und r1 (f = 0 … 1) um die feste Achse von r0ᵀr1."""
    d = r0.T @ r1
    w = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])
    s = np.linalg.norm(w) / 2
    c = (np.trace(d) - 1) / 2
    winkel = math.atan2(s, c)
    if winkel < 1e-9:
        return r0.copy()
    achse = w / (2 * s) if s > 1e-9 else np.array([1.0, 0.0, 0.0])  # 180°: Achse beliebig senkrecht (kommt nicht vor)
    return r0 @ _drehung(achse, f * winkel)


def _drehung(achse: np.ndarray, winkel: float) -> np.ndarray:
    k = np.array([[0, -achse[2], achse[1]], [achse[2], 0, -achse[0]], [-achse[1], achse[0], 0]])
    return np.eye(3) + math.sin(winkel) * k + (1 - math.cos(winkel)) * k @ k


def pfad(posen: list[np.ndarray], rundung_m: float = 0.03, schritt_m: float = SCHRITT_M,
         schritt_deg: float = SCHRITT_DEG) -> list[np.ndarray]:
    """Dichter Pfad durch die Wegpunkte (4×4-Posen). Gerade Stücke, an inneren Ecken eine Bézierkurve, die ``rundung_m``
    vor der Ecke beginnt und danach endet (höchstens bis zur halben Länge der angrenzenden Stücke). Die Drehung läuft auf
    jedem Stück gleichmäßig mit."""
    if len(posen) < 2:
        raise BahnFehler("eine Bahn braucht mindestens zwei Posen")
    p = [np.asarray(t[:3, 3], float) for t in posen]
    r = [np.asarray(t[:3, :3], float) for t in posen]
    laengen = [float(np.linalg.norm(p[i + 1] - p[i])) for i in range(len(p) - 1)]
    d = [0.0] * len(p)
    for i in range(1, len(p) - 1):
        if laengen[i - 1] > 1e-6 and laengen[i] > 1e-6:
            d[i] = min(rundung_m, 0.5 * laengen[i - 1], 0.5 * laengen[i])

    def auf_stueck(i: int, s_m: float) -> np.ndarray:
        """Pose auf Stück i (von p[i] nach p[i+1]) bei s_m Metern ab p[i]."""
        f = 0.0 if laengen[i] < 1e-9 else min(1.0, max(0.0, s_m / laengen[i]))
        return kin.transform(_slerp(r[i], r[i + 1], f), p[i] + f * (p[i + 1] - p[i]))

    def anzahl(weg_m: float, r_a: np.ndarray, r_b: np.ndarray) -> int:
        return max(1, math.ceil(weg_m / schritt_m), math.ceil(kin.winkel_abstand_deg(r_a, r_b) / schritt_deg))

    aus = [kin.transform(r[0], p[0])]
    for i in range(len(p) - 1):
        # gerades Stück zwischen dem Ende der Rundung an p[i] und dem Beginn der Rundung an p[i+1]
        s0, s1 = d[i], laengen[i] - d[i + 1]
        a, b = auf_stueck(i, s0), auf_stueck(i, s1)
        n = anzahl(s1 - s0, a[:3, :3], b[:3, :3])
        aus += [auf_stueck(i, s0 + (s1 - s0) * k / n) for k in range(1, n + 1)]
        if i + 1 < len(p) - 1 and d[i + 1] > 0:  # Rundung an p[i+1]
            e = auf_stueck(i + 1, d[i + 1])
            n = anzahl(2 * d[i + 1], b[:3, :3], e[:3, :3])
            for k in range(1, n + 1):
                f = k / n
                pos = (1 - f) ** 2 * b[:3, 3] + 2 * f * (1 - f) * p[i + 1] + f ** 2 * e[:3, 3]
                aus.append(kin.transform(_slerp(b[:3, :3], e[:3, :3], f), pos))
    return [t for k, t in enumerate(aus) if k == 0 or np.linalg.norm(t[:3, 3] - aus[k - 1][:3, 3]) > 1e-7
            or kin.winkel_abstand_deg(t[:3, :3], aus[k - 1][:3, :3]) > 1e-4]


# --- 2. Gelenkpfad -------------------------------------------------------------------------------------------------


def signiert(q_deg) -> np.ndarray:
    """Gelenkwinkel in (−180°, 180°], wie ROS-Gelenkzustände (ros_kortex 2.2.1 schickte MoveIt-Bahnen so)."""
    q = np.asarray(q_deg, float)
    return -((-q + 180.0) % 360.0 - 180.0)


def gelenkpfad(posen: list[np.ndarray], q_start_deg, werkzeug: np.ndarray, t_gemessen: np.ndarray) -> np.ndarray:
    """Gelenkwinkel (N × 7, stetig) zum dichten Pfad. Erste Zeile = Startstellung (vorzeichenbehaftet).
    t_gemessen: vom Arm gemessene Werkzeugpose in der Startstellung. Die Abweichung des Nennmodells dort wird von allen
    Zielen abgezogen."""
    q0 = np.asarray(q_start_deg, float)
    t_fk = kin.vorwaerts(q0, werkzeug)["werkzeug"]
    fehler = t_gemessen @ kin.invers(t_fk)  # gemessen = fehler · Nennmodell
    korr = kin.invers(fehler)
    q_vorher = signiert(q0)
    aus = [q_vorher]
    for k, t in enumerate(posen[1:], start=1):
        q, rest_m, rest_deg = kin.inverse_kinematik(korr @ t, q_vorher, werkzeug, iterationen=60)
        if rest_m > IK_REST_M or rest_deg > IK_REST_DEG:
            raise BahnFehler(f"Pose bei {100 * k // len(posen)} % der Bahn nicht erreichbar "
                             f"(Rest {rest_m * 1000:.1f} mm, {rest_deg:.2f}°)")
        q = q_vorher + ((q - q_vorher + 180.0) % 360.0 - 180.0)  # stetig weiter, kein Sprung um 360°
        sprung = float(np.max(np.abs(q - q_vorher)))
        if sprung > SPRUNG_MAX_DEG:
            raise BahnFehler(f"Gelenksprung {sprung:.1f}° bei {100 * k // len(posen)} % der Bahn "
                             "(andere Armstellung)")
        aus.append(q)
        q_vorher = q
    return np.array(aus)


# --- 3. Zeitoptimales Tempo entlang des Pfads --------------------------------------------------------------------


def glaetten(q: np.ndarray, sigma: float = GLAETTUNG_PROBEN) -> np.ndarray:
    """Gelenkpfad mit einem Gauß-Filter glätten. Knicke im Drehtempo an den Ecken und Rauschen der inversen Kinematik
    werden sonst zu Beschleunigungsspitzen. Die Enden werden punktgespiegelt fortgesetzt: Anfang und Ende bleiben exakt,
    gerade Stücke bleiben gerade. Die Abweichung vom kartesischen Pfad prüft ``planen``."""
    if sigma <= 0 or len(q) < 5:
        return q.copy()
    r = min(math.ceil(3 * sigma), len(q) - 1)  # Spiegelung braucht r < Länge (kurze Bahnen: schmaler Kern)
    kern = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    kern /= kern.sum()
    vorn = 2 * q[0] - q[r:0:-1]
    hinten = 2 * q[-1] - q[-2:-r - 2:-1]
    lang = np.concatenate([vorn, q, hinten])
    aus = np.stack([np.convolve(lang[:, j], kern, mode="valid") for j in range(q.shape[1])], axis=1)
    aus[0], aus[-1] = q[0], q[-1]
    return aus


def _u_grenzen(q1: np.ndarray, q2: np.ndarray, a: np.ndarray, x: float) -> tuple[float, float]:
    """Zulässige Bahnbeschleunigung u = s̈ bei ṡ² = x: |q′·u + q″·x| ≤ a je Gelenk. (lo > hi: unmöglich.)"""
    lo, hi = -math.inf, math.inf
    for j in range(len(a)):
        if abs(q1[j]) < 1e-12:
            if abs(q2[j]) * x > a[j]:
                return 1.0, -1.0
            continue
        g1 = (-a[j] - q2[j] * x) / q1[j]
        g2 = (a[j] - q2[j] * x) / q1[j]
        lo, hi = max(lo, min(g1, g2)), min(hi, max(g1, g2))
    return lo, hi


def _groesstes_x(bedingung, x_oben: float) -> float:
    """Größtes x in [0, x_oben] mit bedingung(x) (die Menge ist ein Intervall ab 0)."""
    if bedingung(x_oben):
        return x_oben
    lo, hi = 0.0, x_oben
    for _ in range(40):
        mitte = 0.5 * (lo + hi)
        lo, hi = (mitte, hi) if bedingung(mitte) else (lo, mitte)
    return lo


def zeitplan(q: np.ndarray, v: np.ndarray, a: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Zeitoptimales ṡ entlang des Gelenkpfads (Index s = 0 … N−1, Ruhe am Anfang und Ende).
    Liefert (Zeiten t_k, ṡ_k, s̈ auf dem Stück k → k+1)."""
    n = len(q)
    q1 = np.gradient(q, axis=0)
    q2 = np.gradient(q1, axis=0)
    x_max = np.empty(n)
    for k in range(n):
        mit = np.abs(q1[k]) > 1e-12
        x_v = float(np.min((v[mit] / np.abs(q1[k][mit])) ** 2)) if mit.any() else 1e6
        def moeglich(x, k=k):
            lo, hi = _u_grenzen(q1[k], q2[k], a, x)
            return lo <= hi
        x_max[k] = _groesstes_x(moeglich, x_v)
    x_max[0] = x_max[-1] = 0.0
    zurueck = x_max.copy()
    for k in range(n - 2, -1, -1):  # bremsen können: x_k + 2·u_min(x_k) ≤ x_{k+1}
        def bremsbar(x, k=k):
            lo, hi = _u_grenzen(q1[k], q2[k], a, x)
            return lo <= hi and x + 2 * lo <= zurueck[k + 1] + 1e-12
        zurueck[k] = _groesstes_x(bremsbar, x_max[k])
    x = np.zeros(n)
    for k in range(n - 1):  # so schnell beschleunigen wie erlaubt
        _, hi = _u_grenzen(q1[k], q2[k], a, x[k])
        x[k + 1] = max(0.0, min(zurueck[k + 1], x[k] + 2 * hi))
    sd = np.sqrt(x)
    dt = 2.0 / np.maximum(sd[:-1] + sd[1:], 1e-9)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    u = (x[1:] - x[:-1]) / 2.0
    return t, sd, u


def _b_spline(q: np.ndarray, s: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Gleichmäßiger kubischer B-Spline mit den Pfadpunkten q als Kontrollpunkten (Abstand 1 in s): Wert, 1. und 2.
    Ableitung nach s. Anders als eine Catmull-Rom-Kurve ist er zweimal stetig differenzierbar: Die Beschleunigung springt
    an den Pfadpunkten nicht (sonst Ruckspitzen von 18 °/s² je ms, gerechnet 12.11.2026). Die Enden werden
    punktgespiegelt fortgesetzt: Die Kurve beginnt und endet genau im ersten bzw. letzten Punkt. Zwischen den Enden liegt
    sie um (q[i−1] − 2·q[i] + q[i+1]) / 6 neben den Punkten (glatte Pfade: weit unter 0,1 mm)."""
    n = len(q)
    p = np.concatenate([2 * q[:1] - q[1:2], q, 2 * q[-1:] - q[-2:-1]])  # p[i + 1] = q[i]
    i = np.clip(np.floor(s).astype(int), 0, n - 2)
    f = (s - i)[:, None]
    p0, p1, p2, p3 = p[i], p[i + 1], p[i + 2], p[i + 3]
    g = 1 - f
    wert = (g ** 3 * p0 + (3 * f ** 3 - 6 * f ** 2 + 4) * p1 + (-3 * f ** 3 + 3 * f ** 2 + 3 * f + 1) * p2
            + f ** 3 * p3) / 6
    erste = (-3 * g ** 2 * p0 + (9 * f ** 2 - 12 * f) * p1 + (-9 * f ** 2 + 6 * f + 3) * p2 + 3 * f ** 2 * p3) / 6
    zweite = g * p0 + (3 * f - 2) * p1 + (1 - 3 * f) * p2 + f * p3
    return wert, erste, zweite


def abtasten(q: np.ndarray, t: np.ndarray, sd: np.ndarray, u: np.ndarray, dehnung: float = 1.0,
             rampe_s: float = 0.0):
    """Proben im 1-ms-Takt: (Zeit, Winkel, Tempo, Beschleunigung). dehnung > 1 verlangsamt gleichmäßig. rampe_s > 0
    glättet vorher den Fortschritt s(t) (Ruckbegrenzung, Schritt 4 im Kopf)."""
    t = t * dehnung
    sd = sd / dehnung
    u = u / dehnung ** 2
    zeit = np.arange(math.ceil(t[-1] / TAKT_S - 1e-9) + 1) * TAKT_S  # genau 1 ms Abstand, letzte Probe ≥ Ende
    k = np.clip(np.searchsorted(t, zeit, side="right") - 1, 0, len(t) - 2)
    tau = np.minimum(zeit, t[-1]) - t[k]
    s = np.minimum(k + sd[k] * tau + 0.5 * u[k] * tau ** 2, len(q) - 1)
    s_d = np.maximum(sd[k] + u[k] * tau, 0.0)
    s_dd = u[k].copy()
    s[-1], s_d[-1], s_dd[-1] = len(q) - 1, 0.0, 0.0
    if rampe_s >= 2 * TAKT_S:
        zeit, s, s_d, s_dd = (y[:, 0] if y.ndim == 2 else y for y in
                              ruck_begrenzen(s[:, None], s_d[:, None], s_dd[:, None], rampe_s))
    wert, erste, zweite = _b_spline(q, s)
    wert[0], wert[-1] = q[0], q[-1]
    tempo = erste * s_d[:, None]
    beschl = zweite * (s_d ** 2)[:, None] + erste * s_dd[:, None]
    tempo[0] = tempo[-1] = 0.0
    return zeit, wert, tempo, beschl


def rampe_fuer(a_deg_s2: np.ndarray, ruck_deg_s3: float = RUCK_GRENZE_DEG_S3) -> float:
    """Rampe, mit der der Ruck bei der Beschleunigungsgrenze a höchstens ``ruck_deg_s3`` beträgt (2 · a / Ruck)."""
    return min(RAMPE_MAX_S, max(RAMPE_MIN_S, 2.0 * float(np.max(a_deg_s2)) / ruck_deg_s3))


def ruck_begrenzen(x: np.ndarray, x_d: np.ndarray, x_dd: np.ndarray, rampe_s: float):
    """Gleitender Mittelwert über ``rampe_s`` (Schritt 4 im Kopf) für eine Größe x (M × k, 1-ms-Takt) mit ihrer ersten
    und zweiten Ableitung. Vorher und nachher steht der Arm: x wird mit Start- und Endwert, die Ableitungen mit 0
    fortgesetzt. Probe k mittelt die Eingaben k − n … k − 1, die erste Probe ist damit genau der Stillstand am Start, die
    letzte genau der am Ende. Mittelwerte der Ableitungen sind die Ableitungen des Mittelwerts, alles passt zueinander.
    Liefert (Zeit, x, x_d, x_dd), n + 1 Proben länger."""
    n = round(rampe_s / TAKT_S)
    if n < 2:
        return np.arange(len(x)) * TAKT_S, x, x_d, x_dd

    def mittel(y: np.ndarray, vorn: np.ndarray, hinten: np.ndarray) -> np.ndarray:
        lang = np.concatenate([np.repeat(vorn[None], n, axis=0), y, np.repeat(hinten[None], n, axis=0)])
        summe = np.concatenate([np.zeros((1, y.shape[1])), np.cumsum(lang, axis=0)])
        return (summe[n:] - summe[:-n]) / n

    null = np.zeros(x.shape[1])
    g = mittel(x, x[0], x[-1])
    g[0], g[-1] = x[0], x[-1]  # exakt (Summen runden)
    g_d = mittel(x_d, null, null)
    g_dd = mittel(x_dd, null, null)
    g_d[0] = g_d[-1] = g_dd[0] = g_dd[-1] = 0.0
    return np.arange(len(g)) * TAKT_S, g, g_d, g_dd


def planen(posen: list[np.ndarray], q_start_deg, werkzeug: np.ndarray, t_gemessen: np.ndarray,
           v_faktor: float = 0.8, a_deg_s2: float | None = None, rundung_m: float = 0.03,
           rampe_s: float | None = None) -> Bahn:
    """Bahn von der jetzigen Stellung durch alle ``posen`` (die erste ist die jetzige Werkzeugpose).
    v_faktor: Anteil der Tempogrenze der Gelenke. a_deg_s2: Beschleunigungsgrenze aller Gelenke (Standard: weiche Grenze
    40 °/s², mal 0,9). rampe_s: Anfahr- und Bremsrampe der Beschleunigung (Standard: ``rampe_fuer``, Ruck ≤ 400 °/s³;
    0 = ohne Rampe, nur zum Vergleich)."""
    v = GELENK_V_GRENZE_DEG_S * min(1.0, max(0.05, v_faktor))
    a = (GELENK_A_GRENZE_DEG_S2 * 0.9 if a_deg_s2 is None
         else np.minimum(np.full(7, float(a_deg_s2)), GELENK_A_HART_DEG_S2))
    dicht = pfad(posen, rundung_m)
    if len(dicht) < 3:
        raise BahnFehler("Bahn zu kurz")
    q_roh = gelenkpfad(dicht, q_start_deg, werkzeug, t_gemessen)
    q = glaetten(q_roh)
    abw = max(float(np.linalg.norm(kin.vorwaerts(qa, werkzeug)["werkzeug"][:3, 3]
                                   - kin.vorwaerts(qb, werkzeug)["werkzeug"][:3, 3]))
              for qa, qb in zip(q[::3], q_roh[::3]))
    if abw > GLAETTUNG_MAX_M:
        raise BahnFehler(f"geglättete Bahn weicht {abw * 1000:.1f} mm vom Pfad ab")
    t, sd, u = zeitplan(q, v, a)
    rampe = rampe_fuer(a) if rampe_s is None else max(0.0, float(rampe_s))
    rampe = rampe if rampe >= 2 * TAKT_S else 0.0
    dehnung = 1.0
    for _ in range(6):  # Proben gegen die Grenzen prüfen, sonst Zeit dehnen
        zeit, wert, tempo, beschl = abtasten(q, t, sd, u, dehnung, rampe)
        rv = float(np.max(np.abs(tempo) / v))
        ra = float(np.max(np.abs(beschl) / a))
        if rv <= 1.0 and ra <= 1.0:
            break
        dehnung *= max(rv, math.sqrt(ra)) * 1.02
    else:
        raise BahnFehler("Bahn hält die Gelenkgrenzen nicht ein")
    return Bahn(zeit, wert, tempo, beschl, dicht, q, v, a, rampe)


# --- Glätte-Kennwerte (Plan und Messung am Arm nach demselben Verfahren) -----------------------------------------


def _gleitend(x: np.ndarray, n: int = MESS_FENSTER) -> np.ndarray:
    """Zentrierter gleitender Mittelwert über n Proben (Achse 0), Ränder fortgesetzt."""
    if n < 2 or len(x) < n:
        return x
    r = n // 2
    lang = np.concatenate([np.repeat(x[:1], r, axis=0), x, np.repeat(x[-1:], n - 1 - r, axis=0)])
    summe = np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(lang, axis=0)])
    return (summe[n:] - summe[:-n]) / n


def kennwerte(zeit_s, pos_m, gelenke_deg=None, gelenk_tempo_deg_s=None, imu_m_s2=None) -> dict:
    """Glätte einer Fahrt aus Proben (geplant oder am Arm gemessen, beliebiger Takt): Proben auf ein 10-ms-Raster
    legen, vor jeder Ableitung über 30 ms mitteln. Werkzeug aus der Pose, Gelenke aus dem gemessenen Tempo (sonst aus
    den Winkeln), IMU am Handgelenk als Beschleunigung gegenüber der ersten Probe (enthält die Schwerkraft, solange das
    Werkzeug nicht kippt, fällt sie heraus). Das Raster glättet: Ein Beschleunigungssprung erscheint als Ruck von etwa
    Sprung / 30 ms. Geeignet zum Vergleichen (Plan gegen Messung, ReachPose gegen Gelenkbahn), nicht als Absolutwert."""
    t = np.asarray(zeit_s, float)
    aus = {"proben": len(t), "dauer_s": round(float(t[-1] - t[0]), 3) if len(t) else 0.0}
    if len(t) < 2 or t[-1] - t[0] < 5 * MESS_RASTER_S:
        return aus
    t = t - t[0]
    raster = np.arange(0.0, t[-1] + 1e-9, MESS_RASTER_S)
    dt = MESS_RASTER_S

    def auf_raster(x) -> np.ndarray:
        x = np.asarray(x, float)
        return np.stack([np.interp(raster, t, x[:, j]) for j in range(x.shape[1])], axis=1)

    def ableiten(x: np.ndarray) -> np.ndarray:
        return np.gradient(_gleitend(x), dt, axis=0)

    def groesste_norm(x: np.ndarray) -> float:
        return float(np.max(np.linalg.norm(x, axis=1)))

    p = auf_raster(pos_m)
    v = ableiten(p)
    a = ableiten(v)
    aus.update(v_max_m_s=round(groesste_norm(v), 3), a_max_m_s2=round(groesste_norm(a), 2),
               ruck_max_m_s3=round(groesste_norm(ableiten(a)), 1))
    if gelenke_deg is not None:
        if gelenk_tempo_deg_s is not None:
            w = auf_raster(gelenk_tempo_deg_s)
        else:
            w = ableiten(auf_raster(np.degrees(np.unwrap(np.radians(np.asarray(gelenke_deg, float)), axis=0))))
        al = ableiten(w)
        aus.update(gelenk_a_max_deg_s2=round(float(np.max(np.abs(al))), 1),
                   gelenk_ruck_max_deg_s3=round(float(np.max(np.abs(ableiten(al)))), 0))
    if imu_m_s2 is not None:
        b = _gleitend(auf_raster(imu_m_s2))
        b = b - b[0]
        aus.update(imu_max_m_s2=round(groesste_norm(b), 2), imu_ruck_max_m_s3=round(groesste_norm(ableiten(b)), 1))
    return aus


def plan_kennwerte(plan: Bahn, werkzeug: np.ndarray) -> dict:
    """``kennwerte`` der geplanten Bahn (Nennmodell, alle 10 ms), zum Vergleich mit der Messung am Arm."""
    k = max(1, round(MESS_RASTER_S / TAKT_S))
    q = plan.winkel_deg[::k]
    pos = np.array([kin.vorwaerts(qq, werkzeug)["werkzeug"][:3, 3] for qq in q])
    return kennwerte(plan.zeit_s[::k], pos, q, plan.tempo_deg_s[::k])
