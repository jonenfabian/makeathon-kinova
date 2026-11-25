"""Simulation des Kinova-Arbeitsplatzes ohne Arm und ohne Kortex-API (Team-Laptops und Tests, 10.11.2026).

Start: ``python kinova/dashboard/server.py --sim`` (Bedienseite http://127.0.0.1:8080, Team-Schlüssel ``sim``).

Nachgebildet:
- **Arm:** dieselbe Klasse ``arm.Arm`` mit allen Prüfungen (Arbeitsraum, Tempo, Bahnsimulation, Zieltoleranz); statt der
  Kortex-API ein Ersatz mit denselben Aufrufen. Kartesische Fahrten laufen auf einer Geraden mit der befohlenen
  Geschwindigkeit, die Gelenke folgen per inverser Kinematik. Ist ein Bahnpunkt nicht ohne Gelenkanschlag erreichbar,
  bricht die Fahrt wie am Arm sofort mit ``CONTROL_CARTESIAN_CANNOT_START`` ab.
- **Greifer:** ganzer Hub in ``GREIFER_HUB_S``; ein Würfel zwischen den Fingern hält sie an und hängt dann im Greifer.
- **Würfel:** Kante wie ``--wuerfel-mm``, oben ein AprilTag 36h11 (Sticker 45 mm, schwarze Kante 36 mm, IDs 200–205).
  Beim Loslassen fällt ein Würfel senkrecht auf den Tisch oder auf den Würfel darunter.
- **Kameras:** Farb- und Tiefenbild per Strahlverfolgung, Kinovas Vorgabe-Intrinsik, Nennmaße der Hand-Auge-Kette.
- **Kollision:** Fingerspitzen oder Flansch im Tisch, Finger in einem Würfel oder der gehaltene Würfel im Tisch/in einem
  anderen Würfel versetzen den Arm in den Fehlerzustand. Quittieren kann nur der Betreuer (Dashboard der Simulation).

Nicht nachgebildet (Unterschiede zum echten Arbeitsplatz): Kollision der übrigen Glieder mit Würfeln, Umkippen und
Rutschen, Linsenverzeichnung, Videolatenz, Bewegungsunschärfe, Abweichung der echten Kinematik vom Nennmodell
(≈ 7 mm, siehe kinematik.py) und der Fehler der Hand-Auge-Kette. Erfolg in der Simulation ist kein Nachweis am Arm.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from types import SimpleNamespace

import bahn as bahn_mod
import kinematik as kin
import kollision
import numpy as np

TISCH_Z_M = kollision.TISCH_Z_M  # wie die Tischplatte am echten Arbeitsplatz (Tiefenbild 9.11.2026)
FARBE_INTR = {"aufloesung": "RESOLUTION_1280x720", "fx": 1297.67, "fy": 1298.63, "cx": 620.91, "cy": 238.28,
              "verzeichnung": [0.0] * 5}  # Kinovas Vorgabe (vom Arm gelesen 9.11.2026)
TIEFE_INTR = {"aufloesung": "RESOLUTION_480x270", "fx": 335.76, "fy": 335.76, "cx": 236.14, "cy": 134.15,
              "verzeichnung": [0.0] * 5}
FARBE_GROESSE = (1280, 720)
TIEFE_GROESSE = (480, 270)
TIEFE_MIN_M = 0.12  # ANNAHME: näher misst die Tiefenkamera nicht (am Arm nicht geprüft)
STICKER_M = 0.045
GREIFER_HUB_S = 0.6  # ganz offen ↔ ganz zu (Robotiq 2F-85: bis 150 mm/s laut Datenblatt)
TAKT_S = 0.02
FINGER_DICKE_M = 0.008  # ANNAHME: Fingerpolster außen an der Innenfläche
FINGER_BREITE_M = 0.022  # Robotiq-2F-85-Polster (Datenblatt: 22 mm)
FINGER_LAENGE_M = 0.035  # Bereich über dem Spitzenende, der greift
GREIF_WINKEL_DEG = 20.0  # weicht die Schließrichtung mehr ab, rutscht der Würfel aus den Fingern
FARBE_HOLZ = np.array([95.0, 160.0, 205.0])  # BGR
FARBE_TISCH = np.array([108.0, 112.0, 116.0])


class SimFehler(Exception):
    """Ersatz für die Kortex-Ausnahmen (wird in der Simulation nicht ausgelöst)."""


# --- Ersatz für die benutzten Kortex-Nachrichten ----------------------------------------------------------------


class _Nachricht:
    """Kleiner Ersatz für eine protobuf-Nachricht: Unterfelder entstehen beim ersten Zugriff, Listen über ``add()``."""

    def __init__(self, **werte):
        self.__dict__["_liste"] = []
        self.__dict__.update(werte)

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        neu = _Nachricht()
        self.__dict__[name] = neu
        return neu

    def add(self, **werte) -> _Nachricht:
        n = _Nachricht(**werte)
        self._liste.append(n)
        return n

    def __iter__(self):
        return iter(self._liste)

    def __len__(self) -> int:
        return len(self._liste)

    def __getitem__(self, i):
        return self._liste[i]

    def __bool__(self) -> bool:
        return bool(self._liste) or len(self.__dict__) > 1

    def HasField(self, name: str) -> bool:
        return name in self.__dict__


class _Aufzaehlung:
    def __init__(self, **werte):
        self._werte = werte
        self._namen = {v: k for k, v in werte.items()}

    def Name(self, wert) -> str:
        return self._namen.get(int(wert), str(wert))

    def Value(self, name: str) -> int:
        return self._werte[name]


ARMZUSTAENDE = {"ARMSTATE_BASE_INITIALIZATION": 0, "ARMSTATE_IDLE": 1, "ARMSTATE_ARM_INITIALIZATION": 2,
                "ARMSTATE_IN_FAULT": 3, "ARMSTATE_MAINTENANCE": 4, "ARMSTATE_SERVOING_LOW_LEVEL": 5,
                "ARMSTATE_SERVOING_READY": 6, "ARMSTATE_SERVOING_PLAYING_SEQUENCE": 7,
                "ARMSTATE_SERVOING_MANUALLY_CONTROLLED": 8}
EREIGNISSE = {"ACTION_END": 0, "ACTION_ABORT": 1, "ACTION_PAUSE": 2, "ACTION_START": 3, "ACTION_PREPROCESS_ABORT": 4}
BAHNFEHLER = {"TRAJECTORY_ERROR_TYPE_UNSPECIFIED": 0, "TRAJECTORY_ERROR_TYPE_LARGE_SPEED": 1,
              "TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION": 2, "TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP": 3,
              "TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT": 4, "TRAJECTORY_ERROR_TYPE_INVALID_POSITION": 5}
BAHNGROESSEN = {"TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED": 0, "TRAJECTORY_ERROR_IDENTIFIER_TIME": 1,
                "TRAJECTORY_ERROR_IDENTIFIER_POSITION": 2, "TRAJECTORY_ERROR_IDENTIFIER_VELOCITY": 3,
                "TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION": 4}
UNTERFEHLER = {"SUB_ERROR_NONE": 0, "CONTROL_CARTESIAN_CANNOT_START": 1, "SIMULATION_KOLLISION": 2,
               "SIMULATION_STOPP": 3}
B2 = SimpleNamespace(
    **ARMZUSTAENDE, **EREIGNISSE, **UNTERFEHLER,
    ArmState=_Aufzaehlung(**ARMZUSTAENDE), ActionEvent=_Aufzaehlung(**EREIGNISSE),
    SubErrorCodes=_Aufzaehlung(**UNTERFEHLER),
    SINGLE_LEVEL_SERVOING=2, GRIPPER_SPEED=2, GRIPPER_POSITION=3, JOINT_CONSTRAINT_DURATION=1,
    TRAJECTORY_CONTINUITY_MODE_POSITION=1,
    TrajectoryErrorType=_Aufzaehlung(**BAHNFEHLER), TrajectoryErrorIdentifier=_Aufzaehlung(**BAHNGROESSEN),
    Action=_Nachricht, ServoingModeInformation=_Nachricht, NotificationOptions=_Nachricht,
    GripperCommand=_Nachricht, GripperRequest=_Nachricht, PreComputedJointTrajectory=_Nachricht)


def kortex_sim(welt: SimWelt) -> dict:
    """Dasselbe Wörterbuch wie ``kinova_lesen._kortex()``, soweit ``arm.Arm`` es benutzt."""

    class Base:
        def __init__(self, router):
            pass

        def GetServoingMode(self, options=None):
            return _Nachricht(servoing_mode=B2.SINGLE_LEVEL_SERVOING)

        def SetServoingMode(self, info, options=None):
            pass

        def OnNotificationActionTopic(self, rueckruf, optionen, options=None):
            return welt.abonnieren(rueckruf)

        def Unsubscribe(self, griff, options=None):
            welt.abbestellen(griff)

        def ExecuteAction(self, aktion, options=None):
            welt.aktion(aktion)

        def PlayPreComputedJointTrajectory(self, spur, options=None):
            welt.bahn(spur)

        def GetTrajectoryErrorReport(self, options=None):
            return welt.bahn_bericht

        def Stop(self, options=None):
            welt.anhalten()

        def ClearFaults(self, options=None):
            welt.quittieren()

        def SendGripperCommand(self, befehl, options=None):
            welt.greifer_befehl(befehl.mode, float(befehl.gripper.finger[0].value))

        def GetMeasuredGripperMovement(self, anfrage, options=None):
            g = _Nachricht()
            g.finger.add(finger_identifier=1, value=welt.greifer)
            return g

    class Zyklisch:
        def __init__(self, router):
            pass

        def RefreshFeedback(self, options=None):
            return welt.rueckmeldung()

    class Steuerung:
        def __init__(self, router):
            pass

        def GetToolConfiguration(self, options=None):
            t = _Nachricht()
            p = kin.Pose.aus_matrix(welt.werkzeug)
            t.tool_transform = _Nachricht(x=p.x, y=p.y, z=p.z, theta_x=p.theta_x, theta_y=p.theta_y,
                                          theta_z=p.theta_z)
            return t

    return {"Base_pb2": B2, "klassen": {"base": Base, "base_cyclic": Zyklisch, "control_config": Steuerung},
            "RouterClientSendOptions": _Nachricht, "fehlerarten": (SimFehler,)}


# --- Welt ---------------------------------------------------------------------------------------------------------


@dataclass
class SimWuerfel:
    id: int
    mitte: np.ndarray  # Mittelpunkt in der Basis (m)
    kante_m: float
    r: np.ndarray = field(default_factory=lambda: np.eye(3))  # Lage (z oben, solange er nicht im Greifer hängt)

    @property
    def gier_deg(self) -> float:
        return math.degrees(math.atan2(self.r[1, 0], self.r[0, 0]))

    def matrix(self) -> np.ndarray:
        return kin.transform(self.r, self.mitte)

    def enthaelt(self, p: np.ndarray, rand_m: float = 0.001) -> np.ndarray:
        """Welche Punkte (N×3) liegen im Würfel (um ``rand_m`` verkleinert)?"""
        lokal = (np.atleast_2d(p) - self.mitte) @ self.r
        return np.all(np.abs(lokal) < self.kante_m / 2 - rand_m, axis=1)

    def als_dict(self) -> dict:
        return {"id": self.id, "mitte_mm": [round(float(v) * 1000, 1) for v in self.mitte],
                "gier_deg": round(self.gier_deg, 1), "kante_mm": round(self.kante_m * 1000, 1)}


def _gier(r: np.ndarray) -> np.ndarray:
    return kin.rot_z(math.atan2(r[1, 0], r[0, 0]))


def start_szene(kante_m: float = 0.070) -> list[SimWuerfel]:
    """Sechs Würfel (Tag-IDs 200–205) vor dem Arm, alle Tags aus der Home-Pose im Farbbild. Blickfeld dort in
    Höhe der Würfeloberseiten etwa x 0,33–0,53 m, y −0,18…0,17 m (Hauptpunkt cy = 238 px liegt hoch im Bild)."""
    orte = [(0.37, -0.15, 12.0), (0.37, 0.00, -8.0), (0.37, 0.15, 25.0),
            (0.48, -0.15, -20.0), (0.48, 0.00, 5.0), (0.48, 0.15, 33.0)]
    return [SimWuerfel(200 + i, np.array([x, y, TISCH_Z_M + kante_m / 2]), kante_m, kin.rot_z(math.radians(g)))
            for i, (x, y, g) in enumerate(orte)]


def zufalls_szene(seed: int, anzahl: int = 6, kante_m: float = 0.070) -> list[SimWuerfel]:
    """Zufällige Lagen im Bereich x 0,33–0,55 m, y −0,22…0,22 m, Mitten mindestens 1,6 Kanten auseinander (Platz für die
    Finger). Nicht alle Tags sind dann aus der Home-Pose zu sehen."""
    if not 1 <= anzahl <= 8:
        raise ValueError("1 bis 8 Würfel")
    rng = np.random.default_rng(seed)
    wuerfel: list[SimWuerfel] = []
    for _ in range(20000):
        if len(wuerfel) == anzahl:
            break
        x, y = rng.uniform(0.33, 0.55), rng.uniform(-0.22, 0.22)
        if all(math.hypot(x - w.mitte[0], y - w.mitte[1]) > 1.6 * kante_m for w in wuerfel):
            wuerfel.append(SimWuerfel(200 + len(wuerfel), np.array([x, y, TISCH_Z_M + kante_m / 2]), kante_m,
                                      kin.rot_z(math.radians(rng.uniform(-45, 45)))))
    if len(wuerfel) < anzahl:
        raise ValueError(f"{anzahl} Würfel passen nicht nebeneinander")
    return wuerfel


def _drehung_zwischen(r0: np.ndarray, r1: np.ndarray, f: float) -> np.ndarray:
    import cv2

    rvec, _ = cv2.Rodrigues(r0.T @ r1)
    return r0 @ cv2.Rodrigues(rvec * f)[0]


def _winkel_diff(a, b) -> np.ndarray:
    return (np.asarray(b, dtype=float) - np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


class SimWelt:
    """Gemeinsamer Zustand: Arm, Greifer, Würfel. Ein Taktgeber-Thread rechnet Bewegungen fort."""

    def __init__(self, gelenke_deg, wuerfel: list[SimWuerfel], zeitraffer: float = 1.0,
                 werkzeug: np.ndarray = kin.WERKZEUG_STANDARD):
        self.sperre = threading.RLock()
        self.werkzeug = werkzeug
        self.zeitraffer = max(0.1, float(zeitraffer))
        self.gelenke = np.asarray(gelenke_deg, dtype=float) % 360.0
        self.pose = kin.vorwaerts(self.gelenke, werkzeug)["werkzeug"]
        self.greifer = 0.0
        self._greifer_befehl: tuple[str, float] | None = None
        self.fehler = 0
        self.meldung: str | None = None
        self.wuerfel = list(wuerfel)
        self.gehalten: tuple[SimWuerfel, np.ndarray, float] | None = None  # Würfel, T_ee_würfel, Weite
        self._abos: dict[int, object] = {}
        self._fahrt: dict | None = None
        # Vorberechnete Bahnen: Grenzen, gegen die die Steuerung prüft (ANNAHME wie bahn.py: weiche Grenze 40 °/s²)
        self.bahn_v_grenze = bahn_mod.GELENK_V_GRENZE_DEG_S.copy()
        self.bahn_a_grenze = bahn_mod.GELENK_A_GRENZE_DEG_S2.copy()
        self.bahn_bericht = _Nachricht()
        # so viele Aktionen/Bahnen enden sofort mit ACTION_END, ohne dass sich der Arm bewegt (am Arm 9. und 12.11.2026
        # direkt nach einem Greiferbefehl, der am Würfel stehen blieb); für Tests
        self.leer_beenden = 0
        self._geduldet: str | None = None  # Kollision beim Quittieren: gilt bis zum Herausfahren nicht erneut
        self._halt = threading.Event()
        self._thread: threading.Thread | None = None

    # --- Taktgeber ----------------------------------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._schleife, name="sim-takt", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._halt.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _schleife(self) -> None:
        letzte = time.monotonic()
        while not self._halt.wait(TAKT_S):
            jetzt = time.monotonic()
            self.takt(min(0.1, jetzt - letzte) * self.zeitraffer)
            letzte = jetzt

    def takt(self, dt: float) -> None:
        ereignisse = []
        with self.sperre:
            lief = self._fahrt is not None
            fertig = lief and self._fahrt_takt(dt)
            self._greifer_takt(dt)
            self._gehaltenen_mitfuehren()
            kollision = None if self.fehler else self._kollision_pruefen()
            if kollision is None:
                self._geduldet = None
            elif kollision == self._geduldet:
                kollision = None
            if kollision is not None:
                # Arm bleibt stehen und meldet einen Fehler (wie die Kollisionserkennung am Arm), auch wenn die Fahrt
                # in diesem Takt ihr Ziel erreicht hätte
                self._fahrt = None
                self._greifer_befehl = None
                self.fehler = 1
                self.meldung = f"Kollision: {kollision}"
                if lief:
                    ereignisse.append((B2.ACTION_ABORT, B2.SIMULATION_KOLLISION))
            elif fertig:
                ereignisse.append((B2.ACTION_END, 0))
        self._melden(ereignisse)

    # --- Meldungen an arm.Arm -------------------------------------------------------------------------------
    def abonnieren(self, rueckruf):
        with self.sperre:
            griff = _Nachricht(identifier=max(self._abos, default=0) + 1)
            self._abos[griff.identifier] = rueckruf
            return griff

    def abbestellen(self, griff) -> None:
        with self.sperre:
            self._abos.pop(griff.identifier, None)

    def _melden(self, ereignisse) -> None:
        for ereignis, details in ereignisse:
            n = _Nachricht(action_event=ereignis, abort_details=details)
            for rueckruf in list(self._abos.values()):
                rueckruf(n)

    def _spaeter_melden(self, ereignis: int, details: int = 0) -> None:
        threading.Timer(0.01, self._melden, args=([(ereignis, details)],)).start()

    # --- Zustand --------------------------------------------------------------------------------------------
    def rueckmeldung(self) -> _Nachricht:
        with self.sperre:
            p = kin.Pose.aus_matrix(self.pose)
            fb = _Nachricht()
            fb.base = _Nachricht(tool_pose_x=p.x, tool_pose_y=p.y, tool_pose_z=p.z, tool_pose_theta_x=p.theta_x,
                                 tool_pose_theta_y=p.theta_y, tool_pose_theta_z=p.theta_z,
                                 active_state=B2.ARMSTATE_IN_FAULT if self.fehler else B2.ARMSTATE_SERVOING_READY,
                                 fault_bank_a=self.fehler, fault_bank_b=0, arm_voltage=24.0)
            for g in self.gelenke:
                fb.actuators.add(position=float(g))
            fb.interconnect.gripper_feedback.motor.add(position=self.greifer * 100.0)
            return fb

    def t_ee(self) -> np.ndarray:
        return self.pose @ kin.invers(self.werkzeug)

    def schnappschuss(self) -> tuple[np.ndarray, list[tuple[int, np.ndarray, float]]]:
        """Armpose und Würfel (ID, Matrix, Kante) für die Kameras."""
        with self.sperre:
            return self.pose.copy(), [(w.id, w.matrix(), w.kante_m) for w in self.wuerfel]

    def info(self, wahrheit: bool) -> dict:
        with self.sperre:
            d = {"meldung": self.meldung, "zeitraffer": self.zeitraffer}
            if wahrheit:
                d["wuerfel"] = [w.als_dict() for w in self.wuerfel]
                d["gehalten"] = None if self.gehalten is None else self.gehalten[0].id
            return d

    # --- Fahrten --------------------------------------------------------------------------------------------
    def aktion(self, aktion) -> None:
        with self.sperre:
            if self.fehler:
                self._spaeter_melden(B2.ACTION_ABORT, B2.SIMULATION_KOLLISION)
                return
            if self.leer_beenden > 0:
                self.leer_beenden -= 1
                self._spaeter_melden(B2.ACTION_END)
                return
            if aktion.HasField("reach_pose"):
                fahrt = self._pose_planen(aktion.reach_pose)
            elif aktion.HasField("reach_joint_angles"):
                fahrt = self._gelenke_planen(aktion.reach_joint_angles)
            else:
                fahrt = None
            if fahrt is None:
                self._spaeter_melden(B2.ACTION_ABORT, B2.CONTROL_CARTESIAN_CANNOT_START)
                return
            self._fahrt = fahrt
            self.meldung = None

    def bahn(self, spur) -> None:
        """PlayPreComputedJointTrajectory: prüfen wie die Steuerung (Start = jetzige Stellung, 1-ms-Takt, Tempo und
        Beschleunigung in den Grenzen, keine Sprünge), sonst ACTION_PREPROCESS_ABORT mit Fehlerbericht."""
        with self.sperre:
            if self.fehler:
                self._spaeter_melden(B2.ACTION_ABORT, B2.SIMULATION_KOLLISION)
                return
            if self.leer_beenden > 0:
                self.leer_beenden -= 1
                self._spaeter_melden(B2.ACTION_END)
                return
            el = list(spur.trajectory_elements)
            zeit = np.array([float(e.time_from_start) for e in el])
            q = np.array([list(e.joint_angles) for e in el], dtype=float)
            v = np.array([list(e.joint_speeds) for e in el], dtype=float)
            a = np.array([list(e.joint_accelerations) for e in el], dtype=float)
            fehler = []

            def melden(art, groesse, index, wert, lo, hi):
                fehler.append({"error_type": BAHNFEHLER[art], "error_identifier": BAHNGROESSEN[groesse],
                               "index": int(index), "error_value": float(wert), "min_value": float(lo),
                               "max_value": float(hi), "message": ""})

            start = float(np.max(np.abs(_winkel_diff(self.gelenke, q[0] % 360.0)))) if len(q) else 0.0
            if len(q) < 2 or abs(zeit[0]) > 1e-9 or start > 0.5:
                melden("TRAJECTORY_ERROR_TYPE_WRONG_STARTING_POINT", "TRAJECTORY_ERROR_IDENTIFIER_POSITION",
                       0, start, 0.0, 0.5)
            elif np.any(np.abs(np.diff(zeit) - 0.001) > 1e-6):
                melden("TRAJECTORY_ERROR_TYPE_INVALID_TIME_STEP", "TRAJECTORY_ERROR_IDENTIFIER_TIME",
                       0, float(np.max(np.diff(zeit))), 0.001, 0.001)
            else:
                for j in range(7):
                    if np.max(np.abs(v[:, j])) > self.bahn_v_grenze[j] * 1.001:
                        melden("TRAJECTORY_ERROR_TYPE_LARGE_SPEED", "TRAJECTORY_ERROR_IDENTIFIER_VELOCITY", j,
                               np.max(np.abs(v[:, j])), -self.bahn_v_grenze[j], self.bahn_v_grenze[j])
                    if np.max(np.abs(a[:, j])) > self.bahn_a_grenze[j] * 1.001:
                        melden("TRAJECTORY_ERROR_TYPE_INVALID_ACCELERATION", "TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION",
                               j, np.max(np.abs(a[:, j])), -self.bahn_a_grenze[j], self.bahn_a_grenze[j])
                    sprung = float(np.max(np.abs(np.diff(q[:, j]))))
                    if sprung > self.bahn_v_grenze[j] * 0.001 * 1.5:
                        melden("TRAJECTORY_ERROR_TYPE_INVALID_POSITION", "TRAJECTORY_ERROR_IDENTIFIER_POSITION", j,
                               sprung, 0.0, self.bahn_v_grenze[j] * 0.001)
            self.bahn_bericht = _Nachricht()
            for f in fehler:
                self.bahn_bericht.trajectory_error_elements.add(**f)
            if fehler:
                self._spaeter_melden(B2.ACTION_PREPROCESS_ABORT)
                return
            self._fahrt = {"art": "bahn", "pfad": [g % 360.0 for g in q], "dauer": max(0.001, float(zeit[-1])),
                           "zeit": 0.0}
            self.meldung = None

    def _pose_planen(self, rp) -> dict | None:
        z = rp.target_pose
        ziel = kin.Pose(z.x, z.y, z.z, z.theta_x, z.theta_y, z.theta_z).matrix()
        start = self.pose.copy()
        v = max(1e-3, float(rp.constraint.speed.translation or 0.05))
        w = max(1e-3, float(rp.constraint.speed.orientation or 15.0))
        w = min(w, math.degrees(v))  # wie Firmware 2.2.0: Drehtempo in rad/s ≈ Wegtempo in m/s (arm.DREH_RAD_S_JE_M_S)
        weg = float(np.linalg.norm(ziel[:3, 3] - start[:3, 3]))
        dreh = kin.winkel_abstand_deg(start[:3, :3], ziel[:3, :3])
        dauer = max(0.05, weg / v, dreh / w)
        n = max(4, math.ceil(dauer / 0.05))
        q = self.gelenke.copy()
        pfad = [q]
        for i in range(1, n + 1):
            t = self._zwischen(start, ziel, i / n)
            q_neu, fehler_m, fehler_deg = kin.inverse_kinematik(t, q, self.werkzeug, iterationen=80)
            reserve = min(kin.anschlag_reserven(q_neu).values())
            sprung = float(np.max(np.abs(_winkel_diff(q, q_neu))))
            if fehler_m > 0.002 or fehler_deg > 0.3 or reserve < 0.5 or sprung > 20.0:
                grund = (f"Gelenkanschlag ({reserve:.1f}° Reserve)" if reserve < 0.5 else
                         f"Gelenksprung {sprung:.0f}°" if sprung > 20.0 else "Ziel nicht erreichbar")
                self.meldung = f"Fahrt abgelehnt bei {100 * i // n} % der Bahn: {grund}"
                return None
            pfad.append(q_neu)
            q = q_neu
        return {"art": "pose", "start": start, "ziel": ziel, "pfad": pfad, "dauer": dauer, "zeit": 0.0}

    def _gelenke_planen(self, rj) -> dict:
        ziel = [float(g.value) for g in rj.joint_angles.joint_angles]
        dauer = float(rj.constraint.value or 3.0)
        pfad = [np.asarray(q, dtype=float) % 360.0 for q in kollision.gelenk_bahn(list(self.gelenke), ziel, 60)]
        return {"art": "gelenke", "pfad": pfad, "dauer": max(0.05, dauer), "zeit": 0.0}

    @staticmethod
    def _zwischen(start: np.ndarray, ziel: np.ndarray, f: float) -> np.ndarray:
        return kin.transform(_drehung_zwischen(start[:3, :3], ziel[:3, :3], f),
                             start[:3, 3] + f * (ziel[:3, 3] - start[:3, 3]))

    @staticmethod
    def _pfad_bei(pfad: list[np.ndarray], f: float) -> np.ndarray:
        x = f * (len(pfad) - 1)
        i = min(len(pfad) - 2, int(x))
        return (pfad[i] + (x - i) * _winkel_diff(pfad[i], pfad[i + 1])) % 360.0

    def _fahrt_takt(self, dt: float) -> bool:
        """Fahrt fortschreiben; True, wenn sie in diesem Takt ihr Ziel erreicht."""
        fahrt = self._fahrt
        fahrt["zeit"] += dt
        f = min(1.0, fahrt["zeit"] / fahrt["dauer"])
        self.gelenke = self._pfad_bei(fahrt["pfad"], f)
        if fahrt["art"] == "pose":
            self.pose = fahrt["ziel"].copy() if f >= 1.0 else self._zwischen(fahrt["start"], fahrt["ziel"], f)
        else:
            self.pose = kin.vorwaerts(self.gelenke, self.werkzeug)["werkzeug"]
        if f >= 1.0:
            self._fahrt = None
            return True
        return False

    def anhalten(self) -> None:
        with self.sperre:
            lief = self._fahrt is not None
            self._fahrt = None
            if self._greifer_befehl is not None and self._greifer_befehl[0] == "tempo":
                self._greifer_befehl = None
        if lief:
            self._spaeter_melden(B2.ACTION_ABORT, B2.SIMULATION_STOPP)

    def quittieren(self) -> None:
        """Fehler zurücksetzen. Steckt der Greifer noch im Hindernis, darf er erst einmal herausfahren."""
        with self.sperre:
            self.fehler = 0
            self.meldung = None
            self._geduldet = self._kollision_pruefen()

    # --- Greifer --------------------------------------------------------------------------------------------
    def greifer_befehl(self, modus: int, wert: float) -> None:
        with self.sperre:
            if self.fehler:
                return
            if modus == B2.GRIPPER_POSITION:
                self._greifer_befehl = ("position", min(1.0, max(0.0, wert)))
            elif modus == B2.GRIPPER_SPEED:
                self._greifer_befehl = None if wert == 0 else ("tempo", max(-1.0, min(1.0, wert)))

    def _finger_punkte(self, position: float | None = None) -> np.ndarray:
        """Stützpunkte beider Fingerpolster in der Basis (Polster als Quader, 3 × 3 × 3 Punkte je Finger)."""
        p = self.greifer if position is None else position
        halb = kin.greifer_weite_m(p) / 2
        unten = kin.fingerspitze_unter_flansch_m(p)
        xs = np.linspace(halb, halb + FINGER_DICKE_M, 3)
        ys = np.linspace(-FINGER_BREITE_M / 2, FINGER_BREITE_M / 2, 3)
        zs = np.linspace(unten - FINGER_LAENGE_M, unten, 3)
        gitter = np.array([(x, y, z) for x in xs for y in ys for z in zs])
        beide = np.vstack([gitter, gitter * np.array([-1.0, 1.0, 1.0])]) + (kin.GREIFER_MITTE_X_M, 0.0, 0.0)
        return beide @ self.t_ee()[:3, :3].T + self.t_ee()[:3, 3]

    def _zwischen_den_fingern(self) -> tuple[SimWuerfel, float, bool] | None:
        """Würfel, der beim Schließen zwischen die Finger gerät: (Würfel, Weite beim Kontakt, sicher gegriffen)."""
        t_ee = self.t_ee()
        weite = kin.greifer_weite_m(self.greifer)
        unten = kin.fingerspitze_unter_flansch_m(self.greifer)
        ex = t_ee[:3, 0]
        if self.gehalten is not None:
            return self.gehalten[0], self.gehalten[2], True
        for w in self.wuerfel:
            c = (kin.invers(t_ee) @ np.append(w.mitte, 1.0))[:3]
            h = w.kante_m / 2
            if abs(c[1]) > h - 0.004:  # Polster müssen die Fläche treffen
                continue
            ueberlappung = min(unten, c[2] + h) - max(unten - FINGER_LAENGE_M, c[2] - h)
            if ueberlappung < 0.008:
                continue
            phi = (math.degrees(math.atan2(ex[1], ex[0])) - w.gier_deg + 45.0) % 90.0 - 45.0
            breite = w.kante_m * (abs(math.cos(math.radians(phi))) + abs(math.sin(math.radians(phi))))
            if abs(c[0] - kin.GREIFER_MITTE_X_M) + breite / 2 > weite / 2 + 0.001:
                continue
            return w, breite, abs(phi) <= GREIF_WINKEL_DEG
        return None

    def _greifer_takt(self, dt: float) -> None:
        befehl = self._greifer_befehl
        if befehl is None:
            return
        art, wert = befehl
        schritt = dt / GREIFER_HUB_S
        if art == "position":
            delta = max(-schritt, min(schritt, wert - self.greifer))
        else:
            delta = -wert * schritt  # Kortex: negativ schließt
        neu = min(1.0, max(0.0, self.greifer + delta))
        if delta > 0:
            kontakt = self._zwischen_den_fingern()
            if kontakt is not None:
                w, breite, sicher = kontakt
                grenze = kin.greifer_position_fuer_weite(breite)
                if neu >= grenze:
                    neu = max(self.greifer, grenze)
                    if self.gehalten is None and sicher:
                        self._anhaengen(w, breite)
        elif delta < 0 and self.gehalten is not None and kin.greifer_weite_m(neu) > self.gehalten[2] + 0.002:
            self._loslassen()
        self.greifer = neu
        if art == "position" and abs(neu - wert) < 1e-9:
            self._greifer_befehl = None

    def _anhaengen(self, w: SimWuerfel, breite: float) -> None:
        t_ee = self.t_ee()
        c = (kin.invers(t_ee) @ np.append(w.mitte, 1.0))[:3]
        c[0] = kin.GREIFER_MITTE_X_M  # die Finger schieben den Würfel in ihre Mitte, wie am Arm knapp neben die Achse
        w.mitte = (t_ee @ np.append(c, 1.0))[:3]
        w.r = _gier(t_ee[:3, :3])  # Kanten parallel zu den Fingern
        self.gehalten = (w, kin.invers(t_ee) @ w.matrix(), breite)

    def _gehaltenen_mitfuehren(self) -> None:
        if self.gehalten is None:
            return
        w, rel, _ = self.gehalten
        t = self.t_ee() @ rel
        w.mitte, w.r = t[:3, 3].copy(), t[:3, :3].copy()

    def _loslassen(self) -> None:
        w, _, _ = self.gehalten
        self.gehalten = None
        w.r = _gier(w.r)
        unterlage = TISCH_Z_M
        for andere in self.wuerfel:
            if andere is not w and math.hypot(*(andere.mitte[:2] - w.mitte[:2])) < 0.5 * (andere.kante_m + w.kante_m):
                unterlage = max(unterlage, andere.mitte[2] + andere.kante_m / 2)
        fall = w.mitte[2] - w.kante_m / 2 - unterlage
        w.mitte = np.array([w.mitte[0], w.mitte[1], unterlage + w.kante_m / 2])
        if fall > 0.01:
            self.meldung = f"Würfel {w.id} aus {fall * 1000:.0f} mm Höhe fallen gelassen"

    # --- Kollision ------------------------------------------------------------------------------------------
    def _kollision_pruefen(self) -> str | None:
        """Beschreibung der ersten Kollision oder None."""
        finger = self._finger_punkte()
        flansch = self.t_ee()[:3, 3]
        if np.min(finger[:, 2]) < TISCH_Z_M - 0.0005 or flansch[2] < TISCH_Z_M + 0.01:
            return "Greifer stößt auf den Tisch"
        gehalten = None if self.gehalten is None else self.gehalten[0]
        for w in self.wuerfel:
            if w is not gehalten and np.any(w.enthaelt(finger)):
                return f"Finger stößt an Würfel {w.id}"
        if gehalten is not None:
            h = gehalten.kante_m / 2
            ecken = np.array([(x, y, z) for x in (-h, h) for y in (-h, h) for z in (-h, h)])
            ecken = np.vstack([ecken, [0.0, 0.0, -h]]) @ gehalten.r.T + gehalten.mitte
            if np.min(ecken[:, 2]) < TISCH_Z_M - 0.0005:
                return f"Würfel {gehalten.id} stößt auf den Tisch"
            for w in self.wuerfel:
                if w is not gehalten and np.any(w.enthaelt(ecken)):
                    return f"Würfel {gehalten.id} stößt an Würfel {w.id}"
        return None

    # --- Szene (Team-API in der Simulation) -----------------------------------------------------------------
    def szene_setzen(self, wuerfel: list[SimWuerfel]) -> None:
        with self.sperre:
            self.gehalten = None
            self.wuerfel = wuerfel


# --- Kameras ------------------------------------------------------------------------------------------------------


def _sticker(tag_id: int, zelle_px: int = 20) -> np.ndarray:
    """Sticker 45 mm: weißer Rand (eine Zelle), schwarzer Tag-Rand und 6 × 6 Datenzellen (36 mm schwarze Kante)."""
    import cv2

    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                        tag_id, 8 * zelle_px, borderBits=1)
    return cv2.copyMakeBorder(tag, zelle_px, zelle_px, zelle_px, zelle_px, cv2.BORDER_CONSTANT, value=255)


_STICKER: dict[int, np.ndarray] = {}
_STRAHLEN: dict[tuple, np.ndarray] = {}


def _strahlen(intr: dict, groesse: tuple[int, int]) -> np.ndarray:
    schluessel = (intr["fx"], intr["fy"], intr["cx"], intr["cy"], groesse)
    if schluessel not in _STRAHLEN:
        v, u = np.mgrid[0:groesse[1], 0:groesse[0]].astype(float)
        _STRAHLEN[schluessel] = np.stack([(u - intr["cx"]) / intr["fx"], (v - intr["cy"]) / intr["fy"],
                                          np.ones_like(u)], axis=-1)
    return _STRAHLEN[schluessel]


def rendern(t_bk: np.ndarray, wuerfel, intr: dict, groesse: tuple[int, int], farbe: bool,
            seed: int = 0) -> np.ndarray:
    """Bild der Szene aus der Kamera ``t_bk`` (Basis ← optischer Kamera-Frame). farbe=True: BGR uint8, sonst Tiefe in
    mm (uint16, 0 = keine Messung). Strahl d = ((u−cx)/fx, (v−cy)/fy, 1): Strahlparameter = Kamera-z."""
    import cv2

    breite, hoehe = groesse
    r, o = t_bk[:3, :3], t_bk[:3, 3]
    d = _strahlen(intr, groesse) @ r.T
    with np.errstate(divide="ignore", invalid="ignore"):
        tiefe = np.where(d[..., 2] < -1e-9, (TISCH_Z_M - o[2]) / d[..., 2], np.inf)
    tiefe = np.where(tiefe > 0, tiefe, np.inf)
    if farbe:
        p = o + d * np.where(np.isfinite(tiefe), tiefe, 0.0)[..., None]
        linie = (np.abs((p[..., 0] + 0.025) % 0.05 - 0.025) < 0.0008) | (np.abs((p[..., 1] + 0.025) % 0.05 - 0.025)
                                                                          < 0.0008)
        bild = np.broadcast_to(FARBE_TISCH, (hoehe, breite, 3)) * np.where(linie, 0.88, 1.0)[..., None]
        bild = np.where(np.isfinite(tiefe)[..., None], bild, 60.0)
    t_kb = kin.invers(t_bk)
    for tag_id, t_w, kante in wuerfel:
        h = kante / 2
        ecken = np.array([(x, y, z, 1.0) for x in (-h, h) for y in (-h, h) for z in (-h, h)]) @ (t_kb @ t_w).T
        if np.all(ecken[:, 2] > 0.02):
            uv = ecken[:, :2] / ecken[:, 2:3] * (intr["fx"], intr["fy"]) + (intr["cx"], intr["cy"])
            x0, y0 = np.clip(np.floor(uv.min(axis=0)).astype(int) - 2, 0, (breite, hoehe))
            x1, y1 = np.clip(np.ceil(uv.max(axis=0)).astype(int) + 3, 0, (breite, hoehe))
        elif np.any(ecken[:, 2] > 0.02):
            x0, y0, x1, y1 = 0, 0, breite, hoehe
        else:
            continue
        if x1 <= x0 or y1 <= y0:
            continue
        rw, c = t_w[:3, :3], t_w[:3, 3]
        dl = d[y0:y1, x0:x1] @ rw
        ol = rw.T @ (o - c)
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-h - ol) / dl
            t2 = (h - ol) / dl
        t1 = np.where(np.isnan(t1), -np.inf, t1)
        t2 = np.where(np.isnan(t2), np.inf, t2)
        nah = np.minimum(t1, t2)
        fern = np.maximum(t1, t2)
        t_nah, t_fern = nah.max(axis=-1), fern.min(axis=-1)
        treffer = (t_fern >= t_nah) & (t_nah > 0) & (t_nah < tiefe[y0:y1, x0:x1])
        if not np.any(treffer):
            continue
        tiefe[y0:y1, x0:x1] = np.where(treffer, t_nah, tiefe[y0:y1, x0:x1])
        if not farbe:
            continue
        achse = nah.argmax(axis=-1)
        normale_z = (achse == 2) & (np.take_along_axis(dl, achse[..., None], -1)[..., 0] < 0)
        hit = ol + dl * t_nah[..., None]
        oben = treffer & normale_z & (hit[..., 2] > 0)
        seite = treffer & ~oben
        teil = bild[y0:y1, x0:x1].copy()
        welt_normale = np.eye(3)[achse] @ rw.T
        hell = 0.55 + 0.35 * np.abs(welt_normale @ np.array([0.3, -0.4, 0.87]))
        teil = np.where(seite[..., None], FARBE_HOLZ * hell[..., None], teil)
        if tag_id not in _STICKER:
            _STICKER[tag_id] = _sticker(tag_id)
        st = _STICKER[tag_id]
        n = st.shape[0]
        su = ((hit[..., 0] + STICKER_M / 2) / STICKER_M * n).astype(int)
        sv = ((STICKER_M / 2 - hit[..., 1]) / STICKER_M * n).astype(int)
        auf = oben & (su >= 0) & (su < n) & (sv >= 0) & (sv < n)
        grau = st[np.clip(sv, 0, n - 1), np.clip(su, 0, n - 1)].astype(float)
        oben_farbe = np.where(auf[..., None], (grau[..., None] * 0.92 + 8.0) * np.ones(3), FARBE_HOLZ * 1.05)
        teil = np.where(oben[..., None], oben_farbe, teil)
        bild[y0:y1, x0:x1] = teil
    rng = np.random.default_rng(seed)
    if farbe:
        bild = cv2.GaussianBlur(np.asarray(bild, dtype=np.float32), (0, 0), 0.7)
        bild += rng.normal(0, 2.0, bild.shape).astype(np.float32)
        return np.clip(bild, 0, 255).astype(np.uint8)
    mm = np.where(np.isfinite(tiefe) & (tiefe > TIEFE_MIN_M), tiefe * 1000.0 + rng.normal(0, 1.0, tiefe.shape), 0.0)
    return np.clip(mm, 0, 65535).astype(np.uint16)


class SimLeser:
    """Kamera der Simulation mit derselben Schnittstelle wie ``kamera.Leser``."""

    def __init__(self, welt: SimWelt, name: str, hz: float = 5.0):
        self.welt, self.name, self.hz = welt, name, hz
        self.farbe = name == "farbe"
        self.intr = FARBE_INTR if self.farbe else TIEFE_INTR
        self.groesse = FARBE_GROESSE if self.farbe else TIEFE_GROESSE
        self.t_ee_kamera = kin.T_EE_FARBE if self.farbe else kin.T_EE_TIEFE
        self._an = False
        self._bild: np.ndarray | None = None
        self._zeit = 0.0
        self._zaehler = 0
        self._neu = threading.Condition()
        self._seed = 0
        self.fehler = None

    @property
    def laeuft(self) -> bool:
        return self._an

    def bild(self) -> np.ndarray:
        pose, wuerfel = self.welt.schnappschuss()
        self._seed += 1
        t_bk = pose @ kin.invers(self.welt.werkzeug) @ self.t_ee_kamera
        return rendern(t_bk, wuerfel, self.intr, self.groesse, self.farbe, seed=self._seed)

    def start(self) -> None:
        if self._an:
            return
        self._an = True
        threading.Thread(target=self._schleife, name=f"sim-kamera-{self.name}", daemon=True).start()

    def stop(self) -> None:
        self._an = False
        with self._neu:
            self._bild = None
            self._neu.notify_all()

    def _schleife(self) -> None:
        while self._an:
            b = self.bild()
            with self._neu:
                self._bild, self._zeit = b, time.time()
                self._zaehler += 1
                self._neu.notify_all()
            time.sleep(1.0 / self.hz)

    def letztes(self):
        with self._neu:
            return self._bild, self._zeit, self._zaehler

    def naechstes(self, nach: int, zeitlimit_s: float = 3.0):
        ende = time.time() + zeitlimit_s
        with self._neu:
            while self._zaehler <= nach and self._an:
                rest = ende - time.time()
                if rest <= 0:
                    break
                self._neu.wait(rest)
            return self._bild, self._zeit, self._zaehler

    def frisch(self, anzahl: int = 1, zeitlimit_s: float = 5.0) -> list[np.ndarray]:
        return [self.bild() for _ in range(anzahl)]


# --- Betrieb ------------------------------------------------------------------------------------------------------


def aufbauen(kante_m: float = 0.070, zeitraffer: float = 1.0, wuerfel: list[SimWuerfel] | None = None,
             schluessel: str = "sim", tisch_grenze_m: float = -0.025):
    """Dashboard mit simuliertem Arm (Start in der Home-Pose) und Kameras; liefert (Dashboard, Welt).
    tisch_grenze_m: Tischhöhe für die Arbeitsraumgrenze wie ``server.py --tisch-mm`` (Standard −25 mm wie am echten Server,
    der Tisch der Simulation liegt bei ``TISCH_Z_M``). So lehnt die Simulation dieselben Posen ab wie der Arm."""
    import shutil

    import arm as arm_mod
    import server

    home_original = server.KONFIG / "home_pose.json"
    home = arm_mod.home_laden(home_original)
    welt = SimWelt(home["gelenke_deg"] if home else [0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0],
                   start_szene(kante_m) if wuerfel is None else wuerfel, zeitraffer)
    welt.start()
    arm = arm_mod.Arm(kortex_sim(welt), router=None, arbeitsraum=kin.Arbeitsraum(tisch_z_m=tisch_grenze_m))
    arm.start()
    leser = {"farbe": SimLeser(welt, "farbe"), "tiefe": SimLeser(welt, "tiefe")}
    db = server.Dashboard(arm, leser, {"farbe": dict(FARBE_INTR), "tiefe": dict(TIEFE_INTR)},
                          schluessel=schluessel, freigabe=True)
    db.simulation = welt
    db.wuerfel_kante_m = kante_m
    # eigene Dateien, damit die Simulation die Home-Pose und Kalibrierung des echten Arms nicht überschreibt
    db.home_pfad = server.KONFIG / "home_pose_sim.json"
    if not db.home_pfad.exists() and home_original.exists():
        shutil.copyfile(home_original, db.home_pfad)
    db.kamera_konfig_pfad = server.KONFIG / "kamera_farbe_sim.json"
    db.park_pfad = server.KONFIG / "park_pose_sim.json"

    def szene(d: dict) -> dict:
        if d.get("zufall") is not None:
            neu = zufalls_szene(int(d["zufall"]), int(d.get("anzahl", 6)), db.wuerfel_kante_m)
        elif d.get("wuerfel") is not None:
            neu = [SimWuerfel(int(w["id"]), np.array([float(w["x_m"]), float(w["y_m"]),
                                                      TISCH_Z_M + db.wuerfel_kante_m / 2]),
                              db.wuerfel_kante_m, kin.rot_z(math.radians(float(w.get("gier_deg", 0.0)))))
                   for w in d["wuerfel"]]
        else:
            neu = start_szene(db.wuerfel_kante_m)
        if db.ablauf.status.get("laeuft") or db.arm.zustand().get("bewegt"):
            raise arm_mod.BewegungAbgelehnt("erst warten, bis der Arm steht")
        welt.szene_setzen(neu)
        db.ablauf.losgelassen()
        return {"wuerfel": len(neu)}

    db.sim_szene = szene
    return db, welt


def betreiben(a) -> int:
    """Simulation als Server betreiben (``server.py --sim``)."""
    db, welt = aufbauen(a.wuerfel_mm / 1000.0, a.zeitraffer, tisch_grenze_m=a.tisch_mm / 1000.0)
    import server

    srv = server.server_starten(db, a)
    print("SIMULATION: kein echter Arm. Zurücksetzen der Würfel: POST /api/sim/szene")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for l in db.leser.values():
            l.stop()
        db.arm.stop()
        welt.stop()
        srv.server_close()
    return 0
