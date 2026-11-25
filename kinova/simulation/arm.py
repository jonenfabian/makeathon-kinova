"""Anbindung an den Kinova Gen3 über die Kortex-API 2.2.0: Zustand lesen, sicher bewegen, Greifer.

Alle Bewegungen laufen über ``Arm.pose_anfahren()`` bzw. ``Arm.gelenke_anfahren()``. Beide prüfen vorher den
Arbeitsraum (``kinematik.Arbeitsraum``), den Armzustand und begrenzen die Geschwindigkeit. Zielposen werden aus
der **gemessenen** Pose des Arms berechnet, nicht aus der URDF-Kinematik (Abweichung ≈ 7 mm, siehe Tests).
Muster für Aktionen und Meldungen aus Kinovas Beispiel ``102-Movement_high_level/01-move_angular_and_cartesian.py``.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import bahn as bahn_mod
import kinematik as kin
import kollision
import numpy as np

GESCHW_MAX_M_S = 0.05
GESCHW_MAX_DEG_S = 15.0
# Am Arm gemessen (11.11.2026, Firmware 2.2.0): Bei ReachPose folgt das Drehtempo dem Wegtempo. Die Firmware nimmt den
# Zahlenwert in m/s offenbar zugleich als Drehgrenze in rad/s: 50 mm/s → 2,8°/s, 20 mm/s → 1,1°/s, 10 mm/s → ≈ 0,6°/s.
# Die Vorgabe orientation = 15°/s wirkt nicht. Große Drehungen deshalb über Gelenk 7 (ablauf.py), Zeitlimit hiermit.
DREH_RAD_S_JE_M_S = 1.0
GELENK_GESCHW_MAX_DEG_S = 15.0
# Schnellbetrieb (Fabian 11.11.2026: Stapeln „so schnell wie möglich“): nur für Abläufe des Betreuers, nur innerhalb von
# ``Arm.schnell()`` und nur im Thread dieses Auftrags. Teams bleiben bei den Grenzen oben. Obergrenzen sind die Grenzen
# der Firmware (kinova_lesen 9.11.2026: kartesisch hart = weich 0,5 m/s und 100 °/s, Gelenke 80/70 °/s).
SCHNELL_MAX_M_S = 0.50
SCHNELL_MAX_DEG_S = 90.0
SCHNELL_GELENK_MAX_DEG_S = 70.0
SCHNELL_GELENK_MIN_DAUER_S = 0.5
# Gelenkfahrt mit Dauer: Wie die Firmware das Profil formt, ist nicht dokumentiert. ANNAHME: Polynom fünften Grades
# (Spitzentempo 1,875 · Weg/Dauer, Spitzenbeschleunigung 5,77 · Weg/Dauer²). Die Dauer hält damit die weiche
# Beschleunigungsgrenze ANGULAR_TRAJECTORY von 40 °/s² ein. Am Arm prüfen: Bricht eine kurze Dauer mit
# CONTROL_INVALID_ACCELERATION ab, ist das Profil steiler.
GELENK_BESCHL_DEG_S2 = 40.0
NACHLAUF_S = 0.15  # nach ACTION_END warten, bevor die gemessene Pose mit dem Ziel verglichen wird
NACHLAUF_SCHNELL_S = 0.05
# Glätte messen (Fabian 12.11.2026: „nicht ruckeln, sanft anfahren und abbremsen“): Im Schnellbetrieb schreibt jede Fahrt
# die Rückmeldung des Arms mit bis zu 100 Hz mit (Gelenkwinkel und -tempo, Werkzeugpose, IMU am Handgelenk). Kennwerte
# nach bahn.kennwerte in Ergebnis.messung, Rohdaten in messungen/fahrten_<Datum>.jsonl.
MESS_HZ = 100.0
ZUSTAND_HZ = 10.0
# Der Arm meldet nach Aktionen und Greiferbefehlen kurz MANUALLY_CONTROLLED (beobachtet 9.11.2026, Pose unverändert,
# niemand am Joystick). Kein Fehlerzustand; neue Aktionen nimmt die Firmware an.
BEREIT = ("ARMSTATE_SERVOING_READY", "ARMSTATE_SERVOING_MANUALLY_CONTROLLED")
GELENK_MIN_DAUER_S = 3.0
GREIFER_TAKT_S = 0.05
# Nach Greiferbefehlen auf SERVOING_READY warten (Anlass 9.11.2026, siehe bereit_abwarten). Am Arm (12.11.2026) blieb er
# danach bis zur nächsten Fahrt MANUALLY_CONTROLLED, jeder Greiferbefehl wartete so die vollen 2 s (12 s je Turm).
# Im Schnellbetrieb nach „erreicht“ (Öffnen, Loslassen) nur kurz warten: danach fuhr jede Fahrt sofort. Nach
# „stillstand“ (Finger am Würfel) endete die nächste Fahrt 0,3 s später zweimal sofort ohne Bewegung (12.11.2026, Stufe
# 4), nach 2 s lief sie. Dort deshalb weiter die vollen 2 s; wie kurz es ginge, ist nicht gemessen.
GREIFER_BEREIT_S = 2.0
# Am 12.11.2026 schloss der Arm nach dem Senden einer Gelenkbahn mit 150 °/s² die Verbindung (Ursache offen). Dauert
# das Senden einer Aktion oder Bahn länger, steht ein Hinweis im Protokoll. Funkstille ist es nicht: Den Server 3 s und
# 8 s anzuhalten (connection_inactivity_timeout steht auf 2 s) ließ die Verbindung am Arm offen.
SENDEN_LANGSAM_S = 0.5
# Verbindung neu aufbauen (Fabian 12.11.2026): Am Arm schloss die Steuerung dreimal die Verbindung („Broken pipe“), die
# Kortex-Bibliothek merkt das nicht, alle Anfragen laufen in Zeitüberschreitungen. Antwortet die Rückmeldung so lange
# nicht, öffnet der Zustands-Thread eine neue Verbindung mit neuer Sitzung (``neu_verbinden``), höchstens alle
# VERBINDUNG_NEU_PAUSE_S. Danach gilt es wie ein STOPP: Stop an den Arm, laufende Fahrt und Ablauf enden.
# Am Arm geprüft (12.11.2026): Reißt die Verbindung während einer Fahrt, fährt der Arm sie zu Ende (5 cm mit 20 mm/s,
# nach 2 cm gekappt, Ziel erreicht). Bis zum Neuaufbau erreicht ihn kein STOPP aus dem Dashboard, nur der Not-Halt.
VERBINDUNG_NEU_NACH_S = 1.5
VERBINDUNG_NEU_PAUSE_S = 5.0
# Stop nach dem Neuaufbau: Fährt der Arm noch die Fahrt der alten Sitzung zu Ende, hält diese die Steuerung, und Stop
# über die neue scheitert mit SESSION_NOT_IN_CONTROL (am Arm 12.11.2026). So lange wiederholen.
STOP_NACH_NEUAUFBAU_S = 5.0
GREIFER_BEREIT_SCHNELL_S = 0.3
ZIEL_TOLERANZ_MM = 3.0  # gemessene Pose nach ACTION_END; sonst gilt die Bewegung als nicht erreicht
ZIEL_TOLERANZ_DEG = 1.0
# Gelenkbahn: Ende im Nennmodell auf wenige mm genau (am Arm 12.11.2026: 1,0–1,4 mm). Weiter weg trotz ACTION_END gilt
# als nicht erreicht; ohne jede Gelenkbewegung meldet das Ergebnis „ohne Bewegung“ (Ablauf fährt dann per ReachPose).
BAHN_ENDE_TOLERANZ_MM = 5.0
BAHN_UNBEWEGT_DEG = 0.5
STILLSTAND_PROBEN = 6  # 0,3 s ohne Bewegung = Stillstand


class BewegungAbgelehnt(Exception):
    """Zielpose oder Zustand verbietet die Bewegung; es wurde nichts gesendet."""


@dataclass
class Ergebnis:
    ok: bool
    meldung: str
    ziel: dict | None = None
    ist: dict | None = None
    dauer_s: float = 0.0
    messung: dict | None = None  # Glätte der Fahrt (nur Schnellbetrieb, siehe MESS_HZ)

    def als_dict(self) -> dict:
        d = {"ok": self.ok, "meldung": self.meldung, "ziel": self.ziel, "ist": self.ist,
             "dauer_s": round(self.dauer_s, 2)}
        if self.messung is not None:
            d["messung"] = self.messung
        return d


def _zahl(x) -> float | None:
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) else None


class Fahrtmesser:
    """Rückmeldung des Arms mit bis zu ``MESS_HZ`` mitschreiben, solange eine Fahrt läuft (eigener Thread, dieselbe
    Sperre wie alle Aufrufe). Felder, die die Rückmeldung nicht hat (Simulation: Gelenktempo, IMU), fehlen."""

    def __init__(self, arm: Arm):
        self.arm = arm
        self.proben: list[tuple] = []
        self._ende = threading.Event()
        self._thread = threading.Thread(target=self._lauf, name="fahrtmesser", daemon=True)

    def start(self) -> Fahrtmesser:
        self._thread.start()
        return self

    def _lauf(self) -> None:
        takt = 1.0 / MESS_HZ
        naechste = time.monotonic()
        while not self._ende.is_set():
            try:
                fb = self.arm._r(self.arm.zyklisch.RefreshFeedback)
            except self.arm.k["fehlerarten"]:
                return
            jetzt = time.monotonic()
            b = fb.base
            akt = list(fb.actuators)
            ic = fb.interconnect
            imu = [_zahl(getattr(ic, f"imu_acceleration_{a}", None)) for a in "xyz"]
            tempo = [_zahl(getattr(x, "velocity", None)) for x in akt]
            self.proben.append((jetzt, (b.tool_pose_x, b.tool_pose_y, b.tool_pose_z), [x.position for x in akt],
                                None if None in tempo else tempo, None if None in imu else imu))
            naechste += takt
            self._ende.wait(max(0.0, naechste - time.monotonic()))

    def stop(self) -> tuple[dict, dict]:
        """Mitschreiben beenden: (Kennwerte, Rohdaten)."""
        self._ende.set()
        self._thread.join(timeout=1.0)
        pr = self.proben
        if len(pr) < 2:
            return {"proben": len(pr)}, {}
        t = [p[0] for p in pr]
        pos = [p[1] for p in pr]
        q = [p[2] for p in pr]
        tempo = [p[3] for p in pr] if all(p[3] is not None for p in pr) else None
        imu = [p[4] for p in pr] if all(p[4] is not None for p in pr) else None
        kw = bahn_mod.kennwerte(t, pos, q, tempo, imu)
        kw["takt_hz"] = round((len(t) - 1) / max(1e-6, t[-1] - t[0]), 1)
        roh = {"t": [round(x - t[0], 4) for x in t], "pos_m": [[round(v, 5) for v in x] for x in pos],
               "gelenke_deg": [[round(v, 3) for v in x] for x in q]}
        if tempo is not None:
            roh["gelenk_tempo_deg_s"] = [[round(v, 3) for v in x] for x in tempo]
        if imu is not None:
            roh["imu_m_s2"] = [[round(v, 4) for v in x] for x in imu]
        return kw, roh


def fahrten_zusammenfassen(fahrten: list[dict]) -> dict:
    """Kennwerte vieler Fahrten (``Arm.fahrten()``) je Art (pose, gelenke, bahn): Anzahl und Größtwerte, dazu die drei
    Fahrten mit dem größten Ruck (IMU, sonst aus der Pose)."""
    felder = ("v_max_m_s", "a_max_m_s2", "ruck_max_m_s3", "gelenk_a_max_deg_s2", "gelenk_ruck_max_deg_s3",
              "imu_max_m_s2", "imu_ruck_max_m_s3")
    je_art: dict = {}
    for f in fahrten:
        d = je_art.setdefault(f["art"], {"fahrten": 0})
        d["fahrten"] += 1
        for k in felder:
            if f.get(k) is not None:
                d[k] = max(d.get(k, 0.0), f[k])

    def ruck(f):
        return f.get("imu_ruck_max_m_s3", f.get("ruck_max_m_s3", 0.0)) or 0.0
    schlimmste = sorted(fahrten, key=ruck, reverse=True)[:3]
    return {"je_art": je_art, "schlimmste": [{k: f.get(k) for k in ("art", "grund", "dauer_s", "ruck_max_m_s3",
                                                                     "imu_ruck_max_m_s3", "gelenk_ruck_max_deg_s3")}
                                             for f in schlimmste]}


class Arm:
    def __init__(self, k: dict, router, arbeitsraum: kin.Arbeitsraum | None = None, protokoll: Path | None = None,
                 neu_verbinden=None):
        """neu_verbinden: Funktion ohne Argumente, die eine neue Verbindung mit Sitzung öffnet und ihren Router
        liefert (die alte schließt sie selbst). Ohne sie bleibt eine abgerissene Verbindung tot."""
        self.k = k
        self.b2 = k["Base_pb2"]
        self._clients_anlegen(router)
        self._neu_verbinden = neu_verbinden
        self.verbindung_nr = 1  # zählt Neuaufbauten mit; Fahrten, die über eine alte Verbindung laufen, enden
        self.neu_verbunden: str | None = None  # Uhrzeit des letzten Neuaufbaus
        self._fehler_seit: float | None = None
        self._letzter_neuaufbau = -1e9
        self._neuaufbau_fehler: str | None = None
        self.raum = arbeitsraum or kin.Arbeitsraum()
        self.protokoll = protokoll
        self._ruf = threading.Lock()  # serialisiert Aufrufe an den Router (nicht das Warten auf Aktionsende)
        self._bewegung = threading.Lock()  # höchstens eine Bewegung gleichzeitig
        self._daten = threading.Lock()
        self._zustand: dict = {}
        self._halt = threading.Event()
        self.abbruch = threading.Event()  # von anhalten() gesetzt; Abläufe prüfen es zwischen den Schritten
        # Jeder STOPP erhöht den Zähler. Ein Auftrag (eine HTTP-Anfrage) merkt sich beim Beginn den Stand; danach sendet
        # er keine Aktion und keinen Greiferbefehl mehr, sobald ein STOPP dazwischenkam (auch vor dem ersten Befehl).
        self.stopp_zaehler = 0
        self._lokal = threading.local()
        # Hängt ein Würfel im Greifer, ragt er so weit unter die Fingerspitzen; alle Zielposen halten ihn über dem Tisch
        self.last_unter_spitze_m = 0.0
        self.letzte_bewegung_ende = 0.0
        self._thread: threading.Thread | None = None
        self.optionen = k["RouterClientSendOptions"]()
        self.optionen.timeout_ms = 3000
        tool = self._r(self.cc.GetToolConfiguration).tool_transform
        self.werkzeug = kin.werkzeug_aus_kortex({f: getattr(tool, f) for f in
                                                 ("x", "y", "z", "theta_x", "theta_y", "theta_z")})

    # --- Grundlagen ---------------------------------------------------------------------------------------
    def _clients_anlegen(self, router) -> None:
        self.router = router
        self.base = self.k["klassen"]["base"](router)
        self.zyklisch = self.k["klassen"]["base_cyclic"](router)
        self.cc = self.k["klassen"]["control_config"](router)

    def _r(self, f, *args):
        with self._ruf:
            return f(*args, options=self.optionen)

    def _log(self, eintrag: dict) -> None:
        if self.protokoll is None:
            return
        self.protokoll.parent.mkdir(parents=True, exist_ok=True)
        eintrag = {"zeit": time.strftime("%Y-%m-%dT%H:%M:%S"), **eintrag}
        with self.protokoll.open("a", encoding="utf-8") as f:
            f.write(json.dumps(eintrag, ensure_ascii=False) + "\n")

    # --- Zustand ---------------------------------------------------------------------------------------------
    def start(self) -> None:
        self._lesen()
        self._thread = threading.Thread(target=self._schleife, name="arm-zustand", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._halt.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _schleife(self) -> None:
        while not self._halt.wait(1.0 / ZUSTAND_HZ):
            try:
                self._schleife_schritt()
            except Exception as exc:  # noqa: BLE001 - der Zustands-Thread darf nie enden (am Arm 12.11.2026 zweimal)
                traceback.print_exc()
                with self._daten:
                    self._zustand["verbindungsfehler"] = f"Zustands-Thread: {type(exc).__name__}: {exc}"

    def _schleife_schritt(self) -> None:
        try:
            self._lesen()
            self._fehler_seit = None
        except self.k["fehlerarten"] as exc:
            text = f"{type(exc).__name__}: {exc}"
            if self._neuaufbau_fehler:
                text += f"; {self._neuaufbau_fehler}"
            with self._daten:
                self._zustand["verbindungsfehler"] = text
            jetzt = time.monotonic()
            if self._fehler_seit is None:
                self._fehler_seit = jetzt
            sicher_weg = isinstance(exc, ConnectionError)  # Broken pipe: nicht erst abwarten
            if (self._neu_verbinden is not None
                    and (sicher_weg or jetzt - self._fehler_seit >= VERBINDUNG_NEU_NACH_S)
                    and jetzt - self._letzter_neuaufbau >= VERBINDUNG_NEU_PAUSE_S):
                self.verbindung_erneuern(f"{type(exc).__name__}: {exc}")

    def verbindung_trennen(self) -> dict:
        """Nur zum Prüfen des Neuaufbaus am Arm (Betreuer): die laufende Verbindung kappen, wie es der Arm am 12.11.2026
        tat. Der Zustands-Thread muss das erkennen und neu verbinden."""
        import socket

        transport = getattr(self.router, "transport", None)
        if transport is None or self._neu_verbinden is None:
            raise BewegungAbgelehnt("Verbindung trennen geht nur am echten Arm mit Neuaufbau")
        with self._ruf:
            transport.sock.shutdown(socket.SHUT_RDWR)
        self._log({"art": "verbindung_getrennt", "nr": self.verbindung_nr})
        return {"ok": True, "getrennt": self.verbindung_nr}

    def verbindung_neu(self) -> dict:
        """Verbindung von Hand neu aufbauen (Betreuer), statt den Server neu zu starten."""
        if self._neu_verbinden is None:
            raise BewegungAbgelehnt("Neuaufbau ist hier nicht eingerichtet (Simulation)")
        ok = self.verbindung_erneuern("am Mac Mini von Hand ausgelöst")
        return {"ok": ok, "nr": self.verbindung_nr, "fehler": None if ok else self._neuaufbau_fehler}

    def verbindung_erneuern(self, grund: str) -> bool:
        """Neue Verbindung mit Sitzung öffnen, Clients austauschen, dann wie ein STOPP: Stop an den Arm, laufende
        Fahrten und Abläufe enden. Liefert, ob es geklappt hat."""
        self._letzter_neuaufbau = time.monotonic()
        try:
            router = self._neu_verbinden()
        except self.k["fehlerarten"] as exc:
            self._log({"art": "verbindung_neu", "ok": False, "grund": grund, "fehler": f"{type(exc).__name__}: {exc}"})
            self._neuaufbau_fehler = (f"Neuaufbau um {time.strftime('%H:%M:%S')} fehlgeschlagen ({type(exc).__name__}: "
                                      f"{exc}), nächster Versuch in {VERBINDUNG_NEU_PAUSE_S:.0f} s")
            with self._daten:
                self._zustand["verbindungsfehler"] = self._neuaufbau_fehler
            return False
        with self._ruf:
            self._clients_anlegen(router)
            self.verbindung_nr += 1
        self.neu_verbunden = time.strftime("%H:%M:%S")
        self._fehler_seit = None
        self._neuaufbau_fehler = None
        self._log({"art": "verbindung_neu", "ok": True, "grund": grund, "nr": self.verbindung_nr})
        print(f"{self.neu_verbunden} Verbindung zum Arm neu aufgebaut ({grund}); laufende Fahrten abgebrochen",
              flush=True)
        self._stop_nach_neuaufbau()
        with contextlib.suppress(*self.k["fehlerarten"]):
            self._lesen()
        return True

    def _stop_nach_neuaufbau(self) -> bool:
        """Wie anhalten(), aber Stop wiederholen, bis die neue Sitzung die Steuerung hat (STOP_NACH_NEUAUFBAU_S)."""
        with self._daten:
            self.stopp_zaehler += 1
        self.abbruch.set()
        ende = time.monotonic() + STOP_NACH_NEUAUFBAU_S
        versuche = 0
        while True:
            versuche += 1
            try:
                with self._ruf:
                    self.base.Stop(options=self.optionen)
            except self.k["fehlerarten"] as exc:
                if time.monotonic() < ende:
                    time.sleep(0.2)
                    continue
                self._log({"art": "stopp", "grund": "Neuaufbau", "ok": False, "versuche": versuche,
                           "fehler": f"{type(exc).__name__}: {exc}"})
                return False
            self._log({"art": "stopp", "grund": "Neuaufbau", "ok": True, "versuche": versuche})
            return True

    def _lesen(self) -> None:
        fb = self._r(self.zyklisch.RefreshFeedback)
        b = fb.base
        pose = kin.Pose(b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                        b.tool_pose_theta_x, b.tool_pose_theta_y, b.tool_pose_theta_z)
        gelenke = [a.position for a in fb.actuators]
        greifer = None
        try:
            motor = fb.interconnect.gripper_feedback.motor
            greifer = motor[0].position / 100.0 if len(motor) else None  # Rückmeldung in %
        except AttributeError:
            pass
        zustand = {
            "zeit": time.time(),
            "pose": pose.als_dict(),
            "gelenke_deg": gelenke,
            "greifer_position": greifer,
            "armzustand": self.b2.ArmState.Name(b.active_state) if b.active_state else "UNBEKANNT",
            "fehler": int(b.fault_bank_a or 0) or int(b.fault_bank_b or 0),
            "spannung_v": round(b.arm_voltage, 2),
            "verbindungsfehler": None,
            "neu_verbunden": self.neu_verbunden,
        }
        with self._daten:
            bewegt = self._zustand.get("bewegt", False)
            self._zustand = {**zustand, "bewegt": bewegt}

    def zustand(self) -> dict:
        with self._daten:
            return dict(self._zustand)

    def pose_matrix(self) -> np.ndarray:
        p = self.zustand()["pose"]
        return kin.Pose(**p).matrix()

    # --- Aufträge und STOPP -------------------------------------------------------------------------------------
    def auftrag_beginnen(self) -> None:
        """Ab hier gehören alle Befehle dieses Threads zu einem Auftrag, den jeder spätere STOPP beendet."""
        self._lokal.stand = self.stopp_zaehler

    def auftrag_beenden(self) -> None:
        self._lokal.stand = None

    def gestoppt(self) -> bool:
        """Kam seit Beginn des laufenden Auftrags ein STOPP?"""
        stand = getattr(self._lokal, "stand", None)
        return stand is not None and stand != self.stopp_zaehler

    # --- Schnellbetrieb ---------------------------------------------------------------------------------------
    @contextlib.contextmanager
    def schnell(self, v_max_m_s: float, gelenk_max_deg_s: float, dreh_max_deg_s: float = SCHNELL_MAX_DEG_S):
        """Höhere Tempogrenzen für die Befehle dieses Threads (Abläufe des Betreuers), höchstens bis zu den Grenzen der
        Firmware. Außerhalb des Blocks und in allen anderen Threads gelten wieder GESCHW_MAX_M_S usw."""
        werte = [float(v) for v in (v_max_m_s, gelenk_max_deg_s, dreh_max_deg_s)]
        if not all(math.isfinite(v) and v > 0 for v in werte):
            raise BewegungAbgelehnt("Tempogrenzen ungültig")
        alt = getattr(self._lokal, "grenzen", None), getattr(self._lokal, "fahrten", None)
        self._lokal.grenzen = (min(werte[0], SCHNELL_MAX_M_S), min(werte[1], SCHNELL_GELENK_MAX_DEG_S),
                               min(werte[2], SCHNELL_MAX_DEG_S))
        self._lokal.fahrten = []
        try:
            yield
        finally:
            self._lokal.grenzen, self._lokal.fahrten = alt

    def fahrten(self) -> list[dict]:
        """Kennwerte aller gemessenen Fahrten dieses Threads im laufenden ``schnell()``-Block (ohne Rohdaten)."""
        return list(getattr(self._lokal, "fahrten", None) or [])

    def _messung_ablegen(self, art: str, grund: str, geplant: dict | None = None) -> dict | None:
        """Messung der letzten Aktion (``_aktion``) übernehmen: in die Fahrtenliste, Rohdaten ins Fahrtenprotokoll."""
        m = getattr(self._lokal, "messung", None)
        self._lokal.messung = None
        if m is None:
            return None
        kw, roh = m
        eintrag = {"art": art, "grund": grund, **kw}
        if geplant is not None:
            eintrag["geplant"] = geplant
        liste = getattr(self._lokal, "fahrten", None)
        if liste is not None:
            liste.append(eintrag)
        if self.protokoll is not None and roh:
            ziel = self.protokoll.parent / f"fahrten_{time.strftime('%Y%m%d')}.jsonl"
            ziel.parent.mkdir(parents=True, exist_ok=True)
            with ziel.open("a", encoding="utf-8") as f:
                f.write(json.dumps({"zeit": time.strftime("%Y-%m-%dT%H:%M:%S"), **eintrag, "roh": roh},
                                   ensure_ascii=False) + "\n")
        return eintrag

    def grenzen(self) -> tuple[float, float, float]:
        """(m/s kartesisch, °/s Gelenke, °/s Drehung kartesisch) für den aufrufenden Thread."""
        return getattr(self._lokal, "grenzen", None) or (GESCHW_MAX_M_S, GELENK_GESCHW_MAX_DEG_S, GESCHW_MAX_DEG_S)

    def ist_schnell(self) -> bool:
        return getattr(self._lokal, "grenzen", None) is not None

    def _tisch_abstand(self, abstand_tisch_m: float | None) -> float | None:
        if abstand_tisch_m is None and self.last_unter_spitze_m > 0:
            return self.raum.abstand_tisch_m + self.last_unter_spitze_m
        return abstand_tisch_m

    # --- Bewegen ---------------------------------------------------------------------------------------------
    def _vorpruefen(self) -> None:
        z = self.zustand()
        if z.get("verbindungsfehler"):
            raise BewegungAbgelehnt(f"keine Verbindung: {z['verbindungsfehler']}")
        if z.get("armzustand") == "ARMSTATE_IN_FAULT" or z.get("fehler"):
            raise BewegungAbgelehnt("Arm meldet Fehler; erst „Fehler quittieren“")
        if z.get("armzustand") not in BEREIT:
            raise BewegungAbgelehnt(f"Arm nicht bereit ({z.get('armzustand')})")

    def _servo_einzeln(self) -> None:
        modus = self._r(self.base.GetServoingMode)
        if modus.servoing_mode != self.b2.SINGLE_LEVEL_SERVOING:
            info = self.b2.ServoingModeInformation()
            info.servoing_mode = self.b2.SINGLE_LEVEL_SERVOING
            self._r(self.base.SetServoingMode, info)

    def _aktion(self, aktion, zeitlimit_s: float, senden=None) -> tuple[bool, str]:
        """Aktion senden und auf ACTION_END warten. senden: statt ExecuteAction(aktion) diese Funktion aufrufen
        (vorberechnete Bahn); dann zählt auch ACTION_PREPROCESS_ABORT (Bahn von der Steuerung abgelehnt)."""
        fertig = threading.Event()
        ergebnis = {"ereignis": None}
        vorab_abbruch = getattr(self.b2, "ACTION_PREPROCESS_ABORT", None)
        self._lokal.messung = None
        messer = Fahrtmesser(self).start() if self.ist_schnell() else None

        def meldung(n):
            if n.action_event in (self.b2.ACTION_END, self.b2.ACTION_ABORT, vorab_abbruch):
                ergebnis["ereignis"] = self.b2.ActionEvent.Name(n.action_event)
                if n.action_event == self.b2.ACTION_ABORT:
                    ergebnis["details"] = self.b2.SubErrorCodes.Name(n.abort_details) if n.abort_details else ""
                fertig.set()

        with self._ruf:
            nr = self.verbindung_nr
            griff = self.base.OnNotificationActionTopic(meldung, self.b2.NotificationOptions())
        neu_verbunden = False
        try:
            with self._ruf:  # dieselbe Sperre wie Base.Stop: ein STOPP kommt vor dieser Prüfung oder nach dem Senden
                if self.gestoppt():
                    return False, "STOPP vor dem Start, nichts gesendet"
                t_senden = time.monotonic()
                if senden is None:
                    self.base.ExecuteAction(aktion, options=self.optionen)
                else:
                    senden()
                t_senden = time.monotonic() - t_senden
            if t_senden > SENDEN_LANGSAM_S:  # Hinweis für die Fehlersuche (Verbindung hing am 12.11.2026)
                self._log({"art": "senden_langsam", "dauer_s": round(t_senden, 2),
                           "bahn": senden is not None, "grund": getattr(aktion, "name", "")})
            ende = time.monotonic() + zeitlimit_s
            ok = False
            while time.monotonic() < ende:
                if fertig.wait(0.1):
                    ok = True
                    break
                if self.verbindung_nr != nr:  # Meldungen der alten Verbindung kommen nie mehr an
                    neu_verbunden = True
                    break
        finally:
            if messer is not None:
                self._lokal.messung = messer.stop()
            with self._ruf:
                if self.verbindung_nr == nr:
                    with contextlib.suppress(*self.k["fehlerarten"]):
                        self.base.Unsubscribe(griff)
        if neu_verbunden:
            return False, "Verbindung zum Arm neu aufgebaut, Fahrt abgebrochen"
        if not ok:
            try:
                self.anhalten()
            except self.k["fehlerarten"] as exc:  # Verbindung tot: Stop kam nicht an; der Neuaufbau sendet ihn erneut
                return False, f"Zeitlimit {zeitlimit_s:.0f} s überschritten, Stop nicht zugestellt ({type(exc).__name__})"
            return False, f"Zeitlimit {zeitlimit_s:.0f} s überschritten, angehalten"
        if ergebnis["ereignis"] != "ACTION_END":
            return False, f"abgebrochen ({ergebnis.get('details') or ergebnis['ereignis']})"
        return True, "erreicht"

    def pose_anfahren(self, t_ziel: np.ndarray, grund: str, geschw_m_s: float | None = None,
                      geschw_deg_s: float | None = None, greifer_position: float | None = None,
                      abstand_tisch_m: float | None = None) -> Ergebnis:
        """Werkzeugpunkt auf Zielpose (Basis) fahren. Lehnt ab, ohne etwas zu senden, wenn verboten. Tempo: höchstens
        die Grenze des Threads (``grenzen()``), ohne Angabe genau diese."""
        if not np.all(np.isfinite(t_ziel)):
            raise BewegungAbgelehnt("Zielpose enthält ungültige Zahlen")
        gp = 1.0 if greifer_position is None else greifer_position
        gruende = self.raum.pruefen(t_ziel, self.werkzeug, gp, self._tisch_abstand(abstand_tisch_m))
        ziel = kin.Pose.aus_matrix(t_ziel)
        if gruende:
            self._log({"art": "pose", "grund": grund, "ziel": ziel.als_dict(), "abgelehnt": gruende})
            raise BewegungAbgelehnt("; ".join(gruende))
        if not self._bewegung.acquire(blocking=False):
            raise BewegungAbgelehnt("es läuft bereits eine Bewegung")
        try:
            self._vorpruefen()
            self._servo_einzeln()
            start_t = self.pose_matrix()
            weg = float(np.linalg.norm(t_ziel[:3, 3] - start_t[:3, 3]))
            dreh = kin.winkel_abstand_deg(t_ziel[:3, :3], start_t[:3, :3])
            v_max, _, w_max = self.grenzen()
            v = min(v_max if geschw_m_s is None else geschw_m_s, v_max)
            w = min(w_max if geschw_deg_s is None else geschw_deg_s, w_max,
                    math.degrees(v * DREH_RAD_S_JE_M_S))  # tatsächliches Drehtempo
            limit = 10.0 + 2.0 * max(weg / v, dreh / w)
            aktion = self.b2.Action()
            aktion.name = grund[:60]
            zp = aktion.reach_pose.target_pose
            zp.x, zp.y, zp.z = ziel.x, ziel.y, ziel.z
            zp.theta_x, zp.theta_y, zp.theta_z = ziel.theta_x, ziel.theta_y, ziel.theta_z
            aktion.reach_pose.constraint.speed.translation = v
            aktion.reach_pose.constraint.speed.orientation = w
            self._setze_bewegt(True)
            t0 = time.time()
            for versuch in range(2):
                ok, meldung = self._aktion(aktion, limit)
                time.sleep(NACHLAUF_SCHNELL_S if self.ist_schnell() else NACHLAUF_S)
                self._lesen()
                ist = self.zustand()["pose"]
                ist_t = kin.Pose(**ist).matrix()
                abw_mm = float(np.linalg.norm(ist_t[:3, 3] - t_ziel[:3, 3])) * 1000
                abw_deg = kin.winkel_abstand_deg(ist_t[:3, :3], t_ziel[:3, :3])
                if not ok or (abw_mm <= ZIEL_TOLERANZ_MM and abw_deg <= ZIEL_TOLERANZ_DEG):
                    break
                # ACTION_END ohne Bewegung (am Arm 9.11.2026 direkt nach einem Greiferbefehl): warten, neu senden
                self._log({"art": "pose", "grund": grund, "hinweis": f"Ende ohne Ziel ({abw_mm:.1f} mm), Versuch "
                                                                     f"{versuch + 1}"})
                self.bereit_abwarten()
                ok, meldung = False, f"Ziel nicht erreicht trotz ACTION_END ({abw_mm:.1f} mm, {abw_deg:.2f}°)"
            if not ok and "CANNOT_START" in meldung:
                knapp = [f"Gelenk {n}: {r:.1f}°" for n, r in kin.anschlag_reserven(self.zustand()["gelenke_deg"]).items()
                         if r < 10]
                meldung += ("; nahe am Anschlag: " + ", ".join(knapp) + " → Gelenkschritt oder Home") if knapp else ""
            e = Ergebnis(ok, meldung, ziel.als_dict(), ist, time.time() - t0, self._messung_ablegen("pose", grund))
            e.meldung += f"; Abweichung {abw_mm:.1f} mm, {abw_deg:.2f}°"
            self._log({"art": "pose", "grund": grund, **e.als_dict()})
            return e
        finally:
            self._setze_bewegt(False)
            self._bewegung.release()

    def bereit_abwarten(self, zeitlimit_s: float = 2.0) -> bool:
        """Warten, bis der Arm wieder SERVOING_READY meldet (nach Greiferbefehlen meldet er kurz
        MANUALLY_CONTROLLED; eine Aktion in dieser Zeit endete am 9.11.2026 sofort ohne Bewegung)."""
        ende = time.time() + zeitlimit_s
        while time.time() < ende:
            self._lesen()
            if self.zustand().get("armzustand") == "ARMSTATE_SERVOING_READY":
                return True
            time.sleep(0.05)
        return False

    def schritt(self, d_pos_m, d_rot_deg, bezug: str) -> Ergebnis:
        if not all(math.isfinite(float(v)) for v in [*d_pos_m, *d_rot_deg]):
            raise BewegungAbgelehnt("Schritt enthält ungültige Zahlen")
        gruende = self.raum.schritt_pruefen(d_pos_m, d_rot_deg)
        if gruende:
            raise BewegungAbgelehnt("; ".join(gruende))
        ziel = kin.schritt(self.pose_matrix(), d_pos_m, d_rot_deg, bezug)
        return self.pose_anfahren(ziel, f"Schritt {bezug} {list(d_pos_m)} m {list(d_rot_deg)}°")

    def gelenk_schritt(self, gelenk: int, d_deg: float) -> Ergebnis:
        """Ein Gelenk um d_deg drehen (Gelenkfahrt). Zielprüfung über die URDF-Kinematik (Nennmodell, ≈ 7 mm
        gegen Kinovas Rechnung; die Arbeitsraumgrenzen haben mehr Reserve) und den Anschlag des Gelenks."""
        if not 1 <= int(gelenk) <= 7:
            raise BewegungAbgelehnt("Gelenk 1 bis 7")
        if not math.isfinite(d_deg):
            raise BewegungAbgelehnt("Gelenkschritt ungültig")
        if abs(d_deg) > self.raum.schritt_max_deg + 1e-9:
            raise BewegungAbgelehnt(f"Gelenkschritt > {self.raum.schritt_max_deg:.0f}°")
        ziel = list(self.zustand()["gelenke_deg"])
        ziel[gelenk - 1] = (ziel[gelenk - 1] + d_deg) % 360.0
        reserve = kin.anschlag_reserven(ziel).get(gelenk)
        if reserve is not None and reserve < 2.0:
            raise BewegungAbgelehnt(f"Gelenk {gelenk} käme bis auf {reserve:.1f}° an den Anschlag")
        t = kin.vorwaerts(ziel, self.werkzeug)["werkzeug"]
        return self.gelenke_anfahren(ziel, f"Gelenk {gelenk} {d_deg:+.1f}°",
                                     pose_pruefen=kin.Pose.aus_matrix(t).als_dict())

    def gelenke_anfahren(self, gelenke_deg, grund: str, pose_pruefen: dict | None = None,
                         geschw_deg_s: float | None = None, bekannt=()) -> Ergebnis:
        """Gelenkwinkel anfahren (Home). pose_pruefen: gespeicherte Pose des Ziels für die Arbeitsraumprüfung.
        bekannt: weitere vom Arm nachweislich eingenommene Stellungen (Parkposition), siehe kollision.bahn_pruefen;
        die gemessene Startstellung zählt immer dazu."""
        gelenke_deg = [float(g) for g in gelenke_deg]
        if len(gelenke_deg) != 7 or not all(math.isfinite(g) for g in gelenke_deg):
            raise BewegungAbgelehnt("7 gültige Gelenkwinkel erwartet")
        if pose_pruefen is not None:
            gruende = self.raum.pruefen(kin.Pose(**pose_pruefen).matrix(), self.werkzeug, 1.0,
                                        self._tisch_abstand(None))
            if gruende:
                raise BewegungAbgelehnt("; ".join(gruende))
        if not self._bewegung.acquire(blocking=False):
            raise BewegungAbgelehnt("es läuft bereits eine Bewegung")
        try:
            self._vorpruefen()
            jetzt = self.zustand()["gelenke_deg"]
            # Bahn vorher simulieren: Glieder untereinander, Tisch, bekannte Hindernisse, Anschläge (Fabian 9.11.2026)
            frei, fehler = kollision.bahn_pruefen(jetzt, gelenke_deg, self.zustand().get("greifer_position") or 0.0,
                                                  bekannt=[jetzt, *bekannt])
            if not frei:
                i, befund = fehler[0]
                self._log({"art": "gelenke", "grund": grund, "abgelehnt": befund.text(), "bahnschritt": i})
                raise BewegungAbgelehnt(f"Bahnsimulation: {befund.text()} (bei {i * 100 // 60} % der Bahn)")
            self._servo_einzeln()
            weg = max(abs(((a - b) + 180) % 360 - 180) for a, b in zip(gelenke_deg, jetzt))
            w = min(self.grenzen()[1] if geschw_deg_s is None else geschw_deg_s, self.grenzen()[1])
            aktion = self.b2.Action()
            aktion.name = grund[:60]
            for i, a in enumerate(gelenke_deg):
                g = aktion.reach_joint_angles.joint_angles.joint_angles.add()
                g.joint_identifier = i
                g.value = float(a) % 360.0
            # Dauer statt Geschwindigkeit: JOINT_CONSTRAINT_SPEED (15 °/s) brach mit Firmware 2.2.0 sofort mit
            # CONTROL_INVALID_ACCELERATION ab (9.11.2026). Dauer so, dass kein Gelenk schneller als w dreht.
            if self.ist_schnell():  # Spitzentempo ≤ w und Beschleunigung ≤ Grenze bei einem Profil 5. Grades (ANNAHME)
                dauer = max(SCHNELL_GELENK_MIN_DAUER_S, 1.875 * weg / w, math.sqrt(5.77 * weg / GELENK_BESCHL_DEG_S2))
            else:
                dauer = max(GELENK_MIN_DAUER_S, weg / w)
            aktion.reach_joint_angles.constraint.type = self.b2.JOINT_CONSTRAINT_DURATION
            aktion.reach_joint_angles.constraint.value = dauer
            self._setze_bewegt(True)
            t0 = time.time()
            ok, meldung = self._aktion(aktion, 10.0 + 2.0 * dauer)
            time.sleep(NACHLAUF_SCHNELL_S if self.ist_schnell() else NACHLAUF_S)
            self._lesen()
            ist = self.zustand()
            abw = max(abs(((a - b) + 180) % 360 - 180) for a, b in zip(gelenke_deg, ist["gelenke_deg"]))
            e = Ergebnis(ok, f"{meldung}; größte Gelenkabweichung {abw:.2f}°", None, ist["pose"], time.time() - t0,
                         self._messung_ablegen("gelenke", grund))
            self._log({"art": "gelenke", "grund": grund, "ziel_gelenke": list(gelenke_deg), **e.als_dict()})
            return e
        finally:
            self._setze_bewegt(False)
            self._bewegung.release()

    def bahn_fahren(self, posen: list[np.ndarray], grund: str, greifer_position: float | None = None,
                    abstand_tisch_m: float | None = None, v_faktor: float = 0.8, a_deg_s2: float | None = None,
                    rundung_m: float = 0.03, hindernisse=(), rampe_s: float | None = None) -> Ergebnis:
        """Von der jetzigen Pose ohne Halt durch alle ``posen`` fahren (vorberechnete Gelenkbahn, bahn.py). Nur im
        Schnellbetrieb (``schnell()``). Wie ``pose_anfahren`` lehnt es ab, ohne etwas zu senden, wenn eine Pose der Bahn
        verboten ist. Geprüft werden alle Wegpunkte und die tatsächlich gefahrene Bahn (Nennmodell, alle 4 mm):
        Arbeitsraum und Tisch, die Bahnsimulation der Glieder und Abstand zu ``hindernisse``.
        hindernisse: (x, y, oben_z, radius) in m: Würfel und Türme, über die Greifer und gehaltener Würfel nicht tiefer
        als 15 mm über ``oben_z`` fahren dürfen, solange sie waagerecht näher als radius + 60 mm sind.
        Das Ende ist auf wenige mm genau (Nennmodell). Genau sitzende Ziele danach mit ``pose_anfahren``."""
        if not self.ist_schnell():
            raise BewegungAbgelehnt("vorberechnete Bahnen nur im Schnellbetrieb der lokalen Oberfläche am Mac Mini")
        if not posen or not all(np.all(np.isfinite(t)) for t in posen):
            raise BewegungAbgelehnt("Bahn enthält ungültige Zahlen")
        gp = 1.0 if greifer_position is None else greifer_position
        tisch = self._tisch_abstand(abstand_tisch_m)
        for i, t in enumerate(posen):
            gruende = self.raum.pruefen(t, self.werkzeug, gp, tisch)
            if gruende:
                self._log({"art": "bahn", "grund": grund, "wegpunkt": i, "abgelehnt": gruende})
                raise BewegungAbgelehnt(f"Wegpunkt {i + 1}: " + "; ".join(gruende))
        if not self._bewegung.acquire(blocking=False):
            raise BewegungAbgelehnt("es läuft bereits eine Bewegung")
        try:
            self._vorpruefen()
            self._servo_einzeln()
            z = self.zustand()
            q_jetzt, t_jetzt = list(z["gelenke_deg"]), kin.Pose(**z["pose"]).matrix()
            if all(np.linalg.norm(t[:3, 3] - t_jetzt[:3, 3]) < 0.0005 and kin.winkel_abstand_deg(t[:3, :3],
                   t_jetzt[:3, :3]) < 0.2 for t in posen):
                return Ergebnis(True, "schon am Ziel, nichts gesendet", kin.Pose.aus_matrix(posen[-1]).als_dict(),
                                z["pose"], 0.0)
            t_plan = time.time()
            try:
                plan = bahn_mod.planen([t_jetzt, *posen], q_jetzt, self.werkzeug, t_jetzt, v_faktor, a_deg_s2,
                                       rundung_m, rampe_s)
            except bahn_mod.BahnFehler as exc:
                raise BewegungAbgelehnt(f"Bahn: {exc}") from exc
            self._bahn_pruefen(plan, t_jetzt, q_jetzt, gp, tisch, hindernisse)
            t_plan = time.time() - t_plan
            spur = self.b2.PreComputedJointTrajectory()
            spur.mode = self.b2.TRAJECTORY_CONTINUITY_MODE_POSITION  # wie ros_kortex 2.2.1
            for k in range(len(plan.zeit_s)):
                spur.trajectory_elements.add(joint_angles=[float(v) for v in plan.winkel_deg[k]],
                                             joint_speeds=[float(v) for v in plan.tempo_deg_s[k]],
                                             joint_accelerations=[float(v) for v in plan.beschl_deg_s2[k]],
                                             time_from_start=float(plan.zeit_s[k]))
            self._setze_bewegt(True)
            t0 = time.time()
            ok, meldung = self._aktion(None, 10.0 + 2.0 * plan.dauer_s,
                                       senden=lambda: self.base.PlayPreComputedJointTrajectory(spur,
                                                                                              options=self.optionen))
            if not ok and "PREPROCESS" in meldung:
                meldung += "; " + self._bahn_fehlerbericht()
            time.sleep(NACHLAUF_SCHNELL_S)
            self._lesen()
            ist = self.zustand()
            ende = posen[-1]
            ist_t = kin.Pose(**ist["pose"]).matrix()
            abw_mm = float(np.linalg.norm(ist_t[:3, 3] - ende[:3, 3])) * 1000
            if ok and abw_mm > BAHN_ENDE_TOLERANZ_MM:  # ACTION_END, aber nicht am Ziel (Scheinerfolg wie 9.11.2026)
                bewegt = max(abs(((a - b) + 180.0) % 360.0 - 180.0) for a, b in zip(ist["gelenke_deg"], q_jetzt))
                ok = False
                meldung = (f"Ende ohne Ziel ({abw_mm:.1f} mm)"
                           + ("; ohne Bewegung" if bewegt < BAHN_UNBEWEGT_DEG else f"; Gelenke bis {bewegt:.1f}° bewegt"))
            v_aus, a_aus = plan.auslastung()
            messung = self._messung_ablegen("bahn", grund, bahn_mod.plan_kennwerte(plan, self.werkzeug))
            e = Ergebnis(ok, f"{meldung}; Bahn {plan.dauer_s:.2f} s geplant in {t_plan:.2f} s, Tempo {v_aus:.0%}, "
                             f"Beschleunigung {a_aus:.0%} der Grenze, Rampe {plan.rampe_s:.2f} s, Ruck "
                             f"{plan.ruck_max_deg_s3:.0f} °/s³; Ende {abw_mm:.1f} mm neben dem letzten Wegpunkt",
                         kin.Pose.aus_matrix(ende).als_dict(), ist["pose"], time.time() - t0, messung)
            self._log({"art": "bahn", "grund": grund, "wegpunkte": len(posen), "proben": len(plan.zeit_s),
                       "dauer_plan_s": round(plan.dauer_s, 3), **e.als_dict()})
            return e
        finally:
            self._setze_bewegt(False)
            self._bewegung.release()

    def _bahn_pruefen(self, plan, t_jetzt: np.ndarray, q_jetzt, gp: float, tisch: float | None, hindernisse) -> None:
        """Gefahrene Bahn (geglätteter Gelenkpfad im Nennmodell, um die Abweichung an der Startstellung korrigiert)."""
        fehler = t_jetzt @ kin.invers(kin.vorwaerts(q_jetzt, self.werkzeug)["werkzeug"])
        spitze_z = kin.fingerspitze_unter_flansch_m(max(gp, 0.0))
        bekannt_reserven = kin.anschlag_reserven(q_jetzt)
        n = len(plan.gelenkpfad_deg)
        # Hindernis, an dem die Bahn beginnt (Finger um den eben losgelassenen oder gegriffenen Würfel): erlaubt, bis die
        # Bahn es einmal verlassen hat (senkrecht heraus); danach gilt es wie jedes andere
        drin = [True] * len(hindernisse)
        # Ebenso der Tischabstand: Gleich nach dem Greifen steht der gehaltene Würfel noch auf dem Tisch. Erlaubt, solange
        # die Bahn von dort aus steigt, bis sie den Abstand einmal einhält
        am_start, z_vorher = True, -math.inf
        for k, q in enumerate(plan.gelenkpfad_deg):
            wo = f"bei {100 * k // max(1, n - 1)} % der Bahn"
            t = fehler @ kin.vorwaerts(q, self.werkzeug)["werkzeug"]
            gruende = self.raum.pruefen(t, self.werkzeug, gp, tisch)
            if not gruende:
                am_start = False
            elif not (am_start and t[2, 3] >= z_vorher - 0.0005 and all("über dem Tisch" in g for g in gruende)):
                raise BewegungAbgelehnt(f"Bahn {wo}: " + "; ".join(gruende))
            z_vorher = t[2, 3]
            if k % 3 == 0 or k == n - 1:
                befund = kollision.pruefen(q % 360.0, gp, eigen=False)
                if befund.tisch or befund.hindernis:
                    raise BewegungAbgelehnt(f"Bahn {wo}: {befund.text()}")
                for g, r in kin.anschlag_reserven(q % 360.0).items():
                    if r < min(2.0, bekannt_reserven.get(g, 2.0)):
                        raise BewegungAbgelehnt(f"Bahn {wo}: Gelenk {g} nur {r:.1f}° vor dem Anschlag")
            mitte = (t @ kin.invers(self.werkzeug) @ np.array([kin.GREIFER_MITTE_X_M, 0.0, spitze_z, 1.0]))[:3]
            unten = mitte[2] - self.last_unter_spitze_m
            for i, (x, y, oben, radius) in enumerate(hindernisse):
                zu_nah = math.hypot(mitte[0] - x, mitte[1] - y) < radius + 0.06 and unten < oben + 0.015
                if not zu_nah:
                    drin[i] = False
                elif not drin[i]:
                    raise BewegungAbgelehnt(f"Bahn {wo}: zu tief über dem Würfel bei ({x * 1000:.0f}, "
                                            f"{y * 1000:.0f}) mm ({(unten - oben) * 1000:.0f} mm über der Oberseite)")

    def _bahn_fehlerbericht(self) -> str:
        """Warum die Steuerung die Bahn abgelehnt hat (GetTrajectoryErrorReport, wie ros_kortex 2.2.1)."""
        try:
            bericht = self._r(self.base.GetTrajectoryErrorReport)
        except self.k["fehlerarten"] as exc:
            return f"Fehlerbericht nicht lesbar: {exc}"
        teile = []
        for f in bericht.trajectory_error_elements:
            teile.append(f"{self.b2.TrajectoryErrorType.Name(f.error_type)} "
                         f"{self.b2.TrajectoryErrorIdentifier.Name(f.error_identifier)} Gelenk {f.index + 1}: "
                         f"{f.error_value:.3f} (erlaubt {f.min_value:.3f} … {f.max_value:.3f}) {f.message}".strip())
        return "Steuerung: " + (" | ".join(teile[:5]) if teile else "kein Fehlerbericht")

    def _setze_bewegt(self, wert: bool) -> None:
        with self._daten:
            self._zustand["bewegt"] = wert
        if not wert:
            self.letzte_bewegung_ende = time.time()

    def anhalten(self) -> None:
        """Bewegung sofort beenden (Base.Stop) und laufende Aufträge und Abläufe abbrechen. Wartet nicht."""
        with self._daten:
            self.stopp_zaehler += 1
        self.abbruch.set()
        with self._ruf:
            self.base.Stop(options=self.optionen)
        self._log({"art": "stopp"})

    def fehler_quittieren(self) -> None:
        self._r(self.base.ClearFaults)
        self._log({"art": "fehler_quittiert"})

    # --- Greifer ---------------------------------------------------------------------------------------------
    def _greifer_senden(self, modus, wert: float) -> None:
        befehl = self.b2.GripperCommand()
        befehl.mode = modus
        finger = befehl.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = wert
        self._r(self.base.SendGripperCommand, befehl)

    def greifer_messen(self) -> float:
        anfrage = self.b2.GripperRequest(mode=self.b2.GRIPPER_POSITION)
        antwort = self._r(self.base.GetMeasuredGripperMovement, anfrage)
        return float(antwort.finger[0].value) if len(antwort.finger) else float("nan")

    def _greifer_gestoppt(self, stand: int) -> bool:
        """STOPP seit Beginn dieses Greiferbefehls oder des Auftrags (ältere STOPPs zählen nicht)."""
        return self.stopp_zaehler != stand or self.gestoppt()

    def _greifer_abwarten(self, position: float, t0: float, zeitlimit_s: float, stand: int) -> str:
        """Nach einem Positionsbefehl warten, bis das Ziel erreicht ist oder die Finger stillstehen (Objekt
        gegriffen oder Anschlag). Am Arm (9.11.2026) wartete das Schließen sonst die vollen 6 s am Würfel."""
        verlauf = [self.greifer_messen()]
        while time.time() - t0 < zeitlimit_s:
            if self._greifer_gestoppt(stand):
                return "abgebrochen"
            time.sleep(GREIFER_TAKT_S)
            p = self.greifer_messen()
            verlauf.append(p)
            if abs(p - position) < 0.01:
                return "erreicht"
            fenster = verlauf[-STILLSTAND_PROBEN:]
            angelaufen = abs(p - verlauf[0]) > 0.01 or time.time() - t0 > 0.6
            if angelaufen and len(fenster) == STILLSTAND_PROBEN and max(fenster) - min(fenster) < 0.002:
                return "stillstand"
        return "zeitlimit"

    def greifer(self, position: float, geschwindigkeit: float = 1.0, zeitlimit_s: float = 6.0) -> dict:
        """Greifer auf Stellung 0 (offen) … 1 (zu). geschwindigkeit 1 = Positionsbefehl mit Standardtempo;
        darunter Geschwindigkeitsbefehl (Kortex: positiv öffnet, negativ schließt) mit Positionsüberwachung."""
        if not (0.0 <= position <= 1.0) or not math.isfinite(position):
            raise BewegungAbgelehnt("Greiferstellung muss zwischen 0 und 1 liegen")
        if not math.isfinite(geschwindigkeit):
            raise BewegungAbgelehnt("Greifertempo ungültig")
        geschwindigkeit = min(1.0, max(0.05, geschwindigkeit))
        stand = self.stopp_zaehler
        if self.gestoppt():
            return {"ok": False, "soll": position, "ist": self.greifer_messen(), "ausgang": "abgebrochen",
                    "weite_mm": None, "dauer_s": 0.0}
        start = self.greifer_messen()
        t0 = time.time()
        if geschwindigkeit >= 0.999:
            self._greifer_senden(self.b2.GRIPPER_POSITION, position)
            ausgang = self._greifer_abwarten(position, t0, zeitlimit_s, stand)
        else:
            richtung = -1.0 if position > start else 1.0
            vorlauf = max(0.03, 0.12 * geschwindigkeit)  # vor dem Ziel auf Positionsbefehl umschalten
            self._greifer_senden(self.b2.GRIPPER_SPEED, richtung * geschwindigkeit)
            letzte, still, blockiert = start, 0, False
            try:
                while time.time() - t0 < zeitlimit_s and not self._greifer_gestoppt(stand):
                    time.sleep(0.03)
                    p = self.greifer_messen()
                    rest = position - p if richtung < 0 else p - position
                    if rest <= vorlauf:
                        break
                    still = still + 1 if abs(p - letzte) < 0.002 else 0
                    if still > 15:  # ≈ 0,5 s ohne Bewegung: Objekt gegriffen oder Anschlag
                        blockiert = True
                        break
                    letzte = p
            finally:
                self._greifer_senden(self.b2.GRIPPER_SPEED, 0.0)
            ausgang = "stillstand" if blockiert else "erreicht"
            if self._greifer_gestoppt(stand):
                ausgang = "abgebrochen"
            elif not blockiert:  # Rest genau anfahren (Geschwindigkeitsbefehl allein schoss 4,5 mm übers Ziel)
                self._greifer_senden(self.b2.GRIPPER_POSITION, position)
                ausgang = self._greifer_abwarten(position, t0, zeitlimit_s, stand)
        ende = self.greifer_messen()
        kurz = self.ist_schnell() and ausgang == "erreicht"
        self.bereit_abwarten(GREIFER_BEREIT_SCHNELL_S if kurz else GREIFER_BEREIT_S)
        self._log({"art": "greifer", "soll": position, "geschwindigkeit": geschwindigkeit, "start": start,
                   "ende": ende, "ausgang": ausgang})
        return {"ok": ausgang != "abgebrochen", "soll": position, "ist": ende, "ausgang": ausgang,
                "weite_mm": round(kin.greifer_weite_m(ende) * 1000, 1), "dauer_s": round(time.time() - t0, 2)}

    # --- Kamera-Parameter ------------------------------------------------------------------------------------
    def kamera_intrinsik(self) -> dict:
        """Intrinsik von Farbe und Tiefe aus dem Kameramodul (Gerät mit device_type VISION)."""
        dm = self.k["klassen"]["device_manager"](self.router)
        vc = self.k["klassen"]["vision_config"](self.router)
        geraete = self._r(dm.ReadAllDevices)
        vid = next((h.device_identifier for h in geraete.device_handle
                    if h.device_type == self.k["DeviceConfig_pb2"].VISION), 0)
        v = self.k["VisionConfig_pb2"]
        aus = {}
        for name, sensor in (("farbe", v.SENSOR_COLOR), ("tiefe", v.SENSOR_DEPTH)):
            with self._ruf:
                i = vc.GetIntrinsicParameters(v.SensorIdentifier(sensor=sensor), vid, self.optionen)
            d = i.distortion_coeffs
            aus[name] = {"aufloesung": v.Resolution.Name(i.resolution), "fx": i.focal_length_x, "fy": i.focal_length_y,
                         "cx": i.principal_point_x, "cy": i.principal_point_y,
                         "verzeichnung": [d.k1, d.k2, d.p1, d.p2, d.k3]}
        return aus


def home_laden(pfad: Path) -> dict | None:
    return json.loads(pfad.read_text(encoding="utf-8")) if pfad.exists() else None


def home_speichern(pfad: Path, zustand: dict) -> dict:
    daten = {"gespeichert": time.strftime("%Y-%m-%dT%H:%M:%S"), "gelenke_deg": zustand["gelenke_deg"],
             "pose": zustand["pose"], "hinweis": "Werkzeug-z zeigt auf die Tischplatte (Fabian, 9.11.2026)"}
    pfad.parent.mkdir(parents=True, exist_ok=True)
    pfad.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return daten
