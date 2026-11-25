"""Würfel-Ablauf als Skill: suchen, von oben scannen, greifen, abstellen (Wunsch Fabian, 9.11.2026).

Aufbau aus den Beobachtungen am Arm am 9.11.2026:
- Umsehen über Gelenk 1 (dreht den ganzen Arm, ändert das Handgelenk nicht). Kartesisch zur Seite zu fahren hatte
  Gelenk 6 in den Anschlag geschoben (CONTROL_CARTESIAN_CANNOT_START).
- Gesucht und vermessen wird mit der Tiefenkamera; sie trifft seitlich auf < 1 mm, die Farbkamera nicht.
- Geschlossen wird per Positionsbefehl; bleiben die Finger vor dem Ziel stehen, sitzt der Würfel dazwischen.
Jeder Schritt läuft über ``Arm`` (Arbeitsraum, Geschwindigkeit, Anschläge). STOPP setzt ``arm.abbruch`` und erhöht
``arm.stopp_zaehler``; der Ablauf bricht vor dem nächsten Schritt ab, ``Arm`` sendet im selben Auftrag nichts mehr.
"""

from __future__ import annotations

import dataclasses
import math
import threading
import time
from dataclasses import dataclass

import arm as arm_mod
import bahn as bahn_mod
import kinematik as kin
import numpy as np
import sehen

SUCH_HOEHE_M = 0.20  # über der Home-Pose für ein breiteres Blickfeld, aber nur so hoch, wie mindestens
# ANSCHLAG_RESERVE_DEG bleiben. Mit der alten Home-Pose brachten +15 cm Gelenk 6 auf 0,7° an den Anschlag; die neue
# Home-Pose (TCP z ≈ 0,30 m, Reserven 114°/52°/50°, 9.11.2026) lässt mehr Luft.
ANSCHLAG_RESERVE_DEG = 10.0
SUCH_DREHUNGEN_DEG = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0)  # Gelenk 1; positiv dreht nach −y
KAMERA_UEBER_TISCH_M = 0.355  # Abstand Tiefenkamera → Tisch beim Scan von oben (≈ 0,28 m über dem Würfel)
TIEFENBILDER = 5
ABSETZ_LUFT_M = 0.002  # beim Abstellen 2 mm über der Griffhöhe öffnen
LUFT_BEREICH_M = (0.0, 0.010)  # erlaubte Absetzhöhe beim Ablegen
LOSLASSEN_BEREICH_M = (0.002, 0.030)  # zum Loslassen so viel weiter öffnen als am Würfel gemessen
ANHEBEN_M = 0.05
HANDGELENK_AB_DEG = 10.0  # größere Drehungen um die Senkrechte über Gelenk 7 (kartesisch nur ≈ 2,8°/s, arm.py)
WEG_NACH_ABSTELLEN_M = 0.10
KAMERA_UEBER_OBERSEITE_M = 0.28  # Tiefenkamera mindestens so hoch über einer Oberseite (Turm: sonst zu nah)
# Überblick zum Stapeln: Tiefenkamera 0,55 m über dem Tisch sah am Arm (11.11.2026) von (0,43 / 0) aus x 0,17–0,79 m
# und y −0,42 … 0,38 m. Dazu je eine Stellung links und rechts für Würfel weiter außen.
UMSEHEN_STELLEN_M = ((0.43, 0.0), (0.43, 0.28), (0.43, -0.28))
UMSEHEN_KAMERA_UEBER_TISCH_M = 0.55
ORT_RADIUS_M = 0.03  # Funde aus verschiedenen Stellungen näher als das sind derselbe Würfel bzw. Turm
TURM_PLATZ_M = (0.45, 0.0)  # bei gleich hohen Kandidaten wird der Turm hier am nächsten gebaut (gut erreichbar)
# Plätze zum Verteilen nach einem Tempoversuch: wie Aufgabe B3 des Probelaufs (x 300–550, y ±200 mm, ≥ 100 mm voneinander
# und vom Turm bei etwa (420, −10) mm)
VERTEIL_PLAETZE_M = ((0.40, 0.18), (0.50, -0.17), (0.32, -0.16))
PLATZ_FREI_M = 0.10  # so weit muss ein Platz von jedem Würfel und Turm weg sein


@dataclass(frozen=True)
class Tempo:
    """Tempo und Ablauf beim Stapeln (Fabian 11.11.2026: „so schnell wie möglich“). Der Standard ist der bisherige,
    am Arm erprobte Ablauf mit den Grenzen der Teams. Die schnellen Stufen gibt es nur für den Betreuer
    (/api/wuerfel/stapeln_schnell). Sie gelten nur innerhalb von ``Arm.schnell()``."""
    name: str = "normal"
    fahrt_m_s: float = arm_mod.GESCHW_MAX_M_S  # freie Fahrten
    nah_m_s: float = 0.02  # Absenken auf Würfel und Turm
    gelenk_deg_s: float = arm_mod.GELENK_GESCHW_MAX_DEG_S
    schnell: bool = False  # höhere Grenzen (Arm.schnell) und schlanker Ablauf, sonst alles wie bisher
    vorgriff_m: float = 0.10  # Vorgriff bzw. Halt vor dem Ablegen so hoch über dem Ziel
    ueber_m: float = 0.10  # Überfahrt: Fingerspitzen bzw. gehaltener Würfel so hoch über allen Würfeln und Türmen
    turm_nachmessen: bool = True  # Turm vor jedem Würfel neu vermessen (sonst nur vor dem ersten)
    loslassen_m: float | None = None  # beim Ablegen nur so viel weiter öffnen (None: ganz auf die Vorgriff-Öffnung)
    handgelenk_ab_deg: float = 10.0  # größere Drehungen per Gelenk 7 statt kartesisch (kartesisch dreht ≈ v rad/s)
    direkt: bool = False  # aus dem Überblick greifen, ohne den Würfel vorher einzeln zu vermessen
    bahn: bool = False  # freie Fahrten als vorberechnete Gelenkbahn ohne Halt (bahn.py, Versuch)
    bahn_v_faktor: float = 0.8
    bahn_a_deg_s2: float | None = None  # None: weiche Grenze des Arms (40 °/s²) mal 0,9
    rampe_s: float | None = None  # Anfahr-/Bremsrampe der Bahnen (None: Ruck ≤ 400 °/s³, bahn.rampe_fuer; 0: ohne)
    tiefenbilder: int = 5
    beruhigung_s: float | None = None  # Wartezeit vor einem Bild nach der Bewegung (None: server.BERUHIGUNG_S = 0,6 s)
    umsehen_stellen: int = 3  # 1: nur von der Mitte aus umsehen (reicht, wenn alle Würfel in x 0,17–0,79, y ±0,38 m)

    def als_dict(self) -> dict:
        return dataclasses.asdict(self)


def _stufe(name: str, fahrt: float, nah: float, gelenk: float) -> Tempo:
    return Tempo(name, fahrt, nah, gelenk, schnell=True, vorgriff_m=0.05, ueber_m=0.04, turm_nachmessen=False,
                 loslassen_m=0.010, handgelenk_ab_deg=90.0, tiefenbilder=3)


# Für den Versuch am 12.11.2026 von unten nach oben (Fabian: „so schnell wie möglich“). Grenzen der Firmware: 0,5 m/s,
# Gelenke 70 °/s. Absenken auf Würfel und Turm bleibt langsamer.
TEMPO_STUFEN = {
    "normal": Tempo(),
    "1": _stufe("1", 0.10, 0.04, 30.0),
    "2": _stufe("2", 0.20, 0.05, 45.0),
    "3": _stufe("3", 0.30, 0.06, 60.0),
    "4": _stufe("4", 0.40, 0.08, 70.0),
    "max": _stufe("max", 0.50, 0.10, 70.0),
}


# Erlaubte Änderungen an einer Stufe (Betreuer-API): Feld → (kleinster, größter Wert); None erlaubt, wo es sinnvoll ist
TEMPO_BEREICHE = {
    "fahrt_m_s": (0.01, arm_mod.SCHNELL_MAX_M_S), "nah_m_s": (0.005, 0.10),
    "gelenk_deg_s": (5.0, arm_mod.SCHNELL_GELENK_MAX_DEG_S), "vorgriff_m": (0.03, 0.10), "ueber_m": (0.03, 0.15),
    "loslassen_m": (0.002, 0.030), "bahn_v_faktor": (0.1, 1.0), "bahn_a_deg_s2": (10.0, 298.0), "tiefenbilder": (1, 10),
    "beruhigung_s": (0.0, 0.6), "umsehen_stellen": (1, 3), "rampe_s": (0.0, bahn_mod.RAMPE_MAX_S),
}
TEMPO_SCHALTER = ("turm_nachmessen", "direkt", "bahn")
TEMPO_NONE = ("loslassen_m", "bahn_a_deg_s2", "beruhigung_s", "rampe_s")


def tempo_pruefen(aenderungen: dict) -> dict:
    """Änderungen an einer Tempo-Stufe prüfen; unbekannte Felder und Werte außerhalb der Bereiche lehnen ab."""
    aus = {}
    for k, v in aenderungen.items():
        if k in TEMPO_SCHALTER:
            if not isinstance(v, bool):
                raise arm_mod.BewegungAbgelehnt(f"{k}: true oder false")
            aus[k] = v
        elif k in TEMPO_BEREICHE:
            if v is None and k in TEMPO_NONE:
                aus[k] = None
                continue
            lo, hi = TEMPO_BEREICHE[k]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
                raise arm_mod.BewegungAbgelehnt(f"{k}: {lo} bis {hi}")
            aus[k] = int(v) if k in ("tiefenbilder", "umsehen_stellen") else float(v)
        else:
            raise arm_mod.BewegungAbgelehnt(f"unbekanntes Feld {k!r}")
    return aus


class Abgebrochen(Exception):
    """STOPP gedrückt; der Ablauf hat keinen weiteren Schritt gesendet."""


class WuerfelAblauf:
    def __init__(self, db):
        self.db = db
        self.arm = db.arm
        self.sperre = threading.Lock()
        self.status: dict = {"laeuft": False, "schritt": None, "verlauf": []}
        self.gegriffen: dict | None = None  # Plan des Würfels, der gerade im Greifer hängt
        self.tempo = Tempo()
        self.hindernisse: list[tuple[float, float, float, float]] = []  # (x, y, oben, radius) für Bahnen
        self._erst_hoch_m = 0.0  # Finger stehen um einen losgelassenen Würfel: nächste freie Fahrt erst so weit hoch
        self.zeiten: dict = {}

    # --- Hilfen ---------------------------------------------------------------------------------------------
    def _schritt(self, text: str) -> None:
        if self.arm.abbruch.is_set() or self.arm.gestoppt():
            raise Abgebrochen(f"STOPP vor „{text}“")
        self.status["schritt"] = text
        self.status["verlauf"].append({"zeit": time.strftime("%H:%M:%S"), "schritt": text})

    def _ok(self, e, text: str) -> dict:
        d = e.als_dict() if hasattr(e, "als_dict") else e
        self.status["verlauf"][-1]["ergebnis"] = d.get("meldung") or d.get("ausgang")
        if isinstance(d.get("dauer_s"), (int, float)):
            art = "greifer_s" if "ausgang" in d else "fahrt_s"
            self.zeiten[art] = self.zeiten.get(art, 0.0) + float(d["dauer_s"])
        if not d.get("ok"):
            raise arm_mod.BewegungAbgelehnt(f"{text}: {d.get('meldung') or d.get('ausgang')}")
        return d

    def _starten(self, name: str):
        if not self.sperre.acquire(blocking=False):
            raise arm_mod.BewegungAbgelehnt("es läuft bereits ein Ablauf")
        self.arm.abbruch.clear()
        self.status = {"laeuft": True, "name": name, "schritt": None, "verlauf": [], "fehler": None}
        self.zeiten = {"start": time.time()}

    def _beenden(self, fehler: Exception | None = None) -> None:
        self.status["laeuft"] = False
        self.status["schritt"] = None
        if fehler is not None:
            self.status["fehler"] = f"{type(fehler).__name__}: {fehler}"
        self.sperre.release()

    def _ausfuehren(self, name: str, funktion, *args):
        self._starten(name)
        try:
            ergebnis = funktion(*args)
        except Exception as exc:
            self._beenden(exc)
            raise
        self._beenden()
        return {**(ergebnis or {}), "verlauf": self.status["verlauf"]}

    def _home(self) -> None:
        self._frei_machen()
        self._schritt("Home anfahren")
        home = arm_mod.home_laden(self.db.home_pfad)
        if home is None:
            raise arm_mod.BewegungAbgelehnt("keine Home-Pose gespeichert")
        self._ok(self.arm.gelenke_anfahren(home["gelenke_deg"], "Ablauf: Home", pose_pruefen=home["pose"]), "Home")

    def _tiefe_in_basis(self) -> np.ndarray:
        t0 = time.time()
        anzahl = TIEFENBILDER if not self.tempo.schnell else self.tempo.tiefenbilder
        bilder, t_bk = (self.db._bild_und_kamera("tiefe", anzahl=anzahl, beruhigung_s=self.tempo.beruhigung_s)
                        if self.tempo.schnell else self.db._bild_und_kamera("tiefe", anzahl=anzahl))
        tiefe = np.median(np.stack(bilder), axis=0).astype(np.uint16)
        punkte = sehen.in_basis(sehen.punktwolke(tiefe, self.db.intrinsik["tiefe"]), t_bk)
        self.zeiten["bild_s"] = self.zeiten.get("bild_s", 0.0) + time.time() - t0
        return punkte

    # --- freie Fahrten im Schnellbetrieb --------------------------------------------------------------------------
    def _unter_werkzeug_m(self, greifer_position: float) -> float:
        """So weit reichen Fingerspitzen und ein gehaltener Würfel unter den Werkzeugpunkt (Werkzeug senkrecht)."""
        return (kin.fingerspitze_unter_flansch_m(greifer_position) - float(self.arm.werkzeug[2, 3])
                + self.arm.last_unter_spitze_m)

    def _ueber_z(self, greifer_position: float = 1.0) -> float:
        """Werkzeughöhe für Überfahrten: Fingerspitzen bzw. gehaltener Würfel ``tempo.ueber_m`` über allen bekannten
        Würfeln und Türmen (mindestens über dem Tisch)."""
        oben = max([self.arm.raum.tisch_z_m, *(h[2] for h in self.hindernisse)])
        return oben + self.tempo.ueber_m + self._unter_werkzeug_m(greifer_position)

    def _fahren_nach(self, ziel: np.ndarray, grund: str, ueber_z: float | None = None,
                     greifer_position: float | None = None, abstand_tisch_m: float | None = None,
                     geschw_m_s: float | None = None):
        """Freie Fahrt zur Werkzeugpose ``ziel``. Stehen die Finger noch um einen losgelassenen Würfel, zuerst
        senkrecht hoch. ueber_z: unterwegs mindestens so hoch (hoch, waagerecht, runter). Mit ``tempo.bahn`` eine
        vorberechnete Gelenkbahn ohne Halt, sonst ReachPose je Wegpunkt. Ohne ueber_z und ohne Hochfahren ist das
        dasselbe wie ``Arm.pose_anfahren(ziel)``."""
        jetzt = self.arm.pose_matrix()
        z_hoch = jetzt[2, 3] + self._erst_hoch_m
        if ueber_z is not None:
            z_hoch = max(z_hoch, ueber_z)
        punkte = []
        if z_hoch > jetzt[2, 3] + 0.001:
            hoch = jetzt.copy()
            hoch[2, 3] = z_hoch
            punkte.append(hoch)
        if ueber_z is not None and ziel[2, 3] < z_hoch - 0.001:
            drueber = ziel.copy()
            drueber[2, 3] = z_hoch
            punkte.append(drueber)
        punkte.append(ziel)
        e = None
        if self.tempo.bahn:
            try:
                e = self.arm.bahn_fahren(punkte, grund, greifer_position, abstand_tisch_m, self.tempo.bahn_v_faktor,
                                         self.tempo.bahn_a_deg_s2, hindernisse=self.hindernisse,
                                         rampe_s=self.tempo.rampe_s)
            except arm_mod.BewegungAbgelehnt as exc:
                if not str(exc).startswith("Bahn"):  # Wegpunkt verboten, Arm nicht bereit: gilt auch für ReachPose
                    raise
                self._rueckfall(grund, str(exc))
            # Steuerung lehnt ab oder die Bahn endet sofort ohne Bewegung: nichts bewegt, per ReachPose weiter
            if e is not None and not e.ok and ("PREPROCESS" in e.meldung or "ohne Bewegung" in e.meldung):
                self._rueckfall(grund, e.meldung)
                e = None
        if e is None:
            dauer, e = 0.0, None
            for i, t in enumerate(punkte):
                e = self.arm.pose_anfahren(t, grund if i == len(punkte) - 1 else f"{grund} (Wegpunkt {i + 1})",
                                           geschw_m_s=geschw_m_s or self.arm.grenzen()[0],
                                           greifer_position=greifer_position, abstand_tisch_m=abstand_tisch_m)
                dauer += e.dauer_s
                if not e.ok:
                    break
            e.dauer_s = dauer
        if e.ok:
            self._erst_hoch_m = 0.0
        return e

    def _rueckfall(self, grund: str, warum: str) -> None:
        """Bahn nicht planbar oder von der Steuerung abgelehnt (nichts bewegt): dieselben Wegpunkte mit ReachPose."""
        liste = self.zeiten.setdefault("bahn_rueckfall", [])
        liste.append(f"{grund}: {warum}"[:300])
        if self.status.get("verlauf"):
            self.status["verlauf"][-1]["bahn_rueckfall"] = warum[:300]

    def _frei_machen(self) -> None:
        """Steht der Greifer noch um einen losgelassenen Würfel, senkrecht herausfahren (vor Gelenkfahrten)."""
        if self._erst_hoch_m > 0:
            t = self.arm.pose_matrix()
            t[2, 3] += self._erst_hoch_m
            self._schritt(f"{self._erst_hoch_m * 100:.0f} cm hochfahren")
            self._ok(self.arm.pose_anfahren(t, "nach dem Ablegen hoch", geschw_m_s=self.arm.grenzen()[0]), "Hochfahren")
            self._erst_hoch_m = 0.0

    def _handgelenk_ausrichten(self, r_ziel: np.ndarray) -> None:
        """Werkzeug um die Senkrechte auf die Gier von ``r_ziel`` drehen, per Gelenkfahrt von Gelenk 7 (15°/s, Bahn
        vorher simuliert). Kartesisch dreht die Firmware nur mit ≈ 2,8°/s; ein Würfel bei −87° brauchte so 30 s.
        Gilt für senkrecht stehendes Werkzeug; das Vorzeichen bestimmt die Vorwärtskinematik."""
        t = self.arm.pose_matrix()
        ziel = math.degrees(math.atan2(r_ziel[1, 0], r_ziel[0, 0]))
        d = ((ziel - math.degrees(math.atan2(t[1, 0], t[0, 0])) + 180.0) % 360.0) - 180.0
        if abs(d) < (self.tempo.handgelenk_ab_deg if self.tempo.schnell else HANDGELENK_AB_DEG):
            return
        self._frei_machen()
        jetzt = list(self.arm.zustand()["gelenke_deg"])
        kandidaten = []
        for vorzeichen in (1.0, -1.0):
            q = list(jetzt)
            q[6] = (q[6] + vorzeichen * d) % 360.0
            f = kin.vorwaerts(q, self.arm.werkzeug)["werkzeug"]
            rest = abs(((math.degrees(math.atan2(f[1, 0], f[0, 0])) - ziel + 180.0) % 360.0) - 180.0)
            kandidaten.append((rest, q, f))
        _, q, f = min(kandidaten, key=lambda k: k[0])
        self._schritt(f"Handgelenk {d:+.0f}° drehen (Gelenk 7)")
        self._ok(self.arm.gelenke_anfahren(q, "Handgelenk ausrichten", pose_pruefen=kin.Pose.aus_matrix(f).als_dict()),
                 "Handgelenk")

    def _gelenk1_auf(self, basis_deg, versatz_deg: float, text: str) -> None:
        ziel = list(basis_deg)
        ziel[0] = (ziel[0] + versatz_deg) % 360.0
        pose = kin.Pose.aus_matrix(kin.vorwaerts(ziel, self.arm.werkzeug)["werkzeug"]).als_dict()
        self._ok(self.arm.gelenke_anfahren(ziel, text, pose_pruefen=pose), text)

    def _masse(self) -> dict:
        """Suchbereiche für die eingestellte Würfelkante (``Dashboard.wuerfel_kante_m``)."""
        return sehen.wuerfel_masse(getattr(self.db, "wuerfel_kante_m", 0.070))

    # --- Bausteine ------------------------------------------------------------------------------------------
    def suchen(self) -> sehen.Wuerfel:
        """Von der Suchpose (Home + 15 cm) aus Gelenk 1 drehen, bis die Tiefenkamera einen Würfel findet."""
        self._home()
        self._schritt(f"bis {SUCH_HOEHE_M * 100:.0f} cm anheben")
        for _ in range(round(SUCH_HOEHE_M / 0.02)):  # in 2-cm-Stufen, solange die Anschläge Luft haben
            t = self.arm.pose_matrix()
            t[2, 3] += 0.02
            self._ok(self.arm.pose_anfahren(t, "Ablauf: Suchhöhe"), "Suchhöhe")
            if min(kin.anschlag_reserven(self.arm.zustand()["gelenke_deg"]).values()) < ANSCHLAG_RESERVE_DEG:
                t[2, 3] -= 0.02  # eine Stufe zurück
                self._ok(self.arm.pose_anfahren(t, "Ablauf: Suchhöhe zurück"), "Suchhöhe zurück")
                break
        such_gelenke = list(self.arm.zustand()["gelenke_deg"])
        self.status["verlauf"][-1]["ergebnis"] = (
            f"Reserve Gelenke 2/4/6: {', '.join(f'{r:.0f}°' for r in kin.anschlag_reserven(such_gelenke).values())}")
        kandidaten = []
        for drehung in SUCH_DREHUNGEN_DEG:
            if drehung:
                self._schritt(f"umsehen: Gelenk 1 {drehung:+.0f}°")
                self._gelenk1_auf(such_gelenke, drehung, f"Ablauf: umsehen {drehung:+.0f}°")
            self._schritt("Tiefenbild prüfen")
            punkte = self._tiefe_in_basis()
            try:
                w = sehen.wuerfel_finden(punkte, **self._masse())
            except ValueError as exc:
                # aus der Suchstellung oft nur angeschnitten: dann darüberfahren und dort vollständig vermessen
                w = sehen.angeschnittener_wuerfel(sehen.objekte_finden(punkte), hoehe_m=self._masse()["hoehe_m"])
                if w is None:
                    kandidaten.append(f"{drehung:+.0f}°: {exc}")
                    continue
            self.status["verlauf"][-1]["ergebnis"] = f"Würfel(-kandidat) {sehen._text(w)}"
            return w
        raise ValueError("kein Würfel gefunden (" + " | ".join(kandidaten) + ")")

    def _kamera_ueber(self, x: float, y: float, z_kamera: float, grund: str):
        """Tiefenkamera senkrecht über (x, y) auf Höhe z_kamera, Drehung um die Senkrechte wie jetzt."""
        t = self.arm.pose_matrix()
        r = sehen.senkrecht_ausgerichtet(t[:3, :3] @ kin.rot_x(math.pi)) @ kin.rot_x(math.pi)  # z nach unten
        t_w_k = kin.invers(self.arm.werkzeug) @ kin.T_EE_TIEFE
        ziel = kin.transform(r, np.array([x, y, z_kamera]) - r @ t_w_k[:3, 3])
        if self.tempo.schnell:
            gp = self.arm.zustand().get("greifer_position") or 0.0
            return self._fahren_nach(ziel, grund, ueber_z=self._ueber_z(gp), greifer_position=gp)
        return self.arm.pose_anfahren(ziel, grund)

    def ueber_scannen(self, x: float, y: float, tisch_z: float, oben_z: float, bei: bool = False) -> dict:
        """Tiefenkamera senkrecht über (x, y) fahren und dort genau vermessen; Ergebnis wie /api/wuerfel/scannen.
        Kamera 0,355 m über dem Tisch, über einem Turm höher. bei=True: den Würfel bzw. Turm an dieser Stelle."""
        z_kamera = max(tisch_z + KAMERA_UEBER_TISCH_M, oben_z + KAMERA_UEBER_OBERSEITE_M)
        self._schritt("Kamera über den Würfel")
        e = self._kamera_ueber(x, y, z_kamera, "Ablauf: Kamera über Würfel")
        if not e.ok and "CANNOT_START" in e.meldung:
            # Ausweg wie am Arm erprobt: Home, Gelenk 1 in Richtung Würfel drehen, dann kartesisch
            self._home()
            home = arm_mod.home_laden(self.db.home_pfad)
            richtung = math.degrees(math.atan2(y, x))
            home_richtung = math.degrees(math.atan2(home["pose"]["y"], home["pose"]["x"]))
            self._schritt("Gelenk 1 Richtung Würfel")
            self._gelenk1_auf(home["gelenke_deg"], home_richtung - richtung, "Ablauf: Richtung Würfel")
            self._schritt("Kamera über den Würfel (zweiter Versuch)")
            e = self._kamera_ueber(x, y, z_kamera, "Ablauf: Kamera über Würfel")
        self._ok(e, "Kamera über Würfel")
        self._schritt("Würfel von oben vermessen")
        return self.scannen_hier((x, y) if bei else None)

    def ueber_wuerfel_scannen(self, w: sehen.Wuerfel, bei: bool = False) -> dict:
        """Tiefenkamera senkrecht über den Würfel fahren und dort genau vermessen; Ergebnis wie /api/wuerfel/scannen."""
        return self.ueber_scannen(w.mitte_m[0], w.mitte_m[1], w.tisch_z_m, w.tisch_z_m + w.hoehe_m, bei)

    def scannen_hier(self, bei_m=None) -> dict:
        """Würfel im aktuellen Tiefenbild vermessen und Greifplan berechnen (ohne Bewegung). bei_m (x, y): den Würfel
        nehmen, der dort liegt (z. B. unter einem erkannten AprilTag)."""
        if self.gegriffen is not None:  # der hängende Würfel sitzt unter der Tiefenkamera und verdeckt die Sicht
            raise arm_mod.BewegungAbgelehnt("Vermessen geht nicht, solange ein Würfel im Greifer hängt: er verdeckt die "
                                            "Kamera. Erst den Turm vermessen, dann greifen, dann ablegen")
        w = sehen.wuerfel_finden(self._tiefe_in_basis(), bei_m=bei_m, **self._masse())
        return self._plan_setzen(w)

    def _plan_setzen(self, w: sehen.Wuerfel) -> dict:
        """Greifplan für den Würfel w berechnen und als aktuellen Plan merken (wie nach einem Scan)."""
        plan = sehen.greifpose_wuerfel(w, self.arm.pose_matrix(), self.arm.werkzeug,
                                       kante_m=getattr(self.db, "wuerfel_kante_m", None))
        self.db._plan = plan
        self.db.letzter_wuerfel = {
            **w.als_dict(), "vorgriff": kin.Pose.aus_matrix(plan["vorgriff"]).als_dict(),
            "griff": kin.Pose.aus_matrix(plan["griff"]).als_dict(),
            "greifer_offen_mm": round(kin.greifer_weite_m(plan["greifer_offen"]) * 1000, 1),
            "spitze_z_mm": round(plan["spitze_z_m"] * 1000, 1), "stapel": plan["stapel"],
            "unten_z_mm": round(plan["unten_z_m"] * 1000, 1), "boden_z_mm": round(plan["boden_z_m"] * 1000, 1),
            "pruefung_vorgriff": self.arm.raum.pruefen(plan["vorgriff"], self.arm.werkzeug, plan["greifer_offen"]),
            "pruefung_griff": self.arm.raum.pruefen(plan["griff"], self.arm.werkzeug, plan["greifer_zu"])}
        return self.db.letzter_wuerfel

    def greifen(self) -> dict:
        p = self.db._plan
        if p is None:
            raise arm_mod.BewegungAbgelehnt("erst Würfel scannen")
        w = self.db.letzter_wuerfel or {}
        if w.get("pruefung_vorgriff") or w.get("pruefung_griff"):
            raise arm_mod.BewegungAbgelehnt("Greifplan verletzt den Arbeitsraum: "
                                            + "; ".join(w.get("pruefung_vorgriff", []) + w.get("pruefung_griff", [])))
        self._schritt("Greifer öffnen")
        self._ok(self.arm.greifer(p["greifer_offen"]), "Greifer öffnen")
        self._handgelenk_ausrichten(p["vorgriff"][:3, :3])
        vorgriff = p["griff"].copy()
        vorgriff[2, 3] += self.tempo.vorgriff_m
        self._schritt(f"Vorgriff {self.tempo.vorgriff_m * 100:.0f} cm über dem Würfel")
        ueber_z = self._ueber_z(p["greifer_offen"]) if self.tempo.schnell else None  # über allem, was im Weg steht
        self._ok(self._fahren_nach(vorgriff, "Würfel Vorgriff", ueber_z=ueber_z, greifer_position=p["greifer_offen"]),
                 "Vorgriff")
        self._schritt("absenken")
        self._ok(self.arm.pose_anfahren(p["griff"], "Würfel Griff", geschw_m_s=self.tempo.nah_m_s,
                                        greifer_position=p["greifer_zu"]), "Absenken")
        self._schritt("schließen")
        g = self._ok(self.arm.greifer(p["greifer_zu"]), "Schließen")
        if g["ausgang"] != "stillstand" or g["ist"] >= p["greifer_zu"] - 0.01:
            # Ziel (Würfelbreite − 10 mm) ohne Widerstand erreicht: nichts zwischen den Fingern
            self._schritt("kein Würfel gegriffen: öffnen und hochfahren")
            self.arm.greifer(p["greifer_offen"])
            self.arm.pose_anfahren(vorgriff, "Würfel verfehlt: zurück", greifer_position=p["greifer_offen"])
            raise arm_mod.BewegungAbgelehnt(f"Greifer hat ohne Widerstand geschlossen ({g['weite_mm']} mm)")
        if not self.tempo.schnell:  # schnell: ablegen() hebt gleich auf Überfahrhöhe
            self._schritt(f"{ANHEBEN_M * 100:.0f} cm anheben")
            anheben = p["griff"].copy()
            anheben[2, 3] += ANHEBEN_M
            self._ok(self.arm.pose_anfahren(anheben, "Würfel anheben", geschw_m_s=0.02, greifer_position=1.0),
                     "Anheben")
        self.gegriffen = {**p, "greifer_ist": g["ist"], "weite_mm": g["weite_mm"]}
        self.arm.last_unter_spitze_m = max(0.0, p["spitze_z_m"] - p["unten_z_m"])
        return {"gegriffen": True, "greifer_weite_mm": g["weite_mm"], "dauer_schliessen_s": g["dauer_s"]}

    def abstellen(self) -> dict:
        """Würfel dort absetzen, wo er gegriffen wurde (2 mm über Griffhöhe öffnen), dann 10 cm hochfahren."""
        p = self.gegriffen
        if p is None:
            raise arm_mod.BewegungAbgelehnt("es hängt kein Würfel im Greifer (erst greifen)")
        ziel = p["griff"].copy()
        ziel[2, 3] += ABSETZ_LUFT_M
        self._schritt("absetzen")
        self._ok(self.arm.pose_anfahren(ziel, "Würfel absetzen", geschw_m_s=0.02, greifer_position=p["greifer_ist"],
                                        abstand_tisch_m=self.arm.raum.abstand_tisch_m), "Absetzen")
        self._schritt("loslassen")
        self._ok(self.arm.greifer(p["greifer_offen"]), "Loslassen")
        self.losgelassen()
        self._schritt(f"{WEG_NACH_ABSTELLEN_M * 100:.0f} cm hochfahren")
        weg = ziel.copy()
        weg[2, 3] += WEG_NACH_ABSTELLEN_M
        self._ok(self.arm.pose_anfahren(weg, "nach dem Abstellen hoch", greifer_position=p["greifer_offen"]),
                 "Hochfahren")
        return {"abgestellt": True}

    def _loslass_stellung(self, p: dict, loslassen_m: float | None) -> float:
        """Greiferstellung zum Loslassen: ganz auf die Öffnung des Vorgriffs oder nur ``loslassen_m`` weiter als am
        Würfel gemessen."""
        if loslassen_m is None:
            return p["greifer_offen"]
        if not math.isfinite(loslassen_m) or not LOSLASSEN_BEREICH_M[0] <= loslassen_m <= LOSLASSEN_BEREICH_M[1]:
            raise arm_mod.BewegungAbgelehnt(f"Loslassen: {LOSLASSEN_BEREICH_M[0] * 1000:.0f} bis "
                                            f"{LOSLASSEN_BEREICH_M[1] * 1000:.0f} mm weiter öffnen")
        weite = min(kin.greifer_weite_m(0.0), kin.greifer_weite_m(p["greifer_ist"]) + loslassen_m)
        return kin.greifer_position_fuer_weite(weite)

    def ablegen(self, x_m: float, y_m: float, gier_deg: float | None = None, unten_z_m: float | None = None,
                luft_m: float = ABSETZ_LUFT_M, loslassen_m: float | None = None, ueber_z_m: float | None = None) -> dict:
        # Fingerspitzen werden mit der gemessenen Greiferstellung geprüft: am Würfel können die Finger nicht weiter zu
        """Gegriffenen Würfel mit seiner Mitte bei (x, y) ablegen, Unterseite auf ``unten_z_m`` (Standard: die beim
        Greifen gemessene Tischhöhe, auch wenn er oben von einem Turm kam; zum Stapeln ``oben_z_mm`` des Scans vom
        unteren Würfel). gier_deg: Drehung des Würfels um
        die Senkrechte (Standard: wie gegriffen). Ablauf: 10 cm darüber, mit 20 mm/s absenken, ``luft_m`` über der
        Ablagehöhe öffnen (ganz oder nur ``loslassen_m`` weiter als am Würfel), 10 cm hochfahren."""
        p = self.gegriffen
        if p is None:
            raise arm_mod.BewegungAbgelehnt("es hängt kein Würfel im Greifer (erst greifen)")
        if not all(math.isfinite(v) for v in (x_m, y_m)):
            raise arm_mod.BewegungAbgelehnt("Ablageort ungültig")
        if not math.isfinite(luft_m) or not LUFT_BEREICH_M[0] <= luft_m <= LUFT_BEREICH_M[1]:
            raise arm_mod.BewegungAbgelehnt(f"Absetzhöhe {LUFT_BEREICH_M[0] * 1000:.0f} bis "
                                            f"{LUFT_BEREICH_M[1] * 1000:.0f} mm über der Ablagehöhe")
        if loslassen_m is None and self.tempo.schnell:
            loslassen_m = self.tempo.loslassen_m
        loslassen = self._loslass_stellung(p, loslassen_m)
        boden = p.get("boden_z_m", p["unten_z_m"])  # Tisch unter dem Würfel bzw. unter dem Turm, aus dem er kam
        unten = boden if unten_z_m is None else float(unten_z_m)
        if not math.isfinite(unten) or unten < boden - 0.005:
            raise arm_mod.BewegungAbgelehnt("Ablagehöhe unter der beim Greifen gemessenen Tischhöhe")
        dreh = 0.0 if gier_deg is None else float(gier_deg) - p["gier_deg"]
        if not math.isfinite(dreh):
            raise arm_mod.BewegungAbgelehnt("Gierwinkel ungültig")
        dreh = ((dreh + 45.0) % 90.0) - 45.0  # Würfel: alle 90° gleich, Handgelenk höchstens 45° drehen
        r = kin.rot_z(math.radians(dreh))
        alt = np.array([p["mitte_m"][0], p["mitte_m"][1], 0.0])
        neu = np.array([x_m, y_m, unten - p["unten_z_m"] + luft_m])
        ziel = kin.transform(r, neu - r @ alt) @ p["griff"]
        if self.tempo.schnell:
            return self._ablegen_schnell(p, ziel, loslassen, ueber_z_m, x_m, y_m, unten)
        ueber = ziel.copy()
        ueber[2, 3] += WEG_NACH_ABSTELLEN_M
        # erst senkrecht auf Überfahrhöhe, dann waagerecht: der Würfel hängt nach dem Greifen nur 5 cm über dem Tisch und
        # würde auf einer schrägen Bahn andere Würfel rammen (Review 10.11.2026)
        jetzt = self.arm.pose_matrix()
        hoehe = max(jetzt[2, 3], ueber[2, 3])
        if jetzt[2, 3] < hoehe - 0.001:
            hoch = jetzt.copy()
            hoch[2, 3] = hoehe
            self._schritt("senkrecht anheben")
            self._ok(self.arm.pose_anfahren(hoch, "Ablegen: anheben", greifer_position=1.0), "Anheben")
        ueber[2, 3] = hoehe
        self._handgelenk_ausrichten(ueber[:3, :3])
        self._schritt("über den Ablageort")
        self._ok(self.arm.pose_anfahren(ueber, "Ablegen: darüber", greifer_position=1.0), "über den Ablageort")
        self._schritt("absenken")
        self._ok(self.arm.pose_anfahren(ziel, "Ablegen: absenken", geschw_m_s=0.02, greifer_position=p["greifer_ist"],
                                        abstand_tisch_m=self.arm.raum.abstand_tisch_m), "Absenken")
        self._schritt("loslassen")
        self._ok(self.arm.greifer(loslassen), "Loslassen")
        self.losgelassen()
        self._schritt(f"{WEG_NACH_ABSTELLEN_M * 100:.0f} cm hochfahren")
        self._ok(self.arm.pose_anfahren(ueber, "nach dem Ablegen hoch", greifer_position=loslassen), "Hochfahren")
        return {"abgelegt": True, "mitte_mm": [round(x_m * 1000, 1), round(y_m * 1000, 1)],
                "unten_z_mm": round(unten * 1000, 1)}

    def _ablegen_schnell(self, p: dict, ziel: np.ndarray, loslassen: float, ueber_z_m: float | None,
                         x_m: float, y_m: float, unten: float) -> dict:
        """Schnellbetrieb: auf Überfahrhöhe (über allem, was im Weg steht), waagerecht hin, bis ``vorgriff_m`` über das
        Ziel, langsam absenken, loslassen. Hochfahren übernimmt die nächste freie Fahrt."""
        vor = ziel.copy()
        vor[2, 3] += self.tempo.vorgriff_m
        ueber_z = max(vor[2, 3], self._ueber_z(1.0) if ueber_z_m is None else ueber_z_m)
        self._handgelenk_ausrichten(vor[:3, :3])
        self._schritt("über den Ablageort")
        self._ok(self._fahren_nach(vor, "Ablegen: darüber", ueber_z=ueber_z, greifer_position=1.0),
                 "über den Ablageort")
        self._schritt("absenken")
        self._ok(self.arm.pose_anfahren(ziel, "Ablegen: absenken", geschw_m_s=self.tempo.nah_m_s,
                                        greifer_position=p["greifer_ist"], abstand_tisch_m=self.arm.raum.abstand_tisch_m),
                 "Absenken")
        self._schritt("loslassen")
        self._ok(self.arm.greifer(loslassen), "Loslassen")
        self.losgelassen()
        self._erst_hoch_m = max(self.tempo.vorgriff_m, 0.04)  # Fingerspitzen tauchen 25 mm ein
        return {"abgelegt": True, "mitte_mm": [round(x_m * 1000, 1), round(y_m * 1000, 1)],
                "unten_z_mm": round(unten * 1000, 1)}

    def anfahren_vermessen(self, x: float, y: float) -> dict:
        """Tiefenkamera über (x, y) fahren und den Würfel bzw. Turm dort vermessen. Erst so hoch, dass auch ein Turm aus
        sehen.STAPEL_MAX Würfeln ins Bild passt, dann, wenn die Oberseite tiefer liegt, auf die übliche Höhe darüber.
        Im Probelauf (11.11.2026) musste ein Team das selbst herausfinden: wuerfel_vermessen allein fährt nicht."""
        if self.gegriffen is not None:
            raise arm_mod.BewegungAbgelehnt("Vermessen geht nicht, solange ein Würfel im Greifer hängt: er verdeckt die "
                                            "Kamera. Erst den Turm vermessen, dann greifen, dann ablegen")
        kante = getattr(self.db, "wuerfel_kante_m", 0.070)
        tisch = self.arm.raum.tisch_z_m
        hoch = tisch + sehen.STAPEL_MAX * kante
        w = self.ueber_scannen(x, y, tisch, hoch, bei=True)
        if hoch - w["oben_z_mm"] / 1000 > 0.04:
            w = self.ueber_scannen(w["mitte_m"][0], w["mitte_m"][1], tisch, w["oben_z_mm"] / 1000, bei=True)
        return w

    def umsehen(self) -> list[dict]:
        """Aus erhöhten Stellungen (UMSEHEN_STELLEN_M) alle Würfel und Türme im Arbeitsbereich finden. Stellungen, die
        der Arm nicht erreicht, werden übersprungen. Ergebnis: Orte mit Mitte, Tischhöhe, Oberseite und Anzahl."""
        kante = getattr(self.db, "wuerfel_kante_m", 0.070)
        funde: list[sehen.Wuerfel] = []
        for x, y in UMSEHEN_STELLEN_M[:self.tempo.umsehen_stellen] if self.tempo.schnell else UMSEHEN_STELLEN_M:
            self._schritt(f"umsehen über ({x * 1000:.0f}, {y * 1000:.0f}) mm")
            try:
                e = self._kamera_ueber(x, y, self.arm.raum.tisch_z_m + UMSEHEN_KAMERA_UEBER_TISCH_M,
                                       "Stapeln: umsehen")
            except arm_mod.BewegungAbgelehnt as exc:
                self.status["verlauf"][-1]["ergebnis"] = f"übersprungen: {exc}"
                continue
            if not e.ok:
                self.status["verlauf"][-1]["ergebnis"] = f"übersprungen: {e.meldung}"
                continue
            neu = sehen.alle_wuerfel(self._tiefe_in_basis(), **self._masse())
            self.status["verlauf"][-1]["ergebnis"] = f"{len(neu)} gesehen"
            funde += neu
        orte: list[sehen.Wuerfel] = []
        for w in sorted(funde, key=lambda w: -w.punkte_oben):  # vollständigste Sicht zuerst
            if all(math.hypot(*(w.mitte_m[:2] - o.mitte_m[:2])) > ORT_RADIUS_M for o in orte):
                orte.append(w)
        aus = [{"x": float(w.mitte_m[0]), "y": float(w.mitte_m[1]), "tisch": float(w.tisch_z_m),
                "oben": float(w.tisch_z_m + w.hoehe_m), "n": sehen.stapel(w, kante)[0], "gier": float(w.gier_deg)}
               for w in orte]
        self._sicht = {id(o): w for o, w in zip(aus, orte)}  # Überblick je Ort, zum Greifen ohne Einzelscan (direkt)
        return aus

    def _phase(self, text: str, **mehr) -> None:
        self.status.update(phase=text, **mehr)
        self.zeiten.setdefault("phasen", []).append((text, time.time()))

    def _hindernisse_setzen(self, orte: list[dict]) -> None:
        """Alle bekannten Würfel und Türme als Hindernisse für Überfahrten (Schnellbetrieb)."""
        kante = getattr(self.db, "wuerfel_kante_m", 0.070)
        self.hindernisse = [(o["x"], o["y"], o["oben"], 0.75 * kante) for o in orte if o["n"] > 0]

    def stapeln(self) -> dict:
        """Alle Würfel im Arbeitsbereich zu Türmen aus höchstens sehen.STAPEL_MAX Würfeln stapeln (Wunsch Fabian,
        11.11.2026). Umsehen, Turm dort, wo schon die meisten Würfel stehen (sonst nahe TURM_PLATZ_M), dann je
        Würfel: Turm von oben vermessen, Würfel von oben vermessen, greifen, mit der Gier des Turms auf seine
        Oberseite ablegen. Ein Würfel, der sich nicht greifen lässt, wird übersprungen; hängt einer im Greifer,
        bricht der Ablauf ab. Schnellbetrieb (``self.tempo``): Turm nur vor dem ersten Würfel vermessen, wahlweise ohne
        Einzelscan greifen (``direkt``), Überfahrten über allen bekannten Würfeln."""
        kante = getattr(self.db, "wuerfel_kante_m", 0.070)
        self._home()
        self._phase("Umsehen")
        orte = self.umsehen()
        anzahl = sum(o["n"] for o in orte)
        if anzahl < 2:
            raise ValueError(f"{anzahl} Würfel gefunden, zum Stapeln braucht es mindestens zwei")
        self.zeiten["umsehen_s"] = round(time.time() - self.zeiten["start"], 2)

        def abstand(a: dict, b) -> float:
            return math.hypot(a["x"] - b[0], a["y"] - b[1])

        def neuer_turm(kandidaten: list[dict]) -> dict:
            return max(kandidaten, key=lambda o: (o["n"], -abstand(o, TURM_PLATZ_M)))

        turm = neuer_turm(orte)
        tuerme, uebersprungen, gestapelt, je_wuerfel = [], [], 0, []
        self.status.update(orte=orte, turm=turm)  # für Dashboard und Video: Orte werden unten fortgeschrieben
        while True:
            self._hindernisse_setzen(orte)
            # Quellen: Würfel und Türme, die nicht der aktuelle Turm, nicht fertig und nicht übersprungen sind
            rest = [o for o in orte if o is not turm and o["n"] > 0 and "grund" not in o and not o.get("fertig")]
            if not rest:
                break
            if turm["n"] >= sehen.STAPEL_MAX:  # voll: abschließen, nie wieder Quelle; nächster Turm aus dem Rest
                turm["fertig"] = True
                tuerme.append(turm)
                turm = neuer_turm(rest)
                self.status["turm"] = turm
                continue
            quelle = min(rest, key=lambda o: abstand(o, (turm["x"], turm["y"])))
            beginn = time.time()
            if not self.tempo.schnell or self.tempo.turm_nachmessen or not turm.get("vermessen"):
                self._phase(f"Turm vermessen ({turm['n']} Würfel)")
                t = self.ueber_scannen(turm["x"], turm["y"], turm["tisch"], turm["oben"], bei=True)
                turm.update(x=t["mitte_m"][0], y=t["mitte_m"][1], oben=t["oben_z_mm"] / 1000, n=t["stapel"],
                            gier=t["gier_deg"], vermessen=True)
                if turm["n"] >= sehen.STAPEL_MAX:
                    continue
            try:
                self._phase("Würfel holen", quelle=quelle)
                sicht = getattr(self, "_sicht", {}).get(id(quelle))
                if self.tempo.schnell and self.tempo.direkt and sicht is not None:
                    w = self._plan_setzen(sicht)  # Greifplan aus dem Überblick
                else:
                    w = self.ueber_scannen(quelle["x"], quelle["y"], quelle["tisch"], quelle["oben"], bei=True)
                self.greifen()
            except (arm_mod.BewegungAbgelehnt, ValueError) as exc:
                if self.gegriffen is not None:
                    raise
                quelle["grund"] = str(exc)
                uebersprungen.append({"x_mm": round(quelle["x"] * 1000), "y_mm": round(quelle["y"] * 1000),
                                      "grund": str(exc)})
                continue
            quelle.update(n=w["stapel"] - 1, oben=w["oben_z_mm"] / 1000 - kante)
            if sicht is not None and quelle["n"] > 0:  # Überblick des Rests: ein Würfel weniger
                self._sicht[id(quelle)] = dataclasses.replace(
                    sicht, hoehe_m=sicht.hoehe_m - kante, mitte_m=sicht.mitte_m - np.array([0.0, 0.0, kante / 2]))
            self._hindernisse_setzen(orte)
            self._phase(f"Auf den Turm ({turm['n'] + 1}. Würfel)")
            self.ablegen(turm["x"], turm["y"], turm["gier"], turm["oben"])
            gestapelt += 1
            turm.update(n=turm["n"] + 1, oben=turm["oben"] + kante)
            je_wuerfel.append(round(time.time() - beginn, 2))
        if turm not in tuerme:
            tuerme.append(turm)
        self.zeiten["bis_letzter_wuerfel_s"] = round(time.time() - self.zeiten["start"], 2)
        ergebnis = []
        self._phase("Türme nachmessen")
        self._hindernisse_setzen(orte)
        for tm in tuerme:
            n = self.ueber_scannen(tm["x"], tm["y"], tm["tisch"], tm["oben"], bei=True)
            ergebnis.append({"x_mm": round(n["mitte_m"][0] * 1000, 1), "y_mm": round(n["mitte_m"][1] * 1000, 1),
                             "wuerfel": n["stapel"], "oben_z_mm": n["oben_z_mm"]})
        self._home()
        self._phase("fertig")
        z = self.zeiten
        zeiten = {"gesamt_s": round(time.time() - z["start"], 2), "umsehen_s": z.get("umsehen_s"),
                  "bis_letzter_wuerfel_s": z.get("bis_letzter_wuerfel_s"), "je_wuerfel_s": je_wuerfel,
                  **{k: round(z.get(k, 0.0), 2) for k in ("fahrt_s", "greifer_s", "bild_s")},
                  "bahn_rueckfall": len(z.get("bahn_rueckfall", [])), "bahn_rueckfall_gruende": z.get("bahn_rueckfall", [])[:3]}
        if self.arm.ist_schnell():  # Glätte aller Fahrten (Arm.Fahrtmesser), Fabian 12.11.2026: „nicht ruckeln“
            zeiten["glaette"] = arm_mod.fahrten_zusammenfassen(self.arm.fahrten())
        return {"gefunden": anzahl, "gestapelt": gestapelt, "tuerme": ergebnis, "uebersprungen": uebersprungen,
                "zeiten": zeiten, "tempo": self.tempo.als_dict()}

    def verteilen(self, plaetze_m=VERTEIL_PLAETZE_M) -> dict:
        """Türme abbauen: Würfel einzeln auf freie Plätze legen, bis kein Turm mehr steht oder kein Platz mehr frei ist
        (für Tempoversuche ohne Umräumen von Hand). Plätze, an denen schon etwas steht, bleiben frei. Jeder Würfel
        wird vorher von oben vermessen. Im Schnellbetrieb mit dessen Tempo."""
        kante = getattr(self.db, "wuerfel_kante_m", 0.070)
        self._home()
        self._phase("Umsehen")
        orte = self.umsehen()
        gelegt = []
        frei = [p for p in plaetze_m if all(math.hypot(o["x"] - p[0], o["y"] - p[1]) >= PLATZ_FREI_M for o in orte)]
        while frei:
            self._hindernisse_setzen(orte)
            tuerme = [o for o in orte if o["n"] >= 2]
            if not tuerme:
                break
            turm = max(tuerme, key=lambda o: o["n"])
            platz = frei.pop(0)
            self._phase(f"Turm abbauen ({turm['n']} Würfel)", quelle=turm)
            w = self.ueber_scannen(turm["x"], turm["y"], turm["tisch"], turm["oben"], bei=True)
            self.greifen()
            turm.update(n=w["stapel"] - 1, oben=w["oben_z_mm"] / 1000 - kante)
            neu = {"x": platz[0], "y": platz[1], "tisch": float(w["boden_z_mm"]) / 1000,
                   "oben": float(w["boden_z_mm"]) / 1000 + kante, "n": 1, "gier": 0.0}
            self._hindernisse_setzen(orte)
            self._phase(f"Würfel auf ({platz[0] * 1000:.0f}, {platz[1] * 1000:.0f}) mm")
            self.ablegen(platz[0], platz[1], 0.0)
            orte.append(neu)
            gelegt.append({"x_mm": round(platz[0] * 1000), "y_mm": round(platz[1] * 1000)})
        self._home()
        self._phase("fertig")
        return {"gelegt": gelegt, "tuerme_uebrig": [o["n"] for o in orte if o["n"] >= 2]}

    def losgelassen(self) -> None:
        """Kein Würfel mehr im Greifer (abgelegt oder Greifer geöffnet)."""
        self.gegriffen = None
        self.arm.last_unter_spitze_m = 0.0

    # --- Abläufe für die API ---------------------------------------------------------------------------------
    def api_suchen(self) -> dict:
        def lauf():
            return {"wuerfel": self.ueber_wuerfel_scannen(self.suchen())}
        return self._ausfuehren("Würfel suchen und scannen", lauf)

    def api_greifen(self) -> dict:
        return self._ausfuehren("Würfel greifen", self.greifen)

    def api_abstellen(self) -> dict:
        def lauf():
            erg = self.abstellen()
            self._home()
            return erg
        return self._ausfuehren("Würfel abstellen", lauf)

    def api_ablegen(self, x_m: float, y_m: float, gier_deg: float | None = None, unten_z_m: float | None = None,
                    luft_m: float = ABSETZ_LUFT_M, loslassen_m: float | None = None) -> dict:
        return self._ausfuehren("Würfel ablegen", self.ablegen, x_m, y_m, gier_deg, unten_z_m, luft_m, loslassen_m)

    def api_anfahren_vermessen(self, x: float, y: float) -> dict:
        return self._ausfuehren("Würfel anfahren und vermessen", self.anfahren_vermessen, x, y)

    def api_stapeln(self) -> dict:
        return self._ausfuehren("Würfel suchen und stapeln", self.stapeln)

    def api_verteilen(self, plaetze_m=VERTEIL_PLAETZE_M, stufe: str | None = None) -> dict:
        """Türme auf Plätze verteilen, mit normalem Tempo oder einer Schnellstufe (nur Betreuer)."""
        if stufe is None:
            return self._ausfuehren("Türme verteilen", self.verteilen, plaetze_m)
        tempo = TEMPO_STUFEN.get(str(stufe))
        if tempo is None or not tempo.schnell:
            raise arm_mod.BewegungAbgelehnt(f"unbekannte Stufe {stufe!r}")

        def lauf():
            self.tempo = tempo
            try:
                with self.arm.schnell(tempo.fahrt_m_s, tempo.gelenk_deg_s):
                    return self.verteilen(plaetze_m)
            finally:
                self.tempo = Tempo()
                self.hindernisse = []
        return self._ausfuehren(f"Türme verteilen (Stufe {tempo.name})", lauf)

    def api_stapeln_schnell(self, stufe: str = "1", **aenderungen) -> dict:
        """Stapeln im Schnellbetrieb (nur Betreuer). stufe: Schlüssel aus TEMPO_STUFEN; aenderungen: einzelne Felder von
        Tempo (geprüft, siehe TEMPO_BEREICHE)."""
        tempo = TEMPO_STUFEN.get(str(stufe))
        if tempo is None or not tempo.schnell:
            raise arm_mod.BewegungAbgelehnt(f"unbekannte Stufe {stufe!r} (" + ", ".join(
                k for k, v in TEMPO_STUFEN.items() if v.schnell) + ")")
        tempo = dataclasses.replace(tempo, **tempo_pruefen(aenderungen))

        def lauf():
            self.tempo = tempo
            try:
                with self.arm.schnell(tempo.fahrt_m_s, tempo.gelenk_deg_s):
                    return self.stapeln()
            finally:
                self.tempo = Tempo()
                self.hindernisse = []
        return self._ausfuehren(f"Würfel stapeln schnell (Stufe {tempo.name})", lauf)

    def api_alles(self, abstellen: bool) -> dict:
        def lauf():
            w = self.ueber_wuerfel_scannen(self.suchen())
            g = self.greifen()
            erg = {"wuerfel": w, **g}
            if abstellen:
                erg.update(self.abstellen())
                self._home()
            return erg
        return self._ausfuehren("Würfel suchen, greifen" + (", abstellen" if abstellen else ""), lauf)
