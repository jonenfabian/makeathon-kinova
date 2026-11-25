#!/usr/bin/env python3
"""Kinova-Dashboard: HTTP-Server mit API, MJPEG-Streams und Bedienseite.

Start (Zugangsdaten des Arms: kinova/README.md, Abschnitt „Lese-Skript“)::

    kinova/.venv-kortex/bin/python kinova/dashboard/server.py            # nur am Mac Mini: http://127.0.0.1:8080
    kinova/.venv-kortex/bin/python kinova/dashboard/server.py --netz     # zusätzlich für Team-Laptops im Netz
    python kinova/dashboard/server.py --sim                              # Simulation ohne Arm und ohne Kortex-API

Rollen (``netz.py``): **Betreuer** am Mac Mini selbst darf alles. **Teams** brauchen den Team-Schlüssel, zum Bewegen
zusätzlich das Fahrrecht und die Freigabe des Betreuers.
Schutz gegen fremde Webseiten im Browser des Betreuers: POST nur mit Kopfzeile ``X-Kinova: 1`` (Browser senden eigene
Kopfzeilen nicht ohne CORS-Freigabe quer zu anderen Seiten; die CORS-Antwort erlaubt diese Kopfzeile nicht) und nur
mit Host 127.0.0.1/localhost (gegen DNS-Rebinding). Antworten an den Betreuer tragen keine CORS-Freigabe.
"""

from __future__ import annotations

import argparse
import base64
import concurrent.futures
import contextlib
import json
import math
import mimetypes
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

if __name__ == "__main__":
    # Meldungen sofort ausgeben, auch in eine Datei umgeleitet (Probelauf 11.11.2026: Log blieb leer). Der erste Start
    # nach der Installation lädt numpy und OpenCV zum ersten Mal; auf dem MacBook dauerte das über 20 s.
    sys.stdout.reconfigure(line_buffering=True)
    print("Dashboard startet … (beim ersten Start nach der Installation bis zu 30 s)")

try:
    import numpy as np
except ImportError:  # direkt mit ./server.py gestartet: das System-Python hat die Abhängigkeiten nicht
    sys.exit("numpy fehlt. Am Mac Mini mit kinova/.venv-kortex/bin/python kinova/dashboard/server.py starten, im "
             "Team-Paket erst pip install -r requirements.txt.")

HIER = Path(__file__).resolve().parent
sys.path.insert(0, str(HIER.parent / "tools"))

import ablauf as ablauf_mod
import arm as arm_mod
import bahn as bahn_mod
import kalibrierung
import kamera
import kinematik as kin
import netz as netz_mod
import sehen

STATIC = HIER / "static"
KONFIG = HIER / "config"
MESSUNGEN = HIER.parent / "messungen"
BERUHIGUNG_S = 0.6  # Wartezeit nach einer Bewegung, bevor Bilder ausgewertet werden
ANFRAGE_MAX_BYTES = 64 * 1024
WUERFEL_KANTE_M = 0.060  # Holzwürfel der Teams, geliefert 11.11.2026 (Fabian); Ryobi-Würfel mit --wuerfel-mm 70
TISCH_Z_MM = -25.0  # Arbeitsraumgrenze: Tischplatte laut Tiefenbild −26,0 mm (11.11.2026), 1 mm Reserve


class Dashboard:
    """Hält Arm, Kameras und die letzten Erkennungsergebnisse; wird vom HTTP-Handler benutzt."""

    def __init__(self, arm: arm_mod.Arm, leser: dict[str, kamera.Leser], intrinsik: dict,
                 schluessel: str | None = None, freigabe: bool = False):
        self.arm = arm
        self.leser = leser
        self.intrinsik = intrinsik
        self.home_pfad = KONFIG / "home_pose.json"
        self.park_pfad = KONFIG / "park_pose.json"
        self.messungen = MESSUNGEN
        # Kamera → Endeffektor; Farbe wird durch die Kalibrierung ersetzt (config/kamera_farbe.json)
        self.t_ee = {"farbe": kin.T_EE_FARBE.copy(), "tiefe": kin.T_EE_TIEFE.copy()}
        self.intrinsik_werk = dict(intrinsik.get("farbe") or {})
        self.kamera_konfig_pfad = KONFIG / "kamera_farbe.json"
        self.farbkamera: dict | None = None  # None = Werkswerte
        self.letzter_tag: dict | None = None
        self.letzter_wuerfel: dict | None = None
        self._plan: dict | None = None
        self.ablauf = ablauf_mod.WuerfelAblauf(self)
        self.kalibrierung = kalibrierung.KalibrierAblauf(self)
        self.wuerfel_kante_m = WUERFEL_KANTE_M
        self.netz = netz_mod.Netzzugang(schluessel or netz_mod.neuer_schluessel(), anhalten=self.arm.anhalten,
                                        freigabe=freigabe, wechsel=self.erkennung_vergessen)
        self.simulation = None  # sim.SimWelt, wenn ohne Arm betrieben

    def erkennung_vergessen(self) -> None:
        """Neues Team fährt: Tag und Greifplan des vorigen Teams nicht weiterverwenden (der gehaltene Würfel bleibt)."""
        self.letzter_tag = None
        if self.ablauf.gegriffen is None:
            self.letzter_wuerfel = None
            self._plan = None

    def farbkamera_setzen(self, daten: dict | None) -> None:
        """Kalibrierte Farbkamera (Intrinsik und Endeffektor → Kamera) verwenden; None: Werkswerte."""
        if daten is None:
            self.intrinsik["farbe"] = dict(self.intrinsik_werk)
            self.t_ee["farbe"] = kin.T_EE_FARBE.copy()
            self.farbkamera = None
            return
        self.intrinsik["farbe"] = dict(daten["intrinsik"])
        self.t_ee["farbe"] = np.array(daten["t_ee_kamera"], dtype=float)
        self.farbkamera = {"kalibriert": daten.get("kalibriert"), "guete": daten.get("guete")}

    # --- Zustand ---------------------------------------------------------------------------------------------
    def zustand(self) -> dict:
        z = self.arm.zustand()
        if not z:
            return {"bereit": False}
        frames = kin.vorwaerts(z["gelenke_deg"], self.arm.werkzeug)
        greifer = z.get("greifer_position") or 0.0
        gemessen = kin.Pose(**z["pose"]).matrix()
        glieder = {n: frames[n].tolist() for n in kin.GLIEDER}
        glieder.update({n: t.tolist() for n, t in kin.greifer_frames(frames["end_effector_link"], greifer).items()})
        abw = float(np.linalg.norm(frames["werkzeug"][:3, 3] - gemessen[:3, 3]))
        return {
            "bereit": True, **z,
            "greifer_weite_mm": round(kin.greifer_weite_m(greifer) * 1000, 1),
            "anschlag_reserve_deg": {str(n): round(r, 1) for n, r in kin.anschlag_reserven(z["gelenke_deg"]).items()},
            "glieder": glieder,
            "frames": {"werkzeug_gemessen": gemessen.tolist(),
                       "kamera_farbe": (gemessen @ kin.invers(self.arm.werkzeug) @ self.t_ee["farbe"]).tolist()},
            "fk_abweichung_mm": round(abw * 1000, 1),
            "home": arm_mod.home_laden(self.home_pfad),
            "park": arm_mod.home_laden(self.park_pfad),
            "streams": {n: l.laeuft for n, l in self.leser.items()},
            "tag": self.letzter_tag, "wuerfel": self.letzter_wuerfel, "ablauf": self.ablauf.status,
            "haelt_wuerfel": self.ablauf.gegriffen is not None,
            "farbkamera": self.farbkamera or "Werkswerte",
            "kalibrierung": None if self.kalibrierung.letztes is None else {
                k: self.kalibrierung.letztes.get(k) for k in ("guete", "aenderung", "brett", "tiefe_gegen_farbe",
                                                              "intrinsik", "ordner", "abdeckung", "gegenprobe",
                                                              "ausgelassen")},
            "arbeitsraum": vars(self.arm.raum),
            "wuerfel_kante_mm": round(self.wuerfel_kante_m * 1000, 1),
            "simulation": None if self.simulation is None else self.simulation.info(wahrheit=True),
        }

    # --- Aktionen --------------------------------------------------------------------------------------------
    def schritt(self, d: dict) -> dict:
        bezug = d.get("bezug", "basis")
        pos = [float(d.get(a, 0.0)) for a in ("dx", "dy", "dz")]
        rot = [float(d.get(a, 0.0)) for a in ("drx", "dry", "drz")]
        return self.arm.schritt(pos, rot, bezug).als_dict()

    def gelenke(self, d: dict) -> dict:
        """Alle Gelenkwinkel anfahren (Gelenkfahrt mit Bahnsimulation)."""
        ziel = [float(v) for v in d.get("gelenke_deg", [])]
        if len(ziel) != 7:
            raise arm_mod.BewegungAbgelehnt("7 Gelenkwinkel erwartet")
        pose = kin.Pose.aus_matrix(kin.vorwaerts(ziel, self.arm.werkzeug)["werkzeug"]).als_dict()
        return self.arm.gelenke_anfahren(ziel, d.get("grund", "Gelenkwinkel"), pose_pruefen=pose).als_dict()

    def gelenk(self, d: dict) -> dict:
        return self.arm.gelenk_schritt(int(d.get("gelenk", 0)), float(d.get("d_deg", 0.0))).als_dict()

    def fahren(self, d: dict) -> dict:
        """Werkzeugpunkt auf eine Pose in Basiskoordinaten fahren (Meter, Grad). Fehlende x/y/z bleiben wie jetzt.
        Orientierung: theta_x/y/z_deg (Kinova-Euler wie die Pose im Zustand) oder gier_deg (Werkzeug senkrecht nach
        unten, Schließrichtung des Greifers um gier_deg gegen Basis-x gedreht); sonst unverändert."""
        ziel = self.arm.pose_matrix()
        for i, k in enumerate(("x_m", "y_m", "z_m")):
            if k in d:
                ziel[i, 3] = float(d[k])
        winkel = ("theta_x_deg", "theta_y_deg", "theta_z_deg")
        if any(k in d for k in winkel):
            ziel[:3, :3] = kin.euler_matrix(*(float(d[k]) for k in winkel))
        elif "gier_deg" in d:
            ziel[:3, :3] = kin.rot_z(math.radians(float(d["gier_deg"]))) @ kin.rot_x(math.pi)
        if not np.all(np.isfinite(ziel)):
            raise arm_mod.BewegungAbgelehnt("Zielpose enthält ungültige Zahlen")
        # hängt ein Würfel im Greifer, hält Arm.pose_anfahren seine Unterseite über dem Tisch (last_unter_spitze_m)
        p = self.ablauf.gegriffen
        greifer = p["greifer_ist"] if p is not None else (self.arm.zustand().get("greifer_position") or 0.0)
        geschw = float(d.get("geschw_m_s", arm_mod.GESCHW_MAX_M_S))
        if not math.isfinite(geschw) or geschw <= 0:
            raise arm_mod.BewegungAbgelehnt("Geschwindigkeit muss positiv sein")
        return self.arm.pose_anfahren(ziel, str(d.get("grund", "Pose anfahren"))[:60], geschw_m_s=geschw,
                                      greifer_position=greifer).als_dict()

    def bild(self, d: dict) -> dict:
        """Frisches Bild nach Stillstand mit allem, was zur Umrechnung in Basiskoordinaten nötig ist: Kamerapose
        (T_basis_kamera), Intrinsik, gemessene Armpose. Farbe als JPEG (Qualität 95) oder PNG, Tiefe als 16-bit-PNG
        in Millimetern; Bilddaten base64-kodiert."""
        import cv2

        name = d.get("strom", "farbe")
        if name not in self.leser:
            raise arm_mod.BewegungAbgelehnt(f"unbekannter Stream {name!r} (farbe oder tiefe)")
        bilder, t_bk = self._bild_und_kamera(name)
        b = bilder[-1]
        if name == "tiefe" or d.get("format") == "png":
            ok, daten = cv2.imencode(".png", b)
            fmt = "png"
        else:
            ok, daten = cv2.imencode(".jpg", b, [cv2.IMWRITE_JPEG_QUALITY, 95])
            fmt = "jpg"
        if not ok:
            raise RuntimeError("Bild ließ sich nicht kodieren")
        z = self.arm.zustand()
        return {"strom": name, "format": fmt, "breite": int(b.shape[1]), "hoehe": int(b.shape[0]),
                "einheit": "mm" if name == "tiefe" else "bgr8", "daten_base64": base64.b64encode(daten.tobytes()).decode(),
                "t_basis_kamera": t_bk.tolist(), "intrinsik": self.intrinsik[name], "pose": z["pose"],
                "gelenke_deg": z["gelenke_deg"], "zeit": time.time()}

    def home_speichern(self, _d: dict) -> dict:
        return arm_mod.home_speichern(self.home_pfad, self.arm.zustand())

    def park_speichern(self, _d: dict) -> dict:
        daten = arm_mod.home_speichern(self.park_pfad, self.arm.zustand())
        daten["hinweis"] = "Parkposition. Von hier zuerst Home anfahren"
        self.park_pfad.write_text(json.dumps(daten, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return daten

    def park_fahren(self, _d: dict) -> dict:
        """Gespeicherte Parkposition anfahren. Sie darf laut Modell hinter Anschlägen oder unter dem Gliederabstand
        liegen, weil der echte Arm dort stand (kollision.bahn_pruefen, bekannt); keine Arbeitsraumprüfung des Ziels,
        die Parkposition liegt nah an der Basis."""
        park = arm_mod.home_laden(self.park_pfad)
        if park is None:
            raise arm_mod.BewegungAbgelehnt("keine Parkposition gespeichert")
        return self.arm.gelenke_anfahren(park["gelenke_deg"], "Park", bekannt=[park["gelenke_deg"]]).als_dict()

    def home_fahren(self, _d: dict) -> dict:
        home = arm_mod.home_laden(self.home_pfad)
        if home is None:
            raise arm_mod.BewegungAbgelehnt("keine Home-Pose gespeichert")
        return self.arm.gelenke_anfahren(home["gelenke_deg"], "Home", pose_pruefen=home["pose"]).als_dict()

    def greifer(self, d: dict) -> dict:
        if "weite_mm" in d:
            weite = float(d["weite_mm"])
            if not math.isfinite(weite):
                raise arm_mod.BewegungAbgelehnt("Greiferweite ungültig")
            position = kin.greifer_position_fuer_weite(weite / 1000.0)
        else:
            position = float(d["position"])
        erg = self.arm.greifer(position, float(d.get("geschwindigkeit", 1.0)))
        p = self.ablauf.gegriffen
        if p is not None and erg.get("weite_mm") is not None and erg["weite_mm"] > p["weite_mm"] + 5.0:
            self.ablauf.losgelassen()  # Greifer von Hand geöffnet: der Würfel ist nicht mehr gegriffen
        return erg

    def stream(self, name: str, an: bool) -> dict:
        leser = self.leser[name]
        (leser.start if an else leser.stop)()
        return {"stream": name, "laeuft": leser.laeuft}

    def _bild_und_kamera(self, name: str, anzahl: int = 1, beruhigung_s: float | None = None):
        """Frisches Bild nach Stillstand plus Kamerapose aus der gemessenen Werkzeugpose. beruhigung_s: Wartezeit nach
        der letzten Bewegung (Standard BERUHIGUNG_S; kürzer nur zum Messen im Schnellbetrieb)."""
        if self.arm.zustand().get("bewegt"):
            raise arm_mod.BewegungAbgelehnt("Arm bewegt sich noch")
        warten = BERUHIGUNG_S if beruhigung_s is None else min(BERUHIGUNG_S, beruhigung_s)
        ruhe = warten - (time.time() - getattr(self.arm, "letzte_bewegung_ende", 0.0))
        if ruhe > 0:  # Videolatenz: kein Bild aus der Bewegung verwenden
            time.sleep(ruhe)
        bilder = self.leser[name].frisch(anzahl)
        t_ee_kamera = self.t_ee[name]
        t_basis_kamera = sehen.basis_kamera(self.arm.pose_matrix(), self.arm.werkzeug, t_ee_kamera)
        return bilder, t_basis_kamera

    def aufnahme(self, d: dict) -> dict:
        """Rohbild(er) nach Stillstand speichern (Tiefe: Median aus ``anzahl`` Bildern als .npy, Farbe als .png)
        zusammen mit gemessener Pose, Kamerapose und Intrinsik als .json. Für Auswertungen außerhalb des Servers,
        weil das Kameramodul je Stream nur einen Empfänger zulässt."""
        import cv2

        name = d.get("strom", "tiefe")
        if name not in self.leser:
            raise arm_mod.BewegungAbgelehnt(f"unbekannter Stream {name!r}")
        bilder, t_bk = self._bild_und_kamera(name, anzahl=max(1, min(20, int(d.get("anzahl", 5)))))
        stamm = MESSUNGEN / f"aufnahme_{name}_{time.strftime('%Y%m%d_%H%M%S')}"
        MESSUNGEN.mkdir(parents=True, exist_ok=True)
        if name == "tiefe":
            np.save(stamm.with_suffix(".npy"), np.median(np.stack(bilder), axis=0).astype(np.uint16))
            datei = stamm.with_suffix(".npy")
        else:
            cv2.imwrite(str(stamm.with_suffix(".png")), bilder[-1])
            datei = stamm.with_suffix(".png")
        info = {"datei": str(datei), "zustand": self.arm.zustand(), "t_basis_kamera": t_bk.tolist(),
                "intrinsik": self.intrinsik[name], "werkzeug": self.arm.werkzeug.tolist()}
        stamm.with_suffix(".json").write_text(json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
        return {"datei": str(datei), "json": str(stamm.with_suffix(".json"))}

    def tag_erkennen(self, d: dict) -> dict:
        """Alle AprilTags im Farbbild; gewählt wird der nächstgelegene oder der mit ``id``."""
        bilder, t_bk = self._bild_und_kamera("farbe")
        tags = sehen.tags_erkennen(bilder[-1], self.intrinsik["farbe"])
        if d.get("id") is not None:
            alle = tags
            tags = [t for t in tags if t.id == int(d["id"])]
            if not tags:
                self.letzter_tag = None
                return {"tags": [t.als_dict(t_bk) for t in alle], "gewaehlt": None,
                        "meldung": f"Tag {int(d['id'])} nicht im Bild"}
        if not tags:
            self.letzter_tag = None
            return {"tags": [], "gewaehlt": None, "meldung": "kein AprilTag im Bild"}
        tag = min(tags, key=lambda t: np.linalg.norm(t.t_kamera_tag[:3, 3]))
        self.letzter_tag = tag.als_dict(t_bk)
        return {"tags": [t.als_dict(t_bk) for t in tags], "gewaehlt": self.letzter_tag}

    def tag_anfahren(self, d: dict) -> dict:
        abstand = float(d.get("abstand_spitze_m", 0.10))
        if not math.isfinite(abstand) or abstand < 0.03 - 1e-9:
            raise arm_mod.BewegungAbgelehnt("Abstand der Fingerspitzen zum Tag mindestens 30 mm")
        if self.letzter_tag is None:
            raise arm_mod.BewegungAbgelehnt("erst Tag erkennen")
        t_tag = np.array(self.letzter_tag["basis_matrix"])
        greifer = self.arm.zustand().get("greifer_position") or 0.0
        if greifer < 0.95:
            raise arm_mod.BewegungAbgelehnt("Greifer erst schließen (Abstand gilt für geschlossene Finger)")
        senkrecht = d.get("ausrichtung", "senkrecht") != "tag"
        ziel = sehen.ziel_ueber_tag(t_tag, self.arm.pose_matrix(), self.arm.werkzeug, abstand, 1.0, senkrecht)
        e = self.arm.pose_anfahren(ziel, f"Fingerspitzen {abstand * 1000:.0f} mm über Tag {self.letzter_tag['id']}",
                                   geschw_m_s=0.03)
        return {**e.als_dict(), "ziel_spitze_ueber_tag_mm": abstand * 1000,
                "ausrichtung": "senkrecht" if senkrecht else "tag"}

    def wuerfel_scannen(self, d: dict) -> dict:
        """Würfel im aktuellen Tiefenbild vermessen (ohne Bewegung). bei_m [x, y]: den Würfel an dieser Stelle."""
        bei = d.get("bei_m")
        if bei is not None:
            bei = (float(bei[0]), float(bei[1]))
        return self.ablauf._ausfuehren("Würfel hier vermessen", self.ablauf.scannen_hier, bei)

    def wuerfel_anfahren_vermessen(self, d: dict) -> dict:
        """Tiefenkamera über bei_m [x, y] fahren und dort den Würfel bzw. Turm vermessen (auch für Teams)."""
        bei = d["bei_m"]
        return self.ablauf.api_anfahren_vermessen(float(bei[0]), float(bei[1]))

    def wuerfel_suchen(self, _d: dict) -> dict:
        """Würfel suchen, Kamera darüber fahren und vermessen. Nur Such- und Scanbewegungen, kein Griff."""
        return self.ablauf.api_suchen()

    @staticmethod
    def _bestaetigt(d: dict, was: str) -> None:
        if d.get("bestaetigt") is not True:
            raise arm_mod.BewegungAbgelehnt(f"{was} nur mit ausdrücklicher Bestätigung im Dashboard")

    def wuerfel_greifen(self, d: dict) -> dict:
        self._bestaetigt(d, "Greifen")
        return self.ablauf.api_greifen()

    def wuerfel_abstellen(self, d: dict) -> dict:
        self._bestaetigt(d, "Abstellen")
        return self.ablauf.api_abstellen()

    def wuerfel_ablauf(self, d: dict) -> dict:
        self._bestaetigt(d, "Der Greifablauf")
        return self.ablauf.api_alles(abstellen=d.get("abstellen", True) is not False)

    def wuerfel_stapeln(self, d: dict) -> dict:
        self._bestaetigt(d, "Das Stapeln")
        return self.ablauf.api_stapeln()

    def wuerfel_stapeln_schnell(self, d: dict) -> dict:
        """Stapeln im Schnellbetrieb (Fabian 11.11.2026), nur Betreuer: {"stufe": "1" … "max", Änderungen …}.
        Ergebnis mit Zeiten; zusätzlich als messungen/stapeln_schnell_<Zeit>.json gespeichert."""
        self._bestaetigt(d, "Das schnelle Stapeln")
        aenderungen = {k: v for k, v in d.items() if k not in ("bestaetigt", "stufe")}
        erg = self.ablauf.api_stapeln_schnell(str(d.get("stufe", "1")), **aenderungen)
        MESSUNGEN.mkdir(parents=True, exist_ok=True)
        pfad = MESSUNGEN / f"stapeln_schnell_{time.strftime('%Y%m%d_%H%M%S')}.json"
        pfad.write_text(json.dumps({**erg, "simulation": self.simulation is not None}, indent=2, ensure_ascii=False,
                                   default=str), encoding="utf-8")
        return {**erg, "datei": str(pfad)}

    def bahn_probe(self, d: dict) -> dict:
        """Probefahrt des Betreuers: ``hub_mm`` senkrecht hoch und zurück, mit Messung der Glätte (Arm.Fahrtmesser).
        art "bahn" (Standard): vorberechnete Gelenkbahn. Zeigt, ob die Firmware 2.2.0 sie annimmt und gegen welche Grenze
        sie prüft (Fehlerbericht in der Meldung). a_deg_s2: Beschleunigungsgrenze (Standard 40 °/s² · 0,9), rampe_s:
        Anfahr-/Bremsrampe (Standard nach Ruckgrenze, 0 = ohne, zum Vergleich).
        art "pose": dieselbe Fahrt mit ReachPose und v_m_s (Standard 0,10 m/s). So lässt sich vergleichen, wie sanft die
        Steuerung selbst anfährt und bremst (Fabian 12.11.2026: „nicht ruckeln“)."""
        art = str(d.get("art", "bahn"))
        if art not in ("bahn", "pose"):
            raise arm_mod.BewegungAbgelehnt("art: bahn oder pose")
        hub = float(d.get("hub_mm", 20.0)) / 1000
        if not math.isfinite(hub) or not 0.005 <= hub <= 0.10:
            raise arm_mod.BewegungAbgelehnt("Hub 5 bis 100 mm")
        a = d.get("a_deg_s2")
        a = None if a is None else float(a)
        if a is not None and not (math.isfinite(a) and 10.0 <= a <= 298.0):
            raise arm_mod.BewegungAbgelehnt("Beschleunigung 10 bis 298 °/s²")
        rampe = d.get("rampe_s")
        rampe = None if rampe is None else float(rampe)
        if rampe is not None and not (math.isfinite(rampe) and 0.0 <= rampe <= bahn_mod.RAMPE_MAX_S):
            raise arm_mod.BewegungAbgelehnt(f"Rampe 0 bis {bahn_mod.RAMPE_MAX_S} s")
        v = float(d.get("v_m_s", 0.10))
        if not (math.isfinite(v) and 0.01 <= v <= arm_mod.SCHNELL_MAX_M_S):
            raise arm_mod.BewegungAbgelehnt(f"Tempo 0,01 bis {arm_mod.SCHNELL_MAX_M_S} m/s")
        start = self.arm.pose_matrix()
        hoch = start.copy()
        hoch[2, 3] += hub
        with self.arm.schnell(v if art == "pose" else 0.10, 30.0):
            if art == "pose":
                e1 = self.arm.pose_anfahren(hoch, "Probe hoch (ReachPose)", geschw_m_s=v)
                e2 = self.arm.pose_anfahren(start, "Probe zurück (ReachPose)", geschw_m_s=v) if e1.ok else None
            else:
                e1 = self.arm.bahn_fahren([hoch], "Bahnprobe hoch", a_deg_s2=a, rampe_s=rampe)
                e2 = self.arm.bahn_fahren([start], "Bahnprobe zurück", a_deg_s2=a, rampe_s=rampe) if e1.ok else None
        return {"ok": e1.ok and e2 is not None and e2.ok, "art": art, "hoch": e1.als_dict(),
                "zurueck": None if e2 is None else e2.als_dict()}

    def wuerfel_verteilen(self, d: dict) -> dict:
        """Türme auf freie Plätze verteilen (nach Tempoversuchen), nur Betreuer: {"plaetze_mm": [[x, y], …], "stufe"}."""
        self._bestaetigt(d, "Das Verteilen")
        plaetze = d.get("plaetze_mm")
        if plaetze is None:
            plaetze_m = ablauf_mod.VERTEIL_PLAETZE_M
        else:
            plaetze_m = [(float(p[0]) / 1000, float(p[1]) / 1000) for p in plaetze]
            if not plaetze_m or not all(math.isfinite(v) for p in plaetze_m for v in p):
                raise arm_mod.BewegungAbgelehnt("Plätze ungültig")
        return self.ablauf.api_verteilen(plaetze_m, d.get("stufe"))

    def wuerfel_ablegen(self, d: dict) -> dict:
        self._bestaetigt(d, "Ablegen")
        gier = d.get("gier_deg")
        unten = d.get("unten_z_m")
        luft = d.get("luft_mm")
        loslassen = d.get("loslassen_mm")
        return self.ablauf.api_ablegen(float(d["x_m"]), float(d["y_m"]), None if gier is None else float(gier),
                                       None if unten is None else float(unten),
                                       ablauf_mod.ABSETZ_LUFT_M if luft is None else float(luft) / 1000,
                                       None if loslassen is None else float(loslassen) / 1000)


# --- HTTP ---------------------------------------------------------------------------------------------------------


TEAM_KOPF = "X-Kinova-Schluessel"
FAHR_KOPF = "X-Kinova-Fahrrecht"
# Stufen: lesen = Schlüssel genügt; fahrrecht = Schlüssel + Fahrrecht (verändert gemeinsame Erkennungsergebnisse);
# fahren = zusätzlich Freigabe des Betreuers; betreuer = nur am Mac Mini selbst.
LESEN, FAHRRECHT, FAHREN, BETREUER = "lesen", "fahrrecht", "fahren", "betreuer"


def _keine_zahl(text: str):
    raise ValueError(f"{text} ist keine erlaubte Zahl")


def ist_lokal(adresse: str) -> bool:
    return adresse in ("127.0.0.1", "::1", "::ffff:127.0.0.1")


def handler_klasse(db: Dashboard, port: int, lokal=ist_lokal):
    erlaubte_hosts = {f"127.0.0.1:{port}", f"localhost:{port}", f"[::1]:{port}"}
    lokale_namen = {"127.0.0.1", "localhost", "[::1]"}
    nz = db.netz
    routen = {
        "/api/schritt": (db.schritt, FAHREN), "/api/gelenk": (db.gelenk, FAHREN),
        "/api/gelenke": (db.gelenke, FAHREN), "/api/fahren": (db.fahren, FAHREN),
        "/api/home/fahren": (db.home_fahren, FAHREN), "/api/greifer": (db.greifer, FAHREN),
        "/api/apriltag/anfahren": (db.tag_anfahren, FAHREN), "/api/wuerfel/suchen": (db.wuerfel_suchen, FAHREN),
        "/api/wuerfel/greifen": (db.wuerfel_greifen, FAHREN), "/api/wuerfel/abstellen": (db.wuerfel_abstellen, FAHREN),
        "/api/wuerfel/ablegen": (db.wuerfel_ablegen, FAHREN), "/api/wuerfel/ablauf": (db.wuerfel_ablauf, FAHREN),
        "/api/wuerfel/stapeln": (db.wuerfel_stapeln, FAHREN),
        "/api/wuerfel/stapeln_schnell": (db.wuerfel_stapeln_schnell, BETREUER),
        "/api/wuerfel/verteilen": (db.wuerfel_verteilen, BETREUER),
        "/api/bahn/probe": (db.bahn_probe, BETREUER),
        "/api/wuerfel/anfahren_vermessen": (db.wuerfel_anfahren_vermessen, FAHREN),
        "/api/apriltag/erkennen": (db.tag_erkennen, FAHRRECHT), "/api/wuerfel/scannen": (db.wuerfel_scannen, FAHRRECHT),
        "/api/stream/farbe/aus": (lambda _d: db.stream("farbe", False), FAHRRECHT),
        "/api/stream/tiefe/aus": (lambda _d: db.stream("tiefe", False), FAHRRECHT),
        "/api/bild": (db.bild, LESEN),
        "/api/stream/farbe/an": (lambda _d: db.stream("farbe", True), LESEN),
        "/api/stream/tiefe/an": (lambda _d: db.stream("tiefe", True), LESEN),
        "/api/home/speichern": (db.home_speichern, BETREUER), "/api/aufnahme": (db.aufnahme, BETREUER),
        "/api/park/speichern": (db.park_speichern, BETREUER), "/api/park/fahren": (db.park_fahren, BETREUER),
        "/api/fehler_quittieren": (lambda _d: (db.arm.fehler_quittieren(), {"ok": True})[1], BETREUER),
        "/api/verbindung/neu": (lambda _d: db.arm.verbindung_neu(), BETREUER),
        "/api/verbindung/trennen": (lambda _d: db.arm.verbindung_trennen(), BETREUER),  # nur zum Prüfen des Neuaufbaus
        "/api/kalibrierung/aufnehmen": (db.kalibrierung.api_aufnehmen, BETREUER),
        "/api/kalibrierung/rechnen": (db.kalibrierung.api_rechnen, BETREUER),
        "/api/kalibrierung/uebernehmen": (db.kalibrierung.api_uebernehmen, BETREUER),
        "/api/kalibrierung/werkswerte": (db.kalibrierung.api_werkswerte, BETREUER),
        "/api/netz/freigabe": (lambda d: nz.freigabe_setzen(d.get("an") is True), BETREUER),
        "/api/netz/entziehen": (lambda _d: nz.entziehen(), BETREUER),
    }
    if getattr(db, "sim_szene", None) is not None:
        routen["/api/sim/szene"] = (db.sim_szene, LESEN)  # nur in der Simulation: Würfel neu hinlegen
    bestaetigung = {"/api/wuerfel/greifen", "/api/wuerfel/abstellen", "/api/wuerfel/ablegen", "/api/wuerfel/ablauf",
                    "/api/wuerfel/stapeln"}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        team = False  # Anfrage trägt einen gültigen Team-Schlüssel: Antwort mit CORS-Freigabe

        def log_message(self, *args) -> None:
            pass

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")

        def _json(self, code: int, daten) -> None:
            text = json.dumps(daten, ensure_ascii=False, default=lambda o: o.tolist()).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(text)))
            self.send_header("Cache-Control", "no-store")
            if self.team or code == 401:
                self._cors()
            self.end_headers()
            self.wfile.write(text)

        def _host_ok(self) -> bool:
            # Lokaler Name mit beliebigem Port: auch ein SSH-Tunnel auf einen anderen lokalen Port (z. B. 18080,
            # damit 8080 der Simulation bleibt) zählt als Mac Mini. DNS-Rebinding bleibt ausgeschlossen, weil dort der
            # fremde Domainname im Host steht.
            host = self.headers.get("Host", "")
            name, _, port = host.rpartition(":")
            return host in erlaubte_hosts or (name in lokale_namen and port.isdigit())

        def _rolle(self, schluessel: str | None) -> str:
            """betreuer oder team; KeinZugang, wenn weder lokal noch mit gültigem Schlüssel."""
            if schluessel is not None:
                nz.schluessel_pruefen(schluessel)
                self.team = True
                return "team"
            if lokal(self.client_address[0]) and self._host_ok():
                return "betreuer"
            raise netz_mod.KeinZugang(f"Team-Schlüssel fehlt (Kopfzeile {TEAM_KOPF})")

        def do_OPTIONS(self) -> None:
            """CORS-Vorabfrage für Team-Seiten im Browser. X-Kinova (Betreuer-Kopfzeile) ist bewusst nicht erlaubt."""
            self.send_response(204)
            self._cors()
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", f"Content-Type, {TEAM_KOPF}, {FAHR_KOPF}")
            self.send_header("Access-Control-Max-Age", "600")
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self) -> None:
            self.team = False
            url = urlparse(self.path)
            pfad = url.path
            if pfad.startswith(("/api/", "/video/")):
                q = parse_qs(url.query)
                schluessel = self.headers.get(TEAM_KOPF) or (q.get("schluessel") or [None])[0]
                marke = self.headers.get(FAHR_KOPF) or (q.get("fahrrecht") or [None])[0]
                try:
                    rolle = self._rolle(schluessel)
                except netz_mod.KeinZugang as exc:
                    return self._json(401, {"fehler": str(exc)})
                if pfad == "/api/zustand":  # verlängert das Fahrrecht nicht (Befehle zählen, Abfragen nicht)
                    z = {**db.zustand(), "rolle": rolle, "netz": nz.status(betreuer=rolle == "betreuer", marke=marke)}
                    if rolle == "team" and db.simulation is not None:  # Teams sehen die wahren Würfellagen nicht
                        z["simulation"] = db.simulation.info(wahrheit=False)
                    return self._json(200, z)
                if pfad in ("/video/farbe.mjpg", "/video/tiefe.mjpg"):
                    return self._mjpeg(pfad.split("/")[-1].split(".")[0])
                return self._json(404, {"fehler": "nicht gefunden"})
            datei = (STATIC / ("index.html" if pfad == "/" else pfad.lstrip("/"))).resolve()
            if STATIC not in datei.parents or not datei.is_file():
                return self._json(404, {"fehler": "nicht gefunden"})
            inhalt = datei.read_bytes()
            self.send_response(200)
            typ = mimetypes.guess_type(datei.name)[0] or "application/octet-stream"
            self.send_header("Content-Type", "text/javascript" if datei.suffix == ".js" else typ)
            self.send_header("Content-Length", str(len(inhalt)))
            self.end_headers()
            self.wfile.write(inhalt)

        def do_POST(self) -> None:
            self.team = False
            pfad = urlparse(self.path).path
            try:
                laenge = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                laenge = -1
            if not 0 <= laenge <= ANFRAGE_MAX_BYTES:
                self.close_connection = True
                return self._json(413, {"fehler": f"Anfrage zu groß (höchstens {ANFRAGE_MAX_BYTES} Byte)"})
            roh = self.rfile.read(laenge)
            try:
                rolle = self._rolle(self.headers.get(TEAM_KOPF))
            except netz_mod.KeinZugang as exc:
                return self._json(401, {"fehler": str(exc)})
            if rolle == "betreuer" and self.headers.get("X-Kinova") != "1":
                return self._json(403, {"fehler": "nur vom Dashboard (Kopfzeile X-Kinova fehlt)"})
            try:
                daten = json.loads(roh or b"{}", parse_constant=_keine_zahl)
            except (ValueError, UnicodeDecodeError):  # auch NaN/Infinity: Vergleiche mit NaN sind immer falsch
                return self._json(400, {"fehler": "kein gültiges JSON (NaN und Infinity sind nicht erlaubt)"})
            if not isinstance(daten, dict):
                return self._json(400, {"fehler": "JSON-Objekt erwartet"})
            marke = self.headers.get(FAHR_KOPF)
            if pfad == "/api/stopp":  # vor allem anderen, ohne Fahrrecht
                nz.stopp_gedrueckt(rolle == "betreuer", marke)
                db.arm.anhalten()
                return self._json(200, {"ok": True})
            db.arm.auftrag_beginnen()  # jeder spätere STOPP beendet diesen Auftrag (siehe Arm.gestoppt)
            beginn = time.time()

            def mit_hinweis(erg):
                """Abgebrochene Fahrt: sagen, wer STOPP gedrückt hat (Probelauf 11.11.2026)."""
                h = nz.stopp_hinweis(beginn)
                if h and isinstance(erg, dict) and erg.get("ok") is False:
                    erg = {**erg, "meldung": f"{erg.get('meldung', '')} – {h}", "stopp": h}
                return erg

            try:
                if pfad == "/api/fahrrecht/anmelden":
                    return self._json(200, nz.anmelden(daten.get("team", ""), marke))
                if pfad == "/api/fahrrecht/abmelden":
                    return self._json(200, nz.abmelden(marke))
                if pfad not in routen:
                    return self._json(404, {"fehler": "unbekannte Aktion"})
                funktion, stufe = routen[pfad]
                if rolle == "betreuer":
                    return self._json(200, mit_hinweis(funktion(daten)))
                if stufe == BETREUER:
                    raise netz_mod.NichtErlaubt("nur direkt am Mac Mini möglich")
                if stufe == FAHRRECHT:
                    nz.fahrrecht_pruefen(marke)
                if stufe != FAHREN:
                    return self._json(200, funktion(daten))
                f = nz.fahren_beginnen(marke)
                try:
                    if pfad in bestaetigung:  # die Freigabe des Betreuers ersetzt die Bestätigung im Dashboard
                        daten["bestaetigt"] = True
                    return self._json(200, mit_hinweis(funktion(daten)))
                finally:
                    nz.fahren_beenden(f)
            except netz_mod.NichtErlaubt as exc:
                return self._json(403, {"fehler": str(exc), "abgelehnt": True})
            except arm_mod.BewegungAbgelehnt as exc:
                h = nz.stopp_hinweis(beginn)
                return self._json(409, {"fehler": f"{exc} – {h}" if h else str(exc), "abgelehnt": True,
                                        **({"stopp": h} if h else {})})
            except ValueError as exc:  # z. B. kein Würfel im Bild: nichts bewegt
                return self._json(409, {"fehler": str(exc), "abgelehnt": True})
            except (KeyError, TypeError, IndexError) as exc:
                return self._json(400, {"fehler": f"Eingabe fehlt oder hat den falschen Typ: {exc}"})
            except Exception as exc:  # noqa: BLE001 - Fehler sichtbar ans Dashboard melden statt Thread-Abbruch
                traceback.print_exc()
                if isinstance(exc, (TimeoutError, concurrent.futures.TimeoutError, ConnectionError)):
                    return self._json(503, {"fehler": f"keine Antwort vom Arm ({type(exc).__name__}). Der Server baut "
                                                      "die Verbindung selbst neu auf; danach Zustand prüfen und neu "
                                                      "starten", "verbindung": True})
                return self._json(500, {"fehler": f"{type(exc).__name__}: {exc}"})
            finally:
                db.arm.auftrag_beenden()

        def _mjpeg(self, name: str) -> None:
            import cv2

            leser = db.leser[name]
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=bild")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            zaehler = -1
            try:
                while leser.laeuft:
                    bild, _, zaehler = leser.naechstes(zaehler, 2.0)
                    if bild is None:
                        continue
                    if name == "tiefe":
                        bild = kamera.tiefe_einfaerben(bild)
                    else:
                        bild = sehen_overlay(bild, db)
                    ok, jpg = cv2.imencode(".jpg", bild, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok:
                        continue
                    self.wfile.write(b"--bild\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            self.close_connection = True

    return Handler


def sehen_overlay(bild: np.ndarray, db: Dashboard) -> np.ndarray:
    """Zuletzt erkannten Tag ins Farbbild zeichnen (nur Anzeige)."""
    import cv2

    if db.letzter_tag is None:
        return bild
    bild = bild.copy()
    ecken = np.array(db.letzter_tag["ecken_px"], dtype=np.int32)
    cv2.polylines(bild, [ecken], True, (0, 255, 0), 2)
    return bild


def argumente(argv: list[str] | None = None):
    p = argparse.ArgumentParser(description="Kinova-Dashboard")
    try:
        import kinova_lesen as kl

        kl.verbindungs_argumente(p)
    except ImportError:  # Team-Paket: nur Simulation, ohne die Werkzeuge für den echten Arm
        pass
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--netz", action="store_true",
                   help="auch aus dem Netz erreichbar (Team-Laptops mit Schlüssel; Fahren erst nach Freigabe)")
    p.add_argument("--neuer-schluessel", action="store_true", help="neuen Team-Schlüssel erzeugen")
    p.add_argument("--wuerfel-mm", type=float, default=WUERFEL_KANTE_M * 1000,
                   help="Kantenlänge der Würfel für Suche und Griff (Standard 60 mm, Holzwürfel der Teams)")
    p.add_argument("--tisch-mm", type=float, default=TISCH_Z_MM,
                   help="Tischplatte im Basissystem für die Arbeitsraumgrenze (Standard −25 mm: gemessen −26,0 mm am "
                        "11.11.2026; an einem anderen Tisch neu messen). Gilt auch für --sim, deren Tisch bei −27 mm liegt")
    p.add_argument("--sim", action="store_true", help="Simulation ohne Arm (keine Kortex-API nötig)")
    p.add_argument("--zeitraffer", type=float, default=1.0, help="nur --sim: so viel schneller als echt")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    a = argumente(argv)
    if not 30.0 <= a.wuerfel_mm <= 80.0:
        print("--wuerfel-mm: 30 bis 80 mm (Greifer öffnet höchstens 85 mm)")
        return 1
    if not -40.0 <= a.tisch_mm <= 0.0:
        print("--tisch-mm: −40 bis 0 mm")
        return 1
    if not port_frei(a.port, "0.0.0.0" if a.netz else "127.0.0.1"):
        print(f"Port {a.port} ist belegt: läuft noch ein Dashboard? Dort mit Strg-C beenden.")
        return 1
    if a.sim:
        import sim

        return sim.betreiben(a)
    if not hasattr(a, "ip"):
        print("Ohne kinova/tools/kinova_lesen.py geht nur die Simulation: --sim")
        return 1
    import kinova_lesen as kl

    k = kl._kortex()
    benutzer, passwort = kl.zugangsdaten(a.ip, a.benutzer, a.passwort)
    try:
        return _betreiben(k, a, benutzer, passwort)
    except k["fehlerarten"] as exc:
        if "INVALID_PASSWORD" in str(exc) or "INVALID_USER" in str(exc):
            print(f"Anmeldung abgelehnt: Benutzer oder Passwort für {a.ip} falsch.")
        else:
            print(f"Verbindung zum Arm fehlgeschlagen: {type(exc).__name__}: {exc}")
        return 2


def port_frei(port: int, adresse: str = "127.0.0.1") -> bool:
    import socket

    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # wie HTTPServer: TIME_WAIT nach Neustart zulassen
        try:
            s.bind((adresse, port))
        except OSError:
            return False
    return True


def netz_adressen(port: int) -> list[str]:
    """IPv4-Adressen dieses Rechners außer 127.x und dem Armnetz 192.168.1.x (Anzeige für die Teams)."""
    import socket

    adressen = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            adressen.add(info[4][0])
    except OSError:
        pass
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:  # Absenderadresse der Standardroute, sendet nichts
        try:
            s.connect(("10.255.255.255", 1))
            adressen.add(s.getsockname()[0])
        except OSError:
            pass
    gefiltert = sorted(a for a in adressen if not a.startswith(("127.", "192.168.1.")))
    namen = [f"{socket.gethostname().split('.')[0]}.local:{port}"]  # Bonjour-Name; gethostname() kann .fritz.box tragen
    return [f"{a}:{port}" for a in gefiltert] + namen


def server_starten(db: Dashboard, a) -> ThreadingHTTPServer:
    """HTTP-Server binden und die Zugangsdaten für die Teams ausgeben."""
    if a.netz:
        db.netz.adressen = netz_adressen(a.port)
    server = ThreadingHTTPServer(("0.0.0.0" if a.netz else "127.0.0.1", a.port), handler_klasse(db, a.port))
    server.daemon_threads = True
    print(f"Dashboard bereit: http://127.0.0.1:{a.port}  (Strg-C beendet; Arm bleibt stehen)")
    if a.netz:
        print("Team-Laptops: " + ", ".join(f"http://{x}" for x in db.netz.adressen))
    print(f"Team-Schlüssel: {db.netz.schluessel}   Fahren übers Netz: "
          + ("freigegeben" if db.netz.freigabe else "gesperrt (im Dashboard freigeben)"))
    return server


ALTE_VERBINDUNG_WARTEZEIT_S = 3.0


def _alte_verbindung_schliessen(stapel: contextlib.ExitStack, router) -> None:
    """Alte Verbindung schließen, bevor die neue aufgeht: Solange die alte Sitzung lebt, behält sie die Steuerung, die neue
    bekam am Arm (12.11.2026) „SESSION_NOT_IN_CONTROL“. Erst sauber (CloseSession, 1 s), hängt das (Empfangsthread der
    Kortex-Bibliothek mitten in einem Rahmen), nach ALTE_VERBINDUNG_WARTEZEIT_S den Socket hart schließen."""
    fertig = threading.Event()

    def schliessen():
        with contextlib.suppress(Exception):
            stapel.close()  # CloseSession, dann transport.disconnect; bei toter Verbindung scheitert beides
        fertig.set()

    threading.Thread(target=schliessen, name="alte-verbindung", daemon=True).start()
    fertig.wait(ALTE_VERBINDUNG_WARTEZEIT_S)
    if router is not None:
        with contextlib.suppress(Exception):
            router.transport.sock.close()  # disconnect lässt ihn offen, wenn shutdown scheitert
    fertig.wait(1.0)


@contextlib.contextmanager
def verbindung_mit_neuaufbau(k: dict, ip: str, benutzer: str, passwort: str):
    """Wie kinova_lesen.verbindung, liefert dazu ``neu_verbinden`` für Arm: schließt die alte Verbindung, öffnet eine
    neue mit Sitzung und gibt ihren Router zurück. Die Zugangsdaten bleiben dafür im Speicher dieses Prozesses (nie im
    Protokoll)."""
    import kinova_lesen as kl

    offen = {"stapel": contextlib.ExitStack()}
    offen["router"] = offen["stapel"].enter_context(kl.verbindung(k, ip, benutzer, passwort))

    def neu_verbinden():
        alt = (offen["stapel"], offen["router"])
        offen["stapel"], offen["router"] = contextlib.ExitStack(), None
        _alte_verbindung_schliessen(*alt)
        neu = contextlib.ExitStack()
        router = neu.enter_context(kl.verbindung(k, ip, benutzer, passwort))
        offen["stapel"], offen["router"] = neu, router
        return router

    try:
        yield offen["router"], neu_verbinden
    finally:
        offen["stapel"].close()


def _betreiben(k: dict, a, benutzer: str, passwort: str) -> int:
    with verbindung_mit_neuaufbau(k, a.ip, benutzer, passwort) as (router, neu_verbinden):
        arm = arm_mod.Arm(k, router, arbeitsraum=kin.Arbeitsraum(tisch_z_m=a.tisch_mm / 1000.0),
                          protokoll=MESSUNGEN / f"dashboard_{time.strftime('%Y%m%d')}.jsonl",
                          neu_verbinden=neu_verbinden)
        intr = arm.kamera_intrinsik()
        arm.start()
        leser = {n: kamera.Leser(s) for n, s in kamera.stroeme(a.ip).items()}
        schluessel_pfad = KONFIG / "netz.json"
        if a.neuer_schluessel and schluessel_pfad.exists():
            schluessel_pfad.unlink()
        db = Dashboard(arm, leser, intr, schluessel=netz_mod.schluessel_laden(schluessel_pfad))
        db.wuerfel_kante_m = a.wuerfel_mm / 1000.0
        kal = kalibrierung.laden(db.kamera_konfig_pfad)
        if kal is not None:
            db.farbkamera_setzen(kal)
            print(f"Farbkamera: Kalibrierung vom {kal['kalibriert']} ({db.kamera_konfig_pfad.name})")
        server = server_starten(db, a)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            for l in leser.values():
                l.stop()
            arm.stop()
            server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
