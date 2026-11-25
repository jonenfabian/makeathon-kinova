"""Zugang zum Kinova-Arbeitsplatz für die Teams: echter Arm oder Simulation, gleiche Befehle.

Nur Python-Standardbibliothek; für Bilder zusätzlich numpy und OpenCV. Längen in Millimetern, Winkel in Grad,
alles im Basis-Koordinatensystem des Roboters (x nach vorn, y nach links, z nach oben, Ursprung in der Basis).

    from kinova_team import Roboter

    with Roboter() as r:          # Verbindung aus verbindung.json, sonst die Simulation auf diesem Rechner
        r.home()
        for t in r.tags():
            print(t["id"], t["x_mm"], t["y_mm"])

Jeder Befehl wartet, bis der Arm fertig ist. Strg-C hält den Arm an. Lehnt der Server einen Befehl ab oder bricht eine
Fahrt ab (STOPP, Gelenkanschlag, Kollision), kommt ``Abgelehnt`` mit dem Grund. Der Arm steht dann irgendwo zwischen
Start und Ziel: vor dem nächsten Befehl die Pose lesen. Drückt jemand am Mac Mini oder ein anderes Team STOPP,
verliert ihr das Fahrrecht und der nächste Befehl bringt ``KeinZugang``.
"""

from __future__ import annotations

import base64
import json
import math
import os
import urllib.error
import urllib.request
from pathlib import Path

STANDARD = {"adresse": "127.0.0.1:8080", "schluessel": "sim", "team": "Team"}
VERBINDUNG_DATEI = "verbindung.json"


class KinovaFehler(Exception):
    """Oberbegriff aller Fehler dieser Bibliothek."""


class Abgelehnt(KinovaFehler):
    """Der Server hat den Befehl abgelehnt (Arbeitsraum, Gelenkanschlag, kein Würfel im Bild ...)."""


class KeinZugang(KinovaFehler):
    """Schlüssel falsch, kein Fahrrecht oder Fahren nicht freigegeben."""


def verbindung_laden() -> dict:
    """Adresse, Schlüssel und Teamname: Umgebungsvariablen KINOVA_ADRESSE/KINOVA_SCHLUESSEL/KINOVA_TEAM, sonst
    ``verbindung.json`` im Projektordner (aktueller Ordner), sonst die Simulation auf diesem Rechner.

    Eine zweite Datei im Bibliotheksordner wird nicht still bevorzugt: Liegen zwei verschiedene Dateien vor, bricht
    das Laden ab. So fährt nach dem Löschen der Projektdatei nicht unbemerkt eine alte Datei den echten Arm.
    """
    daten = dict(STANDARD)
    dateien = []
    for ordner in (Path.cwd(), Path(__file__).resolve().parents[1]):
        datei = (ordner / VERBINDUNG_DATEI).resolve()
        if datei.is_file() and datei not in dateien:
            dateien.append(datei)
    if len(dateien) > 1:
        raise KinovaFehler(f"Zwei {VERBINDUNG_DATEI} gefunden: {dateien[0]} und {dateien[1]}. Nur die Datei im "
                           "Projektordner behalten, die andere löschen.")
    if dateien:
        try:
            # utf-8-sig: Windows-Editoren schreiben gern ein BOM an den Anfang.
            daten.update(json.loads(dateien[0].read_text(encoding="utf-8-sig")))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise KinovaFehler(f"{dateien[0]} ist kein gültiges JSON in UTF-8 ({exc}). Datei aus "
                               "verbindung.beispiel.json neu anlegen.") from None
    for feld in ("adresse", "schluessel", "team"):
        wert = os.environ.get(f"KINOVA_{feld.upper()}")
        if wert:
            daten[feld] = wert
    return daten


def _mm(m: float) -> float:
    return round(float(m) * 1000.0, 1)


class Bild:
    """Kamerabild mit allem, was zum Umrechnen in Basiskoordinaten nötig ist.

    bild: numpy-Array (Farbe: BGR uint8 720×1280×3, Tiefe: uint16 in mm 270×480, 0 = keine Messung)
    t_basis_kamera: 4×4-Matrix (Meter), Kamera-Frame: z nach vorn aus der Linse, x nach rechts, y nach unten im Bild
    intrinsik: fx, fy, cx, cy in Pixeln
    """

    def __init__(self, daten: dict):
        import cv2
        import numpy as np

        roh = np.frombuffer(base64.b64decode(daten["daten_base64"]), dtype=np.uint8)
        self.strom = daten["strom"]
        self.bild = cv2.imdecode(roh, cv2.IMREAD_UNCHANGED)
        self.t_basis_kamera = np.array(daten["t_basis_kamera"], dtype=float)
        self.intrinsik = daten["intrinsik"]
        self.pose = daten["pose"]
        self.zeit = daten.get("zeit")

    def pixel_zu_basis(self, u: float, v: float, tiefe_mm: float) -> tuple[float, float, float]:
        """Bildpunkt (u nach rechts, v nach unten, Pixel) mit Abstand ``tiefe_mm`` entlang der Kameraachse →
        Punkt in Basiskoordinaten (mm)."""
        import numpy as np

        i = self.intrinsik
        z = tiefe_mm / 1000.0
        p_kamera = np.array([(u - i["cx"]) * z / i["fx"], (v - i["cy"]) * z / i["fy"], z, 1.0])
        p = self.t_basis_kamera @ p_kamera
        return _mm(p[0]), _mm(p[1]), _mm(p[2])

    def tiefe_farbig(self, nah_mm: int = 150, fern_mm: int = 1000):
        """Tiefenbild als Farbbild zum Ansehen (nah rot, fern blau, keine Messung schwarz). Das 16-Bit-Bild selbst
        sieht in Bildbetrachtern fast schwarz aus, weil seine Werte (mm) nur einen kleinen Teil von 0–65535 belegen."""
        import cv2
        import numpy as np

        if self.strom != "tiefe":
            raise ValueError("nur für das Tiefenbild")
        skaliert = np.clip((self.bild.astype(np.float32) - nah_mm) * (255.0 / (fern_mm - nah_mm)), 0, 255)
        farbig = cv2.applyColorMap(255 - skaliert.astype(np.uint8), cv2.COLORMAP_TURBO)
        farbig[self.bild == 0] = 0
        return farbig

    def speichern(self, pfad) -> list[str]:
        """Als PNG speichern. Beim Tiefenbild zusätzlich ``<name>_farbig.png`` zum Ansehen; die PNG selbst behält die
        Rohwerte in mm (16 Bit) für eigene Auswertungen. Liefert die geschriebenen Pfade."""
        from pathlib import Path

        import cv2

        pfad = Path(pfad).with_suffix(".png")
        cv2.imwrite(str(pfad), self.bild)
        pfade = [str(pfad)]
        if self.strom == "tiefe":
            farbig = pfad.with_name(pfad.stem + "_farbig.png")
            cv2.imwrite(str(farbig), self.tiefe_farbig())
            pfade.append(str(farbig))
        return pfade

    def basis_zu_pixel(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float] | None:
        """Punkt in Basiskoordinaten (mm) → Bildpunkt (u, v); None, wenn er hinter der Kamera liegt."""
        import numpy as np

        p = np.linalg.inv(self.t_basis_kamera) @ np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, 1.0])
        if p[2] <= 0:
            return None
        i = self.intrinsik
        return float(i["fx"] * p[0] / p[2] + i["cx"]), float(i["fy"] * p[1] / p[2] + i["cy"])


class Roboter:
    def __init__(self, adresse: str | None = None, schluessel: str | None = None, team: str | None = None,
                 zeitlimit_s: float = 180.0):
        v = verbindung_laden()
        adresse = adresse or v["adresse"]
        self.basis_url = adresse if adresse.startswith("http") else f"http://{adresse}"
        self.basis_url = self.basis_url.rstrip("/")
        self.schluessel = schluessel or v["schluessel"]
        self.team = team or v["team"]
        self.zeitlimit_s = zeitlimit_s
        self._fahrrecht: str | None = None
        self._stoppt = False

    def __repr__(self) -> str:
        return f"Roboter({self.basis_url!r}, team={self.team!r})"

    # --- HTTP -------------------------------------------------------------------------------------------------
    def _anfrage(self, pfad: str, daten: dict | None = None, zeitlimit_s: float | None = None) -> dict:
        kopf = {"Content-Type": "application/json", "X-Kinova-Schluessel": self.schluessel}
        if self._fahrrecht:
            kopf["X-Kinova-Fahrrecht"] = self._fahrrecht
        try:
            text = None if daten is None else json.dumps(daten, allow_nan=False).encode()
        except ValueError:
            raise Abgelehnt(f"ungültige Zahl (NaN oder unendlich) im Befehl {pfad}") from None
        anfrage = urllib.request.Request(self.basis_url + pfad, headers=kopf, data=text,
                                         method="GET" if daten is None else "POST")
        try:
            with urllib.request.urlopen(anfrage, timeout=zeitlimit_s or self.zeitlimit_s) as antwort:
                return json.loads(antwort.read())
        except urllib.error.HTTPError as exc:
            try:
                text = json.loads(exc.read()).get("fehler", str(exc))
            except (ValueError, AttributeError):
                text = str(exc)
            if exc.code in (401, 403):
                raise KeinZugang(text) from None
            if exc.code == 409:
                raise Abgelehnt(text) from None
            raise KinovaFehler(f"HTTP {exc.code}: {text}") from None
        except urllib.error.URLError as exc:
            raise KinovaFehler(f"Server {self.basis_url} nicht erreichbar ({exc.reason}). Läuft die Simulation "
                               "(python start.py kinova) bzw. stimmt die Adresse in verbindung.json?") from None
        except TimeoutError:
            self._stopp_leise()
            raise KinovaFehler(f"keine Antwort nach {zeitlimit_s or self.zeitlimit_s:.0f} s, STOPP gesendet") from None
        except KeyboardInterrupt:
            self._stopp_leise()
            raise

    def _stopp_leise(self) -> None:
        """Nach Strg-C: einmal STOPP senden, Fehler dabei schlucken (das Programm bricht ohnehin ab)."""
        if self._stoppt:
            return
        self._stoppt = True
        try:
            self._anfrage("/api/stopp", {}, zeitlimit_s=3.0)
        except BaseException:  # noqa: BLE001, S110 - beim Abbruch zählt nur der Versuch
            pass
        finally:
            self._stoppt = False

    def _fahren(self, pfad: str, daten: dict, zeitlimit_s: float | None = None) -> dict:
        """Befehl, der Fahrrecht braucht: bei Bedarf erst anmelden, nach verfallenem Fahrrecht einmal erneut. Ein
        entzogenes Fahrrecht (STOPP, Entzug am Mac Mini) wird nicht automatisch neu geholt. Abgebrochene Fahrten → Abgelehnt."""
        if self._fahrrecht is None:
            self.anmelden()
        try:
            antwort = self._anfrage(pfad, daten, zeitlimit_s)
        except KeinZugang as exc:
            if "kein Fahrrecht" not in str(exc):
                raise
            self.anmelden()
            antwort = self._anfrage(pfad, daten, zeitlimit_s)
        if isinstance(antwort, dict) and antwort.get("ok") is False:
            raise Abgelehnt(f"{pfad}: {antwort.get('meldung') or antwort.get('ausgang') or 'nicht ausgeführt'}")
        return antwort

    # --- Fahrrecht --------------------------------------------------------------------------------------------
    def anmelden(self) -> None:
        """Fahrrecht holen (oder das eigene verlängern). Es fährt immer nur ein Team; das Recht verfällt nach einer
        Minute ohne Befehl."""
        try:
            antwort = self._anfrage("/api/fahrrecht/anmelden", {"team": self.team})
        except KinovaFehler:
            self._fahrrecht = None
            raise
        self._fahrrecht = antwort["fahrrecht"]
        if not antwort.get("freigabe"):
            print("Hinweis: Fahren ist noch nicht freigegeben. Am Mac Mini „Fahren für Teams freigeben“ einschalten.")

    def abmelden(self) -> None:
        if self._fahrrecht:
            try:
                self._anfrage("/api/fahrrecht/abmelden", {})
            finally:
                self._fahrrecht = None

    def __enter__(self) -> Roboter:  # noqa: PYI034 - Python 3.10 kennt typing.Self nicht
        # Immer sichtbar machen, wohin das Programm fährt (Schlüssel "sim" = Simulation auf diesem Rechner).
        ziel = "Simulation" if self.schluessel == STANDARD["schluessel"] else "ECHTER ARM"
        print(f"Verbunden mit: {ziel} ({self.basis_url})", flush=True)
        self.anmelden()
        return self

    def __exit__(self, *_) -> None:
        self.abmelden()

    # --- Lesen ------------------------------------------------------------------------------------------------
    def zustand(self) -> dict:
        """Alles, was der Server weiß (Rohdaten in Metern), z. B. armzustand, bewegt, greifer_weite_mm."""
        return self._anfrage("/api/zustand")

    def pose(self) -> dict:
        """Werkzeugpunkt (zwischen den Fingern, 12 cm unter dem Flansch) in mm und Kinova-Winkeln in Grad."""
        p = self.zustand()["pose"]
        return {"x_mm": _mm(p["x"]), "y_mm": _mm(p["y"]), "z_mm": _mm(p["z"]),
                "theta_x_deg": round(p["theta_x"], 2), "theta_y_deg": round(p["theta_y"], 2),
                "theta_z_deg": round(p["theta_z"], 2)}

    def gelenke(self) -> list[float]:
        return [round(g, 2) for g in self.zustand()["gelenke_deg"]]

    def greifer_weite_mm(self) -> float:
        return self.zustand()["greifer_weite_mm"]

    def haelt_wuerfel(self) -> bool:
        """True, wenn ein mit greifen() gegriffener Würfel im Greifer hängt, auch nach einem Abbruch. Dann vor allem
        anderen ablegen() oder abstellen()."""
        return bool(self.zustand().get("haelt_wuerfel"))

    def fahrer(self) -> str | None:
        """Name des Teams, das gerade das Fahrrecht hat, sonst None."""
        return ((self.zustand().get("netz") or {}).get("fahrer") or {}).get("team")

    def letzter_stopp(self) -> dict | None:
        """Letzter STOPP am Server: {"zeit": "14:27:53", "von": "Betreuer" | "anderes Team" | Teamname} oder None.

        Der feste Wert "Betreuer" heißt: STOPP in der lokalen Oberfläche am Mac Mini gedrückt, egal von wem."""
        return (self.zustand().get("netz") or {}).get("letzter_stopp")

    def bild(self, kamera: str = "farbe") -> Bild:
        """Frisches Bild der Farb- oder Tiefenkamera, aufgenommen im Stillstand."""
        return Bild(self._anfrage("/api/bild", {"strom": kamera}))

    # --- Bewegen ----------------------------------------------------------------------------------------------
    def home(self) -> dict:
        """Gespeicherte Home-Pose anfahren (Gelenkfahrt mit Kollisionsprüfung)."""
        return self._fahren("/api/home/fahren", {})

    def fahre_zu(self, x_mm: float | None = None, y_mm: float | None = None, z_mm: float | None = None,
                 gier_deg: float | None = None, tempo_mm_s: float = 50.0) -> dict:
        """Werkzeugpunkt geradlinig auf (x, y, z) fahren; weggelassene Werte bleiben. gier_deg: Werkzeug senkrecht
        nach unten, Finger schließen entlang dieser Richtung (0° = entlang x). Höchstens 50 mm/s."""
        daten = {"geschw_m_s": tempo_mm_s / 1000.0, "grund": f"{self.team}: fahre_zu"}
        for name, wert in (("x_m", x_mm), ("y_m", y_mm), ("z_m", z_mm)):
            if wert is not None:
                daten[name] = wert / 1000.0
        if gier_deg is not None:
            daten["gier_deg"] = gier_deg
        return self._fahren("/api/fahren", daten)

    def schritt(self, dx_mm: float = 0, dy_mm: float = 0, dz_mm: float = 0, drx_deg: float = 0,
                dry_deg: float = 0, drz_deg: float = 0, bezug: str = "basis") -> dict:
        """Kleiner Schritt (höchstens 50 mm, 15°). bezug "basis" oder "werkzeug" (Achsen des Greifers)."""
        return self._fahren("/api/schritt", {"bezug": bezug, "dx": dx_mm / 1000, "dy": dy_mm / 1000,
                                             "dz": dz_mm / 1000, "drx": drx_deg, "dry": dry_deg, "drz": drz_deg})

    def gelenk(self, nummer: int, um_deg: float) -> dict:
        """Ein Gelenk (1–7) um höchstens 15° drehen."""
        return self._fahren("/api/gelenk", {"gelenk": nummer, "d_deg": um_deg})

    def gelenke_anfahren(self, winkel_deg: list[float]) -> dict:
        """Alle sieben Gelenkwinkel anfahren (vorher wird die Bahn auf Kollisionen geprüft)."""
        return self._fahren("/api/gelenke", {"gelenke_deg": list(winkel_deg), "grund": f"{self.team}: Gelenke"})

    def greifer(self, weite_mm: float, tempo: float = 1.0) -> dict:
        """Greifer auf eine Weite fahren (0 = zu, 85 = ganz offen). tempo 0,05 … 1."""
        return self._fahren("/api/greifer", {"weite_mm": weite_mm, "geschwindigkeit": tempo})

    def oeffnen(self) -> dict:
        return self._fahren("/api/greifer", {"position": 0.0})

    def schliessen(self) -> dict:
        return self._fahren("/api/greifer", {"position": 1.0})

    def stopp(self) -> None:
        """Arm sofort anhalten und laufende Abläufe abbrechen (geht auch ohne Fahrrecht)."""
        self._anfrage("/api/stopp", {}, zeitlimit_s=5.0)

    # --- Erkennen und Greifen ---------------------------------------------------------------------------------
    def tags(self, id: int | None = None) -> list[dict]:
        """AprilTags im Farbbild (im Stillstand) mit Lage in Basiskoordinaten. id: nur diesen Tag."""
        antwort = self._fahren("/api/apriltag/erkennen", {} if id is None else {"id": id})
        ergebnis = []
        for t in antwort.get("tags", []):
            m = t["basis_matrix"]
            if id is not None and t["id"] != id:
                continue
            ergebnis.append({"id": t["id"], "x_mm": _mm(m[0][3]), "y_mm": _mm(m[1][3]), "z_mm": _mm(m[2][3]),
                             "gier_deg": round(math.degrees(math.atan2(m[1][0], m[0][0])), 1),
                             "abstand_mm": _mm(t["abstand_m"]), "ecken_px": t["ecken_px"]})
        return ergebnis

    def wuerfel_vermessen(self, bei_x_mm: float | None = None, bei_y_mm: float | None = None,
                          hinfahren: bool = False) -> dict:
        """Würfel oder Turm im Tiefenbild vermessen und Greifplan für den obersten Würfel berechnen.

        hinfahren=False: Der Arm bewegt sich nicht. Gemessen wird nur, was die Kamera gerade sieht. Mit
        bei_x_mm/bei_y_mm den Würfel, der dort liegt (höchstens 50 mm daneben).
        hinfahren=True: Der Arm fährt die Kamera senkrecht über bei_x_mm/bei_y_mm (Lage auf etwa 5 cm genau), über
        einem Turm entsprechend höher, und misst dort. Danach kann ``greifen()`` direkt folgen.
        Liefert Mitte, Kanten, Höhe, Oberseite (oben_z_mm), Zahl der Würfel übereinander (stapel) und Gier in mm/Grad."""
        daten = {}
        if bei_x_mm is not None and bei_y_mm is not None:
            daten["bei_m"] = [bei_x_mm / 1000.0, bei_y_mm / 1000.0]
        if hinfahren:
            if not daten:
                raise ValueError("hinfahren braucht bei_x_mm und bei_y_mm")
            w = self._fahren("/api/wuerfel/anfahren_vermessen", daten)
        else:
            w = self._fahren("/api/wuerfel/scannen", daten)
        return {"x_mm": _mm(w["mitte_m"][0]), "y_mm": _mm(w["mitte_m"][1]), "z_mm": _mm(w["mitte_m"][2]),
                "kanten_mm": w["kanten_mm"], "hoehe_mm": w["hoehe_mm"], "gier_deg": w["gier_deg"],
                "tisch_z_mm": w["tisch_z_mm"], "oben_z_mm": w.get("oben_z_mm"), "stapel": w.get("stapel", 1),
                "greifen_moeglich": not (w["pruefung_vorgriff"] or w["pruefung_griff"]),
                "pruefung": w["pruefung_vorgriff"] + w["pruefung_griff"]}

    def wuerfel_suchen(self) -> dict:
        """Mit der Tiefenkamera umsehen, über den ersten gefundenen Würfel fahren und ihn vermessen."""
        return self._fahren("/api/wuerfel/suchen", {})

    def greifen(self) -> dict:
        """Zuletzt vermessenen Würfel von oben greifen und 5 cm anheben."""
        return self._fahren("/api/wuerfel/greifen", {})

    def ablegen(self, x_mm: float, y_mm: float, gier_deg: float | None = None,
                unten_z_mm: float | None = None) -> dict:
        """Gehaltenen Würfel mit seiner Mitte bei (x, y) absetzen. Geht auch nach einem Abbruch (STOPP), solange
        haelt_wuerfel() True ist. unten_z_mm: Höhe der Unterseite (Standard: Tisch; zum Stapeln ``oben_z_mm`` aus dem
        Scan des unteren Würfels oder Turms). gier_deg: Drehung des Würfels, z. B. die Gier des unteren Würfels."""
        daten = {"x_m": x_mm / 1000.0, "y_m": y_mm / 1000.0}
        if gier_deg is not None:
            daten["gier_deg"] = gier_deg
        if unten_z_mm is not None:
            daten["unten_z_m"] = unten_z_mm / 1000.0
        return self._fahren("/api/wuerfel/ablegen", daten)

    def stapeln(self) -> dict:
        """Alle Würfel im Arbeitsbereich zu Türmen aus höchstens drei Würfeln stapeln (dauert einige Minuten).
        Liefert gefundene und gestapelte Würfel, die Türme und übersprungene Würfel mit Grund."""
        return self._fahren("/api/wuerfel/stapeln", {}, zeitlimit_s=1200.0)

    def abstellen(self) -> dict:
        """Gehaltenen Würfel dort abstellen, wo er gegriffen wurde, dann Home."""
        return self._fahren("/api/wuerfel/abstellen", {})

    # --- Nur Simulation ---------------------------------------------------------------------------------------
    def sim_szene(self, zufall: int | None = None, wuerfel: list[dict] | None = None, anzahl: int = 6) -> dict:
        """Würfel neu hinlegen (nur Simulation). zufall: Startwert für zufällige Lagen. wuerfel: Liste mit
        {"id", "x_mm", "y_mm", "gier_deg"}. Ohne Angabe: Startszene."""
        daten: dict = {}
        if zufall is not None:
            daten = {"zufall": zufall, "anzahl": anzahl}
        elif wuerfel is not None:
            daten = {"wuerfel": [{"id": w["id"], "x_m": w["x_mm"] / 1000.0, "y_m": w["y_mm"] / 1000.0,
                                  "gier_deg": w.get("gier_deg", 0.0)} for w in wuerfel]}
        return self._anfrage("/api/sim/szene", daten)
