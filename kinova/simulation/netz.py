"""Zugang für Team-Laptops über das Netz: Schlüssel, Fahrrecht, Freigabe durch den Betreuer (Fabian, 10.11.2026).

Regeln:
- Seit 19.11.2026 arbeiten die Teams selbstständig. „Betreuer“ ist hier nur noch der Name der Rolle
  „lokale Oberfläche am Mac Mini“, die auch Teams bedienen. Die Feldwerte `rolle: "betreuer"` und
  `von: "Betreuer"` bleiben aus Gründen der Verträglichkeit unverändert.
- **Betreuer** ist, wer am Mac Mini selbst arbeitet (Verbindung von 127.0.0.1/::1, Host localhost, Kopfzeile
  ``X-Kinova: 1``, kein Team-Schlüssel). Er darf alles, auch Home speichern, kalibrieren und Fehler quittieren.
- **Teams** schicken den Team-Schlüssel in ``X-Kinova-Schluessel``. Ohne gültigen Schlüssel gibt es nur 401.
- Bewegen darf immer nur **ein** Team: das mit dem Fahrrecht (``X-Kinova-Fahrrecht``, eine zufällige Marke). Das
  Fahrrecht verfällt nach ``FAHRRECHT_S`` ohne Befehl dieses Teams (Zustandsabfragen zählen nicht), aber nie, solange
  einer seiner Befehle noch läuft. Verlängern kann es nur, wer die Marke hat; derselbe Teamname genügt nicht.
- Zusätzlich muss der Betreuer das Fahren übers Netz **freigegeben** haben. Nimmt er die Freigabe zurück oder entzieht
  er das Fahrrecht, hält der Arm sofort an.
- STOPP darf jedes Team mit gültigem Schlüssel, auch ohne Fahrrecht. Kommt der STOPP vom Betreuer oder von einem
  anderen Team, verliert das fahrende Team sein Fahrrecht: Sein Skript bricht beim nächsten Befehl ab, statt
  weiterzufahren. Entzogene Marken gelten nie wieder; das Team meldet sich bewusst neu an.

Der Schlüssel schützt nicht gegen Mitlesen im WLAN (HTTP ohne Verschlüsselung). Er verhindert, dass zufällig jemand im
selben Netz fährt; die Sicherheit am Arm bleiben Not-Halt, Firmwaregrenzen und die Prüfungen in ``arm.py``.
"""

from __future__ import annotations

import hmac
import json
import secrets
import threading
import time
from pathlib import Path

FAHRRECHT_S = 60.0
ZEICHEN = "abcdefghjkmnpqrstuvwxyz23456789"  # ohne 0/o, 1/l/i: wird abgetippt
TEAMNAME_MAX = 40


class KeinZugang(Exception):
    """Schlüssel fehlt oder ist falsch (HTTP 401)."""


class NichtErlaubt(Exception):
    """Aktion ist Betreuern vorbehalten oder das Fahrrecht fehlt (HTTP 403)."""


def neuer_schluessel() -> str:
    roh = "".join(secrets.choice(ZEICHEN) for _ in range(8))
    return f"{roh[:4]}-{roh[4:]}"


def schluessel_laden(pfad: Path) -> str:
    """Team-Schlüssel aus ``pfad`` (bleibt über Neustarts gleich); fehlt die Datei, wird einer erzeugt."""
    if pfad.exists():
        daten = json.loads(pfad.read_text(encoding="utf-8"))
        if daten.get("schluessel"):
            return str(daten["schluessel"])
    schluessel = neuer_schluessel()
    pfad.parent.mkdir(parents=True, exist_ok=True)
    pfad.write_text(json.dumps({"schluessel": schluessel, "erzeugt": time.strftime("%Y-%m-%dT%H:%M:%S")}) + "\n",
                    encoding="utf-8")
    pfad.chmod(0o600)
    return schluessel


class Netzzugang:
    def __init__(self, schluessel: str, anhalten=None, freigabe: bool = False, fahrrecht_s: float = FAHRRECHT_S,
                 wechsel=None):
        if not schluessel or not schluessel.strip():
            raise ValueError("Team-Schlüssel fehlt")
        self.schluessel = schluessel
        self.freigabe = freigabe
        self.fahrrecht_s = fahrrecht_s
        self._anhalten = anhalten or (lambda: None)
        self._wechsel = wechsel or (lambda: None)  # neues Team fährt: gemeinsame Erkennungsergebnisse verwerfen
        self._sperre = threading.Lock()
        self._fahrer: dict | None = None  # {"team", "marke", "seit", "zuletzt", "laufend"}
        self._entzogen: dict[str, str] = {}  # Marke → Grund und Uhrzeit, für die Fehlermeldung an das Team
        self.ereignisse: list[dict] = []
        # letzter STOPP, für alle Teams sichtbar: im Probelauf (11.11.2026) sagte „CONTROL_MANUAL_STOP“ dem Team nicht,
        # wer gestoppt hatte und ob es etwas tun musste
        self.letzter_stopp: dict | None = None  # {"t", "zeit", "von"}: von = "Betreuer", "anderes Team" oder Teamname
        self.adressen: list[str] = []  # nur Anzeige: unter diesen Adressen erreichen die Teams den Server

    # --- Hilfen -----------------------------------------------------------------------------------------------
    def _merken(self, text: str) -> None:
        self.ereignisse = [*self.ereignisse[-29:], {"zeit": time.strftime("%H:%M:%S"), "text": text}]

    def _aktiver_fahrer(self) -> dict | None:
        """Fahrrecht, falls nicht verfallen (Aufruf mit gehaltener Sperre)."""
        f = self._fahrer
        if f is not None and f["laufend"] == 0 and time.time() - f["zuletzt"] > self.fahrrecht_s:
            self._merken(f"Fahrrecht von {f['team']} verfallen")
            self._fahrer = None
        return self._fahrer

    def schluessel_pruefen(self, schluessel: str | None) -> None:
        if not schluessel or not hmac.compare_digest(schluessel.strip().encode(), self.schluessel.encode()):
            raise KeinZugang("Team-Schlüssel fehlt oder ist falsch")

    # --- Fahrrecht --------------------------------------------------------------------------------------------
    @staticmethod
    def _gleich(marke: str | None, f: dict | None) -> bool:
        return bool(f is not None and marke and hmac.compare_digest(marke.encode(), f["marke"].encode()))

    def _entziehen(self, grund: str) -> dict | None:
        """Fahrrecht wegnehmen (Aufruf mit gehaltener Sperre); liefert den bisherigen Fahrer."""
        f = self._fahrer
        self._fahrer = None
        if f is not None:
            self._entzogen[f["marke"]] = f"{grund} um {time.strftime('%H:%M:%S')}"
            while len(self._entzogen) > 50:
                self._entzogen.pop(next(iter(self._entzogen)))
            self._merken(f"{grund}: {f['team']} verliert das Fahrrecht")
        return f

    def anmelden(self, team: str, marke: str | None = None) -> dict:
        """Fahrrecht holen oder mit der eigenen Marke verlängern."""
        team = " ".join(str(team or "").split())[:TEAMNAME_MAX]
        if not team:
            raise NichtErlaubt("Teamname fehlt")
        neu = False
        with self._sperre:
            f = self._aktiver_fahrer()
            if f is not None and not self._gleich(marke, f):
                rest = max(0.0, self.fahrrecht_s - (time.time() - f["zuletzt"]))
                laeuft = ", ein Befehl läuft noch" if f["laufend"] else ""
                raise NichtErlaubt(f"Fahrrecht hat gerade {f['team']} (verfällt frühestens in {rest:.0f} s ohne "
                                   f"Befehl dieses Teams{laeuft})")
            if f is None:
                f = {"team": team, "marke": secrets.token_urlsafe(12), "seit": time.time(), "laufend": 0}
                self._fahrer = f
                self._merken(f"Fahrrecht an {team}")
                neu = True
            f["zuletzt"] = time.time()
            antwort = {"team": f["team"], "fahrrecht": f["marke"], "verfaellt_nach_s": self.fahrrecht_s,
                       "freigabe": self.freigabe}
        if neu:
            self._wechsel()
        return antwort

    def abmelden(self, marke: str | None) -> dict:
        """Fahrrecht abgeben. Läuft noch ein Befehl dieses Teams, hält der Arm an (das Team hat aufgegeben)."""
        with self._sperre:
            f = self._aktiver_fahrer()
            if not self._gleich(marke, f):
                return {"abgegeben": False}
            self._fahrer = None
            lief = f["laufend"] > 0
            self._merken(f"{f['team']} hat das Fahrrecht abgegeben" + (" (Befehl lief noch: STOPP)" if lief else ""))
        if lief:
            self._anhalten()
        return {"abgegeben": True, "angehalten": lief}

    def _pruefen(self, marke: str | None) -> dict:
        """Fahrer zur Marke oder NichtErlaubt mit Grund (Aufruf mit gehaltener Sperre)."""
        if marke and marke in self._entzogen:
            raise NichtErlaubt(f"Fahrrecht entzogen ({self._entzogen[marke]}). Am Arm ist kein Fehler. Programm "
                               "prüfen, dann neu anmelden (r.anmelden() oder neues with Roboter())")
        f = self._aktiver_fahrer()
        if not self._gleich(marke, f):
            wer = f"; gerade fährt {f['team']}" if f is not None else ""
            raise NichtErlaubt(f"kein Fahrrecht (erst anmelden{wer})")
        return f

    def fahrrecht_pruefen(self, marke: str | None) -> dict:
        """Für Befehle ohne Bewegung, die gemeinsame Ergebnisse ändern; verlängert das Fahrrecht."""
        with self._sperre:
            f = self._pruefen(marke)
            f["zuletzt"] = time.time()
            return f

    def fahren_beginnen(self, marke: str | None) -> dict:
        """Prüft Freigabe und Fahrrecht vor einer Bewegung; danach ``fahren_beenden`` aufrufen."""
        with self._sperre:
            if not self.freigabe:
                raise NichtErlaubt("Fahren übers Netz ist nicht freigegeben (am Mac Mini „Fahren für Teams "
                                   "freigeben“ einschalten)")
            f = self._pruefen(marke)
            f["laufend"] += 1
            f["zuletzt"] = time.time()
            return f

    def fahren_beenden(self, f: dict) -> None:
        with self._sperre:
            f["laufend"] = max(0, f["laufend"] - 1)
            f["zuletzt"] = time.time()

    def stopp_gedrueckt(self, betreuer: bool, marke: str | None) -> None:
        """STOPP vom Betreuer oder von einem anderen Team: Fahrrecht entziehen, damit das Skript nicht weiterfährt.
        Der eigene STOPP (Strg-C) lässt das Fahrrecht bestehen. Anhalten muss der Aufrufer."""
        with self._sperre:
            f = self._aktiver_fahrer()
            eigen = not betreuer and self._gleich(marke, f)
            von = "Betreuer" if betreuer else (f["team"] if eigen else "anderes Team")
            self.letzter_stopp = {"t": time.time(), "zeit": time.strftime("%H:%M:%S"), "von": von}
            if f is not None and not eigen:
                self._entziehen("STOPP am Mac Mini" if betreuer else "STOPP von einem anderen Team")

    def stopp_hinweis(self, seit: float) -> str | None:
        """Klartext für eine Fahrt, die ein STOPP seit ``seit`` (time.time()) abgebrochen hat, sonst None."""
        s = self.letzter_stopp
        if s is None or s["t"] < seit:
            return None
        if s["von"] in ("Betreuer", "anderes Team"):
            wer = "am Mac Mini" if s["von"] == "Betreuer" else "von einem anderen Team"
            return (f"STOPP {wer} um {s['zeit']}. Am Arm ist kein Fehler. Euer Fahrrecht ist entzogen, für die "
                    "nächste Fahrt neu anmelden. Hängt ein Würfel im Greifer, mit ablegen() oder abstellen() wegbringen")
        return f"eigener STOPP um {s['zeit']}"

    # --- Betreuer ---------------------------------------------------------------------------------------------
    def freigabe_setzen(self, an: bool) -> dict:
        with self._sperre:
            self.freigabe = bool(an)
            self._merken("Fahren übers Netz freigegeben" if an else "Freigabe zurückgenommen")
        if not an:
            self._anhalten()
        return self.status(betreuer=True)

    def entziehen(self) -> dict:
        with self._sperre:
            f = self._entziehen("am Mac Mini im Dashboard")
        if f is not None:
            self._anhalten()
        return self.status(betreuer=True)

    def status(self, betreuer: bool = False, marke: str | None = None) -> dict:
        with self._sperre:
            f = self._aktiver_fahrer()
            d = {"freigabe": self.freigabe, "fahrrecht_s": self.fahrrecht_s,
                 "fahrer": None if f is None else {
                     "team": f["team"], "seit_s": round(time.time() - f["seit"]),
                     "ruhig_s": round(time.time() - f["zuletzt"]), "laeuft": f["laufend"] > 0},
                 "eigenes_fahrrecht": self._gleich(marke, f),
                 "letzter_stopp": None if self.letzter_stopp is None else {
                     "zeit": self.letzter_stopp["zeit"], "von": self.letzter_stopp["von"]}}
        if betreuer:
            d.update({"schluessel": self.schluessel, "adressen": self.adressen, "ereignisse": self.ereignisse[-10:]})
        return d
