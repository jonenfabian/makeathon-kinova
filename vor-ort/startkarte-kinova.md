# Startkarte Kinova

Stand: 24. November 2026 · Ziel: denselben Python-Ablauf zuerst in der Simulation und
danach selbstständig am echten Arm ausführen. Ihr habt vollen Gerätezugang.

**Team:** ____________________ **Datum:** ____________________ **Gerät:** ____________________

**Serveradresse vom Mac Mini (ohne Port):** ____________________ **Freie Zielfelder:** ____________________

Team-Schlüssel und Zugang zum Mac Mini erhaltet ihr mit den Geräteangaben. Jeder Teamname muss eindeutig sein.

## 0 Repository aktualisieren

Öffnet ein Terminal im Projektordner `makeathon-kinova`, also im Ordner mit `start.py`.
Holt den aktuellen Stand und installiert die Abhängigkeiten neu.

macOS und Linux:

```sh
git pull
.venv/bin/python -m pip install -r requirements.txt
```

Windows (PowerShell oder Eingabeaufforderung):

```text
git pull
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Bricht `git pull` wegen geänderter Beispieldateien ab, sichert eure Datei unter einem neuen
Namen. Setzt das Original mit `git restore DATEI` zurück und wiederholt `git pull`.
Fehlt der Ordner `.venv`, installiert ihr nach der README des Team-Repositorys.

## 1 Die Verbindung zur Simulation ausdrücklich einstellen

Alle Laptop-Befehle laufen im Projektordner. Unter Windows ersetzt ihr `.venv/bin/python`
durch `.\.venv\Scripts\python.exe`. Legt im Projektordner neben `start.py` die Datei
`verbindung.json` an:

```json
{
  "adresse": "127.0.0.1:8080",
  "schluessel": "sim",
  "team": "Team Blau"
}
```

Ersetzt `Team Blau` durch euren Teamnamen. `verbindung.json` legt fest, mit welchem Server
eure Programme sprechen. Startet sie deshalb immer aus dem Projektordner. Bereits gesetzte
Umgebungsvariablen `KINOVA_ADRESSE`, `KINOVA_SCHLUESSEL` und `KINOVA_TEAM` haben Vorrang.
Entfernt solche alten Einstellungen, falls die folgende Verbindungsprüfung einen anderen Server zeigt.

**Terminal 1:** Simulation starten und geöffnet lassen.

```sh
.venv/bin/python start.py
```

Öffnet `http://127.0.0.1:8080`. **Terminal 2 im selben Projektordner:**

```sh
.venv/bin/python kinova/beispiele/01_verbinden.py
.venv/bin/python kinova/beispiele/04_wuerfel_umsetzen.py
```

Führt Beispiel 04 erst aus, wenn Beispiel 01 **Simulation** und den lokalen Server meldet.
Beispiel 04 kann sofort den Arm bewegen. Beobachtet Würfel 201, Greifen, Ablegen und Home.

- [ ] Bekannter Ablauf in der Simulation gelungen.

## 2 Mit dem echten Arm verbinden

Beendet den lokalen Simulationsserver in Terminal 1 mit Strg+C. Verbindet euren Laptop
mit dem vereinbarten WLAN. Der echte Arm wird vom zentralen Server am Mac Mini bedient.
Auf eurem Laptop startet ihr dafür keinen zweiten Armserver.

Tragt in derselben `verbindung.json` die **Serveradresse des Mac Mini** vor `:8080`, den
erhaltenen Team-Schlüssel und euren Teamnamen ein. Ersetzt die Platzhalter, bevor ihr die
Datei speichert:

```json
{
  "adresse": "SERVERADRESSE:8080",
  "schluessel": "TEAM-SCHLUESSEL",
  "team": "Team Blau"
}
```

```sh
.venv/bin/python kinova/beispiele/01_verbinden.py
```

- [ ] Richtiger Server und **echter Arm** werden angezeigt. Diese Prüfung bewegt nichts.
- [ ] Beispiel 01 zeigt `Fahren freigegeben: True`. Steht dort `False`, schaltet ihr am Mac Mini
  in der lokalen Oberfläche (dort `http://127.0.0.1:8080`) „Fahren für Teams freigeben“ ein.
  Fehlt euch dafür der Zugang, sprecht die Orga an.
- [ ] Arbeitsbereich frei und freie Zielfelder geprüft.

**Regeln am echten Arm**

1. Vor jedem Lauf steht eine Person eures Teams am Not-Halt.
   **Not-Halt befindet sich:** ____________________
2. Niemand greift in den Arbeitsbereich, solange der Arm fahren kann.
3. Würfel stellt ihr nur im Stillstand um.

Bewegt sich der Arm unerwartet, drückt diese Person den Not-Halt. Klärt die Ursache.
Erst danach entriegelt ihr den Not-Halt und quittiert am Mac Mini mit „Fehler quittieren“.

Jetzt führt ihr Beispiel 04 am Gerät aus. Stimmt euch mit anderen Teams ab und prüft vorher
die tatsächliche Würfel-ID und `ZIEL_MM` in der Datei. `(280.0, 70.0)` ist der Beispielwert
der Simulation und muss zum freien, geprüften Ablageplatz passen. Fehlt eine reale
AprilTag-Erkennung, bearbeitet zunächst die Bild-/Statusaufgabe K1 und untersucht die Erkennung.

Es fährt immer nur ein Team. `with Roboter() as r:` holt das Fahrrecht und gibt es am Ende ab.
Für eine Bedienseite könnt ihr `kinova/beispiele/06_eigenes_dashboard.html` im Browser
öffnen. Tragt Adresse, Schlüssel und Team dort separat ein. Die Seite liest `verbindung.json`
nicht. Holt das Fahrrecht nur in eurem abgestimmten Zeitraum und beendet parallele Bedienprogramme.

## 3 Stoppen und weiterarbeiten

Strg+C im Python-Programm oder STOPP im Team-Dashboard senden einen Stoppbefehl.
Bei Gefahr drückt eure Person am Not-Halt den Not-Halt. Ein Software-Stopp erreicht den Arm
bei unterbrochener Verbindung möglicherweise nicht. Nach einem fremden STOPP ist euer
Fahrrecht entzogen. Prüft dann die Ursache, die tatsächliche Pose und ob der Greifer noch
einen Würfel hält. Plant den Ablauf danach bewusst neu.

**Zurück zur Simulation:** Programme und Bedienseiten für den echten Arm schließen. In
`verbindung.json` wieder `127.0.0.1:8080` und `sim` eintragen oder die Datei löschen. Ohne
Datei verbinden sich die Programme ebenfalls mit der Simulation. Simulation starten und
Beispiel 01 erneut prüfen. `start.py` startet immer eine Simulation. Ein bereits
laufendes Python-Programm behält sein Verbindungsziel, bis ihr es neu startet.
`verbindung.json` mit echten Zugangsdaten bleibt lokal und kommt nicht ins Git.

**Ergebnis / Datum:** ______________________________________________________________

Weiter mit dem [Aufgabenblatt Kinova](aufgabenblatt-kinova.md).
