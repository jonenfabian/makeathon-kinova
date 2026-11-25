# Makeathon Kinova: Roboterarm

Den Kinova-Gen3-Arm mit Greifer und Kamera auf dem eigenen Rechner in einer 3D-Simulation steuern,
Python-Code ändern und danach denselben Code am echten Arm ausführen. Ihr braucht **Git, Python ab 3.10
und einen Browser**. Wir empfehlen Python 3.11. Die Bedienoberfläche läuft lokal.

## Loslegen: macOS und Linux

Prüft zuerst die Python-Version. Am Mac ist `python3` ohne weitere Installation oft Apples Python 3.9,
das ist zu alt. Dann Python 3.11 von python.org oder über Homebrew installieren.

```sh
python3 --version
git clone https://github.com/jonenfabian/makeathon-kinova.git
cd makeathon-kinova
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python start.py
```

## Loslegen: Windows

PowerShell oder Eingabeaufforderung:

```text
py -3 --version
git clone https://github.com/jonenfabian/makeathon-kinova.git
cd makeathon-kinova
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe start.py
```

Wartet auf die Zeile „Dashboard bereit“ und öffnet **http://127.0.0.1:8080**. Ihr seht Arm, Greifer,
Würfel und Kamerabilder. Später genügt der letzte Befehl. Die virtuelle Python-Umgebung muss nicht extra
aktiviert werden. Strg-C beendet den Server. `start.py` startet immer die Simulation.

## Eigene Programme

Im zweiten Terminal, ebenfalls im Projektordner (Windows: Python-Pfad wie oben ersetzen):

```sh
.venv/bin/python kinova/beispiele/01_verbinden.py
.venv/bin/python kinova/beispiele/04_wuerfel_umsetzen.py
```

Die [Kinova-Anleitung](kinova/README.md) erklärt Bibliothek, Befehle, Koordinaten und die Verbindung
zum echten Arm. Ohne `verbindung.json` sprechen eure Programme mit der Simulation.

## Vor Ort und eigene Ideen

- [Leitseite vor Ort](vor-ort/README.md) mit Startkarte und Aufgabenblatt. Die Startkarte erklärt den
  Übergang von der Simulation zum echten Arm.
- [Ideen für eigene Erweiterungen](vor-ort/ideen.md), auch zum Umsetzen mit einem Coding Agent.
- [`AGENTS.md`](AGENTS.md): Kurzbeschreibung des Repositorys und der Schnittstellen für Coding Agents.

Ihr habt vollen Zugang zum Arm-Arbeitsplatz und testet selbstständig. Der Not-Halt am Arm und die
Prüfungen im Server bleiben immer aktiv. Es fährt immer nur ein Team.

| Einstieg | Nächste eigene Aufgabe |
|---|---|
| Einen Würfel umsetzen | Zwei Würfel auf verschiedene Plätze legen |
| Tags finden | Würfel nach ID sortiert ablegen |
| Eigener Turm | Turm aus drei Würfeln bauen und wieder abbauen |

## Plattformen

Der automatische Lauf in GitHub Actions installiert `requirements.txt` und prüft die Simulation (Tags,
Greifen, Ablegen) unter Windows x64, macOS Intel/Apple Silicon sowie Linux x64/ARM64. Die Simulation
vereinfacht Kamera, Greifer und Kontakte. Reale Abnahmen finden am Arm statt. Das Repository enthält
keine Zugangsdaten zu echten Geräten.

## Lizenzen

Der Kinova-Teamcode steht unter der MIT-Lizenz in `kinova/LICENSE`. Die Lizenztexte der Arm- und
Greifermodelle (BSD-3-Clause) und von three.js (MIT) liegen direkt neben diesen Dateien.
