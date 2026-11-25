# Hinweise für Coding Agents: Kinova-Arbeitsplatz

Dieses Repository ist der Team-Einstieg für den Kinova-Gen3-Arm beim Makeathon. Die Teams schreiben
eigene Python-Programme mit der Bibliothek `kinova_team`. Dieselben Programme sprechen mit der lokalen
Simulation oder, mit `verbindung.json`, mit dem Server am echten Arm.

## Ordner

| Ort | Inhalt | Ändern? |
| --- | --- | --- |
| `kinova/beispiele/` | Beispielprogramme 01 bis 08 | Kopieren und anpassen |
| `kinova/kinova_team/` | Bibliothek mit der Klasse `Roboter` | Nein, nur lesen |
| `kinova/simulation/` | Simulationsserver, Bedienseite, Modelle | Nein |
| `kinova/README.md` | Befehle, Koordinaten, Regeln am echten Arm | Lesen |
| `vor-ort/` | Startkarte, Aufgabenblatt, Ideen | Nein |
| `start.py` | Startet die Simulation unter http://127.0.0.1:8080 | Nein |

## Bibliothek

```python
from kinova_team import Roboter

with Roboter() as r:          # holt das Fahrrecht und gibt es am Ende ab
    r.home()
    tags = r.tags()           # AprilTags mit Lage in Basiskoordinaten
```

Eigene Programme in `kinova/beispiele/` ablegen oder die `sys.path`-Zeile aus den Beispielen übernehmen,
sonst findet Python die Bibliothek nicht. Die vollständige Befehlsliste steht in `kinova/README.md` unter „Befehle“. Längen in Millimetern,
Winkel in Grad, Ursprung an der Roboterbasis (x nach vorn, y nach links, z nach oben). Würfel tragen
AprilTags 36h11 mit den IDs 200 bis 205.

Ohne `verbindung.json` im Projektordner verbinden sich Programme mit der Simulation auf `127.0.0.1:8080`.
Mit der Datei sprechen sie mit dem echten Arm.

## Grenzen

- Der Server prüft jede Bewegung (Arbeitsraum, Tischabstand, höchstens 50 mm/s, Gelenkanschläge) und
  lehnt unzulässige Befehle mit `Abgelehnt` ab. Programme reagieren darauf, statt es zu umgehen.
- Es fährt immer nur ein Team. Verlorenes Fahrrecht meldet `KeinZugang`, danach nicht blind weiterfahren.
- Nach einer abgebrochenen Fahrt zuerst die Pose lesen. Positionen aus alten Bildern nicht wiederverwenden.
- `verbindung.json` enthält einen Team-Schlüssel und kommt nie ins Git.

## Prüfen

```sh
.venv/bin/python start.py
.venv/bin/python kinova/beispiele/EUER_PROGRAMM.py
```

Erst wenn das Programm in der Simulation wie erwartet läuft, am echten Arm starten. Der Ablauf steht in
`vor-ort/startkarte-kinova.md`.
