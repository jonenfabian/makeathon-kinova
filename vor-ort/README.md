<!-- Wird als vor-ort/README.md in das Team-Repository makeathon-kinova exportiert (tools/team_repository.py). Relative Links gelten dort. -->
# Start und Aufgaben vor Ort: Kinova

Stand: 24. November 2026 · Makeathon Bremerhaven, 3.–5. Dezember 2026

Ihr habt die Beispiele auf eurem Laptop in der Simulation ausprobiert. Vor Ort übertragt ihr einen
bekannten Ablauf auf den echten Arm und entwickelt danach eine eigene Aufgabe.

| Schritt | Unterlage |
| --- | --- |
| 1 Von der Simulation zum echten Arm | [Startkarte Kinova](startkarte-kinova.md) |
| 2 Grundaufgaben und eine Erweiterung | [Aufgabenblatt Kinova](aufgabenblatt-kinova.md) |
| 3 Eigene Ideen | [Ideen für Erweiterungen](ideen.md) |

## So arbeitet ihr

1. Übernehmt die vollständigen Geräteangaben. Wechselt im Team zwischen Programmieren,
   Versuchsbedienung und Protokollieren.
2. Führt die Startkarte selbstständig durch und tragt das Ergebnis eurer ersten Hardwareprobe ein.
3. Bearbeitet die Grundaufgabe und wählt anschließend eine Erweiterung. Testet jede Änderung
   zunächst in der Simulation und danach am Arm.
4. Speichert euren Python-Code und einen kurzen Bericht. Das Aufgabenblatt enthält
   Prüffälle und eine Vorlage für eure Ergebnisse.

Die Teams am Kinova teilen sich einen Arm. Während ein Team am Gerät testet, arbeiten die
anderen in ihren lokalen Simulationen weiter. Geänderte Ausgangslagen werden jeweils neu
aufgenommen. Eine erfolgreiche Simulation ist ein eigener Nachweis. Die Hardwareprobe
bekommt einen getrennten Eintrag.

Ihr habt vollen Zugriff auf den Arm-Arbeitsplatz. Für Hardwaretests ist keine Anwesenheit
oder einzelne Freigabe eines Betreuers erforderlich. Stimmt die Nutzung des Arms
untereinander ab. Der Not-Halt und die Koordination eines einzigen Fahrers bleiben bestehen.

## Regeln am Arm

Vor jedem Lauf am echten Arm steht eine Person eures Teams am Not-Halt. Die drei Regeln
am Arm stehen in der [Startkarte Kinova](startkarte-kinova.md).

## Arbeitsordner und Befehle

Die Laptop-Befehle beziehen sich auf euren Klon
[`makeathon-kinova`](https://github.com/jonenfabian/makeathon-kinova) aus der Vorab-Übung.
Öffnet das Terminal in diesem Projektordner. Dort liegen `start.py`, `.venv` und `kinova/`.
Neu klonen müsst ihr nicht. Holt zu Beginn den aktuellen Stand mit Schritt 0 der Startkarte.
Die [README](../README.md) enthält die Installation für Windows, macOS und Linux.

Die Startkarte zeigt Python-Befehle mit `.venv/bin/python` für macOS/Linux.
Unter Windows ersetzt ihr diesen Teil durch `.\.venv\Scripts\python.exe`.

Großgeschriebene Angaben wie `SERVERADRESSE` sind Platzhalter. Ersetzt sie durch eure
Geräteangaben. Team-Schlüssel gehören weder in euren Bericht noch ins Git.

## Was ihr zum Schluss zeigt

Zeigt einen vollständigen Ablauf, wiederholt ihn mit veränderter Ausgangslage und
erklärt einen behandelten Fehlerfall. Gebt euren Code und `bericht.md` ab. Nennt darin
die getestete Dateiversion, Gerät, Bedingungen, Ergebnis und eine verbleibende Grenze.
Legt vor der Abschlussvorführung gemeinsam fest, wer den Versuch startet und stoppt.
