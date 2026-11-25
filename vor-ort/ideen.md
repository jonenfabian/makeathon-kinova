<!-- Wird als vor-ort/ideen.md in das Team-Repository makeathon-kinova exportiert (tools/team_repository.py). Relative Links gelten dort. -->
# Ideen für eigene Erweiterungen: Kinova

Stand: 24. November 2026. Diese Liste ist eine Sammlung von Anregungen, keine Pflicht. Wählt nach
dem [Aufgabenblatt](aufgabenblatt-kinova.md) eine Idee, legt vor dem Programmieren ein Erfolgskriterium
fest und testet jede Änderung zuerst in der Simulation.

Arbeitsplatz: Kinova Gen3 mit sieben Gelenken, Robotiq-2F-85-Greifer und Farb-/Tiefenkamera am Arm,
Holzwürfel mit AprilTags (IDs 200 bis 205). Greifen, Ablegen, Vermessen und Tag-Erkennung sind fertige
Befehle der Bibliothek `kinova_team`, siehe [Kinova-Anleitung](../kinova/README.md). Der Server prüft
jede Bewegung (Arbeitsraum, Tischabstand, höchstens 50 mm/s, Gelenkanschläge).

Die Reihenfolge ist eine Einschätzung des Aufwands, keine gemessene Zeit.

| Stufe | Idee | Ergebnis | Worauf achten |
| --- | --- | --- | --- |
| 1 | Einen Würfel umsetzen | Würfel über seinen Tag finden, greifen, an festem Platz ablegen | Ausgangspunkt ist `04_wuerfel_umsetzen.py` |
| 2 | Auftrag auswählen | Würfel-ID und Zielplatz über eine Eingabe oder eigene Bedienseite wählen | `06_eigenes_dashboard.html` als Vorlage |
| 3 | Sortieren | Würfel nach ID auf bekannte Ablagefelder verteilen | Felder vorher festlegen und freihalten |
| 4 | Muster legen | Eine vorgegebene Anordnung auf einem Raster aufbauen | Raster in Basiskoordinaten beschreiben |
| 5 | Turm auf- und abbauen | Turm aus bis zu drei Würfeln bauen, von oben wieder abbauen | Obere Tags verdecken untere. Vor dem Greifen neu vermessen |
| 6 | Nach menschlichem Umstellen weitermachen | Pausieren, nach „Weiter“ neu aufnehmen, alte Positionen verwerfen | Hände nur im Stillstand in den Arbeitsbereich |
| 7 | Pyramide oder mehrlagiger Aufbau | Kleinen stabilen Aufbau planen | Tragende Flächen und Freiraum für den Greifer |
| 8 | Foto als Vorlage | Draufsicht eines bekannten Rasters auswerten und nachlegen | Nur kontrollierte Fotos, keine beliebigen Bilder |
| 9 | Fehler erkennen | Prüfen, ob Greifen und Ablegen geklappt haben, begrenzt wiederholen | `r.haelt_wuerfel()` und neues Vermessen nutzen |
| 10 | Spiel gegen einen Menschen | 3×3-Raster, drei in einer Reihe, Zug und Gegenzug | Belegung nach jedem Zug neu erfassen |
| 11 | Mehrere Ansichten zusammenführen | Tiefendaten aus mehreren Armstellungen in ein Koordinatensystem bringen | Szene darf sich zwischen den Aufnahmen nicht bewegen |

Für den Einstieg eignen sich die Stufen 1 bis 4, ein kleiner Turm als Erweiterung.

## Am echten Arm

- Es fährt immer nur ein Team. Vor jedem Lauf steht eine Person eures Teams am Not-Halt.
- Die Simulation vereinfacht Kamera, Greifer und Kontakte. Dokumentiert Ergebnisse getrennt nach
  Simulation und echtem Arm.

## Mit einem Coding Agent arbeiten

- Öffnet den ganzen Klon im Agenten. [`AGENTS.md`](../AGENTS.md) beschreibt Ordner, Bibliothek und Grenzen.
- Beschreibt das Ziel mit einem prüfbaren Erfolgskriterium, etwa: „Die Würfel 200 bis 202 liegen danach
  in aufsteigender Reihenfolge auf den Feldern A, B und C.“
- Lasst den Agenten eigene Programme schreiben, nicht `kinova/simulation/` und nicht die Bibliothek ändern.
- Prüft jede Version in der Simulation, bevor sie am Arm läuft. Lest den Code, bevor ihr ihn am Arm startet.
