# Aufgabenblatt Kinova

Stand: 24. November 2026 · **Team:** ____________________ **Datum:** ____________________

Euer Ziel ist ein eigener Ablauf aus Erkennen, Greifen und Ablegen. Beginnt mit der
[Startkarte Kinova](startkarte-kinova.md). Testet neue Versionen zuerst in der Simulation.
Am echten Arm prüft ihr vor jedem Lauf mit Beispiel 01 das Verbindungsziel und arbeitet
selbstständig mit eurem vollständigen Gerätezugang. Während andere Teams fahren, entwickelt
ihr lokal weiter. Ein geteilter Teamname darf nicht zur gemeinsamen gleichzeitigen Bedienung führen.

## K1 Zustand und Bilder verstehen

Startet `kinova/beispiele/01_verbinden.py`. Lest Server, Simulation/echter Arm, Pose,
Gelenkwinkel, Greiferweite und Fahrrecht. Speichert anschließend mit
`kinova/beispiele/07_bilder_speichern.py` ein Farb- und ein Tiefenbild der Würfelszene.
Schaltet die benötigten Kamerastreams am Arm-Arbeitsplatz ein.

**Geschafft, wenn:** Ihr zeigt die beiden Bilder und erklärt einen Würfel sowie einen
zugehörigen Tiefenbereich. Nennt die Einheiten: Werkzeugposition in **mm**, Winkel in
**Grad**, Bezug zur Roboterbasis. Beschreibt, warum eine geänderte Armstellung auch das
Kamerabild verändert. Diese Leseaufgaben bewegen den Arm nicht. Sie funktionieren auch,
solange Beispiel 01 noch `Fahren freigegeben: False` zeigt.

## K2 Einen Würfel an verschiedene Plätze setzen

Startet mit `kinova/beispiele/04_wuerfel_umsetzen.py`. Speichert eure Arbeitskopie im
gleichen Beispiele-Ordner, damit die vorhandene Einbindung von `kinova_team` weiter funktioniert.
`TAG` bestimmt die gesuchte Würfel-ID, `ZIEL_MM` den Ablageplatz in Basiskoordinaten.
Prüft am echten Tisch freie Felder und passende Koordinaten.

| Prüffall | Erwartetes Ergebnis |
| --- | --- |
| Würfel 201 sichtbar und Zielplatz frei | Richtigen Würfel erkennen, vermessen, greifen, am Ziel ablegen und nach Home fahren |
| Derselbe Würfel liegt an einer anderen erreichbaren Stelle | Szene neu erfassen und am aktuellen Ort greifen |
| Gesuchter Marker fehlt | Verständliche Meldung und kein blinder Griff zur alten Position |

Das Beispiel fährt zunächst Home, bevor es nach dem Tag sucht. „Marker fehlt“ bedeutet
deshalb nicht „überhaupt keine Armbewegung“. Beobachtet, dass danach kein Griff ausgeführt wird.

**Geschafft, wenn:** Drei vereinbarte Ausgangslagen gelingen und der fehlende Marker
verständlich behandelt wird. Protokolliert Simulation und echte Hardware getrennt.
Eine abgelehnte Bewegung wird untersucht. Derselbe Befehl wird nicht ungeprüft wiederholt.

## K3 Zwei Würfel sortieren

Erweitert euren Ablauf auf zwei unterschiedliche IDs, etwa 200 und 201, und zwei freie
Zielfelder A und B. Welche ID zu welchem Feld gehört, legt ihr als Zuordnung im Code fest.
Erfasst die Szene für den nächsten Griff neu, nachdem der vorherige Würfel abgelegt wurde.

| Prüffall | Erwartetes Ergebnis |
| --- | --- |
| Beide Würfel vorhanden, Startplätze vertauscht | Die IDs landen weiter auf den zugeordneten Zielfeldern |
| Ein benötigter Würfel fehlt | Fehlenden Würfel melden und den betroffenen Auftrag zurückstellen |
| Zielplatz als belegt bekannt | Dort nicht ablegen. Einen freien Platz wählen oder auf eine Entscheidung warten |

Verwaltet die Belegung eurer bekannten Felder zunächst selbst. Stellt Würfel nur bei
stillstehendem Arm um. Prüft danach Szene und Belegung neu.
Eine freie Fläche aus einem alten Bild gilt nach dem Umstellen nicht weiter als bestätigt.

**Geschafft, wenn:** Die Zuordnung bei drei veränderten Ausgangslagen stimmt und euer
Programm fehlende Würfel sowie bekannte belegte Zielfelder nachvollziehbar behandelt.

## K4 Eine Erweiterung wählen

Wählt nach K1–K3 eine Erweiterung und vereinbart ein überprüfbares Ergebnis:

| Erweiterung | Euer Ergebnis |
| --- | --- |
| Muster legen | Eine gespeicherte Anordnung auf bekannten Feldern aufbauen |
| Auftrag auswählen | Bekannte ID und Zielfeld per begrenzter Eingabe wählen. Ungültige Eingaben zurückweisen |
| Kleiner Turm | Zwei bis drei Würfel stabil aufbauen und von oben wieder abbauen. Gemessene Höhe und Greiferfreiraum berücksichtigen |
| Nach einer Pause fortsetzen | In einer geprüften Warteposition pausieren. Nach menschlichem Umstellen und bewusstem „Weiter“ neu erfassen |

Verwendet `r.greifen()` und `r.ablegen(...)`, damit der Server einen gehaltenen Würfel
berücksichtigen kann. Ein gegriffener Würfel kann die Kamera verdecken. Legt ihn vor der
nächsten Messung erst ab. Obere Tags können beim Stapeln verschwinden. `r.sim_szene(...)`
setzt ausschließlich Simulationsszenen zurück und darf nicht Bestandteil des realen Ablaufs sein.
Das fertige `r.stapeln()` ersetzt eure eigene Ablaufsteuerung bei der Turmaufgabe nicht.

Das Eingießen von alkoholfreiem Bier ist eine gesonderte Bonusaufgabe. Sie braucht einen
vorbereiteten, real geprüften Flaschengriff und Kippablauf. Die
Würfelsimulation prüft weder Flaschen noch Flüssigkeiten.

## Ergebnisse und Abgabe

Speichert euren Python-Code und `bericht.md`. Für die Abschlussaufgabe zeigt ihr drei
Wiederholungen und einen behandelten Fehlerfall. Am Tisch greift ihr nur im Stillstand ein.
Niemand verändert Würfel, solange der Arm fahren kann.

| Aufgabe / Dateiversion | Simulation oder Arm | Ausgangslage / Ziel | Erwartung | Beobachtung | Bestanden / offen |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
| | | | | | |
| | | | | | |

Nennt Gerät, Datum, IDs und gewählte Zielfelder. Erklärt eure Zustände und was nach einem
STOPP mit einem eventuell gehaltenen Würfel passieren soll. Beschreibt eine Grenze und
eine nächste Verbesserung. Team-Schlüssel und `verbindung.json` gebt ihr nicht ab.
