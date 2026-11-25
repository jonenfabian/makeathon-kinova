# Kinova-Arbeitsplatz: Team-Paket

Mit diesem Paket steuert ihr den Kinova-Gen3-Arm selbstständig vom eigenen Laptop aus.
Ihr habt vollen Zugang zum Arm-Arbeitsplatz und arbeitet selbstständig. Dieselben Befehle laufen
auch in einer Simulation auf eurem Laptop. So könnt ihr ohne Arm üben und dann am echten Arm ausprobieren.

## Einrichten und Simulation starten

Installation und Start der Simulation mit `start.py` stehen auf der
[Leitseite des Team-Repositorys](https://github.com/jonenfabian/makeathon-kinova#readme).
Wir empfehlen Python 3.11. Alle Befehle startet ihr im Projektordner, also im Ordner mit `start.py`.
Unter Windows ersetzt ihr `.venv/bin/python` durch `.\.venv\Scripts\python.exe`.

Wartet nach `start.py` auf die Zeile „Dashboard bereit“. Der erste Start nach der Installation kann bis zu
30 s dauern, weil numpy und OpenCV zum ersten Mal geladen werden. Dann zeigt <http://127.0.0.1:8080> im Browser den
Arm als 3D-Modell, die Kamerabilder und alle Werte.

Eure Programme startet ihr in einem zweiten Terminal, ebenfalls im Projektordner:

```bash
.venv/bin/python kinova/beispiele/01_verbinden.py
.venv/bin/python kinova/beispiele/08_eigener_turm.py
```

- Ohne `verbindung.json` verbinden sich die Programme mit der Simulation auf eurem Laptop. Was die Datei bewirkt,
  steht unter [Mit dem echten Arm verbinden](#mit-dem-echten-arm-verbinden).
- `.venv/bin/python start.py --zeitraffer 5` lässt die Simulation fünfmal so schnell laufen.
- `r.sim_szene(wuerfel=[{"id": 200, "x_mm": 420, "y_mm": -10}, …])` legt die Würfel an feste Stellen,
  `r.sim_szene(zufall=3)` zufällig, `r.sim_szene()` wie beim Start.

| Beispiel | Was es zeigt |
|---|---|
| `01_verbinden.py` | Verbindung, Zustand, ob Fahren freigegeben ist und wer das Fahrrecht hat |
| `02_bewegen.py` | Home, Punkte anfahren, Greifer |
| `03_tags_finden.py` | AprilTags auf den Würfeln finden |
| `04_wuerfel_umsetzen.py` | Würfel über seinen AprilTag finden, greifen, woanders ablegen |
| `05_eigene_erkennung.py` | Tag selbst erkennen und mit dem Tiefenbild umrechnen |
| `06_eigenes_dashboard.html` | eigene Bedienseite im Browser |
| `07_bilder_speichern.py` | Farb- und Tiefenbild speichern und ansehen, Bildpunkt umrechnen |
| `08_eigener_turm.py` | Turm aus den Grundbausteinen bauen und wieder abbauen, ohne Tags |

## Mit dem echten Arm verbinden

`verbindung.json` im Projektordner legt fest, mit welchem Server eure Programme sprechen. Ohne diese Datei
verbinden sie sich mit der Simulation auf `127.0.0.1:8080` (Schlüssel `sim`).

1. Euer Laptop ist im selben WLAN wie der Mac Mini am Arm.
2. Kopiert `kinova/verbindung.beispiel.json` in den Projektordner als `verbindung.json`. Tragt die Serveradresse
   des Mac Mini mit `:8080`, den Team-Schlüssel und euren Teamnamen ein. Adresse und Schlüssel stehen in euren
   Geräteangaben. `verbindung.json` kommt nicht ins Git, weil der Schlüssel darin steht.
3. Startet im Projektordner `.venv/bin/python kinova/beispiele/01_verbinden.py`. Steht dort „echter Arm“, seid ihr
   verbunden.

**Zurück zur Simulation:** `verbindung.json` löschen oder darin Adresse `127.0.0.1:8080` und Schlüssel `sim`
eintragen. Ein laufendes Programm behält sein Ziel, bis ihr es neu startet. Die Umgebungsvariablen `KINOVA_ADRESSE`,
`KINOVA_SCHLUESSEL` und `KINOVA_TEAM` haben Vorrang vor der Datei.

Ihr könnt Adresse und Schlüssel auch direkt angeben: `Roboter("192.168.0.20:8080", "abcd-efgh", "Team Blau")`.

## Regeln am echten Arm

- Es fährt immer nur ein Team. Wer fahren will, holt sich das **Fahrrecht**. `with Roboter() as r:` macht das
  automatisch und gibt es am Ende wieder ab. Nach einer Minute ohne Befehl verfällt das Fahrrecht.
- Fahren über das Netz geht nur, wenn am Mac Mini in der lokalen Oberfläche „Fahren für Teams freigeben“
  eingeschaltet ist. Das ist eine Servereinstellung, keine Freigabe einzelner Versuche. `01_verbinden.py` zeigt sie
  als `Fahren freigegeben`. Mit vollem Zugang zum Mac Mini schaltet ihr sie bei Bedarf selbst ein.
- **Strg-C** in eurem Programm hält den Arm an. `r.stopp()` geht immer, auch ohne Fahrrecht.
- Drückt jemand am Mac Mini oder ein anderes Team STOPP, verliert ihr das Fahrrecht. Euer Programm bricht dann beim
  nächsten Befehl mit `KeinZugang` ab. Prüft, was passiert ist, und startet neu.
- Bricht eine Fahrt ab (STOPP, Gelenkanschlag, Kollision), kommt `Abgelehnt`. Der Arm steht dann irgendwo zwischen
  Start und Ziel. Lest vor dem nächsten Befehl die Pose.
- Der rote **Not-Halt** am Arm ist der eigentliche Schutz. Vor jedem Lauf steht eine Person eures Teams am
  Not-Halt. Sie drückt ihn, wenn sich der Arm unerwartet bewegt.
- Niemand greift in den Arbeitsbereich, solange der Arm fahren kann. Würfel stellt ihr nur im Stillstand um.
- Der Server prüft jede Bewegung: Arbeitsraum, Abstand zum Tisch, höchstens 50 mm/s, Gelenkanschläge. Gelenkfahrten
  werden vorher auf Kollisionen simuliert. Bei einer Ablehnung bleibt der Arm stehen und ihr bekommt den Grund.
- Hängt ein Würfel im Greifer, hält der Server auch den Würfel über dem Tisch. Das gilt nur, wenn ihr mit
  `r.greifen()` gegriffen habt. Von Hand mit `r.schliessen()` gegriffene Würfel kennt er nicht.
- Meldet der Arm einen Fehler, prüft den Grund. Quittiert ihn danach am Mac Mini in der lokalen Oberfläche mit
  „Fehler quittieren“. Nach einem Not-Halt entriegelt ihr zuerst den Not-Halt.

## Koordinaten

Alle Längen in Millimetern, Winkel in Grad. Ursprung ist die Roboterbasis.

| Achse | Richtung |
|---|---|
| x | nach vorn, vom Arm weg |
| y | nach links (von hinten auf den Arm gesehen) |
| z | nach oben |

Die Pose ist der **Werkzeugpunkt**. Er liegt zwischen den Fingern, 12 cm unter dem Flansch. `gier_deg` ist die
Richtung, in der die Finger schließen (0° = entlang x).

## Befehle

| Befehl | Was passiert |
|---|---|
| `r.zustand()`, `r.pose()`, `r.gelenke()` | lesen |
| `r.home()` | Home-Pose anfahren |
| `r.fahre_zu(x_mm, y_mm, z_mm, gier_deg=None)` | geradlinig zu einem Punkt |
| `r.schritt(dx_mm=10)` | kleiner Schritt, höchstens 50 mm und 15° |
| `r.gelenk(1, 10)` | ein Gelenk drehen |
| `r.greifer(60)`, `r.oeffnen()`, `r.schliessen()` | Greifer |
| `r.bild("farbe")`, `r.bild("tiefe")` | Kamerabild mit Kamerapose zum Umrechnen |
| `r.tags()` | AprilTags mit Lage in Basiskoordinaten |
| `r.wuerfel_vermessen(x_mm, y_mm, hinfahren=True)` | Kamera über die Stelle fahren (auf etwa 5 cm genau), Würfel oder Turm vermessen, Greifplan für den obersten Würfel. `oben_z_mm` ist die Oberseite, `stapel` die Zahl der Würfel übereinander |
| `r.wuerfel_vermessen(x_mm, y_mm)` | wie oben, aber **ohne zu fahren**: misst nur, was die Kamera gerade sieht. Beide gehen nur mit leerem Greifer, ein hängender Würfel verdeckt die Kamera. Zum Stapeln also erst den Turm vermessen, dann greifen |
| `r.greifen()` | vermessenen Würfel greifen und 5 cm anheben |
| `r.ablegen(x_mm, y_mm, gier_deg=None, unten_z_mm=None)` | gehaltenen Würfel absetzen. Zum Stapeln `unten_z_mm` = `oben_z_mm` und `gier_deg` = `gier_deg` des unteren Würfels |
| `r.abstellen()` | gehaltenen Würfel dort abstellen, wo er gegriffen wurde, dann Home |
| `r.haelt_wuerfel()` | hängt ein mit `greifen()` gegriffener Würfel im Greifer? |
| `r.fahrer()`, `r.letzter_stopp()` | wer das Fahrrecht hat, wann und von wem der letzte STOPP kam |
| `bild.speichern("name")` | Bild als PNG speichern, beim Tiefenbild zusätzlich eingefärbt zum Ansehen |
| `r.stapeln()` | alle Würfel im Arbeitsbereich zu Türmen aus höchstens drei Würfeln stapeln |
| `r.stopp()` | sofort anhalten |
| `r.sim_szene(zufall=3)`, `r.sim_szene(wuerfel=[…])` | nur Simulation: Würfel zufällig oder an feste Stellen legen |

Die Würfel tragen oben AprilTags der Familie 36h11 mit den IDs 200 bis 205. Die schwarze Kante ist 36 mm lang.

## Wenn etwas schiefgeht

**Fahrt abgebrochen** (`Abgelehnt: … abgebrochen (CONTROL_MANUAL_STOP) – STOPP … um …`):
- Jemand hat STOPP gedrückt. Die Meldung sagt, woher er kam: von der lokalen Oberfläche am Mac Mini, von einem
  anderen Team oder von euch selbst (Strg-C).
- Am Arm ist dann **kein Fehler**, nichts muss quittiert werden. Der Armzustand `ARMSTATE_SERVOING_MANUALLY_CONTROLLED`
  ist nach einem STOPP normal.
- Nach einem fremden STOPP ist euer Fahrrecht weg. Der nächste Befehl meldet `KeinZugang: Fahrrecht entzogen`.

So geht es weiter:
1. Nichts mehr senden. Lesen geht immer: `r.pose()`, `r.haelt_wuerfel()`, `r.letzter_stopp()`.
2. Klären, warum gestoppt wurde, zum Beispiel mit dem Team abstimmen, das gestoppt hat.
3. Fahrrecht neu holen: `r.anmelden()` oder ein neues `with Roboter() as r:`.
4. Hängt ein Würfel im Greifer (`r.haelt_wuerfel()` ist True), zuerst ihn wegbringen:
   - `r.ablegen(x, y)` legt ihn an einer freien Stelle ab.
   - `r.abstellen()` stellt ihn dorthin zurück, wo er gegriffen wurde, und fährt nach Home.
5. Euer Programm an der richtigen Stelle fortsetzen. Ein Turm hat nach einem Abbruch vielleicht einen Würfel weniger.
   `stapel` aus `wuerfel_vermessen` sagt, wie viele noch stehen.

**Bewegung abgelehnt** (`Abgelehnt: …` ohne STOPP): Der Server hat die Fahrt nicht gestartet, zum Beispiel wegen
Arbeitsraum, Tischabstand oder Gelenkanschlag. Der Arm steht noch. Ziel ändern, nicht dieselbe Fahrt wiederholen.

**Keine Antwort vom Arm** (`HTTP 503: keine Antwort vom Arm …` oder `Verbindung zum Arm neu aufgebaut, Fahrt
abgebrochen`): Die Verbindung zwischen Server und Arm ist abgerissen. Der Server baut sie nach wenigen Sekunden selbst
neu auf und hält den Arm dabei an, wie bei einem STOPP. Dann weiter wie oben ab Schritt 1. Steht im Zustand
`verbindungsfehler` länger als 15 s, Verbindung und Server am Mac Mini prüfen.
- Solange die Verbindung weg ist, fährt der Arm eine laufende Fahrt zu Ende. Ein STOPP vom Rechner erreicht ihn in
  dieser Zeit nicht. Im Notfall drückt eure Person am Not-Halt den Not-Halt.

**Würfel umgefallen oder Arm meldet einen Fehler** (`fehler` im Zustand ungleich 0): Anhalten und Ursache prüfen. Nicht in den
Arbeitsbereich greifen, solange der Arm sich bewegen kann.

## Eigenes Dashboard

`beispiele/06_eigenes_dashboard.html` im Browser öffnen. Die Seite spricht direkt mit dem Server. Ihr könnt sie
erweitern oder ein Dashboard mit Python bauen, das die Bibliothek benutzt.

Die Schnittstelle ist einfaches HTTP mit JSON:

- Jede Anfrage trägt die Kopfzeile `X-Kinova-Schluessel`.
- Bewegungen tragen zusätzlich `X-Kinova-Fahrrecht`.
- `GET /api/zustand` liefert den Zustand.
- Befehle gehen per `POST /api/...`. Die Pfade stehen in `kinova_team/roboter.py`.

## Was die Simulation nicht kann

Die Simulation zeigt, ob euer Ablauf stimmt. Sie beweist nicht, dass er am echten Arm klappt. Am echten Arm gibt es:

- Messfehler der Kamera von einigen Millimetern
- Licht, Spiegelungen und verdeckte Tags
- Würfel, die rutschen oder kippen
- einen Arm, der sich etwas anders bewegt als das Modell

Die Simulation erkennt nur, wenn Finger oder Würfel an Tisch oder Würfel stoßen. Prüft deshalb am echten Arm erst
langsam und mit Abstand.

## Lizenz

Der Code in diesem Ordner steht unter der MIT-Lizenz, siehe `LICENSE`. Ihr dürft ihn nutzen, ändern und
weitergeben. Der Lizenzhinweis muss dabei erhalten bleiben. Die 3D-Modelle des Arms und des Greifers sowie three.js
haben eigene Lizenzen. Sie liegen jeweils daneben in `simulation/static/`.
