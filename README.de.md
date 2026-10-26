# Super Heavy: Rocket Simulator VR

[English](README.md) · **Deutsch**

**Super Heavy: Rocket Simulator VR** ist ein physikalisch genauer Simulator für ein zweistufiges
Schwerlast-Startsystem (Booster + Ship) und seine Begleiter: ein PC-VR-Flugsimulator (OpenXR, HOTAS, Sitzcockpit –
in Arbeit) und eine Missions- und Ingenieurssimulation (Planung, Lenkung, Telemetrie, Validierung). Kein
Arcade-Spiel: Physikalische Korrektheit hat Vorrang, und jedes Modell wird mit dokumentierter Toleranz gegen
Referenzdaten oder in einer Streuungskampagne geprüft (~1.400 automatische Testfälle im Simulationskern, dazu GPU-
und Spielprüfungen).

Ein unabhängiges Hobbyprojekt ohne Verbindung zu SpaceX. Fahrzeugnamen dienen nur dazu, die realen Systeme zu
benennen, denen die Konfigurationen nachempfunden sind.

## Wie er entstanden ist

Dieser Simulator ist vollständig im Dialog mit Claude Code entstanden, im Verfahren des Agentic Engineering: Der Autor
beschreibt Ziele, trifft die Entscheidungen und prüft die Ergebnisse – im Headset, an Bildern und an Messdaten –,
während Claude Code plant, schreibt, testet, prüft und dokumentiert. Keine einzige Zeile Code wurde von Hand
geschrieben oder bearbeitet. Physikkern, Numerik, Darstellung, VR-Anbindung, Tests, Datenpipeline und Release-Pakete
sind alle aus diesem Gespräch hervorgegangen.

## Highlights

**Von der Startrampe zum Mond und zurück, geflogen vom Bordrechner**
- Aufstieg mit optimiertem Nickmanöver (Probeflüge bei jedem Start), Stufentrennung (Hot Staging bei Starship),
  PEG-Lenkung in den Orbit; der Booster wird immer am Turm gefangen.
- Betankung im Orbit: Tankerflüge zu einem Treibstoffdepot mit Rendezvous, Phasing und Treibstoffübergabe; die
  Planung beantwortet, wie viele Tanker eine Mondmission braucht (13 Tanker / 52,5 Tage für das HLS), und fliegt
  die ganze Kampagne.
- Eine im Erdorbit betankte Mondlandefähre (HLS) fliegt Mondtransfer, Kurskorrekturen und das Einschwenken in einen
  polaren Mondorbit in 100 km Höhe oder in den 9:2-NRHO (die Gateway-Bahn), aufgebaut per Mehrfachschießverfahren im
  vollen Kraftmodell Erde-Mond-Sonne; der NRHO wird über einen angetriebenen Mond-Swing-by erreicht.
- Angetriebener Abstieg zum Mondsüdpol auf echtem LOLA-Gelände: Bremsphase nach Apollo-Art (gegen Apollo 11
  validiert: 508,5 s gegenüber 507 s geflogen), Anflug per konvexer Optimierung (SCvx), Endanflug auf zwölf
  schräg gestellten Landetriebwerken; 21 von 21 gestreuten Abstiegen landen innerhalb von 0,8 m. Aufstieg zurück in
  den Orbit mit Triebwerksausfall und Abbrüchen.
- Rückkehr zur Erde: Rückflug-Zündung, ein Bremsmanöver vor dem Eintritt, dann direkter Eintritt zum Startplatz mit
  Flip und Turmfang oder ein Aerocapture in den Erdorbit mit anschließender normaler Rückkehr und Fang.
- Ehrliche Budgets: Missionen, die nicht aufgehen, werden mit dem Treibstoffdefizit abgelehnt statt passend
  gerechnet.

**Booster und Ship kommen heim**
- Booster: Flip, Rückflugzündung mit Luftwiderstand, Abstieg mit Gitterflossen, Landezündung 13 -> 5 -> 3
  Triebwerke auf SCvx-Plänen, die im Flug neu gelöst werden, Fang mit den Turmarmen (21 von 21, auch mit
  Triebwerksausfall).
- Ship: Rückkehrzündung zum nächsten erreichbaren Überflug, Bauchlage-Eintritt mit Prädiktor-Korrektor-Lenkung und
  Grenzen für Wärmestrom, Last und Staudruck, Klappensteuerung, Flip und Fang mit 3 -> 2 Triebwerken (14 von 14
  Rückkehren aus dem Orbit gefangen).
- Bemannte Ship-Rückkehren haben Schleudersitze: Bei jedem Abbruch löst der Bordrechner sie aus, früh und möglichst
  über Wasser; jedes Besatzungsmitglied fliegt als eigener Körper am Fallschirm.
- Falcon 9: Rückkehr zur Landezone und Landung auf dem Drohnenschiff (je 21 von 21), das Drohnenschiff bewegt sich in
  echtem Seegang (Streifentheorie, Grenzen der Deckbewegung); die Zeitabläufe treffen die Webcasts von CRS-35 und
  Starlink 15-25 auf Sekunden.
- Crew Dragon: einzelne Draco-Triebwerke, Eintritt mit Auftrieb, Brems- und Hauptschirme, Wasserung im Bergegebiet.

**Andocken an die ISS**
- Rendezvous und Andocken der Crew Dragon an die ISS (echte NASA-Bahndaten, wo verfügbar): Phasing über Tage,
  Anflugkorridore zu den Ports IDA-2 / IDA-3, Kontaktmodell mit Soft und Hard Capture; Abkoppeln und Rückkehr bis zur
  Wasserung.

**Physikmodelle**
- Starrkörper in 6 Freiheitsgraden mit veränderlicher Masse: Schub und Momente je Triebwerk, Übergänge, Drossel- und
  Schwenkgrenzen, Wiederzündungen, wandernder Schwerpunkt, Stufentrennung, Landebeine, Turmarme, Wasserung, RCS mit
  einzelnen Düsen.
- US-Standardatmosphäre 1976, druckabhängiger Schub und Isp, machabhängige Aerodynamik bis zur freien
  Molekularströmung, Gitterflossen, Klappen, Kapseln, Strahlungsdruck der Sonne mit Erd- und Mondschatten.
- Eintrittserwärmung: Sutton-Graves (konvektiv) plus Strahlungserwärmung aus den Gleichgewichtstabellen der NASA;
  Versagen des Hitzeschilds und Zerbrechen als mögliche Ausgänge.
- Gravitation: Erde, Mond und Sonne als Punktmassen plus Erd-J2; DOP853 mit dichter Ausgabe für Freiflugphasen,
  200 Hz im Antriebsflug; die Simulation läuft in einem eigenen Thread.
- Fahrzeugdaten von Starship V3, ermittelt aus eigener Abschrift der Webcasts der Testflüge 12-14; widerspricht eine
  Anzeige der Physik, gewinnt die Physik.

**Eigene Numerik, keine Black Boxes**
- Eigener Innere-Punkte-Löser für Kegelprogramme zweiter Ordnung (SOCP), geprüft gegen Clarabel; SCvx für alle
  angetriebenen Landungen; PEG, Lambert und Zielsuche im vollen Kraftmodell, eingeschränktes Dreikörperproblem,
  Eintrittskorridore.

**Die echte Erde und der echte Himmel zum echten Zeitpunkt**
- Sonne und Mond aus JPL DE440, Erdorientierung aus dem NAIF-ITRF93-PCK, UTC -> TDB mit Schaltsekunden – validiert
  gegen JPL Horizons (Sonne < 4 cm, Startplätze < 8 cm); scheinbare Positionen (Lichtlaufzeit + Aberration); die
  totale Sonnenfinsternis vom 8.4.2024 erscheint über Texas mit Kontaktzeiten auf 2 s genau.
- Gelände: Copernicus-Höhendaten und Sentinel-2-Bilder um die Start- und Landeplätze (EGM2008-Geoid), der ganze Mond
  aus LOLA, ein nahtloser Übergang von der Rampe in den Orbit; Floating Origin mit float64-Positionen durchgehend
  (Godot 4.7 in doppelter Genauigkeit).
- Atmosphäre: vorberechnete Mehrfachstreuung (Tabellen nach Bruneton), Refraktion aus dem Dichteprofil,
  Augenadaption mit Nachtsehen „wie das Auge“, Finsternisschatten am Himmel.
- Wetter und Wolken: echtes Wetter zum gewählten Zeitpunkt – ERA5-Reanalyse für die Vergangenheit,
  GFS-Vorhersage für die nächsten 16 Tage, typisches Wetter darüber hinaus – als volumetrische Wolken mit
  physikalisch berechnetem Licht (Monte-Carlo-Mehrfachstreuung), Wolkenschatten am Boden und Wolken aus dem Orbit.
  Ein Umbau zu statischen, schärferen Wolken mit vorab berechnetem Licht ist in Arbeit.

**Im Spiel**
- Missionsauswahl nach Ort, Fahrzeug und Mission; reale Missionen starten zu ihrer historischen Startzeit.
- Overlay im Stil der Webcasts (Missionsuhr, Zeitbogen, Rundinstrumente), Außenansichten (Fahrzeug, Turm,
  Drohnenschiff, Bodenstandorte), HUD mit NavBall.
- PC-VR (OpenXR, erprobt mit Pimax): das Overlay als Panel, ein gemeinsames Wolkenbild für beide Augen, ein
  zuschaltbares Leistungsprotokoll (`Start-VR.cmd --perf-log`) und eine Bildratenanzeige.
- Autopiloten fliegen jeden Missionsschritt; selbst fliegen mit Thrustmaster-T.16000M-Stick und TWCS-Schubhebel kommt
  mit dem Cockpit (M7).

**Datengetrieben und frei**
- Fahrzeuge, Triebwerke, Aerodynamiktabellen, Start- und Landeplätze und Missionen sind TOML/CSV-Konfiguration mit
  Herkunftskennzeichen ([S] belegt, [E] geschätzt, [D] abgeleitet); `tools/vehicle_report.py` gibt Massen- und
  Delta-v-Budgets aus.
- Nur freie Daten und Werkzeuge; jede Quelle wird mit ihrer Lizenz erfasst, `CREDITS.md` wird daraus erzeugt.

## Trägerraketen und Raumschiffe

| Trägerrakete / Raumschiff | Was es ist |
|---|---|
| Starship V3 (Super Heavy Block 3 / Ship Block 3) | das zweistufige Startsystem: Booster mit 33 Triebwerken, Ship mit sechs Raptoren, Klappen, Hitzeschild |
| Starship Tanker | Ship-Variante, die Treibstoff zum Depot bringt |
| Starship Depot | das Treibstoffdepot im Erdorbit |
| Starship HLS (+ Startkonfiguration) | Mondlandefähre: ohne Hitzeschild und Klappen, mit Beinen und zwölf schräg gestellten Landetriebwerken |
| Falcon 9 Block 5 | mit Rückkehr zur Landezone oder Landung auf dem Drohnenschiff |
| Crew Dragon | Besatzungskapsel mit Draco-RCS, SuperDracos, Fallschirmen |

Außerdem simuliert: die ISS als Andockziel, Schleudersitze und Fallschirmspringer für die Besatzungsrettung.

## Missionen

| Gruppe | Missionen |
|---|---|
| Starship | in einen niedrigen Erdorbit mit Boosterfang; Testflug 14 (Validierung); Rückkehr nach Starbase (auch bemannt) |
| Betankung | Depotstart; Tanker in den Phasenorbit; Tankerflug zum Depot; die geflogene Betankungskampagne |
| Mond | HLS in den Phasenorbit; HLS-Betankung am Depot und Mondmission; HLS-Mondlandung über LLO, über LLO ab Depot, über NRHO; Rückkehr vom Mond (LLO direkt, auch bemannt; LLO mit Aircatch; NRHO direkt) |
| Falcon 9 | Landung auf LZ-40; Landung auf dem Drohnenschiff (auch bei Seegang); CRS-35 (Validierung); mit Crew Dragon |
| Crew Dragon / ISS | Falcon 9 mit Crew Dragon zur ISS; Crew Dragon aus dem Orbit zur ISS; Abkoppeln und Rückkehr; Crew-Dragon-Rückkehr |

## Stand und Ausblick

Fertig: Himmel, Zeit und Ephemeriden; Flugphysik und Aufstieg; Booster- und Ship-Rückkehr mit Turmfang;
Falcon-9-Landungen und Seegang; Crew Dragon; Mondmissionen im Kern und im Spiel; Andocken an die ISS; Betankung im
Orbit; Besatzungsrettung; Gelände- und Atmosphärendarstellung (M8); echtes Wetter mit volumetrischen Wolken;
Missionsauswahl; Release-Pakete; VR-Leistungsoptimierung. In Arbeit: statische Wolken (vorab berechnetes Licht, klare
Konturen, Wolkenarten, Wolkenbasis auf Kondensationshöhe). Als Nächstes: Regen, Nebel und Blitze, Wind sowie das
VR-Cockpit mit eigenem Fliegen und der Missionsplaner im Spiel (M7). Ansichten und Bedienung des fertigen Spiels:
[docs/ansichten.md](docs/ansichten.md); Modellentscheidungen und Validierungszahlen:
[docs/subsystems.md](docs/subsystems.md) (Englisch).

## Architektur

```
core/      C++20-Simulationskern, ohne Godot-Abhängigkeit, durchgehend float64
bindings/  schlanker GDExtension-Adapter (godot-cpp, precision=double)
game/      Godot-4-Projekt: Darstellung, Oberfläche, VR, Eingabe – Physik läuft nie in GDScript
config/    Fahrzeuge, Triebwerke, Aerodynamiktabellen, Start- und Mondlandeplätze, Missionen (siehe config/README.md)
tools/     Datenpipeline, Validierungsdaten, Telemetrie-Abschrift, Berichte, Paketbau
tests/     Validierungsdaten (JPL Horizons, Webcast-Telemetrie, Referenzlöser)
```

## Installation (Windows)

Jedes GitHub-Release enthält:
- `SuperHeavyRocketSimulatorVR-<version>-setup.exe` – den Installer mit allem (pro Benutzer, ohne Adminrechte). Das
  Erdgelände und die Luftbilder der Startplätze sind optionale Komponenten; ohne sie ist der Boden um die Rampen flach
  und ohne Textur. Startmenü: der Simulator (VR, OpenXR-Runtime gestartet) und sein Desktop-Modus.
- `...-windows.zip` – dasselbe Spiel ohne Installer, mit den Basisdaten spielbar (`Start-VR.cmd`,
  `Start-Desktop.cmd`); `...-data-terrain.zip` und `...-data-imagery.zip` werden in denselben Ordner entpackt.

## Bauen (Windows)

Voraussetzungen: MSVC 2022 Build Tools, CMake, Ninja, SCons, Python 3. Die einmalige Einrichtung (Laufzeitdaten,
CSPICE, der Godot-Editor in doppelter Genauigkeit und godot-cpp) steht in [docs/setup.md](docs/setup.md).

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1                         # Kern + GDExtension + alle Tests
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[weather]" # nur die Tests eines Tags
Start-Desktop.cmd                                                                  # Spiel starten (Start-VR.cmd für VR)
powershell -ExecutionPolicy Bypass -File tools/package.ps1                        # Release-Build: Installer in build/
```

## Quellen

Datenquellen und ihre Lizenzen: [CREDITS.md](CREDITS.md).
