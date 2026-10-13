# Super Heavy Rocket Simulator VR

[English](README.md) · **Deutsch**

Ein physikalisch genauer Simulator für ein zweistufiges Schwerlast-Startsystem (Booster + Ship): ein
PC-VR-Flugsimulator (OpenXR, HOTAS, Sitzcockpit – in Arbeit) und eine Missions- und Ingenieurssimulation
(Planung, Lenkung, Telemetrie, Validierung). Kein Arcade-Spiel: Physikalische Korrektheit hat Vorrang, und
jedes Modell wird mit dokumentierter Toleranz gegen Referenzdaten geprüft.

Ein unabhängiges Hobbyprojekt ohne Verbindung zu SpaceX. Fahrzeugnamen dienen nur dazu, die realen Systeme zu
benennen, denen die Konfigurationen nachempfunden sind.

## Highlights

**Der echte Himmel zum echten Zeitpunkt**
- Sonne und Mond aus JPLs Ephemeride DE440, Erdorientierung aus dem binären NAIF-PCK (ITRF93), Zeitskalen
  UTC → TAI → TT → TDB mit Schaltsekunden – validiert gegen JPL Horizons (Sonne < 4 cm, Startplätze < 8 cm).
- Scheinbare Positionen (Lichtlaufzeit + Aberration) für die Darstellung: Die totale Sonnenfinsternis vom
  8.4.2024 liegt über Texas, Kontaktzeiten auf 2 s genau, der Schatten wird im Shader gezeichnet.
- Floating Origin mit float64-Positionen durchgehend (Godot 4.7.2 mit doppelter Genauigkeit) und skaliertem
  Raum für ferne Körper mit exakter Winkelgröße; Erde aus Blue/Black Marble, Mond aus LROC, HYG-Sternkatalog.

**Flugphysik**
- Starrkörper mit 6 Freiheitsgraden und veränderlicher Masse: Schub und Momente je Triebwerk,
  Start-/Abschalttransienten, Ratengrenzen für Drossel und Schwenk, Wiederzündregeln, Brennschluss bei
  leerem Tank, wandernder Schwerpunkt und Trägheit, Stufentrennung und Hot Staging.
- US-Standardatmosphäre 1976 (≤ 0,2 % zu den Tabellen), druckabhängiger Schub und Isp mit Grenze für
  Strömungsablösung, Mach-abhängige Aerodynamik schlanker Körper mit Übergang zur freien Molekularströmung
  über die Knudsen-Zahl, Strahlungsdruck der Sonne mit Erd- und Mondschatten.
- Gravitation: Punktmassen Erde, Mond und Sonne plus J2 der Erde (jeder Term schaltbar); DOP853 mit
  Dense Output und Ereignissuche für Freiflug, 200-Hz-RK4 im angetriebenen Flug.
- Fahrzeugdaten des aktuellen Starship (V3) aus eigener Auswertung der Webcasts der Flüge 12–14 bestimmt
  (Geschwindigkeit, Höhe, Treibstoffanzeigen, Triebwerkssymbole): Massen, Schub, Booster-Abstiegswiderstand (auf ~1–3 %).

**Lenkung, Navigation und Regelung**
- Aufstiegs-Autopilot: Startazimut mit Korrektur der Erddrehung, Pitch Kick, Gravity Turn, Powered Explicit
  Guidance (PEG) in der Oberstufe – 200-km-Zielorbit auf etwa 1 km genau, auch mit drei bei T+60 s
  ausgefallenen Booster-Triebwerken.
- Manöverknoten mit geregelter Brennausführung; automatische Wahl der Triebwerke, RCS-Translation für
  Manöver unterhalb des Mindestimpulses der Haupttriebwerke (0,5 m/s auf ~1 mm/s genau).
- Lambert-Löser, Zielsuche für den Mondtransfer im vollen Kraftmodell, Kurskorrekturen und Einschwenken in
  den Mondorbit: Eine geplante Erde-Mond-Mission erreicht einen polaren Mondorbit in 100 km Höhe.
- Rückkehr des Boosters zum Startplatz: Wende, Lambert-Boostback mit Schießverfahren inklusive Luftwiderstand,
  Abstieg mit Gitterflossen-Steuerung (Anstellwinkel, transsonisches Blockieren modelliert), Landebrennen
  13 → 3 Triebwerke – alle drei Booster-Generationen landen auf der Rampe.

**Selbst fliegen**
- Thrustmaster T.16000M + TWCS-Schubhebel (Rohachsen über ein SDL-Mapping), Tastatur als Ersatz.
- HUD mit NavBall (Prograd/Retrograd- und Zielmarker), Flugdirektor-Fadenkreuz, geplanter (grün) und
  vorhergesagter (orange) Bahn, Δv-Balken für Manöver.

**Datengetrieben und kostenlos**
- Fahrzeuge (aktuelles Starship V3, Falcon 9 zum Vergleich), Triebwerke, Aerotabellen, Startplätze (Starbase,
  LC-39A) und Missionen sind TOML/CSV-Konfiguration mit Herkunftskennung ([S] belegt, [E] geschätzt,
  [D] abgeleitet); `tools/vehicle_report.py` gibt Massen- und Δv-Bilanz aus.
- Nur freie Daten und Werkzeuge; jede Quelle ist mit Lizenz erfasst, `CREDITS.md` wird daraus erzeugt.

## Stand und Ausblick

Meilensteine M0–M5 sind fertig, M6 (Missionsplanung, Booster-Rückkehr) läuft. Als Nächstes: konvexe
Landeführung für einen präzisen Turmfang (eigener SOCP-Löser, Turm und Fangarme), der Missionsplaner im Spiel,
das VR-Cockpit (M7), hochaufgelöstes Gelände, Atmosphäre und Effekte (M8), Rendezvous, Docking und Betankung
im Orbit (M9). Ansichten und Bedienung des fertigen Spiels: [docs/ansichten.md](docs/ansichten.md).

## Bauen (Windows)

Voraussetzungen: MSVC 2022 Build Tools, CMake, Ninja, SCons, Python 3. Die einmalige Einrichtung
(Laufzeitdaten, CSPICE, Godot-Editor mit doppelter Genauigkeit, godot-cpp) steht in [CLAUDE.md](CLAUDE.md)
(Abschnitt „Build & test“).

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1   # Kern + GDExtension + Tests
Start-Desktop.cmd                                            # Spiel starten
```

## Quellen

Datenquellen und Lizenzen: [CREDITS.md](CREDITS.md).
