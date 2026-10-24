# Super Heavy Rocket Simulator VR

[English](README.md) · **Deutsch**

Ein physikalisch genauer Simulator für ein zweistufiges Schwerlast-Startsystem (Booster + Ship) und seine
Begleiter: ein PC-VR-Flugsimulator (OpenXR, HOTAS, Sitzcockpit – in Arbeit) und eine Missions- und
Ingenieurssimulation (Planung, Lenkung, Telemetrie, Validierung). Kein Arcade-Spiel: Physikalische Korrektheit hat
Vorrang, und jedes Modell wird mit dokumentierter Toleranz gegen Referenzdaten oder in einer Streuungskampagne
geprüft (~600 automatische Testfälle).

Ein unabhängiges Hobbyprojekt ohne Verbindung zu SpaceX. Fahrzeugnamen dienen nur dazu, die realen Systeme zu
benennen, denen die Konfigurationen nachempfunden sind.

## Highlights

**Zum Mond und zurück, geflogen vom Bordrechner**
- Eine in einer 200-km-Erdumlaufbahn betankte Mondlandefähre (HLS-Variante) fliegt Mondtransfer (TLI),
  Kurskorrekturen und das Einschwenken in einen polaren Mondorbit in 100 km Höhe oder in den 9:2-NRHO
  (Near-Rectilinear Halo Orbit, die Gateway-Bahn), der per Mehrfachschießverfahren im vollen Kraftmodell
  Erde-Mond-Sonne aufgebaut wird. Der NRHO wird über einen angetriebenen Mond-Swing-by erreicht (3,2 km/s TLI +
  0,24 + 0,22 km/s): Ein direkter Transfer träfe die Bahn frontal mit 3,5 km/s.
- Angetriebener Abstieg zum Mond-Südpol auf echtem LOLA-Gelände (20-m-Kacheln um den Landeplatz): Bremsphase nach
  einem Lenkgesetz im Apollo-Stil (validiert an Apollo 11: 508,5 s gegenüber 507 s geflogen), Anflug geplant per
  konvexer Optimierung (SCvx) und Endanflug ähnlich Apollos P66 auf zwölf schräg eingebauten Landetriebwerken, die
  durch differenzielles Drosseln steuern. Kampagne: 21 von 21 gestreuten Abstiegen landen auf 0,8 m genau mit ~1 m/s
  auf Hängen bis ~9°.
- Aufstieg zurück in den Orbit mit PEG und Triebwerksausfall-Behandlung; Abbrüche aus 3 km, 1 km und 0,3 km
  erreichen den Orbit.
- Rückkehr zur Erde: Rückflugbrennen (TEI), Korrekturen, ein Bremsbrennen vor dem Eintritt (der Hitzeschild des
  Ship verkraftet keinen vollen Mond-Rückkehr-Eintritt mit 11 km/s – die Simulation zeigt, warum), dann entweder
  Direkteintritt zum Startplatz mit Wende und Turmfang oder ein **Aerocapture** in eine Erdumlaufbahn mit
  anschließendem normalem Deorbit und Fang. Eintrittslenkung mit energielinearem Querneigungsprofil (FNPEG-Prinzip)
  und Skip-Behandlung.
- Ehrliche Bilanzen: Missionen, die nicht aufgehen (z. B. eine schwerere Fähre, die NRHO-Variante mit 120 t
  Leermasse), werden mit dem Treibstoff-Fehlbetrag abgelehnt statt passend getrimmt.

**Booster und Ship kommen heim**
- Booster-Rückkehr zum Turm: Wende, Boostback mit Schießverfahren inklusive Luftwiderstand, Abstieg mit
  Gitterflossen (transsonisches Blockieren modelliert), Landebrennen 13 → 5 → 3 Triebwerke nach SCvx-Plänen, die im
  Flug auf einem eigenen Thread neu gelöst werden, Fang mit den Turmarmen (Kontaktmodell für Arme, Schlitten und
  Pins). Kampagne: 21 von 21 gefangen (< 0,1 m, ~1 m/s Sinkrate), Triebwerksausfall eingeschlossen.
- Ship-Rückkehr aus dem Orbit: Deorbit-Planung an Bord für den nächsten erreichbaren Überflug, Bauch-voran-Eintritt
  mit Prädiktor-Korrektor-Lenkung (Grenzen für Wärmestrom, Lastvielfaches und Staudruck, Querneigungswechsel),
  Klappensteuerung, Wende auf drei Raptoren in 800 m und Fang mit 3 → 2 Triebwerken. Kampagnen: 14 von 14
  Orbit-Rückkehrern gefangen; 20 von 21 gestreuten Wenden gefangen (der Fall mit ausgefallenem Triebwerk bricht
  sicher zum Meer ab).
- Falcon 9 zum Vergleich: Rückkehr zur Landezone (21 von 21 auf 0,01 m) und Landungen auf dem Drohnenschiff
  (21 von 21), mit Zeitabläufen auf Sekunden wie in den Webcasts von CRS-35 und Starlink 15-25.
- Crew Dragon: einzelne Draco-Düsen (RCS-Verteiler + Pulsmodulation), Auftriebs-Eintritt mit versetztem Schwerpunkt
  aus einer modifiziert-Newtonschen Aerotabelle, gereffte Brems- und Hauptschirme, Wasserung in der Bergungszone
  (8 von 8 auf 1,1 km mit ~7 m/s; Deorbit bis Wasserung 50,5 min gegenüber 54 min bei Crew-10).

**Physikmodelle**
- Starrkörper mit 6 Freiheitsgraden und veränderlicher Masse: Schub und Momente je Triebwerk,
  Start-/Abschalttransienten, Ratengrenzen für Drossel und Schwenk, Wiederzündregeln, Brennschluss bei leerem Tank,
  wandernder Schwerpunkt und Trägheit, Stufentrennung, Hot Staging, Landebeine mit Stauchung und Reibung, Wasserung.
- US-Standardatmosphäre 1976 (≤ 0,2 % zu den Tabellen), druckabhängiger Schub und Isp mit Grenze für
  Strömungsablösung, Mach-abhängige Aerodynamik schlanker Körper mit Übergang zur freien Molekularströmung,
  Gitterflossen, Ship-Klappen, stumpfe Kapseln, Strahlungsdruck der Sonne mit Erd- und Mondschatten.
- Eintrittserwärmung: konvektiv nach Sutton-Graves plus **Strahlungserwärmung** aus den Gleichgewichtstabellen der
  NASA (TM-102652, 624 übertragene Werte); Versagen des Hitzeschilds und Zerbrechen als Ergebnis.
- Gravitation: Punktmassen Erde, Mond und Sonne plus J2 der Erde (jeder Term schaltbar); Zustände immer im
  Inertialsystem, mit Erde oder Mond als Bezugskörper. DOP853 mit Dense Output und Ereignissuche für Freiflug,
  200 Hz im angetriebenen Flug; tagelange Freiflüge in großen Schritten.
- Fahrzeugdaten des aktuellen Starship (V3) aus eigener Auswertung der Webcasts der Flüge 12–14 bestimmt
  (Geschwindigkeit, Höhe, Treibstoffanzeigen, Triebwerkssymbole). Widerspricht eine Anzeige der Physik, gilt die
  Physik.

**Eigene Numerik, keine Black Boxes**
- Dünnbesetzter Innere-Punkte-Löser für Kegelprogramme zweiter Ordnung (SOCP), geprüft gegen Referenzlösungen von
  Clarabel; darauf SCvx (sukzessive Konvexifizierung) für alle angetriebenen Landungen (Booster, Ship, Falcon 9,
  Mondfähre).
- Powered Explicit Guidance (PEG), Lambert und Zielsuche im vollen Kraftmodell, eingeschränktes Dreikörperproblem,
  Eintrittskorridore und die höchste zulässige Eintrittsgeschwindigkeit für einen gegebenen Hitzeschild.

**Der echte Himmel zum echten Zeitpunkt**
- Sonne und Mond aus JPLs Ephemeride DE440, Erdorientierung aus dem binären NAIF-PCK (ITRF93), Zeitskalen
  UTC → TAI → TT → TDB mit Schaltsekunden – validiert gegen JPL Horizons (Sonne < 4 cm, Startplätze < 8 cm).
- Scheinbare Positionen (Lichtlaufzeit + Aberration) für die Darstellung: Die totale Sonnenfinsternis vom
  8.4.2024 liegt über Texas, Kontaktzeiten auf 2 s genau, der Schatten wird im Shader gezeichnet.
- Floating Origin mit float64-Positionen durchgehend (Godot 4.7.2 mit doppelter Genauigkeit) und skaliertem Raum
  für ferne Körper mit exakter Winkelgröße; Erde aus Blue/Black Marble, Mond aus LROC, HYG-Sternkatalog.

**Selbst fliegen**
- Thrustmaster T.16000M + TWCS-Schubhebel (Rohachsen über ein SDL-Mapping), Tastatur als Ersatz.
- HUD mit NavBall (Prograd/Retrograd- und Zielmarker), Flugdirektor-Fadenkreuz, geplanter (grün) und
  vorhergesagter (orange) Bahn, Δv-Balken für Manöver.
- Autopiloten fliegen jeden Missionsschritt; der Pilot kann die Steuerung oder Steuerung und Schub übernehmen, der
  Bordrechner plant vom neuen Zustand aus neu.
- Heute im Spiel fliegbar: Starship in den Orbit mit Boosterfang und Ship-Rückkehr, Falcon 9 (RTLS, Drohnenschiff,
  Start mit Crew Dragon) und die Crew-Dragon-Rückkehr. Die Mondmissionen laufen im Simulationskern und seinen Tests;
  ihre Spieloberfläche ist der nächste Schritt.

**Datengetrieben und kostenlos**
- Fahrzeuge (Starship V3 mit HLS-Variante, Falcon 9, Crew Dragon), Triebwerke, Aerotabellen, Start- und Landeplätze
  und Missionen sind TOML/CSV-Konfiguration mit Herkunftskennung ([S] belegt, [E] geschätzt, [D] abgeleitet);
  `tools/vehicle_report.py` gibt Massen- und Δv-Bilanz aus.
- Nur freie Daten und Werkzeuge; jede Quelle ist mit Lizenz erfasst, `CREDITS.md` wird daraus erzeugt.

## Stand und Ausblick

Fertig: Himmel, Zeit und Ephemeriden (M0–M4), Flugphysik und Aufstiegslenkung (M5), Booster-Rückkehr und Turmfang,
Ship-Eintritt / Wende / Fang / Rückkehr aus dem Orbit, Falcon-9-Landungen, Crew Dragon mit RCS, Eintritt und
Fallschirmen sowie die Mondmissionen l1–l5 (Bezugskörper, Fähre und Gelände, Hinflug zu LLO / NRHO, Landung und
Aufstieg, Rückflug mit Direkteintritt oder Aerocapture). Als Nächstes: die Mondmissionen im Spiel (l6), der
Missionsplaner im Spiel, das VR-Cockpit (M7), Gelände- und Atmosphärendarstellung mit Höhendaten der Erde (M8),
Tanker und Betankung im Orbit (M9) sowie das Andocken an die ISS. Ansichten und Bedienung des fertigen Spiels:
[docs/ansichten.md](docs/ansichten.md); Modellentscheidungen und Validierungszahlen:
[docs/subsystems.md](docs/subsystems.md) (Englisch).

## Architektur

```
core/      C++20-Simulationskern, ohne Godot-Abhängigkeit, durchgehend float64
bindings/  schlanker GDExtension-Adapter (godot-cpp, precision=double)
game/      Godot-4-Projekt: Darstellung, UI, VR, Eingabe – Physik läuft nie in GDScript
config/    Fahrzeuge, Triebwerke, Aerotabellen, Start- und Mondlandeplätze, Missionen (siehe config/README.md)
tools/     Datenpipeline, Validierungsdaten, Telemetrie-Auswertung, Berichte
tests/     Validierungsdaten (JPL Horizons, Webcast-Telemetrie, Referenzlöser)
```

## Installation (Windows)

Jedes GitHub-Release enthält:
- `SuperHeavyRocketSimulatorVR-<Version>-setup.exe` - den Installer mit allem (pro Benutzer, ohne Adminrechte).
  Erdgelände und Luftbilder der Startplätze sind abwählbare Komponenten; ohne sie ist der Boden um die Startplätze
  flach und ohne Bild. Startmenü: der Simulator (VR, OpenXR-Laufzeit aktiv) und sein Desktop-Modus.
- `...-windows.zip` - dasselbe Spiel ohne Installer, mit den Basisdaten spielbar (`Start-VR.cmd`,
  `Start-Desktop.cmd`); `...-data-terrain.zip` und `...-data-imagery.zip` werden in denselben Ordner entpackt.

## Bauen (Windows)

Voraussetzungen: MSVC 2022 Build Tools, CMake, Ninja, SCons, Python 3. Die einmalige Einrichtung
(Laufzeitdaten, CSPICE, Godot-Editor mit doppelter Genauigkeit, godot-cpp) steht in [CLAUDE.md](CLAUDE.md)
(Abschnitt „Build & test“).

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1         # Kern + GDExtension + alle Tests
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Fast   # ohne die langen Streuungskampagnen
Start-Desktop.cmd                                                  # Spiel starten
powershell -ExecutionPolicy Bypass -File tools/package.ps1        # Release-Build: exe, ZIPs, Installer in build/
```

## Quellen

Datenquellen und Lizenzen: [CREDITS.md](CREDITS.md).
