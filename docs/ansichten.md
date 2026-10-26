# Ansichten und Bedienung im fertigen Simulator

Stand: 22.10.2026. Lebendes Dokument: Es wird bei jeder Änderung an Ansichten oder Bedienung mitgepflegt.
Markierungen: **[Entschieden]** = mit dem Nutzer abgestimmt, **[Vorschlag]** = Entwurf, noch zu bestätigen.

## 1. Grundsätze

- **[Entschieden]** Alle Ansichten gibt es auf dem Desktop und in VR (OpenXR, PC-VR). VR und Desktop sind
  getrennte Startmodi (Muster aus Stunt Track Racer VR, siehe Plan M7): `Start-Desktop.cmd` und `Start-VR.cmd`
  (Headset an, OpenXR-Laufzeit läuft; ohne Headset zeigt das Spiel eine Meldung und beendet sich).
- **[Entschieden]** Die Oberfläche ist deutsch, Einheiten SI (Anzeige: km, km/h, m/s, t, MN, kPa).
- **[Entschieden]** Alle Eingabebelegungen (HOTAS, VR-Controller, Tastatur, Maus) sind änderbar und werden
  über einen **Einstellungsassistenten** abgefragt (Abschnitt 7).
- **[Vorschlag]** Jede Ansicht zeigt die Simulationszeit (UTC) und den Zeitraffer; im Flug zusätzlich T+.

## 2. Übersicht

| Ansicht | Zweck | Hauptbedienung | Meilenstein |
|---|---|---|---|
| Missionsplanung | Mission entwerfen: Start, Ziele, Landeplätze, Manöver | 6DOF-Navigation (HOTAS / VR), Maus | M6 |
| Cockpit | Fliegen aus dem Pilotensitz | HOTAS, Berührung der Bildschirme | M7 |
| Außenansichten | Fahrzeug von außen, Webcast-Overlay, Kameras an Bord | Maus / Kopf, Tab | Webcast-Ansicht vorhanden (Desktop + VR), Ausbau M7/M8 |
| Karte | Erde-Mond-System, Bahnen, Manöver im Flug | wie Missionsplanung | vorhanden, Ausbau M6 |
| Auswertung | Telemetrie, Soll/Ist, Export | Maus | M6 |

## 3. Missionsplanung

Ein 3D-Raum mit Erde und Mond an ihrer echten Position zur gewählten UTC-Zeit.

**Inhalte**
- Startplatz, Startzeit/Startfenster, Zielorbit, Landeplätze auf Erde und Mond (Booster, Ship).
- Soll-Bahn mit Manöverknoten; Δv-Budget je Stufe gegen den verfügbaren Treibstoff.
- Ergebnis: Missionsdatei (`config/missions/*.toml`), die der Bordcomputer im Flug abarbeitet.

**Bahnen bearbeiten mit der Maus** [Entschieden, Interpretation zu bestätigen]
- Ein Bahnpunkt (z. B. Manöverknoten, Bahnhochpunkt) wird angeklickt und mit der Maus gezogen.
- Die Bewegung hat 2 Freiheitsgrade: Der Punkt bleibt auf einer Kugel um die aktuelle Kamera (gleicher
  Abstand) und folgt der Maus in X/Y der Bildschirmebene.
- [Vorschlag] Mausrad während des Ziehens = Abstand zur Kamera (3. Freiheitsgrad), Umschalt + Ziehen =
  Bewegung entlang der Bahn (Zeitpunkt des Knotens).
- Die Bahn wird während des Ziehens live neu gerechnet (Soll grün, Ist orange).

**Navigation im Raum mit dem HOTAS: 6 Freiheitsgrade** [Entschieden: Achsenaufteilung, Vorschlag: Zuordnung]

| Gerät | Achse | Freiheitsgrad | Wirkung [Vorschlag] |
|---|---|---|---|
| T.16000M | Stick vor/zurück | Nicken | Blick hoch/runter (Drehrate) |
| T.16000M | Stick links/rechts | Gieren | Blick links/rechts (Drehrate) |
| T.16000M | Drehung | Rollen | Kamera um die Blickachse drehen |
| TWCS | Mini-Stick X | seitlich | nach links/rechts bewegen |
| TWCS | Mini-Stick Y | vertikal | nach oben/unten bewegen |
| TWCS | Schubhebel | vorwärts | Fluggeschwindigkeit nach vorn (0 … max) |
| TWCS | Wippe | rückwärts | rückwärts (der Hebel hat keine Mittelraste) |

- [Vorschlag] Die Geschwindigkeit skaliert mit dem Abstand zum nächsten Körper (nahe am Fahrzeug Meter pro
  Sekunde, im Erde-Mond-System tausende Kilometer pro Sekunde), damit dieselbe Hebelstellung überall passt.
- [Vorschlag] Taste am Stick: "Auf Objekt zentrieren" (Erde, Mond, Fahrzeug, gewählter Bahnpunkt).

**Navigation mit VR-Controllern: 2 Mini-Sticks = 4 Freiheitsgrade + Ergänzungen** [Vorschlag]

| Eingabe | Wirkung |
|---|---|
| linker Stick vor/zurück | vorwärts/rückwärts bewegen |
| linker Stick links/rechts | seitlich bewegen |
| rechter Stick links/rechts | gieren (in 30°-Schritten oder stufenlos, wählbar) |
| rechter Stick vor/zurück | nicken |
| linker Griff gehalten + linker Stick vor/zurück | hoch/runter (5. Freiheitsgrad) |
| rechter Griff gehalten + rechter Stick links/rechts | rollen (6. Freiheitsgrad) |
| beide Griffe: Welt greifen und ziehen/drehen/skalieren | direkte Manipulation wie bei VR-Kartenanwendungen |
| Abzug auf Bahnpunkt | Punkt greifen und frei im Raum ziehen |

- Hinweis Komfort: Rollen und Nicken der Welt kann in VR Übelkeit auslösen; deshalb wählbar
  (aus / stufenweise / stufenlos) und mit Vignette.

## 4. Cockpit

Hauptansicht im Flug. Der Spieler sitzt angeschnallt im Raumanzug im Pilotensitz; in VR mit freier
Kopfbewegung (6DOF), am Desktop mit Umschauen per Maus. F12 setzt die Sitzposition zurück.

**Zwei Varianten je nach Fahrzeug** [Entschieden]
- **Crew-Dragon-Stil**: bekannte Anordnung (Sitzreihe, Konsole mit drei großen Bildschirmen, wenige
  physische Tasten unterhalb), leicht abgewandelt.
- **Starship-Stil**: eigenes Crewdeck in der Nasenspitze mit großen Fenstern. Offizielle Cockpit-Entwürfe
  gibt es nicht (veröffentlicht sind nur HLS-Luftschleuse und Renderings mit großen Fenstern im oberen
  Bereich); die Gestaltung ist daher unsere eigene. Dieselben drei Bildschirme wie im Crew-Dragon-Stil.

**Die drei Bildschirme** [Entschieden]

```
 ┌───────────────────┐ ┌───────────────────────────────┐ ┌───────────────────┐
 │  AUSSENANSICHT    │ │  STEUERUNG & STATUS           │ │  AUSSENANSICHT    │
 │  BOOSTER          │ │  (Crew-Dragon-Design)         │ │  SHIP             │
 │                   │ │                               │ │                   │
 │  Kamera am        │ │  Flugdirektor / NavBall       │ │  Kamera am Ship   │
 │  Booster, nach    │ │  Höhe, Geschwindigkeit, q     │ │  (nach hinten,    │
 │  der Trennung     │ │  Δv-Balken, Manöverknoten     │ │  Triebwerke,      │
 │  Booster-Rückflug │ │  Triebwerke, Tanks, Autopilot │ │  Hitzeschild)     │
 │                   │ │  Abbruch / Stufentrennung     │ │                   │
 └───────────────────┘ └───────────────────────────────┘ └───────────────────┘
```

- Links: Außenansicht Booster; nach der Stufentrennung begleitet sie den Booster beim Rückflug und
  Abfangen am Turm. Im HUD steht dazu der Zustand des Turms: „Turm: offen / schließt / gefangen /
  verfehlt / Abbruch“, die Phase der Booster-Führung „Booster <Phase> (SCvx/explizit)“ und am Ende
  „Ergebnis gefangen / gelandet / Wasserung / Aufprall“ (heute schon im Desktop-HUD).
- Mitte: alles zu Steuerung und Status im heutigen Crew-Dragon-Design (dunkler Hintergrund, klare
  Typografie, Berührflächen): Flugdirektor, NavBall, Bahnwerte, Δv-Balken, Manöverknoten, Triebwerks-
  matrix (33/6 Raptors), Tanks, Autopilot, Stufentrennung, Abbruch.
- Rechts: Außenansicht Ship. Im Wiedereintritt zeigt das HUD Wärmestrom (kW/m²) und Wärmelast (MJ/m²),
  „!!! Hitze / Last nahe der Grenze !!!“ ab 80 % einer Grenze, „Abprall: zurück über 100 km“ und am Ende
  „Hitzeschild versagt“ / „Strukturüberlast“ (heute schon im Desktop-HUD). Die Eintrittslenkung zeigt Phase
  („Eintritt“, „Endanflug“, „Tor“), Soll-/Ist-Anstellwinkel, Soll-Rollwinkel, die vorhergesagte Ablage längs/quer
  zum Ziel und „Korridorgrenze“, wenn eine Grenze den Rollwinkel beschränkt (heute schon im Desktop-HUD).
  Ab dem Tor zeigt es Flip und Landebrennen des Ships (Phase, Lenkung SCvx) und das Ergebnis wie beim Booster
  (heute schon im Desktop-HUD).
  Rückkehr aus dem Orbit (Taste R am Desktop): der Bordcomputer plant den Deorbit-Brennvorgang für den nächsten
  Überflug des Startplatzes in Reichweite; das HUD zeigt „Rückkehr Δv … m/s (… t), Quer … km“, „Eintritt in h:mm:ss“
  und den Manöverknoten mit „Zündung in h:mm:ss“; ohne Plan den Grund („Zu wenig Treibstoff …“, „Kein Überflug …“).
  [Vorschlag] Dieselben Werte als Soll/Ist-Anzeige im Flugdirektor (Steuerungsstufen: der Spieler kann rollen,
  der Bordcomputer rechnet das Soll nach).
- [Vorschlag] Jeder Außenbildschirm hat 3–4 umschaltbare Kameras (Berührung oder HOTAS-Taste).
- Bedienung: HOTAS fliegt; die Bildschirme werden in VR mit dem Finger des Controllers berührt, am Desktop
  per Maus.

**Steuerungsstufen** [Entschieden 2026-10-13]
- Grundsatz: Die Autopiloten von Booster und Ship können jeden geplanten Missionsschritt allein fliegen
  (Aufstieg, Stufentrennung, Rückflug und Fang, Bahnmanöver, Wiedereintritt, Landung bzw. Fang des Ships).
- Optional übernimmt der Spieler:
  - **Lenkung von Hand**: Lage/Steuerung beim Aufstieg und mit dem Starship per HOTAS, der Schub bleibt
    beim Autopiloten.
  - **Experte**: Schub und Lenkung von Hand.
  - **Im All genauso**: Entweder zündet der Autopilot die Steuerdüsen nach der Vorgabe des Bordcomputers und
    der Spieler schaut zu, oder der Spieler steuert die Steuerdüsen selbst und hält das Schiff mit dem
    Steuerkreuz auf der aktuellen Soll-Trajektorie des Bordcomputers ausgerichtet („in der Mitte halten“,
    wie beim Flugdirektor).
- In jeder Stufe rechnet der Bordcomputer das Soll aus dem tatsächlichen Zustand laufend nach (Flugdirektor,
  Brennschluss, Manöver, Rückflugbedarf des Boosters) und zeigt es an; der Autopilot kann jederzeit wieder
  übernehmen.
- [Vorschlag] Umschalten über eine HOTAS-Taste und den mittleren Bildschirm; die aktive Stufe steht im HUD.

**Crew** [Entschieden]
- Der Spieler fliegt allein (Kommandant, mittlerer Sitz).
- Zwei weitere Crewmitglieder sitzen sichtbar daneben (Raumanzug, Kopf- und Handbewegungen) und machen
  Ansagen.
- [Vorschlag] Ansagen (Sprachausgabe + Untertitel, abschaltbar):
  "Triebwerke gezündet", "Abheben", "Max-Q", "Drossel hoch", "MECO", "Stufentrennung bestätigt",
  "Ship-Triebwerke laufen", "SECO", "Orbit erreicht", "Manöver in 1 Minute", "Zündung", "Brennschluss",
  Warnungen (Triebwerksausfall, Treibstoff niedrig, Lagefehler).

## 5. Außenansichten

### Webcast-Overlay [Entschieden, umgesetzt 2026-10-22]

Die Außenansicht trägt ein Overlay im Stil der Starship-Webcasts (Spec `docs/superpowers/archive/specs/2026-10-22-external-view-design.md`).
Der alte Text-HUD mit NavBall bleibt hinter **F1** (aus im Normalzustand), bis es das Cockpit gibt.
- **Oben:** UTC der Simulation (der Zeitraffer steht im Zeit-Panel, nicht im Overlay). Darunter, kleiner, die
  Ortszeit am Ort der **Kamera** (nicht des betrachteten Objekts), z. B. "Ortszeit 13:42:05 CDT (UTC−5)"; hat die
  Zone in tzdata keine Abkürzung, nur der Versatz ("Ortszeit 17:30:00 (UTC+5:30)"). Weicht das Ortsdatum vom
  UTC-Datum ab, stehen Tag und Monat davor ("Ortszeit 20.10. 19:28:00 CDT (UTC−5)" unter "2024-10-21 00:28:00 UTC"). Sichtbar, solange die Kamera
  höchstens 100 km (Kármán-Linie) über dem WGS84-Ellipsoid steht; darüber, im All und am Mond ausgeblendet, ebenso
  ohne die Zeitzonendaten. Zonen: timezone-boundary-builder (mit Ozeanen, Raster 0,02°, an Grenzen bis ~1 km
  Unschärfe), Regeln: IANA tzdata, Umstellungen 1990-2060 sekundengenau (`game/util/local_time.gd`).
  [Entschieden 2026-10-25] Nur unter der UTC des Overlays, nicht im Zeit-Panel.
  Darunter die **Wetterzeile** (auch im Zeit-Panel): woher
  das Wetter der Wolken kommt, z. B. "Wetter: ERA5, 08.04.2024 18:00 UTC" (Wiederanalyse, bis wenige Tage vor heute),
  "Wetter: GFS-Vorhersage vom 25.10. 00 UTC, +54 h" (bis 16 Tage voraus), "Wetter: typisch (Analogtag 08.04.2025)"
  (weiter weg in der Zeit, oder solange nichts geladen ist / ohne Netz: das mitgelieferte typische Wetter des Monats
  und der 6-Stunden-Zeit), im hohen Zeitraffer "Wetter: Stand 18:00 UTC" (das letzte Wetter wird gehalten),
  "Wetter: wird geladen ..." und "Wetter: aus" (Schalter aus; "aus (keine Daten)", wenn nicht einmal das typische
  Wetter da ist). Das Netz wird nie abgewartet: ohne Verbindung oder bei einem Fehler fliegt die Szene mit dem letzten
  Stand oder dem typischen Wetter weiter und sagt es hier; Wechsel zwischen Quellen und Stunden werden übergeblendet
  (kein Springen der Wolken).
- **Schalter "Wetter"** im Zeit-Panel (Kontrollkästchen unter den Voreinstellungen, gemerkt in den Einstellungen, an
  als Standard): aus = keine Wolken, die Szene wie vor dem Wetter (Pixel für Pixel), und kein Download. Die Wolken
  sind echte Volumen, überall auf der Erde sichtbar: von unten am Startplatz, beim Aufstieg hindurch (innen ein
  helles Grau, oben ein Wolkenmeer), aus dem Orbit als Decke mit Schatten und aus der Mondrichtung als Erde mit Wolken.
- **Zeitbogen:** flacher Bogen oben in der Mitte, "jetzt" am Scheitel. Ereignisse laufen rechts herein und links hinaus,
  die Enden blenden aus. Die Zeitskala ist gestaucht (grob nach relativer Zeit, kein strenger Maßstab) und um "jetzt"
  symmetrisch; sie richtet sich nach den nächsten zwei bis drei Ereignissen und gleitet ohne Sprünge. Beschriftungen
  stehen tangential am Bogen und wechseln zwischen oben und unten; es erscheinen keine Teilnamen (Booster, Ship) am Bogen.
- **Uhr:** T-/T+ unter dem Scheitel des Bogens, darunter der Missionsname. T-0 ist der Start (Abheben), bei
  Missionen ohne Start die erste Zündung bzw. das Abkoppeln.
- **Ereignisse:** Vergangene Ereignisse sind volle Punkte (Abheben, Max Q, MECO, Stufentrennung, SECO, Boostback,
  Landebrennen, Fangen, Zündung, Abkoppeln ...); MECO/SECO zählen ab dem Abschaltbefehl. **Vorhersagen des Bordrechners**
  sind hohle Punkte; ein Ereignis erscheint erst, wenn der Bordrechner seinen Zeitpunkt festgelegt hat.
- **Anzeigen:** links Zustand des Fahrzeugs in dieser Rangfolge (Triebwerke, Treibstoff, Wärmestrom, Wärmelast,
  g-Last, Mach, Staudruck), rechts die
  Bahn, je nach Situation: **Annäherung** (Abstand, Annäherungsgeschwindigkeit, Höhe, Geschwindigkeit), **Transfer**
  (Abstand und Geschwindigkeit zu Erde und Mond, Höhe > 5000 km und Apoapsis > 100000 km), **Orbit** (inertiale
  Geschwindigkeit, Höhe, Apo-/Periapsis), sonst (Oberflächengeschwindigkeit, Höhe, Vertikalgeschwindigkeit,
  inertiale Geschwindigkeit). Die Höhe-Anzeige zeigt die Höhe über dem Boden darunter (Gelände, sonst der flache
  Boden der Physik): 0 auf der Rampe wie im Webcast; die Schwellen der Situationen (Orbit, Transfer) vergleichen die
  Höhe über dem Ellipsoid. Höchstens 4 Anzeigen je Seite; sinnlose Werte (Mach/g am Boden) werden ausgeblendet und
  neue Anzeigen halten 5 s. Treibstoff ist **eine** Anzeige: Treibstoff (Haupt- und Header-Tanks), also aller
  Treibstoff der Haupttriebwerke des Körpers in Prozent der Füllung. Das Triebwerksbild
  hat dieselbe Größe wie die runden Anzeigen. Die Zuordnung soll später datengetrieben in
  `config/ui/webcast_gauges.toml` stehen und vom Einstellungsassistenten änderbar sein [Entschieden, Folgeplan].
- **Bedienung (Desktop):** rechte Maustaste halten = Nicken und Drehen um die Lotrechte; Mausrad = Zoom
  (logarithmisch); die linke Taste bleibt frei. Die Kamera folgt zunächst dem gesteuerten Fahrzeug, auch über jede
  Trennung hinweg (nach dem Hot Staging dem Ship, bei Falcon 9 der Dragon). **Tab / Umschalt+Tab** wechseln das Objekt,
  dem die Kamera folgt (Booster, Ship, Kapsel, Station ...), und halten die Kamera auf diesem Objekt fest; schaltet
  Tab wieder auf das gesteuerte Fahrzeug, folgt sie ihm erneut (ebenso, wenn das gewählte Objekt verschwindet oder
  ein neuer Flug beginnt). **F1** alter HUD. **F12** zentriert in VR neu (außerdem automatisch 2 s nach dem
  Start).
- **VR:** das Overlay liegt auf einer gekrümmten Fläche (Radius 2 m, 64° horizontal) an der Grundpose des Kopfes
  (nach F12 neu gesetzt), nicht am Kopf festgeklebt; das Zeit-Panel liegt oben links auf derselben Fläche und wird mit
  einem Mauszeiger bedient (die Maus ist gefangen; Tab, F1 und F12 gehen an das Spiel, nicht an das Panel). Die
  VR-Sitzung mit dem Headset ist bestanden (2026-10-23).
- **Bahnlinien** [Entschieden 2026-10-23]: grün die Soll-Bahn des Bordrechners (Zielorbit), weiß die geflogene Bahn
  jedes Objekts (Booster, Ship, Kapsel ...), erdfest mitgeschrieben (Aufstieg und Rückflug bleiben über der Rampe; im
  Orbit wandert sie gegen die grüne Bahn) und erst ab 1 km hinter dem Objekt langsam eingeblendet. Beide als weiches
  Band gleicher Pixelbreite, von Erde und Fahrzeugen verdeckt, hinter dem Erdrand abgeschnitten und dort, wo die
  Sichtlinie die Atmosphäre streift, ausgeblendet. Die orange Freiflug-Vorhersage entfällt (sie sprang ständig).
- **Autopilot** [Entschieden 2026-10-22]: In den Außenansichten fliegt immer der Autopilot. Beim Wechsel in die
  Außenansicht übernimmt er das gesteuerte Fahrzeug, falls er aus ist: auf der Rampe der Aufstiegs-Autopilot (wie
  Taste P), im Orbit ohne Plan der Orbit-Zustand des Autopiloten. Handsteuerung (Nicken/Gieren/Rollen, Schub, HOTAS)
  und die Einzelaktionen I (Zünden), L (Klammern lösen), K (Aus), G (Stufentrennung), M (Steuermodus) und
  X (Autopilot aus) sind hier ohne Wirkung; es bleiben die Befehle an den Bordrechner: P (Aufstieg), O (Zirkularisieren),
  R (Rückkehr zum Turm) und die Annäherungsbefehle. Handsteuerung gibt es später nur im Cockpit und nur für das gerade
  gewählte Fahrzeug.
- **Missionsstart** [umgesetzt 2026-10-24]: Unter "Start vorbereiten" im Zeit-Panel stehen drei Auswahlfelder und
  der Knopf **"Starten"**: **Ort** (erst die Startrampen von `config/launch_sites.toml`, dann die Startorte im Flug aus
  `config/start_places.toml`: "Erdorbit", "ISS (angedockt)", "Mondumlaufbahn"; nur Orte, an denen etwas startet), **Fahrzeug**
  (die Fahrzeuge der Rampe, `vehicles` in launch_sites.toml, und die Fahrzeuge der Missionen dieses Orts) und
  **Mission** (an einer Rampe zuerst "Freier Flug", dann die Missionen mit diesem `[mission] start_place` und
  `start_vehicle`, nach Namen). Ort füllt Fahrzeug neu, Fahrzeug füllt Mission neu; die bisherige Wahl bleibt, wenn
  sie dort noch angeboten wird. Die Auswahl liest `game/ui/mission_catalogue.gd` nur aus `config/`.
  - "Freier Flug": das Fahrzeug steht auf der Rampe, der Countdown läuft 60 s als T-
    (`FlightController.LAUNCH_COUNTDOWN_S`), und der Aufstiegs-Autopilot hebt zum T-0 ab. Ohne Abheben bleibt die Uhr
    bei T-0 stehen (wie ein Startabbruch auf der Rampe), statt auf der Rampe T+ zu zählen.
  - Eine Mission startet über `start_mission` mit `wait_on_pad`: ein Aufstieg von der Rampe mit demselben Countdown
    (ein eigenes `[mission] countdown_s` der Mission gilt vor), mit Startfenster (ISS) wartet das Fahrzeug auf der
    Rampe bis zum Fenster; Missionen mit `liftoff_utc` stehen zu ihrem historischen Start. Die Betankungsflüge von der
    Rampe (Depotstart, Tanker, HLS) haben keinen 60-s-Countdown: sie stehen eine Zündvorlaufzeit vor ihrer
    konfigurierten Epoche bzw. ihrem Fenster auf der Rampe (liegt diese vor der Uhr, springt die Uhr zurück; liegt sie
    danach, wartet das Fahrzeug auf der Rampe). Missionen im Flug (Rückkehr der Crew-Kapsel, Mondmissionen, ...)
    beginnen zu ihrer eigenen Epoche. Danach springt die Weltuhr auf den Beginn der Mission (`mission_start_tdb_s`,
    auch rückwärts), der Zeitraffer auf 1x, und die Kamera folgt dem Fahrzeug. Die geflogene Betankungskampagne
    beginnt immer zu ihrer eigenen Epoche.
  - Solange der Start läuft (bis knapp eine Minute), steht die Weltuhr still, "Starten" ist gesperrt und darunter
    steht **"Start wird geplant ..."**; das letzte Ereignis zeigt bei Starts von der Rampe "Startvorbereitung: Bahn
    wird berechnet", sonst "Mission wird gestartet". Eine Ablehnung steht dort als **"Start abgelehnt: <Grund>"**,
    und der Knopf ist wieder frei; hat der abgelehnte Start schon ein Fahrzeug gesetzt (z. B. eine Mondlandung, deren
    Planung scheitert), geht die Weltuhr zu dessen Epoche.
  `render_check` startet Missionen über `--mission=` direkt (ohne Countdown; `--wait_on_pad` lässt sie auf der Rampe
  auf ihr T-0 oder Fenster warten).
- **Startvorbereitung** [Entschieden 2026-10-22, umgesetzt]: Vor jedem Start von der Rampe (freier Start, Taste P auf
  der Rampe, jede Mission) berechnet der Bordrechner die Aufstiegsbahn zum Ziel (Kippwinkel, Bergung der ersten Stufe:
  Fangturm, Landezone oder Drohnenschiff). Solange er rechnet (Sekunden bis knapp eine Minute), steht das Fahrzeug auf
  der Rampe, die Weltuhr und das T- stehen still, und das Webcast-Overlay zeigt in der Namenszeile (und als letztes
  Ereignis) **"Startvorbereitung: Bahn wird berechnet"**; danach läuft der Countdown weiter. Ist das Ziel mit der
  Bergung nicht erreichbar, bleibt das Fahrzeug auf der Rampe, der Autopilot aus, und die Meldung lautet
  **"Start abgelehnt: <Grund>"** (z. B. "Start abgelehnt: Kein Fangturm am Startplatz.", "Ziel mit Booster-Rückkehr
  nicht erreichbar (...)"). Beim Missionsstart (`start_mission` einer Mission, die von der Rampe startet) erscheint
  die Anzeige ebenso, schon bevor das Fahrzeug zu sehen ist: Platzierung und Planung laufen in einem Befehl, das
  Fahrzeug erscheint erst mit dem fertigen Plan.

### Boden und Gelände [Entschieden, umgesetzt 2026-10-24, M8 Plan B]

- Unterhalb des Skalenraums (näher als 15 000 km) zeichnet ein gestreamtes Gelände Erde und Mond: genau die Oberfläche,
  auf der die Physik steht (Relief, ebene Plateaus unter Rampen und Landezonen, Meer auf dem Geoid). Die Auflösung
  folgt dem Abstand zur Kamera; nahe den Startplätzen bis 1 m Gitterweite (3DEP), sonst GLO-30 / ETOPO bzw. LDEM_16
  und die 20-m-Kacheln der Mondlandeplätze.
- Luftbilder: NAIP (0,6 m) an Starbase und Cape, darunter ein eigenes Sentinel-2-Komposit (10 m), außerhalb die
  globale Tagestextur; Übergänge ohne Farbsprung und ohne sichtbare Kante (aus der Ferne weich, kleine Bildflecken
  blenden aus großer Entfernung ganz aus).
- Meer: See mit Fresnel-Spiegelung, Sonnenglitzern und Wellen (Normalen, keine Geometrie). Ohne Seegang sanfte
  Wellen; hat die Mission Seegang, folgen die Wellen ihm (Wellenlänge aus Tp, Laufrichtung, Steilheit aus Hs; nur
  Schattierung, nicht phasengleich mit der Deckbewegung). Das Meer spiegelt den berechneten Himmel und geht in der
  Ferne nahtlos in den Dunst am Horizont über (siehe "Himmel, Dunst und Dämmerung").
- Drohnenschiff [umgesetzt 2026-10-24]: ist ein Drohnenschiff die Landezone, steht es dort als schlichter grauer Körper
  (Rumpf mit Tiefgang und Freibord, Deck 91 x 52 m mit Flügeln, niedrige Wände an den Längsseiten, keine Logos) und hebt,
  rollt und stampft mit der Deckbewegung der Physik; die gelandete Stufe steht auf dem gezeichneten Deck.
- Seegang-Anzeige im Zeit-Panel, solange ein Drohnenschiff die Landezone ist: "Seegang Hs 2.5 m, Tp 8 s - Deck: Hub
  +0.26 m, Rollen +0.0°, Stampfen -0.8°" (Hub über der Ruhelage, Rollen Steuerbord nach unten positiv, Stampfen Bug
  hoch positiv); bei ruhiger See "Seegang ruhig - Deck in Ruhe".
- Kein Aufpoppen: Detailstufen gehen gleitend ineinander über (Geomorphing); spät geladene Kacheln gleiten in einer
  halben Sekunde aus der gröberen Fläche heraus. Ein Sprung zu einem anderen Ort (Tab) lädt in rund 8 s nach,
  grob zuerst.
- Mond: keine Schlagschatten (Kraterränder, Polschatten) - ein späterer Teil; am Pol bei tiefer Sonne ist der
  Boden daher sehr dunkel.

### Himmel, Dunst und Dämmerung [Entschieden, umgesetzt 2026-10-25, M8 Atmosphäre]

- Ein physikalisches Atmosphärenmodell für jeden Standort (Rampe, Aufstieg, Orbit, Mond): der Himmel, der blaue
  Erdrand und der Dunst hängen nur davon ab, wo die Kamera ist, wohin sie schaut und wo die Sonne steht - kein
  Umschalten, kein Überblenden zwischen Techniken. Die Farben entstehen aus der Streuung (kein Farbabgleich, keine
  gewählten Farbverläufe).
- Am Boden: blauer Taghimmel, zum Horizont heller und weißlicher, weißer Schein um die Sonne; ferne Landschaft, Meer,
  Turm und Raketen verblassen mit der Entfernung im Dunst (eine aufsteigende Rakete wird aus der Rampensicht immer
  blasser). Bei Sonnenuntergang wird die Sonnenscheibe rot und schwächer, das Licht auf Fahrzeugen rötlich.
- Aufstieg: der Himmel wird mit der Höhe dunkler, ab etwa 50 km fast schwarz; aus dem Orbit eine dünne blaue
  Atmosphäre am Erdrand, auch vom Mond aus.
- Mond, Sonne und Sterne werden durch die Atmosphäre gesehen: der Mond am Taghimmel ist blass, Sterne verschwinden am
  Taghimmel und tauchen in der Dämmerung nach Helligkeit nacheinander auf (Sichtbarkeitsschwelle des Auges über der
  Himmelshelligkeit); über der Atmosphäre ist der ganze Sternhimmel da.
- Augenanpassung: die Belichtung folgt dem Licht am Betrachter wie ein Auge (heller werden in etwa einer halben
  Sekunde, dunkler werden über etwa zwei Minuten; Zeitraffer läuft entsprechend schneller, ein Sprung in Zeit oder Ort
  ist sofort angepasst). Fern der Erde zählt das Sonnenlicht am Betrachter (auch im Erdschatten), auf oder nahe dem
  Mond zusätzlich der Mondboden in Sicht: am Mondtag Tagesbelichtung, in der Mondnacht passt sich das Auge an Erde und
  erdbeschienenen Boden an (die Erde erscheint dann überstrahlt).
- Nachtsehen [Entschieden 2026-10-25, Nutzer: "wie das Auge"]: bei Tag unverändert; in der Dämmerung passt sich das
  Auge an den hellsten Himmel an (das Glühen über dem Horizont zur Sonne), der Zenit wird tiefblau, der Horizont
  bleibt farbig; tiefer in der Nacht übernehmen die Stäbchen: das Bild wird dunkel, entsättigt und bläulich
  (Purkinje-Effekt). Vollmond: Landschaft grau und gedämpft erkennbar, der Himmel dunkelblau-grau, nur hellere Sterne;
  der Mond beleuchtet Boden, Meer (Glitzerpfad), Himmel und Fahrzeuge. Mondlose Nacht: dunkel, Sterne bis etwa
  6. Größe. Aschgraues Mondlicht (Erdschein) auf der Mondsichel ist in der Dämmerung schwach sichtbar. Stadtlichter
  leuchten mit ihrer physikalischen Leuchtdichte und gehen wie alles andere durch das Augenmodell; Triebwerksflammen,
  Sterne und Anzeigen behalten ihre Farbe (bewusst nicht entsättigt). In der totalen Sonnenfinsternis passt sich das
  Auge wie in der Dämmerung an den hellsten Himmel an (den Ring am Horizont). Gleich in VR (beide Augen).
  Helligkeit und Farbigkeit [Entschieden 2026-10-25, Nutzer]: auch angepasst wirkt eine dunklere Szene dunkler und
  weniger farbig (Stevens- und Hunt-Effekt, CIECAM02): die Abenddämmerung ist auf dem Bildschirm dunkler und blasser
  als der Mittag, immer dunkler, je tiefer die Sonne steht; der Tag (Sonne höher als ~15 Grad) bleibt unverändert.
- Sonnenfinsternis: im Kernschatten wird der Himmel dunkel, ringsum am Horizont leuchtet die Dämmerung (360 Grad),
  nur helle Sterne und Planeten erscheinen; der Mondschatten ist aus dem Orbit als dunkler Fleck mit weichem Rand zu
  sehen.
- Wolken und Wetter: siehe die Wetterzeile und den Schalter "Wetter" im Overlay-Abschnitt. Noch nicht: Regen, Nebel, Gewitter, Dunst, Wind (kommt nach dem Sehen), Luftleuchten.

### Weitere Außenansichten

[Vorschlag] Umschaltbar per Taste / Bildschirm:
- Verfolgerkamera um das Fahrzeug (vorhanden, siehe oben), mit lokaler Vertikalen als "oben".
- Kameras an Bord: Booster-Triebwerke, Ship-Triebwerke, Ship nach vorn, Hot-Staging-Ring.
- Bodenbeobachter an Startplatz/Landeplatz [Entschieden, umgesetzt 2026-10-25]: Die Fokusliste der Zeitleiste bietet
  je Platz "<Platz> (Aussicht)": der Beobachter steht auf einer Aussichtsplattform beim nächsten Kontrollzentrum
  (`[site.viewpoint]` in `config/launch_sites.toml`), nicht in der Rampe. Augenhöhe = Gelände dort + Dach-/Deckhöhe
  + 1,7 m; der Blick zeigt anfangs zur Rampe, leicht angehoben (Rampe und Turm im Bild). Starbase OLP-1: Dach des
  (früheren) Startkontrollzentrums auf dem Produktionsgelände, ~3,6 km; KSC LC-39A: Dach des Launch Control Center
  neben dem VAB, ~5,1 km; CCSFS SLC-40 und Landezone LZ-40: Aussichtsplattform LC-39 Gantry am Banana Creek (KSC),
  ~5,5 km (je mit eigenem Ziel). Plätze ohne Aussichtspunkt bleiben "(Boden)" an der Rampe.
- Freie Kamera (6DOF-Navigation wie in der Missionsplanung).
- [Entschieden, Darstellung später] Rettung der Crew (Schleudersitze, Spec 2026-10-22): Bei jedem Abbruch einer
  bemannten Rückkehr steigt die Crew aus; die Ansicht folgt dem Piloten aus seiner Sicht (Ausstieg, Bremsschirm,
  Hauptschirm, Aufsetzen), daneben das leere Ship bis zum Aufschlag. Datenquelle: `FlightSim.crew_escape_state()`
  (je Crewmitglied Lage, Phase, g-Last, Aufsetzpunkt) und die Körper der Sitze und Personen.

## 6. Karte und Auswertung

- ISS-Missionen (i6): Die Spielschnittstelle liefert schon alles für die Ansichten. `FlightSim.iss_state` enthält die
  ISS mit ihren Ports, `rendezvous_plan` die Manöver des Phasenflugs, `docking_state` Annäherung, Andocken und Abflug
  im Portsystem, `station_geometry` die Grundkörper zum Zeichnen und `iss_mission` den Missionsabschnitt. Befehle:
  `proximity_hold` / `_retreat` / `_abort` / `_resume`, `undock`. Die Ansichten selbst legt der Nutzer fest.
- Betankung (r5): Die Missionen `depot_launch`, `tanker_flight`, `hls_refuel` laufen über dieselbe Abschnittsfolge;
  neue Abschnitte „Perigäum anheben“, „Treibstofftransfer“, „Warten auf Fortsetzung“ (bis zum Start der Mondmission).
  Spielschnittstelle (r6): `start_mission` startet die neuen Arten; `FlightSim.transfer_state` zeigt den laufenden
  Treibstofftransfer (Phase, Fluss, gelieferte Mengen, Absetzgas), `depot_state` Füllstand, Tankvolumen und Boil-off
  des Depots. Der Befehl `refuelling_campaign(params)` rechnet die Tankerkampagne mit den Stellgrößen (Depothöhe,
  Startabstand, Phasenflug-Grenze, Trockenmasse und Reserve des Tankers, Anfangsfüllung, Isolierung, Wartezeit des HLS);
  `result(id)["data"]` enthält Anzahl der Tankerflüge, Dauer, Bedarf, die Flüge mit Start/Ankunft/Lieferung, den
  Füllstandsverlauf und die Verluste. Eine Ansicht dafür (z. B. in der Missionsplanung) legt der Nutzer fest.

- Karte: Erde-Mond-System mit Bahnen, Mondbahn, Manöverknoten; Bearbeitung wie in der Missionsplanung.
- Auswertung nach dem Flug: Telemetrie-Diagramme (wie `tools/plot_telemetry.py`), Soll/Ist-Vergleich,
  Ereignisliste, Export als CSV.

## 7. Einstellungsassistent für Eingaben [Entschieden: Assistent; Vorschlag: Ablauf]

1. Geräte erkennen und benennen (z. B. "T.16000M", "TWCS", VR-Controller), Zuordnung per Gerätename.
2. Je Profil (Flug, Planung/Navigation, VR) die Funktionen nacheinander abfragen:
   "Bewege jetzt die Achse für Nicken nach vorn" → Achse und Richtung werden erkannt (wie
   `game/tests/joy_probe.gd`), danach Totzone und Kurve einstellen.
3. Tasten ebenso: "Drücke die Taste für Stufentrennung".
4. Zusammenfassung zeigen, testen, speichern im Benutzerprofil; jederzeit erneut aufrufbar.
5. Vorlagen für bekannte Geräte (T.16000M + TWCS ist hinterlegt).

## 8. Offene Fragen

- Bahnpunkt-Bearbeitung: Ist "Kugelebene XY relativ zur Kamera" wie in Abschnitt 3 beschrieben gemeint?
- Belegung der VR-Freiheitsgrade 5 und 6 (Vorschlag in Abschnitt 3).
- Rückwärtsbewegung mit dem TWCS: Wippe oder Hebel mit künstlicher Mitte?
- Cockpit Starship-Stil: Wie viele Fenster, wo sitzen die drei Plätze relativ zu den Fenstern?
- UI-Sprache: heute deutsch; die Anzeigenamen der Fahrzeuge und Stufen bleiben vorerst englisch (`display_name` in
  der Config, keine Markennamen). Die Sprachen legt der Nutzer später fest.
- Fahrzeugmodell beim Wiedereintritt: Hitzeschild und Klappen sichtbar, ggf. Anstell-/Rollwinkel-Anzeigen [Entschieden, Folgeplan].
- Sollen die zwei Crewmitglieder auch Aufgaben übernehmen (z. B. Checklisten), oder nur Ansagen?
