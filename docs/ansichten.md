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

Die Außenansicht trägt ein Overlay im Stil der Starship-Webcasts (Spec `docs/superpowers/specs/2026-10-22-external-view-design.md`).
Der alte Text-HUD mit NavBall bleibt hinter **F1** (aus im Normalzustand), bis es das Cockpit gibt.
- **Oben:** UTC der Simulation (der Zeitraffer steht im Zeit-Panel, nicht im Overlay).
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
  inertiale Geschwindigkeit). Höchstens 4 Anzeigen je Seite; sinnlose Werte (Mach/g am Boden) werden ausgeblendet und
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
  (nach F12 neu gesetzt), nicht am Kopf festgeklebt. Die manuelle VR-Sitzung mit dem Headset steht noch aus.
- **Autopilot** [Entschieden 2026-10-22]: In den Außenansichten fliegt immer der Autopilot. Beim Wechsel in die
  Außenansicht übernimmt er das gesteuerte Fahrzeug, falls er aus ist: auf der Rampe der Aufstiegs-Autopilot (wie
  Taste P), im Orbit ohne Plan der Orbit-Zustand des Autopiloten. Handsteuerung (Nicken/Gieren/Rollen, Schub, HOTAS)
  und die Einzelaktionen I (Zünden), L (Klammern lösen), K (Aus), G (Stufentrennung), M (Steuermodus) und
  X (Autopilot aus) sind hier ohne Wirkung; es bleiben die Befehle an den Bordrechner: P (Aufstieg), O (Zirkularisieren),
  R (Rückkehr zum Turm) und die Annäherungsbefehle. Handsteuerung gibt es später nur im Cockpit und nur für das gerade
  gewählte Fahrzeug.
- **Missionsstart:** Das Spiel startet frei aus dem Zeit-Panel: das Fahrzeug steht auf der Rampe, der Countdown läuft
  60 s als T- (`FlightController.LAUNCH_COUNTDOWN_S`), und der Aufstiegs-Autopilot hebt zum T-0 ab. Ohne Abheben bleibt
  die Uhr bei T-0 stehen (wie ein Startabbruch auf der Rampe), statt auf der Rampe T+ zu zählen. Missionen, die auf der
  Rampe auf ihr T-0 warten (`start_mission` mit `wait_on_pad`, Fensterstart bei ISS-Missionen), gibt es im Kern und in
  den Bindings und in `render_check` (`--mission= --wait_on_pad`); eine Missionsauswahl im Spiel folgt.

### Weitere Außenansichten

[Vorschlag] Umschaltbar per Taste / Bildschirm:
- Verfolgerkamera um das Fahrzeug (vorhanden, siehe oben), mit lokaler Vertikalen als "oben".
- Kameras an Bord: Booster-Triebwerke, Ship-Triebwerke, Ship nach vorn, Hot-Staging-Ring.
- Bodenbeobachter an Startplatz/Landeplatz (heute vorhanden).
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
