# Ansichten und Bedienung im fertigen Simulator

Stand: 12.10.2026. Lebendes Dokument: Es wird bei jeder Änderung an Ansichten oder Bedienung mitgepflegt.
Markierungen: **[Entschieden]** = mit dem Nutzer abgestimmt, **[Vorschlag]** = Entwurf, noch zu bestätigen.

## 1. Grundsätze

- **[Entschieden]** Alle Ansichten gibt es auf dem Desktop und in VR (OpenXR, PC-VR). VR und Desktop sind
  getrennte Startmodi (Muster aus Stunt Track Racer VR, siehe Plan M7).
- **[Entschieden]** Die Oberfläche ist deutsch, Einheiten SI (Anzeige: km, km/h, m/s, t, MN, kPa).
- **[Entschieden]** Alle Eingabebelegungen (HOTAS, VR-Controller, Tastatur, Maus) sind änderbar und werden
  über einen **Einstellungsassistenten** abgefragt (Abschnitt 7).
- **[Vorschlag]** Jede Ansicht zeigt die Simulationszeit (UTC) und den Zeitraffer; im Flug zusätzlich T+.

## 2. Übersicht

| Ansicht | Zweck | Hauptbedienung | Meilenstein |
|---|---|---|---|
| Missionsplanung | Mission entwerfen: Start, Ziele, Landeplätze, Manöver | 6DOF-Navigation (HOTAS / VR), Maus | M6 |
| Cockpit | Fliegen aus dem Pilotensitz | HOTAS, Berührung der Bildschirme | M7 |
| Außenansichten | Fahrzeug von außen, Kameras an Bord | Maus / Kopf, Umschalten | vorhanden, Ausbau M7/M8 |
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
- Rechts: Außenansicht Ship.
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

[Vorschlag] Umschaltbar per Taste / Bildschirm:
- Verfolgerkamera um das Fahrzeug (heute vorhanden), mit lokaler Vertikalen als "oben".
- Kameras an Bord: Booster-Triebwerke, Ship-Triebwerke, Ship nach vorn, Hot-Staging-Ring.
- Bodenbeobachter an Startplatz/Landeplatz (heute vorhanden).
- Freie Kamera (6DOF-Navigation wie in der Missionsplanung).

## 6. Karte und Auswertung

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
- Sollen die zwei Crewmitglieder auch Aufgaben übernehmen (z. B. Checklisten), oder nur Ansagen?
