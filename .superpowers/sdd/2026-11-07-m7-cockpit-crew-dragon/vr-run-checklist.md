# VR-Lauf Crew-Dragon-Cockpit (Checkliste)

## Start
1. Headset an, OpenXR-Runtime laeuft, HOTAS (T.16000M + TWCS) angeschlossen.
2. `Start-VR.cmd --perf-log --hotas-nudge` (perf_log schreibt `data/perf/perf_<datum>_<zeit>.csv`, `--hotas-nudge` schaltet die Verschiebe-Tasten fuer die Props ein).
3. Im Missionsfenster Fahrzeug "Crew Dragon", Ort "Erdorbit", Mission "Crew-Kapsel zur Raumstation" (`crew_dragon_iss`, 4 Mann Besatzung) starten; ein Flug mit Kabine startet im Cockpit.

## Pruefen und zurueckmelden
- **Sitz- und Augenhoehe:** Sitzt der Kopf natuerlich im Helm-Mittelpunkt, passt die Sitzhoehe zur Konsole? Abweichung in cm (hoeher/tiefer, vor/zurueck).
- **HOTAS-Props gegen die echte HOTAS:** Liegen Stick (rechts) und Gashebel (links) dort, wo deine echten Geraete stehen? Mit den Tasten verschieben, bis es passt:
  Alt+1 = Stick, Alt+2 = Gashebel; Alt+Pfeil links/rechts = zur linken/rechten Seite der Crew, Alt+Pfeil hoch/runter = vor/zurueck,
  Alt+Bild hoch/runter = hoch/runter an der Sitzlehne; 1 cm pro Druck (mit Shift 2 mm). Jeder Druck gibt die neuen Werte aus (`[hotas-nudge] stick_grip_m = [...]`
  am Ende von `%APPDATA%\Godot\app_userdata\Super-Heavy-Rocket-Simulator-VR\logs\godot.log`; PowerShell: `Get-Content "$env:APPDATA\Godot\app_userdata\Super-Heavy-Rocket-Simulator-VR\logs\godot.log" -Tail 20`); melde uns die letzte Zeile je Prop oder die Abweichung in cm.
- **Props folgen den Achsen:** Stick bewegen (Nicken, Rollen, Drehen), Gashebel vor/zurueck: Prop und Hand bewegen sich mit, Richtungen stimmen? Stick zur Seite kippt den Prop zur Seite, Drehen des Sticks dreht den Prop; der Gashebel zeigt die echte TWCS-Stellung (ohne TWCS die Tastatur-Schubstellung).
- **Umschalten Cockpit <-> Aussen:** Taste `C` und Stick-Knopf (derzeit Knopf 3), mehrfach hin und her; Zustand sauber, kein Haengen?
- **F12:** zentriert die Blickrichtung wieder auf die Sitz-Vorwaerts-Richtung (Kopf in VR gerade halten, F12, Bild muss mittig stehen).
- **Zeiger und Hand:** Maus-/Zeigerbewegung ueber die Konsole: bewegt sich die Hand weich (kein Ruckeln, keine eckigen Gelenke)?
- **Linker Bildschirm:** die vier Kamera-Tasten und das Antippen am Bildschirm: wechselt das Bild, reagiert die Hand/Taste?
- **Reichweite:** die ganz rechte Taste `r_upper_5` ansteuern: erreicht die Hand sie, beugt sich der Oberkoerper plausibel vor?
- **Schulter:** den Arm weit nach hinten schwingen lassen (Zeiger zur Seite, dann zurueck zum Gashebel/Stick): sieht die Schulter dabei zerknittert oder eingefallen aus?
- **Bildzeiten:** nach ca. 2 Minuten im Cockpit beenden; aus `data/perf/perf_*.csv` (neueste Datei) die Spalten Frame-Intervall p50 / p99 / max und `script_max_ms`
  melden (oder die Datei nennen, wir lesen sie). Ziel: 90 Hz (p50 11,1 ms), p99 unter 22 ms.

## Stick-Knopf-Nummern messen (falls Umschalten per Knopf nicht klappt)
`$env:SDL_GAMECONTROLLERCONFIG_FILE="config\sdl_gamecontrollerdb.txt"`, dann
`external\godot\bin\godot.windows.editor.double.x86_64.console.exe --path game --script res://tests/joy_probe.gd -- --buttons --seconds=30`
(PowerShell im Repo-Ordner, zuerst die `$env:`-Zeile, dann der Befehl; die `.console.exe` schreibt ins Terminal). Die gewuenschten Knoepfe nacheinander druecken und die ausgegebenen 0-basierten
Indizes melden; sie kommen nach `config/input_hotas.toml [buttons]` (`cockpit_toggle`, `camera_next`).
