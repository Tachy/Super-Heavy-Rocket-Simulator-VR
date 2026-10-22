@echo off
rem VR version of the simulator: headset on, its OpenXR runtime running (desktop: Start-Desktop.cmd).
rem Without a headset the game shows a message and quits.
rem Uses the double-precision Godot build from external\godot (see CLAUDE.md); override with start.cfg:
rem   GODOT=C:\path\to\godot.windows.editor.double.x86_64.exe
setlocal
set "GODOT=%~dp0external\godot\bin\godot.windows.editor.double.x86_64.exe"
if exist "%~dp0start.cfg" for /f "usebackq eol=# tokens=1,* delims==" %%a in ("%~dp0start.cfg") do if /i "%%a"=="GODOT" set "GODOT=%%b"
if not exist "%GODOT%" (
  echo Godot double-precision build not found: "%GODOT%"
  echo Build it as described in CLAUDE.md or set GODOT in start.cfg.
  set /p "_=Press Enter to close ..."
  exit /b 1
)
rem Raw HOTAS axes (twist, throttle lever) need these SDL mappings before the engine starts.
set "SDL_GAMECONTROLLERCONFIG_FILE=%~dp0config\sdl_gamecontrollerdb.txt"
echo Preparing the project ...
"%GODOT%" --headless --xr-mode off --path "%~dp0game" --import >nul 2>&1
start "" "%GODOT%" --xr-mode on --path "%~dp0game" -- --vr
exit /b 0
