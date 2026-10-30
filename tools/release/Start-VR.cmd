@echo off
rem VR version of the simulator: headset on, its OpenXR runtime running (desktop: Start-Desktop.cmd).
rem Without a headset the game shows a message and quits.
rem Raw HOTAS axes (twist, throttle lever) need these SDL mappings before the engine starts.
set "SDL_GAMECONTROLLERCONFIG_FILE=%~dp0config\sdl_gamecontrollerdb.txt"
start "" "%~dp0Super-Heavy-Rocket-Simulator-VR.exe" --xr-mode on -- --vr
