@echo off
rem Desktop version of the simulator (VR: Start-VR.cmd). OpenXR stays off.
rem Raw HOTAS axes (twist, throttle lever) need these SDL mappings before the engine starts.
set "SDL_GAMECONTROLLERCONFIG_FILE=%~dp0config\sdl_gamecontrollerdb.txt"
start "" "%~dp0SuperHeavyRocketSimulatorVR.exe" --xr-mode off -- --no-xr
