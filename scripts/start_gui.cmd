@echo off
rem Launch the redPitaLock monitor GUI connected to the Raman RP (rp-f0f9f8).
cd /d "%~dp0"
set RP_HOST=rp-f0f9f8.local
if not "%~1"=="" set RP_HOST=%~1
echo Starting redPitaLock GUI for %RP_HOST% ...
python monitor_rp.py %RP_HOST%
if errorlevel 1 (
  echo.
  echo The GUI exited with an error (see above^).
  pause
)
