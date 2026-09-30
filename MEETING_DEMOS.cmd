@echo off
cd /d "%~dp0"
echo Group meeting demos - original servo limits, real simulated physics
echo 1. Normal programmed walk on flat ground
echo 2. Learned RL walk on flat ground
echo 3. Learned RL on the 3 degree ramp
echo 4. Open recorded videos
choice /c 1234 /m "Choose demo"
if errorlevel 4 goto videos
if errorlevel 3 goto ramp
if errorlevel 2 goto flat
.venv\Scripts\python.exe robot.py demo normal
goto end
:flat
.venv\Scripts\python.exe robot.py demo rl-flat
goto end
:ramp
.venv\Scripts\python.exe robot.py demo rl-ramp
goto end
:videos
start "" "%~dp0results\meeting"
:end
pause
