# ROBOT_A · MuJoCo

**Robot A7.2 er overført fra Fusion til en bevegelig MuJoCo-modell.**

I VS Code: **Ctrl+Shift+B** starter **Start robot A7.2**.

```powershell
.\.venv\Scripts\python.exe run_robot.py
```

Du får en 3D-visning og et kontrollpanel. Begynn i **Visning – uten fysikk**, velg bein og prøv leddvinklene. Bytt til **Fysikk – begrenset servo** for tyngdekraft, gulvkontakt, brems og servobegrensninger.

Les [brukerveiledningen for A7.2](ROBOT_A7_2.md) for kontroller, modellantakelser og gjenværende arbeid. Modellen bruker foreløpige masser og motorverdier, og har ingen ferdig gå-/skøytepolicy.

## Kontroller modellen

```powershell
.\.venv\Scripts\python.exe run_robot.py --check
```

Resultater lagres i `results/robot_check.json`. Modell: `models/robot_a7_2.xml`. Innstillinger: `reference/robot_parameters.json`. Etter endring av innstillinger bygges modellen med `build_robot.py`.

## Tidligere servotestbenk

```powershell
.\.venv\Scripts\python.exe run_sim.py
```

Den separate testen av en servoarm er bevart. [Tidligere dokumentasjon](SERVO_TESTBENK.md) beskriver denne testbenken; henvisninger der til planlagt robotimport er historikk.

## Miljø

Lokal Python 3.12 og MuJoCo 3.13.0 i `.venv`. Installer med `requirements-lock.txt` ved behov. Ikke kopier `.venv` mellom maskiner.

[MuJoCo Python-dokumentasjon](https://mujoco.readthedocs.io/en/stable/python.html)