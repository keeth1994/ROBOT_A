# ROBOT_A · MuJoCo

Miljøet er installert lokalt i `.venv` med Python 3.12 og MuJoCo 3.13.0.

## Start

I VS Code: **Ctrl+Shift+B** starter oppgaven **Start MuJoCo**.

Eller kjør fra terminalen i denne mappen:

```powershell
.\.venv\Scripts\python.exe run_sim.py
```

To vinduer åpnes: MuJoCo-visning og kontrollpanel. Lukk ett av dem for å stoppe begge.

## Hva du ser

Dette er en **servotestbenk**, ikke den konverterte A7-roboten.

- **Blå arm** viser ønsket bevegelse.
- **Oransje arm** beveges av simulert motormoment og påvirkes av tyngdekraft og belastning.
- Prøv 0,3 Hz og deretter 1,5 Hz. Kurvene og vinkelavviket viser om armen følger kommandoen.
- Endre belastning, antatt tomgangshastighet og stoppmoment mens scenen kjører.
- **Lagre CSV** lagrer måleserien i `results/`.

Standardverdiene 230°/s og 0,27 Nm er **foreløpige antakelser**. Testbenken bruker en generisk PD-regulator og en lineær moment–hastighetskurve, ikke en kalibrert modell av Parallax-servoen. Den inkluderer ikke målt forsinkelse, temperatur, strømforsyningsfall eller slark. Stoppmoment er ikke en kontinuerlig belastningsgrense. Resultatene bekrefter derfor ikke at den virkelige roboten klarer å skøyte.

## Kontroll

```powershell
.\.venv\Scripts\python.exe run_sim.py --check
```

Kontrollerer lasting, fem sekunder fysikk per scenario, endelige verdier, momentgrense, lastendring og at det raske testforløpet gir større sporingsfeil enn det langsomme. Resultat: `results/check.json`.

## Videre arbeid

`reference/` inneholder A7-ledddata og STEP-geometri fra Fusion. Neste steg er å konvertere A7 til en MJCF-modell med separate stive deler, hjulstyring, frie hjul og bremser. Deretter settes realistiske masser, treghet, friksjon og målte servoegenskaper før helrobot-testing og RL.

Offisiell dokumentasjon: https://mujoco.readthedocs.io/en/stable/python.html

## Reinstaller miljøet

Bruk en kompatibel Python 3.12-installasjon:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
```

Det nåværende miljøet ble opprettet fra Python-runtime som allerede finnes på denne PC-en. Det er ikke nødvendig å endre systemets standard-Python. `.venv` er lokal og skal ikke kopieres til en annen PC.
