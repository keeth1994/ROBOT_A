# Servokalibrering

Start fra prosjektmappen:

```bash
python3 -m pip install -r servo_calibration/requirements.txt
python3 servo_calibration/gui.py
```

ESP32 må kjøre `esp32_calibration/esp32_calibration.ino`. Skissen er uendret;
den trenger ikke lastes opp på nytt for å bruke den nye lagreknappen.
Lukk Serial Monitor før tilkobling. Støtt roboten, velg kanal/ledd og aktiver.
Aktivering kommanderer 1500 µs umiddelbart; dette er ikke et målt nullpunkt.
Juster med små steg og mål vinkelen fysisk før du lagrer.

## Din pitch-nullstilling

Pitch-null er nå 45° opp fra CAD-stillingen. Målingene som allerede er lagret
i `measurements.json`, beholdes:

| Lagreknapp | Stilling | Vinkel i simuleringen |
|---|---|---:|
| −90° | Beinet 45° over horisontalt | −135° |
| −45° | Beinet horisontalt ut | −90° |
| 0° | Halvveis opp fra CAD mot horisontalt | −45° |
| +45° | CAD-stilling, fot ned | 0° |
| +90° | Gammel nedre endestilling, 45° videre fra CAD | +45° |

For å beholde hele det gamle området og kunne løfte over horisontalt, må du nå
måle **to ekstra punkter (−90° og +90°) på hvert av de fire pitch-leddene**.
Dette gir 180° totalt rundt den nye nullstillingen. De eksisterende punktene
skal ikke måles på nytt. Bruk justeringsknappene for å nå stillingen;
«Lagre −90°» og «Lagre +90°» lagrer bare pulsen og flytter ikke
servoen. Stopp før binding eller fysisk stopp. Hvis mekanikken ikke tillater
stillingen, skal den ikke tvinges dit; da må området begrenses.

Yaw bruker fortsatt −45°, 0° og +45° relativt til CAD. Begge ±90°-knappene er
deaktivert for yaw, og yaw-feltene for disse vinklene er blanke. Tabellen viser
kanalene i stigende rekkefølge.

Etter de åtte ekstra målingene: eksporter `servo_table.h` på nytt. Eksporten
krever tre punkter for yaw og fem for pitch, med konsekvent retning og minst
20 µs mellom hvert punkt. Erstatt `servos`-tabellen i
`firmware/include/hardware_config.h` med eksporten. `calibrated` skal fortsatt
være `false` til resten av oppsettet også er kontrollert; se firmware-README.
Det eksisterende eksporterte headeret med tre eller fire pitch-punkter er ikke
komplett for den nye firmwaren. Bruk eksporten for å få riktig C++-feltrekkefølge:
`channel, minus45us, zeroUs, plus45us, plus90us, minus90us`.

Standardmodellen i simuleringen får det utvidede området, −135° til +45° i
CAD-skalaen. Rampemodellen beholder de opprinnelige grensene −45° til +45° og
samme gangkommandoer. Den nye nullstillingen oversettes i firmware og flytter
ikke CAD-geometrien. Utvidelsen legger ikke automatisk til en operatørkommando
for å heve beina.

Programmet endrer ikke de lagrede målingene før du lagrer et punkt. Målinger
lastes automatisk ved oppstart. `--file PATH` velger en annen målefil.
Ikke kjør flere programmer mot samme USB-port eller målefil samtidig.

STOPP ber om PWM av og kobler fra. Ved kommunikasjonsfeil kan fysisk bryter
være nødvendig for å stoppe servoene. Elektriske pulsgrenser i skissen er
ikke mekaniske bevegelsesgrenser.

Sjekk uten maskinvare:

```bash
python3 -m unittest discover -s servo_calibration -p 'test_*.py'
```
