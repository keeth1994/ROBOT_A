# A17 i MuJoCo – yaw → pitch → fot

## Start
Åpne mappen C:\PROJECTS\ROBOT_A i Visual Studio Code. Ctrl+Shift+B starter standardoppgaven **Start robot A17 - forward gait**.

Alternativt fra prosjektmappen:
```
.\.venv\Scripts\python.exe run_robot_a17.py --viewer
```
Mellomrom pauser/fortsetter. R starter på nytt. Lukk MuJoCo-vinduet for å stoppe.

Reproduser testen:
```
.\.venv\Scripts\python.exe run_robot_a17.py --test --seconds 60
```
Resultatene skrives til results/a17_walk.json og .csv. Gangparametere ligger i reference/a17_gait.json.

## Hva modellen gjør
151 synlige CAD-legemer fra A17, fordelt på plattform og åtte bevegelige lenker. Åtte motorer, yaw ±90°, pitch 0–90°. Alle fire fotposisjonene er kontrollert mot CAD ved nullstilling. Ingen hjul eller ekstra pitchledd.

Motorene bruker 0,26834 Nm stallmoment og 315,79°/s fri hastighet fra tidligere prosjektantakelser. Tilgjengelig motormoment reduseres lineært med hastigheten i drivretningen. Ingen ekstra bærekrefter, låst kropp, redusert tyngdekraft eller direkte animasjon av basen brukes. Gange beregnes av gravitasjon, kontakt og motorene. Modellen settes kun i startstilling ved oppstart/reset.

Gangsekvensen veksler diagonale bein, med 1,166 s periode. Yaw-amplitude ca. 18,9°, pitch-mål 13,8–44,2°. Målene rampes inn etter to sekunders oppstart. En enkel korreksjon fra ideell IMU-retning justerer venstre/høyre steg. Dette er et håndlaget gangskript med parametersøk, ikke reinforcement learning.

## Målt i 60-sekunders test
- 58 sekunder med aktiv gangsekvens, 3,985 m framover (ca. 0,069 m/s).
- Sideavvik −0,027 m; sluttretning −1,84°.
- Maksimal helning 4,12°; ingen fall og ingen gulvkontakt med andre deler enn føttene.
- Ingen MuJoCo-advarsler. Høyeste leddhastighet 243,5°/s.
- Moment nær stallgrensen i ca. 13 % av ledd-tidssamplene.

Separate 20-sekunders følsomhetstester ga framdrift uten fall: friksjon 0,5 → 1,17 m; friksjon 1,0 → 0,88 m; masse +20 % → 0,80 m; moment −20 % → 0,74 m. Se results/a17_sensitivity.json. Dette er enkeltstående tester på flatt gulv, ikke en robusthetsgaranti.

## Begrensninger
Masseanslag 2,566 kg: 98 uendrede printdeler matcher tidligere slicerdata, nye hule føtter beregnes konservativt som solid PLA i materialvolumet. Servoer er 44 g hver. Elektronikk/batteri er fortsatt en uverifisert 200 g totalavsetning. Inertier er tilnærmet fra delenes avgrensningsbokser; kollisjonsformer er forenklede bokser, kapsler og sylindre. Geometrien vises fra CAD.

Kontaktfriksjon er antatt 0,8. Servomodellen bruker optimistiske øyeblikkelige grenser og tar ikke med varme, spenningsfall, dødgang, fleksibilitet eller girslark. Sensorer er ideelle. Reelt servoutslag, hornfeste, printstyrke og strømforsyning må kontrolleres fysisk. Resultatet viser mulighet i denne modellen, ikke at den ferdige roboten er verifisert.

A16-filene er beholdt.
