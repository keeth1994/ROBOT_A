# Robot A7.2 i MuJoCo

Modellen bruker den eksporterte A7.2-geometrien fra Fusion, med to 608-lagre og fast Ø8 mm aksling per hjul. Yaw-hus og servofeste er endret for 180° mekanisk klaring. De tidligere A7.1-filene er bevart.

## Åpne

I VS Code: **Ctrl+Shift+B → Start robot A7.2** (standardoppgaven).

```powershell
.\.venv\Scripts\python.exe run_robot.py
```

MuJoCo-vinduet og et eget kontrollpanel åpnes. Lukk ett av dem for å avslutte begge.

1. Start i **Visning – uten fysikk**. Velg FL, RL, FR eller RR og prøv sliderne. FL/FR er foran, RL/RR er bak. Alle vinkler er relative til CAD-stillingen, ikke servoens PWM-nullpunkt.
2. Pitch går fra 0 til 90°, yaw fra −90 til +90°, og hjulstyring fra −90 til +90°. De fire hjulene har fri rotasjon.
3. **Svingtest** beveger pitch ved plattformen på valgt bein, med 10° amplitude. Testen gjelder ett ledd, ikke en gå- eller skøytebevegelse. Skru den av før manuell styring av samme ledd.
4. Bytt til **Fysikk – begrenset servo** for tyngdekraft, gulvkontakt, servomoment og brems. Svake servoer kan føre til at roboten synker eller velter; stillingen holdes ikke kunstig fast.
5. Velg bremseprosent per bein. **Lagre CSV** lagrer målvinkler, faktiske vinkler, moment, hjulenes normalkraft mot gulvet og posisjonen til plattformen. En sidefil oppgir innstillingene som ble brukt ved lagring.
6. **Nullstill** setter roboten tilbake til CAD-stillingen. Visning setter plattformen tilbake til utgangsposisjonen og bruker direkte leddplassering uten fysikk. Vinkelsporing i denne modusen sier derfor ingenting om servoytelsen.

Venstre musetast roterer MuJoCo-kameraet, høyre flytter det og rullehjulet zoomer.

## Hva er overført?

- CAD-deler og farger, gruppert etter hvilke deler som skal bevege seg sammen. Et 0,25 mm punktrutenett reduserer svært tette visningsflater; dette er visningsgeometri, ikke produksjonsfiler.
- Plattform → pitch → yaw → pitch → hjulstyring → hjulrotasjon, på fire bein.
- 16 styrte ledd og fire passive hjul. Fire ekstra interne ledd kobler de like store styretannhjulene 1:1 med motsatt rotasjon.
- Idealisert gyro, akselerometer, orientering, leddvinkler og leddhastigheter. Dette er simulatordata; modellen påstår ikke at den virkelige servoen har posisjonstilbakemelding.
- Bremsevirkning som regulerbar friksjon i hjulleddet. Kam, trykkplate og bremsekloss er vist i CAD-hvilestilling, og bremsens interne bevegelse er ikke simulert.

## Foreløpige antakelser

`reference/robot_parameters.json` inneholder masse-, friksjons- og motorantakelsene. Endre dem og kjør `build_robot.py` for å bygge modellen på nytt. Panelet kan endre stoppmoment og tomgangshastighet under kjøring.

- **0,27 Nm og 230°/s** er videreførte antakelser fra testbenken, ikke verifiserte Parallax-data. Regulatoren har en lineær moment–hastighetsbegrensning ved motordrift og begrenset bremsemoment. Ingen målt PWM-forsinkelse, slark, temperatur eller spenningsfall er inkludert.
- **120 g per roterende hjulenhet** er utgangspunktet fra samtalen. Øvrige masser anslås fra CAD-volum, effektiv printtetthet og egne anslag for motorer/elektronikk. Dette erstatter ikke veiing. Lager og aksling er egne deler.
- Treghet anslås fra delmasser, CAD-massesentre og avgrensende bokser. Den er ikke en beregnet modell av PLA-skall og faktisk fyllmønster.
- CAD-formene er visuelle. Kollisjoner bruker bokser, kapsler og hjulsylindre. Slå på **Vis forenklede kontaktformer** for å se dem. Dette er egnet til tidlige bevegelsesforsøk, men ikke detaljert kontroll av sammenstøt mellom moduler eller girtenner.
- Returfjærens gamle, skjulte plassholder er utelatt. Faktisk bremsereturfjær, lagerslark, fleksibilitet og friksjonsbelegg gjenstår.
- Sensorhus er med visuelt. IR-sensorenes støy, rekkevidde og synsfelt samt kamerabilder/FSR-respons er ikke implementert.
- Modellen har ingen ferdig RL-policy, gangart eller skøytekontroller. Den kan brukes som utgangspunkt for disse etter at masser, servodata og kontaktmodellen er kalibrert.

## Filer og kontroll

- `models/robot_a7_2.xml`: kjørbar MJCF-modell.
- `models/robot_meshes_a7_2/`: CAD-overflater i meter, plassert lokalt på hvert bevegelig legeme.
- `reference/a7_1_cad_meshes.json.gz`: bevart grunneksport fra Fusion.
- `reference/a7_2_yaw_mesh_patch.json`: åtte nye yaw-solider som erstatter de tilsvarende delene i grunneksporten via `cad_source.py`.
- `reference/robot_a7_2_manifest.json`: full deltilordning, leddakser, masser og forbehold.
- `results/robot_check.json`: resultat fra kontrollkjøringen.
- `run_sim.py`: den tidligere servotestbenken, fortsatt tilgjengelig.

```powershell
.\.venv\Scripts\python.exe build_robot.py
.\.venv\Scripts\python.exe run_robot.py --check
```

Kontrollen sammenligner CAD- og MuJoCo-geometri, verifiserer akser med en separat rotasjonsberegning, prøver leddgrensene, kjører fysikk med kontakt og kontrollerer at hjulbremsen reduserer hjulhastigheten. Dette er modellkontroller, ikke bevis på at den virkelige roboten kan gå eller skøyte.

Format og simulator-API: [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html), [Python bindings](https://mujoco.readthedocs.io/en/stable/python.html).

## Yaw-endringen i A7.2

Alle fire yaw-ledd har nå −90° til +90° relativt til robotens CAD-stilling. Den separate yaw-modulen har et bevegelig Fusion-ledd fra 0° til 180°; robotens nullstilling tilsvarer modulens 90°-stilling. Pitch beholder 0–90°.

Huset og servofestet har fått avrunding på begge sider av sveipet. Direkte drift, servoens plassering og de plane P40-festene er beholdt. Hjulmodulen med to 608-lagre er uendret fra A7.1.

CAD-kontroll: yaw-modul 0–180° i trinn på 2°, helt bein −90–90° i trinn på 5°, og 67 robotstillinger mot plattform og andre bein. Ingen kollisjoner ble funnet i disse kontrollene. Pitch var i CAD-utgangsstilling; kombinasjoner av alle pitch- og yaw-vinkler er ikke kontrollert. Kollisjon mellom bein er etter avtale et styringsproblem og begrenser ikke det tillatte yaw-området. Ingen aktiv kollisjonsunngåelse er implementert i kontrollpanelet.

MuJoCo-kontroll: alle fire yaw-ledd når −90°, 0° og +90° i visningsmodus. Akser, CAD-plassering, endestillinger, tre sekunder fysikk og hjulbrems bestod modellkontrollene uten numeriske advarsler. Dette dokumenterer ikke servoens faktiske belastede hastighet eller robotens evne til å gå/skøyte.

Nominell lokal klaring er ca. 0,5 mm. PLA-passform, kabelsløyfe, festehull rundt servohornet, faktiske servoendepunkter og styrke må kontrolleres på en fysisk prøve. De nye vinkelgrensene er programvaregrenser; egne mekaniske endestopp er ikke lagt til.
