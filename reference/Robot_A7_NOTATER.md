# A7 – direkte servodrift

Alle tolv pitch/yaw-ledd drives nå direkte fra den valgte Parallax 900-00005-servoen, med 1:1 forhold mellom servoens utgang og leddet. De eksterne 30T/60T-tannhjulene er fjernet fra disse modulene. Servoen har fortsatt sitt interne gir.

Modulrekkefølgen er fortsatt plattform → pitch → yaw → pitch → opprinnelig hjulmodul. De fire beina peker 45° ut fra plattformens hjørner. Hjulmodulens styring og brems, inkludert dens tannhjul, er beholdt.

## Mekanisk løsning

- Servoen står på leddaksen. Et nav på den avtakbare gaffelplaten kobles til servoens originale horn.
- Gaffelen støttes av et fremre lager med nominelle mål 25 × 37 × 7 mm og et bakre lager 8 × 16 × 5 mm. Fremre lagerlokk og drivkinn kan tas av for montering.
- Plane modulflenser med fire Ø4,4 mm hull i et 40 × 40 mm mønster er beholdt. Nominell flenstykkelse er 8 mm. Plattformen har samme hullmønster.
- Modulens ytre mål i vist nullstilling er omtrent 88 × 90 × 80 mm. A6 var omtrent 122 × 90 × 77 mm i tilsvarende stilling.

Det gule servo-hornet er en geometrisk plassholder. Diameter, spline, hullplassering og aksial plassering må bekreftes mot hornet som følger med den faktiske servoen. De modellerte to små hullene med 16 mm avstand er foreløpige, ikke dokumenterte produsentmål. Navet må tilpasses disse målene før printing. Endelige skrue-/innsatsløsninger, lagerpasninger, aksial sikring og konstruksjonsradier må også ferdigstilles.

Direkte drift fjerner reduksjonen av hastighet fra det eksterne giret. Det dokumenterer ikke at servoen er rask eller sterk nok til skøyting under robotens belastning. Ingen fysisk belastningsprøve eller momentberegning er utført.

## Fusion og kontroll

U90_Direkte_A7.f3d inneholder et faktisk revolute-ledd med 0–90° grense. En 5° bevegelse ble kjørt og kontrollert mot komponentens transformasjon, deretter ble leddet satt tilbake til 0°.

Gaffel, drivnav og hornplassholder er geometrisk kontrollert mot den faste kassetten fra 0–90° i trinn på 5°. Ingen volumoverlapp ble funnet. Den statiske kontrollen av roboten fant heller ingen overlapp mellom de kontrollerte modulene, beina og plattformen, og ingen ugyldige solide legemer.

Dette er ikke en kontinuerlig bevegelseskontroll eller en kontroll av alle vinkler i hele beinkjeden. Skruehoder, kabler, materialstyrke og belastet funksjon er ikke verifisert. Hele roboten og enkeltbeinet er fortsatt statiske sammenstillinger, med faste og roterende deler i separate komponenter. Bare den separate standardkassetten har et aktivt Fusion-ledd.

## For neste steg mot simulering

Robot_A7.step, Bein_R7_Direkte.step og U90_Direkte_A7.step er eksportert. Robot_A7_simulering.json og Robot_A7_leddakser.csv inneholder posisjon og retning for alle tolv nye leddakser, grenser relativt til eksportert stilling og 1:1-utveksling. Tolv skjulte konstruksjonsakser er også lagt inn i robotens Fusion-modell.

I leddlisten er q = 0 den eksporterte stillingen. Pitchleddene har relativt område 0 til +90°, mens yaw har −90 til 0° fordi yaw-gaffelen er modellert i den andre endestillingen. Servopulser og fysisk nullstilling må kalibreres separat.

Dette er importgrunnlag, ikke en kjørbar fysikkmodell. Før simulering må vi definere hjulenes rulle- og styreledd, bremsemodell, stive forbindelser, kollisjonsgeometri, masser/treghetsmomenter, friksjon og målte servo-begrensninger. Ukjente kraft- og hastighetsgrenser er markert som null i JSON, slik at de ikke forveksles med målte verdier. Ingen simulering eller RL-trening er kjørt.
