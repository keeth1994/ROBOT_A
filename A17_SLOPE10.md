# A17: 10 graders bakke

Start run_a17_slope10.py med prosjektets Python, eller Ctrl+Shift+B i VS Code.
Mellomrom pauser/fortsetter; R starter pa nytt.

Samme CAD, masse (2,566 kg), tyngdekraft, friksjon (0,8), gangparametere og servoantakelser som flatgulvtesten. Planet stiger 10 grader mot +X. Robotens startstilling roteres til bakken, deretter beveger den seg fritt i fysikksimuleringen. Ingen kunstig drivkraft eller kroppsposisjonering under gangen.

30 sekunder total test, hvorav 28 sekunder med gangsekvens:
- Netto bevegelse langs bakken: -0,591 m (nedover).
- Ingen fall etter kriteriet >45 graders relativ helning eller <55 mm normal baseklarering.
- Maksimal helning relativt til bakken: 3,67 grader.
- Ingen gulvkontakt med andre deler enn fotkontaktformene.
- Ca. 25,7 % av ledd-tidssamplene naer stallmoment.
- Ingen MuJoCo-advarsler.

Konklusjon: Flatgulvkontrolleren demonstrerer ikke oppovergang pa 10 grader. Dette beviser heller ikke at mekanikken aldri kan klare bakken; en annen kontroller kan gi et annet resultat. Start skjer pa bakken, sa flat-til-bakke-overgang er ikke testet. Masse, kollisjonsformer, friksjon og servorespons har samme usikkerheter som tidligere A17-test.

Reproduser:
.\.venv\Scripts\python.exe run_a17_slope10.py --test

Resultat: results/a17_slope10.json og .csv.
Video: results/a17_slope10.mp4.
