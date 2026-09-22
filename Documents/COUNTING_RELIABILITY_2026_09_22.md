# Oppdatering av dørtelleren – 22. september 2026

Det er rettet konkrete feil i måleinnhenting og gjenoppretting. Endringene er
testet lokalt og kompilert for ESP32. De er **ikke installert på enhetene**, og
forbedret treffrate må bekreftes med nye fysiske passeringer.

## Endringer

- En optisk ugyldig avstandsmåling utløser ikke lenger full omstart av sensoren.
  Ugyldige målinger er fortsatt ugyldige for telling. Vedvarende tap av resultater,
  gjentatte I²C-feil og maskinvarefeil utløser fortsatt gjenoppretting.
- En sensor som ikke starter, nullstiller ikke lenger de andre sensorenes spor
  med mindre den delte I²C-bussen faktisk holdes lav.
- Hvert måleresultat knyttes til sitt målefelt før neste felt startes.
  Automatisk gjentakelse i sensoren utsettes med en kontrollert sikkerhetsmargin.
  Ved forsinket avlesning tømmes en mulig pågående måling før feltet endres.
  Ventingen er asynkron; de andre sensorene kan fortsette å arbeide.
- Det er lagt inn kontroll av registerskrivingen for denne tidsmarginen.
  Den medfølgende driveren rapporterer ellers ikke alle skrivefeil.
- Oppe/Nede-profilene bruker kortdistansemodus og 20 ms måletid for denne
  sidemonterte installasjonen med 80–90 cm åpning. Generelle profiler beholder
  sine målemoduser.
- En feil i ESPHome-valideringen som avviste kortdistansemodus med 20 ms, er rettet.
- Baseline-tilpasning pauses også mens første aktiveringsmåling venter på bekreftelse.
- Loggen viser hvor mange komplette spor som har tvetydig timing. Valget
  `require_timing_evidence: true` kan avvise dem, men er **av som standard**:
  streng kontroll forkaster også riktige passeringer i de gamle opptakene.
  Motstridende komplette retningsspor har fortsatt veto; det er ikke innført
  flertallsgjetting.
- Sporingsverktøyet skiller datarader fra hendelseslogg, unngår tett polling av
  tomme sider og varsler om tap av historikk.

ST beskriver autonome målinger og når ROI-parametere tas i bruk i
[UM2555](https://www.st.com/resource/en/user_manual/um2555-vl53l1x-ultra-lite-driver-multiple-zone-implementation-stmicroelectronics.pdf).
Kortdistansemodus og måletid er beskrevet i
[VL53L1X-databladet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf).
At dette forklarer alle feil i den fysiske installasjonen, er foreløpig en hypotese.

## Kontrollert

- `bash tests/run.sh`: kjernekontroller, 1000 passeringer, 729 sekvenskombinasjoner,
  nærpasseringer, tildekking, gjenoppretting, tidsomslag, kalibreringsmigrering og
  parserkontroller. C++-testene bruker AddressSanitizer og UndefinedBehaviorSanitizer.
- `tests/config_test.py` med ESPHome 2026.8.1: 10 konfigurasjonskontroller.
- Begge distribuerte YAML-filene: godkjent konfigurasjon og C++-generering.
- Full ESP32-bygging med de lokale komponentene og kortdistanse/20 ms: bestått.
  Den generelle valideringsprofilen bruker testhemmeligheter; dens binærfiler skal
  ikke installeres som enhetsspesifikk firmware.
- Åttepasseringsopptakene er lagret som testdata. De mangler noen målinger og
  eksakte integrasjonstider. De kan ikke dokumentere ny fysisk treffrate.
  Se `tests/fixtures/doorway_20260922.md` for før/etter-begrensninger.

## Installasjon og neste test

Kjør `python3 tools/package_doorway_update.py` for å lage
`.esphome/roode-doorway-update.zip`. Arkivet inneholder aktuelle komponentfiler,
to YAML-filer som bruker disse lokalt, instruksjoner og SHA-256-kontrollsummer.
Det inneholder ingen passord. De vanlige YAML-filene i repoet peker fortsatt på
GitHub/main og får ikke upubliserte lokale kodeendringer automatisk.

Kopier arkivets YAML-filer og hele `roode_components` til `/config/esphome/`.
Behold eksisterende `secrets.yaml`. Installer Oppe på **10.0.0.100** og Nede på
**10.0.0.101**. Begynn med én enhet. Gamle kalibreringer ugyldiggjøres én gang;
persontall og lagret tuning beholdes. Hold døren tom til kalibrering og Ready er på plass.

Test åtte vekslende UT/IN-passeringer med fem sekunders mellomrom, og deretter
nærpasseringer, skrå gange, rask gange, stopp og vending. Skriv ned faktisk retning
for hver passering. Sammenlign antall hendelser, bekreftede retninger, feil retning,
usikre hendelser, sensorutfall og målefrekvens. Ikke aktiver streng tidskontroll
før nye opptak viser at den gir en nyttig forbedring.
