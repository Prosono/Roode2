# Analyse av RooDe2

> Historisk analyse før forbedringsrunden. Se `COUNTING_ROBUSTNESS.md` for implementerte rettinger og gjeldende begrensninger.

Gjennomført 20. september 2026 på `main`, utgangspunkt `2ae106e`. Analysen gjelder lokal kildekode og konfigurasjon. Ingen firmware er flashet, og ingen fysiske sensorer er testet. Funn merket kodefunn er basert på lesing av den faktiske implementasjonen; konsekvenser i en konkret installasjon krever reproduksjon.

## Hva prosjektet er

RooDe2 er en ESPHome-basert personteller for ESP32/ESP8266 med VL53L1X-avstandssensorer. Den utleder bevegelsesretning gjennom en døråpning og vedlikeholder antall personer inne. ESPHome står for enhetslivssyklus, nettverk, OTA, Home Assistant-integrasjon og entiteter. Repoet leverer egen C++-logikk, Python-skjemaer for konfigurasjon/kodegenerering, YAML-profiler og nettgrensesnitt innebygd i firmware.

Det finnes to sentrale implementasjoner:

1. **Opprinnelig RooDe:** én fysisk sensor med to ROI-er, styrt av `roode` og `vl53l1x`. Home-profilene med to eller tre sensorer oppretter separate tellere. De er ikke én samlet flertallsteller.
2. **Fire-sensor-løsningen:** `tof_overdoor_counter` eier fire sensorer direkte gjennom `VL53L1X_ULD`. Hver sensor har to ROI-er; retning vurderes per sensor og kombineres med et quorum. Denne bruker ikke `components/vl53l1x` som sitt driverlag.

Profilen `peopleCounter32FourSensorOverdoor.yaml` og de nyere endringene peker mot fire-sensor-løsningen som hovedsporet for videre arbeid. Dette fastslår ikke hvilken firmware som faktisk er installert på enhetene.

## Kart over repoet

| Område | Rolle |
| --- | --- |
| `components/roode` | Opprinnelig sekvensteller, kalibrering, justerbare terskler og ESPHome-entiteter |
| `components/vl53l1x` | Driverintegrasjon for opprinnelig teller, adressering, målinger og recovery |
| `components/tof_overdoor_counter` | Ny samlet teller: I2C, åtte ROI-er, kalibrering, sporing, avstemning, lagring og diagnostikk |
| `components/tof_array_test` | Sensoroppdagelse og målinger for bringup og maskinvarefeilsøking |
| `components/roode_ui` | Nettgrensesnitt for én eller flere uavhengige RooDe-instanser |
| `components/tof_overdoor_ui` | Nettgrensesnitt, innstillinger, tilstand og trace for fire-sensor-telleren |
| `components/room_counter_ui` | Eget, enklere romtellergrensesnitt |
| `components/persisted_number` | Tallentitet med gjenoppretting av lagret verdi |
| `esphome/packages` | Gjenbrukbare basis- og sensorprofiler for Home-variantene |
| `peopleCounter32SixSensor*`, `*Isolation*`, `*MicroProbe*`, `powerBootDiagnostic.yaml` | Forsøks- og diagnostikkprofiler; bør ikke forveksles med hovedprofilen |
| `home_assistant_*.yaml` | Alternative tellere, presentasjon og sammenstilling i Home Assistant |
| `calibration`, `arduino_test`, `standalone_test` | Programmer for fysisk kalibrering og manuell sensortest |
| `ci`, `.github/workflows` | Konfigurasjonsvalidering, firmwarebygg og release-automatisering |
| `STL`, `Documents` | Mekanikk, diagrammer og valideringsbeskrivelse |

`platformio.ini` i roten er uttrykkelig merket som IDE-konfigurasjon. Vanlig bygging skjer gjennom ESPHome-profilene. Noen eldre eksempelprofiler henter eksterne komponenter fra GitHub; Home- og fire-sensor-profilene bruker lokal `components/`.

## Hvordan hovedløsningen fungerer

### Oppstart og maskinvare

Produksjonsprofilen bruker ESP32 med Arduino, SDA 21, SCL 22, I2C 400 kHz og XSHUT 16/17/23/25. De fysiske sensorene navngis U3/U4/U7/U8 i koden og får faste adresser 0x30–0x33 etter plass i listen.

Ved oppstart settes XSHUT tidlig. Oppdagelse utsettes til hovedløkken er i gang. En konfigurert engangs programvarerestart ved power-on/brownout skal håndtere kaldstartproblemer; dette er en eksplisitt omvei rundt oppstartsstabilitet, ikke dokumentasjon på at strømforsyningen er verifisert. Hver sensor vekkes og adresseres separat.

Lagring bruker et versjonert datasett med tellere, tuning og kalibrering per ROI. Koden migrerer eldre formater. Lagrede innstillinger overstyrer flere YAML-startverdier, så en terskelendring i YAML er ikke nødvendigvis den effektive innstillingen etter omstart. Etter gjenopprettet kalibrering må åpningen vurderes som tom i 500 ms før telling tillates.

### Målinger og telling

```text
4 fysiske sensorer × 2 vekslende ROI-er
    → gyldighetskontroll av avstand og status
    → medianfilter og eksponentiell glatting
    → forskjell fra egen gulvbaseline
    → terskler med hysterese og debounce
    → bevegelsesspor per sensor
    → retningsstemmer fra flere sensorer
    → quorum → IN / OUT → oppdatert personantall
    → lagring, ESPHome-entiteter og webdiagnostikk
```

Profilen bruker 33 ms målebudsjett, 37 ms intermeasurement og 5 ms komponentoppdatering. **5 ms er ikke en ny fysisk måling per ROI.** Sensoren veksler mellom feltene, og implementasjonen stopper og starter ranging ved hvert ROI-bytte. Målefrekvens og forsinkelse må derfor skilles fra hvor ofte kontrollkoden kjøres.

Et felt blir aktivt når avstanden faller tilstrekkelig fra sin kalibrerte baseline. Standardprofilen bruker 280 mm aktivering og 140 mm frigivelse, med støypåslag og 25 ms debounce. En sensor følger overganger mellom tomt, ett felt, begge felt og motsatt felt. Den aktive algoritmen kan sende en stemme allerede når bare motsatt felt er aktivt. Normalt kreves tre stemmer. Antall inne avgrenses til 0–50 i profilen, mens kumulative inn-/ut-tellere beholdes separat.

### Home Assistant har to ulike roller

`home_assistant_room_counter_package.yaml` er en separat, eldre sekvensteller basert på to grupper av råavstander og forutsetter monitor-modus i firmware. Den må ikke behandles som samme algoritme som den nye ROI-fusjonen.

`home_assistant_combined_people_counter.yaml` kombinerer derimot to allerede beregnede personantall fra side- og toppteller. Den bruker avrundet gjennomsnitt ved liten forskjell, høyeste verdi ved konflikt og én teller hvis den andre er utilgjengelig. Dette er en policy for uenige estimater; den rekonstruerer ikke individuelle passeringer og retter ikke automatisk feil i kildetellerne. Entitets-ID-ene er installasjonsspesifikke.

## Styrker

- Retning vurderes lokalt på ESP-en, uten at Home Assistant må behandle hver råmåling.
- Faste sensoradresser knyttet til XSHUT-plass gjør fravær og senere recovery enklere å håndtere.
- Den nye telleren har separat baseline og støy per ROI, hysterese, kalibreringskontroll og versjonert persistens.
- Diagnostikken omfatter status, avstander, stemmer, beslutningsforsinkelse, hendelseslogg og et begrenset trace-buffer.
- Webhandlerne registreres gjennom ESPHomes `add_handler`; den lokalt genererte ESPHome-koden legger på autentisering når web-auth er konfigurert. Fravær av lokale `authenticate()`-kall er dermed ikke i seg selv en auth-feil.
- `Documents/TOF_COUNTER_VALIDATION.md` beskriver relevante fysiske scenarioer og skiller uttrykkelig mellom vellykket kompilering og dokumentert tellepresisjon.

## Prioriterte funn

### 1. Tidlig godkjenning gjør sene vendinger problematiske — høy prioritet

**Kodefunn:** `update_channel_path_tracker_()` sender en stemme før det siste feltet er tomt, og `update_detection_state_machine_()` registrerer telling straks quorum er nådd. Se `components/tof_overdoor_counter/tof_overdoor_counter.cpp:2010` og `:2221`.

Et spor som går fra A til begge til B kan dermed telles før personen deretter snur tilbake via begge til A. Den opprinnelige tellingen trekkes ikke tilbake av denne beslutningsflyten. Den videre effekten påvirkes av cooldown og ny sporing, men det finnes ingen garanti for at vendingen gir null nettoendring. Dokumentasjonen beskriver sterkere beskyttelse mot vending enn denne tidlige godkjenningen gir.

**Neste tiltak:** Definer eksplisitt når en passering er endelig, og test full sekvens A → begge → B → begge → A → tomt, inkludert stopp på B-siden.

### 2. Kalibrering og debounce bruker kontrollsykluser som om de var nye målinger — høy prioritet

**Kodefunn:** `process_calibration_()` øker `calibration_samples` hver gang funksjonen kjøres, uten å kontrollere at en ny ROI-måling er mottatt. Se samme fil `:1413`. Med 5 ms oppdatering kan 24 kalibreringsbidrag samles på omtrent 120 ms, selv om langt færre uavhengige målinger foreligger per ROI.

Det kan gi et svakere statistisk grunnlag og mer optimistisk støy-/kvalitetsvurdering enn antallet «samples» tilsier. `update_sensor_states_()` kjører også debounce over en gjenbrukt avstand; én måling kan bli aktiv etter 25 ms uten en ny bekreftende måling. Kommentaren i profilen om at debounce krever gjentatte treff er derfor misvisende. Tomgangstilpasning av baseline og støy gjentas på samme måte per kontrollsyklus.

**Neste tiltak:** La nye målinger, med sekvensnummer eller tidsstempel per ROI, drive statistikk og eksplisitt bekreftelse av treff. Behold separate tidsfrister for stale-data og hendelser.

### 3. Sensor-recovery blokkerer også behandlingen av friske sensorer — høy prioritet

**Kodefunn:** `service_recovery_()` kalles synkront fra `update()`. `recover_channel_()` bruker `delay(15)`, wake-delay og post-address-delay, i tillegg til boot- og konfigurasjonskall. Se samme fil `:271`, `:828` og `:882`.

På vellykket recovery med produksjonsverdiene er de eksplisitte ventene alene minst 285 ms. Andre sensorer kan fortsette å måle internt, men hovedløkken leser dem ikke og veksler ikke ROI mens den venter. Isolert elektrisk restart betyr derfor ikke uavbrutt programvarebehandling av de tre andre kanalene. `cycle_duration_ms_` beregnes før recovery og viser ikke hele denne pausen.

**Neste tiltak:** Gjør recovery til en trinnvis, ikke-blokkerende tilstandsmaskin og mål hele oppdateringstiden.

### 4. Kravet om friske sensorer kan svekkes av gamle målinger — høy prioritet

**Kodefunn:** `ready_for_counting_()` bruker `reporting_sensor_count_()`, som ikke sjekker `stale` eller kommunikasjonsfeil. Quorum beregnes separat fra antallet friske sensorer og kan senkes til to. Se samme fil `:2215`, `:2688` og `:2728`.

Dermed kan tre tidligere rapporterende og kalibrerte sensorer holde «ready» sann samtidig som færre er friske, og to gjenværende sensorer kan være nok til en beslutning. Dette avviker fra et strengt krav om tre friske sensorer. Avstemningsvinduet fjerner dessuten stemmer etter alder, uten å kontrollere aktuell sensorhelse.

**Neste tiltak:** Definer én felles regel for hvilke sensorer som er gyldige for readiness og quorum, og test sensorbortfall midt i en passering.

### 5. Opprinnelig kalibrering lekker minne og mangler vern mot nullreferanse — høy prioritet for Home-profilene

**Kodefunn:** `components/roode/zone.cpp:46` allokerer `new int[number_attempts]` uten tilsvarende frigjøring. Vanlig full kalibrering kaller funksjonen fire ganger; med 20 forsøk og 4-byte `int` lekker dette omtrent 320 byte per full kalibrering per sensor, før allokatoroverhead.

Funksjonen tar heller ikke hensyn til returstatus fra målingen før avstanden brukes. Hvis gulvreferansen blir null, brukes den likevel som divisor i logguttrykk på `:63` og `:64`. Argumentet til `value_or(...)` evalueres også når en prosentverdi finnes.

**Neste tiltak:** Bruk automatisk minnehåndtering, aksepter bare gyldige kalibreringsmålinger og avbryt kontrollert ved null/ugyldig baseline.

### 6. Gammel beslutningskode og innstillinger lever videre — middels prioritet

`finalize_event_()` er definert, men har ingen kall i repoet. De nye passeringene går gjennom avstemningen i `update_detection_state_machine_()`. `UNSURE_IN/OUT` genereres i den gamle funksjonen; den aktive flyten registrerer bekreftede eller avviste hendelser. De synlige unsure-tellerne kan dermed inneholde historiske verdier uten at den nye flyten øker dem.

`direction_window_ms_` leses i eldre retningshjelpekode, men bestemmer ikke vinduet i den aktive avstemningen. Der brukes `detection_timeout_ms_`. En synlig justerbar parameter kan derfor gi forventning om effekt uten å påvirke tellingen.

### 7. Trace mangler informasjonen som trengs for å gjenspille den nye algoritmen — middels prioritet

`HistorySample` og `record_history_snapshot_()` lagrer fire aggregerte sensorverdier, ikke åtte separate ROI-historikker. `get_trace_log_text()` beregner historisk drop med kanalens nåværende baseline. Se samme fil `:2487` og `:3385`.

Trace er nyttig for oversikt, men kan ikke alene rekonstruere de faktiske ROI-overgangene og tersklene bak en beslutning. Ved baselineendring kan eksportert historisk drop også avvike fra datidens verdi.

**Neste tiltak:** Lagre tidsstemplet avstand, baseline, gyldighet og aktiv-status per ROI, samt stemmer og endelig beslutning.

### 8. CI og prosjektstruktur gir begrenset regresjonsvern — middels prioritet

- Ingen automatiske enhetstester eller sekvens-/replaytester av telleren ble funnet. Mappene med «test» er programmer for fysisk utprøving.
- CI kjører ved push til `components/**` og `ci/**`, men ikke ved endringer bare i hovedprofilene, pakkene eller selve workflowen. Den har ingen `pull_request`-trigger.
- Fire-sensor-bygget i CI inkluderer en minimal tellerkonfigurasjon; det bygger ikke hele produksjonsprofilens webgrensesnitt og entiteter.
- ESPHome installeres med `pip install -U`, og flere Actions bruker flytende referanser. Resultater er derfor ikke fullt versjonslåst.
- Release-workflows peker delvis på `master` og gammel upstream, mens lokal gren er `main` og README peker på Prosono/Roode2.
- Bringup-varianter, eldre algoritmer og aktive profiler ligger side om side uten en tydelig statusoversikt.

## Anbefalt videre arbeid

1. Avklar ønsket tellekontrakt: endelig passering, vending, tett følge og minste antall friske sensorer.
2. Trekk tids- og hendelseslogikken ut i en del som kan kjøres uten ESP32. Lag deterministiske tester for de konkrete feilforløpene ovenfor før algoritmeendringer.
3. Rett målingsbasert kalibrering, helsekrav og blokkerende recovery. Rett minnelekkasjen separat i den eldre implementasjonen.
4. Utvid trace til å kunne gjenspille alle åtte ROI-er. Samle feltdata med en uavhengig fasit etter protokollen i `TOF_COUNTER_VALIDATION.md`.
5. Fjern eller merk utdaterte innstillinger og profiler. Dokumenter effektive lagrede innstillinger og hvilken profil som er aktiv på hver installasjon.
6. Utvid CI til PR-er og alle aktive profiler, og inkluder bygg av webgrensesnittet. Lås en kjent fungerende verktøykjede.

Jeg ville beholdt den lokale sensorvise retningsvurderingen og quorum som utgangspunkt. Den største forbedringen nå er å gjøre tidsgrunnlaget, hendelsesreglene og verifikasjonen presise, før mer funksjonalitet bygges inn i den allerede store tellerklassen.

## Verifikasjon

- Lokal ESPHome: `2026.3.0`.
- `esphome config` besto for `ci/esp32.yaml`, `ci/esp32_manual.yaml`, `ci/esp8266.yaml`, `ci/esp8266_manual.yaml`, `ci/tof_overdoor.yaml` og Home-, HomeDual-, HomeTriple- og FourSensorOverdoor-profilene: ni konfigurasjoner.
- `esphome compile ci/tof_overdoor.yaml` besto. Dette verifiserer den minimale fire-sensor-firmwaren; full produksjonsprofil med webgrensesnitt ble konfigurasjonsvalidert, men ikke kompilert i denne gjennomgangen.
- Ingen fysiske passeringer, strømbrudd, nettverksbrudd eller sensortap ble testet. Ingen påstand om prosentvis tellepresisjon kan utledes fra denne analysen.
- Endringen fra analysen er denne rapporten; produktkode og YAML-profiler er ikke endret.
