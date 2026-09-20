# Robust tellelogikk – implementasjon og verifikasjon

Denne endringen retter funnene i `REPO_ANALYSE.md`. Den oppdaterer firmware og tester, men er ikke flashet til fysisk maskinvare.

## Tellekontrakten

En passering er én sammenhengende bevegelsesepisode mellom to bekreftet tomme perioder. Hver fysisk sensor observerer sine to ROI-er. Vi teller først når åpningen er tom igjen, begge felt har ferske målinger, og tomperioden har vart lenge nok.

- A → begge → B → tomt og B → begge → A → tomt gir motsatte retninger. Et raskt A → B → tomt uten målt overlapp kan også godkjennes.
- A → begge → B → begge → A → tomt er en vending og gir ingen telling.
- Oppstart midt i en passering, start med begge felt aktive og gjeninntreden før tomperioden er bekreftet godkjennes ikke som sikre spor.
- Et langt stopp i åpningen beholder sporet. Det er ingen tvungen telling etter en tidsfrist.
- Fire-sensor-profilen krever minst `max(min_event_sensors, min_valid_sensors)` friske, kalibrerte sensorer. Kravet reduseres aldri automatisk ved sensorfeil.
- En komplett motstridende retningsstemme avviser episoden. For få samstemte, komplette spor registreres som `UNSURE_IN/OUT`, uten å endre personantallet.
- Et kort optisk måleavbrudd i et allerede aktivt felt kan holdes i inntil 150 ms, uten nye kanter eller bekreftet tomperiode. En sensor som deretter mister helse, mister sitt spor og kan ikke komme inn igjen i samme episode. Hvis quorum forsvinner, kreves en ny tomperiode før neste forsøk.
- Home-profilene bruker samme beslutningskjerne, med én fysisk sensor per uavhengig teller.

Retningskonvensjonen er beholdt: OUT-felt først gir OUT uten `invert_direction`. Bruk fysisk inn-/ut-test for å sette retningen riktig på installasjonen.

Det er et bevisst kompromiss at to personer uten tilstrekkelig tomperiode ikke automatisk separeres. Koden avviser tvetydige forløp i stedet for å gjette et personantall. Dette kan redusere deteksjonsgraden ved svært tett trafikk, men unngår enkelte falske og doble tellinger. Uavhengig måling av både feilpositive og feilnegative hendelser er nødvendig.

## Målinger, kalibrering og feil

Kalibrering bruker Welford-statistikk over nye, gyldige råmålinger per ROI. Samme avstand brukes aldri flere ganger som et nytt kalibreringssample. Debounce krever minst to nye bekreftende målinger; en enkelt kort avstandsfeil kan ikke bli en deteksjon bare fordi hovedløkken gjentas. Baseline og støy tilpasses bare på nye målinger mens åpningen er inaktiv.

Begge ROI-er må være gyldige og ferske, eller innenfor den avgrensede holdtiden for et allerede aktivt felt, for at sensoren kan delta i telling. Tidsgrensen for gamle målinger tilpasses målebudsjettet. Readiness, quorum og helsediagnostikk bruker samme helseregel. En fjerde sensor som kommer tilbake uten kalibrering, kan kalibreres mens de tre andre er i drift og åpningen er tom.

Fire-sensor-oppstart og recovery skjer trinnvis: XSHUT lav → oppvåkning → boot → adresse → registerinitialisering → første måling → konfigurasjon. Lange oppstartsventer er erstattet med tidsfrister mellom hovedløkkekjøringer. ST-bibliotekets `Init()` hadde en venteløkke uten tidsgrense; den er erstattet med en delt, kontrollert initializer som sjekker returverdier. Konfigurasjon og I2C-transaksjoner er fortsatt synkrone, men avgrensede. En fysisk låst felles buss kan fortsatt påvirke alle kanalene.

Den eldre driveren har tidsgrense på ranging og initialisering, skiller optisk ugyldige målinger fra bussfeil, og lagrer en kopi av sist brukte ROI så endrede ROI-verdier faktisk sendes til sensoren. Kalibrering har automatisk minnehåndtering, vern mot nullreferanse og kontroll av ugyldige/ustabile målinger. Den eldre driveren bruker fortsatt synkrone målinger og kalibrering; dual/triple er uavhengige tellere, ikke en parallell fusjonsløsning.

ESP8266-koden bruker ikke lenger ESP32-spesifikke FreeRTOS- og Wire-kall. Home- og fire-sensor-profiler restarter ikke automatisk ved Wi-Fi/API-brudd.

## Oppgradering og innstillinger

Lagringsversjonen er økt til 7. Tellerverdier og tuning beholdes, men eldre kalibrering forkastes fordi den kan være basert på gjentatte scheduler-målinger. Hold døråpningen tom under første oppstart med denne versjonen. Lagrede innstillinger gjelder fortsatt foran YAML-startverdier.

| Innstilling | Gjeldende betydning |
| --- | --- |
| `trigger_delta` / `release_delta` | Aktiverings-/frigivelsesforskjell fra baseline, med støypåslag |
| `debounce` | Minste tid mellom bekreftende nye målinger; minst to målinger kreves |
| `direction_window` | Bekreftelsestid for tom åpning; vises som Clear Confirmation |
| `cooldown` | Minste tomperiode; effektiv tomtid er maksimum av denne og `direction_window` |
| `sequence_timeout` | Maksimal spredning mellom sensorenes fullførte spor; stopper ikke et pågående stillestående spor |
| `min_active_duration` | Minste varighet for et gyldig sensorspor |
| `standing_timeout` | Diagnostikk for en person som stopper, uten alarm eller tvungen telling |
| `blocked_timeout` | Vedvarende nærvær innen 100 mm i begge felt: tildekkingsalarm og utelatelse av denne sensoren |
| `min_event_sensors` | Antall samstemte, komplette retningsspor; fire-sensorprofilen starter med 2 |
| `min_valid_sensors` | Minste antall brukbare, ikke tildekkede sensorer; fire-sensorprofilen starter med 3 |
| `init_retries` | Antall gjenoppstartsforsøk med kort retry-intervall før eksponentiell backoff |

Endring av sentrale telleinnstillinger forkaster pågående spor; en ny tomperiode kreves. Enkelte innstillingsentiteter har mer presise navn og kan derfor få nye Home Assistant-entitets-ID-er. Personantallets navn er beholdt.

Evidence score er en heuristisk score basert på samstemte spor. 99 betyr ikke dokumentert 99 % nøyaktighet.

## Sporlogg

`/tof-overdoor-ui/trace` eksporterer TSV med åtte ROI-er: måletid, råavstand, filtrert avstand, datidens baseline, drop, effektive terskler, rangestatus og feltaktivitet. Hendelsestilstand, helse og maskene for ferske/gyldige målinger følger hver rad. Historisk drop beregnes ikke lenger fra nåværende baseline.

Ringbufferen har 256 snapshots. Nye målinger lagres i tillegg til den ordinære kadensen, så beholdningstiden varierer og kan være kortere enn 6,4 sekunder. En HTTP-side inneholder høyst 64 rader for å begrense minnebruk. Hent neste side med `?after_ms=<siste t_ms>`. Collectoren under tømmer sider fortløpende; start den før testpasseringene:

```sh
export ROODE_USER='brukernavn'
# Sett ROODE_PASSWORD i miljøet uten å lagre passord i repoet.
python3 tools/capture_trace.py http://10.0.0.100 > /tmp/roode-trace.tsv
```

Logger har uptime-tidsstempler. Nettverksbrudd, omstart og overskriving av ringbufferen kan gi hull. Ikke bruk en kort utskrift som eneste fasit for en lang passering.

## Automatiske kontroller

`tests/run.sh` kompilerer den faktiske, delte C++-kjernen med AddressSanitizer og UndefinedBehaviorSanitizer. Den tester normale og raske passeringer, sene vendinger, lange stopp, tvetydige starter, motstridende stemmer, quorumtap, gjenoppdagede sensorer, ferske målinger, gjentatt trafikk og 32-bits klokkeoverløp. I tillegg testes 1 000 etterfølgende passeringer og alle 729 seks-trinns forløp med aktive felt.

Samme skript kompilerer også den faktiske `tof_overdoor_counter.cpp` mot små maskinvarestubber og tester målingsdrevet kalibrering, debounce, ROI-helse, historiske baselines, trinnvis recovery, avgrenset initialisering og hele flyten fra feltmåling til tellerendring. Stubbene verifiserer programlogikk, ikke fysisk I2C eller optikk.

`python3 tests/config_test.py` tester den virkelige ESPHome-konfigurasjonsvalideringen: korrekt profil og avvisning av overlappende pinner, ugyldig målebudsjett, hysterese, adresseområde og for kort initialiseringsfrist.

CI er utvidet til PR-er og alle relevante konfigurasjonsfiler. ESPHome er låst i `ci/requirements.txt`. Alle ni aktive/CI-profiler bygges, inkludert de komplette Home-profilene og fire-sensor-nettgrensesnittet. Release-oppsettet bruker `main`, én publiseringsflyt og vellykket CI som forutsetning.

## Før feltgodkjenning

Kjør scenarioene i `TOF_COUNTER_VALIDATION.md`, særlig sene vendinger, raske passeringer, tett følge, sensorfrakobling og strømbrudd. Mål falske tellinger, tapte personer, retningsfeil og recovery-tid mot en uavhengig fasit. Side-ved-side-trafikk og motsatt samtidig trafikk kan ikke garanteres separert med disse sensorene. Test også optisk interferens med én og fire aktive sensorer.

Automatiske tester og kompilering er nødvendig verifikasjon, men dokumenterer ikke fysisk tellepresisjon.

## Lokal verifikasjon 20. september 2026

- Sanitizer-testene av kjernen og den faktiske firmwarekomponenten besto.
- Alle sju positive/negative konfigurasjonstester besto.
- Disse ni profilene ble kompilert med ESPHome 2026.3.0: `ci/esp32.yaml`, `ci/esp32_manual.yaml`, `ci/esp8266.yaml`, `ci/esp8266_manual.yaml`, `ci/tof_overdoor.yaml`, `peopleCounter32Home.yaml`, `peopleCounter32HomeDual.yaml`, `peopleCounter32HomeTriple.yaml` og `peopleCounter32FourSensorOverdoor.yaml`.
- Python-verktøyene besto syntakskontroll, og `git diff --check` var ren.
- GitHub Actions er oppdatert, men ikke kjørt på GitHub fra denne økten. Ingen enhet er flashet eller fysisk validert.


## Nærpassering og tildekking – 20. september 2026

Et opptak fra sidemonteringen viste to kanselleringer når sensorer rapporterte 0–29 mm med gyldig status. Den gamle 30 mm-grensen avviste disse målingene, slettet spor og kunne utløse unødvendig power-cycle. Gyldige nærmålinger beholdes nå som nærvær; kun logikkavstanden begrenses til 30 mm, mens råverdien beholdes i diagnostikken. Ugyldige statuskoder gjøres ikke om til gyldige avstander. Et optisk avbrudd i et allerede aktivt felt kan holde forrige tilstand i inntil 150 ms, men kan ikke danne nye kanter eller bekrefte tom åpning. Bussfeil, maskinvarefeil og manglende initialisering får ikke denne toleransen.

Begge felt innen 100 mm i minst `blocked_timeout` (standard 1800 ms) utløser `Sensor Covered`, en problem-entitet i Home Assistant. Dette angir mulig tildekking; avstandssensoren kan ikke avgjøre om objektet er en hånd eller en kropp som står svært nær. Den tildekkede sensoren tas ut av avstemningen, slik at den ikke holder hele døråpningen permanent aktiv. De andre kan fortsette å telle hvis helse- og stemmekravene er oppfylt. Ved full tildekking mangler retningsinformasjon: alarmen består, men programmet finner ikke på passeringer.

Helsekrav og retningskrav er nå uavhengige. Fire-sensorprofilene bruker tre brukbare sensorer og to enige retningsspor. Dette er mindre strengt enn tre retningsstemmer og må feltvalideres for falske tellinger. En fullført motsatt retningsstemme avviser fortsatt episoden. Gamle lagrede innstillinger bevares: etter oppgradering settes **Min matching direction tracks = 2** og **Min valid sensors = 3** i Detection tuning. Ingen automatisk senking skjer ved sensorfeil.

`tests/fixtures/near_passages.txt` inneholder relative tidspunkter, råavstander, status og referanser fra de to faktiske passeringene, uten innlogging eller nettverksdata. Regresjonstesten kjører disse gjennom den faktiske måle-, filtrerings-, debounce- og tellekoden. Med to retningsstemmer gir opptaket én OUT og én IN, uten kansellering eller unsure. Med tre stemmer blir begge fortsatt unsure: opptaket mangler U8-data under omstart og inneholder ett vending-lignende spor i hver episode. Testen fyller ikke inn de manglende målingene.

Andre tester verifiserer nærpassering uten alarm, vedvarende tildekking av én sensor med fortsatt telling på de tre andre, fjerning av tildekking uten ekstra telling, stopp lenger unna uten tildekkingsalarm, full tildekking uten oppdiktet telling, og utløp av toleransen for optiske måleavbrudd. Dette er programtester; ny firmware må fortsatt valideres fysisk på installasjonen.

Verifisert for denne rettelsen: `tests/run.sh` (ASan/UBSan), alle tilfellene i `tests/config_test.py` med ESPHome 2026.8.1, og komplett ESP32-firmwarebygg av fire-sensorprofilen med ESPHome 2026.8.1. Bygget brukte offentlige testhemmeligheter og er ikke en ferdig konfigurert binærfil for brukerens enhet. Rettelsen er ikke flashet til sensoren.
