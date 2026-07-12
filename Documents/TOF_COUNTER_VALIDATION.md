# Produksjonsvalidering av fire-sensor-telleren

## Arkitektur

`peopleCounter32FourSensorOverdoor.yaml` bruker én samlet `tof_overdoor_counter` i stedet for fire blokkerende
`roode`-instanser og en YAML-avstemning. Hver VL53L1X kjører kontinuerlig og veksler mellom to ROI-er langs
passasjeretningen. Dermed gir fire fysiske sensorer åtte logiske målefelt.

En passering godkjennes først når:

1. hvert deltakende sensorpar ser en stabil reduksjon fra sin egen kalibrerte gulvreferanse;
2. sporet starter på én side, når motsatt ROI og avsluttes på motsatt side;
3. minst tre friske sensorer leverer samme retning innen hendelsesvinduet.

Et spor som ender på samme side som det startet, klassifiseres som en vending og teller ikke. En person som
stopper i åpningen beholdes i samme hendelse; sporet vurderes først når vedkommende fortsetter eller trekker seg
tilbake. Kort cooldown brukes bare til å absorbere den siste, forsinkede sensorstemmen fra samme person.

## Terskler

- `trigger_delta` er høydeforskjellen fra tom døråpning som kreves for aktivering, ikke en absolutt avstand.
- `release_delta` er hysteresegrensen for å frigi ROI-en, og skal være lavere enn `trigger_delta`.
- Målt støy øker automatisk begge effektive grensene. Dette hindrer at en støyende sensor blir mer følsom enn en
  stabil sensor.
- Baseline og støy lagres separat for alle åtte ROI-er og følges langsomt når døråpningen er tom.

Startverdiene 280 mm / 140 mm er konservative. De skal ikke justeres før råavstand, baseline, drop og
kalibreringskvalitet er samlet fra den faktiske installasjonen.

## Oppstart og recovery

- Sensoradressene er bundet til fysisk XSHUT-plass (`0x30`–`0x33`), også når en sensor mangler ved oppstart.
- En defekt sensor power-cycles og gjenoppdages isolert med eksponentiell backoff. De tre andre fortsetter å måle.
- Ved I2C-bussfeil reinitialiseres bussen før neste isolerte forsøk.
- Lagret kalibrering brukes etter restart, men telling låses til minst 500 ms med bekreftet tom åpning.
- Uten gyldig lagret kalibrering må minst tre sensorer se begge ROI-er over `minimum_clear_distance` før ny
  kalibrering godtas.
- Wi-Fi/API-brudd restarter ikke telleren. Flash-skriving batches av ESPHome for å redusere blokkering og slitasje.

## Testmatrise før påstand om >99 %

En kompilert firmware er ikke bevis på 99 % tellepresisjon. Kravet må verifiseres med synkronisert video eller en
annen sannhetskilde i den ferdige mekaniske installasjonen.

Kjør minst disse scenarioene i begge retninger:

| Scenario | Variasjon |
| --- | --- |
| Normal passering | voksne/barn, ulike høyder og klær |
| Hastighet | svært sakte, normal, løp |
| Stopp | 1 s, 3 s og 10 s i åpningen før videre gange |
| Vending | før midten, i overlapp og nesten helt gjennom |
| Tett følge | 150, 250, 400, 700 og 1000 ms mellom personer |
| Bæring | sekk, veske, eske, trillebag |
| Dørmiljø | åpen/lukket dør, skiftende lys, rengjøring og gulvobjekter |
| Feiltilstand | restart, strømbrudd og frakobling av hver enkelt sensor |

Logg for hver hendelse: sann retning/antall, rapportert resultat, confidence, sensorspor, ROI-avstand, baseline,
drop, range-status og tidsstempel. Rapporter minst:

- retningspresisjon = korrekte rapporterte retninger / alle rapporterte passeringer;
- deteksjonsgrad = korrekte registrerte personer / faktiske personer;
- falske tellinger per 24 timer uten trafikk;
- recovery-tid og tapte hendelser under sensorfeil.

For en troverdig 99 %-måling bør datasettet inneholde flere tusen uavhengige passeringer og alle vanskelige
scenarioer, ikke bare normal gange. Bruk trace-endepunktet i servicegrensesnittet til å analysere alle feil før
terskler endres.

## Maskinvarebegrensning

VL53L1X gir i denne modusen ett mål per ROI. To personer som går side ved side eller i motsatt retning samtidig kan
derfor ikke alltid separeres sikkert. Algoritmen avviser motstridende spor fremfor å gjette. Skal slike situasjoner
inngå i et absolutt >99 %-krav, kreves normalt en multizone-sensor/matrise eller større fysisk avstand mellom to
separate sensorlinjer.

ST oppgir også at eksterne 930–950 nm-kilder kan redusere signal/støy-forholdet, og anbefaler at andre IR-sensorer
ikke er aktive samtidig når feltene kan påvirke hverandre. Fire-sensorinstallasjonen må derfor testes både med én og
fire aktive sensorer. Hvis feilraten øker når alle fire er aktive, må sensorene vinkles mot ulike felt eller kjøres i
eksplisitte tidsluker; terskeljustering alene løser ikke optisk interferens.

Primærreferanser:

- [ST UM2510 – VL53L1X Ultra Lite Driver](https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf)
- [ST AN5231 – cover window and external optical interference](https://www.st.com/resource/en/application_note/dm00542648-cover-window-guidelines-for-the-vl53l1x-long-distance-ranging-time-of-flight-sensor-stmicroelectronics.pdf)
