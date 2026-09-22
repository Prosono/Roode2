# Roode: samlet persontall og testdashbord

Pakken er for to tellere som observerer **samme rom**, for eksempel side- og
 toppmontering ved samme dør. Den summerer ikke to uavhengige rom eller passasjer.

## Installer pakken

1. Kopier `dorteller-ha-pakke.yaml` til
   `/config/packages/roode_people_counter.yaml` på Home Assistant.
2. Aktiver pakker i `configuration.yaml` hvis du ikke allerede bruker dem:

   ```yaml
   homeassistant:
     packages: !include_dir_named packages
   ```

   Slå dette sammen med eksisterende `homeassistant:`-blokk. Ikke opprett en
   ekstra blokk. Dashboard-filen skal ikke ligge i packages-mappen.
3. Pakken bruker de eksakte entitets-ID-ene fra installasjonen din:

   - Nede: `number.office_dorteller_nede_dorteller_nede_people_inside`
   - Oppe: `number.office_dorteller_oppe_dorteller_oppe_people_inside`

   Det gjøres ingen automatiske oppslag via integrasjons- eller enhetsnavn.
   Dashbordet viser alle åtte oppgitte entiteter per teller: persontall, IN/OUT,
   klar-status, tildekkingsalarm, systemstatus og usikre IN/OUT. Status og
   hendelsestotaler vises også som attributter på pakkens kildesensorer; bare
   People Inside brukes til beregningen av antall personer.
   Korrigeringsknappen skriver direkte til de to number-entitetene ovenfor.
   Endres disse ID-ene senere, må referansene oppdateres i pakken og dashbordet.

   Hjelpesensorene heter `sensor.roode_nede_people_inside` og
   `sensor.roode_oppe_people_inside`. Testene bruker de samme eksplisitte ID-ene.
4. Erstatt en eventuell tidligere installasjon av kombinasjonspakken; ikke last
   inn begge. Den gamle varslingsautomasjonen inngår ikke i den nye pakken.
5. Kontroller konfigurasjonen i HA og start HA på nytt. Navn som tidligere er
   endret i entitetsregisteret beholdes av HA; tilpass dashboard/referanser hvis
   registeret bruker andre ID-er enn de foreslåtte navnene.

## Installer dashbordet

Opprett et nytt, tomt dashboard. Åpne redigering og råkonfigurasjonseditoren,
og lim inn hele `dorteller-ha-dashboard.yaml`.
Filen har `title:` og `views:` og skal ikke limes inn som ett enkelt kort.
Dashbordet bruker bare innebygde kort; HACS er ikke nødvendig.

Du får samlet antall, begge kildene, avvik, tilgjengelighetshistorikk, fasit for
testing, tellefeil og lenker til enhetene på `.100` og `.101`.

## Regler og begrensninger

| Målinger | Samlet resultat | Status |
| --- | --- | --- |
| Begge har samme gyldige heltall | Dette antallet | agreement |
| Begge gyldige, men uenige | Høyeste antall | conflict |
| Bare side tilgjengelig | Side | nede_only |
| Bare topp tilgjengelig | Topp | oppe_only |
| Ingen gyldige målinger | unavailable, aldri automatisk null | offline |

Negative tall, desimaltall, NaN og ugyldig tekst forkastes. Forskjell og enighet
blir utilgjengelige når en kilde mangler. `Roode Counter Problem` slås på etter
30 sekunder med uenighet eller manglende kilde, slik at en kort forskjell i
rapporteringstid ikke utløser problemindikatoren. Denne ventetiden begynner på
nytt etter HA-omstart. Antallet oppdateres uten denne ventetiden.

Høyeste antall er et forsiktig **estimat**, ikke matematisk bevis på antallet i
rommet. Det kan overestimere hvis en teller har hengt igjen på et høyt tall, og
begge kan ta feil samtidig. Det finnes ingen sikker måte å velge riktig teller
bare ut fra to motstridende totalsummer. Denne pakken fusjonerer totalsummer,
ikke tidsstemplete råpasseringer. Den reparerer heller ikke Wi-Fi/API-utfall.

Tilgjengelighet følger HA-kildene. En fastlåst, men fortsatt numerisk verdi kan
ikke oppdages sikkert uten ekstra helsedata. Ikke bruk en tidsgrense på siste
verdiendring: et tomt rom kan helt legitimt stå på null i mange timer.

Bruk `sensor.roode_combined_people_inside` som estimert antall og
`binary_sensor.roode_room_occupied` som tilstedeværelse. For automasjoner som
krever sikker bekreftelse på tomt rom, krev i tillegg at
`sensor.roode_counter_fusion_status` er `agreement` over en passende periode.
Ikke konverter `unavailable` til null i slike automasjoner.

## Praktisk test

1. Tell faktisk antall personer og legg tallet i «Fasit for testen».
2. Ved behov: trykk «Korriger begge tellere». Bekreftelsen skriver fasiten til
   begge fysiske telleres `People Inside`. IN/OUT-totalene nullstilles ikke.
   Ha døren fri mens dette gjøres. Ingen automatisk korrigering kjøres.
3. Gå inn og ut én av gangen; oppdater fasiten etter hver test. Prøv ulik fart,
   nær sensor, snuing i døren og flere etter hverandre.
4. Kontroller feilen: positivt betyr overestimering, negativt underestimering.
5. Koble fra én teller. Den andre skal fortsatt gi et estimat. Koble fra begge;
   samlet antall og tilstedeværelse skal bli utilgjengelige, ikke null/av.
6. Korrigeringsskriptet krever to gyldige kilder. Skriving til to ESP-er er ikke
   atomisk: hvis forbindelsen brytes midt i handlingen, kontroller begge tallene
   og korriger igjen når begge er tilbake.

Fasit-hjelperen brukes kun til test og gjenopprettes av HA når mulig. Den blir
ikke brukt som en skjult tredje kilde til romestimatet.

## Verifisering

`tests/ha_fusion_test.py` renderer pakkens faktiske Jinja-maler med simulert
HA-tilstand og tester 13 scenarier: enighet, uenighet, én/begge utilgjengelige,
null, negativt/desimaltall, NaN/Infinity og ugyldig tekst. Den kontrollerer også
tilgjengelighet, tilstedeværelse, korrigeringsvilkår og dashboardets entitetsreferanser.
Dette er lokal logikktesting, ikke en full HA-konfigurasjonskontroll eller en
visuell test i din HA-installasjon. Kjør HAs konfigurasjonskontroll før omstart.

Referanser:
- [HA-pakker](https://www.home-assistant.io/docs/configuration/packages/)
- [Template-integrasjonen](https://www.home-assistant.io/integrations/template/)
- [Dashboard-handlinger](https://www.home-assistant.io/dashboards/actions/)

Pakken er JSON-formatert YAML; kopier fra første `{` til siste `}`.
Dashbordet er vanlig YAML uten multiline-tekstblokker eller flow collections.
Kopier hele dashbordfilen fra `title: Roode romtest` til slutten. De lengre filnavnene i repoet
er identiske kompatibilitetskopier; installer bare én pakke, ikke begge.

