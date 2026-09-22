# Separate ESPHome-profiler

| Fil | ESPHome-navn | Visningsnavn / entitetsprefiks | Statisk IP |
| --- | --- | --- | --- |
| `dorteller-oppe.yaml` | `dorteller-oppe` | Dørteller Oppe | 10.0.0.100 |
| `dorteller-nede.yaml` | `dorteller-nede` | Dørteller Nede | 10.0.0.101 |

Begge er komplette YAML-filer for ESPHome Device Builder og bruker eksisterende
`secrets.yaml` med `roode_*`-nøklene. De henter komponentene fra Roode2/main og
eksponerer de samme ti HA-entitetene, men med ulike navn. Gateway er 10.0.0.1,
nettmaske 255.255.255.0. Oppsettene har også hvert sitt navn på fallback-nettverket.

Installer Oppe-filen bare på .100 og Nede-filen bare på .101. `use_address` er
satt til riktig IP i hver fil, også for første OTA-installasjon med nytt navn.
Behold eventuell lokal API-kryptering eller andre installasjonsspesifikke
innstillinger fra ditt eksisterende ESPHome-oppsett dersom disse er lagt til.

Home Assistant kan beholde gamle entitets-ID-er i entitetsregisteret selv om
firmware får nye navn. Etter installasjon: kontroller enhetene og deres
`number.*people_inside`-ID-er i HA. Den nye `dorteller-ha-pakke.yaml` bruker de eksplisitte
`number.office_dorteller_nede_dorteller_nede_people_inside` og
`number.office_dorteller_oppe_dorteller_oppe_people_inside`. Oppdater referansene
dersom HA-entitets-ID-ene endres. Et navnebytte i firmware alene er
ingen garanti for at HA endrer eksisterende ID-er automatisk.

Kontroller lagret kalibrering, retning og persontall etter installasjon. De
generelle telleinnstillingene og de interne C++-ID-ene er beholdt i profilene.

Profilene er kontrollert med ESPHome 2026.8.1 med lokale komponenter og
testverdier for secrets. Valideringen sjekker separate IP-er, enhetsnavn og ti
ulike offentlige entitetsnavn per enhet, samt C++-kildegenerering. Ingen firmware
er lastet opp som del av denne kontrollen.
