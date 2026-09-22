# Minimal Home Assistant interface

Both four-sensor profiles expose ten entities per device:

| Domain | Name suffix | Purpose |
| --- | --- | --- |
| number | People Inside | Current occupancy; supports manual correction |
| sensor | Confirmed IN | Cumulative confirmed entries |
| sensor | Confirmed OUT | Cumulative confirmed exits |
| sensor | Unsure Detection IN | Cumulative uncertain entries |
| sensor | Unsure Detection OUT | Cumulative uncertain exits |
| binary_sensor | Counter Ready | Whether the counter is ready |
| binary_sensor | Sensor Covered | Possible sustained sensor obstruction |
| text_sensor | System Status | Short operational status |
| sensor | Uptime | Seconds since boot, sampled every 60 seconds |
| sensor | WiFi Signal | RSSI in dBm, sampled every 60 seconds |

The remaining entities use `internal: true`, excluding them from the native
Home Assistant API rather than merely disabling their HA entity registry entries.
The web server includes internal entities for local controls and diagnostics.
The custom dashboard also reads the counter directly, so its live graph and
event log remain available. The measurement loop is unchanged.

`People Inside` remains a **number** with the same name and ID. The existing
`home_assistant_combined_people_counter.yaml` package references this domain.
Its two estimates represent the same room and must not simply be added together.
`Confirmed OUT` is an exit total, not the number of people currently outdoors.

Occupancy is checked every second and published on change; confirmed IN/OUT
retain their 100 ms checks and change-only publication. ESPHome retains the
latest state for clients that reconnect. Uncertain totals and system status are
checked every five seconds. Wi-Fi and uptime add only two samples per minute.

## Install and verify

1. Build and install the updated YAML on each counter, preserving each device's
   existing name, credentials and distinct IP: `10.0.0.100` and `10.0.0.101` in
   the current installation. The repository templates default to `.100`.
2. Home Assistant may retain registry entries for entities the new firmware no
   longer exposes. These can show unavailable; remove obsolete entries after
   checking that the ten retained entities work. This is different from all
   retained entities becoming unavailable simultaneously.
3. Walk in and out, verify the cumulative totals and occupancy, and check that
   both inputs to the combined occupancy package still resolve correctly.
4. If the retained entities disconnect, inspect the ESPHome API/Wi-Fi log at
   the time of the failure. A reset in uptime suggests a reboot; continuing
   uptime suggests a connection failure, although its 60-second sampling cannot
   rule out every quick reboot. RSSI provides connection context.

Reducing the exposed entities is not proof that traffic caused previous outages.
Do not treat an unavailable counter as zero occupancy or reset its totals when
Home Assistant reconnects.

## Validation

The two profiles were schema-validated with ESPHome 2026.8.1 using local
components and dummy credentials. Their entity definitions were checked for
parity and exactly ten externally exposed entities. The local four-sensor
profile compiled successfully for ESP32, including the new uptime/Wi-Fi
sensors and the occupancy change check. This build is a validation artifact,
not a firmware image configured for the installed devices.
