#!/usr/bin/env python3
"""Package the current local components and the two named ESPHome profiles."""
import argparse
import hashlib
from pathlib import Path
import re
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]
COMPONENTS = ("counting_core", "tof_overdoor_counter", "tof_overdoor_ui")


def package(destination):
    files = {}
    for device in ("oppe", "nede"):
        config = (ROOT / f"dorteller-{device}.yaml").read_text()
        config = re.sub(
            r"# Standalone upload profile\..*?(?=\nesphome:)",
            "# Uses the tested components included in this archive.\n"
            "external_components:\n  - source:\n      type: local\n"
            "      path: roode_components\n"
            "    components: [counting_core, tof_overdoor_counter, tof_overdoor_ui]\n",
            config, count=1, flags=re.S,
        )
        if "type: git" in config or "path: roode_components" not in config:
            raise ValueError("Could not replace the external component source")
        files[f"dorteller-{device}.yaml"] = config.encode()
    for component in COMPONENTS:
        for path in sorted((ROOT / "components" / component).iterdir()):
            if path.suffix in (".py", ".h", ".cpp"):
                files[f"roode_components/{component}/{path.name}"] = path.read_bytes()
    files["LES-MEG.txt"] = (
        "ROODE – testoppdatering for den sidemonterte telleren\n\n"
        "Kopier begge YAML-filene og hele roode_components-mappen til /config/esphome/.\n"
        "Bruk eksisterende secrets.yaml med roode_* nøklene. Arkivet inneholder ingen passord.\n"
        "Installer dorteller-oppe.yaml på 10.0.0.100 og dorteller-nede.yaml på 10.0.0.101.\n"
        "Ta vare på tidligere konfigurasjon/firmware før oppdatering.\n"
        "Dette er kildefiler for bygging i ESPHome, ikke ferdige .bin-filer.\n\n"
        "La døren være helt fri ved første oppstart; gamle kalibreringer må måles på nytt.\n"
        "Tellere og lagrede innstillinger beholdes. Vent til begge viser Ready.\n"
        "Kortdistansemodus/20 ms er valgt for åpningen på 80–90 cm.\n"
        "Streng tidskontroll er avslått; usikker timing logges for videre testing.\n\n"
        "Test først én enhet, deretter begge: 8 vekslende UT/IN med ca.5 sekunders mellomrom.\n"
        "Test deretter nær sensor, vanlig fart, skrå gange og å snu i døråpningen.\n"
        "Reell treffrate er ennå ikke verifisert med denne firmwareversjonen.\n"
    ).encode()
    files["SHA256SUMS.txt"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(files.items())
    ).encode()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(destination, "w", compression=ZIP_DEFLATED) as archive:
        for name, data in sorted(files.items()):
            archive.writestr(name, data)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", nargs="?", type=Path, default=ROOT / ".esphome/roode-doorway-update.zip")
    print(package(parser.parse_args().output))
