"""Exercise ESPHome's real schema with invalid hardware/timing configurations."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
base = (root / 'ci/tof_overdoor.yaml').read_text().replace('../components', str(root / 'components'))
cases = [
    ('valid', base, True),
    ('short fast doorway', base.replace('  timing_budget: 33ms', '  distance_mode: short\n  timing_budget: 20ms').replace('37ms','24ms'), True),
    ('optional strict timing', base.replace('  sda: 21', '  require_timing_evidence: true\n  sda: 21'), True),
    ('long cannot use 20ms', base.replace('33ms','20ms').replace('37ms','24ms'), False),
    ('overlapping pins', base.replace('GPIO25', 'GPIO16'), False),
    ('bus pin overlap', base.replace('GPIO25', 'GPIO21'), False),
    ('unsupported timing budget', base.replace('33ms', '34ms'), False),
    ('invalid hysteresis', base.replace('140mm', '400mm'), False),
    ('address overflow', base.replace('  sda: 21', '  base_address: 0x77\n  sda: 21'), False),
    ('initialization deadline too short', base.replace('  sda: 21', '  timeout: 100ms\n  sda: 21'), False),
]
with tempfile.TemporaryDirectory(prefix='roode-config-') as directory:
    path = Path(directory) / 'test.yaml'
    for name, config, expected in cases:
        path.write_text(config)
        result = subprocess.run(['esphome', 'config', str(path)], capture_output=True, text=True)
        assert (result.returncode == 0) == expected, f'{name}: {result.stdout}\n{result.stderr}'
        print(f'PASS: {name}')
