#!/usr/bin/env python3
"""Validate capture parsing where replay fidelity can otherwise be misleading."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("replay_capture", Path(__file__).parents[1]/"tools/replay_capture.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def page(now, stamp, raw=650, status=0):
    fields = dict.fromkeys(replay.FIELDS, 0)
    fields.update(t_ms=now, healthy=15, valid=255, fresh=1)
    for zone in replay.ZONES:
        for key, value in {"sample_ms": stamp, "raw": raw, "filtered": raw,
                           "baseline": 700, "trigger": 200, "status": status}.items():
            fields[f"{zone}_{key}"] = value
    return (f"# uptime_ms={now}\n" + "\t".join(replay.FIELDS) + "\n" +
            "\t".join(str(fields[key]) for key in replay.FIELDS) + "\n")


class CaptureTest(unittest.TestCase):
    def test_overlap_and_held_samples_are_not_new_measurements(self):
        capture = {"rows": [{"at": 10100, "trace": page(100, 100)},
                            {"at": 10100, "trace": page(100, 100)},
                            {"at": 10120, "trace": page(120, 100)},
                            {"at": 10180, "trace": page(180, 175)}]}
        rows, report = replay.extract(capture)
        self.assertEqual(len(rows), 3)
        self.assertEqual(report["zones"]["U3_out"]["unique_samples"], 2)
        self.assertEqual(report["zones"]["U3_out"]["sample_gap_median_ms"], 75)
        self.assertEqual(report["clock_offset_ms"], 10000)

    def test_conflicting_same_uptime_must_not_mix_reboots(self):
        capture = {"rows": [{"at": 10100, "trace": page(100, 100)},
                            {"at": 20100, "trace": page(100, 100, raw=100)}]}
        with self.assertRaisesRegex(ValueError, "Conflicting timestamps"):
            replay.extract(capture)

    def test_incomplete_trace_is_not_silently_replayed(self):
        data = page(100, 100).splitlines()
        data[-1] = data[-1].rsplit("\t", 1)[0]
        with self.assertRaisesRegex(ValueError, "Incomplete trace row"):
            replay.extract({"rows": [{"at": 10100, "trace": "\n".join(data)}]})

    def test_window_and_invalid_status_are_preserved(self):
        capture = {"rows": [{"at": 10100, "trace": page(100, 100)},
                            {"at": 10200, "trace": page(200, 200, raw=0, status=2)}]}
        rows, report = replay.extract(capture, start=10150, end=10300)
        self.assertEqual(rows[0]["U3_out_status"], 2)
        self.assertEqual(report["zones"]["U3_out"]["range_status_counts"], {2: 1})


if __name__ == "__main__":
    unittest.main()
