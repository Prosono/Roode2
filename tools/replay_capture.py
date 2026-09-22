#!/usr/bin/env python3
"""Extract a privacy-minimal ROI fixture from the browser's JSON recording.

This is a sampled trace, not a lossless sensor recording. The output retains
sample timestamps so a held ROI value cannot be replayed as a new measurement.
Run tests/replay_capture.cpp against it to compare firmware revisions.
"""
import argparse
import collections
import datetime as dt
import json
import pathlib
import re
import statistics

ZONES = [f"{sensor}_{zone}" for sensor in ("U3", "U4", "U7", "U8") for zone in ("out", "in")]
FIELDS = ["t_ms", "healthy", "valid", "fresh", "active"] + [
    f"{zone}_{field}" for zone in ZONES
    for field in ("sample_ms", "raw", "filtered", "baseline", "trigger", "status")
]


def extract(capture, start=None, end=None):
    rows = {}
    offsets = []
    events = set()
    for item in capture["rows"]:
        if "trace" not in item:
            continue
        page = item["trace"]
        match = re.search(r"^# uptime_ms=(\d+)$", page, re.M)
        if match:
            offsets.append(item["at"] - int(match[1]))
        header = None
        in_events = False
        for line in page.splitlines():
            if line == "# event_log_begin":
                in_events = True
            elif line == "# event_log_end":
                in_events = False
            elif in_events:
                events.add(line)
            elif line.startswith("t_ms\t"):
                header = line.split("\t")
            elif header and re.match(r"^\d+\t", line):
                values = list(map(int, line.split("\t")))
                if len(values) != len(header):
                    raise ValueError("Incomplete trace row")
                row = dict(zip(header, values))
                previous = rows.get(row["t_ms"])
                if previous is not None and previous != row:
                    raise ValueError("Conflicting timestamps; possible device restart")
                rows[row["t_ms"]] = row
    if not rows or not offsets:
        raise ValueError("No timestamped ROI trace data")
    offset = statistics.median(offsets)
    def in_window(milliseconds):
        return (start is None or milliseconds >= start) and (end is None or milliseconds <= end)
    selected = [r for _, r in sorted(rows.items()) if in_window(r["t_ms"] + offset)]
    if not selected:
        raise ValueError("No rows in selected window")
    times = [r["t_ms"] for r in selected]
    gaps = [b-a for a, b in zip(times, times[1:])]
    zone_summary = {}
    for zone in ZONES:
        samples = {}
        for row in selected:
            stamp = row[f"{zone}_sample_ms"]
            if stamp and stamp >= times[0]:
                samples[stamp] = row
        stamps = sorted(samples)
        sample_gaps = [b-a for a, b in zip(stamps, stamps[1:])]
        zone_summary[zone] = {
            "unique_samples": len(stamps),
            "sample_gap_median_ms": statistics.median(sample_gaps) if sample_gaps else None,
            "sample_gap_max_ms": max(sample_gaps, default=0),
            "range_status_counts": dict(collections.Counter(r[f"{zone}_status"] for r in samples.values())),
            "trigger_min_mm": min(r[f"{zone}_trigger"] for r in samples.values()) if samples else None,
            "trigger_max_mm": max(r[f"{zone}_trigger"] for r in samples.values()) if samples else None,
        }
    selected_events = []
    for event in sorted(events):
        match = re.match(r"(\d+):(\d+):(\d+) - ", event)
        if match:
            uptime = sum(int(v)*factor for v, factor in zip(match.groups(), (3600000, 60000, 1000)))
            if in_window(uptime + offset):
                selected_events.append({"approx_wallclock": dt.datetime.fromtimestamp((uptime+offset)/1000, dt.timezone.utc).isoformat(), "event": event})
    report = {
        "format": "roode-replay-v1",
        "limitation": "Sampled snapshots can omit intermediate readings and debounce candidates. Raw replay is diagnostic, not an accuracy measurement. Clock offset includes response latency. Expected directions require a ground-truth test record.",
        "clock_offset_ms": offset,
        "clock_offset_spread_ms": max(offsets)-min(offsets),
        "first_device_ms": times[0], "last_device_ms": times[-1],
        "rows": len(selected), "snapshot_gap_max_ms": max(gaps, default=0),
        "snapshot_gap_p99_ms": sorted(gaps)[min(len(gaps)-1, int(len(gaps)*.99))] if gaps else 0,
        "healthy_mask_rows": dict(collections.Counter(r["healthy"] for r in selected)),
        "zones": zone_summary, "recorded_events": selected_events,
    }
    return selected, report


def timestamp(value):
    date = dt.datetime.fromisoformat(value)
    if date.tzinfo is None:
        raise argparse.ArgumentTypeError("Supply timezone, e.g. 2026-09-22T17:18:35+02:00")
    return date.timestamp()*1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--start", type=timestamp)
    parser.add_argument("--end", type=timestamp)
    args = parser.parse_args()
    rows, report = extract(json.loads(args.capture.read_text()), args.start, args.end)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as stream:
        stream.write("# roode-replay-v1; sampled snapshots, not a lossless capture\n")
        stream.write("# " + "\t".join(FIELDS) + "\n")
        for row in rows:
            stream.write("\t".join(str(row[field]) for field in FIELDS) + "\n")
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({key: report[key] for key in ("rows", "snapshot_gap_max_ms", "snapshot_gap_p99_ms", "healthy_mask_rows")}, indent=2))


if __name__ == "__main__":
    main()
