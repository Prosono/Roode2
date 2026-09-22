#!/usr/bin/env python3
"""Continuously collect bounded trace pages; credentials are supplied via environment."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import re
import sys
import time
import urllib.error
import urllib.request

UINT32_MASK = 0xFFFFFFFF
HALF_RANGE = 0x80000000
PAGE_ROWS = 64
MAX_PAGE_BYTES = 1024 * 1024
IDLE_SECONDS = 0.25
INTEGER = re.compile(r'-?[0-9]+\Z')


@dataclass(frozen=True)
class TracePage:
    timestamps: tuple[int, ...]
    uptime: int | None = None
    oldest: int | None = None
    newest: int | None = None


@dataclass(frozen=True)
class PageProgress:
    after: int
    emit: bool
    wait_seconds: float
    messages: tuple[str, ...] = ()


def is_after(value: int, reference: int) -> bool:
    """Compare bounded device timestamps across an ordinary millis() wrap."""
    return 0 < ((value - reference) & UINT32_MASK) < HALF_RANGE


def parse_page(text: str) -> TracePage:
    """Read only complete numeric TSV records, never timestamped event-log lines."""
    metadata = {}
    timestamps = []
    columns = None
    for line in text.splitlines():
        if line.startswith('# '):
            key, separator, value = line[2:].partition('=')
            if separator and key in ('uptime_ms', 'history_oldest_ms', 'history_newest_ms'):
                if value.isascii() and value.isdecimal() and int(value) <= UINT32_MASK:
                    metadata[key] = int(value)
            continue
        fields = line.split('\t')
        if fields[:9] == ['t_ms', 'state', 'outcome', 'healthy', 'valid', 'fresh',
                          'active', 'rising', 'falling']:
            columns = len(fields)
            continue
        if columns is None or len(fields) != columns:
            continue
        if not all(INTEGER.fullmatch(field) for field in fields):
            continue
        stamp = int(fields[0])
        if 0 <= stamp <= UINT32_MASK:
            timestamps.append(stamp)
    return TracePage(tuple(timestamps), metadata.get('uptime_ms'),
                     metadata.get('history_oldest_ms'), metadata.get('history_newest_ms'))


def page_progress(after: int, page: TracePage) -> PageProgress:
    """Choose the next cursor and pacing without making another device request."""
    if after and page.uptime is not None and is_after(after, page.uptime):
        return PageProgress(0, False, IDLE_SECONDS,
                            ('# GAP: device uptime moved backwards; restarting cursor',))
    messages = ()
    if after and page.oldest is not None and is_after(page.oldest, after):
        messages = ('# GAP: cursor predates retained history; trace samples may be missing',)
    fresh = tuple(stamp for stamp in page.timestamps if not after or is_after(stamp, after))
    if not fresh:
        # Even a full repeated page must yield; event logs and stale records
        # cannot make the collector hammer a quiet device.
        return PageProgress(after, False, IDLE_SECONDS, messages)
    cursor = fresh[-1]
    # Drain the bounded retained history before waiting, but only if the page
    # actually advanced the cursor. All other paths have an explicit wait.
    wait = 0.0 if len(page.timestamps) >= PAGE_ROWS and cursor != after else IDLE_SECONDS
    return PageProgress(cursor, True, wait, messages)


def collect(base: str, opener, *, output=None, errors=None, sleep=time.sleep) -> None:
    output = sys.stdout if output is None else output
    errors = sys.stderr if errors is None else errors
    after = 0
    while True:
        try:
            with opener.open(f'{base}/tof-overdoor-ui/trace?after_ms={after}', timeout=5) as response:
                data = response.read(MAX_PAGE_BYTES + 1)
            if len(data) > MAX_PAGE_BYTES:
                raise ValueError('trace response exceeded the bounded page size')
            text = data.decode('utf-8')
            progress = page_progress(after, parse_page(text))
            for message in progress.messages:
                print(message, file=output, flush=True)
            if progress.emit:
                print(text, end='' if text.endswith('\n') else '\n', file=output, flush=True)
            after = progress.after
            if progress.wait_seconds:
                sleep(progress.wait_seconds)
        except (OSError, urllib.error.URLError, ValueError) as error:
            print(f'Trace connection interrupted: {error}', file=errors, flush=True)
            print('# GAP: connection interrupted; next page may follow a device restart',
                  file=output, flush=True)
            after = 0
            sleep(1)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url', help='Device base URL, e.g. http://10.0.0.100')
    args = parser.parse_args(argv)
    base = args.url.rstrip('/')
    passwords = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    passwords.add_password(None, base, os.environ.get('ROODE_USER', ''), os.environ.get('ROODE_PASSWORD', ''))
    opener = urllib.request.build_opener(urllib.request.HTTPBasicAuthHandler(passwords))
    try:
        collect(base, opener)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
