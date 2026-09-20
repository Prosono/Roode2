#!/usr/bin/env python3
"""Continuously collect bounded trace pages; credentials are supplied via environment."""
import argparse
import os
import sys
import time
import urllib.error
import urllib.request

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('url', help='Device base URL, e.g. http://10.0.0.100')
args = parser.parse_args()
base = args.url.rstrip('/')
passwords = urllib.request.HTTPPasswordMgrWithDefaultRealm()
passwords.add_password(None, base, os.environ.get('ROODE_USER', ''), os.environ.get('ROODE_PASSWORD', ''))
opener = urllib.request.build_opener(urllib.request.HTTPBasicAuthHandler(passwords))
after = 0
try:
    while True:
        try:
            with opener.open(f'{base}/tof-overdoor-ui/trace?after_ms={after}', timeout=5) as response:
                page = response.read().decode('utf-8')
            uptime = next((int(line.split('=', 1)[1]) for line in page.splitlines() if line.startswith('# uptime_ms=')), None)
            if after and uptime is not None and ((uptime - after) & 0xFFFFFFFF) > 0x7FFFFFFF:
                print('# GAP: device uptime moved backwards; restarting cursor', flush=True)
                after = 0
                continue
            rows = [line for line in page.splitlines() if line and line[0].isdigit()]
            if rows:
                print(page, end='', flush=True)
                after = int(rows[-1].split('\t', 1)[0])
            # Drain retained pages before waiting; otherwise 64-row pages could
            # fall behind the sensor cadence during active traffic.
            if len(rows) < 64:
                time.sleep(0.25)
        except (OSError, urllib.error.URLError) as error:
            print(f'Trace connection interrupted: {error}', file=sys.stderr)
            print('# GAP: connection interrupted; next page may follow a device restart', flush=True)
            after = 0
            time.sleep(1)
except KeyboardInterrupt:
    pass
