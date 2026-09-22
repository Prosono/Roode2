#!/usr/bin/env python3
"""Collector regression tests without a running ESP or network access."""
import importlib.util
import io
from pathlib import Path
import sys
import unittest

SPEC = importlib.util.spec_from_file_location(
    'capture_trace', Path(__file__).resolve().parents[1] / 'tools' / 'capture_trace.py')
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)
HEADER = 't_ms\tstate\toutcome\thealthy\tvalid\tfresh\tactive\trising\tfalling\tdrop'
EVENTS = ('# event_log_begin\n'
          '02:23:34 - OUT - Completed clear-doorway episode: 0 IN, 2 OUT\n'
          '# event_log_end\n')


def page(stamps=(), uptime=10000, events=EVENTS, metadata=''):
    return (f'# uptime_ms={uptime}\n{metadata}{events}{HEADER}\n' +
            ''.join(f'{stamp}\t2\t0\t15\t255\t4\t0\t0\t0\t-12\n' for stamp in stamps))


class CaptureTraceTest(unittest.TestCase):
    def test_event_log_only_yields_without_advancing(self):
        parsed = capture.parse_page(page())
        self.assertEqual(parsed.timestamps, ())
        progress = capture.page_progress(9999, parsed)
        self.assertEqual(progress.after, 9999)
        self.assertFalse(progress.emit)
        self.assertEqual(progress.wait_seconds, 0.25)

    def test_only_numeric_tsv_records_count_towards_full_page(self):
        parsed = capture.parse_page(page(range(100, 164)))
        self.assertEqual(len(parsed.timestamps), 64)
        progress = capture.page_progress(99, parsed)
        self.assertEqual(progress.after, 163)
        self.assertTrue(progress.emit)
        self.assertEqual(progress.wait_seconds, 0)
        parsed = capture.parse_page(page(range(100, 163)))
        self.assertEqual(capture.page_progress(99, parsed).wait_seconds, 0.25)

    def test_bad_records_and_headerless_numeric_text_are_ignored(self):
        text = page((100,)) + ('101\t2\n'
                              '102\t2\t0\t15\t255\t4\t0\t0\t0\tbroken\n'
                              '-1\t2\t0\t15\t255\t4\t0\t0\t0\t0\n'
                              '4294967296\t2\t0\t15\t255\t4\t0\t0\t0\t0\n')
        self.assertEqual(capture.parse_page(text).timestamps, (100,))
        self.assertEqual(capture.parse_page('100\t2\t0\t15\t255\t4\t0\t0\t0').timestamps, ())

    def test_duplicate_full_page_waits(self):
        progress = capture.page_progress(163, capture.parse_page(page(range(100, 164))))
        self.assertEqual(progress.after, 163)
        self.assertFalse(progress.emit)
        self.assertEqual(progress.wait_seconds, 0.25)

    def test_reboot_resets_cursor_and_waits(self):
        progress = capture.page_progress(9000, capture.parse_page(page((), uptime=120)))
        self.assertEqual(progress.after, 0)
        self.assertFalse(progress.emit)
        self.assertEqual(progress.wait_seconds, 0.25)
        self.assertIn('uptime moved backwards', progress.messages[0])

    def test_millis_wrap_advances_without_reboot(self):
        stamps = (0xFFFFFFF5, 0xFFFFFFFE, 5, 15)
        progress = capture.page_progress(0xFFFFFFF0, capture.parse_page(page(stamps, uptime=30)))
        self.assertEqual(progress.after, 15)
        self.assertTrue(progress.emit)
        self.assertFalse(progress.messages)
        self.assertFalse(capture.is_after(0xFFFFFFF5, 15))

    def test_optional_history_gap_metadata(self):
        parsed = capture.parse_page(page((200, 210), metadata=
            '# history_oldest_ms=200\n# history_newest_ms=300\n'))
        self.assertEqual((parsed.oldest, parsed.newest), (200, 300))
        progress = capture.page_progress(100, parsed)
        self.assertEqual(progress.after, 210)
        self.assertIn('may be missing', progress.messages[0])
        self.assertFalse(capture.page_progress(0, parsed).messages)
        wrapped = capture.parse_page(page((5,), uptime=20, metadata='# history_oldest_ms=5\n'))
        self.assertIn('may be missing', capture.page_progress(0xFFFFFFF0, wrapped).messages[0])

    def test_malformed_metadata_does_not_crash(self):
        parsed = capture.parse_page('# uptime_ms=oops\n# history_oldest_ms=-1\n'
                                    '# history_newest_ms=4294967296\n')
        self.assertEqual(parsed, capture.TracePage(()))

    def test_live_loop_requests_one_trace_page_per_iteration_and_yields(self):
        class Response(io.BytesIO):
            pass

        class Opener:
            def __init__(self):
                self.urls = []
                self.pages = [page((100,)), page()]

            def open(self, url, timeout):
                self.urls.append(url)
                self.last_timeout = timeout
                if not self.pages:
                    raise KeyboardInterrupt()
                return Response(self.pages.pop(0).encode())

        opener = Opener()
        waits = []
        output = io.StringIO()
        with self.assertRaises(KeyboardInterrupt):
            capture.collect('http://counter', opener, output=output,
                            errors=io.StringIO(), sleep=waits.append)
        self.assertEqual(waits, [0.25, 0.25])
        self.assertEqual(opener.urls, [
            'http://counter/tof-overdoor-ui/trace?after_ms=0',
            'http://counter/tof-overdoor-ui/trace?after_ms=100',
            'http://counter/tof-overdoor-ui/trace?after_ms=100'])
        self.assertEqual(opener.last_timeout, 5)
        self.assertEqual(output.getvalue().count('02:23:34'), 1)


if __name__ == '__main__':
    unittest.main()
