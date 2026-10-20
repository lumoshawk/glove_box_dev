"""Offline regression checks; no ROS node or hardware connection is created."""
import contextlib
import io
import unittest
from types import SimpleNamespace

from glove_box_bringup.read_all import ReadAll, REQUIRED_FIELDS


class ReadAllTest(unittest.TestCase):
    def reader(self, states, raw=False):
        return SimpleNamespace(
            motors=[('test', m) for m in states], states=states, raw=raw,
            profiles={m: {'drive': 'test'} for m in states},
            decode={'test': lambda sw: 'ready'}, deg=lambda m, p: p,
        )

    def valid(self):
        return dict(status_word=0x21, error_code=0, mode_display=8,
                    position=100, velocity=0)

    def report(self, reader):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = ReadAll.report(reader)
        return ok, output.getvalue()

    def test_nonfinite_feedback_and_raw_preserve_other_motors(self):
        for field in REQUIRED_FIELDS:
            for value in (float('nan'), float('inf'), -float('inf')):
                with self.subTest(field=field, value=value):
                    bad = self.valid()
                    bad[field] = value
                    reader = self.reader({'bad': bad, 'good': self.valid()}, raw=True)
                    self.assertFalse(ReadAll.complete(reader))
                    ok, output = self.report(reader)
                    self.assertFalse(ok)
                    self.assertIn(f'{field}={value}', output)
                    self.assertIn('PASS', output)

    def test_missing_fields(self):
        for field in REQUIRED_FIELDS:
            state = self.valid()
            del state[field]
            reader = self.reader({'motor': state})
            self.assertFalse(ReadAll.complete(reader))
            ok, output = self.report(reader)
            self.assertFalse(ok)
            self.assertIn(f'{field}=missing', output)

    def test_no_data(self):
        reader = self.reader({'motor': {}})
        self.assertFalse(ReadAll.complete(reader))
        ok, output = self.report(reader)
        self.assertFalse(ok)
        self.assertIn('NO DATA', output)

    def test_valid_and_fault_feedback(self):
        for sw, error, expected in ((0x21, 0, True), (0x08, 0, False),
                                    (0x21, 1, False), (0, 0, False)):
            state = self.valid()
            state.update(status_word=sw, error_code=error)
            reader = self.reader({'motor': state})
            self.assertTrue(ReadAll.complete(reader))
            ok, output = self.report(reader)
            self.assertEqual(ok, expected)
            self.assertIn('PASS' if expected else 'FAIL', output)


if __name__ == '__main__':
    unittest.main()
