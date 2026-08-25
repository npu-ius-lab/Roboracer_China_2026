#!/usr/bin/env python3
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.telemetry import JsonlTelemetry


class TestJsonlTelemetry(unittest.TestCase):
    def test_disabled_writer_is_a_noop(self):
        telemetry = JsonlTelemetry("")
        self.assertTrue(telemetry.write({"value": 1.0}))
        telemetry.close()

    def test_background_writer_serializes_numpy(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.jsonl"
            telemetry = JsonlTelemetry(path, queue_size=4)
            self.assertTrue(telemetry.write({"array": np.asarray([1.0, 2.0])}))
            telemetry.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows, [{"array": [1.0, 2.0]}])


if __name__ == "__main__":
    unittest.main()
