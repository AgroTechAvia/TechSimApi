"""Offline checks for the position telemetry viewer."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from utils.auto_calibration_tool import default_config, score_position_repeats
from utils.plot_position_telemetry import build_report, read_traces


class PositionPlotTests(unittest.TestCase):
    def test_chart_normalizes_each_origin_and_shows_three_repeats(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            rows = []
            for direction, sign in (("positive", 1), ("negative", -1)):
                for cycle in (1, 2, 3):
                    origin = 7.0 + cycle
                    for tick in range(141):
                        elapsed = tick * 0.05
                        offset = sign * 0.25 * min(elapsed / 2.5, 1.0)
                        rows.append({"segment": direction,
                                     "position_cycle": cycle,
                                     "position_origin_axis": origin,
                                     "segment_elapsed": elapsed,
                                     "x": origin + offset})
            csv_path = run / "trial.csv"
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            traces = read_traces(csv_path, "x", "positive")
            self.assertEqual(set(traces), {1, 2, 3})
            self.assertAlmostEqual(traces[1][-1][1], 0.25)
            config = default_config()
            metrics = score_position_repeats(rows, "x", config, "i", 2.5)
            summary = {"config": config, "evaluations": [{
                "stage": "position", "pid": "pid_pos_x", "valid": True,
                "label": "position_pid_pos_x_i_01", "gains": {"kp": 1, "kd": 0, "ki": 0},
                "metrics": metrics, "telemetry_csv": csv_path.name}]}
            (run / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
            page = build_report(run)
            self.assertIn("position_pid_pos_x_i_01", page)
            self.assertIn("mean-outside", page)
            self.assertIn("mean-inside", page)


if __name__ == "__main__":
    unittest.main()
