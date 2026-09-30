"""Offline checks for the acceleration telemetry viewer."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from utils.plot_acceleration_telemetry import (
    averaged_trace, build_report, completed_trials, plot_svg,
)


class AccelerationPlotTests(unittest.TestCase):
    def test_separate_candidate_plots_and_signed_target_bands(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            evaluations = []
            for index, kp in enumerate((0.5, 0.75), 1):
                filename = f"{index:04d}_acceleration_pid_accel_pitch_kp.csv"
                with (run / filename).open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=(
                        "segment", "acceleration_cycle", "segment_elapsed",
                        "ax_body", "rc_pitch"))
                    writer.writeheader()
                    for cycle in range(1, 4):
                        for direction, sign in (("positive", 1), ("negative", -1)):
                            for tick in range(10):
                                writer.writerow({
                                    "segment": direction,
                                    "acceleration_cycle": cycle,
                                    "segment_elapsed": tick * 0.1,
                                    "ax_body": sign * (0.8 + tick * 0.04),
                                    "rc_pitch": 1580 if tick < 5 else 1500,
                                })
                evaluations.append({
                    "stage": "acceleration", "valid": True,
                    "pid": "pid_accel_pitch", "label": f"candidate_{index}",
                    "telemetry_csv": filename,
                    "gains": {"kp": kp, "kd": 0.01, "ki": 0.0},
                    "metrics": ({"score": 1 / index, "phase": "d",
                                 "plateau_settling_time": 0.3,
                                 "plateau_time_by_direction": {
                                     "positive": 0.2, "negative": 0.4}}
                                if index == 2 else
                                {"score": 1 / index, "phase": "p"}),
                })
            (run / "summary.json").write_text(json.dumps({
                "config": {"acceleration_step": 1.0,
                           "acceleration_settling_band_fraction": 0.15,
                           "max_accel_rc_offset": 80},
                "evaluations": evaluations,
            }), encoding="utf-8")

            _, selected = completed_trials(run, "pid_accel_pitch", 1)
            self.assertEqual([item["label"] for item in selected], ["candidate_2"])
            page = build_report(run, "pid_accel_pitch", 2)
            self.assertEqual(page.count('<section class="trial">'), 2)
            self.assertIn("цель +1.00 м/с²", page)
            self.assertIn("цель -1.00 м/с²", page)
            self.assertIn("коридор", page)
            self.assertIn("±15%", page)
            self.assertEqual(page.count('class="band"'), 2)
            self.assertEqual(page.count('class="grid minor"'), 4 * 32)
            self.assertIn("пролёт 3", page)
            self.assertIn("на пределе 50%", page)
            self.assertIn("выход на уровень 0.30 с", page)
            self.assertEqual(page.count('class="transient"'), 2)

            evaluations.append({
                **evaluations[-1], "label": "candidate_i",
                "metrics": {"score": 4.2, "phase": "i", "hold_fraction": 0.29,
                            "plateau_settling_time": 0.9,
                            "plateau_time_by_direction": {
                                "positive": 0.9, "negative": 0.9}},
            })
            (run / "summary.json").write_text(json.dumps({
                "config": {"acceleration_step": 1.0,
                           "acceleration_settling_band_fraction": 0.20},
                "evaluations": evaluations,
            }), encoding="utf-8")
            i_page = build_report(run, "pid_accel_pitch", 1)
            self.assertIn("попадание в полосу не засчитано", i_page)
            self.assertNotIn('class="transient"', i_page)
            self.assertEqual(i_page.count('class="band"'), 2)
            self.assertEqual(i_page.count('class="mean-trace-outside"'), 2)
            self.assertNotIn('class="arrival-marker"', i_page)

    def test_fixed_vertical_scale_and_p_has_only_target(self):
        series = {1: [(0.0, 0.0), (1.0, 5.0)]}
        positive = plot_svg(series, 1.0, 0.20, "positive", show_band=False)
        negative = plot_svg(series, -1.0, 0.20, "negative", show_band=False)
        self.assertNotIn('class="band"', positive)
        self.assertIn('>-1.0</text>', positive)
        self.assertIn('>3.0</text>', positive)
        self.assertIn('>-3.0</text>', negative)
        self.assertIn('>1.0</text>', negative)
        self.assertEqual(positive.count('class="grid minor"'), 32)
        self.assertEqual(positive.count('class="grid major"'), 9)

    def test_i_marks_only_settled_mean_inside_target_band(self):
        series = {
            cycle: [(tick * 0.05, 1.4 if tick < 20 else
                     0.6 if tick < 35 else 0.7 if 80 <= tick < 95 else 1.0)
                    for tick in range(141)]
            for cycle in range(1, 4)
        }
        svg = plot_svg(series, 1.0, 0.20, "I", highlight_i=True,
                       arrival_limit=3.0, hold_seconds=6.0)
        self.assertEqual(svg.count('class="settled-marker"'), 1)
        self.assertEqual(svg.count('class="mean-trace-outside"'), 1)
        self.assertGreaterEqual(svg.count('class="mean-trace-in-band"'), 1)
        self.assertEqual(svg.count('class="repeat-muted"'), 3)
        self.assertIn("после успокоения", svg)
        missed = plot_svg({1: [(0.0, 0.9), (1.0, 0.95)]}, 1.0, 0.20,
                          "I", highlight_i=True)
        self.assertNotIn('class="settled-marker"', missed)
        self.assertNotIn('class="mean-trace-in-band"', missed)

    def test_average_trace_and_error_use_three_unsmoothed_repeats(self):
        series = {
            1: [(0.0, 0.0), (3.0, 0.8), (6.0, 0.8)],
            2: [(0.0, 0.0), (3.0, 1.0), (6.0, 1.0)],
            3: [(0.0, 0.0), (3.0, 1.2), (6.0, 1.2)],
        }
        trace, error = averaged_trace(series, 1.0)
        self.assertAlmostEqual(trace[-1][1], 1.0)
        self.assertIn("средняя ошибка 3–6 с: +0.000", error)
        self.assertIn("MAE: 0.133", error)
        svg = plot_svg(series, 1.0, 0.20, "three runs")
        self.assertEqual(svg.count('class="mean-trace"'), 1)
        self.assertIn("средняя 3 пролётов", svg)


if __name__ == "__main__":
    unittest.main()
