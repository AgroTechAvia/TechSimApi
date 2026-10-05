"""Offline checks for the velocity telemetry viewer."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from utils.plot_velocity_telemetry import (build_report, completed_trials,
                                           read_trace, repeated_plot_svg)


class VelocityPlotTests(unittest.TestCase):
    def test_repeated_i_plot_marks_d_scoring_start(self):
        traces = {cycle: [(0.0, 0.0), (2.0, 0.20), (4.0, 0.25)]
                  for cycle in range(1, 4)}
        plot = repeated_plot_svg(
            traces, 0.25, "i", {"velocity_i_mean_band_fraction": 0.2},
            "I trial", score_start=2.0)
        self.assertIn('class="score-marker"', plot)
        self.assertIn('class="band" x="327.5"', plot)
        self.assertIn('class="repeat repeat-1 repeat-muted"', plot)
        self.assertIn('class="mean-outside"', plot)
        self.assertIn('class="mean-inside"', plot)

    def test_repeated_d_plot_shows_transient_and_own_plateau_band(self):
        traces = {cycle: [(0.0, 0.0), (2.0, 0.20), (4.0, 0.22)]
                  for cycle in range(1, 4)}
        plot = repeated_plot_svg(
            traces, 0.25, "d", {"velocity_d_plateau_band_fraction": 0.15},
            "D trial", d_settle_time=2.0, d_plateau_center=0.22)
        self.assertIn('class="transient"', plot)
        self.assertIn('class="band" x="55" y="77.8" width="545" height="19.9"',
                      plot)
        self.assertIn('class="plateau"', plot)
        self.assertIn('class="score-marker"', plot)

    def test_validation_plot_marks_its_own_scoring_window(self):
        traces = {cycle: [(0.0, 0.0), (1.0, 1.0), (4.0, 1.0)]
                  for cycle in range(1, 4)}
        plot = repeated_plot_svg(
            traces, 1.0, "validation",
            {"velocity_i_mean_band_fraction": 0.2},
            "1 m/s validation", score_start=1.0)
        self.assertIn('class="score-marker"', plot)
        self.assertIn('class="band"', plot)
        self.assertIn('class="repeat repeat-1 repeat-muted"', plot)
        self.assertIn('class="mean-inside"', plot)

    def test_repeated_p_plots_both_speeds_and_three_flights(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            filename = "velocity_pid_vel_pitch_p_01.csv"
            with (run / filename).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=(
                    "segment", "segment_elapsed", "velocity_requested_speed",
                    "velocity_cycle", "vx_world", "vy_world", "yaw"))
                writer.writeheader()
                for cycle in range(1, 4):
                    for speed in (0.25, 1.0):
                        for direction, sign in (("positive", 1), ("negative", -1)):
                            for tick in range(5):
                                writer.writerow({"segment": direction,
                                                 "segment_elapsed": tick * 0.5,
                                                 "velocity_requested_speed": speed,
                                                 "velocity_cycle": cycle,
                                                 "vx_world": sign * speed * tick / 4,
                                                 "vy_world": 0, "yaw": 0})
            (run / "summary.json").write_text(json.dumps({
                "config": {"velocity_p_targets": [
                    {"speed": 0.25, "arrival_seconds": 1.0},
                    {"speed": 1.0, "arrival_seconds": 3.0}]},
                "evaluations": [{"stage": "velocity", "valid": True,
                                 "pid": "pid_vel_pitch",
                                 "label": "velocity_pid_vel_pitch_p_01",
                                 "telemetry_csv": filename,
                                 "gains": {"kp": 0.25, "kd": 0, "ki": 0},
                                 "metrics": {"phase": "p", "repeat_count": 3,
                                             "repeat_steps": [
                                                 {"speed": speed}
                                                 for speed in (0.25, 1.0)],
                                             "reached_steps": 12, "score": 0.1}}],
            }), encoding="utf-8")
            page = build_report(run, "pid_vel_pitch", 1)
            self.assertEqual(page.count('class="arrival"'), 4)
            self.assertEqual(page.count('class="repeat repeat-1"'), 4)
            self.assertEqual(page.count('class="filtered"'), 0)
            self.assertIn("цель +1.00 м/с", page)
            self.assertIn("цель -0.25 м/с", page)

    def test_confirmation_reads_body_frame_target_when_annotation_is_empty(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "confirmation.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=(
                    "segment", "segment_elapsed", "vx_body",
                    "commanded_axis_speed", "target_vx_body"))
                writer.writeheader()
                writer.writerow({"segment": "positive", "segment_elapsed": 0.0,
                                 "vx_body": 0.02, "commanded_axis_speed": "",
                                 "target_vx_body": 0.05})
                writer.writerow({"segment": "positive", "segment_elapsed": 0.1,
                                 "vx_body": 0.04, "commanded_axis_speed": "",
                                 "target_vx_body": 0.05})
                writer.writerow({"segment": "negative", "segment_elapsed": 0.0,
                                 "vx_body": -0.02, "commanded_axis_speed": "",
                                 "target_vx_body": -0.05})
            self.assertEqual(read_trace(path, "x", "positive")[2], 0.05)
            self.assertEqual(read_trace(path, "x", "positive")[0],
                             [(0.0, 0.02), (0.1, 0.04)])
            self.assertEqual(read_trace(path, "x", "negative")[2], -0.05)

    def test_separate_p_and_i_plots_with_braking(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            evaluations = []
            for phase in ("p", "i"):
                filename = f"velocity_pid_vel_pitch_{phase}_01.csv"
                with (run / filename).open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=(
                        "segment", "segment_elapsed", "vx_body",
                        "commanded_axis_speed"))
                    writer.writeheader()
                    for direction, sign in (("positive", 1), ("negative", -1)):
                        for tick in range(4):
                            writer.writerow({"segment": direction,
                                             "segment_elapsed": tick * 0.5,
                                             "vx_body": sign * tick * 0.08,
                                             "commanded_axis_speed": sign * 0.25})
                        for tick in range(3):
                            writer.writerow({"segment": f"{direction}_stop",
                                             "segment_elapsed": tick * 0.5,
                                             "vx_body": sign * (0.25 - tick * 0.12),
                                             "commanded_axis_speed": 0})
                evaluations.append({
                    "stage": "velocity", "valid": True, "pid": "pid_vel_pitch",
                    "label": f"velocity_pid_vel_pitch_{phase}_01",
                    "telemetry_csv": filename,
                    "gains": {"kp": 1.0, "kd": 0.01, "ki": 0.01},
                    "metrics": {"score": 0.5,
                                "positive": {"arrival_time": 1.0,
                                             "i_hold_fraction_after_arrival": 0.8},
                                "negative": {"arrival_time": 1.1,
                                             "i_hold_fraction_after_arrival": 0.7}},
                })
            (run / "summary.json").write_text(json.dumps({
                "config": {"velocity_p_target_arrival_seconds": 1.0,
                           "velocity_d_hold_band_fraction": 0.15,
                           "velocity_i_hold_band_fraction": 0.10},
                "evaluations": evaluations,
            }), encoding="utf-8")
            _, latest = completed_trials(run, "pid_vel_pitch", 1)
            self.assertEqual(latest[0]["label"], evaluations[-1]["label"])
            page = build_report(run, "pid_vel_pitch", 2)
            self.assertEqual(page.count('<section class="trial">'), 2)
            self.assertEqual(page.count('class="arrival"'), 2)
            self.assertEqual(page.count('class="band"'), 2)
            self.assertEqual(page.count('class="brake-boundary"'), 4)
            self.assertIn("цель +0.250 м/с", page)
            self.assertIn("цель -0.250 м/с", page)


if __name__ == "__main__":
    unittest.main()
