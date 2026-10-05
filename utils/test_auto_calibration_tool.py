"""Offline checks for the standalone calibration controller and score."""

import copy
import json
import io
import math
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from statistics import mean
from unittest.mock import patch

from utils.auto_calibration_tool import (
    CalibrationController,
    CalibrationRunner,
    TimedAccelerationPID,
    InterventionDetected,
    LandingIncomplete,
    Sample,
    TrialInvalid,
    default_config,
    get_drone_pid_setup,
    load_config,
    main,
    score_rows,
    score_acceleration_steps,
    score_acceleration_repeats,
    _acceleration_response_stable,
    score_height_waypoints,
    score_yaw_waypoints,
    score_position_waypoints,
    score_position_repeats,
    _position_p_ready,
    _position_d_ready,
    _position_i_ready,
    score_velocity_steps,
    score_velocity_p_arrival,
    score_velocity_repeats,
    _height_descent_metrics,
    _height_stage2_stable,
    _yaw_stage_stable,
    _velocity_response_stable,
    _velocity_repeated_d_ready,
    _position_waypoints_stable,
    _invalid_xy_trial,
    _prefer_trial,
    _duplicate_kinematics_window,
    validate_config,
    wrap_pi,
)


class CalibrationToolTests(unittest.TestCase):
    def setUp(self):
        self.config = default_config()
        self.controller = CalibrationController(self.config, get_drone_pid_setup("DEFAULT"))
        self.sample = Sample(0.0, 0.0, 0.0, 1.0, 0.0)
        self.target = {
            "height": 1.2,
            "yaw": 0.4,
            "position": (0.5, 0.0),
            "velocity": (0.12, 0.0),
        }

    def test_position_repeats_use_each_start_and_mean_three_flights(self):
        rows = []
        for direction, sign in (("positive", 1), ("negative", -1)):
            for cycle in (1, 2, 3):
                origin = 7.0 + cycle if sign > 0 else -3.0 - cycle
                for tick in range(141):
                    elapsed = tick * 0.05
                    displacement = sign * 0.25 * min(elapsed / 2.5, 1.0)
                    rows.append({"segment": direction, "position_cycle": cycle,
                                 "segment_elapsed": elapsed, "x": origin + displacement,
                                 "position_origin_axis": origin})
        p = score_position_repeats(rows, "x", self.config, "p")
        d = score_position_repeats(rows, "x", self.config, "d")
        i = score_position_repeats(rows, "x", self.config, "i", 2.5)
        self.assertTrue(_position_p_ready(p, self.config))
        self.assertAlmostEqual(p["mean_arrival_seconds"], 2.3, delta=0.15)
        self.assertTrue(_position_d_ready(d, self.config))
        self.assertTrue(_position_i_ready(i, self.config))
        self.assertAlmostEqual(i["terminal_mae"], 0.0, delta=0.005)

    def test_position_p_score_does_not_reward_hold_or_braking(self):
        def rows(overshoot):
            result = []
            for direction, sign in (("positive", 1), ("negative", -1)):
                for cycle in (1, 2, 3):
                    for tick in range(141):
                        t = tick * 0.05
                        displacement = sign * (0.25 * min(t / 2.5, 1.0) +
                                               (overshoot if t > 3.0 else 0.0))
                        result.append({"segment": direction,
                                       "position_cycle": cycle,
                                       "segment_elapsed": t,
                                       "x": 12 + displacement,
                                       "position_origin_axis": 12})
            return result
        clean = score_position_repeats(rows(0.0), "x", self.config, "p")
        shifted = score_position_repeats(rows(0.15), "x", self.config, "p")
        self.assertAlmostEqual(clean["score"], shifted["score"])

    def test_position_tuner_checks_zero_d_and_i_and_stops_after_one_pass(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["position_phase"]
            gains = candidate[name]
            calls.append((phase, gains["kp"], gains["kd"], gains["ki"]))
            steps = [
                {"direction": direction, "arrived": gains["kp"] > 0,
                 "arrival_time": 2.5 if gains["kp"] > 0 else None,
                 "settled": True, "settling_time": 1.0,
                 "plateau_hold_fraction": 1.0,
                 "plateau_center": sign * 0.25}
                for direction, sign in (("positive", 1), ("negative", -1))]
            metrics = {"score": 0.0, "mean_steps": steps,
                       "mean_response_by_direction": {
                           direction: {"score_window_valid": True,
                                       "hold_fraction": 1.0}
                           for direction in ("positive", "negative")}}
            return {"label": label, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_position_pid("pid_pos_x", "x"))
        self.assertEqual([call[0] for call in calls], ["p", "p", "d", "i"])
        self.assertAlmostEqual(calls[2][1], self.config["position_p_start"] * 0.95)
        self.assertEqual(calls[2][2], 0.0)
        self.assertEqual(calls[3][3], 0.0)
        self.assertEqual(runner.stage_summary["pid_pos_x"]["phase_trial_counts"],
                         {"p": 2, "d": 1, "i": 1})

    def test_position_profile_uses_post_brake_origin_for_every_step(self):
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.anchor = runner.last_sample = Sample(0, 0, 4, 9, 0.5)
        runner.initial_sample = runner.last_sample
        runner._write_csv = lambda label, rows: "trace.csv"
        targets = []
        brakes = []

        def brake(label, rows, **kwargs):
            brakes.append(label)
            self.assertIsNone(kwargs.get("active_axis"))
            runner.last_sample = Sample(len(brakes), 6.0 + len(brakes), 4, 9, 0.5)
            return True

        def segment(stage, target, duration, label, rows, **kwargs):
            targets.append((label, target["position"], target["active_axis"]))
            rows.append({"segment": label})

        runner._brake_velocity_vector = brake
        runner._segment = segment
        rows, filename = runner._profile_position_repeats("x", "trial", "p")
        self.assertEqual(filename, "trace.csv")
        self.assertEqual(len(targets), 6)
        self.assertEqual(len(brakes), 7)
        self.assertEqual(targets[0], ("positive", (7.25, 4), "x"))
        self.assertEqual(targets[1], ("negative", (7.75, 4), "x"))
        self.assertEqual(rows[0]["position_origin_axis"], 7)
        self.assertEqual(rows[1]["position_origin_axis"], 8)

    def test_height_disables_yaw_and_xy(self):
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "height", self.target)
        self.assertEqual((output["rc_roll"], output["rc_pitch"], output["rc_yaw"]),
                         (1500, 1500, 1500))
        self.assertGreater(output["rc_throttle"], 1500)

    def test_yaw_disables_xy_but_keeps_height(self):
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "yaw", self.target)
        self.assertEqual((output["rc_roll"], output["rc_pitch"]), (1500, 1500))
        self.assertNotEqual(output["rc_yaw"], 1500)
        self.assertGreater(output["rc_throttle"], 1500)

    def test_direct_world_velocity_is_rotated_to_body_frame(self):
        self.controller.reset(self.sample)
        moving = Sample(0.1, 0.0, 0.0, 1.0, math.pi / 2,
                        vx_world=1.0, vy_world=0.0, vz_world=0.0)
        output = self.controller.step(
            moving, "acceleration",
            {**self.target, "acceleration": (0.0, 0.0), "active_axis": None},
        )
        self.assertAlmostEqual(output["vx_body"], 0.0, places=6)
        self.assertAlmostEqual(output["vy_body"], 1.0, places=6)

    def test_acceleration_holds_selected_yaw(self):
        self.controller.reset(self.sample)
        target = {**self.target, "acceleration": (1.0, 0.0), "active_axis": "x"}
        output = self.controller.step(self.sample, "acceleration", target)
        self.assertNotEqual(output["rc_yaw"], 1500)

    def test_summary_write_retries_transient_windows_file_lock(self):
        runner = CalibrationRunner(self.config)
        original_replace = Path.replace
        attempts = []

        def replace_with_one_lock(source, destination):
            attempts.append(destination)
            if len(attempts) == 1:
                raise PermissionError("temporary lock")
            return original_replace(source, destination)

        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory)
            with patch.object(Path, "replace", replace_with_one_lock), \
                    patch("utils.auto_calibration_tool.time.sleep"):
                runner._save_summary()
            self.assertEqual(len(attempts), 2)
            self.assertTrue((runner.run_dir / "summary.json").is_file())

    def test_resume_acceleration_pitch_prefers_successful_trial(self):
        source = CalibrationRunner(self.config)
        source.records = [
            {"stage": "acceleration", "pid": "pid_accel_pitch", "valid": True,
             "label": label, "telemetry_csv": f"{label}.csv",
             "gains": {"kp": 0.45, "ki": ki, "kd": 0.01},
             "metrics": {"phase": "i", "score": score,
                         "hold_fraction": hold, "reached_directions": 2,
                         "i_filtered_response_by_direction": {
                             "positive": {"settled": True},
                             "negative": {"settled": True}},
                         "hold_fraction_by_direction": {
                             "positive": hold, "negative": hold}}}
            for label, ki, score, hold in (
                ("low_score_low_hold", 0.50, 0.4, 0.80),
                ("successful", 0.68, 0.8, 0.93),
            )]
        with tempfile.TemporaryDirectory() as directory:
            source.run_dir = Path(directory)
            source._save_summary()
            resumed = CalibrationRunner(self.config)
            resumed.resume_acceleration_pitch_from(source.run_dir)
        self.assertAlmostEqual(resumed.best["pid_accel_pitch"]["ki"], 0.68)
        self.assertEqual(resumed.stage_summary["pid_accel_pitch"]["resumed_trial"],
                         "successful")

    def test_acceleration_integral_uses_elapsed_time_and_output_limit(self):
        pid = TimedAccelerationPID(kp=0.0, ki=1.0, kd=0.0,
                                   max_control=0.8, i_limit=0.25,
                                   integral_output_limit=0.4)
        pid.update_control(1.0, dt=0.1)
        self.assertAlmostEqual(pid.get_control(), 0.1)
        pid.update_control(1.0, dt=0.2)
        self.assertAlmostEqual(pid.get_control(), 0.3)
        pid.update_control(1.0, dt=0.2)
        self.assertAlmostEqual(pid.get_control(), 0.4)
        pid.reset()
        self.assertAlmostEqual(pid.integral, 0.0)

    def test_acceleration_integral_does_not_wind_up_at_pwm_limit(self):
        pid = TimedAccelerationPID(kp=2.0, ki=1.0, kd=0.0,
                                   max_control=0.8, integral_output_limit=0.4)
        pid.update_control(1.0, dt=0.1)
        self.assertAlmostEqual(pid.integral, 0.0)
        self.assertAlmostEqual(pid.get_control(), 0.8)
        pid.update_control(0.1, dt=0.1)
        self.assertAlmostEqual(pid.integral, 0.01)
        self.assertAlmostEqual(pid.get_control(), 0.21)

    def test_controller_acceleration_integral_uses_sample_interval(self):
        configs = {name: values.copy() for name, values in
                   self.controller.pid_configs.items()}
        configs["pid_accel_pitch"].update(kp=0.0, ki=1.0, kd=0.0)
        self.controller.set_configs(configs)
        target = {**self.target, "acceleration": (1.0, 0.0), "active_axis": "x"}
        first = self.controller.step(self.sample, "acceleration", target)
        later = Sample(0.02, 0.0, 0.0, 1.0, 0.0)
        second = self.controller.step(later, "acceleration", target)
        self.assertAlmostEqual(
            second["pid_accel_pitch_integral_output"] -
            first["pid_accel_pitch_integral_output"], 0.02)
        self.assertEqual(second["pid_accel_pitch_integral_limited"], 0)

    def test_acceleration_preference_rejects_quiet_but_weak_response(self):
        def metrics(*, reached, terminal, in_band, score):
            return {
                "score": score, "reached_both": reached == 2,
                "reached_directions": reached, "terminal_mae": terminal,
                "in_band_fraction": in_band, "oscillation_rms": 0.02,
                "p95_jerk": 2.0,
                "positive": {"reached": reached >= 1},
                "negative": {"reached": reached >= 2},
            }

        quiet_but_weak = metrics(reached=0, terminal=0.075, in_band=0.15, score=5.0)
        reaches_target = metrics(reached=2, terminal=0.020, in_band=0.55, score=9.0)
        self.assertTrue(_prefer_trial(reaches_target, quiet_but_weak,
                                      "acceleration", 0.15, self.config))

    def test_acceleration_tuner_uses_p_then_d_then_i_and_validation(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def fake_evaluate(stage, name, axis, candidate, label, **kwargs):
            gains = candidate[name]
            calls.append((label, gains["kp"], gains["kd"], gains["ki"],
                          kwargs.get("acceleration_magnitude")))
            magnitude = kwargs.get("acceleration_magnitude") or self.config["acceleration_step"]
            kp, kd, ki = gains["kp"], gains["kd"], gains["ki"]
            reached = 2 if kp >= 0.5 else 0
            terminal = (0.01 if ki >= 0.01 and kd >= 0.01 else
                        0.02 if kd >= 0.01 else 0.04)
            phase = kwargs.get("acceleration_phase")
            score = ({"p": (kp - 1.0) ** 2,
                      "d": (kd - 0.02) ** 2,
                      "i": (ki - 0.02) ** 2}[phase])
            metrics = {
                "score": score,
                "reached_both": reached == 2,
                "reached_directions": reached,
                "p_reached_directions": reached,
                "reached_steps": 6 if reached else 0,
                "mean_arrival_seconds": 0.10 if reached else None,
                "arrival_time_error_seconds": 0.0 if reached else 0.65,
                "p_score": 5.0 if reached == 0 else 0.5,
                "terminal_mae": terminal,
                "in_band_fraction": 0.8 if kd >= 0.01 else 0.2,
                "oscillation_rms": 0.02,
                "p95_jerk": 2.0 if kd >= 0.01 else 4.0,
                "max_peak_overshoot": 0.0,
                "positive": {"reached": reached == 2},
                "negative": {"reached": reached == 2},
            }
            return {"metrics": metrics, "telemetry_csv": "fake.csv"}

        runner._evaluate = fake_evaluate
        self.assertTrue(runner._tune_acceleration_pid("pid_accel_pitch", "x"))
        gains = runner.best["pid_accel_pitch"]
        self.assertAlmostEqual(
            gains["kp"],
            runner.stage_summary["pid_accel_pitch"]["p_search"]["chosen_kp"] *
            self.config["acceleration_p_after_d_factor"])
        self.assertAlmostEqual(gains["kd"], 0.02)
        self.assertAlmostEqual(
            gains["ki"], 0.02,
            delta=self.config["acceleration_i_coarse_step"] *
            self.config["acceleration_search_final_fraction"])
        self.assertEqual(
            [item[4] for item in calls if "validate" in item[0]],
            [0.05, 0.15, 0.5],
        )

    def test_acceleration_p_search_refines_first_reached_bracket(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def fake_evaluate(stage, name, axis, candidate, label, **kwargs):
            kp = candidate[name]["kp"]
            calls.append((label, kp))
            arrived = kp >= 0.3
            arrival = 0.05 / kp if arrived else None
            error = abs(arrival - 0.10) if arrived else 1.40
            score = error / 0.10 + (0 if arrived else 16)
            return {"metrics": {
                "score": score, "reached_both": arrived,
                "reached_directions": 2 if arrived else 0,
                "p_reached_directions": 2 if arrived else 0,
                "reached_steps": 6 if arrived else 0,
                "mean_arrival_seconds": arrival,
                "arrival_time_error_seconds": error,
                "p_score": score,
                "terminal_mae": score / 400,
                "in_band_fraction": 0.0, "oscillation_rms": 0.02,
                "p95_jerk": 2.0,
                "max_peak_overshoot": 0.0,
                "positive": {"reached": False}, "negative": {"reached": False},
            }, "telemetry_csv": "fake.csv"}

        runner._evaluate = fake_evaluate
        runner._tune_acceleration_pid("pid_accel_pitch", "x")
        p_values = [kp for label, kp in calls if "_kp_" in label]
        self.assertEqual(p_values[:3], [0.0, 1.0, 0.5])
        self.assertLessEqual(max(p_values), 1.0)
        self.assertTrue(any("_kd_pass0_" in label for label, _ in calls))
        chosen_p = runner.stage_summary["pid_accel_pitch"]["p_search"]["chosen_kp"]
        self.assertGreaterEqual(chosen_p, 0.4)
        self.assertLessEqual(chosen_p, 0.5)
        self.assertAlmostEqual(runner.best["pid_accel_pitch"]["kp"],
                               chosen_p * self.config["acceleration_p_after_d_factor"])
        self.assertTrue(runner.stage_summary["pid_accel_pitch"]["p_search"]["target_met"])

    def test_acceleration_i_stops_at_success_or_two_scores_worse_than_best(self):
        def search(i_scores, success_at):
            runner = CalibrationRunner(self.config)
            runner._save_summary = lambda: None
            labels = []

            def evaluate(stage, name, axis, candidate, label, **kwargs):
                labels.append(label)
                kp = candidate[name]["kp"]
                ki = candidate[name]["ki"]
                phase = kwargs["acceleration_phase"]
                index = round(ki / self.config["acceleration_i_coarse_step"])
                hold = 0.95 if success_at is not None and index >= success_at else 0.5
                score = (kp - 1.0) ** 2 if phase == "p" else (
                    candidate[name]["kd"] if phase == "d" else
                    i_scores.get(index, 30.0))
                metrics = {
                    "score": score, "reached_directions": 2,
                    "reached_steps": 6 if kp >= 1.0 else 0,
                    "mean_arrival_seconds": 0.1 if kp >= 1.0 else None,
                    "arrival_time_error_seconds": 0.0 if kp >= 1.0 else 1.0,
                    "hold_fraction": hold,
                    "hold_fraction_by_direction": {"positive": hold,
                                                   "negative": hold},
                    "i_filtered_response_by_direction": {
                        "positive": {"settled": True},
                        "negative": {"settled": True}},
                }
                return {"metrics": metrics, "telemetry_csv": "fake.csv"}

            runner._evaluate = evaluate
            runner._tune_acceleration_pid("pid_accel_pitch", "x")
            return runner, labels

        successful, labels = search({0: 20.0, 1: 17.0, 2: 16.0}, 2)
        self.assertEqual([label for label in labels if "_ki_pass0_" in label], [
            "acceleration_pid_accel_pitch_ki_pass0_00",
            "acceleration_pid_accel_pitch_ki_pass0_01",
            "acceleration_pid_accel_pitch_ki_pass0_02",
        ])
        self.assertFalse(any("_ki_pass1_" in label for label in labels))
        self.assertAlmostEqual(successful.best["pid_accel_pitch"]["ki"], 0.08)

        _, labels = search({0: 20.0, 1: 17.0, 2: 19.0, 3: 18.0, 4: 16.0}, None)
        self.assertEqual(len([label for label in labels if "_ki_pass0_" in label]), 4)

    def test_acceleration_p_search_reports_unreachable_arrival_time(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        tested_p = []

        def fake_evaluate(stage, name, axis, candidate, label, **kwargs):
            kp = candidate[name]["kp"]
            phase = kwargs.get("acceleration_phase")
            if phase == "p":
                tested_p.append(kp)
            reached = kp >= 0.5
            arrival = 0.01 if reached else None
            error = 0.09 if reached else 1.40
            return {"metrics": {
                "score": (error / 0.10 + (0 if reached else 16)
                          if phase == "p" else 3.0),
                "reached_steps": 6 if reached else 0,
                "reached_directions": 2 if reached else 0,
                "mean_arrival_seconds": arrival,
                "arrival_time_error_seconds": error,
                "reached_both": reached,
                "terminal_mae": 0.5,
                "oscillation_rms": 0.5,
                "p95_jerk": 2.0,
            }, "telemetry_csv": "fake.csv"}

        runner._evaluate = fake_evaluate
        runner._tune_acceleration_pid("pid_accel_pitch", "x")
        self.assertFalse(runner.stage_summary["pid_accel_pitch"]["p_search"]["target_met"])
        self.assertLessEqual(max(tested_p), 1.0)
        self.assertTrue(any(0 < value < 1 for value in tested_p))
        self.assertAlmostEqual(
            runner.stage_summary["pid_accel_pitch"]["p_search"]["chosen_kp"], 0.5)

    def test_acceleration_keeps_tested_p_if_all_d_and_i_trials_are_invalid(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            if kwargs.get("acceleration_phase") != "p":
                return None
            kp = candidate[name]["kp"]
            reached = kp >= 1.0
            return {"metrics": {
                "score": abs(kp - 1.0),
                "reached_steps": 6 if reached else 0,
                "reached_directions": 2 if reached else 0,
                "mean_arrival_seconds": 0.1 if reached else None,
                "arrival_time_error_seconds": 0.0 if reached else 1.0,
            }, "telemetry_csv": "p.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_acceleration_pid("pid_accel_pitch", "x"))
        chosen = runner.stage_summary["pid_accel_pitch"]["p_search"]["chosen_kp"]
        self.assertEqual(runner.best["pid_accel_pitch"]["kp"], chosen)
        self.assertEqual(runner.best["pid_accel_pitch"]["kd"], 0.0)
        self.assertEqual(runner.best["pid_accel_pitch"]["ki"], 0.0)

    def test_acceleration_p_reaches_by_crossing_without_in_band_hold(self):
        rows = []
        for direction, sign in (("positive", 1), ("negative", -1)):
            for tick in range(20):
                value = sign * (1.2 if tick == 5 else 0.2)
                rows.append({
                    "segment": direction, "ax_body": value,
                    "target_ax_body": sign * self.config["acceleration_step"],
                    "segment_elapsed": tick * 0.02,
                    "segment_seconds": 0.4,
                })
        metrics = score_acceleration_steps(rows, "x", self.config)
        self.assertEqual(metrics["p_reached_directions"], 2)
        self.assertAlmostEqual(metrics["positive"]["p_arrival_time"], 0.1)
        self.assertFalse(metrics["reached_both"])

    def test_acceleration_d_averages_three_flights_and_requires_hold(self):
        rows = []
        for cycle in range(1, 4):
            for direction, sign in (("positive", 1), ("negative", -1)):
                for tick in range(400):
                    elapsed = tick * 0.02
                    value = sign * (1.0 if elapsed >= 0.5 and cycle < 3 else 0.2)
                    rows.append({
                        "acceleration_cycle": cycle,
                        "segment": direction,
                        "ax_body": value,
                        "target_ax_body": sign,
                        "segment_elapsed": elapsed,
                        "segment_seconds": 8.0,
                    })
        metrics = score_acceleration_repeats(rows, "x", self.config, "d")
        self.assertEqual(metrics["repeat_count"], 3)
        self.assertEqual(metrics["reached_directions"], 2)
        self.assertAlmostEqual(metrics["hold_fraction"], 2 / 3, places=2)
        self.assertFalse(_acceleration_response_stable(metrics, self.config))
        truncated = [row for row in rows if row["segment_elapsed"] <= 4.0]
        with self.assertRaises(TrialInvalid):
            score_acceleration_repeats(truncated, "x", self.config, "d")

    def test_acceleration_i_scores_settled_smoothed_mean_not_raw_samples(self):
        def flight(levels):
            rows = []
            for cycle, level in enumerate(levels, 1):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    for tick in range(351):
                        elapsed = tick * 0.02
                        value = sign * (1.1 if elapsed < 0.2 else level)
                        rows.append({
                            "acceleration_cycle": cycle, "segment": direction,
                            "ax_body": value, "target_ax_body": sign,
                            "segment_elapsed": elapsed, "segment_seconds": 7.0,
                        })
            return score_acceleration_repeats(rows, "x", self.config, "i")

        centered = flight((0.65, 1.0, 1.35))
        biased = flight((0.35, 0.7, 1.05))
        self.assertLess(centered["raw_hold_fraction"], 0.4)
        self.assertGreater(centered["hold_fraction"], 0.95)
        self.assertLess(biased["hold_fraction"], 0.05)
        self.assertLess(centered["i_score"], biased["i_score"])
        self.assertTrue(all(item["settled"] for item in
                            centered["i_filtered_response_by_direction"].values()))
        self.assertTrue(all(abs(item["score_start"] - 2.0) < 1e-9 for item in
                            centered["i_filtered_response_by_direction"].values()))

    def test_acceleration_d_prefers_early_plateau_even_with_target_bias(self):
        self.assertAlmostEqual(self.config["acceleration_settling_band_fraction"], 0.20)

        def flight(transient_seconds, level=0.4):
            rows = []
            for cycle in range(1, 4):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    for tick in range(351):
                        elapsed = tick * 0.02
                        value = level + (1.0 if tick % 2 == 0 else -1.0)
                        if elapsed >= transient_seconds:
                            value = level
                        rows.append({
                            "acceleration_cycle": cycle,
                            "segment": direction,
                            "ax_body": sign * value,
                            "target_ax_body": sign,
                            "segment_elapsed": elapsed,
                            "segment_seconds": 7.0,
                        })
            return score_acceleration_repeats(rows, "x", self.config, "d")

        early, late = flight(1.0), flight(3.0)
        self.assertEqual(early["reached_steps"], 6)
        self.assertEqual(early["hold_fraction"], late["hold_fraction"])
        self.assertAlmostEqual(early["plateau_center_by_direction"]["positive"], 0.4)
        self.assertAlmostEqual(early["plateau_center_by_direction"]["negative"], -0.4)
        self.assertLess(early["plateau_settling_time"], late["plateau_settling_time"])
        self.assertLess(early["d_score"], late["d_score"])
        near_target = flight(1.0, level=0.9)
        self.assertGreater(near_target["hold_fraction"], early["hold_fraction"])
        self.assertAlmostEqual(near_target["d_score"], early["d_score"])
        self.assertLess(near_target["i_score"], early["i_score"])
        over_damped = flight(1.0, level=0.05)
        self.assertGreater(over_damped["weak_plateau_penalty"], 0)
        self.assertGreater(over_damped["d_score"], early["d_score"])

    def test_acceleration_p_arrival_score_is_symmetric_around_target_time(self):
        def trial(arrival_tick):
            rows = []
            for cycle in range(1, 4):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    for tick in range(150):
                        rows.append({
                            "acceleration_cycle": cycle,
                            "segment": direction,
                            "ax_body": sign * (1.0 if tick == arrival_tick else 0.1),
                            "target_ax_body": sign,
                            "segment_elapsed": tick * 0.01,
                            "segment_seconds": 1.5,
                        })
            return score_acceleration_repeats(rows, "x", self.config, "p")

        early, on_time, late = trial(7), trial(10), trial(13)
        self.assertAlmostEqual(early["score"], late["score"])
        self.assertAlmostEqual(on_time["score"], 0.0)
        self.assertEqual(on_time["reached_steps"], 6)

    def test_yaw_waypoints_score_wrap_and_reject_transient_overshoot(self):
        self.config["yaw_stage_targets_deg"] = [35, 0, -35, 0]

        def flight(overshoot_deg=0, height_error=0):
            rows = []
            current = math.radians(170)
            origin = current
            for index, angle in enumerate(self.config["yaw_stage_targets_deg"], 1):
                target = wrap_pi(origin + math.radians(angle))
                start = current
                delta = wrap_pi(target - start)
                for tick in range(61):
                    fraction = min(1, tick / 20)
                    extra = (math.radians(overshoot_deg) * (1 if delta >= 0 else -1)
                             if 25 <= tick <= 30 else 0)
                    current = wrap_pi(start + delta * fraction + extra)
                    rows.append({
                        "waypoint_index": index, "requested_target_yaw": target,
                        "segment": f"yaw_wp_{index:02d}_hold", "t": len(rows) * 0.1,
                        "segment_elapsed": tick * 0.1, "segment_seconds": 6.0,
                        "yaw": current, "target_yaw": target,
                        "origin_yaw": origin, "z": 1.0 + height_error,
                        "target_z": 1.0, "x": 0.0, "y": 0.0,
                        "origin_x": 0.0, "origin_y": 0.0,
                        "rc_roll": 1500, "rc_pitch": 1500,
                        "rc_throttle": 1620, "rc_yaw": 1500,
                        "saturated": 0, "warnings": "", "sample_gap": 0.1,
                    })
            return score_yaw_waypoints(rows, self.config)

        good = flight()
        bad = flight(15)
        self.assertTrue(_yaw_stage_stable(good, self.config))
        self.assertFalse(_yaw_stage_stable(bad, self.config))
        self.assertLess(good["score"], bad["score"])
        self.assertAlmostEqual(good["waypoints"][0]["target_yaw"],
                               wrap_pi(math.radians(205)))
        self.assertFalse(_yaw_stage_stable(flight(height_error=0.2), self.config))

    def test_yaw_only_uses_resumed_height_and_does_not_tune_xy(self):
        self.config["vertical_repeated_mode"] = False
        class Link:
            def arm(self):
                pass

            def disarm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, link=Link())
        runner._prepare_height_ascent = lambda: None
        runner.initial_sample = Sample(0, 0, 0, -0.019, 0)
        runner.last_sample = runner.initial_sample
        runner.anchor = runner.initial_sample
        runner.best_height_base_rc = 1620
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None

        def tune(stage, name, axis):
            self.assertEqual((stage, name, axis), ("yaw", "pid_yaw", "yaw"))
            self.assertTrue(runner.height_waypoints_mode)
            self.assertTrue(runner.yaw_waypoints_mode)
            self.assertTrue(runner._target(height=1.0)["height_relative"])
            self.assertEqual(runner.controller.base_throttle_rc, 1620)
            runner.stage_summary[name] = {"stable": True}
            return True

        runner._tune_pid = tune
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_yaw_only()
        self.assertEqual(runner.status, "complete")

    def test_xy_profiles_keep_axes_isolated_and_use_launch_relative_height(self):
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "trial.csv"
        calls = []

        def segment(stage, target, duration, label, rows, **kwargs):
            calls.append((stage, label, target, duration))
            rows.append({"segment": label})

        runner._segment = segment
        runner._wait_velocity_rest = lambda axis, rows, label: rows.append({"segment": label})
        speed_rows, _ = runner._profile("velocity", "x", "speed_trial")
        moves = [call for call in calls if call[1].endswith(("positive", "negative"))]
        self.assertEqual(len(moves), 2)
        self.assertTrue(all(call[2]["active_axis"] == "x" and
                            call[2]["height_relative"] for call in moves))
        self.assertEqual([abs(call[2]["velocity"][0]) for call in moves],
                         [self.config["velocity_target_speed"]] * 2)
        calls.clear()
        pos_rows, _ = runner._profile("position", "y", "position_trial")
        self.assertEqual(len(calls), 4 * len(self.config["position_calibration_distances"]))
        self.assertTrue(all(call[2]["active_axis"] == "y" and
                            call[2]["height_relative"] and
                            call[2]["position"][0] == self.sample.x
                            for call in calls))
        self.assertEqual(len({row["waypoint_index"] for row in pos_rows}), len(calls))

    def test_xy_profiles_hold_nonzero_starting_heading_and_offset_position(self):
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        start = Sample(0.0, 23.0, -17.0, 1.0, 1.1)
        runner.initial_sample = runner.anchor = runner.last_sample = start
        runner.xy_reference_yaw = start.yaw
        runner._write_csv = lambda label, rows: "trial.csv"
        runner._wait_velocity_rest = lambda axis, rows, label: rows.append(
            {"segment": label})
        calls = []

        def segment(stage, target, duration, label, rows, **kwargs):
            calls.append((stage, label, target))
            rows.append({"segment": label})

        runner._segment = segment
        runner._profile("velocity", "x", "speed")
        self.assertTrue(all(math.isclose(call[2]["yaw"], start.yaw)
                            for call in calls))
        calls.clear()
        runner._profile("position", "x", "position")
        self.assertTrue(all(math.isclose(call[2]["yaw"], start.yaw)
                            for call in calls))
        outward = [call[2]["position"] for call in calls
                   if not math.isclose(call[2]["position"][0], start.x)]
        self.assertTrue(outward)
        self.assertTrue(all(math.isclose(position[1], start.y)
                            for position in outward))

    def test_position_world_x_uses_both_body_axes_at_nonzero_yaw(self):
        controller = CalibrationController(
            self.config, get_drone_pid_setup("DEFAULT"))
        start = Sample(0.0, 23.0, -17.0, 1.0, math.pi / 4)
        controller.reset(start)
        target = {
            "height": 1.0, "height_relative": False,
            "yaw": start.yaw, "position": (start.x + 0.5, start.y),
            "velocity": (0.0, 0.0), "active_axis": "x",
        }
        output = controller.step(start, "position", target)
        self.assertNotEqual(output["pid_vel_pitch_error"], 0.0)
        self.assertNotEqual(output["pid_vel_roll_error"], 0.0)
        self.assertNotEqual(output["rc_pitch"], 1500)
        self.assertNotEqual(output["rc_roll"], 1500)

    def test_velocity_p_arrival_score_is_symmetric_around_target_time(self):
        def response(seconds, arrived=True):
            return {"positive": {"arrived": arrived, "arrival_time": seconds,
                                  "duration": 5.0},
                    "negative": {"arrived": arrived, "arrival_time": seconds,
                                  "duration": 5.0}}

        self.assertAlmostEqual(score_velocity_p_arrival(response(0.8), self.config),
                               score_velocity_p_arrival(response(1.2), self.config))
        self.assertEqual(score_velocity_p_arrival(response(1.0), self.config), 0.0)
        self.assertGreater(score_velocity_p_arrival(response(5.0, False), self.config),
                           score_velocity_p_arrival(response(0.8), self.config))

    def test_repeated_velocity_scores_p_d_i_without_braking(self):
        def rows_for(phase, *, damping=0.4, bias=0.0):
            plans = (self.config["velocity_p_targets"] if phase == "p" else
                     [{"speed": self.config["velocity_target_speed"],
                       "arrival_seconds": 1.0}])
            rows = []
            for cycle in range(1, 4):
                for plan in plans:
                    speed = plan["speed"]
                    for direction, sign in (("positive", 1), ("negative", -1)):
                        for tick in range(71):
                            elapsed = tick * 0.1
                            if phase == "p":
                                value = sign * speed * min(
                                    1.0, elapsed / plan["arrival_seconds"])
                            else:
                                value = sign * (speed * (1 - bias) *
                                                (1 - math.exp(-elapsed / 0.35)) +
                                                0.12 * math.exp(-elapsed / damping) *
                                                math.sin(8 * elapsed))
                            rows.append({"segment": direction, "velocity_cycle": cycle,
                                         "velocity_requested_speed": speed,
                                         "segment_elapsed": elapsed, "vx_body": value})
                        rows.append({"segment": f"{direction}_service_brake",
                                     "velocity_cycle": cycle,
                                     "velocity_requested_speed": speed,
                                     "segment_elapsed": 1.0, "vx_body": 20.0})
            return rows

        p = score_velocity_repeats(rows_for("p"), "x", self.config, "p")
        self.assertEqual(p["reached_steps"], 2)
        self.assertEqual(len(p["repeat_steps"]), 6)
        self.assertEqual(len(p["mean_steps"]), 2)
        self.assertLess(p["score"], 0.2)
        d_fast = score_velocity_repeats(rows_for("d", damping=0.4),
                                        "x", self.config, "d")
        d_slow = score_velocity_repeats(rows_for("d", damping=2.5),
                                        "x", self.config, "d")
        self.assertLess(d_fast["score"], d_slow["score"])
        i_good = score_velocity_repeats(rows_for("i", bias=0.15),
                                        "x", self.config, "i", i_score_start=2.0)
        i_bad = score_velocity_repeats(rows_for("i", bias=0.30),
                                       "x", self.config, "i", i_score_start=2.0)
        self.assertGreater(i_good["hold_fraction"], i_bad["hold_fraction"])
        self.assertLess(i_good["score"], i_bad["score"])

    def test_velocity_p_uses_only_015_meter_target_at_one_second(self):
        self.assertEqual(self.config["velocity_p_targets"], [
            {"speed": 0.15, "arrival_seconds": 1.0,
             "tolerance_seconds": 0.20}])
        self.assertEqual(self.config["velocity_validation_speeds"],
                         [0.1, 0.25, 1.0])
        validate_config(self.config)

    def test_velocity_validation_scores_each_requested_speed(self):
        for speed in self.config["velocity_validation_speeds"]:
            rows = []
            for cycle in range(1, 4):
                for direction, sign in (("positive", 1), ("negative", -1)):
                    for tick in range(81):
                        elapsed = tick * 0.1
                        rows.append({
                            "segment": direction, "velocity_cycle": cycle,
                            "velocity_requested_speed": speed,
                            "segment_elapsed": elapsed,
                            "vx_body": sign * speed * min(1.0, elapsed / 0.5),
                        })
            metrics = score_velocity_repeats(
                rows, "x", self.config, "validation", requested_speed=speed)
            self.assertEqual(metrics["validation_target_speed"], speed)
            self.assertEqual(metrics["reached_steps"], 2)
            self.assertTrue(metrics["validation_passed"])

    def test_velocity_validation_runs_three_speeds_without_retuning(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        name = "pid_vel_pitch"
        runner.stage_summary[name] = {"stable": True}
        gains_before = runner.best[name].copy()
        seen = []

        def evaluate(stage, pid, axis, candidate, label, **kwargs):
            seen.append((axis, kwargs["velocity_phase"],
                         kwargs["velocity_magnitude"]))
            speed = kwargs["velocity_magnitude"]
            return {"telemetry_csv": f"{speed:g}.csv",
                    "metrics": {"validation_passed": speed != 1.0}}

        runner._evaluate = evaluate
        self.assertFalse(runner._validate_velocity_speeds(name, "x"))
        self.assertEqual(seen, [("x", "validation", speed) for speed in
                                (0.1, 0.25, 1.0)])
        self.assertEqual(runner.best[name], gains_before)
        self.assertTrue(runner.stage_summary[name]["stable"])
        self.assertEqual([item["passed"] for item in
                          runner.stage_summary[name]["speed_validation"]],
                         [True, True, False])

    def test_repeated_velocity_i_uses_fixed_d_scoring_window(self):
        speed = self.config["velocity_target_speed"]
        rows = []
        for cycle in range(1, 4):
            for direction, sign in (("positive", 1), ("negative", -1)):
                for tick in range(81):
                    elapsed = tick * 0.1
                    rows.append({"segment": direction, "velocity_cycle": cycle,
                                 "velocity_requested_speed": speed,
                                 "segment_elapsed": elapsed,
                                 "vx_body": sign * (0.0 if elapsed < 3.0 else speed)})
        early = score_velocity_repeats(rows, "x", self.config, "i",
                                       i_score_start=2.0)
        late = score_velocity_repeats(rows, "x", self.config, "i",
                                      i_score_start=4.0)
        self.assertEqual(early["i_score_start_seconds"], 2.0)
        self.assertGreater(late["hold_fraction"], early["hold_fraction"])
        self.assertEqual(late["hold_fraction"], 1.0)

    def test_velocity_p_scores_arrival_of_three_flight_mean(self):
        self.config["velocity_p_targets"] = [
            {"speed": 0.25, "arrival_seconds": 1.0,
             "tolerance_seconds": 0.15}]
        rows = []
        for cycle in range(1, 4):
            for direction, sign in (("positive", 1), ("negative", -1)):
                for tick in range(51):
                    elapsed = tick * 0.1
                    rows.append({"segment": direction, "velocity_cycle": cycle,
                                 "velocity_requested_speed": 0.25,
                                 "segment_elapsed": elapsed,
                                 "vx_body": sign * (0.25 if cycle == 1 or
                                                   elapsed >= 3.0 else 0.0)})
        metrics = score_velocity_repeats(rows, "x", self.config, "p")
        individual = [part["arrival_time"] for part in metrics["repeat_steps"]]
        averaged = [part["arrival_time"] for part in metrics["mean_steps"]]
        self.assertLess(mean(individual), 2.2)
        self.assertTrue(all(arrival > 2.8 for arrival in averaged))

    def test_velocity_d_scores_oscillation_of_three_flight_mean(self):
        speed = self.config["velocity_target_speed"]
        rows = []
        for cycle in range(1, 4):
            for direction, sign in (("positive", 1), ("negative", -1)):
                for tick in range(71):
                    elapsed = tick * 0.1
                    offset = ((0.12 if cycle == 1 else -0.12)
                              * math.sin(10 * elapsed) if cycle < 3 else 0.0)
                    rows.append({"segment": direction, "velocity_cycle": cycle,
                                 "velocity_requested_speed": speed,
                                 "segment_elapsed": elapsed,
                                 "vx_body": sign * (speed + offset)})
        metrics = score_velocity_repeats(rows, "x", self.config, "d")
        self.assertTrue(all(part["settled"] for part in metrics["mean_steps"]))
        self.assertLess(sum(part["settled"] for part in metrics["repeat_steps"]), 6)
        self.assertTrue(_velocity_repeated_d_ready(metrics, self.config))

    def test_repeated_velocity_profile_uses_service_brake_between_flights(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "trial.csv"
        calls = []

        def segment(stage, target, duration, label, rows, **kwargs):
            calls.append((label, target["velocity"], duration))
            rows.append({"segment": label, "vx_body": 0.0, "vy_body": 0.0})

        def brake(label, rows, **kwargs):
            calls.append((label, None, 0.0, kwargs["active_axis"]))
            return False  # Failed cleanup must not discard a flight.

        runner._segment = segment
        runner._brake_velocity_vector = brake
        rows, _ = runner._profile_velocity_repeats("x", "repeat_p", "p")
        flights = [item for item in calls if item[0] in ("positive", "negative")]
        brakes = [item for item in calls if item[0].endswith("_service_brake")]
        self.assertEqual(calls[0][0], "pre_first_service_brake")
        self.assertEqual(len(flights), 6)
        self.assertEqual(len(brakes), 7)
        self.assertTrue(all(item[3] == "x" for item in brakes))
        self.assertEqual({abs(item[1][0]) for item in flights},
                         {self.config["velocity_target_speed"]})
        self.assertEqual({row["velocity_cycle"] for row in rows}, {1, 2, 3})
        self.assertFalse(any(item[1] == (0.0, 0.0) for item in flights))

    def test_velocity_validation_profile_uses_short_requested_speed(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "validation.csv"
        flights = []

        def segment(stage, target, duration, label, rows, **kwargs):
            if label in ("positive", "negative"):
                flights.append((target["velocity"], duration))
            rows.append({"segment": label, "vx_body": 0.0, "vy_body": 0.0})

        runner._segment = segment
        runner._brake_velocity_vector = lambda label, rows, **kwargs: True
        runner._profile_velocity_repeats("x", "validation_1", "validation", 1.0)
        self.assertEqual(flights, [((sign * 1.0, 0.0), 5.0)
                                   for _ in range(3) for sign in (1, -1)])

    def test_velocity_flights_start_scored_time_after_signed_zero_crossing(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "trial.csv"
        calls = []

        def sample_with_speed(value):
            return Sample(0.0, 0.0, 0.0, 1.0, 0.0,
                          vx_world=value, vy_world=0.0, vz_world=0.0)

        def brake(label, rows, **kwargs):
            # Each normal brake leaves a small velocity opposite the next
            # command. That speed must not add to the scored arrival time.
            runner.last_sample = sample_with_speed(
                0.07 if label.startswith("positive") else -0.07)
            return True

        def segment(stage, target, duration, label, rows, **kwargs):
            sign = 1 if target["velocity"][0] > 0 else -1
            calls.append((label, duration, target["velocity"][0]))
            if label.startswith("pre_"):
                for elapsed, value in ((0.0, -0.07), (0.25, -0.02),
                                       (0.5, 0.0), (0.75, 0.03)):
                    row = {"segment": label, "segment_elapsed": elapsed,
                           "vx_world": sign * value, "vy_world": 0.0,
                           "yaw": 0.0}
                    rows.append(row)
                    if kwargs["stop_when"](rows[-1:]):
                        break
            else:
                for elapsed in (0.0, 0.25, 0.5, 0.75, 1.0, 5.0):
                    rows.append({"segment": label,
                                 "segment_elapsed": elapsed,
                                 "vx_world": sign * min(0.15, elapsed * 0.2),
                                 "vy_world": 0.0, "yaw": 0.0})

        runner._brake_velocity_vector = brake
        runner._segment = segment
        rows, _ = runner._profile_velocity_repeats("x", "zero_cross", "p")
        warmups = [call for call in calls if call[0].endswith("_zero_cross")]
        flights = [call for call in calls if call[0] in ("positive", "negative")]
        self.assertEqual(len(warmups), 6)
        self.assertEqual(len(flights), 6)
        self.assertTrue(all(call[1] == 5.0 for call in flights))
        self.assertTrue(all(max(row["segment_elapsed"] for row in rows
                                if row["segment"] == label) == 0.5
                            for label in ("pre_positive_zero_cross",
                                          "pre_negative_zero_cross")))
        self.assertTrue(all(row["segment_elapsed"] == 0.0 for row in rows
                            if row["segment"] in ("positive", "negative") and
                            row["vx_world"] == 0.0))

    def test_velocity_zero_cross_timeout_scores_candidate_as_failure(self):
        rows = [{"segment": "pre_positive_zero_cross",
                 "velocity_zero_cross_failed": 1,
                 "velocity_cycle": 1,
                 "velocity_requested_speed": 0.15}]
        metrics = score_velocity_repeats(rows, "x", self.config, "p")
        self.assertEqual(metrics["zero_cross_failures"], 1)
        self.assertFalse(metrics["reached_both"])
        self.assertGreater(metrics["score"], 3.0)

    def test_velocity_y_start_already_toward_target_is_braked_through_zero(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = Sample(
            0.0, 0.0, 0.0, 1.0, 0.0,
            vx_world=0.0, vy_world=0.07, vz_world=0.0)
        runner._write_csv = lambda label, rows: "trial.csv"
        calls = []

        def brake(label, rows, **kwargs):
            calls.append((label, kwargs.get("zero_cross_sign")))
            speed = (-0.01 if kwargs.get("zero_cross_sign") == 1 else
                     0.07 if label == "pre_first_service_brake" or
                     label.startswith("positive") else -0.07)
            runner.last_sample = Sample(0.0, 0.0, 0.0, 1.0, 0.0,
                                        vx_world=0.0, vy_world=speed,
                                        vz_world=0.0)
            return True

        def segment(stage, target, duration, label, rows, **kwargs):
            sign = 1 if target["velocity"][1] > 0 else -1
            value = 0.0 if label.startswith("pre_") else sign * 0.15
            rows.append({"segment": label, "vy_world": value,
                         "vx_world": 0.0, "yaw": 0.0})

        runner._brake_velocity_vector = brake
        runner._segment = segment
        runner._profile_velocity_repeats("y", "y_zero_cross", "p")
        self.assertIn(("pre_positive_axis_zero_brake", 1), calls)

    def test_repeated_velocity_tuner_reaches_d_and_i_with_fixed_upstream_gains(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["velocity_phase"]
            gains = candidate[name].copy()
            calls.append((phase, gains))
            if phase == "p":
                steps = [{"speed": plan["speed"], "direction": direction,
                          "arrived": True, "arrival_time": plan["arrival_seconds"]}
                         for plan in self.config["velocity_p_targets"]
                         for direction in ("positive", "negative")
                         for _ in range(3)]
                metrics = {"phase": phase, "score": abs(gains["kp"] - 0.25),
                           "repeat_steps": steps,
                           "mean_steps": [{"speed": plan["speed"],
                                           "direction": direction, "cycle": "mean",
                                           "arrived": True,
                                           "arrival_time": plan["arrival_seconds"]}
                                          for plan in self.config["velocity_p_targets"]
                                          for direction in ("positive", "negative")]}
            elif phase == "d":
                settled = gains["kd"] >= 0.05
                metrics = {"phase": phase, "score": abs(gains["kd"] - 0.05),
                           "repeat_steps": [{"direction": direction,
                                             "cycle": cycle,
                                             "settled": settled,
                                             "settling_time": 1.5 if settled else 7.0,
                                             "plateau_fraction": 1.0}
                                            for direction in ("positive", "negative")
                                            for cycle in range(1, 4)],
                           "mean_steps": [{"direction": direction,
                                           "cycle": "mean", "settled": settled,
                                           "settling_time": 1.5 if settled else 7.0,
                                           "plateau_fraction": 1.0}
                                          for direction in ("positive", "negative")]}
            else:
                hold = 0.86 if gains["ki"] >= 0.002 else 0.5
                self.assertEqual(kwargs["velocity_i_score_start"], 1.5)
                metrics = {"phase": phase, "score": abs(gains["ki"] - 0.002),
                           "i_mean_response_by_direction": {
                               direction: {"score_window_valid": True,
                                           "hold_fraction": hold}
                               for direction in ("positive", "negative")},
                           "terminal_mae": 0.0, "oscillation_rms": 0.0}
            return {"label": label, "gains": gains, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_velocity_pid_cycle_repeated(
            "pid_vel_pitch", "x"))
        self.assertEqual({phase for phase, _ in calls}, {"p", "d", "i"})
        self.assertEqual([gains["kd"] for phase, gains in calls if phase == "d"][:7],
                         [0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0])
        p_used_for_d = next(gains["kp"] for phase, gains in calls if phase == "d")
        self.assertTrue(all(gains["kp"] == p_used_for_d for phase, gains in calls
                            if phase in ("d", "i")))
        selected_d = runner.best["pid_vel_pitch"]["kd"]
        self.assertTrue(any(gains["kd"] == selected_d for phase, gains in calls
                            if phase == "i"))
        self.assertEqual([gains["ki"] for phase, gains in calls if phase == "i"][-1],
                         0.002)

    def test_velocity_tuner_skips_d_and_i_search_when_zero_passes(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["velocity_phase"]
            gains = candidate[name].copy()
            calls.append((phase, gains["kd"], gains["ki"]))
            if phase == "p":
                metrics = {"score": 0.0, "mean_steps": [
                    {"speed": 0.15, "direction": direction, "arrived": True,
                     "arrival_time": self.config["velocity_p_targets"][0]["arrival_seconds"]}
                    for direction in ("positive", "negative")]}
            elif phase == "d":
                metrics = {"score": 0.1, "mean_steps": [
                    {"direction": direction, "cycle": "mean", "settled": True,
                     "settling_time": 1.0, "plateau_fraction": 1.0}
                    for direction in ("positive", "negative")]}
            else:
                metrics = {"score": 0.1, "terminal_mae": 0.0,
                           "oscillation_rms": 0.0,
                           "i_mean_response_by_direction": {
                               direction: {"score_window_valid": True,
                                           "hold_fraction": 1.0}
                               for direction in ("positive", "negative")}}
            return {"label": label, "gains": gains, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_velocity_pid_cycle_repeated(
            "pid_vel_pitch", "x"))
        self.assertEqual(calls, [("p", 0.0, 0.0),
                                 ("d", 0.0, 0.0),
                                 ("i", 0.0, 0.0)])
        self.assertEqual(runner.best["pid_vel_pitch"]["kd"], 0.0)
        self.assertEqual(runner.best["pid_vel_pitch"]["ki"], 0.0)

    def test_velocity_transfer_measures_y_d_before_scoring_y_i(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            calls.append((kwargs["velocity_phase"],
                          kwargs.get("velocity_i_score_start")))
            if kwargs["velocity_phase"] == "p":
                metrics = {"score": 0.1, "mean_steps": [
                    {"speed": plan["speed"], "direction": direction,
                     "arrived": True,
                     "arrival_time": plan["arrival_seconds"]}
                    for plan in self.config["velocity_p_targets"]
                    for direction in ("positive", "negative")]}
            elif kwargs["velocity_phase"] == "d":
                metrics = {"score": 1.0, "mean_steps": [
                    {"direction": direction, "cycle": cycle, "settled": True,
                     "settling_time": 2.0, "plateau_fraction": 1.0}
                    for direction in ("positive", "negative")
                    for cycle in ("mean",)]}
            else:
                metrics = {"phase": "i", "score": 0.1,
                           "i_mean_response_by_direction": {
                               direction: {"score_window_valid": True,
                                           "hold_fraction": 0.5}
                               for direction in ("positive", "negative")}}
            return {"label": label, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        runner._tune_velocity_pid = lambda *args, **kwargs: self.fail(
            "Y should accept the transferred gains")
        self.assertTrue(runner._transfer_x_gains_to_y(
            "velocity", "pid_vel_pitch", "pid_vel_roll"))
        self.assertEqual(calls, [("p", None), ("d", None), ("i", 2.0)])
        self.assertEqual(runner.stage_summary["pid_vel_roll"][
            "d_settling_reference_seconds"], 2.0)
        self.assertTrue(runner.stage_summary["pid_vel_roll"]["p_criteria_met"])
        self.assertTrue(runner.stage_summary["pid_vel_roll"]["d_criteria_met"])
        self.assertFalse(runner.stage_summary["pid_vel_roll"]["i_criteria_met"])

    def test_failed_velocity_transfer_starts_roll_without_pitch_gains(self):
        for failing_phase in ("p", "d"):
            with self.subTest(failing_phase=failing_phase):
                runner = CalibrationRunner(self.config)
                runner.best["pid_vel_pitch"].update(
                    kp=4.2, ki=0.02, kd=1.7)
                original_roll = runner.best["pid_vel_roll"].copy()
                seen = {}

                def evaluate(stage, name, axis, candidate, label, **kwargs):
                    phase = kwargs["velocity_phase"]
                    if phase == "p":
                        metrics = {"score": 1.0, "mean_steps": [
                            {"speed": plan["speed"], "direction": direction,
                             "arrived": True,
                             "arrival_time": plan["arrival_seconds"] +
                             (1.0 if failing_phase == "p" else 0.0)}
                            for plan in self.config["velocity_p_targets"]
                            for direction in ("positive", "negative")]}
                    else:
                        metrics = {"score": 1.0, "mean_steps": [
                            {"direction": direction, "settled": False,
                             "settling_time": 7.0, "plateau_fraction": 0.0}
                            for direction in ("positive", "negative")]}
                    return {"label": label, "metrics": metrics,
                            "telemetry_csv": "transfer.csv"}

                def tune(name, axis, *, initial_source, **kwargs):
                    seen.update(name=name, axis=axis,
                                initial_source=initial_source,
                                roll=runner.best[name].copy(),
                                pitch=runner.best["pid_vel_pitch"].copy())
                    return True

                runner._evaluate = evaluate
                runner._tune_velocity_pid = tune
                self.assertTrue(runner._transfer_x_gains_to_y(
                    "velocity", "pid_vel_pitch", "pid_vel_roll"))
                self.assertEqual(seen["initial_source"], "zero_gains_start")
                self.assertEqual([seen["roll"][key]
                                  for key in ("kp", "ki", "kd")],
                                 [0.0, 0.0, 0.0])
                self.assertEqual(seen["roll"]["i_limit"],
                                 original_roll["i_limit"])
                self.assertEqual(seen["pitch"]["kp"], 4.2)

    def test_roll_zero_start_advances_to_small_p_after_zero_probe(self):
        config = default_config()
        config["velocity_trials_per_coefficient"] = 2
        runner = CalibrationRunner(config)
        runner._save_summary = lambda: None
        runner.best["pid_vel_roll"].update(kp=0.0, ki=0.0, kd=0.0)
        tried_p = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["velocity_phase"]
            if phase == "p":
                kp = candidate[name]["kp"]
                tried_p.append(kp)
                metrics = {"score": 3.0 if kp == 0 else 0.0,
                           "mean_steps": [
                               {"speed": 0.15, "direction": direction,
                                "arrived": kp > 0,
                                "arrival_time": 1.0 if kp > 0 else None}
                               for direction in ("positive", "negative")]}
            elif phase == "d":
                metrics = {"score": 0.0, "mean_steps": [
                    {"direction": direction, "cycle": "mean",
                     "settled": True, "settling_time": 1.0,
                     "plateau_fraction": 1.0}
                    for direction in ("positive", "negative")]}
            else:
                metrics = {"score": 0.0, "i_mean_response_by_direction": {
                    direction: {"score_window_valid": True,
                                "hold_fraction": 1.0}
                    for direction in ("positive", "negative")}}
            return {"label": label, "gains": candidate[name].copy(),
                    "metrics": metrics, "telemetry_csv": "trial.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_velocity_pid_cycle_repeated(
            "pid_vel_roll", "y", initial_source="zero_gains_start"))
        self.assertEqual(tried_p, [0.0, config["velocity_p_start"]])

    def test_repeated_velocity_cycle_needs_p_arrival_as_well_as_d_and_i(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        self.config["velocity_trials_per_coefficient"] = 1

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["velocity_phase"]
            if phase == "p":
                steps = [{"speed": plan["speed"], "direction": direction,
                          "arrived": True,
                          "arrival_time": plan["arrival_seconds"] + 1.0}
                         for plan in self.config["velocity_p_targets"]
                         for direction in ("positive", "negative")
                         for _ in range(3)]
                metrics = {"score": 1.0, "repeat_steps": steps,
                           "mean_steps": [
                               {"speed": plan["speed"], "direction": direction,
                                "cycle": "mean", "arrived": True,
                                "arrival_time": plan["arrival_seconds"] + 1.0}
                               for plan in self.config["velocity_p_targets"]
                               for direction in ("positive", "negative")]}
            elif phase == "d":
                metrics = {"score": 1.0, "mean_steps": [
                    {"direction": direction, "settled": True,
                     "cycle": "mean",
                     "settling_time": 1.0, "plateau_fraction": 1.0}
                    for direction in ("positive", "negative")]}
            else:
                metrics = {"score": 1.0, "i_mean_response_by_direction": {
                    direction: {"score_window_valid": True,
                                "hold_fraction": 1.0}
                    for direction in ("positive", "negative")}}
            return {"label": label, "gains": candidate[name].copy(),
                    "metrics": metrics, "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        self.assertFalse(runner._tune_velocity_pid_cycle_repeated(
            "pid_vel_pitch", "x"))
        self.assertFalse(runner.stage_summary["pid_vel_pitch"]["p_criteria_met"])
        self.assertFalse(runner.stage_summary["pid_vel_pitch"]["stable"])

    def test_repeated_velocity_stops_after_p_and_d_even_if_i_misses_hold(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        self.config["velocity_trials_per_coefficient"] = 1
        phases = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            phase = kwargs["velocity_phase"]
            phases.append(phase)
            if phase == "p":
                metrics = {"score": 0.0, "mean_steps": [
                    {"speed": 0.15, "direction": direction, "arrived": True,
                     "arrival_time": 1.0}
                    for direction in ("positive", "negative")]}
            elif phase == "d":
                metrics = {"score": 0.1, "mean_steps": [
                    {"direction": direction, "cycle": "mean", "settled": True,
                     "settling_time": 1.0, "plateau_fraction": 1.0}
                    for direction in ("positive", "negative")]}
            else:
                metrics = {"score": 1.0, "repeat_steps": [],
                           "reached_both": True,
                           "plateau_settled_fraction": 1.0,
                           "oscillation_rms": 0.0,
                           "i_mean_response_by_direction": {
                               direction: {"score_window_valid": True,
                                           "hold_fraction": 0.5}
                               for direction in ("positive", "negative")}}
            return {"label": label, "gains": candidate[name].copy(),
                    "metrics": metrics, "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        self.assertTrue(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        summary = runner.stage_summary["pid_vel_pitch"]
        self.assertEqual(phases, ["p", "d", "i"])
        self.assertEqual(summary["completed_tuning_cycles"], 1)
        self.assertTrue(summary["p_criteria_met"])
        self.assertTrue(summary["d_criteria_met"])
        self.assertFalse(summary["i_criteria_met"])
        self.assertTrue(summary["stable"])

    def test_velocity_does_not_repeat_p_d_i_when_criteria_fail(self):
        config = default_config()
        config["velocity_tuning_cycles"] = 5  # Old config value cannot revive repeats.
        runner = CalibrationRunner(config)
        runner._save_summary = lambda: None
        calls = []

        def tune(name, axis, **kwargs):
            calls.append((name, axis))
            runner.stage_summary[name] = {
                "stage": "velocity", "stable": False,
                "best_metrics": {"score": 4.0, "repeat_steps": [],
                                 "reached_both": False,
                                 "plateau_settled_fraction": 0.0,
                                 "oscillation_rms": 0.1}}
            return False

        runner._tune_velocity_pid_cycle_repeated = tune
        self.assertFalse(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        self.assertEqual(calls, [("pid_vel_pitch", "x")])
        self.assertEqual(runner.stage_summary["pid_vel_pitch"][
            "completed_tuning_cycles"], 1)
        self.assertEqual(runner.stage_summary["pid_vel_pitch"][
            "tuning_cycle_limit"], 1)

    def test_velocity_target_corridor_requires_sustained_tracking(self):
        speed = self.config["velocity_target_speed"]

        def flight(slow=False, failed_preparation=False):
            rows = []
            x = 0.0
            for direction, sign in (("positive", 1), ("negative", -1)):
                for _ in range(50):
                    rows.append({
                        "segment": f"pre_{direction}",
                        "segment_seconds": 1.0, "t": len(rows) * 0.02,
                        "x": x, "y": 0.0, "yaw": 0.0, "saturated": 0,
                        "preparation_rest": not (failed_preparation and
                                                  direction == "negative"),
                    })
                for label, duration in ((direction, 5.0), (f"{direction}_stop", 4.0)):
                    for tick in range(int(duration / 0.02)):
                        elapsed = tick * 0.02
                        if label.endswith("stop"):
                            velocity = sign * speed * math.exp(-elapsed / 0.15)
                        else:
                            tau = 20 if slow else 0.55
                            velocity = sign * speed * (1 - math.exp(-elapsed / tau))
                        x += velocity * 0.02
                        rows.append({
                            "segment": label, "segment_seconds": duration,
                            "t": len(rows) * 0.02,
                            "x": x, "y": 0.0, "yaw": 0.0, "saturated": 0,
                        })
            metrics = score_velocity_steps(rows, "x", speed, self.config)
            metrics["preparations_stable"] = not failed_preparation
            return metrics

        good = flight()
        bad = flight(slow=True)
        unsteady_start = flight(failed_preparation=True)
        self.assertTrue(_velocity_response_stable(good, speed, self.config))
        self.assertFalse(_velocity_response_stable(bad, speed, self.config))
        self.assertFalse(_velocity_response_stable(unsteady_start, speed, self.config))
        self.assertLess(good["score"], bad["score"])

    def test_velocity_score_uses_increment_from_frozen_drift_baseline(self):
        rows = []
        clock = 0.0
        x = 0.0
        cases = (("positive", -0.20, 0.10),
                 ("negative", 0.15, -0.10))
        for label, baseline, delta in cases:
            for index in range(50):
                x += baseline * 0.02
                rows.append({
                    "segment": f"pre_{label}", "segment_seconds": 1.0,
                    "t": clock, "x": x, "y": 0.0, "yaw": 0.0,
                    "saturated": 0, "preparation_stable": True,
                })
                clock += 0.02
            for index in range(250):
                elapsed = index * 0.02
                velocity = baseline + delta * (1 - math.exp(-elapsed / 0.55))
                x += velocity * 0.02
                rows.append({
                    "segment": label, "segment_seconds": 5.0,
                    "t": clock, "x": x, "y": 0.0, "yaw": 0.0,
                    "saturated": 0, "baseline_axis_speed": baseline,
                    "commanded_axis_speed": baseline + delta,
                    "requested_delta_speed": delta,
                })
                clock += 0.02
            start_speed = baseline + delta
            for index in range(200):
                elapsed = index * 0.02
                velocity = start_speed * math.exp(-elapsed / 0.15)
                x += velocity * 0.02
                rows.append({
                    "segment": f"{label}_stop", "segment_seconds": 4.0,
                    "t": clock, "x": x, "y": 0.0, "yaw": 0.0,
                    "saturated": 0,
                })
                clock += 0.02
        metrics = score_velocity_steps(rows, "x", 0.10, self.config)
        diagonal_rows = [dict(row, y=50.0 * row["t"]) for row in rows]
        diagonal_metrics = score_velocity_steps(
            diagonal_rows, "x", 0.10, self.config)
        self.assertTrue(metrics["reached_both"])
        self.assertAlmostEqual(metrics["positive"]["baseline_axis_speed"], -0.20)
        self.assertAlmostEqual(metrics["negative"]["baseline_axis_speed"], 0.15)
        self.assertGreater(metrics["positive"]["terminal_progress"], 0.09)
        self.assertGreater(metrics["negative"]["terminal_progress"], 0.09)
        # A free, uncontrolled transverse drift is diagnostic data only.  The
        # X score must be derived strictly from X velocity.
        self.assertAlmostEqual(metrics["score"], diagonal_metrics["score"])

    def test_velocity_profile_commands_relative_step_toward_zero_drift(self):
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "trial.csv"
        clock = 0.0
        x = 0.0

        def wait(axis, rows, label):
            nonlocal clock, x
            for _ in range(20):
                x += -0.20 * 0.1
                rows.append({"segment": label, "t": clock, "x": x,
                             "y": 0.0, "yaw": 0.0})
                clock += 0.1
            return False

        commands = []

        def segment(stage, target, duration, label, rows, **kwargs):
            commands.append((label, target["velocity"]))
            rows.append({"segment": label})

        runner._wait_velocity_rest = wait
        runner._segment = segment
        rows, _ = runner._profile_velocity_range("x", "relative")
        moving = [item for item in commands if item[0] in ("positive", "negative")]
        self.assertAlmostEqual(moving[0][1][0], -0.20 + self.config["velocity_target_speed"])
        self.assertAlmostEqual(moving[1][1][0], -self.config["velocity_target_speed"])
        self.assertEqual([item[0] for item in commands],
                         ["positive", "positive_stop", "negative", "negative_stop"])
        self.assertAlmostEqual(commands[0][1][0], -0.05)
        self.assertAlmostEqual(commands[2][1][0], -0.15)
        step_rows = [row for row in rows if row["segment"] == "positive"]
        self.assertAlmostEqual(step_rows[0]["baseline_axis_speed"], -0.2,
                               places=2)
        self.assertAlmostEqual(step_rows[0]["requested_delta_speed"], 0.15,
                               places=2)

    def test_velocity_p_profile_commands_absolute_target_speed(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "trial.csv"
        runner._brake_velocity_vector = lambda label, rows, **kwargs: True
        runner._velocity_preparation_state = lambda *args, **kwargs: {
            "at_rest": True, "stable": True, "measured": True,
            "speed": -0.02, "acceleration": 0.0, "spread": 0.0}
        commands = []
        runner._segment = lambda stage, target, duration, label, rows, **kwargs: (
            commands.append((label, target["velocity"])),
            rows.append({"segment": label}))
        rows, _ = runner._profile_velocity_range("x", "pure_p", p_mode=True)
        self.assertEqual([value[1][0] for value in commands
                          if value[0] in ("positive", "negative")], [0.15, -0.15])
        positive = next(row for row in rows if row["segment"] == "positive")
        self.assertAlmostEqual(positive["requested_delta_speed"], 0.17)

    def test_service_brake_stops_at_longitudinal_zero_despite_cross_drift(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        outputs = []

        def segment(stage, target, duration, label, rows, *, rc_override, **kwargs):
            # The first two samples capture +X as the braking vector.  At the
            # third X has stopped, while a lateral Y drift still remains.
            for elapsed, sample in (
                    (0.0, Sample(0.0, 0.0, 0.0, 1.0, 0.0)),
                    (0.2, Sample(0.2, 0.02, 0.0, 1.0, 0.0)),
                    (0.4, Sample(0.4, 0.021, 0.03, 1.0, 0.0))):
                outputs.append(rc_override(sample, elapsed))

        runner._segment = segment
        self.assertTrue(runner._brake_velocity_vector("test_brake", []))
        stopped = outputs[-1]
        self.assertEqual((stopped["rc_roll"], stopped["rc_pitch"]), (1500, 1500))
        self.assertEqual(stopped["brake_stop_reason"], "longitudinal_stop")
        self.assertGreater(stopped["brake_speed"],
                           self.config["velocity_p_brake_stop_speed"])

    def test_zeroing_service_brake_continues_below_normal_stop_speed(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        outputs = []

        def segment(stage, target, duration, label, rows, *, rc_override, **kwargs):
            for elapsed, speed in ((0.0, 0.03), (0.2, 0.025),
                                   (0.4, -0.005)):
                sample = Sample(elapsed, 0.0, 0.0, 1.0, 0.0,
                                vx_world=speed, vy_world=0.0, vz_world=0.0)
                outputs.append(rc_override(sample, elapsed))

        runner._segment = segment
        self.assertTrue(runner._brake_velocity_vector(
            "pre_positive_axis_zero_brake", [], active_axis="x",
            zero_cross_sign=1))
        self.assertEqual(outputs[-1]["brake_stop_reason"], "axis_zero_cross")
        self.assertEqual((outputs[-1]["rc_roll"], outputs[-1]["rc_pitch"]),
                         (1500, 1500))

    def test_staged_xy_does_not_require_old_launch_zone(self):
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.launch_xy = (-3.0, -12.0)
        runner.last_sample = Sample(0, 0, 0, 1, 0)
        runner._ensure_xy_zone("velocity")

    def test_xy_recovery_does_not_invoke_stepwise_height_descent(self):
        self.config["hover_height"] = 9.0
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.anchor = runner.last_sample = Sample(0.0, 0.0, 0.0, 9.3, 0.0)
        runner.controller.z_bias = 0.0
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "recovery.csv"
        runner._descend_height = lambda *args, **kwargs: self.fail(
            "XY recovery must not create height descent steps")
        seen = []

        def segment(stage, target, duration, label, rows, **kwargs):
            seen.append((stage, target["height"], target["height_relative"], label))
            runner.last_sample = Sample(1.0, 0.0, 0.0, 9.0, 0.0)
            rows.extend((
                {"t": 0.0, "z": 9.0, "yaw": 0.0, "target_z": 9.0,
                 "rc_throttle": 1620},
                {"t": 1.0, "z": 9.0, "yaw": 0.0, "target_z": 9.0,
                 "rc_throttle": 1620},
            ))

        runner._segment = segment
        runner._recover("velocity")
        self.assertEqual(seen, [("yaw", 9.0, True, "recovery")])

    def test_acceleration_recovery_uses_preflight_heading(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = Sample(0.0, 0.0, 0.0, 1.0, 0.0)
        runner.xy_reference_yaw = 0.4
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "recovery.csv"
        targets = []

        def segment(stage, target, duration, label, rows, **kwargs):
            targets.append((stage, target["yaw"]))
            rows.append({"t": 0.0, "z": target["height"],
                         "yaw": target["yaw"], "target_z": target["height"],
                         "rc_throttle": 1500})

        runner._segment = segment
        runner._recover("acceleration")
        self.assertEqual(targets[0][0], "yaw")
        self.assertAlmostEqual(targets[0][1], 0.4)

    def test_failed_axis_rest_is_scored_instead_of_invalidating_trial(self):
        config = default_config()
        config["velocity_rest_seconds"] = 0.01
        config["velocity_rest_timeout_seconds"] = 0.02
        runner = CalibrationRunner(config)
        runner.xy_waypoints_mode = True
        runner.anchor = runner.last_sample = self.sample
        runner._segment = lambda stage, target, seconds, label, rows, **kwargs: rows.append({
            "segment": label, "t": time.monotonic(), "x": 0.0, "y": 0.0,
            "yaw": 0.0})
        self.assertFalse(runner._wait_velocity_rest("x", [], "pre_positive"))

    def test_position_waypoints_penalize_overshoot_after_arrival(self):
        def flight(overshoot=0.0):
            rows = []
            actual = 0.0
            for index, target in enumerate((0.5, 0.0, -0.5, 0.0), 1):
                start = actual
                for tick in range(61):
                    actual = start + (target - start) * min(1, tick / 20)
                    if index == 1 and 25 <= tick <= 30:
                        actual += overshoot
                    rows.append({
                        "waypoint_index": index, "requested_distance": 0.5,
                        "waypoint_return": index % 2 == 0,
                        "segment": f"position_wp_{index:02d}_hold",
                        "t": len(rows) * 0.1, "x": actual, "y": 0.0,
                        "target_x": target, "target_z": 1.0, "z": 1.0,
                        "saturated": 0,
                    })
            return score_position_waypoints(rows, "x", self.config)

        good = flight()
        bad = flight(0.20)
        self.assertTrue(_position_waypoints_stable(good, self.config))
        self.assertFalse(_position_waypoints_stable(bad, self.config))
        self.assertLess(good["score"], bad["score"])

    def test_xy_only_orders_isolated_then_joint_then_confirmation(self):
        self.config["velocity_repeated_mode"] = False
        class Link:
            def arm(self):
                pass

            def disarm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, link=Link())
        runner._prepare_height_ascent = lambda: None
        runner.anchor = runner.initial_sample = runner.last_sample = self.sample
        runner._takeoff = lambda: None
        runner._save_summary = lambda: None
        order = []

        def tune(stage, name, axis, **kwargs):
            order.append((stage, name, axis))
            self.assertTrue(runner.xy_waypoints_mode)
            self.assertFalse(runner.yaw_waypoints_mode)
            if axis == "x":
                runner.best[name]["kp"] = 2.25
            runner.stage_summary[name] = {"stable": True}
            return True

        def joint(stage, names):
            order.append((f"joint_{stage}", *names))
            runner.stage_summary[f"joint_{stage}"] = {"stable": True}
            return True

        def confirm(stage, name, axis, candidate, label):
            order.append(("confirm", name, axis))
            direction = {"rise_time": 1.5}
            return {"metrics": {
                "score": 1.0, "oscillation_rms": 0.0,
                "terminal_mae": 0.001, "reached_both": True,
                "stopped_both": True, "preparations_stable": True,
                "in_band_fraction": 0.8, "tail_in_band_fraction": 0.9,
                "p95_acceleration": 0.2, "p95_jerk": 1.0,
                "positive": dict(direction), "negative": dict(direction),
            }}

        runner._tune_pid = tune
        runner._tune_velocity_pid = lambda name, axis, **kwargs: tune(
            "velocity", name, axis, **kwargs)
        runner._tune_joint = joint
        runner._evaluate = confirm
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_xy_only("velocity")
        self.assertEqual(order, [
            ("velocity", "pid_vel_pitch", "x"),
            ("confirm", "pid_vel_roll", "y"),
            ("joint_velocity", "pid_vel_pitch", "pid_vel_roll"),
            ("confirm", "pid_vel_pitch", "x"),
            ("confirm", "pid_vel_roll", "y"),
        ])
        self.assertEqual(runner.best["pid_vel_roll"]["kp"], 2.25)
        self.assertTrue(runner.stage_summary["pid_vel_roll"]["transfer_accepted"])
        self.assertEqual(runner.status, "complete")

    def test_velocity_only_validates_both_axes_after_one_failed_search_each(self):
        class Link:
            def arm(self):
                pass

            def disarm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, link=Link())
        runner._prepare_height_ascent = lambda: None
        runner.anchor = runner.initial_sample = runner.last_sample = self.sample
        runner._takeoff = lambda: None
        runner._save_summary = lambda: None
        trials = []

        def tune(name, axis, **kwargs):
            trials.append(("tune", axis, kwargs.get("initial_source")))
            if axis == "y":
                self.assertEqual([runner.best[name][key]
                                  for key in ("kp", "ki", "kd")],
                                 [0.0, 0.0, 0.0])
            runner.stage_summary[name] = {
                "stage": "velocity", "stable": False,
                "best_metrics": {"score": 2.0},
                "p_criteria_met": False, "d_criteria_met": True}
            return False

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            trials.append(("validation", axis, kwargs["velocity_magnitude"]))
            return {"metrics": {"validation_passed": False},
                    "telemetry_csv": f"{label}.csv"}

        runner._tune_velocity_pid = tune
        runner._evaluate = evaluate
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_xy_only("velocity")
        self.assertEqual(trials, [
            ("tune", "x", None),
            *(("validation", "x", speed)
              for speed in self.config["velocity_validation_speeds"]),
            ("tune", "y", "zero_gains_start"),
            *(("validation", "y", speed)
              for speed in self.config["velocity_validation_speeds"]),
        ])
        self.assertEqual(len(runner.stage_summary["pid_vel_pitch"][
            "speed_validation"]), 3)
        self.assertEqual(len(runner.stage_summary["pid_vel_roll"][
            "speed_validation"]), 3)
        self.assertEqual(runner.status, "needs_review")

    def test_failed_x_to_y_transfer_becomes_position_y_baseline(self):
        self.config["position_repeated_mode"] = False
        runner = CalibrationRunner(self.config)
        runner.xy_waypoints_mode = True
        runner.best["pid_pos_x"].update(kp=1.2, ki=0.03, kd=2.4, i_limit=0.07)
        transfer = {"metrics": {
            "score": 9.0, "waypoints_reached_fraction": 0.5,
            "max_overshoot": 0.0, "oscillation_rms": 0.0,
            "waypoints": [{"settling_time": None}],
        }}
        runner._evaluate = lambda *args, **kwargs: transfer
        seen = {}

        def tune(stage, name, axis, **kwargs):
            seen.update(stage=stage, name=name, axis=axis, **kwargs)
            return True

        runner._tune_pid = tune
        self.assertTrue(runner._transfer_x_gains_to_y(
            "position", "pid_pos_x", "pid_pos_y"))
        self.assertEqual(
            {k: runner.best["pid_pos_y"][k] for k in ("kp", "ki", "kd", "i_limit")},
            {"kp": 1.2, "ki": 0.03, "kd": 2.4, "i_limit": 0.07})
        self.assertIs(seen["baseline"], transfer)
        self.assertEqual(seen["initial_source"], "pid_pos_x_gains")

    def test_position_only_validates_both_axes_after_one_failed_pass(self):
        class Link:
            def arm(self):
                pass

            def disarm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, link=Link())
        runner._prepare_height_ascent = lambda: None
        runner.anchor = runner.initial_sample = runner.last_sample = self.sample
        runner._takeoff = lambda: None
        runner._save_summary = lambda: None
        calls = []

        def tune(name, axis, **kwargs):
            calls.append(("tune", axis, kwargs.get("initial_source")))
            if axis == "y":
                self.assertEqual([runner.best[name][key]
                                  for key in ("kp", "ki", "kd")],
                                 [0.0, 0.0, 0.0])
            runner.stage_summary[name] = {
                "stage": "position", "stable": False,
                "best_metrics": {"score": 2.0}}
            return False

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            calls.append(("validation", axis, kwargs["position_distance"]))
            return {"metrics": {"validation_passed": False},
                    "telemetry_csv": f"{label}.csv"}

        runner._tune_position_pid = tune
        runner._evaluate = evaluate
        runner._tune_joint = lambda *args: self.fail("No joint position search")
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_xy_only("position")
        self.assertEqual(calls, [
            ("tune", "x", None),
            *(("validation", "x", d) for d in
              self.config["position_validation_distances"]),
            ("tune", "y", "zero_gains_start"),
            *(("validation", "y", d) for d in
              self.config["position_validation_distances"]),
        ])
        self.assertEqual(runner.status, "needs_review")

    def test_failed_repeated_position_transfer_starts_y_from_zero(self):
        runner = CalibrationRunner(self.config)
        runner.best["pid_pos_x"].update(kp=1.2, kd=0.3, ki=0.02)
        runner._evaluate = lambda *args, **kwargs: {
            "metrics": {"mean_steps": [], "score": 3.0}}
        seen = {}

        def tune(name, axis, **kwargs):
            seen.update(name=name, axis=axis, source=kwargs["initial_source"],
                        gains={key: runner.best[name][key]
                               for key in ("kp", "kd", "ki")})
            return True

        runner._tune_position_pid = tune
        self.assertTrue(runner._transfer_x_gains_to_y(
            "position", "pid_pos_x", "pid_pos_y"))
        self.assertEqual(seen["gains"], {"kp": 0.0, "kd": 0.0, "ki": 0.0})
        self.assertEqual(seen["source"], "zero_gains_start")

    def test_resume_yaw_and_velocity_use_previous_numeric_gains(self):
        with tempfile.TemporaryDirectory() as directory:
            source = CalibrationRunner(self.config)
            source.run_dir = Path(directory)
            source.best["pid_yaw"]["kp"] = 10.4
            source.best["pid_vel_pitch"]["kp"] = 2.7
            source.best["pid_vel_roll"]["kd"] = 3.2
            source._save_summary()
            resumed = CalibrationRunner(self.config)
            resumed.resume_yaw_from(Path(directory))
            resumed.resume_velocity_from(Path(directory))
            self.assertEqual(resumed.best["pid_yaw"]["kp"], 10.4)
            self.assertEqual(resumed.best["pid_vel_pitch"]["kp"], 2.7)
            self.assertEqual(resumed.best["pid_vel_roll"]["kd"], 3.2)
            self.assertEqual(resumed.best["pid_yaw"]["max_control"],
                             source.best["pid_yaw"]["max_control"])

    def test_velocity_tuner_runs_ordered_p_d_and_fine_phases_with_budget(self):
        self.config["velocity_repeated_mode"] = False
        runner = CalibrationRunner(self.config)
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            gains = candidate[name]
            reached = gains["kp"] >= 1.0
            damped = gains["kd"] >= self.config["velocity_d_start"]
            rise = 1.0 if reached else 5.0
            direction = {
                # P may reach in time with a large peak; D is responsible
                # for suppressing that peak and increasing the hold fraction.
                "rise_time": rise, "max_speed": 0.20 if reached else 0.06,
                "in_band_fraction": 0.8 if damped else 0.3,
                "tail_in_band_fraction": 0.9 if damped else 0.4,
            }
            metrics = {
                "score": 1.0 if damped else 4.0,
                "oscillation_rms": 0.005, "terminal_mae": 0.002,
                "terminal_tracking_bias": 0.002,
                "reached_both": reached, "settled_both": damped,
                "stopped_both": True, "preparations_stable": True,
                "in_band_fraction": direction["in_band_fraction"],
                "tail_in_band_fraction": direction["tail_in_band_fraction"],
                "p95_acceleration": 0.2, "p95_jerk": 1.0,
                "positive": dict(direction), "negative": dict(direction),
            }
            calls.append((label, gains.copy()))
            return {"stage": stage, "pid": name, "axis": axis,
                    "label": label, "gains": gains.copy(), "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        runner._save_summary = lambda: None
        self.assertTrue(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        summary = runner.stage_summary["pid_vel_pitch"]
        self.assertTrue(summary["p_criteria_met"])
        self.assertTrue(summary["d_criteria_met"])
        self.assertTrue(summary["repeatable"])
        self.assertLessEqual(summary["trial_count"], 20)
        self.assertEqual(calls[0][1]["ki"], 0.0)
        self.assertEqual(calls[0][1]["kd"], 0.0)
        self.assertTrue(any("_d_" in label for label, _ in calls))

    def test_velocity_tuner_uses_full_budget_when_p_never_reaches_target(self):
        config = default_config()
        config["velocity_repeated_mode"] = False
        config["velocity_trials_per_coefficient"] = 6
        config["velocity_d_trials"] = 1
        config["velocity_i_trials"] = 1
        runner = CalibrationRunner(config)
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            direction = {
                "rise_time": config["velocity_trial_seconds"],
                "max_speed": 0.06,
                "terminal_tracking_bias": 0.04,
            }
            metrics = {
                "score": 8.0, "oscillation_rms": 0.002,
                "terminal_mae": 0.04, "terminal_tracking_bias": 0.04,
                "reached_both": False, "settled_both": False,
                "stopped_both": True, "preparations_stable": True,
                "in_band_fraction": 0.0, "tail_in_band_fraction": 0.0,
                "p95_acceleration": 0.1, "p95_jerk": 0.5,
                "positive": dict(direction), "negative": dict(direction),
            }
            gains = candidate[name].copy()
            calls.append((label, gains))
            return {"stage": stage, "pid": name, "axis": axis,
                    "label": label, "gains": gains, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        runner._save_summary = lambda: None
        self.assertFalse(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        summary = runner.stage_summary["pid_vel_pitch"]
        self.assertFalse(summary["p_criteria_met"])
        self.assertEqual(summary["trial_count"], 10)
        self.assertEqual(len(calls), 10)
        self.assertTrue(calls[-1][0].endswith("_repeat"))
        self.assertTrue(any("_d_" in label for label, _ in calls))
        self.assertTrue(any("_i_" in label for label, _ in calls))
        p_values = [gains["kp"] for label, gains in calls if "_p_" in label]
        self.assertTrue(all(left < right for left, right in zip(
            p_values, p_values[1:])))

    def test_velocity_p_search_ignores_hold_metrics_until_first_arrival(self):
        config = default_config()
        config["velocity_repeated_mode"] = False
        config["velocity_trials_per_coefficient"] = 6
        runner = CalibrationRunner(config)
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            kp = candidate[name]["kp"]
            runaway = kp >= 1.0
            direction = {
                "rise_time": config["velocity_trial_seconds"],
                "max_speed": 0.20 if runaway else 0.05,
                "terminal_tracking_bias": 0.05,
            }
            metrics = {
                "score": 20.0 if runaway else 8.0,
                "oscillation_rms": 0.20 if runaway else 0.002,
                "terminal_mae": 0.10 if runaway else 0.05,
                "terminal_tracking_bias": 0.05,
                "reached_both": False, "settled_both": False,
                "stopped_both": True, "preparations_stable": True,
                "in_band_fraction": 0.0, "tail_in_band_fraction": 0.0,
                "p95_acceleration": 1.0 if runaway else 0.1,
                "p95_jerk": 4.0 if runaway else 0.5,
                "positive": dict(direction), "negative": dict(direction),
            }
            gains = candidate[name].copy()
            calls.append((label, gains))
            return {"stage": stage, "pid": name, "axis": axis,
                    "label": label, "gains": gains, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        runner._save_summary = lambda: None
        self.assertFalse(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        p_values = [gains["kp"] for label, gains in calls if "_p_" in label]
        self.assertTrue(all(left < right for left, right in zip(
            p_values, p_values[1:])))

    def test_velocity_i_stage_keeps_selected_p_and_d_fixed(self):
        config = default_config()
        config["velocity_repeated_mode"] = False
        config.update(
            velocity_trials_per_coefficient=8,
            velocity_d_trials=1,
            velocity_i_trials=2,
            velocity_fine_trials=0,
        )
        runner = CalibrationRunner(config)
        calls = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            gains = candidate[name].copy()
            has_d = gains["kd"] >= config["velocity_d_start"]
            has_i = gains["ki"] > 0.0
            d_hold = 0.8 if has_d else 0.5
            i_hold = 0.8 if has_i else 0.6
            direction = {
                "rise_time": 1.5, "arrival_time": 1.5, "arrived": True,
                "max_speed": 0.1, "terminal_mae": 0.002,
                "d_hold_fraction_after_arrival": d_hold,
                "i_hold_fraction_after_arrival": i_hold,
                "d_brake_reached": True, "d_brake_reversed": False,
            }
            metrics = {
                "score": 1.0 if has_i else 2.0,
                "oscillation_rms": 0.005, "terminal_mae": 0.002,
                "terminal_tracking_bias": 0.002,
                "reached_both": True, "stopped_both": True,
                "in_band_fraction": d_hold, "tail_in_band_fraction": d_hold,
                "p95_acceleration": 0.2, "p95_jerk": 1.0,
                "positive": dict(direction), "negative": dict(direction),
            }
            calls.append((label, gains))
            return {"stage": stage, "pid": name, "axis": axis,
                    "label": label, "gains": gains, "metrics": metrics,
                    "telemetry_csv": f"{label}.csv"}

        runner._evaluate = evaluate
        runner._save_summary = lambda: None
        self.assertTrue(runner._tune_velocity_pid("pid_vel_pitch", "x"))
        d_gains = next(gains for label, gains in calls
                       if "_d_" in label and not label.endswith("_d_base"))
        i_gains = [gains for label, gains in calls if "_i_" in label]
        self.assertTrue(i_gains)
        self.assertTrue(all(gains["kp"] == d_gains["kp"] and
                            gains["kd"] == d_gains["kd"] for gains in i_gains))
        self.assertTrue(runner.stage_summary["pid_vel_pitch"]["i_needed"])
        self.assertTrue(runner.stage_summary["pid_vel_pitch"]["i_criteria_met"])

    def test_velocity_stage_uses_inner_pid_without_position_pid(self):
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "velocity", self.target)
        self.assertNotEqual(output["rc_pitch"], 1500)
        self.assertEqual(output["pid_pos_x_error"], 0.0)
        self.assertEqual(output["pid_pos_y_error"], 0.0)

    def test_acceleration_stage_uses_only_selected_inner_axis(self):
        target = {**self.target, "acceleration": (0.15, 0.0), "active_axis": "x"}
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "acceleration", target)
        self.assertAlmostEqual(output["pid_accel_pitch_error"], 0.15)
        self.assertEqual(output["pid_accel_roll_error"], 0.0)
        self.assertEqual(output["pid_vel_pitch_error"], 0.0)
        self.assertEqual(output["pid_pos_x_error"], 0.0)
        self.assertNotEqual(output["rc_pitch"], 1500)
        self.assertEqual(output["rc_roll"], 1500)

    def test_duplicate_kinematics_window_flags_two_identical_states(self):
        same = Sample(0.0, 1.0, 2.0, 3.0, 0.4, 0.1, -0.1)
        other = Sample(0.02, 1.01, 2.0, 3.0, 0.4, 0.1, -0.1)
        self.assertTrue(_duplicate_kinematics_window(
            [same, Sample(0.01, 1.0, 2.0, 3.0, 0.4, 0.1, -0.1), other],
            self.config))
        self.assertFalse(_duplicate_kinematics_window(
            [same, other, Sample(0.03, 1.02, 2.0, 3.0, 0.4, 0.1, -0.1)],
            self.config))

    def test_velocity_pid_sets_acceleration_and_acceleration_pid_sets_pwm(self):
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "velocity", self.target)
        self.assertAlmostEqual(output["target_ax_body"],
                               min(output["pid_vel_pitch_output"],
                                   self.config["max_xy_acceleration"]))
        expected = int(1500 + output["pid_accel_pitch_output"] *
                       self.config["direction"]["pitch"] * 100)
        self.assertEqual(output["rc_pitch"], expected)
        self.assertNotIn("velocity_boost_pitch", output)

    def test_velocity_estimate_uses_actual_sample_interval(self):
        self.controller.reset(self.sample)
        self.controller.step(self.sample, "velocity", self.target)
        moved = Sample(0.04, 0.004, 0.0, 1.0, 0.0)
        output = self.controller.step(moved, "velocity", self.target)
        self.assertAlmostEqual(output["vx_body"], 0.1)

    def test_velocity_calibration_uses_true_step_while_position_keeps_ramp(self):
        target = {**self.target, "velocity": (0.2, 0.0), "active_axis": "x"}
        self.controller.reset(self.sample)
        step = self.controller.step(self.sample, "velocity", target)
        self.assertAlmostEqual(step["target_vx_body"], 0.2)
        self.controller.reset(self.sample)
        ramp = self.controller.step(self.sample, "position", self.target)
        self.assertAlmostEqual(ramp["target_vx_body"],
                               self.config["max_xy_acceleration"] /
                               self.config["velocity_control_hz"])

    def test_physical_step_metrics_reward_fast_settling_and_stopping(self):
        def flight(time_constant):
            rows = []
            clock = 0.0
            x = 0.0
            for label, sign, duration in (("positive", 1, 6.0),
                                          ("positive_stop", 1, 4.0),
                                          ("negative", -1, 6.0),
                                          ("negative_stop", -1, 4.0)):
                for index in range(int(duration / 0.02)):
                    elapsed = index * 0.02
                    if label.endswith("stop"):
                        velocity = sign * 0.15 * math.exp(-elapsed / 0.15)
                        command = 0.0
                    else:
                        velocity = sign * 0.15 * (1 - math.exp(-elapsed / time_constant))
                        command = sign * 0.15
                    x += velocity * 0.02
                    rows.append({
                        "segment": label, "segment_seconds": duration,
                        "t": clock, "x": x, "y": 0.0, "yaw": 0.0,
                        "z": 1.0, "roll": 0.0, "pitch": 0.0,
                        "command_vx": command, "command_vy": 0.0,
                        "saturated": 0,
                    })
                    clock += 0.02
            return rows

        fast = score_velocity_steps(flight(0.25), "x", 0.15, self.config)
        slow = score_velocity_steps(flight(2.0), "x", 0.15, self.config)
        unreachable = score_velocity_steps(flight(20.0), "x", 0.15, self.config)
        self.assertTrue(fast["reached_both"])
        self.assertTrue(fast["settled_both"])
        self.assertTrue(fast["stopped_both"])
        self.assertLess(fast["score"], slow["score"])
        self.assertLess(fast["positive"]["rise_time"],
                        slow["positive"]["rise_time"])
        self.assertFalse(unreachable["reached_both"])
        self.assertLess(unreachable["positive"]["max_speed"], 0.15)

    def test_grounded_xy_trial_is_invalid(self):
        rows = [{"segment": "positive", "z": -0.19, "roll": 0.0,
                 "pitch": 0.0, "x": 0.0, "y": 0.0} for _ in range(20)]
        self.assertIn("below", _invalid_xy_trial(rows, "velocity", self.config))
        self.assertIn("below", _invalid_xy_trial(rows, "position", self.config))

    def test_stable_candidate_wins_close_score_without_accepting_large_regression(self):
        current = {"score": 2.874, "oscillation_rms": 0.0344}
        stable = {"score": 2.866, "oscillation_rms": 0.0297}
        much_worse = {"score": 3.14, "oscillation_rms": 0.027}
        self.assertTrue(_prefer_trial(stable, current, "height", 0.2, self.config))
        self.assertFalse(_prefer_trial(much_worse, current, "height", 0.2,
                                      self.config))
        self.assertFalse(_prefer_trial(current, stable, "height", 0.2,
                                      self.config))

    def test_xy_trial_waits_for_return_to_launch_zone(self):
        runner = CalibrationRunner(self.config)
        runner.launch_xy = (0.0, 0.0)
        runner.anchor = self.sample
        runner.last_sample = Sample(0.0, 20.0, 0.0, 1.0, 0.0)
        calls = []
        runner._write_csv = lambda label, rows: "zone.csv"

        def segment(stage, target, seconds, label, rows, **_kwargs):
            calls.append((stage, label))
            runner.last_sample = self.sample
            rows.append({"segment": label})

        runner._segment = segment
        runner._ensure_xy_zone("height")
        self.assertEqual(calls, [])
        runner._ensure_xy_zone("velocity")
        self.assertEqual(calls, [("yaw", "waiting_for_xy_zone")])

    def test_velocity_range_confirmation_checks_both_axes_and_speeds(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        seen = []

        def evaluate(stage, name, axis, candidate, label, *, velocity_speed=None):
            seen.append((axis, velocity_speed))
            return {"telemetry_csv": "trial.csv", "metrics": {
                "reached_both": True, "settled_both": True,
                "stopped_both": True, "oscillation_rms": 0.01,
                "terminal_mae": 0.001, "preparations_stable": True,
                "in_band_fraction": 0.8, "tail_in_band_fraction": 0.9,
                "p95_acceleration": 0.2, "p95_jerk": 1.0,
                "positive": {"rise_time": 1.5},
                "negative": {"rise_time": 1.5},
            }}

        runner._evaluate = evaluate
        runner._confirm_velocity_range()
        self.assertEqual(seen, [(axis, speed) for axis in ("x", "y")
                                for speed in self.config["velocity_confirmation_speeds"]])
        self.assertTrue(runner.stage_summary["velocity_range"]["passed"])

    def test_isolated_velocity_axis_keeps_other_pid_and_rc_neutral(self):
        self.controller.reset(self.sample)
        x_target = {**self.target, "active_axis": "x", "velocity": (0.12, 0.12)}
        x_output = self.controller.step(self.sample, "velocity", x_target)
        self.assertNotEqual(x_output["rc_pitch"], 1500)
        self.assertEqual(x_output["rc_roll"], 1500)
        self.assertEqual(x_output["pid_vel_roll_error"], 0.0)
        self.assertEqual(x_output["target_vy_body"], 0.0)

        self.controller.reset(self.sample)
        y_target = {**self.target, "active_axis": "y", "velocity": (0.12, 0.12)}
        y_output = self.controller.step(self.sample, "velocity", y_target)
        self.assertEqual(y_output["rc_pitch"], 1500)
        self.assertNotEqual(y_output["rc_roll"], 1500)
        self.assertEqual(y_output["pid_vel_pitch_error"], 0.0)
        self.assertEqual(y_output["target_vx_body"], 0.0)

    def test_position_stage_uses_both_xy_loops(self):
        self.controller.reset(self.sample)
        output = self.controller.step(self.sample, "position", self.target)
        self.assertEqual(output["pid_pos_x_error"], 0.5)
        self.assertGreater(output["target_vx_body"], 0)
        self.assertNotEqual(output["rc_pitch"], 1500)

    def test_isolated_position_axis_disables_other_position_and_velocity_pid(self):
        self.controller.reset(self.sample)
        target = {**self.target, "position": (0.5, 0.5), "active_axis": "x"}
        output = self.controller.step(self.sample, "position", target)
        self.assertEqual(output["pid_pos_y_error"], 0.0)
        self.assertEqual(output["pid_vel_roll_error"], 0.0)
        self.assertEqual(output["rc_roll"], 1500)
        self.assertNotEqual(output["rc_pitch"], 1500)

        self.controller.reset(self.sample)
        target["active_axis"] = "y"
        output = self.controller.step(self.sample, "position", target)
        self.assertEqual(output["pid_pos_x_error"], 0.0)
        self.assertEqual(output["pid_vel_pitch_error"], 0.0)
        self.assertEqual(output["rc_pitch"], 1500)
        self.assertNotEqual(output["rc_roll"], 1500)

    def test_position_scheduler_runs_at_fifteen_hertz(self):
        self.controller.reset(self.sample)
        for index in range(6):
            sample = Sample(index * 0.02, 0.0, 0.0, 1.0, 0.0)
            self.controller.step(sample, "position", self.target)
        self.assertAlmostEqual(self.controller.last_position, 1 / 15)

    def test_negative_direction_overshoot(self):
        base = {
            "segment": "positive",
            "target_x": -1.0, "origin_x": 0.0, "origin_y": 0.0,
            "t": 0.0, "y": 0.0, "yaw": 0.0, "sample_gap": 0.02,
            "warnings": "",
            "segment_elapsed": 1.0, "segment_seconds": 1.0,
            "saturated": 0,
            "rc_roll": 1500, "rc_pitch": 1500,
            "rc_throttle": 1500, "rc_yaw": 1500,
        }
        within = score_rows([{**base, "x": -0.8}], "position", "x", 1.0)
        beyond = score_rows([{**base, "x": -1.2}], "position", "x", 1.0)
        self.assertEqual(within["max_overshoot"], 0.0)
        self.assertAlmostEqual(beyond["max_overshoot"], 0.2)

    def test_diagnostic_height_threshold_does_not_limit_test_targets(self):
        self.config["height_step"] = 2.0
        validate_config(self.config)

    def test_configuration_merges_nested_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"safety": {"max_height": 3.0}}), encoding="utf-8")
            result = load_config(path)
        self.assertEqual(result["safety"]["max_height"], 3.0)
        self.assertEqual(result["safety"]["max_xy_radius"], 2.5)

    def test_cli_config_creation_and_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--init-config", str(path)]), 0)
                self.assertEqual(main(["--config", str(path)]), 0)
            self.assertTrue(path.exists())

    def test_gain_search_keeps_best_candidate(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        applied = []
        runner.controller.set_configs = lambda configs: applied.append(
            copy.deepcopy(configs))

        def evaluate(stage, name, axis, candidate, label):
            gains = candidate[name]
            score = 1.0 + (gains["kp"] - 5.6) ** 2 + (gains["kd"] - 3.5) ** 2
            return {"metrics": {"score": score, "terminal_mae": score,
                                "oscillation_rms": 0.0}}

        runner._evaluate = evaluate
        runner._tune_pid("height", "pid_height", "z")
        self.assertAlmostEqual(runner.best["pid_height"]["kp"], 5.625)
        self.assertAlmostEqual(runner.best["pid_height"]["kd"], 3.5)
        self.assertLess(runner.stage_summary["pid_height"]["best_score"],
                        runner.stage_summary["pid_height"]["baseline_score"])
        self.assertEqual(applied[-1]["pid_height"], runner.best["pid_height"])

    def test_stage2_expands_derivative_search_and_checks_base_rc_separately(self):
        self.config["search_rounds"] = 1
        self.config["oscillation_extra_rounds"] = 0
        self.config["auto_height_ki_factors"] = []
        self.config["gain_factors"] = [1.0]
        self.config["height_stage2_base_rc_offsets"] = [-30]
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.best_height_base_rc = 1650
        runner.best["pid_height"]["kd"] = 6.114
        runner._save_summary = lambda: None
        observed = []

        def evaluate(stage, name, axis, candidate, label, **kwargs):
            base_rc = kwargs.get("height_base_rc", runner.best_height_base_rc)
            observed.append((label, candidate[name]["kd"], base_rc))
            metrics = {
                "score": abs(base_rc - 1620) / 10 + 1,
                "terminal_mae": 0.02, "oscillation_rms": 0.01,
                "waypoints_reached_fraction": 1.0,
                "max_overshoot": 0.3,
                "waypoints": [{"settling_time": 1.0}],
                "peak_descent_speed": 0.1,
                "descent_oscillation_rms": 0.01,
            }
            return {"metrics": metrics}

        runner._evaluate = evaluate
        runner._tune_pid("height", "pid_height", "z")
        self.assertTrue(any("_broad_kd_" in label and kd > 7.0
                            for label, kd, _ in observed))
        self.assertTrue(any("_base_1620" in label for label, _, _ in observed))
        self.assertEqual(runner.best_height_base_rc, 1620)

    def test_joint_tuning_evaluates_both_axes_and_updates_pair(self):
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        runner.stage_summary["pid_vel_pitch"] = {"stable": False}
        runner.stage_summary["pid_vel_roll"] = {"stable": False}
        labels = []

        def evaluate(stage, names, candidate, label):
            labels.append(label)
            kp = candidate[names[0]]["kp"]
            score = 1.0 if kp < runner.seed[names[0]]["kp"] else 2.0
            axis_metrics = {"score": score, "oscillation_rms": 0.01,
                            "terminal_mae": 0.001, "reached_both": True,
                            "settled_both": True, "stopped_both": True,
                            "preparations_stable": True,
                            "in_band_fraction": 0.8,
                            "tail_in_band_fraction": 0.9,
                            "p95_acceleration": 0.2, "p95_jerk": 1.0,
                            "positive": {"rise_time": 1.5},
                            "negative": {"rise_time": 1.5}}
            return {"metrics": {"score": score * 1.25,
                                "x": axis_metrics, "y": axis_metrics}}

        runner._evaluate_joint = evaluate
        self.assertTrue(runner._tune_joint("velocity", ("pid_vel_pitch", "pid_vel_roll")))
        self.assertEqual(len(labels), 1 + 2 * len(self.config["joint_gain_factors"]))
        self.assertLess(runner.best["pid_vel_pitch"]["kp"],
                        runner.seed["pid_vel_pitch"]["kp"])
        self.assertLess(runner.best["pid_vel_roll"]["kp"],
                        runner.seed["pid_vel_roll"]["kp"])
        self.assertTrue(runner.stage_summary["joint_velocity"]["stable"])

    def test_summary_serializes_processing_functions(self):
        runner = CalibrationRunner(self.config)
        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory)
            runner._save_summary()
            summary = json.loads((runner.run_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["initial_pids"]["pid_height"]["processing_func"],
                         "default_height_pid_processing")

    def test_height_steps_restore_stage_loops_without_scoring_recovery(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        candidate = get_drone_pid_setup("DEFAULT")
        candidate["pid_height"]["kp"] = 5.0
        runner.controller.set_configs(candidate)
        seen = []

        def recovery(stage):
            seen.append(("recovery", stage))
            runner.controller.set_configs(runner.best)

        def segment(stage, target, seconds, label, rows, **_kwargs):
            seen.append((label, stage, runner.controller.pids["pid_height"].kp))
            rows.append({"segment": label})

        runner._recover = recovery
        runner._segment = segment
        def descend(goal, label, rows, **_kwargs):
            seen.append((label, "height", runner.controller.pids["pid_height"].kp))
            rows.append({"segment": label})
            return [{"reached": True, "target_raw_z": goal}]
        runner._descend_height = descend
        runner._write_csv = lambda label, rows: "test.csv"
        rows, _ = runner._profile("height", "z", "test")
        self.assertEqual([row["segment"] for row in rows],
                         ["positive", "negative", "negative_hold"])
        self.assertEqual(seen, [
            ("positive", "height", 5.0),
            ("recovery", "height"),
            ("negative", "height", 5.0),
            ("negative_hold", "height", 5.0),
        ])

    def test_continuous_height_waypoints_use_launch_relative_z(self):
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.controller.z_bias = -0.2
        runner.last_sample = Sample(0.0, 0, 0, 0.8, 0)
        runner.anchor = runner.last_sample
        goals = []
        commands = []
        durations = []

        def descend(goal, label, rows, **kwargs):
            goals.append((goal, kwargs["relative"]))
            runner.last_sample = Sample(1.0, 0, 0, goal, 0)
            rows.append({"segment": label})
            return [{"reached": True, "command_z": goal + 0.2}]

        def segment(stage, target, seconds, label, rows, **kwargs):
            commands.append((target["height"], target["height_relative"]))
            durations.append(seconds)
            runner.last_sample = Sample(1.0, 0, 0,
                                        target["height"] + runner.controller.z_bias, 0)
            rows.append({"segment": label})

        runner._descend_height = descend
        runner._segment = segment
        runner._write_csv = lambda label, rows: "waypoints.csv"
        rows, _ = runner._profile("height", "z", "waypoints")
        self.assertAlmostEqual(goals[0][0], 0.3)
        self.assertTrue(goals[0][1])
        self.assertEqual(commands, [(height, True) for height in
                                     self.config["height_stage2_targets"]])
        self.assertAlmostEqual(durations[-1], 9.5)
        self.assertEqual([row["waypoint_index"] for row in rows if
                          row["segment"].endswith("_hold")], [1, 2, 3, 4, 5])
        self.assertAlmostEqual(rows[-1]["requested_target_z"], 2.8)

    def test_continuous_height_score_checks_each_waypoint(self):
        rows = []
        for index, height in enumerate(self.config["height_stage2_targets"], 1):
            for sample_index in range(10):
                rows.append({
                    "t": 10 * index + sample_index * 0.1,
                    "segment": f"height_wp_{index:02d}_hold",
                    "segment_elapsed": sample_index * 0.1,
                    "segment_seconds": 1.0,
                    "waypoint_index": index,
                    "requested_target_z": height,
                    "z": height, "target_z": height, "origin_z": 1.0,
                    "x": 0.0, "y": 0.0, "yaw": 0.0,
                    "origin_x": 0.0, "origin_y": 0.0,
                    "rc_roll": 1500, "rc_pitch": 1500,
                    "rc_throttle": 1600, "rc_yaw": 1500,
                    "saturated": False, "warnings": "", "sample_gap": 0.1,
                })
        stable = score_height_waypoints(rows, self.config)
        self.assertEqual(stable["waypoints_reached_fraction"], 1.0)
        self.assertEqual(len(stable["waypoints"]), 5)
        for row in rows[-10:]:
            row["z"] += 0.1
        missed = score_height_waypoints(rows, self.config)
        self.assertEqual(missed["waypoints_reached_fraction"], 0.8)
        self.assertGreater(missed["score"], stable["score"])

    def test_stage2_score_rejects_large_transient_despite_perfect_final_hold(self):
        def flight(overshoot):
            rows = []
            for index, height in enumerate(self.config["height_stage2_targets"], 1):
                for sample_index in range(61):
                    z = (height - 0.2 if sample_index < 10 else
                         height + overshoot if sample_index < 20 else height)
                    rows.append({
                        "t": 10 * index + sample_index * 0.1,
                        "segment": f"height_wp_{index:02d}_hold",
                        "segment_elapsed": sample_index * 0.1,
                        "segment_seconds": 6.0, "waypoint_index": index,
                        "requested_target_z": height, "target_z": height,
                        "z": z, "origin_z": height - 0.2,
                        "x": 0.0, "y": 0.0, "yaw": 0.0,
                        "origin_x": 0.0, "origin_y": 0.0,
                        "rc_roll": 1500, "rc_pitch": 1500,
                        "rc_throttle": 1600, "rc_yaw": 1500,
                        "saturated": False, "warnings": "", "sample_gap": 0.1,
                    })
            return score_height_waypoints(rows, self.config)

        smooth = flight(0.0)
        overshooting = flight(0.3)
        self.assertAlmostEqual(smooth["terminal_mae"], 0.0)
        self.assertAlmostEqual(overshooting["terminal_mae"], 0.0)
        self.assertGreater(overshooting["score"], smooth["score"])
        self.assertTrue(_height_stage2_stable(smooth, self.config))
        self.assertFalse(_height_stage2_stable(overshooting, self.config))
        self.assertGreater(overshooting["max_settling_time"],
                           smooth["max_settling_time"])
        overshooting["score"] = 1.0
        self.assertTrue(_prefer_trial(smooth, overshooting, "height", 0.2,
                                      self.config))
        self.assertFalse(_prefer_trial(overshooting, smooth, "height", 0.2,
                                       self.config))

    def test_continuous_height_takeoff_and_recovery_use_launch_bias(self):
        class Link:
            def read_sample(self):
                return Sample(time.monotonic(), 0, 0, -0.2, 0)

        runner = CalibrationRunner(self.config, link=Link())
        runner.height_waypoints_mode = True
        runner.controller.z_bias = -0.2
        runner.anchor = runner.last_sample = Sample(0, 0, 0, -0.2, 0)
        runner._write_csv = lambda label, rows: "test.csv"
        runner._save_summary = lambda: None
        commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            commands.append((label, target["height"], target["height_relative"]))
            runner.last_sample = Sample(time.monotonic(), 0, 0,
                                        target["height"] + runner.controller.z_bias, 0)

        runner._segment = segment
        runner._takeoff()
        self.assertEqual(commands[0], ("takeoff", 1.075, True))
        self.assertTrue(runner.stage_summary["takeoff"]["reached_height"])
        runner.last_sample = Sample(time.monotonic(), 0, 0, 1.3, 0)
        goals = []

        def descend(goal, label, rows, **kwargs):
            goals.append((goal, kwargs["relative"]))
            runner.last_sample = Sample(time.monotonic(), 0, 0, goal, 0)
            return [{"reached": True, "command_z": 1.0}]

        runner._descend_height = descend
        runner._recover("height")
        self.assertEqual(goals, [(0.8, True)])
        self.assertEqual(commands[-1], ("recovery", 1.0, True))

    def test_height_descent_uses_measured_five_centimeter_steps(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            self.assertEqual(stage, "height")
            self.assertTrue(kwargs["scored"])
            commands.append(target["height"])
            first = len(rows)
            for index in range(40):
                t = runner.last_sample.t + 0.02
                runner.last_sample = Sample(t, 0, 0, target["height"], 0)
                rows.append({"t": t, "z": target["height"],
                             "height_step_reached": "", "segment": label})
                if kwargs["stop_when"](rows[first:]):
                    break

        runner._segment = segment
        rows = []
        steps = runner._descend_height(0.8, "negative", rows, scored=True)
        self.assertEqual(len(steps), 4)
        self.assertTrue(all(step["reached"] for step in steps))
        for previous, command in zip([1.0] + commands[:-1], commands):
            self.assertAlmostEqual(previous - command, 0.05)
        self.assertTrue(all(row["height_step_reached"] == 1 for row in rows))

    def test_height_descent_stops_after_unreached_step(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            commands.append(target["height"])
            rows.append({"t": 0.02, "z": 1.0, "height_step_reached": ""})

        runner._segment = segment
        rows = []
        steps = runner._descend_height(0.8, "negative", rows, scored=True)
        self.assertEqual(len(commands), 1)
        self.assertFalse(steps[0]["reached"])
        self.assertEqual(rows[0]["height_step_reached"], 0)

    def test_stage2_descent_deepens_stalled_command_with_bounded_gap(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            command = target["height"]
            actual = command + 0.04
            commands.append((command, runner.last_sample.z))
            for _ in range(40):
                t = runner.last_sample.t + 0.02
                runner.last_sample = Sample(t, 0, 0, actual, 0)
                rows.append({"t": t, "z": actual,
                             "height_step_reached": "", "segment": label})
                if kwargs["stop_when"](rows[-40:]):
                    break

        runner._segment = segment
        steps = runner._descend_height(0.8, "stage2", [], scored=True,
                                       compensate_stall=True)
        self.assertGreater(len(steps), 1)
        self.assertFalse(steps[0]["reached"])
        self.assertLessEqual(runner.last_sample.z, 0.82)
        self.assertTrue(all(actual - command <=
                            self.config["height_descent_max_command_gap"] + 1e-6
                            for command, actual in commands))
        self.assertGreaterEqual(min(command for command, _ in commands), 0.7)

    def test_stage2_descent_does_not_chase_an_altitude_spike(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = runner.last_sample = self.sample
        commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            commands.append(target["height"])
            for _ in range(40):
                t = runner.last_sample.t + 0.02
                runner.last_sample = Sample(t, 0, 0, 2.0, 0)
                rows.append({"t": t, "z": 2.0,
                             "height_step_reached": "", "segment": label})
                if kwargs["stop_when"](rows[-40:]):
                    break

        runner._segment = segment
        runner._descend_height(0.8, "stage2", [], compensate_stall=True)
        self.assertEqual(commands, [0.95, 0.95])

    def test_stage2_recovery_rejects_wrong_start_height(self):
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.anchor = runner.last_sample = Sample(0, 0, 0, 3.0, 0)
        runner._write_csv = lambda label, rows: "recovery.csv"
        runner._save_summary = lambda: None
        runner._descend_height = lambda *args, **kwargs: [
            {"reached": False, "command_z": 2.95, "target_raw_z": 2.95}]

        def segment(stage, target, seconds, label, rows, **kwargs):
            rows.append({"t": 1.0, "z": 3.0, "yaw": 0.0})

        runner._segment = segment
        with self.assertRaisesRegex(TrialInvalid, "Recovery stayed"):
            runner._recover("height")
        self.assertFalse(runner.recoveries[-1]["height_reached"])

    def test_stage2_recovery_accepts_small_oscillations_around_target(self):
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "recovery.csv"
        runner._save_summary = lambda: None
        observed = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            for index in range(50):
                z = target["height"] + (0.02 if index % 2 else 0.07)
                observed.append(z)
                rows.append({"t": index * 0.02, "z": z, "yaw": 0.0,
                             "rc_throttle": 1620})
            runner.last_sample = Sample(1.0, 0, 0, rows[-1]["z"], 0)

        runner._segment = segment
        runner._recover("height")
        self.assertTrue(runner.recoveries[-1]["height_reached"])
        self.assertGreater(max(abs(z - 1.0) for z in observed),
                           self.config["height_stage2_tolerance"])

    def test_stage2_recovery_restores_selected_base_throttle(self):
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.best_height_base_rc = 1620
        runner.controller.base_throttle_rc = 1665
        runner.anchor = runner.last_sample = self.sample
        runner._write_csv = lambda label, rows: "recovery.csv"
        runner._save_summary = lambda: None
        runner._segment = lambda stage, target, seconds, label, rows, **kwargs: rows.append(
            {"t": 1.0, "z": 1.0, "yaw": 0.0, "rc_throttle": 1620})
        runner._recover("height")
        self.assertEqual(runner.controller.base_throttle_rc, 1620)

    def test_stage2_waits_and_repeats_same_trial_after_failed_recovery(self):
        runner = CalibrationRunner(self.config)
        runner.height_waypoints_mode = True
        runner.config["max_intervention_retries"] = 0
        runner._save_summary = lambda: None
        calls = []

        def recover(stage):
            calls.append("recover")
            if calls.count("recover") == 1:
                raise TrialInvalid("Recovery stayed too high")

        def resume(stage):
            calls.append("ground_return")

        runner._recover = recover
        runner._resume_vertical_after_ground = resume
        runner._profile = lambda *args, **kwargs: ([{"segment": "hold"}], "valid.csv")
        metrics = {"score": 1.0, "mae": 0.01, "terminal_mae": 0.01,
                   "oscillation_rms": 0.0, "descent_reached_fraction": 1.0,
                   "descent_steps": 1, "peak_descent_speed": 0.0,
                   "descent_oscillation_rms": 0.0}
        with patch("utils.auto_calibration_tool.score_height_waypoints",
                   return_value=metrics):
            result = runner._evaluate("height", "pid_height", "z", runner.best,
                                      "same_trial")
        self.assertEqual(calls, ["recover", "ground_return", "recover"])
        self.assertEqual([record["valid"] for record in runner.records], [False, True])
        self.assertEqual(result["label"], "same_trial")

    def test_xy_fall_waits_then_retries_same_trial_without_using_retry_budget(self):
        runner = CalibrationRunner(self.config)
        runner.config["max_intervention_retries"] = 0
        runner.xy_waypoints_mode = True
        runner.last_sample = self.sample
        runner.anchor = self.sample
        runner._save_summary = lambda: None
        runner._recover = lambda stage: None
        runner._ensure_xy_zone = lambda stage: None
        calls = []

        def profile(*args, **kwargs):
            calls.append("profile")
            if len(calls) == 1:
                runner.last_sample = Sample(1.0, 0, 0, -1.0, 0)
                raise TrialInvalid("Drone was below the airborne height during the trial")
            return [
                {"segment": "positive", "x": 0.0, "y": 0.0, "z": 1.0,
                 "roll": 0.0, "pitch": 0.0},
                {"segment": "negative", "x": 0.2, "y": 0.0, "z": 1.0,
                 "roll": 0.0, "pitch": 0.0},
            ], "good.csv"

        def respawn(stage):
            calls.append("respawn")
            runner.last_sample = self.sample

        runner._profile = profile
        runner._wait_for_xy_respawn = respawn
        metrics = {"score": 1.0, "waypoints_reached_fraction": 1.0,
                   "terminal_mae": 0.0, "max_overshoot": 0.0,
                   "oscillation_rms": 0.0}
        with patch("utils.auto_calibration_tool.score_position_waypoints",
                   return_value=metrics):
            result = runner._evaluate("position", "pid_pos_x", "x", runner.best,
                                      "same_trial")
        self.assertEqual(calls, ["profile", "respawn", "profile"])
        self.assertTrue(result["valid"])
        self.assertEqual(result["label"], "same_trial")

    def test_xy_respawn_waits_for_stable_ground_then_takes_off(self):
        class Link:
            def __init__(self):
                self.samples = iter((
                    Sample(1.0, 0, 0, 0, 1.2, vx_world=0, vy_world=0, vz_world=0),
                    Sample(1.6, 0, 0, 0, 1.2, vx_world=0, vy_world=0, vz_world=0),
                ))
                self.calls = []

            def disarm(self):
                self.calls.append("disarm")

            def arm(self):
                self.calls.append("arm")

            def read_sample(self):
                return next(self.samples)

        link = Link()
        runner = CalibrationRunner(self.config, link)
        runner._save_summary = lambda: None
        runner.status = "acceleration_x"
        runner.xy_reference_yaw = 0.4
        runner.last_sample = Sample(0.0, 10, 0, -1.8, 0)
        runner.launch_xy = (0, 0)
        runner._takeoff = lambda: runner.stage_summary.update(
            takeoff={"reached_height": True})
        with patch("utils.auto_calibration_tool.time.sleep"):
            runner._wait_for_xy_respawn("acceleration")
        self.assertEqual(link.calls, ["disarm", "arm"])
        self.assertEqual(runner.status, "acceleration_x")
        self.assertEqual(runner.launch_xy, (0, 0))
        self.assertAlmostEqual(runner.xy_reference_yaw, 0.4)
        self.assertAlmostEqual(runner.controller.z_bias, 0.0)

    def test_low_height_relative_target_applies_ground_bias(self):
        self.controller.z_bias = -0.2
        self.controller.reset(self.sample)
        target = dict(self.target, height=1.0, height_relative=True)
        self.controller.step(Sample(0.0, 0, 0, 0.8, 0), "height", target)
        self.assertAlmostEqual(self.controller.pids["pid_height"].current_error, 0.0)

    def test_landing_uses_ground_relative_target(self):
        runner = CalibrationRunner(self.config)
        runner.controller.z_bias = -0.2
        runner.anchor = self.sample
        runner.last_sample = self.sample
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "landing.csv"
        goals = []

        def descend(goal_raw, label, rows, **kwargs):
            goals.append((goal_raw, kwargs["relative"]))
            runner.last_sample = Sample(1.0, 0, 0, goal_raw, 0)
            return [{"reached": True, "command_z": 0.0,
                     "target_raw_z": goal_raw}]

        runner._descend_height = descend
        runner._segment = lambda *args, **kwargs: None
        self.assertTrue(runner._land())
        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0][0], -0.2)
        self.assertTrue(goals[0][1])
        self.assertAlmostEqual(runner.stage_summary["landing"]["height_above_launch"],
                               0.0)

    def test_raw_zero_does_not_count_as_landed_with_negative_bias(self):
        runner = CalibrationRunner(self.config)
        runner.controller.z_bias = -0.2
        runner.anchor = self.sample
        runner.last_sample = Sample(0.0, 0, 0, 0.0, 0)
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "landing.csv"
        runner._descend_height = lambda *args, **kwargs: [
            {"reached": False, "command_z": 0.0, "target_raw_z": -0.2}]
        runner._segment = lambda *args, **kwargs: None
        self.assertFalse(runner._land())
        self.assertAlmostEqual(runner.stage_summary["landing"]["height_above_launch"],
                               0.2)

    def test_height_step_metrics_detect_fast_drop_and_failed_step(self):
        rows = []
        for index, z in enumerate((1.0, 0.96, 0.89, 0.9, 0.91)):
            rows.append({"height_step_index": 1, "height_step_reached": 0,
                         "segment": "negative_step_01", "t": index * 0.1,
                         "z": z, "target_z": 0.95})
        metrics = _height_descent_metrics(rows)
        self.assertEqual(metrics["descent_reached_fraction"], 0.0)
        self.assertGreater(metrics["peak_descent_speed"], 0.3)

    def test_descent_progress_is_distinct_from_settling_at_each_step(self):
        rows = [{"height_step_index": 1, "height_step_reached": 0,
                 "height_step_progressed": 1, "segment": "descent_step_01",
                 "t": index * 0.1, "z": 1.0 - 0.003 * index,
                 "target_z": 0.95} for index in range(11)]
        metrics = _height_descent_metrics(rows)
        self.assertEqual(metrics["descent_reached_fraction"], 0.0)
        self.assertEqual(metrics["descent_progressed_fraction"], 1.0)

    def test_fast_descent_cannot_replace_stable_height_trial(self):
        stable = {"score": 3.0, "oscillation_rms": 0.01,
                  "descent_reached_fraction": 1.0,
                  "peak_descent_speed": 0.2,
                  "descent_oscillation_rms": 0.01}
        fast = dict(stable, score=1.0, peak_descent_speed=0.5)
        self.assertFalse(_prefer_trial(fast, stable, "height", 0.2, self.config))

    def test_height_score_includes_descent_steps(self):
        rows = self._height_rows(0.0)
        for index in range(11):
            row = dict(rows[-1])
            row.update(t=6.1 + index * 0.1, segment="negative_step_01",
                       segment_elapsed=index * 0.1, segment_seconds=1.0,
                       target_z=0.95, z=0.95,
                       height_step_index=1, height_step_reached=1)
            rows.append(row)
        metrics = score_rows(rows, "height", "z", 0.2)
        self.assertEqual(metrics["descent_steps"], 1)
        self.assertEqual(metrics["descent_reached_fraction"], 1.0)

    def test_isolated_profiles_mask_other_axis_and_joint_profile_enables_both(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner._wait_velocity_rest = lambda axis, rows, label: None
        seen = []

        def segment(stage, target, seconds, label, rows, **_kwargs):
            seen.append((stage, target["active_axis"], target["yaw"],
                         target["velocity"], target["position"], seconds))
            rows.append({"segment": label})

        runner._segment = segment
        runner._write_csv = lambda label, rows: "test.csv"
        runner._profile("velocity", "x", "vx")
        self.assertTrue(all(row[1] == "x" and row[2] == 0.0 for row in seen))
        self.assertEqual(seen[0][3], (self.config["velocity_step"], 0.0))
        self.assertEqual(len(seen), 4)
        self.assertEqual(seen[1][3], (0.0, 0.0))
        seen.clear()
        runner._profile("position", "y", "py")
        self.assertTrue(all(row[1] == "y" and row[2] == 0.0 for row in seen))
        self.assertEqual(seen[0][4][0], self.sample.x)
        seen.clear()
        runner._profile("velocity", "xy", "joint", joint=True)
        self.assertTrue(all(row[1] is None for row in seen))
        self.assertEqual(seen[0][3][0], seen[0][3][1])
        self.assertEqual(seen[0][5], self.config["joint_step_seconds"])

    def test_recovery_uses_only_calibrated_loops(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "recovery.csv"
        seen = []

        def segment(stage, target, seconds, label, rows, **_kwargs):
            seen.append(stage)
            rows.append({"t": 1.0, "z": target["height"], "yaw": target["yaw"]})

        runner._segment = segment
        for stage in ("height", "yaw", "velocity", "position"):
            runner._recover(stage)
        self.assertEqual(seen, ["height", "yaw", "yaw", "yaw"])

    def test_takeoff_uses_height_loop_only(self):
        class Link:
            def read_sample(self):
                return Sample(0.0, 0.0, 0.0, 0.0, 0.0)

        runner = CalibrationRunner(self.config, Link())
        runner.anchor = self.sample
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "takeoff.csv"
        seen = []

        def segment(stage, target, seconds, label, rows, **_kwargs):
            seen.append(stage)

        runner._segment = segment
        runner._takeoff()
        self.assertEqual(seen, ["height"])
        self.assertFalse(runner.stage_summary["takeoff"]["reached_height"])

    def test_thresholds_are_warnings_and_xy_ignored_for_height_and_yaw(self):
        class Link:
            stream_error = None

        runner = CalibrationRunner(self.config, Link())
        runner.initial_sample = self.sample
        runner.last_sample = self.sample
        far_sample = Sample(0.02, 10.0, 10.0, 4.0, 0.0)
        height_warnings = runner._check_sample(far_sample, "height")
        self.assertIn("high_altitude", height_warnings)
        self.assertNotIn("xy_radius", height_warnings)
        yaw_warnings = runner._check_sample(far_sample, "yaw")
        self.assertNotIn("xy_radius", yaw_warnings)
        position_warnings = runner._check_sample(far_sample, "position")
        self.assertIn("xy_radius", position_warnings)

    def test_manual_move_is_detected_by_single_sample_jump(self):
        runner = CalibrationRunner(self.config)
        runner.last_sample = self.sample
        self.assertIsNone(runner._intervention_reason(Sample(0.02, 0.1, 0, 1, 0)))
        self.assertIn("XY jump", runner._intervention_reason(Sample(0.02, 40, 30, 0, 0)))

    def test_manual_move_invalidates_measured_segment_and_rebases_anchor(self):
        start = time.monotonic()

        class Link:
            stream_error = None

            def __init__(self):
                self.samples = iter((
                    Sample(start + 0.02, 0.01, 0, 1, 0),
                    Sample(start + 0.04, 40, 30, 0, 0),
                ))
                self.frames = []

            def read_sample(self):
                return next(self.samples)

            def set_frame(self, frame):
                self.frames.append(frame)

        link = Link()
        runner = CalibrationRunner(self.config, link)
        runner.anchor = Sample(start, 0, 0, 1, 0)
        runner.initial_sample = runner.anchor
        runner.last_sample = runner.anchor
        rows = []
        with self.assertRaises(InterventionDetected):
            runner._segment("height", runner._target(height=1.2), 0.2,
                            "positive", rows, scored=True)
        self.assertEqual(len(rows), 1)
        self.assertEqual((runner.anchor.x, runner.anchor.y), (40, 30))
        self.assertEqual(len(runner.interventions), 1)

    def test_manual_move_retries_candidate_without_scoring_bad_attempt(self):
        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        runner.last_csv = "bad.csv"
        runner._save_summary = lambda: None
        runner._recover = lambda stage: None
        calls = []

        def profile(stage, axis, label):
            calls.append(label)
            if len(calls) == 1:
                raise InterventionDetected("simulator reset")
            return self._height_rows(0.0), "good.csv"

        runner._profile = profile
        record = runner._evaluate("height", "pid_height", "z", runner.best, "trial")
        self.assertEqual(len(calls), 2)
        self.assertFalse(runner.records[0]["valid"])
        self.assertTrue(record["valid"])
        self.assertEqual(record["telemetry_csv"], "good.csv")

    @staticmethod
    def _height_rows(oscillation):
        import math

        rows = []
        for index in range(61):
            elapsed = index * 0.1
            rows.append({
                "segment": "positive", "t": elapsed,
                "segment_elapsed": elapsed, "segment_seconds": 6.0,
                "target_z": 1.2, "z": 1.1 + oscillation * math.sin(2 * math.pi * elapsed),
                "origin_z": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "x": 0.0, "y": 0.0, "yaw": 0.0,
                "sample_gap": 0.1, "warnings": "", "saturated": 0,
                "rc_roll": 1500, "rc_pitch": 1500,
                "rc_throttle": 1500, "rc_yaw": 1500,
            })
        return rows

    def test_oscillation_penalized_separately_from_steady_bias(self):
        steady = score_rows(self._height_rows(0.0), "height", "z", 0.2)
        oscillating = score_rows(self._height_rows(0.08), "height", "z", 0.2)
        self.assertAlmostEqual(steady["oscillation_rms"], 0.0)
        self.assertGreater(oscillating["oscillation_rms"], 0.04)
        self.assertGreater(oscillating["score"], steady["score"])

    def test_search_refines_even_when_first_round_has_no_improvement(self):
        self.config["search_rounds"] = 2
        self.config["auto_height_ki_factors"] = []
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        labels = []

        def evaluate(stage, name, axis, candidate, label):
            labels.append(label)
            score = 1 + (candidate[name]["kp"] - 4.2) ** 2
            return {"metrics": {"score": score, "terminal_mae": score,
                                "oscillation_rms": 0.0}}

        runner._evaluate = evaluate
        runner._tune_pid("height", "pid_height", "z")
        self.assertTrue(any("_r1_" in label for label in labels))
        self.assertLess(runner.best["pid_height"]["kp"], 4.5)

    def test_height_integral_is_searched_by_default(self):
        self.config["search_rounds"] = 1
        self.config["min_improvement"] = 0
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None

        def evaluate(stage, name, axis, candidate, label):
            score = 1 + (candidate[name]["ki"] - 0.135) ** 2
            return {"metrics": {"score": score, "terminal_mae": score,
                                "oscillation_rms": 0.0}}

        runner._evaluate = evaluate
        runner._tune_pid("height", "pid_height", "z")
        self.assertAlmostEqual(runner.best["pid_height"]["ki"], 0.135)

    def test_unresolved_oscillation_gets_extra_rounds_and_unstable_status(self):
        self.config["search_rounds"] = 1
        self.config["oscillation_extra_rounds"] = 2
        self.config["auto_height_ki_factors"] = []
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        labels = []

        def evaluate(stage, name, axis, candidate, label):
            labels.append(label)
            return {"metrics": {"score": 1.0, "terminal_mae": 0.1,
                                "oscillation_rms": 0.1}}

        runner._evaluate = evaluate
        runner._tune_pid("height", "pid_height", "z")
        self.assertTrue(any("_r2_" in label for label in labels))
        self.assertFalse(runner.stage_summary["pid_height"]["stable"])

    def test_unscored_height_does_not_activate_later_stages(self):
        class Link:
            def connect(self):
                pass

            def read_sample(self):
                return Sample(0, 0, 0, 0, 0)

            def arm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, Link())
        stages = []
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner._land = lambda: True
        runner._validation = lambda: self.fail("validation should not run")

        def tune(stage, name, axis):
            stages.append(stage)
            return False

        runner._tune_pid = tune
        runner._tune_acceleration_pid = lambda name, axis: tune(
            "acceleration", name, axis)
        runner._tune_velocity_pid = lambda name, axis, **kwargs: tune(
            "velocity", name, axis)
        runner._tune_joint = lambda stage, names: True
        runner._confirm_velocity_range = lambda: runner.stage_summary.update(
            {"velocity_range": {"passed": True}})
        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory) / "run"
            runner.run()
        self.assertEqual(stages, ["height"])
        self.assertEqual(runner.status, "needs_review")

    def test_scored_but_unstable_stage_keeps_best_and_continues_plan(self):
        class Link:
            def connect(self):
                pass

            def read_sample(self):
                return Sample(0, 0, 0, 0, 0)

            def arm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, Link())
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner._land = lambda: True
        runner._validation = lambda: True
        tuned_names = []

        def scored_but_unstable(stage, name, axis):
            tuned_names.append(name)
            runner.best[name]["kp"] += 0.01
            runner.stage_summary[name] = {
                "stable": False,
                "best_metrics": {"score": 1.0},
                "recommended_gains": {
                    key: runner.best[name][key] for key in ("kp", "ki", "kd")},
            }
            return False

        runner._tune_pid = scored_but_unstable
        runner._tune_acceleration_pid = lambda name, axis: scored_but_unstable(
            "acceleration", name, axis)
        runner._tune_velocity_pid = lambda name, axis, **kwargs: scored_but_unstable(
            "velocity", name, axis)
        runner._tune_position_pid = lambda name, axis, **kwargs: scored_but_unstable(
            "position", name, axis)
        runner._validate_velocity_speeds = lambda name, axis: False
        runner._validate_position_distances = lambda name, axis: False
        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory) / "run"
            runner.run()
        self.assertEqual(tuned_names, [
            "pid_height", "pid_yaw",
            "pid_accel_pitch", "pid_accel_roll",
            "pid_vel_pitch", "pid_vel_roll",
            "pid_pos_x", "pid_pos_y",
        ])
        self.assertEqual(runner.status, "needs_review")

    def test_oscillating_stage_prevents_complete_status(self):
        self.config["position_repeated_mode"] = False
        class Link:
            def connect(self):
                pass

            def read_sample(self):
                return Sample(0, 0, 0, 0, 0)

            def arm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, Link())
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner._land = lambda: True
        runner._validation = lambda: True

        def tune(stage, name, axis):
            runner.stage_summary[name] = {"stable": name != "pid_height"}
            return True

        runner._tune_pid = tune
        runner._tune_acceleration_pid = lambda name, axis: tune(
            "acceleration", name, axis)
        runner._tune_velocity_pid = lambda name, axis, **kwargs: tune(
            "velocity", name, axis)
        runner._tune_joint = lambda stage, names: True
        runner._confirm_velocity_range = lambda: runner.stage_summary.update(
            {"velocity_range": {"passed": True}})
        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory) / "run"
            runner.run()
        self.assertEqual(runner.status, "needs_review")

    def test_joint_stages_follow_each_isolated_axis_pair(self):
        self.config["velocity_repeated_mode"] = False
        self.config["position_repeated_mode"] = False
        class Link:
            def connect(self):
                pass

            def read_sample(self):
                return Sample(0, 0, 0, 0, 0)

            def arm(self):
                pass

            def close(self):
                pass

        runner = CalibrationRunner(self.config, Link())
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner._land = lambda: True
        runner._validation = lambda: True
        order = []

        def tune(stage, name, axis):
            order.append(name)
            runner.stage_summary[name] = {"stable": True}
            return True

        def joint(stage, names):
            order.append(f"joint_{stage}")
            runner.stage_summary[f"joint_{stage}"] = {"stable": True}
            return True

        runner._tune_pid = tune
        runner._tune_acceleration_pid = lambda name, axis: tune(
            "acceleration", name, axis)
        runner._tune_velocity_pid = lambda name, axis, **kwargs: tune(
            "velocity", name, axis)
        runner._tune_joint = joint
        runner._confirm_velocity_range = lambda: runner.stage_summary.update(
            {"velocity_range": {"passed": True}})
        with tempfile.TemporaryDirectory() as directory:
            runner.run_dir = Path(directory) / "run"
            runner.run()
        self.assertEqual(order, [
            "pid_height", "pid_yaw", "pid_accel_pitch", "pid_accel_roll",
            "pid_vel_pitch", "pid_vel_roll",
            "joint_velocity", "pid_pos_x", "pid_pos_y", "joint_position",
        ])
        self.assertEqual(runner.status, "complete")

    def test_validation_rejects_oscillation_even_with_small_final_error(self):
        import math

        runner = CalibrationRunner(self.config)
        runner.anchor = self.sample
        runner.last_sample = self.sample
        runner._recover = lambda stage: None
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: "validation.csv"

        def segment(stage, target, seconds, label, rows, **_kwargs):
            if label == "diagonal":
                return
            for index in range(61):
                t = index * 0.1
                rows.append({
                    "segment": "return", "t": t,
                    "x": 0.1 * math.sin(2 * math.pi * t), "y": 0.0,
                    "z": 1.0, "yaw": 0.0,
                    "target_x": 0.0, "target_y": 0.0,
                    "target_z": 1.0, "target_yaw": 0.0,
                    "command_vx": 0.0, "command_vy": 0.0,
                    "vx_body": 0.0, "vy_body": 0.0,
                })

        runner._segment = segment
        self.assertFalse(runner._validation_once())
        self.assertLess(runner.stage_summary["validation"]["final_xy_error"],
                        self.config["validation_tolerances"]["xy_m"])
        self.assertFalse(runner.stage_summary["validation"]["stable"])

    def test_ascent_targets_are_seeded_and_stay_in_range(self):
        runner = CalibrationRunner(self.config)
        targets = runner._ascent_targets()
        self.assertEqual(targets, runner._ascent_targets())
        self.assertEqual(len(targets), self.config["height_ascent_targets"])
        self.assertTrue(all(self.config["height_ascent_min_m"] <= target <=
                            self.config["height_ascent_max_m"] for target in targets))

    def test_direct_throttle_landing_requires_stable_ground(self):
        class Link:
            stream_error = None

            def __init__(self, z):
                self.z = z
                self.frames = []

            def read_sample(self):
                return Sample(time.monotonic(), 0.0, 0.0, self.z, 0.0)

            def set_frame(self, frame):
                self.frames.append(frame)

        config = default_config()
        config["landing_seconds"] = 0.16
        config["throttle_landing_ground_hold_seconds"] = 0.04
        config["throttle_landing_velocity_window"] = 0.04
        config["throttle_landing_rc_per_second"] = 10000.0
        # Ground contact is inferred from a sustained near-zero vertical rate
        # under minimum throttle, even when terrain is above the launch Z.
        for z, expected in ((0.0, True), (-0.3, True), (1.0, True)):
            link = Link(z)
            runner = CalibrationRunner(config, link=link)
            runner.controller.z_bias = 0.0
            runner.last_sample = link.read_sample()
            runner._write_csv = lambda label, rows: "landing.csv"
            landing = runner._throttle_ramp_land("test_landing")
            self.assertEqual(landing["grounded"], expected)
            if z < 0:
                self.assertAlmostEqual(landing["ground_offset_from_launch"], z)
            self.assertTrue(all(1400 <= frame[2] <= 1620 for frame in link.frames))
            if not expected:
                self.assertEqual(landing["final_throttle_rc"], 1400)
            self.assertEqual({(frame[0], frame[1], frame[3]) for frame in link.frames},
                             {(1500, 1500, 1500)})

    def test_ascent_trial_does_not_start_next_trial_without_ground(self):
        class Link:
            stream_error = None

            def __init__(self):
                self.disarmed = False

            def read_sample(self):
                return Sample(time.monotonic(), 0.0, 0.0, -0.035, 0.0)

            def arm(self):
                pass

            def disarm(self):
                self.disarmed = True

        link = Link()
        runner = CalibrationRunner(self.config, link=link)
        runner.controller.z_bias = -0.019
        ascent_commands = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            if label == "ascent":
                for elapsed in (0.0, 1.0, 5.0):
                    kwargs["target_update"](None, elapsed)
                    ascent_commands.append(target["height"])

        runner._segment = segment
        runner._write_csv = lambda label, rows: "trial.csv"
        runner._save_summary = lambda: None
        runner._throttle_ramp_land = lambda label: {
            "grounded": False, "telemetry_csv": "landing.csv"}
        with self.assertRaises(LandingIncomplete):
            runner._ascent_trial(1.0, 0.25, "test", runner.best, 1610)
        self.assertFalse(link.disarmed)
        self.assertEqual(ascent_commands, [0.0, 0.25, 1.0])
        self.assertEqual(runner.controller.z_bias, -0.035)

    def test_two_stage_height_uses_best_ascent_candidate_when_not_stable(self):
        self.config["vertical_repeated_mode"] = False
        class Link:
            def __init__(self):
                self.armed = False

            def arm(self):
                self.armed = True

            def close(self):
                pass

        link = Link()
        runner = CalibrationRunner(self.config, link=link)
        runner._prepare_height_ascent = lambda: None
        runner._run_height_ascent_stage = lambda: False
        continuous = []
        runner._run_height_continuous_stage = lambda: (
            continuous.append(True), setattr(runner, "status", "complete"))
        runner._save_summary = lambda: None
        runner.run_height_two_stage()
        self.assertFalse(link.armed)
        self.assertEqual(continuous, [True])
        self.assertEqual(runner.status, "needs_review")

    def test_two_stage_height_runs_continuous_phase_after_stable_ascent(self):
        self.config["vertical_repeated_mode"] = False
        class Link:
            def __init__(self):
                self.armed = False
                self.disarmed = False

            def arm(self):
                self.armed = True

            def disarm(self):
                self.disarmed = True

            def close(self):
                pass

        link = Link()
        runner = CalibrationRunner(self.config, link=link)
        runner._prepare_height_ascent = lambda: None
        runner._run_height_ascent_stage = lambda: True
        runner.initial_sample = Sample(0, 0, 0, -0.2, 0)
        runner.controller.z_bias = -0.04
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner.last_sample = Sample(time.monotonic(), 0, 0, 0, 0)

        def tune(stage, name, axis):
            self.assertTrue(link.armed)
            self.assertEqual(runner.controller.z_bias, -0.2)
            self.assertEqual((stage, name, axis), ("height", "pid_height", "z"))
            runner.stage_summary[name] = {"stable": True}
            return True

        runner._tune_pid = tune
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_height_two_stage()
        self.assertTrue(link.disarmed)
        self.assertEqual(runner.status, "complete")

    def test_continuous_height_mode_skips_ascent_gate(self):
        class Link:
            def __init__(self):
                self.armed = False

            def arm(self):
                self.armed = True

            def disarm(self):
                pass

            def close(self):
                pass

        link = Link()
        runner = CalibrationRunner(self.config, link=link)
        runner._prepare_height_ascent = lambda: None
        runner._run_height_ascent_stage = lambda: self.fail("ascent gate was called")
        runner._save_summary = lambda: None
        runner._takeoff = lambda: None
        runner._tune_pid = lambda stage, name, axis: False
        runner._throttle_ramp_land = lambda label: {
            "grounded": True, "final_height_above_launch": 0.0,
            "final_vertical_speed": 0.0, "telemetry_csv": "landing.csv"}
        runner.run_height_two_stage(continuous_only=True)
        self.assertTrue(link.armed)
        self.assertEqual(runner.status, "needs_review")

    def test_ascent_search_tries_both_gain_directions_without_duplicates(self):
        self.config["height_ascent_search_rounds"] = 1
        self.config["height_ascent_extra_rounds"] = 1
        runner = CalibrationRunner(self.config)
        runner._save_summary = lambda: None
        seen = []
        applied = []
        runner.controller.set_configs = lambda configs: applied.append(
            copy.deepcopy(configs))

        def evaluate(candidate, base, label, plans):
            gains = candidate["pid_height"]
            seen.append((label, base, gains["kp"], gains["ki"], gains["kd"]))
            return {
                "score": gains["kp"] * 10,
                "stable": False,
                "gains": {key: gains[key] for key in ("kp", "ki", "kd")},
                "trials": [{"final_bias": 0.1, "oscillation_rms": 0.1,
                            "peak_climb_speed": 1.0}],
            }

        runner._evaluate_ascent_candidate = evaluate
        self.assertFalse(runner._run_height_ascent_stage())
        kp_trials = {label: kp for label, _, kp, _, _ in seen}
        self.assertAlmostEqual(kp_trials["ascent_r0_kp_0"], 3.6)
        self.assertAlmostEqual(kp_trials["ascent_r0_kp_1"], 5.625)
        self.assertTrue(any(label.startswith("ascent_extra") for label in kp_trials))
        self.assertFalse(any(label.startswith("ascent_ki") for label in kp_trials))
        self.assertEqual(len(seen), len({values[1:] for values in seen}))
        self.assertEqual(applied[-1]["pid_height"], runner.best["pid_height"])

    def test_resume_height_restores_gains_and_base_without_replacing_pid_function(self):
        with tempfile.TemporaryDirectory() as directory:
            source = CalibrationRunner(self.config)
            source.run_dir = Path(directory)
            source.best["pid_height"]["kp"] = 2.3
            source.best_height_base_rc = 1630
            source._save_summary()
            resumed = CalibrationRunner(self.config)
            resumed.resume_height_from(Path(directory))
            self.assertEqual(resumed.best["pid_height"]["kp"], 2.3)
            self.assertEqual(resumed.best_height_base_rc, 1630)
            self.assertTrue(callable(resumed.best["pid_height"]["processing_func"]))


if __name__ == "__main__":
    unittest.main()
