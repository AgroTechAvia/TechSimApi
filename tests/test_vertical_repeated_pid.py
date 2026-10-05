from __future__ import annotations

from agrotechsimapi.calibration.engine import (
    CalibrationRunner,
    Sample,
    _height_repeated_d_ready,
    _height_repeated_i_ready,
    _height_repeated_p_ready,
    _yaw_repeated_d_ready,
    _yaw_repeated_i_ready,
    _yaw_repeated_p_ready,
    default_config,
)
from agrotechsimapi.calibration.reports import _vertical_pid_chart


def test_height_phase_gates_use_three_flights_and_requested_thresholds():
    config = default_config()
    p = {"reached_trials": 3, "mean_arrival_seconds": 2.8}
    d = {"reached_trials": 3, "settled_trials": 3,
         "plateau_hold_fraction": 0.90, "oscillation_rms": 0.02}
    i = {"reached_trials": 3, "target_hold_fraction": 0.72}
    assert _height_repeated_p_ready(p, config)
    assert _height_repeated_d_ready(d, config)
    assert _height_repeated_i_ready(i, config)
    p["mean_arrival_seconds"] = 3.1
    d["oscillation_rms"] = 0.031
    i["target_hold_fraction"] = 0.69
    assert not _height_repeated_p_ready(p, config)
    assert not _height_repeated_d_ready(d, config)
    assert not _height_repeated_i_ready(i, config)


def test_yaw_phase_gates_require_both_signed_three_flight_means():
    config = default_config()
    part = {"reached_count": 3, "mean_arrival_seconds": 4.0,
            "settled_count": 3, "plateau_hold_fraction": 0.9,
            "oscillation_rms": 0.02, "target_hold_fraction": 0.75}
    metrics = {"mean_response_by_direction": {
        "positive": dict(part), "negative": dict(part)}}
    assert _yaw_repeated_p_ready(metrics, config)
    assert _yaw_repeated_d_ready(metrics, config)
    assert _yaw_repeated_i_ready(metrics, config)
    metrics["mean_response_by_direction"]["negative"]["reached_count"] = 2
    assert not _yaw_repeated_p_ready(metrics, config)
    assert not _yaw_repeated_d_ready(metrics, config)
    assert not _yaw_repeated_i_ready(metrics, config)


def test_vertical_d_precheck_stops_before_changing_zero_gain(tmp_path):
    config = default_config()
    config["output_dir"] = str(tmp_path)
    runner = CalibrationRunner(config, link=object())
    working = runner.best
    probes = []

    def evaluate(base, gain, value, phase, label, score_start):
        probes.append(value)
        return {"metrics": {"score": 1.0, "ready": True},
                "telemetry_csv": "trial.csv"}

    result, history = runner._vertical_three_pass_search(
        "height", "pid_height", "kd", "d", 0.2, working,
        evaluate, lambda metrics, cfg: metrics["ready"])
    assert result is not None
    assert probes == [0.0]
    assert len(history) == 1
    assert working["pid_height"]["kd"] == 0.0


def test_vertical_search_never_exceeds_total_coefficient_budget(tmp_path):
    config = default_config()
    config["output_dir"] = str(tmp_path)
    config["vertical_trials_per_coefficient"] = 15
    config["vertical_search_worse_streak"] = 99
    runner = CalibrationRunner(config, link=object())
    probes = []

    def evaluate(base, gain, value, phase, label, score_start):
        probes.append(value)
        return {"metrics": {"score": abs(value - 0.7)},
                "telemetry_csv": "trial.csv"}

    runner._vertical_three_pass_search(
        "yaw", "pid_yaw", "kd", "d", 0.1, runner.best,
        evaluate, lambda metrics, cfg: False)
    assert len(probes) <= 15
    assert any(value > 0 for value in probes)


def test_vertical_search_reverses_after_two_scores_worse_than_best(tmp_path):
    config = default_config()
    config["output_dir"] = str(tmp_path)
    runner = CalibrationRunner(config, link=object())
    probes = []

    def evaluate(base, gain, value, phase, label, score_start):
        probes.append(value)
        return {"metrics": {"score": (value - .25) ** 2},
                "telemetry_csv": "trial.csv"}

    runner._vertical_three_pass_search(
        "yaw", "pid_yaw", "kp", "p", .25, runner.best,
        evaluate, lambda metrics, cfg: False)
    coarse_end = probes.index(.75)
    assert probes[coarse_end + 1] == .1875
    assert .2675 in probes


def test_vertical_pid_charts_expose_phase_specific_scoring_regions():
    traces = {
        cycle: [(0.0, 0.0), (1.0, 0.6 + cycle * .01),
                (2.0, 1.0), (3.0, 1.01), (4.0, 1.0)]
        for cycle in (1, 2, 3)
    }
    common = dict(traces=traces, target=1.0, title="Height", unit="m",
                  target_band=.07, arrival_time=4.0, arrival_tolerance=1.0)
    p_chart = _vertical_pid_chart(**common, phase="p")
    d_chart = _vertical_pid_chart(**common, phase="d", settle_time=2.0,
                                  plateau_center=1.0)
    i_chart = _vertical_pid_chart(**common, phase="i", score_start=2.0)
    assert 'class="arrival-window"' in p_chart
    assert 'class="arrival"' in p_chart
    assert p_chart.count('class="repeat repeat-') == 3
    assert 'class="transient"' in d_chart
    assert 'class="band"' in d_chart
    assert 'class="filtered"' in d_chart
    assert 'class="score-marker"' in i_chart
    assert 'class="mean-outside"' in i_chart
    assert 'class="mean-inside"' in i_chart
    assert i_chart.count('repeat-muted') == 3


def test_height_p_times_from_liftoff_and_keeps_recording_after_arrival(tmp_path):
    config = default_config()
    config["output_dir"] = str(tmp_path)

    class Link:
        stream_error = None

        def arm(self):
            pass

        def disarm(self):
            pass

        def set_frame(self, frame):
            pass

    runner = CalibrationRunner(config, link=Link())
    runner._save_summary = lambda: None
    ground = Sample(100.0, 0, 0, 0, 0,
                    vx_world=0, vy_world=0, vz_world=0)
    runner._wait_for_ground_return = lambda stage: setattr(
        runner, "last_sample", ground)
    written = {}

    def segment(stage, target, seconds, label, rows, **kwargs):
        samples = ((100.0, 0.00), (101.0, 0.02), (102.0, 0.03),
                   (103.5, 0.94), (105.0, 1.02))
        for absolute, height in samples:
            row = {"t": absolute, "segment_elapsed": absolute-100.0,
                   "z": height, "roll": 0.0, "pitch": 0.0}
            rows.append(row)
            if kwargs["stop_when"](rows):
                break
        runner.last_sample = Sample(105.0, 0, 0, 1.02, 0,
                                    vx_world=0, vy_world=0, vz_world=0)

    def write_csv(label, rows):
        written[label] = list(rows)
        return f"{label}.csv"

    runner._segment = segment
    runner._write_csv = write_csv
    runner._throttle_ramp_land = lambda label: {
        "grounded": True, "telemetry_csv": f"{label}.csv"}
    result = runner._height_pid_flight(runner.best, "p", "height_p_test")
    assert result["liftoff_detected"]
    assert result["arrival_time"] == 1.5
    assert len(written["height_p_test"]) == 5
    assert written["height_p_test"][-1]["t"] == 105.0


def test_height_validation_is_published_as_graphable_evaluation(monkeypatch, tmp_path):
    config = default_config()
    config["output_dir"] = str(tmp_path)

    class Link:
        def arm(self):
            pass

        def disarm(self):
            pass

    runner = CalibrationRunner(config, link=Link())
    runner._save_summary = lambda: None
    ground = Sample(0, 0, 0, 0, 0,
                    vx_world=0, vy_world=0, vz_world=0)
    runner._wait_for_ground_return = lambda stage: setattr(
        runner, "last_sample", ground)
    runner._profile_height_waypoints = lambda label, targets, allow_low: ([
        {"z": 1.0, "roll": 0.0, "pitch": 0.0, "waypoint_index": 1}],
        "height_pid_validation.csv")
    runner._throttle_ramp_land = lambda label: {"grounded": True}
    metrics = {"score": 1.0, "waypoints": [{"reached": True}]}
    monkeypatch.setattr("agrotechsimapi.calibration.engine.score_height_waypoints",
                        lambda rows, cfg, targets: dict(metrics))
    monkeypatch.setattr("agrotechsimapi.calibration.engine._height_stage2_stable",
                        lambda result, cfg: True)
    result = runner._height_validation_repeated_pid()
    record = runner.records[-1]
    assert result["passed"]
    assert record["label"] == "height_pid_validation"
    assert record["valid"]
    assert record["metrics"]["method"] == "height_waypoint_validation"
    assert record["telemetry_csv"] == "height_pid_validation.csv"

