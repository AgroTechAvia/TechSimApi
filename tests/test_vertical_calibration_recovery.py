"""Recovery from a failed landing or fall during vertical calibration."""
import time

from agrotechsimapi.calibration.engine import (
    CalibrationRunner, LandingIncomplete, Sample, TrialInvalid, default_config,
)


def test_height_ascent_replays_same_target_after_failed_landing(tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    attempts = []
    waits = []

    def flight(target, speed, label, candidate, base):
        attempts.append((target, speed, candidate['pid_height']['kp'], base))
        if len(attempts) == 1:
            runner.last_csv = 'failed_landing.csv'
            raise LandingIncomplete('not on ground')
        return {'score': 1.0, 'stable': True, 'reached': True,
                'final_bias': 0.0, 'oscillation_rms': 0.0,
                'landing': {'grounded': True}}

    runner._ascent_trial = flight
    runner._wait_for_ground_return = lambda stage: waits.append(stage)
    result = runner._evaluate_ascent_candidate(
        runner.best, 1620, 'same_candidate', [(1.05, 0.24)])
    assert attempts == [attempts[0], attempts[0]]
    assert waits == ['height_ascent']
    assert len(result['trials']) == 1
    assert result['score'] == 1.0
    assert runner.records[0]['valid'] is False
    assert runner.records[0]['telemetry_csv'] == 'failed_landing.csv'


def test_ground_wait_accepts_stable_nonzero_height_at_launch_area(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.launch_xy = (10.0, 20.0)
    runner.controller.z_bias = 1.0
    runner.last_sample = Sample(0, 10, 20, 1.107, 0, vx_world=0, vy_world=0, vz_world=0)

    class Link:
        def __init__(self):
            self.disarmed = False
            self.samples = iter([
                Sample(0.25, 14, 20, 1.107, 0, vx_world=0, vy_world=0, vz_world=0),
                Sample(0.75, 10, 20, 1.107, 0, vx_world=0, vy_world=0, vz_world=0),
                Sample(1.0, 10, 20, 1.107, 0, vx_world=0, vy_world=0, vz_world=0),
                Sample(1.6, 10, 20, 1.107, 0, vx_world=0, vy_world=0, vz_world=0),
            ])
            self.reads = 0

        def disarm(self):
            self.disarmed = True

        def read_sample(self):
            self.reads += 1
            return next(self.samples)

    link = Link()
    runner.link = link
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    runner._wait_for_ground_return('height_ascent')
    assert link.disarmed
    assert link.reads == 4
    assert runner.initial_sample == runner.last_sample
    assert runner.controller.z_bias == 1.107
    assert runner.status == 'created'


def test_height_ground_wait_uses_current_position_and_only_vertical_stillness(
        monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.launch_xy = (0.0, 0.0)

    class Link:
        def __init__(self):
            self.samples = iter([
                Sample(1.0, 25.0, -12.0, 2.4, 0,
                       vx_world=0.8, vy_world=-0.4, vz_world=0.0),
                Sample(1.6, 25.5, -12.2, 2.4, 0,
                       vx_world=0.8, vy_world=-0.4, vz_world=0.0),
            ])

        def disarm(self):
            pass

        def read_sample(self):
            return next(self.samples)

    runner.link = Link()
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    runner._wait_for_ground_return('height')
    assert runner.initial_sample.x == 25.5
    assert runner.initial_sample.y == -12.2
    assert runner.controller.z_bias == 2.4


def test_yaw_ground_return_preserves_session_altitude(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.session_start_z = -0.02
    runner.controller.z_bias = -0.02
    runner.launch_xy = (0, 0)
    runner.xy_reference_yaw = 0

    class Link:
        def disarm(self):
            pass

        def read_sample(self):
            return Sample(time.monotonic(), 0, 0, 0.4, 0,
                          vx_world=0, vy_world=0, vz_world=0)

    runner.link = Link()
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    runner._wait_for_ground_return('yaw')
    assert runner.initial_sample.z == 0.4
    assert runner.controller.z_bias == -0.02


def test_yaw_ground_return_offers_manual_continue_after_timeout(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    config['manual_continue_seconds'] = 15.0
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.session_start_z = -0.02
    runner.launch_xy = (0, 0)

    samples = iter([
        Sample(1.0, 20, 20, 1.0, 0, roll=1.0,
               vx_world=1, vy_world=1, vz_world=1),
        Sample(2.0, 8, 9, 0.4, 1.2,
               vx_world=0, vy_world=0, vz_world=0),
    ])

    class Link:
        def disarm(self):
            pass

        def read_sample(self):
            return next(samples)

    runner.link = Link()
    clock = iter((0.0, 16.0))
    monkeypatch.setattr(time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    prompts = []
    monkeypatch.setattr('builtins.input', lambda prompt: prompts.append(prompt) or '')
    runner._wait_for_ground_return('yaw')
    assert len(prompts) == 1
    assert runner.initial_sample.x == 8
    assert runner.initial_sample.y == 9
    assert runner.initial_sample.yaw == 1.2
    assert runner.controller.z_bias == -0.02
    assert runner.status == 'created'


def test_takeoff_uses_local_height_origin_but_fixed_yaw_origin(tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)

    class Link:
        def read_sample(self):
            return Sample(0, 0, 0, 0.4, 0,
                          vx_world=0, vy_world=0, vz_world=0)

    for yaw_mode, expected_origin in ((False, 0.4), (True, -0.02)):
        runner = CalibrationRunner(config, link=Link())
        runner._save_summary = lambda: None
        runner._write_csv = lambda label, rows: 'takeoff.csv'
        runner.height_waypoints_mode = True
        runner.yaw_waypoints_mode = yaw_mode
        runner.session_start_z = -0.02
        runner.anchor = Link().read_sample()
        targets = []

        def segment(stage, target, seconds, label, rows, **kwargs):
            targets.append(target['height'] + runner.controller.z_bias)
            runner.last_sample = Sample(1, 0, 0, targets[-1], 0)

        runner._segment = segment
        runner._takeoff()
        assert runner.takeoff_start_z == 0.4
        assert runner.controller.z_bias == expected_origin
        assert abs(targets[0] - (expected_origin + config['hover_height'] + .075)) < 1e-9
        assert runner.stage_summary['takeoff']['reached_height']


def test_ascent_target_is_relative_to_each_nonzero_takeoff_height(tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)

    class Link:
        def read_sample(self):
            return Sample(0, 0, 0, 1.107, 0)

        def arm(self):
            pass

    runner = CalibrationRunner(config, link=Link())
    runner.session_start_z = -0.02
    runner._save_summary = lambda: None
    runner._write_csv = lambda label, rows: 'trial.csv'
    commands = []

    def segment(stage, target, seconds, label, rows, **kwargs):
        if label == 'ascent':
            kwargs['target_update'](None, 1.0)
            commands.append(target['height'] + runner.controller.z_bias)

    runner._segment = segment
    runner._throttle_ramp_land = lambda label: {
        'grounded': False, 'telemetry_csv': 'landing.csv'}
    try:
        runner._ascent_trial(0.5, 0.2, 'test', runner.best, 1620)
    except LandingIncomplete:
        pass
    else:
        assert False, 'Expected the test landing to remain incomplete'
    assert runner.takeoff_start_z == 1.107
    assert runner.controller.z_bias == 1.107
    assert abs(commands[0] - 1.307) < 1e-9
    assert runner.session_start_z == -0.02


def test_xy_respawn_keeps_original_session_altitude(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.session_start_z = -0.02
    runner.controller.z_bias = -0.02
    runner.launch_xy = (0, 0)
    runner.last_sample = Sample(0, 0, 0, 0, 0)

    class Link:
        def disarm(self):
            pass

        def arm(self):
            pass

        def read_sample(self):
            return Sample(time.monotonic(), 0, 0, 0.4, 0,
                          vx_world=0, vy_world=0, vz_world=0)

    runner.link = Link()
    starts = []

    def takeoff():
        starts.append((runner.initial_sample.z, runner.controller.z_bias))
        runner.stage_summary['takeoff'] = {'reached_height': True}

    runner._takeoff = takeoff
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    runner._wait_for_xy_respawn('velocity')
    assert starts == [(0.4, -0.02)]


def test_xy_respawn_offers_manual_continue_and_replays_trial(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    config['manual_continue_seconds'] = 15.0
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner.session_start_z = -0.02
    runner.launch_xy = (0, 0)
    runner.last_sample = Sample(0, 30, 30, 0, 0,
                                vx_world=1, vy_world=1, vz_world=1)
    samples = iter([
        Sample(1, 30, 30, 0, 0, roll=1.0,
               vx_world=1, vy_world=1, vz_world=1),
        Sample(2, 12, -7, 0.4, 0.8,
               vx_world=0, vy_world=0, vz_world=0),
    ])

    class Link:
        def disarm(self):
            pass

        def arm(self):
            pass

        def read_sample(self):
            return next(samples)

    runner.link = Link()
    takeoffs = []

    def takeoff():
        takeoffs.append((runner.anchor.x, runner.anchor.y))
        runner.stage_summary['takeoff'] = {'reached_height': True}

    runner._takeoff = takeoff
    clock = iter((0.0, 16.0))
    monkeypatch.setattr(time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    monkeypatch.setattr('builtins.input', lambda prompt: '')
    runner._wait_for_xy_respawn('acceleration')
    assert takeoffs == [(12, -7)]
    assert runner.launch_xy == (12, -7)
    assert runner.controller.z_bias == -0.02
    assert runner.status == 'created'


def test_xy_overturn_is_treated_as_respawn_condition(tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner.takeoff_start_z = 0.0
    runner.last_sample = Sample(1, 0, 0, 9, 0, roll=1.0,
                                vx_world=0, vy_world=0, vz_world=0)
    assert runner._xy_drone_is_down('acceleration')
    assert runner._xy_drone_is_down('velocity')
    assert runner._xy_drone_is_down('position')


def test_yaw_final_landing_waits_without_discarding_tuned_gains(tmp_path):
    config = default_config()
    config['vertical_repeated_mode'] = False
    config['output_dir'] = str(tmp_path)

    class Link:
        def arm(self):
            pass

        def close(self):
            pass

    runner = CalibrationRunner(config, link=Link())
    runner._save_summary = lambda: None
    runner._prepare_height_ascent = lambda: None
    runner._takeoff = lambda: runner.stage_summary.update(
        takeoff={'reached_height': True})

    def tuned(stage, name, axis):
        runner.stage_summary[name] = {'stable': True, 'best_metrics': {'score': 1.0}}
        return True

    runner._tune_pid = tuned
    runner._throttle_ramp_land = lambda label: {
        'grounded': False, 'telemetry_csv': 'yaw_landing.csv',
        'final_height_above_launch': .107}
    waited = []
    runner._wait_for_ground_return = lambda stage: waited.append(stage)
    runner.run_yaw_only()
    assert waited == ['yaw']
    assert runner.failure is None
    assert runner.status == 'needs_review'
    assert runner.stage_summary['pid_yaw']['best_metrics']['score'] == 1.0
    assert runner.stage_summary['landing']['ground_return_confirmed']


def test_yaw_repeats_interrupted_trial_after_ground_and_takeoff(monkeypatch, tmp_path):
    config = default_config()
    config['output_dir'] = str(tmp_path)
    runner = CalibrationRunner(config)
    runner._save_summary = lambda: None
    runner._recover = lambda stage: None
    runner.yaw_waypoints_mode = True
    runner.controller.z_bias = 0.0
    runner.last_sample = runner.anchor = Sample(
        0, 0, 0, 9, 0, vx_world=0, vy_world=0, vz_world=0)
    flights = []
    returns = []

    def flight(stage, axis, label):
        flights.append(label)
        if len(flights) == 1:
            runner.last_sample = Sample(
                1, 0, 0, 0, 0, vx_world=0, vy_world=0, vz_world=0)
            raise TrialInvalid('Drone was below the airborne height')
        return [{}], 'valid.csv'

    def resume(stage):
        returns.append(stage)
        runner.last_sample = Sample(
            2, 0, 0, 9, 0, vx_world=0, vy_world=0, vz_world=0)

    runner._profile = flight
    runner._resume_vertical_after_ground = resume
    monkeypatch.setattr('agrotechsimapi.calibration.engine.score_yaw_waypoints',
                        lambda rows, c: {'score': 1.0, 'mae': 0.0,
                                         'terminal_mae': 0.0, 'max_overshoot': 0.0,
                                         'max_settling_time': 0.0,
                                         'oscillation_rms': 0.0})
    result = runner._evaluate('yaw', 'pid_yaw', 'yaw', runner.best, 'same_trial')
    assert flights == ['same_trial', 'same_trial']
    assert returns == ['yaw']
    assert [record['valid'] for record in runner.records] == [False, True]
    assert result['metrics']['score'] == 1.0
