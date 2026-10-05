"""Persistence and session tests; these never connect to a simulator."""
import copy
import json
from pathlib import Path

import pytest

from agrotechsimapi.__main__ import main
from agrotechsimapi.calibration import CalibrationStore
from agrotechsimapi.calibration.paths import data_dir
from agrotechsimapi.calibration.service import calibrate, NAMES
from agrotechsimapi.calibration.store import (BUILTIN_PRESETS, DEFAULT_PRESET,
                                               runtime_pids, validate_profile)


def test_default_roundtrip_between_stores(tmp_path):
    one, two = CalibrationStore(tmp_path/'one'), CalibrationStore(tmp_path/'two')
    assert [p['name'] for p in one.list()] == list(BUILTIN_PRESETS)
    profile = one.load()
    assert profile['name'] == DEFAULT_PRESET
    assert one.load('default') == profile
    assert one.load('edu-constructor')['pids'] == profile['pids']
    assert one.load('edu')['pids']['pid_height']['kp'] == pytest.approx(.2)
    assert set(profile['pids']) == set(NAMES['height']+NAMES['yaw']+NAMES['acceleration']+NAMES['velocity']+NAMES['position'])
    exported = one.export('default', tmp_path/'portable.json')
    two.import_file(exported, name='drone_2')
    loaded = two.load('drone_2')
    assert loaded['pids'] == profile['pids']
    assert loaded['config'] == profile['config']
    assert loaded['created_at'] == profile['created_at']
    assert callable(runtime_pids(loaded)['pid_height']['processing_func'])
    with pytest.raises(FileExistsError):
        two.import_file(exported, name='drone_2')
    with pytest.raises(ValueError):
        two.import_file(exported)


@pytest.mark.parametrize('name', [
    '../outside', 'foo/bar', 'C:\\oops', '', '..', 'CON', 'default',
    'edu-ext', 'edu-constructor', 'edu',
])
def test_no_path_escape_or_default_overwrite(tmp_path, name):
    profile = CalibrationStore(tmp_path).load()
    profile['name'] = name
    with pytest.raises(ValueError):
        CalibrationStore(tmp_path).save(profile)


@pytest.mark.parametrize('mutation', [
    lambda p: p['pids']['pid_vel_roll'].update(kp=float('nan')),
    lambda p: p['pids']['pid_height'].update(processing_func='os.system'),
    lambda p: p['pids'].pop('pid_accel_roll'),
    lambda p: p['config'].update(acceleration_control_hz=1000),
    lambda p: p.update(schema_version=2),
    lambda p: p.update(status='aborted'),
    lambda p: p.update(pids=None),
    lambda p: p['pids']['pid_height'].update(processing_func=[]),
])
def test_corrupt_profiles_rejected(tmp_path, mutation):
    profile = CalibrationStore(tmp_path).load()
    mutation(profile)
    with pytest.raises(ValueError):
        validate_profile(profile)


def test_linux_xdg_and_override(monkeypatch, tmp_path):
    monkeypatch.setattr('agrotechsimapi.calibration.paths.sys.platform', 'linux')
    monkeypatch.delenv('AGROTECHSIMAPI_DATA_DIR', raising=False)
    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path/'xdg'))
    assert data_dir() == tmp_path/'xdg/agrotechsimapi'
    monkeypatch.setenv('AGROTECHSIMAPI_DATA_DIR', str(tmp_path/'custom'))
    assert data_dir() == tmp_path/'custom'
    monkeypatch.delenv('AGROTECHSIMAPI_DATA_DIR')
    monkeypatch.setenv('XDG_DATA_HOME', 'relative-invalid')
    assert data_dir() == Path.home()/'.local/share/agrotechsimapi'


def test_preview_does_not_connect_or_create_store(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr('agrotechsimapi.calibration.engine.SimulatorLink.connect',
                        lambda _: pytest.fail('preview connected'))
    root = tmp_path/'store'
    assert main(['--data-dir',str(root),'calibrate','--name','new','--stage','velocity']) == 0
    assert not root.exists()
    assert 'Preview only' in capsys.readouterr().out


def fake_runner(monkeypatch, *, interrupted=False):
    from agrotechsimapi.calibration import engine
    counter = []
    class FakeRunner:
        def __init__(self, config):
            counter.append(config)
            self.config = config
            self.run_dir = Path(config['output_dir'])/f'run-{len(counter)}'
            class Controller:
                def set_configs(self, values):
                    pass
            self.controller = Controller()
        def run_xy_only(self, stage):
            self.run_dir.mkdir(parents=True)
            if stage == 'height':
                self.session_start_z = 1.25
            elif len(counter) > 1:
                assert self.session_start_z == 1.25
            for name in NAMES[stage]:
                self.best[name]['kp'] += 0.01
            data = {'status':'aborted' if interrupted else 'needs_review',
                    'failure':'KeyboardInterrupt' if interrupted else None,
                    'stages':{pid:{'best_metrics':{'score':1},'stable':False}
                              for pid in NAMES[stage]},
                    'recommended_pids':engine.printable_pids(self.best),
                    'started_at':'2026-09-28T12:00:00+00:00',
                    'config':self.config,'evaluations':[]}
            (self.run_dir/'summary.json').write_text(json.dumps(data),encoding='utf-8')
            return self.run_dir
        def run_height_two_stage(self, *, continuous_only=False):
            return self.run_xy_only('height')
        def run_yaw_only(self):
            return self.run_xy_only('yaw')
    monkeypatch.setattr(engine,'CalibrationRunner',FakeRunner)
    monkeypatch.setattr(engine,'inspect_run',lambda _:None)
    return counter


def test_finished_review_profile_saved_with_upstream_gains(monkeypatch,tmp_path):
    fake_runner(monkeypatch)
    store = CalibrationStore(tmp_path)
    calibrate(store,'new',stage='position',fly=True)
    result = store.load('new')
    default = store.load()
    assert result['status'] == 'needs_review'
    assert result['pids']['pid_height'] == default['pids']['pid_height']
    assert result['pids']['pid_pos_x']['kp'] > default['pids']['pid_pos_x']['kp']
    session = json.loads((tmp_path/'runs'/result['session_id']/'session.json').read_text())
    assert session['status'] == 'needs_review'
    assert session['finished_at']


def test_interrupted_run_cannot_replace_profile(monkeypatch,tmp_path):
    fake_runner(monkeypatch,interrupted=True)
    store = CalibrationStore(tmp_path)
    old = store.load(); old['name'] = 'keep'
    store.save(old)
    with pytest.raises(RuntimeError):
        calibrate(store,'keep',stage='velocity',fly=True,replace=True)
    assert store.load('keep') == old
    pointer = json.loads((tmp_path/'active.json').read_text())
    session = json.loads((tmp_path/'runs'/pointer['session_id']/'session.json').read_text())
    assert session['status'] == 'aborted'


def test_full_session_runs_in_order_and_keeps_gains_between_stages(monkeypatch,tmp_path):
    from agrotechsimapi.calibration.service import STAGES
    calls = fake_runner(monkeypatch)
    store = CalibrationStore(tmp_path)
    original = store.load()
    calibrate(store, 'all_stages', fly=True)
    result = store.load('all_stages')
    assert len(calls) == 5
    for index, stage in enumerate(STAGES, 1):
        assert result['sources'][stage]['run_id'] == f'run-{index}'
        for name in NAMES[stage]:
            assert result['pids'][name]['kp'] == pytest.approx(original['pids'][name]['kp'] + .01)
    assert len({source['session_id'] for source in result['sources'].values()}) == 1
    session = json.loads((tmp_path/'runs'/result['session_id']/'session.json').read_text())
    assert session['session_start_z'] == 1.25


def test_reports_offline_bilingual_and_current_pending(monkeypatch,tmp_path):
    from agrotechsimapi.calibration.reports import write_report
    store = CalibrationStore(tmp_path)
    path = write_report(store,name='default',last=1)
    text = path.read_text(encoding='utf-8')
    assert text.count('role="tabpanel"') == 5
    assert text.count('<section class="trial">') == 5
    assert 'data-lang="en"' in text and 'data-lang="ru"' in text
    assert 'http-equiv="refresh"' not in text
    assert '<script src=' not in text
    fake_runner(monkeypatch)
    calibrate(store,'new',stage='velocity',fly=True)
    pointer = json.loads((tmp_path/'active.json').read_text())
    summary = tmp_path/'runs'/pointer['session_id']/'run-1/summary.json'
    summary.unlink()  # First trial is not yet published.
    current = write_report(store,current=True)
    assert current.is_file()
