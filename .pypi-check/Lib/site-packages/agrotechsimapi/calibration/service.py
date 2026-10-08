"""Orchestrate named calibration sessions using the proven staged runner."""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
from uuid import uuid4

from .store import (DEFAULT_PRESET, CalibrationStore, atomic_json, finite_json,
                    runtime_pids, timestamp)

STAGES = ('height', 'yaw', 'acceleration', 'velocity', 'position')
NAMES = {'height': ('pid_height',), 'yaw': ('pid_yaw',),
         'acceleration': ('pid_accel_pitch', 'pid_accel_roll'),
         'velocity': ('pid_vel_pitch', 'pid_vel_roll'),
         'position': ('pid_pos_x', 'pid_pos_y')}


def calibrate(store: CalibrationStore, name: str, *, stage: str = 'all',
              base: str = DEFAULT_PRESET, config_file: Path | None = None,
              fly: bool = False, replace: bool = False,
              continuous_height: bool = False) -> Path | None:
    from .engine import CalibrationRunner, default_config, validate_config, inspect_run
    store.check_destination(name, replace=replace)
    profile = store.load(base)
    config = default_config()
    config.update(copy.deepcopy(profile['config']))
    if config_file:
        changes = json.loads(config_file.read_text(encoding='utf-8'))
        if not isinstance(changes, dict):
            raise ValueError('Calibration config must be a JSON object')
        unknown = changes.keys() - config.keys()
        if unknown:
            raise ValueError(f'Unknown config keys: {sorted(unknown)}')
        for key, value in changes.items():
            if isinstance(value, dict) and isinstance(config.get(key), dict):
                config[key].update(value)
            else:
                config[key] = value
    config['output_dir'] = str(store.root / 'runs')
    validate_config(config)
    if stage not in (*STAGES, 'all'):
        raise ValueError(f'Unknown stage: {stage}')
    plan = STAGES if stage == 'all' else (stage,)
    if not fly:
        print(json.dumps({'name': name, 'base': base, 'stages': plan,
                          'config': config}, ensure_ascii=False, indent=2))
        print('Preview only. Add --fly to connect and start calibration.')
        return None

    session_id = uuid4().hex
    session_dir = store.root / 'runs' / session_id
    session_dir.mkdir(parents=True)
    manifest_path = session_dir / 'session.json'
    manifest = {'schema_version': 1, 'name': name, 'id': session_id,
                'started_at': timestamp(), 'status': 'running', 'process_id': os.getpid(),
                'base': base, 'runs': {}, 'base_sources': profile.get('sources', {}),
                'failure': None}
    atomic_json(manifest_path, manifest)
    # One atomic pointer; the session itself remains available by id.
    atomic_json(store.root / 'active.json', {'session_id': session_id})
    pids = runtime_pids(profile)
    results = copy.deepcopy(profile.get('results', {}))
    sources = copy.deepcopy(profile.get('sources', {}))
    session_start_z = None
    status = 'complete'
    try:
        for current in plan:
            cfg = copy.deepcopy(config)
            cfg['output_dir'] = str(session_dir)
            runner = CalibrationRunner(cfg)
            runner.session_start_z = session_start_z
            runner.seed = copy.deepcopy(pids)
            runner.best = copy.deepcopy(pids)
            runner.best_height_base_rc = config['height_base_throttle_rc']
            runner.controller.set_configs(pids)
            runner.controller.base_throttle_rc = runner.best_height_base_rc
            manifest['current_stage'] = current
            manifest['runs'][current] = runner.run_dir.name
            atomic_json(manifest_path, manifest)
            if current == 'height':
                run = runner.run_height_two_stage(continuous_only=continuous_height)
            elif current == 'yaw':
                run = runner.run_yaw_only()
            else:
                run = runner.run_xy_only(current)
            if session_start_z is None:
                session_start_z = runner.session_start_z
            manifest['session_start_z'] = session_start_z
            atomic_json(manifest_path, manifest)
            inspect_run(run)
            summary = json.loads((run / 'summary.json').read_text(encoding='utf-8'))
            # needs_review may be a completed test with failed tolerances. A lost
            # connection, interrupt or missing trial must never publish a profile.
            complete_trials = all(summary['stages'].get(pid, {}).get('best_metrics')
                                  for pid in NAMES[current])
            if (summary['status'] not in ('complete', 'needs_review') or
                    summary.get('failure') or not complete_trials):
                raise RuntimeError(f'{current}: calibration did not finish: '
                                   f'{summary.get("failure") or summary["status"]}')
            pids = copy.deepcopy(runner.best)
            config['height_base_throttle_rc'] = runner.best_height_base_rc
            results[current] = summary['stages']
            sources[current] = {'run_id': run.name, 'created_at': summary['started_at'],
                                'status': summary['status'], 'session_id': session_id}
            if summary['status'] != 'complete':
                status = 'needs_review'
        from .engine import printable_pids
        # If a subset was tuned, retain the inherited validation status.
        if stage != 'all' and profile['status'] != 'complete':
            status = 'needs_review'
        saved_config = copy.deepcopy(config)
        for key in ('output_dir', 'host', 'sim_port', 'msp_port'):
            saved_config.pop(key, None)
        output = finite_json({'schema_version': 1, 'name': name,
                              'created_at': timestamp(), 'status': status,
                              'drone_name': config['drone_name'], 'config': saved_config,
                              'pids': printable_pids(pids), 'sources': sources,
                              'results': results, 'session_id': session_id})
        saved = store.save(output, replace=replace)
        manifest['status'] = status
        manifest['finished_at'] = output['created_at']
        return saved
    except BaseException as exc:
        manifest['status'] = 'aborted'
        manifest['failure'] = f'{type(exc).__name__}: {exc}'
        manifest['finished_at'] = timestamp()
        raise
    finally:
        atomic_json(manifest_path, manifest)


def read_session(store: CalibrationStore, session_id: str) -> tuple[Path, dict]:
    if len(session_id) != 32 or any(ch not in '0123456789abcdef' for ch in session_id):
        raise ValueError('Invalid session id')
    directory = store.root / 'runs' / session_id
    return directory, json.loads((directory / 'session.json').read_text(encoding='utf-8'))
