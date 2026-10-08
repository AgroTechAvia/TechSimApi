"""Self-contained, bilingual telemetry snapshots. No server or refresh loop."""
from __future__ import annotations
import csv
import gzip
import html
import json
import math
import re
from importlib import resources
from pathlib import Path

from .acceleration_trace import align_repeats, smoothed_mean
from .service import STAGES, read_session
from .store import CalibrationStore, timestamp

LABELS = {'height': ('Высота', 'Altitude', 'ALT'), 'yaw': ('Курс', 'Yaw', 'YAW'),
          'acceleration': ('Ускорение XY', 'XY acceleration', 'XY ACC'),
          'velocity': ('Скорость XY', 'XY velocity', 'XY VEL'),
          'position': ('Положение XY', 'XY position', 'XY POS')}


def _chart(lines: list[tuple[str, list[tuple[float, float]]]], title: str) -> str:
    width, height, left, top, bottom = 640, 290, 58, 38, 40
    points = [point for _, series in lines for point in series]
    if not points:
        return ''
    duration = max(max(t for t, _ in points), .001)
    low, high = min(v for _, v in points), max(v for _, v in points)
    pad = max((high-low)*.12, .02)
    low, high = low-pad, high+pad
    x = lambda t: left + t/duration*(width-left-18)
    y = lambda v: top + (high-v)/(high-low)*(height-top-bottom)
    chunks = [f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">',
              f'<text class="plot-title" x="{left}" y="23">{html.escape(title)}</text>']
    for i in range(5):
        v, t = low + (high-low)*i/4, duration*i/4
        chunks += [f'<line class="grid" x1="{left}" x2="622" y1="{y(v):.1f}" y2="{y(v):.1f}"/>',
                   f'<text class="tick" x="52" y="{y(v)+4:.1f}" text-anchor="end">{v:.2f}</text>',
                   f'<text class="tick" x="{x(t):.1f}" y="270" text-anchor="middle">{t:.1f}</text>']
    for cls, series in lines:
        stride = max(1, math.ceil(len(series)/1800))
        shown = series[::stride]
        if series and shown[-1] != series[-1]:
            shown.append(series[-1])
        coords = ' '.join(f'{x(t):.1f},{y(v):.1f}' for t, v in shown)
        chunks.append(f'<polyline class="{cls}" fill="none" points="{coords}"/>')
    chunks.append('<text class="tick" x="622" y="285" text-anchor="end">время, с</text></svg>')
    return ''.join(chunks)


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def _trace_line(series: list[tuple[float, float]], x, y, css: str) -> str:
    if not series:
        return ''
    stride = max(1, math.ceil(len(series) / 900))
    shown = series[::stride]
    if shown[-1] != series[-1]:
        shown.append(series[-1])
    points = ' '.join(f'{x(t):.1f},{y(value):.1f}' for t, value in shown)
    return f'<polyline class="{css}" fill="none" points="{points}"/>'


def _vertical_pid_chart(
        traces: dict[int, list[tuple[float, float]]], *, target: float,
        phase: str, title: str, unit: str, target_band: float,
        arrival_time: float, arrival_tolerance: float,
        settle_time: float | None = None,
        plateau_center: float | None = None,
        score_start: float | None = None) -> str:
    """Render P/D/I semantics for one height or yaw response."""
    traces = {cycle: sorted(series) for cycle, series in traces.items() if series}
    if not traces:
        return ''
    width, height, left, right, top, bottom = 640, 310, 58, 18, 52, 42
    aligned = align_repeats(traces)
    average = smoothed_mean(aligned)
    duration = max((series[-1][0] for series in traces.values()), default=1.0)
    values = [value for series in traces.values() for _, value in series]
    band_centers = [target]
    if plateau_center is not None and math.isfinite(plateau_center):
        band_centers.append(plateau_center)
    low = min(values + [center - target_band for center in band_centers])
    high = max(values + [center + target_band for center in band_centers])
    pad = max((high - low) * .10, target_band * .5, .02)
    low, high = low - pad, high + pad
    if math.isclose(low, high):
        low, high = low - 1, high + 1

    def x(value: float) -> float:
        return left + max(0.0, min(value, duration)) / max(duration, .001) * (
            width - left - right)

    def y(value: float) -> float:
        return top + (high - value) / (high - low) * (height - top - bottom)

    pieces = [f'<svg viewBox="0 0 {width} {height}" role="img" '
              f'aria-label="{html.escape(title, quote=True)}">',
              f'<text class="plot-title" x="{left}" y="22">{html.escape(title)}</text>']
    if phase == 'p':
        window_start = max(0.0, arrival_time - arrival_tolerance)
        window_end = min(duration, arrival_time + arrival_tolerance)
        pieces.append(f'<rect class="arrival-window" x="{x(window_start):.1f}" '
                      f'y="{top}" width="{max(0.0, x(window_end)-x(window_start)):.1f}" '
                      f'height="{height-top-bottom}"/>')
        pieces.append(f'<line class="arrival" x1="{x(arrival_time):.1f}" y1="{top}" '
                      f'x2="{x(arrival_time):.1f}" y2="{height-bottom}"/>')
    elif phase == 'd':
        center = target if plateau_center is None else plateau_center
        pieces.append(f'<rect class="band" x="{left}" y="{y(center+target_band):.1f}" '
                      f'width="{width-left-right}" '
                      f'height="{max(0.0, y(center-target_band)-y(center+target_band)):.1f}"/>')
        pieces.append(f'<line class="plateau" x1="{left}" y1="{y(center):.1f}" '
                      f'x2="{width-right}" y2="{y(center):.1f}"/>')
        if settle_time is not None and math.isfinite(settle_time):
            marker = x(settle_time)
            pieces.append(f'<rect class="transient" x="{left}" y="{top}" '
                          f'width="{max(0.0, marker-left):.1f}" '
                          f'height="{height-top-bottom}"/>')
            pieces.append(f'<line class="score-marker" x1="{marker:.1f}" y1="{top}" '
                          f'x2="{marker:.1f}" y2="{height-bottom}"/>')
    elif phase in ('i', 'validation'):
        marker = x(score_start or 0.0)
        pieces.append(f'<rect class="band" x="{marker:.1f}" '
                      f'y="{y(target+target_band):.1f}" '
                      f'width="{max(0.0, width-right-marker):.1f}" '
                      f'height="{max(0.0, y(target-target_band)-y(target+target_band)):.1f}"/>')
        if score_start is not None:
            pieces.append(f'<line class="score-marker" x1="{marker:.1f}" y1="{top}" '
                          f'x2="{marker:.1f}" y2="{height-bottom}"/>')
    for index in range(5):
        value = low + (high-low) * index / 4
        elapsed = duration * index / 4
        pieces += [f'<line class="grid" x1="{left}" x2="{width-right}" '
                   f'y1="{y(value):.1f}" y2="{y(value):.1f}"/>',
                   f'<text class="tick" x="{left-6}" y="{y(value)+4:.1f}" '
                   f'text-anchor="end">{value:+.2f}</text>',
                   f'<text class="tick" x="{x(elapsed):.1f}" y="{height-bottom+18}" '
                   f'text-anchor="middle">{elapsed:.1f}</text>']
    pieces.append(f'<line class="target" x1="{left}" y1="{y(target):.1f}" '
                  f'x2="{width-right}" y2="{y(target):.1f}"/>')
    muted = ' repeat-muted' if phase in ('i', 'validation') else ''
    for cycle, series in sorted(traces.items()):
        pieces.append(_trace_line(series, x, y, f'repeat repeat-{cycle}{muted}'))
    if phase == 'd':
        pieces.append(_trace_line(average, x, y, 'filtered'))
    elif phase in ('i', 'validation'):
        pieces.append(_trace_line(average, x, y, 'mean-outside'))
        inside: list[tuple[float, float]] = []
        for elapsed, value in average:
            if (elapsed >= (score_start or 0.0) and
                    abs(value-target) <= target_band):
                inside.append((elapsed, value))
            else:
                if len(inside) >= 2:
                    pieces.append(_trace_line(inside, x, y, 'mean-inside'))
                inside = []
        if len(inside) >= 2:
            pieces.append(_trace_line(inside, x, y, 'mean-inside'))
    pieces.append(f'<text class="axis-label" x="{width-right}" y="{height-6}" '
                  f'text-anchor="end">время, с · {html.escape(unit)}</text></svg>')
    return ''.join(pieces)


def _height_repeated_traces(run: Path, record: dict, summary: dict
                            ) -> dict[int, list[tuple[float, float]]]:
    traces: dict[int, list[tuple[float, float]]] = {}
    filenames = record.get('telemetry_csvs') or [record.get('telemetry_csv')]
    liftoff_threshold = float(summary.get('config', {}).get(
        'height_liftoff_threshold_m', .03))
    for cycle, filename in enumerate(filenames, 1):
        if not filename or Path(filename).name != filename or not (run/filename).is_file():
            continue
        samples = []
        with (run/filename).open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            if row.get('segment') not in ('height_response', 'height_hold'):
                continue
            try:
                absolute = float(row['t'])
                configured_target = float(summary.get('config', {}).get(
                    'height_pid_target_m', 1.0))
                ground_source = row.get('ground_z')
                if ground_source in (None, ''):
                    target_source = row.get('target_z')
                    ground = (float(target_source) - configured_target
                              if target_source not in (None, '') else
                              float(summary.get('z_bias', 0.0)))
                else:
                    ground = float(ground_source)
                value = float(row['z']) - ground
            except (TypeError, ValueError, KeyError):
                continue
            if math.isfinite(absolute) and math.isfinite(value):
                samples.append((absolute, value))
        liftoff = next((absolute for absolute, value in samples
                        if value >= liftoff_threshold), None)
        start = (liftoff if liftoff is not None else
                 samples[0][0] if samples else 0.0)
        points = [(absolute-start, value) for absolute, value in samples
                  if absolute >= start]
        if points:
            traces[cycle] = points
    return traces


def _yaw_repeated_traces(run: Path, record: dict, direction: str
                         ) -> dict[int, list[tuple[float, float]]]:
    filename = record.get('telemetry_csv')
    traces: dict[int, list[tuple[float, float]]] = {}
    if not filename or Path(filename).name != filename or not (run/filename).is_file():
        return traces
    with (run/filename).open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            if row.get('segment') != direction:
                continue
            try:
                cycle = int(row['yaw_cycle'])
                elapsed = float(row['segment_elapsed'])
                origin = float(row['yaw_origin'])
                value = math.degrees(_wrap_pi(float(row['yaw']) - origin))
            except (TypeError, ValueError, KeyError):
                continue
            if math.isfinite(elapsed) and math.isfinite(value):
                traces.setdefault(cycle, []).append((elapsed, value))
    return traces


def _vertical_report(run: Path, stage: str, last: int) -> str:
    summary = json.loads((run/'summary.json').read_text(encoding='utf-8'))
    settings = summary.get('config', {})
    records = [r for r in summary.get('evaluations', [])
               if r.get('stage') in (stage, 'height_ascent' if stage == 'height' else stage)
               and r.get('valid') and r.get('telemetry_csv')]
    # Runs created before waypoint validation became a first-class evaluation
    # still keep its CSV and metrics in stages.pid_height.validation. Surface
    # it in regenerated reports as well, so a completed flight need not be rerun.
    if stage == 'height' and not any(
            record.get('label') == 'height_pid_validation' for record in records):
        height_stage = summary.get('stages', {}).get('pid_height', {})
        validation = height_stage.get('validation') or {}
        validation_csv = validation.get('telemetry_csv')
        if (validation_csv and Path(validation_csv).name == validation_csv and
                (run/validation_csv).is_file()):
            validation_metrics = dict(validation.get('metrics') or {})
            validation_metrics.setdefault('method', 'height_waypoint_validation')
            validation_metrics.setdefault('phase', 'validation')
            records.append({
                'stage': 'height', 'pid': 'pid_height', 'axis': 'z',
                'label': 'height_pid_validation', 'valid': True,
                'gains': height_stage.get('recommended_gains') or {},
                'metrics': validation_metrics,
                'telemetry_csv': validation_csv,
            })
    cards = []
    for record in records[-last:] if last else records:
        metrics = record.get('metrics') or {}
        method = metrics.get('method')
        if method in ('height_repeated_pid_response', 'yaw_repeated_pid_response'):
            phase = metrics.get('phase', 'validation')
            gains = record.get('gains') or {}
            gain_text = ' · '.join(f'{key.upper()}={value:.4g}' for key, value in gains.items()
                                   if key in ('kp', 'ki', 'kd') and
                                   isinstance(value, (int, float)))
            score = metrics.get('score')
            score_text = f'{score:.3f}' if isinstance(score, (int, float)) else '—'
            plots = []
            if stage == 'height':
                traces = _height_repeated_traces(run, record, summary)
                trials = metrics.get('trials') or []
                centers = [trial.get('plateau', {}).get('center') for trial in trials]
                centers = [value for value in centers if isinstance(value, (int, float))]
                score_start = metrics.get('i_score_start_seconds')
                if score_start is None:
                    starts = [trial.get('i_score_start_seconds') for trial in trials]
                    starts = [value for value in starts if isinstance(value, (int, float))]
                    score_start = sum(starts)/len(starts) if starts else None
                target = float(settings.get('height_pid_target_m', 1.0))
                plots.append(_vertical_pid_chart(
                    traces, target=target, phase=phase,
                    title=f'Высота · цель {target:+.2f} м', unit='м',
                    target_band=float(settings.get(
                        'height_d_band_m' if phase == 'd' else 'height_i_band_m', .03)),
                    arrival_time=float(settings.get('height_p_arrival_seconds', 2.5)),
                    arrival_tolerance=float(settings.get(
                        'height_p_time_tolerance_seconds', .5)),
                    settle_time=metrics.get('plateau_settling_time'),
                    plateau_center=(sum(centers)/len(centers) if centers else None),
                    score_start=score_start))
                if phase == 'p':
                    details = (f"достигнуто {metrics.get('reached_trials', 0)}/"
                               f"{metrics.get('repeat_count', 0)} · среднее время "
                               f"{_number(metrics.get('mean_arrival_seconds'), 'с')}")
                elif phase == 'd':
                    details = (f"успокоено {metrics.get('settled_trials', 0)}/"
                               f"{metrics.get('repeat_count', 0)} · выход на уровень "
                               f"{_number(metrics.get('plateau_settling_time'), 'с')} · "
                               f"в коридоре {_percent(metrics.get('plateau_hold_fraction'))}")
                else:
                    details = (f"в целевом коридоре "
                               f"{_percent(metrics.get('target_hold_fraction'))} · "
                               f"MAE {_number(metrics.get('terminal_mae'), 'м')}")
            else:
                responses = metrics.get('mean_response_by_direction') or {}
                score_start = metrics.get('i_score_start_seconds')
                target_magnitude = float(settings.get('yaw_pid_target_deg', 75.0))
                for direction, sign, caption in (
                        ('positive', 1, 'Поворот вправо'),
                        ('negative', -1, 'Поворот влево')):
                    traces = _yaw_repeated_traces(run, record, direction)
                    direction_trials = [trial for trial in metrics.get('trials', [])
                                        if trial.get('direction') == direction]
                    centers = [trial.get('plateau', {}).get('center')
                               for trial in direction_trials]
                    centers = [value for value in centers
                               if isinstance(value, (int, float))]
                    response = responses.get(direction, {})
                    plots.append(_vertical_pid_chart(
                        traces, target=sign*target_magnitude, phase=phase,
                        title=f'{caption} · цель {sign*target_magnitude:+.0f}°', unit='°',
                        target_band=float(settings.get(
                            'yaw_d_band_deg' if phase == 'd' else 'yaw_i_band_deg', 3.0)),
                        arrival_time=float(settings.get('yaw_p_arrival_seconds', 4.0)),
                        arrival_tolerance=float(settings.get(
                            'yaw_p_time_tolerance_seconds', .25)),
                        settle_time=response.get('settling_time'),
                        plateau_center=(sign * sum(centers)/len(centers)
                                        if centers else None),
                        score_start=score_start))
                positive, negative = responses.get('positive', {}), responses.get('negative', {})
                if phase == 'p':
                    details = (f"приход +/−: "
                               f"{_number(positive.get('mean_arrival_seconds'), 'с')} / "
                               f"{_number(negative.get('mean_arrival_seconds'), 'с')}")
                elif phase == 'd':
                    details = (f"успокоение +/−: "
                               f"{_number(positive.get('settling_time'), 'с')} / "
                               f"{_number(negative.get('settling_time'), 'с')}")
                else:
                    details = (f"в целевом коридоре +/−: "
                               f"{_percent(positive.get('target_hold_fraction'))} / "
                               f"{_percent(negative.get('target_hold_fraction'))}")
            files = record.get('telemetry_csvs') or [record.get('telemetry_csv')]
            file_text = ', '.join(str(filename) for filename in files if filename)
            phase_name = {'p': 'P · время достижения', 'd': 'D · затухание',
                          'i': 'I · статическая ошибка',
                          'validation': 'Проверка'}.get(phase, phase.upper())
            cards.append(
                '<section class="trial"><h2>'+html.escape(record.get('label', ''))+'</h2>'
                f'<p><b>{html.escape(phase_name)}</b> · {gain_text} · score={score_text}</p>'
                f'<p>{html.escape(details)}</p><p class="file">{html.escape(file_text)}</p>'
                '<div class="plots">'+''.join(plots)+'</div></section>')
            continue
        filenames = (record.get('telemetry_csvs')
                     if stage == 'height' else None) or [record['telemetry_csv']]
        traces = []
        for filename in filenames:
            if Path(filename).name != filename or not (run/filename).is_file():
                continue
            with (run/filename).open(encoding='utf-8', newline='') as stream:
                rows = list(csv.DictReader(stream))
            actual, target, pwm = [], [], []
            start = None
            for row in rows:
                try:
                    t = float(row['t'])
                    start = t if start is None else start
                    if stage == 'height':
                        ground = float(row.get('ground_z') or summary.get('z_bias', 0))
                        a, b = float(row['z'])-ground, float(row['target_z'])-ground
                        rc = float(row['rc_throttle'])
                    else:
                        a, b = math.degrees(float(row['yaw'])), math.degrees(float(row['target_yaw']))
                        rc = float(row['rc_yaw'])
                    if not all(math.isfinite(v) for v in (t,a,b,rc)):
                        continue
                    actual.append((t-start,a)); target.append((t-start,b)); pwm.append((t-start,rc))
                except (KeyError, ValueError):
                    continue
            if actual:
                traces.append((filename, actual, target, pwm))
        if not traces:
            continue
        metrics = record.get('metrics') or {}
        score = metrics.get('score')
        score_text = f'{score:.3f}' if isinstance(score, (int,float)) else '—'
        gains = record.get('gains') or {}
        gain_text = ' · '.join(f'{k.upper()}={v:.4g}' for k,v in gains.items()
                               if k in ('kp','ki','kd') and isinstance(v,(int,float)))
        title = 'Высота над стартом, м' if stage == 'height' else 'Курс, °'
        file_text = ', '.join(item[0] for item in traces)
        cards.append('<section class="trial"><h2>'+html.escape(record.get('label',traces[-1][0]))+'</h2>'
                     f'<p>{gain_text} · score={score_text}</p><p class="file">{html.escape(file_text)}</p>'
                     '<div class="plots">'+_chart(
                         [(f'actual flight-{index}', item[1])
                          for index, item in enumerate(traces, 1)] +
                         [('target', traces[0][2])], title)+
                     _chart([(f'actual flight-{index}', item[3])
                             for index, item in enumerate(traces, 1)],
                            'Throttle, PWM' if stage == 'height' else 'Yaw, PWM')+
                     '</div></section>')
    return ''.join(cards)


def _number(value, suffix: str = '') -> str:
    return '—' if not isinstance(value, (int, float)) or not math.isfinite(value) \
        else f'{value:.2f}{suffix}'


def _percent(value) -> str:
    return '—' if not isinstance(value, (int, float)) or not math.isfinite(value) \
        else f'{value:.0%}'


def stage_content(run: Path, stage: str, last: int = 0) -> tuple[str,str]:
    if not (run/'summary.json').is_file():
        return '', ''
    if stage in ('height', 'yaw'):
        return '', _vertical_report(run, stage, last)
    from . import plot_acceleration_telemetry, plot_velocity_telemetry, plot_position_telemetry
    module = {'acceleration': plot_acceleration_telemetry, 'velocity': plot_velocity_telemetry,
              'position': plot_position_telemetry}[stage]
    document = module.build_report(run, 'both', last or 1000000)
    style = re.search(r'<style>(.*?)</style>', document, re.S).group(1)
    cards = ''.join(re.findall(r'<section class="trial">.*?</section>', document, re.S))
    return style, cards


def _bundled() -> dict:
    blob = resources.files(__package__).joinpath('data/edu-ext_report.json.gz').read_bytes()
    return json.loads(gzip.decompress(blob))


def _limited(cards: str, last: int) -> str:
    if not last:
        return cards
    return ''.join(re.findall(r'<section class="trial">.*?</section>', cards, re.S)[-last:])


def write_report(store: CalibrationStore, *, name: str | None = None,
                 current: bool = False, run: Path | None = None,
                 output: Path | None = None, last: int = 0, language='ru') -> Path:
    paths, content, styles = {}, {}, []
    base = store.load()
    bundled = _bundled()
    if name is not None:
        profile = store.load(name)
        title, status = profile['name'], profile['status']
        for stage, source in profile.get('sources', {}).items():
            if stage not in STAGES:
                continue
            run_id = source.get('run_id', '')
            if source.get('session_id'):
                directory, manifest = read_session_if_present(store, source['session_id'])
                if directory is not None and Path(run_id).name == run_id:
                    paths[stage] = directory/run_id
            elif run_id == base.get('sources', {}).get(stage, {}).get('run_id'):
                styles.append(bundled[stage]['style'])
                content[stage] = _limited(bundled[stage]['content'], last)
    elif current:
        pointer = json.loads((store.root/'active.json').read_text(encoding='utf-8'))
        directory, manifest = read_session(store, pointer['session_id'])
        title, status = manifest['name'], manifest['status']
        paths = {stage: directory/part for stage, part in manifest['runs'].items()
                 if stage in STAGES and Path(part).name == part}
    elif run is not None:
        directory = run.parent if run.is_file() else run
        summary = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
        title, status = directory.name, summary['status']
        for stage in STAGES:
            if any(r.get('stage') == stage for r in summary.get('evaluations', [])):
                paths[stage] = directory
            elif stage in summary.get('resume_sources', {}):
                source = Path(summary['resume_sources'][stage])
                paths[stage] = source.parent if source.is_file() else source
    else:
        raise ValueError('Choose a calibration, --current, or --run')
    for stage, directory in paths.items():
        style, cards = stage_content(directory, stage, last)
        styles.append(style)
        content[stage] = cards
    destination = output or store.root/'reports'/f'{title}.html'
    destination = destination.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(render_document(title, status, content, styles, language), encoding='utf-8')
    return destination


def read_session_if_present(store, session_id):
    try:
        return read_session(store, session_id)
    except FileNotFoundError:
        return None, None


def render_document(title: str, status: str, content: dict, styles: list[str], language='ru') -> str:
    assets = resources.files(__package__).joinpath('assets')
    css = assets.joinpath('report.css').read_text(encoding='utf-8')
    js = assets.joinpath('report.js').read_text(encoding='utf-8')
    nav, panels = [], []
    for i, stage in enumerate(STAGES):
        ru, en, short = LABELS[stage]
        count = content.get(stage, '').count('<section class="trial">')
        nav.append(f'<button type="button" role="tab" id="tab-{stage}" aria-controls="{stage}" '
                   f'aria-selected="{str(i == 0).lower()}" data-tab="{stage}"'
                   f' tabindex="{0 if i == 0 else -1}"><small>{short}</small>'
                   f'<span data-ru="{ru}" data-en="{en}">{ru}</span><b>{count}</b></button>')
        empty = '<div class="empty" data-ru="Телеметрии этого этапа пока нет. Импорт JSON переносит коэффициенты; графики требуют исходного отчёта." data-en="No telemetry for this stage. JSON imports carry coefficients; charts need the original report.">Телеметрии этого этапа пока нет. Импорт JSON переносит коэффициенты; графики требуют исходного отчёта.</div>'
        panels.append(f'<div id="{stage}" role="tabpanel" aria-labelledby="tab-{stage}" '
                      f'{"hidden" if i else ""}>{content.get(stage) or empty}</div>')
    clean_style = '\n'.join(styles)
    return f'''<!doctype html><html lang="{language}"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AgroTechSim · {html.escape(title)}</title><style>{clean_style}\n{css}</style></head>
<body><header><div class="brand"><span class="brand-icon">A</span>AgroTechSim<span class="brand-divider">/</span><span data-ru="Калибровка" data-en="Calibration">Калибровка</span></div>
<div class="languages" aria-label="Language"><button type="button" data-lang="ru">RU</button><button type="button" data-lang="en">EN</button></div></header>
<main><div class="hero"><div><p class="eyebrow" data-ru="ТЕЛЕМЕТРИЯ ПОЛЁТА" data-en="FLIGHT TELEMETRY">ТЕЛЕМЕТРИЯ ПОЛЁТА</p><h1>{html.escape(title)}</h1>
<p class="subtitle" data-ru="Снимок отчёта · обновляется только по команде" data-en="Report snapshot · regenerated only on command">Снимок отчёта · обновляется только по команде</p></div>
<div class="meta"><span class="status">{html.escape(status)}</span><time>{timestamp()}</time></div></div>
<nav role="tablist" aria-label="Calibration stages">{''.join(nav)}</nav>
<details class="guide"><summary data-ru="Как читать графики" data-en="Reading the charts">Как читать графики</summary>
<p data-ru="P: голубая область — допустимое окно времени, зелёный пунктир — цель. D: розовый участок — переходный процесс, зелёный — коридор установившегося уровня. I: полупрозрачные линии — три повтора, чёрная — их среднее; пунктиром отмечен выход из целевого коридора. Вертикальная черта показывает начало оценки. Служебное торможение исключено из оценки." data-en="P: the blue area is the allowed arrival-time window and the green dashed line is the target. D: pink marks the transient and green marks the steady-level band. I: translucent lines are the three repeats and black is their mean; dashed sections are outside the target band. The vertical marker starts scoring. Service braking is excluded from scoring.">P: голубая область — допустимое окно времени, зелёный пунктир — цель. D: розовый участок — переходный процесс, зелёный — коридор установившегося уровня. I: полупрозрачные линии — три повтора, чёрная — их среднее; пунктиром отмечен выход из целевого коридора. Вертикальная черта показывает начало оценки. Служебное торможение исключено из оценки.</p></details>
{''.join(panels)}<footer>AgroTechSim API · <span data-ru="Автономный отчёт — доступен без интернета" data-en="Self-contained report — works offline">Автономный отчёт — доступен без интернета</span></footer></main>
<script>{js}</script></body></html>'''
