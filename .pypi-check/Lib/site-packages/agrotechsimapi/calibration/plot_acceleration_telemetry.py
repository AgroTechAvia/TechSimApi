"""View completed acceleration calibration trials as separate browser plots.

Only the Python standard library is required. Run this while calibration is
active; --watch regenerates the page when a completed trial appears.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import time
from pathlib import Path

if __package__:
    from .acceleration_trace import align_repeats, settled_band_response, smoothed_mean
else:
    from acceleration_trace import align_repeats, settled_band_response, smoothed_mean


from .paths import data_dir

RUNS = data_dir() / "runs"
COLORS = ("#2563eb", "#d97706", "#7c3aed", "#0d9488", "#dc2626")


def resolve_run(value: str) -> Path:
    if value != "latest":
        path = Path(value).resolve()
    else:
        directories = sorted((path for path in RUNS.iterdir()
                              if path.is_dir() and (path / "summary.json").exists()),
                             reverse=True)
        if not directories:
            raise FileNotFoundError(f"No calibration runs in {RUNS}")
        path = directories[0]
    if not (path / "summary.json").is_file():
        raise FileNotFoundError(f"No summary.json in {path}")
    return path


def completed_trials(run: Path, pid: str, last: int) -> tuple[dict, list[dict]]:
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    trials = []
    for record in summary.get("evaluations", []):
        if (record.get("stage") != "acceleration" or not record.get("valid") or
                (pid != "both" and record.get("pid") != pid)):
            continue
        filename = record.get("telemetry_csv")
        if not filename or not (run / filename).is_file():
            continue
        trials.append(record)
    return summary, trials[-last:]


def trial_rows(path: Path, axis: str) -> tuple[
        dict[str, dict[int, list[tuple[float, float]]]], list[int]]:
    groups: dict[str, dict[int, list[tuple[float, float]]]] = {
        "positive": {}, "negative": {}}
    key = f"a{axis}_body"
    pwm_key = "rc_pitch" if axis == "x" else "rc_roll"
    pwm_values = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            direction = row.get("segment")
            if direction not in groups or not row.get(key):
                continue
            try:
                elapsed = float(row["segment_elapsed"])
                acceleration = float(row[key])
                cycle = int(row.get("acceleration_cycle") or 1)
            except (ValueError, TypeError):
                continue
            if math.isfinite(elapsed) and math.isfinite(acceleration):
                groups[direction].setdefault(cycle, []).append(
                    (elapsed, acceleration))
                if row.get(pwm_key):
                    try:
                        pwm_values.append(int(float(row[pwm_key])))
                    except ValueError:
                        pass
    return groups, pwm_values


def averaged_trace(series: dict[int, list[tuple[float, float]]], target: float
                   ) -> tuple[list[tuple[float, float]], str]:
    """Align repeats in time; smooth only the displayed mean, not its error."""
    if sum(len(points) >= 2 for points in series.values()) < 2:
        return [], ""
    aligned = align_repeats(series)
    if not aligned:
        return [], ""
    errors = [value - target for elapsed, values in aligned
              if 3.0 <= elapsed <= 6.0 for value in values]
    window = "3–6 с"
    if not errors:
        window = "за пролёт"
        errors = [value - target for _, values in aligned for value in values]
    mean_error = sum(errors) / len(errors)
    mae = sum(abs(error) for error in errors) / len(errors)
    return smoothed_mean(aligned), (
        f"средняя ошибка {window}: {mean_error:+.3f} м/с² · MAE: {mae:.3f} м/с²")


def plot_svg(series: dict[int, list[tuple[float, float]]], target: float,
             band_fraction: float, title: str,
             plateau_time: float | None = None,
             show_band: bool = True, highlight_i: bool = False,
             arrival_limit: float = 2.0, hold_seconds: float = 6.0,
             min_tolerance: float = 0.01,
             plateau_band_fraction: float = 0.20) -> str:
    width, height = 600, 280
    left, right, top, bottom = 54, 14, 56, 42
    plot_width, plot_height = width - left - right, height - top - bottom
    band = abs(target) * band_fraction
    low, high = target - 2.0, target + 2.0
    duration = max((elapsed for points in series.values() for elapsed, _ in points),
                   default=1.0)
    duration = max(duration, 0.01)

    def x(value: float) -> float:
        return left + value / duration * plot_width

    def y(value: float) -> float:
        return top + (high - value) / (high - low) * plot_height

    def clipped_y(value: float) -> float:
        return min(max(y(value), top), height - bottom)

    mean_trace, error_text = averaged_trace(series, target)
    filtered = (settled_band_response(
        mean_trace, target, max(band, min_tolerance),
        max(abs(target) * plateau_band_fraction, min_tolerance), arrival_limit)
        if highlight_i and mean_trace else None)
    if filtered:
        if filtered["settled"]:
            error_text = (f"после успокоения: в полосе {filtered['hold_fraction']:.0%} · "
                          f"ошибка {filtered['bias']:+.3f} м/с² · "
                          f"MAE {filtered['mae']:.3f} м/с²")
        else:
            error_text = "уровень не успокоился; попадание в полосу не засчитано"
    pieces = [f'<svg viewBox="0 0 {width} {height}" role="img" '
              f'aria-label="{html.escape(title, quote=True)}">',
              f'<text class="plot-title" x="{left}" y="20">{html.escape(title)}</text>']
    if error_text:
        pieces.append(f'<text class="plot-metrics" x="{left}" y="36">'
                      f'Чёрная — средняя {len(series)} пролётов '
                      f'(сглаживание 0,25 с)</text>')
        pieces.append(f'<text class="plot-metrics" x="{left}" y="50">'
                      f'{html.escape(error_text)}</text>')
    if plateau_time is not None and math.isfinite(plateau_time):
        marker = x(min(max(plateau_time, 0.0), duration))
        pieces.append(f'<rect class="transient" x="{left}" y="{top}" '
                      f'width="{marker - left:.1f}" height="{plot_height}"/>')
        pieces.append(f'<line class="plateau-marker" x1="{marker:.1f}" y1="{top}" '
                      f'x2="{marker:.1f}" y2="{height - bottom}"/>')
    if show_band:
        pieces.append(f'<rect class="band" x="{left}" y="{y(target + band):.1f}" '
                      f'width="{plot_width}" height="{y(target - band) - y(target + band):.1f}"/>')
    for tick in range(41):
        value = low + tick * 0.1
        ordinate = y(value)
        grid_class = "grid major" if tick % 5 == 0 else "grid minor"
        pieces.append(f'<line class="{grid_class}" x1="{left}" y1="{ordinate:.1f}" '
                      f'x2="{width - right}" y2="{ordinate:.1f}"/>')
        if tick % 5 == 0:
            pieces.append(f'<text class="tick" x="{left - 6}" y="{ordinate + 4:.1f}" '
                          f'text-anchor="end">{value:.1f}</text>')
    for tick in range(5):
        value = duration * tick / 4
        abscissa = x(value)
        pieces.append(f'<line class="grid" x1="{abscissa:.1f}" y1="{top}" '
                      f'x2="{abscissa:.1f}" y2="{height - bottom}"/>')
        pieces.append(f'<text class="tick" x="{abscissa:.1f}" y="{height - bottom + 18}" '
                      f'text-anchor="middle">{value:.1f}</text>')
    pieces.append(f'<line class="target" x1="{left}" y1="{y(target):.1f}" '
                  f'x2="{width - right}" y2="{y(target):.1f}"/>')
    for index, (cycle, points) in enumerate(sorted(series.items())):
        if not points:
            continue
        stride = max(1, math.ceil(len(points) / 700))
        sampled = points[::stride]
        if sampled[-1] != points[-1]:
            sampled.append(points[-1])
        coordinates = " ".join(f"{x(elapsed):.1f},{clipped_y(value):.1f}"
                               for elapsed, value in sampled)
        color = COLORS[index % len(COLORS)]
        repeat_class = ' class="repeat-muted"' if highlight_i else ''
        pieces.append(f'<polyline{repeat_class} fill="none" stroke="{color}" stroke-width="1.5" '
                      f'points="{coordinates}"/>')
        legend_x = left + index * 96
        pieces.append(f'<line x1="{legend_x}" y1="{height - 9}" '
                      f'x2="{legend_x + 18}" y2="{height - 9}" '
                      f'stroke="{color}" stroke-width="2"/>')
        pieces.append(f'<text class="tick" x="{legend_x + 23}" '
                      f'y="{height - 5}">пролёт {cycle}</text>')
    if mean_trace:
        coordinates = " ".join(f"{x(elapsed):.1f},{clipped_y(value):.1f}"
                               for elapsed, value in mean_trace)
        pieces.append(f'<polyline class="{"mean-trace-outside" if highlight_i else "mean-trace"}" '
                      f'points="{coordinates}"/>')
        if filtered and filtered["settled"]:
            start = filtered["score_start"]
            marker = x(start)
            pieces.append(f'<line class="settled-marker" x1="{marker:.1f}" '
                          f'y1="{top}" x2="{marker:.1f}" y2="{height - bottom}"/>')
            in_band: list[tuple[float, float]] = []

            def flush_band() -> None:
                if len(in_band) > 1:
                    points = " ".join(f"{x(elapsed):.1f},{clipped_y(value):.1f}"
                                      for elapsed, value in in_band)
                    pieces.append(f'<polyline class="mean-trace-in-band" '
                                  f'points="{points}"/>')
                elif in_band:
                    elapsed, value = in_band[0]
                    pieces.append(f'<circle class="mean-in-band-point" '
                                  f'cx="{x(elapsed):.1f}" '
                                  f'cy="{clipped_y(value):.1f}" r="2"/>')
                in_band.clear()

            for elapsed, value in mean_trace:
                if elapsed >= start and abs(value - target) <= max(band, min_tolerance):
                    in_band.append((elapsed, value))
                else:
                    flush_band()
            flush_band()
    pieces.append(f'<text class="axis-label" x="{width - right}" '
                  f'y="{height - bottom + 33}" text-anchor="end">время, с</text>')
    pieces.append('</svg>')
    return "".join(pieces)


def build_report(run: Path, pid: str = "pid_accel_pitch", last: int = 8) -> str:
    summary, trials = completed_trials(run, pid, last)
    fraction = float(summary.get("config", {}).get(
        "acceleration_settling_band_fraction", 0.20))
    cards = []
    for record in trials:
        name = str(record["pid"])
        axis = "x" if name == "pid_accel_pitch" else "y"
        filename = str(record["telemetry_csv"])
        groups, pwm_values = trial_rows(run / filename, axis)
        metrics = record.get("metrics") or {}
        gains = record.get("gains") or {}
        target = float(summary.get("config", {}).get("acceleration_step", 1.0))
        for direction, sign in (("positive", 1), ("negative", -1)):
            # Validation trials can use a different acceleration magnitude.
            part = metrics.get(direction) or {}
            if isinstance(part.get("target"), (int, float)):
                target = abs(float(part["target"]))
                break
        # The pink transient marker belongs to D; I uses the shared filtered
        # mean curve and its own settled scoring window.
        plateau_times = (metrics.get("plateau_time_by_direction") or {}
                         if metrics.get("phase") == "d" else {})
        phase = metrics.get("phase")
        settings = summary.get("config", {})
        plots = "".join(
            plot_svg(groups[direction], sign * target, fraction,
                     f"{'Положительная' if sign > 0 else 'Отрицательная'} ступень · "
                     f"цель {sign * target:+.2f} м/с²" +
                     (f" · допуск ±{fraction:.0%}" if phase != "p" else ""),
                     plateau_times.get(direction), show_band=phase != "p",
                     highlight_i=phase == "i",
                     arrival_limit=float(settings.get(
                         "acceleration_i_earliest_score_seconds", 2.0)),
                     hold_seconds=float(settings.get("acceleration_d_hold_seconds", 6.0)),
                     min_tolerance=float(settings.get("acceleration_min_tolerance", 0.01)),
                     plateau_band_fraction=float(settings.get(
                         "acceleration_plateau_band_fraction", 0.20)))
            for direction, sign in (("positive", 1), ("negative", -1)))
        score = metrics.get("score")
        score_text = f"{score:.3f}" if isinstance(score, (int, float)) else "—"
        if phase == "i" and "i_filtered_response_by_direction" not in metrics:
            score_text += " (прежняя оценка)"
        gains_text = " · ".join(
            f"{key.upper()}={float(gains[key]):.4g}" for key in ("kp", "kd", "ki")
            if key in gains)
        hold = metrics.get("i_filtered_hold_fraction")
        hold_text = (f" · средняя в полосе {hold:.0%}" if isinstance(hold, (int, float))
                     and metrics.get("phase") == "i" else "")
        plateau_time = metrics.get("plateau_settling_time")
        plateau_text = (f" · выход на уровень {plateau_time:.2f} с"
                        if isinstance(plateau_time, (int, float)) and
                        metrics.get("phase") == "d" else "")
        levels = metrics.get("plateau_level_fraction_by_direction") or {}
        level_text = (f" · уровень +/− {levels['positive']:.0%}/{levels['negative']:.0%}"
                      if metrics.get("phase") == "d" and
                      all(key in levels for key in ("positive", "negative")) else "")
        pwm_limit = int(summary.get("config", {}).get("max_accel_rc_offset", 80))
        pwm_text = (f" · PWM {min(pwm_values)}…{max(pwm_values)} мкс"
                    f" · на пределе {sum(abs(value - 1500) >= pwm_limit for value in pwm_values) / len(pwm_values):.0%}"
                    if pwm_values else "")
        cards.append(
            '<section class="trial">'
            f'<h2>{html.escape(str(record["label"]))}</h2>'
            f'<p>{html.escape(gains_text)} · score={score_text}'
            f'{hold_text}{plateau_text}{level_text}{pwm_text}</p>'
            f'<p class="file">{html.escape(filename)}</p>'
            f'<div class="plots">{plots}</div></section>')
    content = "".join(cards) if cards else (
        '<p>Завершённых проб для выбранного регулятора пока нет.</p>')
    return f'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta http-equiv="refresh" content="5">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Калибровка ускорения · {html.escape(run.name)}</title>
<style>
body {{ font: 15px system-ui, sans-serif; background:#f4f6fa; color:#172033;
       margin:0 auto; max-width:1300px; padding:20px; }}
h1 {{ margin:0 0 6px; font-size:24px; }}
.intro {{ color:#526078; margin:0 0 20px; }}
.trial {{ background:white; border:1px solid #dbe1e9; border-radius:10px;
          padding:15px; margin:0 0 18px; box-shadow:0 2px 8px #12213b0b; }}
.trial h2 {{ font-size:17px; margin:0 0 5px; overflow-wrap:anywhere; }}
.trial p {{ margin:3px 0; }}
.file {{ color:#69758b; font-size:12px; overflow-wrap:anywhere; }}
.plots {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px; }}
svg {{ display:block; width:100%; background:#fff; border:1px solid #e4e8ef;
       border-radius:6px; }}
.band {{ fill:#bbf7d0; opacity:.7; }}
.transient {{ fill:#fbcfe8; opacity:.32; }}
.plateau-marker {{ stroke:#db2777; stroke-width:1.5; stroke-dasharray:5 4; }}
.settled-marker {{ stroke:#111827; stroke-width:1.2; stroke-dasharray:4 3;
                   opacity:.65; }}
.mean-trace-outside {{ fill:none; stroke:#111827; stroke-width:2.8;
                       stroke-dasharray:6 4; stroke-linejoin:round; }}
.mean-trace-in-band {{ fill:none; stroke:#111827; stroke-width:3.5;
                       stroke-linecap:round; stroke-linejoin:round; }}
.mean-in-band-point {{ fill:#111827; }}
.repeat-muted {{ opacity:.35; }}
.target {{ stroke:#15803d; stroke-width:1.5; stroke-dasharray:6 4; }}
.grid.major {{ stroke:#dbe3ed; stroke-width:1; }}
.grid.minor {{ stroke:#eef2f7; stroke-width:.5; }}
.tick {{ fill:#64748b; font-size:11px; }}
.plot-title {{ fill:#172033; font-size:13px; font-weight:600; }}
.plot-metrics {{ fill:#334155; font-size:10px; }}
.mean-trace {{ fill:none; stroke:#111827; stroke-width:2.8;
               stroke-linejoin:round; stroke-linecap:round; }}
.axis-label {{ fill:#64748b; font-size:11px; }}
@media(max-width:850px) {{ .plots {{ grid-template-columns:1fr; }} }}
</style></head><body>
<h1>Телеметрия контура ускорения</h1>
<p class="intro">Запуск {html.escape(run.name)} · {html.escape(pid)} · последние {last} проб.
На этапе P показана только цель; на D и I зелёная область показывает коридор
±{fraction:.0%} от цели. Розовая область на этапе D — среднее время до выхода
на устойчивый уровень. Вертикальная шкала: цель ±2 м/с², сетка через 0,1 м/с².
На этапе I чёрная линия сплошная в целевом коридоре после успокоения;
пунктир обозначает остальные участки. Чёрная вертикальная линия отмечает
начало оценки, не раньше 2-й секунды. Значения вне шкалы обрезаются у её края. Цветные линии —
повторные пролёты, чёрная — их средняя после совмещения по времени и
сглаживания за 0,25 с. Средняя ошибка со знаком и MAE рассчитаны по
несглаженным значениям за 3–6 с (для коротких проб — за весь доступный интервал).
Служебное торможение на график не включено. Страница обновляется каждые 5 секунд.</p>
{content}
</body></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", default="latest", help="Каталог запуска или latest")
    parser.add_argument("--pid", choices=("pid_accel_pitch", "pid_accel_roll", "both"),
                        default="pid_accel_pitch", help="Какой регулятор показать")
    parser.add_argument("--last", type=int, default=8,
                        help="Число последних завершённых проб (по умолчанию 8)")
    parser.add_argument("--output", type=Path,
                        help="Выходной HTML; по умолчанию внутри каталога запуска")
    parser.add_argument("--watch", type=float, default=0,
                        help="Обновлять HTML каждые N секунд до Ctrl+C")
    args = parser.parse_args()
    if args.last < 1 or args.watch < 0:
        parser.error("--last must be positive and --watch cannot be negative")
    run = resolve_run(args.run)
    output = args.output.resolve() if args.output else run / "acceleration_telemetry.html"
    try:
        while True:
            try:
                page = build_report(run, args.pid, args.last)
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(page, encoding="utf-8")
                print(f"Plots updated: {output}")
            except (OSError, json.JSONDecodeError) as exc:
                if not args.watch:
                    raise
                print(f"Report is being updated; retrying in {args.watch:g}s: {exc}")
            if not args.watch:
                break
            time.sleep(args.watch)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
