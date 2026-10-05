"""Build live HTML plots for completed XY velocity calibration trials.

Run while calibration is active with --watch 5, or render once after a run.
Only the Python standard library is required.
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
    from .acceleration_trace import align_repeats, smoothed_mean
else:
    from acceleration_trace import align_repeats, smoothed_mean


from .paths import data_dir

RUNS = data_dir() / "runs"


def resolve_run(value: str) -> Path:
    if value != "latest":
        run = Path(value).resolve()
    else:
        run = next((candidate for candidate in sorted(RUNS.iterdir(), reverse=True)
                    if candidate.is_dir() and (candidate / "summary.json").is_file()
                    and any(record.get("stage") == "velocity" for record in
                            json.loads((candidate / "summary.json").read_text(
                                encoding="utf-8")).get("evaluations", []))), None)
        if run is None:
            raise FileNotFoundError("No velocity calibration run found")
    if not (run / "summary.json").is_file():
        raise FileNotFoundError(f"No summary.json in {run}")
    return run


def completed_trials(run: Path, pid: str, last: int) -> tuple[dict, list[dict]]:
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    records = [record for record in summary.get("evaluations", [])
               if record.get("stage") == "velocity" and record.get("valid")
               and (pid == "both" or record.get("pid") == pid)
               and record.get("telemetry_csv")
               and (run / record["telemetry_csv"]).is_file()]
    return summary, records[-last:]


def read_trace(path: Path, axis: str, direction: str) -> tuple[list[tuple[float, float]],
                                                               list[tuple[float, float]], float]:
    """Return flight and braking in one time frame, using the measured axis."""
    flight, brake = [], []
    target = 0.0
    velocity_key = f"v{axis}_body"
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            segment = row.get("segment")
            if segment not in (direction, f"{direction}_stop"):
                continue
            try:
                elapsed = float(row["segment_elapsed"])
                velocity = float(row[velocity_key])
            except (TypeError, ValueError, KeyError):
                continue
            if not math.isfinite(elapsed) or not math.isfinite(velocity):
                continue
            if segment == direction:
                flight.append((elapsed, velocity))
                # Tuning trials annotate their command explicitly. Range
                # confirmations use the regular flight profile instead.
                for target_key in ("commanded_axis_speed", f"target_v{axis}_body"):
                    try:
                        target = float(row[target_key])
                        break
                    except (TypeError, ValueError, KeyError):
                        continue
            else:
                brake.append((elapsed, velocity))
    return flight, brake, target


def smooth(points: list[tuple[float, float]], seconds: float = 0.25
           ) -> list[tuple[float, float]]:
    values = []
    output = []
    for elapsed, velocity in points:
        values.append((elapsed, velocity))
        while values and elapsed - values[0][0] > seconds:
            values.pop(0)
        output.append((elapsed, sum(value for _, value in values) / len(values)))
    return output


def read_repeated_traces(path: Path, axis: str, direction: str, speed: float
                         ) -> dict[int, list[tuple[float, float]]]:
    traces: dict[int, list[tuple[float, float]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("segment") != direction:
                continue
            try:
                if not math.isclose(float(row["velocity_requested_speed"]), speed,
                                    rel_tol=0, abs_tol=1e-8):
                    continue
                cycle = int(row["velocity_cycle"])
                elapsed = float(row["segment_elapsed"])
                vx, vy, yaw = (float(row[key]) for key in
                               ("vx_world", "vy_world", "yaw"))
                velocity = (vx * math.cos(yaw) - vy * math.sin(yaw)
                            if axis == "x" else
                            vx * math.sin(yaw) + vy * math.cos(yaw))
            except (TypeError, ValueError, KeyError):
                continue
            if math.isfinite(elapsed) and math.isfinite(velocity):
                traces.setdefault(cycle, []).append((elapsed, velocity))
    return traces


def repeated_plot_svg(traces: dict[int, list[tuple[float, float]]], target: float,
                      phase: str, settings: dict, title: str,
                      arrival_target: float | None = None,
                      score_start: float | None = None,
                      d_settle_time: float | None = None,
                      d_plateau_center: float | None = None) -> str:
    width, height = 620, 300
    left, right, top, bottom = 55, 20, 48, 40
    aligned = align_repeats(traces)
    average = smoothed_mean(aligned)
    duration = max((t for series in traces.values() for t, _ in series), default=1.0)
    all_values = [v for series in traces.values() for _, v in series]
    scale = max(abs(target) * 1.6, 0.35,
                max((abs(value) * 1.05 for value in all_values), default=0.0))

    def x(value: float) -> float:
        return left + value / duration * (width - left - right)

    def y(value: float) -> float:
        return top + (scale - value) / (2 * scale) * (height - top - bottom)

    def line(series: list[tuple[float, float]], css: str) -> str:
        if not series:
            return ""
        stride = max(1, math.ceil(len(series) / 600))
        points = " ".join(f"{x(t):.1f},{y(v):.1f}" for t, v in series[::stride])
        return f'<polyline class="{css}" points="{points}"/>'

    result = [f'<svg viewBox="0 0 {width} {height}" role="img" '
              f'aria-label="{html.escape(title, quote=True)}">',
              f'<text class="plot-title" x="{left}" y="22">{html.escape(title)}</text>']
    if phase == "d":
        band = abs(target) * settings["velocity_d_plateau_band_fraction"]
        result.append(f'<rect class="band" x="{left}" '
                      f'y="{y(target+band):.1f}" '
                      f'width="{width-left-right}" '
                      f'height="{y(target-band)-y(target+band):.1f}"/>')
        if d_plateau_center is not None:
            result.append(f'<line class="plateau" x1="{left}" '
                          f'y1="{y(d_plateau_center):.1f}" '
                          f'x2="{width-right}" y2="{y(d_plateau_center):.1f}"/>')
        if d_settle_time is not None and math.isfinite(d_settle_time):
            marker = x(min(max(d_settle_time, 0.0), duration))
            result.append(f'<rect class="transient" x="{left}" y="{top}" '
                          f'width="{marker-left:.1f}" height="{height-top-bottom}"/>')
            result.append(f'<line class="score-marker" x1="{marker:.1f}" '
                          f'y1="{top}" x2="{marker:.1f}" y2="{height-bottom}"/>')
    if phase in ("i", "validation"):
        band = abs(target) * settings["velocity_i_mean_band_fraction"]
        band_left = x(min(score_start, duration)) if score_start is not None else left
        result.append(f'<rect class="band" x="{band_left:.1f}" y="{y(target+band):.1f}" '
                      f'width="{width-right-band_left:.1f}" '
                      f'height="{y(target-band)-y(target+band):.1f}"/>')
        if score_start is not None and score_start <= duration:
            result.append(f'<line class="score-marker" x1="{x(score_start):.1f}" '
                          f'y1="{top}" x2="{x(score_start):.1f}" y2="{height-bottom}"/>')
    if phase == "p" and arrival_target is not None:
        result.append(f'<line class="arrival" x1="{x(arrival_target):.1f}" '
                      f'y1="{top}" x2="{x(arrival_target):.1f}" y2="{height-bottom}"/>')
    for tick in range(5):
        value = -scale + tick * scale / 2
        elapsed = tick * duration / 4
        result.append(f'<line class="grid" x1="{left}" y1="{y(value):.1f}" '
                      f'x2="{width-right}" y2="{y(value):.1f}"/>')
        result.append(f'<text class="tick" x="{left-5}" y="{y(value)+4:.1f}" '
                      f'text-anchor="end">{value:+.2f}</text>')
        result.append(f'<text class="tick" x="{x(elapsed):.1f}" '
                      f'y="{height-bottom+18}" text-anchor="middle">{elapsed:.1f}</text>')
    result.append(f'<line class="target" x1="{left}" y1="{y(target):.1f}" '
                  f'x2="{width-right}" y2="{y(target):.1f}"/>')
    for cycle, series in sorted(traces.items()):
        result.append(line(series, f"repeat repeat-{cycle}" +
                           (" repeat-muted" if phase in ("i", "validation") else "")))
    if phase in ("i", "validation"):
        result.append(line(average, "mean-outside"))
        in_band: list[tuple[float, float]] = []
        for elapsed, value in average:
            if (score_start is not None and elapsed >= score_start and
                    abs(value - target) <= abs(target) *
                    settings["velocity_i_mean_band_fraction"]):
                in_band.append((elapsed, value))
            else:
                if len(in_band) >= 2:
                    result.append(line(in_band, "mean-inside"))
                in_band = []
        if len(in_band) >= 2:
            result.append(line(in_band, "mean-inside"))
    elif phase == "d":
        result.append(line(average, "filtered"))
    result.append('</svg>')
    return "".join(result)


def plot_svg(flight: list[tuple[float, float]], brake: list[tuple[float, float]],
             target: float, phase: str, settings: dict, title: str) -> str:
    width, height = 620, 300
    left, right, top, bottom = 55, 20, 48, 40
    flight_end = max((elapsed for elapsed, _ in flight), default=0.0)
    brake_end = max((elapsed for elapsed, _ in brake), default=0.0)
    duration = max(1.0, flight_end + brake_end)
    scale = max(0.35, abs(target) * 1.65,
                *(abs(value) * 1.12 for _, value in flight + brake))
    low, high = -scale, scale

    def x(value: float) -> float:
        return left + value / duration * (width - left - right)

    def y(value: float) -> float:
        return top + (high - value) / (high - low) * (height - top - bottom)

    def poly(points: list[tuple[float, float]], css: str) -> str:
        if not points:
            return ""
        stride = max(1, math.ceil(len(points) / 650))
        shown = points[::stride]
        if shown[-1] != points[-1]:
            shown.append(points[-1])
        coords = " ".join(f"{x(t):.1f},{y(v):.1f}" for t, v in shown)
        return f'<polyline class="{css}" points="{coords}"/>'

    pieces = [f'<svg viewBox="0 0 {width} {height}" role="img" '
              f'aria-label="{html.escape(title, quote=True)}">',
              f'<text class="plot-title" x="{left}" y="22">{html.escape(title)}</text>']
    for tick in range(5):
        t = duration * tick / 4
        pieces.append(f'<line class="grid" x1="{x(t):.1f}" y1="{top}" '
                      f'x2="{x(t):.1f}" y2="{height-bottom}"/>')
        pieces.append(f'<text class="tick" x="{x(t):.1f}" '
                      f'y="{height-bottom+18}" text-anchor="middle">{t:.1f}</text>')
    for tick in range(5):
        value = low + (high - low) * tick / 4
        pieces.append(f'<line class="grid" x1="{left}" y1="{y(value):.1f}" '
                      f'x2="{width-right}" y2="{y(value):.1f}"/>')
        pieces.append(f'<text class="tick" x="{left-6}" y="{y(value)+4:.1f}" '
                      f'text-anchor="end">{value:+.2f}</text>')
    band_fraction = (settings["velocity_d_hold_band_fraction"] if phase in
                     ("d", "d_base", "repeat", "fine", "confirmation") else
                     settings["velocity_i_hold_band_fraction"] if phase == "i" else None)
    if band_fraction is not None and flight:
        band = abs(target) * band_fraction
        pieces.append(f'<rect class="band" x="{left}" y="{y(target+band):.1f}" '
                      f'width="{x(flight_end)-left:.1f}" '
                      f'height="{y(target-band)-y(target+band):.1f}"/>')
    if phase == "p":
        marker = settings["velocity_p_target_arrival_seconds"]
        pieces.append(f'<line class="arrival" x1="{x(marker):.1f}" y1="{top}" '
                      f'x2="{x(marker):.1f}" y2="{height-bottom}"/>')
    if flight:
        pieces.append(f'<line class="target" x1="{left}" y1="{y(target):.1f}" '
                      f'x2="{x(flight_end):.1f}" y2="{y(target):.1f}"/>')
    if brake:
        pieces.append(f'<line class="brake-boundary" x1="{x(flight_end):.1f}" '
                      f'y1="{top}" x2="{x(flight_end):.1f}" y2="{height-bottom}"/>')
        pieces.append(f'<line class="target" x1="{x(flight_end):.1f}" '
                      f'y1="{y(0):.1f}" x2="{x(duration):.1f}" y2="{y(0):.1f}"/>')
    shifted_brake = [(flight_end + t, v) for t, v in brake]
    pieces.extend((poly(flight, "raw"), poly(shifted_brake, "raw"),
                   poly(smooth(flight), "filtered"),
                   poly(smooth(shifted_brake), "filtered")))
    pieces.append(f'<text class="axis-label" x="{width-right}" '
                  f'y="{height-5}" text-anchor="end">время, с</text></svg>')
    return "".join(pieces)


def build_report(run: Path, pid: str = "pid_vel_pitch", last: int = 8) -> str:
    summary, records = completed_trials(run, pid, last)
    settings = summary.get("config", {})
    cards = []
    for record in records:
        name = record["pid"]
        axis = "x" if name == "pid_vel_pitch" else "y"
        label = record["label"]
        phase = ("p" if "_p_" in label else "i" if "_i_" in label else
                 "d_base" if "_d_base" in label else "d" if "_d_" in label else
                 "repeat" if "_repeat" in label else "confirmation")
        metrics = record.get("metrics") or {}
        gains = record.get("gains") or {}
        filename = record["telemetry_csv"]
        if metrics.get("repeat_count", 0) >= 3 and metrics.get("phase") in (
                "p", "d", "i", "validation"):
            phase = metrics["phase"]
            speeds = sorted({float(part["speed"]) for part in metrics["repeat_steps"]})
            plots = []
            for speed in speeds:
                plan = next((item for item in settings.get("velocity_p_targets", [])
                             if math.isclose(item["speed"], speed)), None)
                for direction, sign in (("positive", 1), ("negative", -1)):
                    traces = read_repeated_traces(run / filename, axis, direction, speed)
                    d_steps = [part for part in metrics.get("mean_steps", [])
                               if part.get("direction") == direction and
                               math.isclose(float(part["speed"]), speed)]
                    plots.append(repeated_plot_svg(
                        traces, sign * speed, phase, settings,
                        f"{'Вперёд' if sign > 0 else 'Назад'} · цель {sign*speed:+.2f} м/с",
                        plan["arrival_seconds"] if phase == "p" and plan else None,
                        (metrics.get("i_score_start_seconds") if phase == "i" else
                         (metrics.get("i_mean_response_by_direction", {})
                          .get(direction, {}).get("score_start"))
                         if phase == "validation" else None),
                        (sum(part["settling_time"] for part in d_steps) / len(d_steps)
                         if phase == "d" and d_steps else None),
                        (sum(part["plateau_center"] for part in d_steps) / len(d_steps)
                         if phase == "d" and d_steps else None)))
            gains_text = " · ".join(f"{key.upper()}={gains[key]:.4g}"
                                    for key in ("kp", "kd", "ki") if key in gains)
            details = (f"средняя достигла {metrics['reached_steps']}/{len(metrics.get('mean_steps', []))} целей"
                       if phase == "p" else
                       f"успокоение {metrics['plateau_settling_time']:.2f} с"
                       if phase == "d" else
                       f"средняя в коридоре {metrics['hold_fraction']:.0%}")
            cards.append('<section class="trial">'
                         f'<h2>{html.escape(label)}</h2>'
                         f'<p>{html.escape(gains_text)} · score={metrics["score"]:.3f} · '
                         f'{details} · 3 пролёта в каждую сторону</p>'
                         f'<p class="file">{html.escape(filename)}</p>'
                         f'<div class="plots">{"".join(plots)}</div></section>')
            continue
        plots = []
        details = []
        for direction, caption in (("positive", "Вперёд"),
                                   ("negative", "Назад")):
            flight, brake, target = read_trace(run / filename, axis, direction)
            plots.append(plot_svg(flight, brake, target, phase, settings,
                                  f"{caption} · цель {target:+.3f} м/с"))
            part = metrics.get(direction) or {}
            arrival = part.get("arrival_time")
            hold_key = ("i_hold_fraction_after_arrival" if phase == "i" else
                        "d_hold_fraction_after_arrival")
            details.append(f"{caption.lower()}: приход "
                           f"{arrival:.2f} с" if isinstance(arrival, (int, float))
                           else f"{caption.lower()}: приход не определён")
            if phase != "p" and isinstance(part.get(hold_key), (int, float)):
                details[-1] += f", удержание {part[hold_key]:.0%}"
        gains_text = " · ".join(f"{key.upper()}={gains[key]:.4g}"
                                for key in ("kp", "kd", "ki") if key in gains)
        score = metrics.get("score")
        score_text = f"{score:.3f}" if isinstance(score, (int, float)) else "—"
        cards.append('<section class="trial">'
                     f'<h2>{html.escape(label)}</h2>'
                     f'<p>{html.escape(gains_text)} · score={score_text} · '
                     f'{html.escape("; ".join(details))}</p>'
                     f'<p class="file">{html.escape(filename)}</p>'
                     f'<div class="plots">{"".join(plots)}</div></section>')
    content = "".join(cards) if cards else "<p>Завершённых проб скорости пока нет.</p>"
    if settings.get("velocity_repeated_mode"):
        legend = ("Цветные линии — три пролёта, зелёный пунктир — цель. "
                  "На P красная черта показывает заданное время достижения. "
                  "На D розовая область показывает колебания до успокоения, "
                  "зелёная — коридор ±15% вокруг цели, серый пунктир — "
                  "оценённый конечный уровень. D оценивает затухание относительно "
                  "этого уровня. На I цветные "
                  "линии полупрозрачны, чёрная — сглаженное среднее трёх пролётов; "
                  "оно пунктирное вне коридора ±20%. Красная черта показывает "
                  "начало оценки по лучшей пробе D; на проверочных скоростях "
                  "оно определяется заново по средней кривой. После каждого пролёта "
                  "применяется служебное торможение; оно не входит в оценку.")
    else:
        legend = ("Синяя линия — фактическая скорость, чёрная — сглаживание за "
                  "0,25 с, зелёный пунктир — команда. Красная черта на этапе P — "
                  "заданное время прихода. Зелёная область — коридор удержания. "
                  "Фиолетовая черта — начало торможения.")
    return f'''<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta http-equiv="refresh" content="5"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Калибровка скорости · {html.escape(run.name)}</title><style>
body {{ font:15px system-ui,sans-serif; background:#f4f6fa; color:#172033; margin:0 auto;
       max-width:1300px; padding:20px }}
.trial {{ background:white; border:1px solid #dbe1e9; border-radius:10px;
          padding:15px; margin:0 0 18px }}
.trial h2 {{ font-size:17px; margin:0 0 5px; overflow-wrap:anywhere }}
.trial p {{ margin:4px 0 }} .file {{ color:#69758b; font-size:12px }}
.plots {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:12px }}
svg {{ width:100%; background:white; border:1px solid #e4e8ef; border-radius:6px }}
.grid {{ stroke:#e5eaf1 }} .tick,.axis-label {{ fill:#64748b; font-size:11px }}
.plot-title {{ fill:#172033; font-size:14px; font-weight:600 }}
.band {{ fill:#bbf7d0; opacity:.65 }} .target {{ stroke:#15803d; stroke-width:2;
       stroke-dasharray:6 4 }} .arrival {{ stroke:#dc2626; stroke-width:1.6;
       stroke-dasharray:5 4 }} .brake-boundary {{ stroke:#7c3aed;
       stroke-width:1.5; stroke-dasharray:4 4 }}
.plateau {{ stroke:#64748b; stroke-width:1.2; stroke-dasharray:2 4 }}
.raw {{ fill:none; stroke:#2563eb; stroke-width:1.2; opacity:.55 }}
.filtered {{ fill:none; stroke:#111827; stroke-width:2.5 }}
.repeat {{ fill:none; stroke-width:1.3; opacity:.5 }}
.repeat-1 {{ stroke:#2563eb }} .repeat-2 {{ stroke:#d97706 }}
.repeat-3 {{ stroke:#7c3aed }}
.repeat-muted {{ opacity:.23 }}
.transient {{ fill:#f9a8d4; opacity:.35 }}
.score-marker {{ stroke:#e11d48; stroke-width:1.5; stroke-dasharray:5 4 }}
.mean-outside {{ fill:none; stroke:#111827; stroke-width:2.5;
                 stroke-dasharray:6 4 }}
.mean-inside {{ fill:none; stroke:#111827; stroke-width:3 }}
@media(max-width:850px) {{ .plots {{ grid-template-columns:1fr }} }}
</style></head><body><h1>Телеметрия контура скорости</h1>
<p>Запуск {html.escape(run.name)} · {html.escape(pid)} · последние {last} проб.
{legend}
Страница обновляется каждые 5 секунд.</p>{content}</body></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", default="latest", help="Каталог запуска или latest")
    parser.add_argument("--pid", choices=("pid_vel_pitch", "pid_vel_roll", "both"),
                        default="pid_vel_pitch")
    parser.add_argument("--last", type=int, default=8)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--watch", type=float, default=0,
                        help="Обновлять HTML каждые N секунд до Ctrl+C")
    args = parser.parse_args()
    if args.last < 1 or args.watch < 0:
        parser.error("--last must be positive and --watch cannot be negative")
    run = resolve_run(args.run)
    output = args.output.resolve() if args.output else run / "velocity_telemetry.html"
    try:
        while True:
            try:
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(build_report(run, args.pid, args.last), encoding="utf-8")
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
