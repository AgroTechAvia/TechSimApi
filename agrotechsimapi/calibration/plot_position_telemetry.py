"""Render live HTML charts for the repeated XY position calibration."""

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
    if value == "latest":
        if not RUNS.exists():
            raise FileNotFoundError("No position calibration runs found")
        for path in sorted(RUNS.iterdir(), reverse=True):
            summary_path = path / "summary.json"
            if not path.is_dir() or not summary_path.is_file():
                continue
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if (str(summary.get("status", "")).startswith("position") or
                    any(item.get("stage") == "position" for item in
                        summary.get("evaluations", []))):
                return path
        raise FileNotFoundError("No position calibration run found")
    path = Path(value).resolve()
    if not (path / "summary.json").is_file():
        raise FileNotFoundError(f"No summary.json in {path}")
    return path


def completed_trials(run: Path, pid: str, last: int) -> tuple[dict, list[dict]]:
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    records = [item for item in summary.get("evaluations", [])
               if item.get("stage") == "position" and item.get("valid")
               and (pid == "both" or item.get("pid") == pid)
               and item.get("metrics", {}).get("method") ==
               "position_repeated_response" and item.get("telemetry_csv")
               and (run / item["telemetry_csv"]).is_file()]
    return summary, records[-last:]


def read_traces(path: Path, axis: str, direction: str
                ) -> dict[int, list[tuple[float, float]]]:
    """Subtract each step's actual start; preserve the sign of its command."""
    traces: dict[int, list[tuple[float, float]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("segment") != direction:
                continue
            try:
                cycle = int(row["position_cycle"])
                elapsed = float(row["segment_elapsed"])
                offset = float(row[axis]) - float(row["position_origin_axis"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(elapsed) and math.isfinite(offset):
                traces.setdefault(cycle, []).append((elapsed, offset))
    return traces


def plot_svg(traces: dict[int, list[tuple[float, float]]], target: float,
             phase: str, band: float, title: str,
             *, arrival_target: float | None = None,
             settle_time: float | None = None,
             plateau_center: float | None = None,
             score_start: float | None = None) -> str:
    width, height = 620, 300
    left, right, top, bottom = 55, 20, 44, 38
    aligned = align_repeats(traces)
    average = smoothed_mean(aligned)
    duration = max((t for series in traces.values() for t, _ in series), default=1.0)
    values = [value for series in traces.values() for _, value in series]
    scale = max(abs(target) * 1.6, band * 3, 0.35,
                max((abs(value) * 1.08 for value in values), default=0.0))

    def x(value: float) -> float:
        return left + value / max(duration, 1e-6) * (width - left - right)

    def y(value: float) -> float:
        return top + (scale - value) / (2 * scale) * (height - top - bottom)

    def line(series: list[tuple[float, float]], css: str) -> str:
        if len(series) < 2:
            return ""
        stride = max(1, math.ceil(len(series) / 650))
        shown = series[::stride]
        if shown[-1] != series[-1]:
            shown.append(series[-1])
        points = " ".join(f"{x(t):.1f},{y(v):.1f}" for t, v in shown)
        return f'<polyline class="{css}" points="{points}"/>'

    parts = [f'<svg viewBox="0 0 {width} {height}" role="img" '
             f'aria-label="{html.escape(title, quote=True)}">',
             f'<text class="plot-title" x="{left}" y="22">{html.escape(title)}</text>']
    if phase == "d" and plateau_center is not None:
        upper, lower = plateau_center + band, plateau_center - band
        parts.append(f'<rect class="band" x="{left}" y="{y(upper):.1f}" '
                     f'width="{width-left-right}" height="{y(lower)-y(upper):.1f}"/>')
        parts.append(f'<line class="plateau" x1="{left}" y1="{y(plateau_center):.1f}" '
                     f'x2="{width-right}" y2="{y(plateau_center):.1f}"/>')
        if settle_time is not None:
            marker = x(min(max(settle_time, 0.0), duration))
            parts.append(f'<rect class="transient" x="{left}" y="{top}" '
                         f'width="{marker-left:.1f}" height="{height-top-bottom}"/>')
            parts.append(f'<line class="marker" x1="{marker:.1f}" y1="{top}" '
                         f'x2="{marker:.1f}" y2="{height-bottom}"/>')
    if phase in ("i", "validation"):
        upper, lower = target + band, target - band
        begin = x(min(max(score_start or 0.0, 0.0), duration))
        parts.append(f'<rect class="band" x="{begin:.1f}" y="{y(upper):.1f}" '
                     f'width="{width-right-begin:.1f}" '
                     f'height="{y(lower)-y(upper):.1f}"/>')
        if score_start is not None:
            parts.append(f'<line class="marker" x1="{begin:.1f}" y1="{top}" '
                         f'x2="{begin:.1f}" y2="{height-bottom}"/>')
    if phase == "p" and arrival_target is not None:
        marker = x(arrival_target)
        parts.append(f'<line class="marker" x1="{marker:.1f}" y1="{top}" '
                     f'x2="{marker:.1f}" y2="{height-bottom}"/>')
    for tick in range(5):
        t = duration * tick / 4
        value = -scale + tick * scale / 2
        parts.append(f'<line class="grid" x1="{left}" y1="{y(value):.1f}" '
                     f'x2="{width-right}" y2="{y(value):.1f}"/>')
        parts.append(f'<text class="tick" x="{left-5}" y="{y(value)+4:.1f}" '
                     f'text-anchor="end">{value:+.2f}</text>')
        parts.append(f'<text class="tick" x="{x(t):.1f}" '
                     f'y="{height-bottom+17}" text-anchor="middle">{t:.1f}</text>')
    parts.append(f'<line class="target" x1="{left}" y1="{y(target):.1f}" '
                 f'x2="{width-right}" y2="{y(target):.1f}"/>')
    for cycle, series in sorted(traces.items()):
        muted = " muted" if phase in ("i", "validation") else ""
        parts.append(line(series, f"repeat repeat-{cycle}{muted}"))
    if phase == "d":
        parts.append(line(average, "mean"))
    elif phase in ("i", "validation"):
        parts.append(line(average, "mean-outside"))
        inside = []
        for elapsed, value in average:
            if elapsed >= (score_start or 0.0) and abs(value - target) <= band:
                inside.append((elapsed, value))
            else:
                parts.append(line(inside, "mean-inside"))
                inside = []
        parts.append(line(inside, "mean-inside"))
    parts.append('</svg>')
    return "".join(parts)


def build_report(run: Path, pid: str = "both", last: int = 8) -> str:
    summary, records = completed_trials(run, pid, last)
    settings = summary.get("config", {})
    band = settings.get("position_d_band_m", 0.07)
    cards = []
    for record in records:
        metrics = record["metrics"]
        phase = metrics["phase"]
        axis = "x" if record["pid"] == "pid_pos_x" else "y"
        distance = metrics["distance"]
        plots = []
        for direction, sign in (("positive", 1), ("negative", -1)):
            step = next(item for item in metrics["mean_steps"]
                        if item["direction"] == direction)
            response = metrics.get("mean_response_by_direction", {}).get(direction, {})
            plots.append(plot_svg(
                read_traces(run / record["telemetry_csv"], axis, direction),
                sign * distance, phase, band,
                f"{'Вперёд' if sign > 0 else 'Назад'} · {axis.upper()} · цель {sign*distance:+.3f} м",
                arrival_target=(settings.get("position_p_arrival_seconds", 2.5)
                                if phase == "p" else None),
                settle_time=(step["settling_time"] if phase == "d" else None),
                plateau_center=(step["plateau_center"] if phase == "d" else None),
                score_start=(metrics.get("i_score_start_seconds") if phase == "i"
                             else response.get("score_start")
                             if phase == "validation" else None)))
        gains = record.get("gains", {})
        gain_text = " · ".join(f"{key.upper()}={gains[key]:.4g}"
                               for key in ("kp", "kd", "ki") if key in gains)
        detail = (f"приход {metrics['mean_arrival_seconds']:.2f} с"
                  if phase == "p" and metrics["mean_arrival_seconds"] is not None else
                  f"успокоение {metrics['plateau_settling_time']:.2f} с"
                  if phase == "d" else
                  f"в целевом коридоре {metrics['hold_fraction']:.0%}"
                  if phase in ("i", "validation") else "цель не достигнута")
        cards.append('<section class="trial">'
                     f'<h2>{html.escape(record["label"])}</h2>'
                     f'<p>{html.escape(gain_text)} · score={metrics["score"]:.3f} · '
                     f'{html.escape(detail)} · 3 пролёта в каждую сторону</p>'
                     f'<p class="file">{html.escape(record["telemetry_csv"])}</p>'
                     f'<div class="plots">{"".join(plots)}</div></section>')
    content = "".join(cards) if cards else "<p>Завершённых проб положения пока нет.</p>"
    return f'''<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta http-equiv="refresh" content="5"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Калибровка положения · {html.escape(run.name)}</title><style>
body {{font:15px system-ui,sans-serif;background:#f4f6fa;color:#172033;margin:0 auto;max-width:1300px;padding:20px}}
.trial {{background:#fff;border:1px solid #dbe1e9;border-radius:10px;padding:15px;margin:0 0 18px}}
.trial h2 {{font-size:17px;margin:0 0 5px;overflow-wrap:anywhere}} .trial p {{margin:4px 0}}
.file {{color:#69758b;font-size:12px}} .plots {{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}}
svg {{width:100%;background:#fff;border:1px solid #e4e8ef;border-radius:6px}}
.grid {{stroke:#e5eaf1}} .tick {{fill:#64748b;font-size:11px}} .plot-title {{fill:#172033;font-size:14px;font-weight:600}}
.band {{fill:#bbf7d0;opacity:.65}} .target {{stroke:#15803d;stroke-width:2;stroke-dasharray:6 4}}
.plateau {{stroke:#64748b;stroke-width:1.2;stroke-dasharray:2 4}}
.marker {{stroke:#e11d48;stroke-width:1.5;stroke-dasharray:5 4}}
.transient {{fill:#f9a8d4;opacity:.35}} .repeat {{fill:none;stroke-width:1.3;opacity:.6}}
.repeat-1 {{stroke:#2563eb}} .repeat-2 {{stroke:#d97706}} .repeat-3 {{stroke:#7c3aed}}
.muted {{opacity:.23}} .mean {{fill:none;stroke:#111827;stroke-width:2.5}}
.mean-outside {{fill:none;stroke:#111827;stroke-width:2.5;stroke-dasharray:6 4}}
.mean-inside {{fill:none;stroke:#111827;stroke-width:3}}
@media(max-width:850px) {{.plots {{grid-template-columns:1fr}}}}
</style></head><body><h1>Телеметрия контура положения</h1>
<p>Запуск {html.escape(run.name)} · последние {last} проб. По вертикали — смещение от начала каждой ступени.
Цветные линии — три пролёта, чёрная — их сглаженное среднее. P: целевая точка и время прихода.
D: розовый переходный участок и зелёный коридор ±{band:.2f} м вокруг конечного уровня.
I: целевой коридор ±{band:.2f} м после времени D; средняя вне коридора пунктирная.
Служебное торможение не входит в оценку. Страница обновляется каждые 5 секунд.</p>
{content}</body></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", default="latest", help="Каталог запуска или latest")
    parser.add_argument("--pid", choices=("pid_pos_x", "pid_pos_y", "both"),
                        default="both")
    parser.add_argument("--last", type=int, default=8)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--watch", type=float, default=0,
                        help="Обновлять HTML каждые N секунд до Ctrl+C")
    args = parser.parse_args()
    if args.last < 1 or args.watch < 0:
        parser.error("--last must be positive and --watch cannot be negative")
    output = (args.output.resolve() if args.output else
              Path(__file__).resolve().parent / "position_telemetry_latest.html"
              if args.run == "latest" else
              Path(args.run).resolve() / "position_telemetry.html")
    try:
        while True:
            try:
                run = resolve_run(args.run)
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(build_report(run, args.pid, args.last),
                                  encoding="utf-8")
                print(f"Plots updated: {output} (run {run.name})")
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
