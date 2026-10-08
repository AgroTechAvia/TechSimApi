"""Shared time alignment and settled-band scoring for acceleration trials."""

from __future__ import annotations

import bisect
from statistics import mean, median


TRACE_STEP_SECONDS = 0.05
SMOOTH_RADIUS_SAMPLES = 2  # Five 50 ms samples: a 250 ms display filter.


def align_repeats(series: dict[int, list[tuple[float, float]]]
                  ) -> list[tuple[float, list[float]]]:
    """Interpolate every repeat onto their common 50 ms time grid."""
    repeats = [sorted(points) for _, points in sorted(series.items()) if len(points) >= 2]
    if not repeats:
        return []
    start = max(points[0][0] for points in repeats)
    end = min(points[-1][0] for points in repeats)
    if end <= start:
        return []
    timestamps = [[point[0] for point in points] for points in repeats]
    aligned = []
    for tick in range(int((end - start) / TRACE_STEP_SECONDS) + 1):
        elapsed = start + tick * TRACE_STEP_SECONDS
        values = []
        for points, times in zip(repeats, timestamps):
            right = bisect.bisect_right(times, elapsed)
            if right == 0:
                value = points[0][1]
            elif right == len(points):
                value = points[-1][1]
            else:
                before, after = points[right - 1], points[right]
                span = after[0] - before[0]
                value = (after[1] if span == 0 else
                         before[1] + (after[1] - before[1]) *
                         (elapsed - before[0]) / span)
            values.append(value)
        aligned.append((elapsed, values))
    return aligned


def smoothed_mean(aligned: list[tuple[float, list[float]]]
                  ) -> list[tuple[float, float]]:
    means = [(elapsed, mean(values)) for elapsed, values in aligned]
    trace = []
    for index, (elapsed, _) in enumerate(means):
        window = means[max(0, index - SMOOTH_RADIUS_SAMPLES):
                       index + SMOOTH_RADIUS_SAMPLES + 1]
        trace.append((elapsed, mean(value for _, value in window)))
    return trace


def settled_band_response(trace: list[tuple[float, float]], target: float,
                          tolerance: float, plateau_band: float,
                          earliest_score_time: float = 2.0) -> dict[str, float | bool]:
    """Score the mean curve after it calms near its own final level.

    Settling is independent of target accuracy: a quiet but biased response
    can settle, then receives a low in-band fraction and a high MAE.
    """
    if not trace:
        return {"settled": False, "settling_time": 0.0,
                "score_start": 0.0, "hold_fraction": 0.0,
                "mae": abs(target), "bias": -target}
    duration = trace[-1][0]
    final = [value for elapsed, value in trace if elapsed >= duration - 1.0]
    center = median(final)
    within = [abs(value - center) <= plateau_band for _, value in trace]
    settled_at = None
    for index, (elapsed, _) in enumerate(trace):
        if duration - elapsed < 1.0:
            break
        remaining = within[index:]
        if mean(remaining) < 0.85:
            continue
        stable_windows = True
        window_start = elapsed
        while window_start < duration:
            window = [inside for (time, _), inside in zip(trace[index:], remaining)
                      if window_start <= time < window_start + 0.5]
            if len(window) >= 5 and mean(window) < 0.70:
                stable_windows = False
                break
            window_start += 0.5
        if stable_windows:
            settled_at = elapsed
            break
    score_start = max(earliest_score_time, settled_at or 0.0)
    scored = ([value for elapsed, value in trace if elapsed >= score_start]
              if settled_at is not None and duration - score_start >= 0.5 else [])
    tail = [value for elapsed, value in trace if elapsed >= duration - 1.0]
    basis = scored or tail
    return {
        "settled": settled_at is not None and bool(scored),
        "settling_time": duration if settled_at is None else settled_at,
        "score_start": score_start,
        "hold_fraction": mean(abs(value - target) <= tolerance for value in scored)
                         if scored else 0.0,
        "mae": mean(abs(value - target) for value in basis),
        "bias": mean(value - target for value in basis),
    }
