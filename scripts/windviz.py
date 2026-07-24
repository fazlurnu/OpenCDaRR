"""Shared wind visualisation helpers for the Phase-5 demos (5b multirotor, 5c fixed-wing).

One small helper both demos reuse so the wind is drawn the same way everywhere: an arrow pointing
where the air *moves* (the velocity vector), labelled with the speed and the meteorological
"coming-from" bearing.
"""

from __future__ import annotations

from typing import Any

from opencdarr.wind import WindField


def draw_wind_arrow(
    ax: Any, wind: WindField, at_xy: tuple[float, float], length: float,
    color: str = "0.35", label: bool = True,
) -> None:
    """Draw a wind arrow at data point ``at_xy``, ``length`` units long, pointing downwind.

    The arrow shows where the air moves (the wind velocity vector); ``length`` is in the axes' data
    units so it scales with the plot. A north wind (from 0°) therefore points *down* (south).
    """
    speed = wind.speed
    if speed < 1e-9:
        return
    we, wn = wind.components()
    ux, uy = we / speed, wn / speed
    ax.annotate(
        "", xy=(at_xy[0] + ux * length, at_xy[1] + uy * length), xytext=at_xy,
        arrowprops={"arrowstyle": "-|>", "color": color, "lw": 2.4},
    )
    if label:
        ax.text(at_xy[0], at_xy[1], f"  wind {speed:.0f} m/s\n  from {wind.coming_from:.0f}°",
                fontsize=8, color=color, va="center", ha="left")
