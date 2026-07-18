"""Stateless geodesy — the math OpenCDaRR owns (ADR 0003).

Replaces the former ``bluesky.tools.geo`` call in ``dynamics.py`` so shipping code depends
only on numpy. :func:`forward` mirrors BlueSky's ``qdrpos`` (same WGS84 latitude-dependent
radius and great-circle formula), so the two agree to floating-point precision — the
BlueSky anchor test checks exactly that.
"""

from __future__ import annotations

import numpy as np

_WGS84_A = 6378137.0  # semi-major axis [m]
_WGS84_B = 6356752.314245  # semi-minor axis [m]


def earth_radius(lat_deg: float) -> float:
    """WGS84 earth radius at a given latitude, in metres (matches BlueSky ``rwgs84``)."""
    lat = np.radians(lat_deg)
    an = _WGS84_A * _WGS84_A * np.cos(lat)
    bn = _WGS84_B * _WGS84_B * np.sin(lat)
    ad = _WGS84_A * np.cos(lat)
    bd = _WGS84_B * np.sin(lat)
    return float(np.sqrt((an * an + bn * bn) / (ad * ad + bd * bd)))


def forward(
    lat_deg: float, lon_deg: float, bearing_deg: float, dist_m: float
) -> tuple[float, float]:
    """Great-circle destination from a point, given a bearing and distance.

    Parameters are in degrees and metres; returns ``(lat, lon)`` in degrees. Standard
    destination-point formula on a sphere of the local WGS84 radius (movable-type.co.uk).
    """
    radius = earth_radius(lat_deg)
    lat1 = np.radians(lat_deg)
    lon1 = np.radians(lon_deg)
    bearing = np.radians(bearing_deg)
    ang = dist_m / radius  # angular distance [rad]

    lat2 = np.arcsin(
        np.sin(lat1) * np.cos(ang) + np.cos(lat1) * np.sin(ang) * np.cos(bearing)
    )
    lon2 = lon1 + np.arctan2(
        np.sin(bearing) * np.sin(ang) * np.cos(lat1),
        np.cos(ang) - np.sin(lat1) * np.sin(lat2),
    )
    return float(np.degrees(lat2)), float(np.degrees(lon2))


def qdrdist(
    lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float
) -> tuple[float, float]:
    """Initial bearing [deg] and great-circle distance [m] from point 1 to point 2.

    The inverse of :func:`forward` on a sphere of the local WGS84 radius at point 1, so
    ``qdrdist(p1, *forward(p1, brg, d))`` returns ``(brg, d)`` to floating-point precision.
    Bearing follows the aviation convention (0 = North, clockwise). Mirrors BlueSky's
    ``qdrdist`` (own geodesy, ADR 0003).
    """
    radius = earth_radius(lat1_deg)
    lat1 = np.radians(lat1_deg)
    lat2 = np.radians(lat2_deg)
    dlat = lat2 - lat1
    dlon = np.radians(lon2_deg - lon1_deg)

    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    dist = 2.0 * radius * np.arcsin(np.sqrt(a))

    y = np.sin(dlon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    qdr = np.degrees(np.arctan2(y, x)) % 360.0
    return float(qdr), float(dist)
