"""Pure mission mathematics; no ROS dependency.

PX4 local coordinates use NED: positive Z points down.
"""
from math import hypot, pi, sin


def figure_eight(elapsed_s: float, home: tuple[float, float, float],
                 amplitude_x: float = 6.0, amplitude_y: float = 3.0,
                 period_s: float = 20.0, altitude_m: float = 5.0) -> tuple[float, float, float]:
    """Return NED position setpoint for a horizontal figure-eight."""
    if period_s <= 0 or altitude_m < 0 or amplitude_x < 0 or amplitude_y < 0:
        raise ValueError('period must be positive; amplitudes and altitude nonnegative')
    omega = 2 * pi / period_s
    return (home[0] + amplitude_x * sin(omega * elapsed_s),
            home[1] + amplitude_y * sin(2 * omega * elapsed_s),
            home[2] - altitude_m)


def outside_geofence(position: tuple[float, float, float],
                      home: tuple[float, float, float],
                      radius_m: float = 12.0, max_altitude_m: float = 8.0) -> bool:
    """True when horizontal distance or height above home exceeds limits."""
    if radius_m <= 0 or max_altitude_m <= 0:
        raise ValueError('geofence limits must be positive')
    horizontal = hypot(position[0] - home[0], position[1] - home[1])
    height_above_home = home[2] - position[2]
    return horizontal > radius_m or height_above_home > max_altitude_m
