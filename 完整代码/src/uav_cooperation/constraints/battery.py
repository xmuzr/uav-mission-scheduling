"""Flight and dwell consume budget; preflight ground waiting does not."""


def battery_feasible(drone,airborne_seconds):
    return airborne_seconds <= drone.usable_seconds+1e-8
