"""Offline-only rejected candidate. Not qualified for deployment."""
import math

def fill_single_bin_gaps(ranges, angle_increment):
    """Experimental same-scan line interpolation; never change measured bins.

    Fill only one missing bin bracketed by original finite positive returns,
    with <=0.3 m range difference and <=2 degrees total angular separation.
    Intersect the middle ray with the chord between the two measured points.
    No temporal smoothing, multi-bin filling, wraparound or extrapolation.
    The 0.3 m surface bound matches RF2O's existing pyramid filter bound.
    Returned count identifies inferred values, not new sensor measurements.
    """
    if not math.isfinite(angle_increment) or not 0 < angle_increment <= math.radians(1.00001):
        raise ValueError('Single-bin interpolation requires positive spacing <=1 degree')
    result = list(ranges)
    valid = [math.isfinite(v) and v > 0 for v in ranges]
    count = 0
    for i in range(1, len(ranges)-1):
        if valid[i] or not (valid[i-1] and valid[i+1]):
            continue
        left, right = ranges[i-1], ranges[i+1]
        if abs(left-right) > 0.3:
            continue
        result[i] = 2*left*right*math.cos(angle_increment)/(left+right)
        count += 1
    return result, count
