"""Stereo design calculations, not a promised or measured accuracy specification."""
import math


def positive(value, name, allow_zero=False):
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f'{name} must be finite and {"nonnegative" if allow_zero else "positive"}')


def focal_from_hfov(width, hfov_deg):
    positive(width, 'width')
    if not math.isfinite(hfov_deg) or not 1 < hfov_deg < 179:
        raise ValueError('HFOV must be between 1 and 179 degrees')
    return width/(2*math.tan(math.radians(hfov_deg)/2))


def error_budget(distance_m, focal_px, baseline_m, disparity_error_px,
                 skew_s=0., lateral_speed_mps=0., yaw_rate_rad_s=0.,
                 scale_error_fraction=0., target_error_m=1.):
    """Conditional worst direction depth change, exact inverse-disparity relation.

    The motion terms are small-angle/central-field approximations. Caller must
    supply defensible error bounds: they are not estimated from SGBM confidence.
    Does not include forward motion, arbitrary object motion, rolling-shutter
    geometry, false correspondences or unknown calibration biases.
    """
    for name,value in [('distance',distance_m),('focal',focal_px),('baseline',baseline_m),
                       ('target_error',target_error_m)]:
        positive(value,name)
    for name,value in [('disparity_error',disparity_error_px),('skew',skew_s),
                       ('lateral_speed',lateral_speed_mps),('yaw_rate',yaw_rate_rad_s),
                       ('scale_error_fraction',scale_error_fraction)]:
        positive(value,name,True)
    if scale_error_fraction >= 1:
        raise ValueError('scale_error_fraction must be <1')
    (distance_m,focal_px,baseline_m,disparity_error_px,skew_s,lateral_speed_mps,
     yaw_rate_rad_s,scale_error_fraction,target_error_m)=map(float,
        (distance_m,focal_px,baseline_m,disparity_error_px,skew_s,lateral_speed_mps,
         yaw_rate_rad_s,scale_error_fraction,target_error_m))
    fb=focal_px*baseline_m
    disparity=fb/distance_m
    motion=focal_px*skew_s*(lateral_speed_mps/distance_m+yaw_rate_rad_s)
    total=disparity_error_px+motion
    lower=fb*(1-scale_error_fraction)/(disparity+total)
    upper=fb*(1+scale_error_fraction)/(disparity-total) if disparity>total else None
    error=max(distance_m-lower,upper-distance_m) if upper is not None else None
    return dict(distance_m=distance_m,disparity_px=disparity,
                assumed_disparity_error_px=disparity_error_px,motion_disparity_px=motion,
                lower_m=lower,upper_m=upper,conditional_error_m=error,
                within_assumed_budget=error is not None and error<=target_error_m,
                accuracy_verified=False)
