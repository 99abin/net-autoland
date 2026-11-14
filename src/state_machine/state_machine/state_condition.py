def should_enter_approach(bearing_error, xtrack_error, wp_proportion, curr_alt, start_alt):
    if bearing_error is None:
        return False

    bearing_ok = abs(bearing_error) <= 10.0
    xtrack_ok = abs(xtrack_error) <= 5.0

    condition_1 = (
        wp_proportion >= 0.0
        and bearing_ok
        and xtrack_ok
    )

    condition_2 = (
        wp_proportion > 0.15
        and bearing_ok
        and curr_alt <= start_alt
    )

    condition_3 = wp_proportion > 0.5

    return condition_1 or condition_2 or condition_3

def should_enter_preflare(pf_arspd, curr_alt, land_alt, sink_rate, pf_alt, pf_sec):
    height = current_alt - land_alt

    if pf_arspd <= 0:
        return False

    condition_1 = (
        pf_alt > 0
        and height <= pf_alt
    )

    condition_2 = (
        pf_sec > 0
        and sink_rate > 0
        and height <= sink_rate * pf_sec
    )

    return condition_1 or condition_2

def should_enter_flare(curr_alt, f_alt, f_sec, land_alt, sink_rate, wp_proportion):
    height = curr_alt - land_alt

    condition_1 = (
        height <= f_alt
    )

    condition_2 = (
        f_sec > 0.0 and
        sink_rate > 0.0 and
        height <= sink_rate * f_sec and
        wp_proportion > 0.5
    )



