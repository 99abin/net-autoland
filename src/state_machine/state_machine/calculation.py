import math
from math import radians, cos

def calc_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0

    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2 - lat1)
    delta_lambda = radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2)
        * math.sin(delta_lambda / 2.0) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters

def calc_bearing_error(curr_lat, curr_lon, target_lat, target_lon, vx, vy):
    if None in (curr_lat, curr_lon, vx, vy):
        return None

    lat1 = math.radians(curr_lat)
    lat2 = math.radians(target_lat)
    delta_lon = math.radians(target_lon - curr_lon)

    y = math.sin(delta_lon) * math.cos(lat2)

    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2)
        * math.cos(delta_lon)
    )

    target_bearing = (
        math.degrees(math.atan2(y, x)) + 360
    ) % 360

    if math.hypot(vx, vy) < 0.5:
        return None

    track_bearing = (
        math.degrees(math.atan2(vx, vy)) + 360
    ) % 360

    bearing_error = (
        target_bearing - track_bearing + 540
    ) % 360 - 180

    return bearing_error
    
def calc_xtrack_error(curr_lat: float, curr_lon: float, start_lat: float, start_lon: float, land_lat: float, land_lon: float) -> float:
    R = 6371000.0  

    lat1, lon1 = math.radians(start_lat), math.radians(start_lon)
    lat2, lon2 = math.radians(land_lat), math.radians(land_lon)
    lat3, lon3 = math.radians(curr_lat), math.radians(curr_lon)

    def get_bearing(la1, lo1, la2, lo2):
        dlon = lo2 - lo1
        y = math.sin(dlon) * math.cos(la2)
        x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlon)
        return math.atan2(y, x)

    dlat = lat3 - lat1
    dlon = lon3 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat3) * math.sin(dlon / 2)**2
    delta_13 = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    theta_13 = get_bearing(lat1, lon1, lat3, lon3)
    theta_12 = get_bearing(lat1, lon1, lat2, lon2)

    xtrack_distance = math.asin(math.sin(delta_13) * math.sin(theta_13 - theta_12)) * R

    return xtrack_distance


def calc_wp_proportion(start_lat, start_lon, curr_lat, curr_lon, land_lat, land_lon):

    R = 6371000.0

    lat_ref = radians(start_lat)

    curr_e = R * cos(lat_ref) * radians(curr_lon - start_lon)
    curr_n = R * radians(curr_lat - start_lat)

    land_e = R * cos(lat_ref) * radians(land_lon - start_lon)
    land_n = R * radians(land_lat - start_lat)

    denominator = land_e**2 + land_n**2

    if denominator < 1e-6:
        raise ValueError("Start and landing positions are too close")

    proportion = (
        curr_e * land_e + curr_n * land_n
    ) / denominator

    return proportion