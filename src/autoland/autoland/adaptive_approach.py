"""Integrate a piecewise-linear true-airspeed profile through steady wind."""
from dataclasses import dataclass, field
import math
from .geometry import finite


@dataclass
class AdaptiveConfig:
    t_reference_s: float = 33.3333333333
    airspeed_profile_time_fractions: list = field(default_factory=lambda: [0., .8, 1.])
    airspeed_profile_tas_mps: list = field(default_factory=lambda: [15.45, 15.45, 13.70])
    approach_distance_min_m: float = 300.
    approach_distance_max_m: float = 900.
    max_planned_sink_mps: float = 3.
    max_wind_mps: float = 10.
    min_predicted_groundspeed_mps: float = 3.
    wind_window_s: float = 8.
    wind_timeout_s: float = 2.
    wind_max_std_mps: float = 1.
    wind_min_samples: int = 20
    minimum_planning_alt_m: float = 20.
    minimum_route_time_s: float = 30.
    append_timeout_s: float = 5.
    turn_radius_m: float = 60.
    alignment_straight_time_s: float = 8.
    alignment_margin_m: float = 20.
    alignment_distance_min_m: float = 50.
    alignment_distance_max_m: float = 1000.

    def validate(self):
        scalars = [v for v in self.__dict__.values() if not isinstance(v, list)]
        f, v = self.airspeed_profile_time_fractions, self.airspeed_profile_tas_mps
        if not finite(*scalars, *f, *v) or self.t_reference_s <= 0:
            raise ValueError('Non-finite or nonpositive adaptive approach settings')
        if len(f) != len(v) or len(f)<2 or f[0]!=0 or f[-1]!=1 or any(b<=a for a,b in zip(f,f[1:])):
            raise ValueError('Airspeed profile fractions must increase from 0 to 1')
        if min(v)<=0 or not 0<self.approach_distance_min_m<self.approach_distance_max_m:
            raise ValueError('Invalid airspeed or approach distance limits')
        if min(self.max_planned_sink_mps,self.max_wind_mps,self.min_predicted_groundspeed_mps,
               self.wind_window_s,self.wind_timeout_s,self.wind_max_std_mps,self.minimum_planning_alt_m,
               self.minimum_route_time_s,self.append_timeout_s)<=0:
            raise ValueError('Adaptive planning limits must be positive')
        if self.wind_min_samples<3:
            raise ValueError('Invalid wind observation requirements')
        if min(self.turn_radius_m,self.alignment_straight_time_s,self.alignment_margin_m,
               self.alignment_distance_min_m)<=0 or self.alignment_distance_max_m<=self.alignment_distance_min_m:
            raise ValueError('Invalid turn radius or alignment distance settings')


def integrate_distance(c, heading_deg, wind_east, wind_north, height_drop):
    c.validate()
    if not finite(heading_deg,wind_east,wind_north,height_drop) or height_drop<=0:
        raise ValueError('Invalid wind, heading or height drop')
    if height_drop/c.t_reference_s>=c.max_planned_sink_mps:
        raise ValueError('Reference duration requires excessive average sink rate')
    if math.hypot(wind_east,wind_north)>c.max_wind_mps:
        raise ValueError('Wind exceeds configured planning limit')
    h=math.radians(heading_deg)
    along=wind_east*math.sin(h)+wind_north*math.cos(h)
    cross=wind_east*math.cos(h)-wind_north*math.sin(h)
    def speed(va):
        if va<=abs(cross):
            raise ValueError('Crosswind prevents the planned ground track')
        ground=math.sqrt(va*va-cross*cross)+along
        if ground<=c.min_predicted_groundspeed_mps:
            raise ValueError('Predicted approach groundspeed is too low')
        return ground
    # Simpson quadrature on every linear-profile segment; endpoints are included
    # in feasibility checks. Constant and no-crosswind profiles integrate exactly.
    distance=0.
    for i,(a,b) in enumerate(zip(c.airspeed_profile_time_fractions,c.airspeed_profile_time_fractions[1:])):
        va,vb=c.airspeed_profile_tas_mps[i:i+2]
        n=128
        values=[speed(va+(vb-va)*j/n) for j in range(n+1)]
        distance+=(b-a)*c.t_reference_s/(3*n)*(values[0]+values[-1]
            +4*sum(values[1:-1:2])+2*sum(values[2:-1:2]))
    if not c.approach_distance_min_m<=distance<=c.approach_distance_max_m:
        raise ValueError('Integrated approach distance is outside configured limits')
    return distance,dict(wind_along_mps=along,wind_cross_mps=cross,
        predicted_average_groundspeed_mps=distance/c.t_reference_s,
        planned_average_sink_mps=height_drop/c.t_reference_s)


def stable_wind(c, samples, now):
    """Samples are (ROS receipt seconds, east, north); no direction-change gate."""
    if not samples or now-samples[-1][0]>c.wind_timeout_s:
        raise ValueError('Wind stream is stale')
    samples=[s for s in samples if now-s[0]<=c.wind_window_s+1.]
    if len(samples)<c.wind_min_samples or samples[-1][0]-samples[0][0]<c.wind_window_s:
        raise ValueError('Collecting wind observations')
    east=sum(s[1] for s in samples)/len(samples)
    north=sum(s[2] for s in samples)/len(samples)
    std=math.sqrt(sum((s[1]-east)**2+(s[2]-north)**2 for s in samples)/len(samples))
    if std>c.wind_max_std_mps:
        raise ValueError('Wind estimate is not stable')
    if not finite(east,north,std):
        raise ValueError('Invalid wind observations')
    return east,north,std


def track_groundspeed(tas, heading_deg, east, north, minimum=3.):
    """Steady-wind, horizontal track-holding speed; wind is toward ENU."""
    if not finite(tas,heading_deg,east,north) or tas<=0:
        raise ValueError('Invalid cruise speed or wind')
    h=math.radians(heading_deg)
    along=east*math.sin(h)+north*math.cos(h)
    cross=east*math.cos(h)-north*math.sin(h)
    if tas<=abs(cross):raise ValueError('Crosswind prevents the planned ground track')
    speed=math.sqrt(tas*tas-cross*cross)+along
    if speed<=minimum:raise ValueError('Predicted route groundspeed is too low')
    return speed


def cruise_true_airspeed(eas, altitude_amsl):
    """ISA troposphere density estimate, not an aircraft atmospheric measurement."""
    if not finite(eas,altitude_amsl) or eas<=0 or not -500<=altitude_amsl<=11000:
        raise ValueError('Invalid FCU cruise EAS or AMSL altitude for ISA conversion')
    density_ratio=(1.-.0065*altitude_amsl/288.15)**(9.80665/(287.05*.0065)-1.)
    return eas/math.sqrt(density_ratio)


def remaining_route(points, tas, east, north, minimum=3.):
    """Current GPS then active original WP through END, individually wind projected."""
    from .geometry import R, wrap
    if len(points)<2:raise ValueError('No remaining original route')
    if not all(finite(*p) and -89<=p[0]<=89 and -180<=p[1]<=180 for p in points):
        raise ValueError('Invalid route GPS coordinates')
    legs=[]
    for a,b in zip(points,points[1:]):
        n=R*math.radians(b[0]-a[0])
        e=R*math.cos(math.radians(a[0]))*wrap(math.radians(b[1]-a[1]))
        distance=math.hypot(e,n)
        if distance<.01:continue
        heading=math.degrees(math.atan2(e,n))%360
        speed=track_groundspeed(tas,heading,east,north,minimum)
        legs.append(dict(distance_m=distance,heading_deg=heading,groundspeed_mps=speed,time_s=distance/speed))
    return dict(remaining_route_distance_m=sum(x['distance_m'] for x in legs),
        remaining_route_time_s=sum(x['time_s'] for x in legs),route_legs=legs)
