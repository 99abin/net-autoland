"""Local tangent-circle geometry for ordinary NAV_WAYPOINTs, not enforced arcs."""
import math
from .geometry import R, wrap, finite
from .mission_builder import Item, landing_pair
from .adaptive_approach import track_groundspeed


def local(lat,lon,origin):
    return (R*math.cos(math.radians(origin[0]))*wrap(math.radians(lon-origin[1])),
            R*math.radians(lat-origin[0]))


def add(a,b):return (a[0]+b[0],a[1]+b[1])
def sub(a,b):return (a[0]-b[0],a[1]-b[1])
def mul(a,s):return (a[0]*s,a[1]*s)
def dot(a,b):return a[0]*b[0]+a[1]*b[1]
def rot(a):return (-a[1],a[0])
def norm(a):return math.hypot(*a)


def alignment_items(config,c,end,east,north,cruise_tas):
    """Solve both circle sides and alignment setback together, choose shortest path.

    Circle tangent at ALIGNMENT; approach and LAND lie downstream on the same
    line. END->Adjustment is tangent in the matching direction of rotation.
    Actual L1 waypoint trajectories can cut the reference arc.
    """
    c.validate();config.validate()
    origin=(config.land_lat,config.land_lon)
    pair=landing_pair(config)
    app=local(pair[0].x_lat,pair[0].y_long,origin)
    u=mul(app,-1/norm(app));left=rot(u)
    end=local(end.x_lat,end.y_long,origin)
    speed=track_groundspeed(cruise_tas,config.approach_heading_deg,east,north,c.min_predicted_groundspeed_mps)
    straight=speed*c.alignment_straight_time_s+c.alignment_margin_m
    radius=c.turn_radius_m
    candidates=[]

    def candidate(distance,side):
        q=sub(app,mul(u,distance));center=add(q,mul(left,side*radius))
        v=sub(end,center);d2=dot(v,v)
        if d2<=radius*radius+1e-6:return None
        for sign in (-1,1):
            t=add(center,add(mul(v,radius*radius/d2),
                mul(rot(v),sign*radius*math.sqrt(d2-radius*radius)/d2)))
            incoming=sub(t,end);incoming=mul(incoming,1/norm(incoming))
            tangent=mul(rot(sub(t,center)),side/radius)
            if dot(incoming,tangent)<1-1e-7:continue
            chord=sub(q,t);length=norm(chord)
            if length<1.:return None
            delta=math.acos(max(-1.,min(1.,dot(chord,u)/length)))
            if delta>math.radians(170):return None
            required=radius*math.tan(delta/2)+straight
            arc=(side*(math.atan2(q[1]-center[1],q[0]-center[0])-
                       math.atan2(t[1]-center[1],t[0]-center[0])))%(2*math.pi)
            return dict(adjustment=t,alignment=q,circle_center=center,circle_side='left' if side==1 else 'right',
                alignment_distance_m=distance,alignment_turn_deg=math.degrees(delta),
                alignment_turn_allowance_m=required-straight,alignment_straight_allowance_m=straight,
                reference_arc_deg=math.degrees(arc),reference_path_m=norm(sub(t,end))+radius*arc+distance,
                residual=distance-required)
        return None

    # An already collinear entry needs no artificial circle or loop.
    q=sub(app,mul(u,straight));lead=dot(sub(q,end),u)
    if (c.alignment_distance_min_m<=straight<=c.alignment_distance_max_m
            and abs(dot(sub(end,q),left))<.1 and lead>20):
        candidates.append(dict(adjustment=sub(q,mul(u,min(radius,lead/2))),alignment=q,
            circle_center=None,circle_side='straight',alignment_distance_m=straight,
            alignment_turn_deg=0.,alignment_turn_allowance_m=0.,alignment_straight_allowance_m=straight,
            reference_arc_deg=0.,reference_path_m=lead+straight,residual=0.))
    # Bracket roots separately on both continuous tangent branches. Refinement
    # refuses branch discontinuities by checking the final equation residual.
    for side in (-1,1):
        previous=None
        for i in range(513):
            distance=c.alignment_distance_min_m+(c.alignment_distance_max_m-c.alignment_distance_min_m)*i/512
            value=candidate(distance,side)
            if value is None:previous=None;continue
            if abs(value['residual'])<.001:candidates.append(value)
            if previous and previous['residual']*value['residual']<0:
                lo,hi=previous,value
                for _ in range(48):
                    mid=candidate((lo['alignment_distance_m']+hi['alignment_distance_m'])/2,side)
                    if mid is None:break
                    if lo['residual']*mid['residual']<=0:hi=mid
                    else:lo=mid
                best=min((lo,hi),key=lambda x:abs(x['residual']))
                if abs(best['residual'])<.01:candidates.append(best)
            previous=value
    if not candidates:raise ValueError('No bounded tangent/alignment solution for original END waypoint')
    solution=min(candidates,key=lambda x:x['reference_path_m'])
    if not all(finite(*solution[k]) for k in ('adjustment','alignment')):
        raise ValueError('Non-finite alignment solution')

    def waypoint(point):
        lat=origin[0]+math.degrees(point[1]/R)
        lon=origin[1]+math.degrees(point[0]/(R*math.cos(math.radians(origin[0]))))
        if not -89<=lat<=89 or not -180<=lon<=180:raise ValueError('Alignment GPS out of bounds')
        return Item(x_lat=lat,y_long=lon,z_alt=config.approach_alt)

    items=[waypoint(solution['adjustment']),waypoint(solution['alignment'])]+pair
    solution.pop('residual')
    solution.update(turn_radius_m=radius,alignment_groundspeed_mps=speed,
        alignment_straight_time_s=c.alignment_straight_time_s,alignment_margin_m=c.alignment_margin_m,
        appended_items=[dict(name=name,lat=w.x_lat,lon=w.y_long,alt=w.z_alt)
            for name,w in zip(('adjustment','alignment','approach','land'),items)])
    return items,solution
