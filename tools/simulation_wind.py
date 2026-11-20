"""Local JSON SITL relay for consistent world wind and simulated pitot speed."""
import argparse
import json
import math
import socket


def state_with_wind(state, wind_enu):
    """JSON uses NED velocity and a body-FRD to NED quaternion."""
    east,north,up=wind_enu
    if not all(math.isfinite(v) for v in wind_enu):
        raise ValueError('Wind must be finite')
    wind=[north,east,-up]
    velocity=state['velocity']
    w,x,y,z=state['quaternion']
    norm=math.sqrt(w*w+x*x+y*y+z*z)
    if norm<1e-9:raise ValueError('Invalid orientation')
    w,x,y,z=(v/norm for v in (w,x,y,z))
    forward=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)]
    relative=[v-a for v,a in zip(velocity,wind)]
    output=dict(state)
    output['velocity_wind']=wind
    output['airspeed']=max(0.,sum(a*b for a,b in zip(relative,forward)))
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--east',type=float,required=True)
    parser.add_argument('--north',type=float,required=True)
    parser.add_argument('--up',type=float,default=0.)
    args=parser.parse_args();wind=[args.east,args.north,args.up]
    peer=None;count=0
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as channel:
        channel.bind(('127.0.0.1',9002))
        print('World wind ENU',wind,'relay 9002 to Gazebo 9003',flush=True)
        while True:
            packet,address=channel.recvfrom(65535)
            if address[0]!='127.0.0.1':continue
            if address[1]==9003:
                if peer is None:continue
                state=state_with_wind(json.loads(packet),wind)
                channel.sendto(('\n'+json.dumps(state,separators=(',',':'))+'\n').encode(),peer)
                count+=1
                if count==1:print('First sensor state relayed',state['velocity_wind'],state['airspeed'],flush=True)
            else:
                peer=address
                channel.sendto(packet,('127.0.0.1',9003))

if __name__=='__main__':main()
