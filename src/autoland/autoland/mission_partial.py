"""Minimal MAVLink2 framing for one-item ArduPilot append via MAVROS router.

No socket, Python environment change or independent aircraft connection.
"""
import struct

CRC_EXTRA={38:9,39:254,73:38}


def crc(data):
    value=0xffff
    for byte in data:
        tmp=byte^(value&255)
        tmp=(tmp^(tmp<<4))&255
        value=((value>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    return value


def packet(msgid,payload,seq,sysid=1,compid=192):
    # MAVConn trims trailing zero bytes when serializing MAVLink2. Compute CRC
    # over that same canonical length, including for zero-valued extensions.
    payload=payload.rstrip(b'\0') or b'\0'
    header=bytes([len(payload),0,0,seq,sysid,compid])+msgid.to_bytes(3,'little')
    return dict(magic=253,len=len(payload),incompat_flags=0,compat_flags=0,seq=seq,
        sysid=sysid,compid=compid,msgid=msgid,
        checksum=crc(header+payload+bytes([CRC_EXTRA[msgid]])),
        payload64=list(struct.unpack('<'+'Q'*((len(payload)+7)//8),payload.ljust((len(payload)+7)//8*8,b'\0'))))


def write_one(index,seq):
    if not 1<=index<=32767:
        raise ValueError('Append index outside partial-write range')
    return packet(38,struct.pack('<hhBBB',index,index,1,1,0),seq)


def item_packet(item,index,seq,use_int=True):
    tail=(index,item.command,1,1,6 if use_int else 3,0,1,0)
    if use_int:
        payload=struct.pack('<ffffiifHHBBBBBB',item.param1,item.param2,item.param3,item.param4,
            round(item.x_lat*1e7),round(item.y_long*1e7),item.z_alt,*tail)
    else:
        payload=struct.pack('<fffffffHHBBBBBB',item.param1,item.param2,item.param3,item.param4,
            item.x_lat,item.y_long,item.z_alt,*tail)
    return packet(73 if use_int else 39,payload,seq)


def decode_reply(message,index):
    if message.sysid!=1 or message.compid!=1 or message.framing_status!=1:
        return None
    data=b''.join(struct.pack('<Q',v) for v in message.payload64)[:message.len]
    if message.msgid in (40,51):
        data=data.ljust(5,b'\0')
        request,system,component,mission_type=struct.unpack('<HBBB',data[:5])
        if (system,component,mission_type)!=(1,192,0):
            return None
        if request!=index:
            raise ValueError('FCU requested an unexpected mission index')
        return ('request',message.msgid==51)
    if message.msgid==47:
        data=data.ljust(4,b'\0')
        system,component,result,mission_type=data[:4]
        if (system,component,mission_type)==(1,192,0):
            return ('ack',result)
    return None
