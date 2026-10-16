#!/usr/bin/env python3
"""Agent C independent, read-only, narrow PX4 ULog v1 parser.
Only parses supported primitive PX4 v1.16 ULog F/A/D/P/L entries; no image OCR.
This is not a full ULog decoder. Timestamps printed are PX4 boot seconds.
"""
import sys,struct,collections
from pathlib import Path
TYPES={'uint64_t':'Q','int64_t':'q','uint32_t':'I','int32_t':'i','uint16_t':'H',
       'int16_t':'h','uint8_t':'B','int8_t':'b','char':'c','bool':'?','float':'f','double':'d'}
PARAMS=['COM_OF_LOSS_T','COM_OBL_RC_ACT','COM_RC_IN_MODE','COM_RCL_EXCEPT','COM_FAIL_ACT_T','COM_LOW_BAT_ACT','NAV_RCL_ACT','COM_RC_OVERRIDE','MPC_THR_HOVER']
TOPICS={'vehicle_status','vehicle_land_detected','vehicle_local_position','offboard_control_mode','failsafe_flags','battery_status','vehicle_attitude_setpoint'}
def parse_fmt(fmt):
    fields=[];off=0
    for desc in fmt.split(':',1)[1].strip(';').split(';'):
        if not desc:continue
        typ,name=desc.split(' ',1)
        if '[' in typ:
            t,n=typ[:-1].split('[');count=int(n)
        else:t=typ;count=1
        if t not in TYPES: raise ValueError((t,name))
        f='<'+TYPES[t]*count;size=struct.calcsize(f)
        fields.append((name,off,f,count));off+=size
    return fields,off

def parse(path):
    blob=Path(path).read_bytes()
    if not blob.startswith(b'ULog\x01\x125'):raise ValueError('bad ULog magic')
    p=16;fmts={};topics={};records=collections.defaultdict(list);params={};logs=[]
    while p+3<=len(blob):
        n,t=struct.unpack_from('<HB',blob,p);p+=3
        if p+n>len(blob):raise ValueError('truncated ULog record')
        b=blob[p:p+n];p+=n
        if t==ord('F'):
            txt=b.decode('utf8','replace');fmts[txt.split(':',1)[0]]=txt
        elif t==ord('A'):
            topics[struct.unpack_from('<H',b,1)[0]]=b[3:].decode('utf8','replace')
        elif t==ord('P') and b:
            k=b[0];key=b[1:1+k].decode(errors='replace');raw=b[1+k:]
            if len(raw)==4 and ' ' in key:
                ty,name=key.split(' ',1)
                if name in PARAMS:params[name]=struct.unpack('<i' if ty=='int32_t' else '<f',raw)[0]
        elif t==ord('L') and len(b)>=9:
            logs.append((struct.unpack_from('<Q',b,1)[0]/1e6,b[9:].decode(errors='replace').strip()))
        elif t==ord('D') and len(b)>10:
            mid=struct.unpack_from('<H',b,0)[0]
            topic=topics.get(mid)
            if topic in TOPICS and topic in fmts:
                if topic not in records or not records[topic]:
                    pass
                fields,size=parse_fmt(fmts[topic])
                data=b[2:]
                # ULog may omit the trailing explicit _padding fields; all named fields
                # still have to fit individually before unpacking.
                values={}
                for key,off,fmt,count in fields:
                    if key.startswith('_padding'):continue
                    v=struct.unpack_from(fmt,data,off)
                    values[key]=v[0] if count==1 else v
                records[topic].append(values)
    return params,logs,records

def transitions(rows,fields):
    last=None;out=[]
    for r in rows:
        key=tuple(r.get(f) for f in fields)
        if key!=last:out.append((r['timestamp']/1e6,dict(zip(fields,key))));last=key
    return out
if __name__=='__main__':
    for path in sys.argv[1:]:
        pars,logs,records=parse(path)
        print('\n===',Path(path).name,'===')
        print('PARAMS:', ' '.join(f'{k}={pars.get(k,"MISSING")}' for k in PARAMS))
        for time,msg in logs:
            if any(q in msg.lower() for q in ['armed by','disarmed','takeoff detected','landing detected','failsafe','battery warning','no manual','matching flight task']):
                print(f'LOG t={time:.3f}s {msg}')
        for topic,fields in [
            ('vehicle_status',['arming_state','nav_state','failsafe']),
            ('vehicle_land_detected',['landed','at_rest']),
            ('failsafe_flags',['offboard_control_signal_lost','manual_control_signal_lost','battery_warning','local_position_invalid']),
            ('battery_status',['warning'])]:
            rows=records[topic]
            changes=transitions(rows,fields)
            print(f'TOPIC {topic} count={len(rows)} transitions={len(changes)}')
            for t,k in changes[-30:]:print(f'  {t:10.3f}s {k}')
        print('TOPIC position samples=',len(records['vehicle_local_position']), 'attitude_sp=',len(records['vehicle_attitude_setpoint']), 'offboard_ctl=',len(records['offboard_control_mode']))
        if records['offboard_control_mode']:
            last=records['offboard_control_mode'][-1]['timestamp']/1e6
            losses=transitions(records['failsafe_flags'],['offboard_control_signal_lost'])
            first_loss=next((t for t,k in losses if t>last and k['offboard_control_signal_lost']),None)
            print(f'OFFBOARD LAST logged message t={last:.6f}s; loss flag t={first_loss}; gap={(first_loss-last) if first_loss else None:.6f}s' if first_loss else f'OFFBOARD last={last:.6f}s loss not observed')
        if records['battery_status']:
            print('BATTERY WARNING UNIQUE=',sorted({r['warning'] for r in records['battery_status']}))
        if records['vehicle_local_position']:
            pos=records['vehicle_local_position'];
            t0=next((t for t,msg in logs if 'Takeoff detected' in msg),None)
            if t0:
                window=[r for r in pos if t0-2 <=r['timestamp']/1e6 <=t0+23]
                if window:
                    print('LOCAL POSITION window first=', {k:window[0].get(k) for k in ['timestamp','z','vz','xy_valid','z_valid','v_xy_valid','v_z_valid']})
                    print('LOCAL POSITION window last=', {k:window[-1].get(k) for k in ['timestamp','z','vz','xy_valid','z_valid','v_xy_valid','v_z_valid']})
                    print('LOCAL POSITION window min_z/max_z/min_vz/max_vz=', min(r['z'] for r in window),max(r['z'] for r in window),min(r['vz'] for r in window),max(r['vz'] for r in window))
