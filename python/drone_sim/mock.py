"""PC-only UDP/PTY test peer. Not firmware and not a physical position sensor."""
import argparse
import math
import socket
import time
from .protocol import decode, encode, newer, MASK
from .simulation import mechanical_target

class MockController:
    def __init__(self):
        self.armed=False; self.source=None; self.received=None; self.seq=0
        self.az=None; self.el=None; self.position=0.; self.reason='UNCALIBRATED_OR_ARMED'
        self.control_seq=None; self.ack=None
    def accept(self,p,now):
        if p['protocol']=='ANTENNA_CONTROL':
            if p['seq']==self.control_seq: return self.ack
            if not newer(p['seq'],self.control_seq): return None
            ok=True; reason='OK'
            if p['action']=='DISARM': self.armed=False; self.reason='UNCALIBRATED_OR_ARMED'
            elif self.armed: ok=False; reason='UNCALIBRATED_OR_ARMED'
            elif not -90<=p['az_mech_deg']<=90: ok=False; reason='AZ_UNREACHABLE'
            else:
                self.position=p['az_mech_deg']; self.az=self.position%360; self.el=0.; self.armed=True; self.reason='OK'
            self.control_seq=p['seq']; self.ack=dict(protocol='ANTENNA_ACK',version=1,seq=p['seq'],ok=ok,reason=reason)
            return self.ack
        if p['protocol']!='DRONE_SIM' or not newer(p['seq'],self.source): return None
        self.source=p['seq']; self.received=now
        if self.armed:
            self.reason='OK'
            if p['az_valid']:
                m=mechanical_target(p['az_deg'],self.position,-90,90)
                if m is None: self.reason='AZ_UNREACHABLE'
                else: self.position=m; self.az=m%360
            if p['el_valid']: self.el=max(0,min(90,p['el_deg']))
        return None
    def state(self,now):
        status='STALE' if self.received is None or now-self.received>.25 else ('FAULT' if self.reason!='OK' else 'NO_FEEDBACK')
        p=dict(protocol='ANTENNA_STATE',version=2,seq=self.seq,source_seq=self.source,mcu_ms=int(now*1000)&MASK,
               cmd_az_deg=self.az,cmd_el_deg=self.el,act_az_deg=None,act_el_deg=None,act_az_valid=False,act_el_valid=False,status=status,reason=self.reason)
        self.seq=(self.seq+1)&MASK; return p

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--port',type=int,default=5005); args=parser.parse_args()
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(('127.0.0.1',args.port)); sock.settimeout(.005)
    controller=MockController(); peer=None; due=time.monotonic()
    print(f'Mock: 127.0.0.1:{args.port}; example AZ limits [-90,90]; no sensors. Ctrl+C to stop.')
    try:
        while True:
            now=time.monotonic()
            try:
                data,address=sock.recvfrom(4096); p=decode(data,False)
                if p['protocol'] not in ('DRONE_SIM','ANTENNA_CONTROL'): continue
                peer=address; ack=controller.accept(p,now)
                if ack: sock.sendto(encode(ack,False),peer)
            except (socket.timeout,ValueError): pass
            if peer and now>=due:
                sock.sendto(encode(controller.state(now),False),peer); due=now+.04
    except KeyboardInterrupt: pass
    finally: sock.close()

if __name__=='__main__': main()
