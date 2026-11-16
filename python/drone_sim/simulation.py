import math
import threading
from .protocol import MASK

def geometry(x, y, z):
    h = math.hypot(x, y)
    return ((math.degrees(math.atan2(x,y)) % 360) if h else None,
            math.degrees(math.atan2(z,h)) if h or z else None, h, math.hypot(h,z))

def wrap180(a):
    return (a+180) % 360-180

def mechanical_target(az, current, low, high, offset=0):
    base = az-offset
    a, b = math.ceil((low-base)/360), math.floor((high-base)/360)
    if a > b: return None
    k = max(a, min(b, round((current-base)/360)))
    return base+360*k

class Simulation:
    def __init__(self):
        self.lock = threading.RLock()
        self.reset()
    def reset(self):
        with self.lock:
            self.x, self.y, self.z = 0.,100.,50.
            self.vx = self.vy = self.vz = self.t = 0.
            self.state, self.flight, self.scale = 'STOP','manual',1.
            self.manual_antenna = False
            self.manual_az, self.manual_el = 0.,0.
            self.end = (100.,100.,50.)
            self.velocity = 10.
            self.circle = (0.,0.,100.,50.,0.,1.)
            self.phi = 0.
    def configure(self, flight, **kw):
        with self.lock:
            self.flight = flight
            for k,v in kw.items(): setattr(self,k,v)
            if flight == 'circle':
                self.phi = math.radians(self.circle[4])
                self._circle(0)
    def _circle(self, dt):
        cx,cy,r,z,_,sign = self.circle
        omega = sign*self.velocity/r
        self.phi += omega*dt
        self.x, self.y, self.z = cx+r*math.cos(self.phi), cy+r*math.sin(self.phi), z
        self.vx, self.vy, self.vz = -r*omega*math.sin(self.phi), r*omega*math.cos(self.phi), 0.
    def step(self, dt):
        with self.lock:
            if self.state != 'RUN': return
            dt *= self.scale
            self.t += dt
            if self.flight == 'circle': self._circle(dt)
            elif self.flight == 'line':
                d = [a-b for a,b in zip(self.end,(self.x,self.y,self.z))]
                length = math.sqrt(sum(a*a for a in d))
                if length <= self.velocity*dt or length == 0:
                    self.x,self.y,self.z = self.end
                    self.vx=self.vy=self.vz=0.
                    self.state='STOP'
                else:
                    self.vx,self.vy,self.vz = (a*self.velocity/length for a in d)
                    self.x+=self.vx*dt; self.y+=self.vy*dt; self.z+=self.vz*dt
            else:
                self.x+=self.vx*dt; self.y+=self.vy*dt; self.z=max(0.,self.z+self.vz*dt)
                if self.z == 0 and self.vz < 0: self.vz=0.
    def packet(self, seq, host_ms):
        with self.lock:
            x,y,z=self.x,self.y,self.z
            vx,vy,vz=(self.vx,self.vy,self.vz) if self.state=='RUN' else (0.,0.,0.)
            az,el,_,_=geometry(x,y,z)
            mode=self.state
            if self.manual_antenna:
                az,el=self.manual_az,self.manual_el
                a,e=math.radians(az),math.radians(el)
                x,y,z=100*math.cos(e)*math.sin(a),100*math.cos(e)*math.cos(a),100*math.sin(e)
                vx=vy=vz=0.; mode='RUN'
            return dict(protocol='DRONE_SIM',version=2,seq=seq & MASK,sim_ms=int(self.t*1000)&MASK,
                        host_ms=host_ms & MASK,mode=mode,x=x,y=y,z=z,vx=vx,vy=vy,vz=vz,
                        speed=math.sqrt(vx*vx+vy*vy+vz*vz),az_deg=az,el_deg=el,
                        az_valid=az is not None,el_valid=el is not None)
