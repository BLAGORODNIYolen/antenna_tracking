import math
import unittest
from drone_sim.simulation import Simulation,geometry,mechanical_target
from drone_sim.protocol import encode,decode,validate,Framer,newer
from drone_sim.mock import MockController

class GeometryTests(unittest.TestCase):
    def test_cardinal(self):
        for point,az,el in [((0,100,0),0,0),((100,0,0),90,0),((0,-100,0),180,0),((-100,0,0),270,0),((100,0,100),90,45),((0,0,100),None,90),((0,0,0),None,None),((0,0,-100),None,-90)]:
            a,e,*_=geometry(*point); self.assertEqual(a,az); self.assertEqual(e,el)
    def test_mechanics(self):
        self.assertEqual(mechanical_target(270,0,-90,90),-90)
        self.assertIsNone(mechanical_target(180,0,-90,90))
        self.assertEqual(mechanical_target(1,359,0,540),361)
        self.assertEqual(mechanical_target(100,0,-90,90,100),0)

class ModelTests(unittest.TestCase):
    def test_line(self):
        m=Simulation(); m.x=m.y=m.z=0; m.configure('line',end=(3,4,0),velocity=2); m.state='RUN'
        m.step(1); self.assertAlmostEqual(m.x,1.2); m.step(10)
        self.assertEqual((m.x,m.y,m.z),(3,4,0)); self.assertEqual(m.state,'STOP')
    def test_circle(self):
        for sign in (-1,1):
            m=Simulation(); m.configure('circle',circle=(20,30,100,50,0,sign),velocity=10); m.state='RUN'; m.step(2)
            self.assertAlmostEqual(math.hypot(m.x-20,m.y-30),100); self.assertEqual(m.z,50)
            self.assertAlmostEqual(math.hypot(m.vx,m.vy),10); self.assertEqual(m.vy>0,sign>0)
    def test_pause(self):
        m=Simulation(); m.state='RUN'; m.vx=10; m.step(.1); m.state='PAUSE'; m.step(10)
        self.assertEqual(m.t,.1); self.assertEqual(m.packet(0,0)['speed'],0)
        m.state='RUN'; m.step(.1); self.assertEqual(m.x,2)
    def test_manual_zenith(self):
        m=Simulation(); m.manual_antenna=True; m.manual_az=270; m.manual_el=90
        p=m.packet(0,0); self.assertTrue(p['az_valid']); self.assertEqual(p['az_deg'],270); self.assertEqual(p['speed'],0)

class ProtocolTests(unittest.TestCase):
    def setUp(self): self.p=Simulation().packet(1,100)
    def test_roundtrip(self):
        for uart in (True,False):
            raw=encode(self.p,uart); self.assertLessEqual(len(raw),384); self.assertEqual(len(decode(raw,uart)),17)
    def test_bad_types(self):
        for key,value in [('version',True),('version',2.0),('seq',True),('seq',-1),('host_ms',2**32),('x',float('nan')),('speed',-1),('az_deg',360),('az_valid',1),('el_deg',None),('mode','BAD')]:
            with self.subTest(key=key,value=value),self.assertRaises(ValueError): validate(dict(self.p,**{key:value}))
    def test_missing(self):
        for k in self.p:
            p=self.p.copy(); del p[k]
            with self.subTest(key=k),self.assertRaises(ValueError): validate(p)
    def test_framer(self):
        f=Framer(); raw=encode(self.p)
        self.assertEqual(f.feed(raw[:30]),[]); self.assertEqual(f.feed(raw[30:]+raw),[raw,raw])
        self.assertEqual(f.feed(b'x'*400+b'\n'+raw),[raw]); self.assertEqual(f.overflow,1)
    def test_malformed(self):
        for raw in (b'{bad}\n',b'{"seq":1,"seq":2}\n',b'[]\n',b'{}junk\n',b'{}'):
            with self.assertRaises(ValueError): decode(raw)
    def test_size(self):
        with self.assertRaises(ValueError): encode(dict(self.p,extra='a'*384))
    def test_maximum_fields(self):
        p=dict(self.p,seq=0xffffffff,host_ms=0xffffffff,sim_ms=0xffffffff,
               x=-100000.,y=-100000.,z=100000.,vx=-1000.,vy=-1000.,vz=-1000.,
               speed=1732.051,az_deg=359.999,el_deg=-89.999)
        self.assertLessEqual(len(encode(p)),384)
    def test_wrap(self):
        self.assertTrue(newer(0,0xffffffff)); self.assertFalse(newer(0xffffffff,0)); self.assertFalse(newer(1,1))
    def test_rounding(self):
        p=decode(encode(dict(self.p,az_deg=359.9999))); self.assertEqual(p['az_deg'],0)
    def test_no_sensor(self):
        p=MockController().state(0); self.assertEqual(len(p),13)
        self.assertIsNone(p['act_az_deg']); self.assertFalse(p['act_az_valid']); validate(p)
        with self.assertRaises(ValueError): validate(dict(p,act_az_valid=True))
    def test_control_and_stale(self):
        c=MockController(); p=self.p; c.accept(p,0); self.assertEqual(c.state(.1)['status'],'FAULT')
        cmd=dict(protocol='ANTENNA_CONTROL',version=1,seq=0,action='CALIBRATE',az_mech_deg=0)
        self.assertTrue(c.accept(cmd,.1)['ok']); self.assertTrue(c.accept(cmd,.2)['ok'])
        c.accept(dict(p,seq=2,az_deg=270),.2); self.assertEqual(c.az,270)
        c.accept(dict(p,seq=1,az_deg=90),.21); self.assertEqual(c.az,270)
        self.assertEqual(c.state(.451)['status'],'STALE')
        c.accept(dict(p,seq=3,az_deg=180),.5); self.assertEqual(c.state(.51)['status'],'FAULT'); self.assertEqual(c.az,270)

if __name__=='__main__': unittest.main()
