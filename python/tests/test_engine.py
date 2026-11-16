import socket
import threading
import time
import unittest
from drone_sim.transport import Engine
from drone_sim.simulation import Simulation
from drone_sim.protocol import decode,encode
from drone_sim.mock import MockController

class EngineTests(unittest.TestCase):
    def test_commands_feedback_and_history(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(('127.0.0.1',0)); sock.settimeout(.005)
        stop=threading.Event(); c=MockController()
        def server():
            while not stop.is_set():
                try:
                    data,peer=sock.recvfrom(4096); p=decode(data,False); ack=c.accept(p,time.monotonic())
                    if ack: sock.sendto(encode(ack,False),peer)
                    sock.sendto(encode(c.state(time.monotonic()),False),peer)
                except socket.timeout: pass
        t=threading.Thread(target=server); t.start(); e=Engine(Simulation()); e.start()
        try:
            e.command('connect',kind='UDP',port='',host='127.0.0.1',remote=sock.getsockname()[1],local=0,hz=25)
            time.sleep(.1); e.command('control',action='CALIBRATE',az_mech_deg=0)
            time.sleep(.2); s=e.snapshot()
            self.assertTrue(s['ack']['ok']); self.assertIsNotNone(s['matched']); self.assertEqual(s['rx']['status'],'NO_FEEDBACK')
            e.command('control',action='DISARM'); time.sleep(.2); self.assertFalse(c.armed)
            e.command('feedback',enabled=False); time.sleep(.1); self.assertIsNone(e.snapshot()['rx'])
            sent=e.snapshot()['count']['tx_sent']; time.sleep(.1); self.assertGreater(e.snapshot()['count']['tx_sent'],sent)
            e.command('disconnect'); time.sleep(.05); self.assertFalse(e.snapshot()['connected'])
        finally:
            e.quit.set(); e.join(2); stop.set(); t.join(2); sock.close()
    def test_invalid_port(self):
        e=Engine(Simulation()); e.start()
        try:
            e.command('connect',kind='UART',port='/nonexistent/device',host='',remote=0,local=0,hz=25)
            time.sleep(.1); s=e.snapshot(); self.assertFalse(s['connected']); self.assertTrue(s['error'])
        finally: e.quit.set(); e.join(2)

if __name__=='__main__': unittest.main()
