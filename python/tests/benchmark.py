"""60s host scheduler + fragmented PTY/UDP integration. PTY does not emulate AVR."""
import argparse,json,os,pty,select,socket,threading,time
from drone_sim.transport import Engine
from drone_sim.simulation import Simulation
from drone_sim.protocol import decode,encode,Framer
from drone_sim.mock import MockController

def run(kind,duration):
    stop=threading.Event(); c=MockController(); frames=Framer(); accepted=0
    if kind=='UART':
        master,slave=pty.openpty(); port=os.ttyname(slave); os.set_blocking(master,False)
        def read():
            try: return os.read(master,4096)
            except BlockingIOError: return b''
        def write(data):
            # Stream in fragments to exercise framing; serial physical rate is not modeled.
            os.write(master,data[:31]); os.write(master,data[31:])
        args=dict(kind='UART',port=port,host='',remote=0,local=0,hz=25)
    else:
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(('127.0.0.1',0)); sock.setblocking(False); peer=[None]
        def read():
            try:
                data,peer[0]=sock.recvfrom(4096); return data
            except BlockingIOError: return b''
        def write(data):
            if peer[0]: sock.sendto(data,peer[0])
        args=dict(kind='UDP',port='',host='127.0.0.1',remote=sock.getsockname()[1],local=0,hz=25)
    def server():
        nonlocal accepted
        due=time.monotonic()
        while not stop.is_set():
            now=time.monotonic(); data=read()
            for raw in (frames.feed(data) if kind=='UART' else ([data] if data else [])):
                p=decode(raw,kind=='UART'); ack=c.accept(p,now)
                if p['protocol']=='DRONE_SIM': accepted+=1
                if ack: write(encode(ack,kind=='UART'))
            if now>=due:
                write(encode(c.state(now),kind=='UART')); due=now+.04
            stop.wait(.001)
    thread=threading.Thread(target=server); thread.start()
    m=Simulation(); e=Engine(m); e.start(); e.command('connect',**args)
    start=time.monotonic()
    while time.monotonic()-start<duration:
        elapsed=time.monotonic()-start
        with m.lock: m.state='RUN' if elapsed<duration/3 else ('PAUSE' if elapsed<2*duration/3 else 'STOP')
        time.sleep(.02)
    snapshot=e.snapshot(); e.quit.set(); e.join(2); stop.set(); thread.join(2)
    if kind=='UART': os.close(master); os.close(slave)
    else: sock.close()
    result=dict(kind=kind,seconds=duration,accepted=accepted,**snapshot['count'],error=snapshot['error'])
    print(json.dumps(result),flush=True)
    assert result.get('tx_sent',0)>=duration*20 and result.get('rx_valid',0)>=duration*20 and not result['error']
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=60); p.add_argument('--kind',choices=['UART','UDP'],default='UART'); a=p.parse_args(); run(a.kind,a.seconds)
