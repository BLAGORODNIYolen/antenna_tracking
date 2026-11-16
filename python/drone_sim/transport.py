"""I/O runs in its own worker; no GUI-timed transmission."""
import collections
import csv
import queue
import socket
import threading
import time
from .protocol import encode, decode, Framer, newer, MASK

class SerialTransport:
    def __init__(self, port):
        import serial
        self.io=serial.serial_for_url(port,baudrate=115200,timeout=0,write_timeout=.08)
    def send(self, data):
        n=self.io.write(data)
        if n != len(data): raise OSError('Partial UART write; disconnect to resynchronize')
    def receive(self):
        n=self.io.in_waiting
        return self.io.read(min(n,4096)) if n else b''
    def close(self): self.io.close()

class UDPTransport:
    """For local mock/testing only; the Mega has no network hardware."""
    def __init__(self, host, remote=5005, local=5006):
        self.peer=(socket.gethostbyname(host),remote)
        self.io=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try: self.io.bind(('0.0.0.0',local)); self.io.setblocking(False)
        except Exception: self.io.close(); raise
    def send(self,data):
        if self.io.sendto(data,self.peer)!=len(data): raise OSError('Partial UDP send')
    def receive(self):
        try:
            data,peer=self.io.recvfrom(4096)
            return data if peer==self.peer else b''
        except BlockingIOError: return b''
    def close(self): self.io.close()

class Engine(threading.Thread):
    def __init__(self, model):
        super().__init__(daemon=True)
        self.model=model; self.commands=queue.Queue(); self.quit=threading.Event()
        self.lock=threading.RLock(); self.link=None; self.uart=True; self.hz=25; self.two_way=True
        self.seq=0; self.control_seq=0; self.start_time=time.monotonic()
        self.framer=Framer(); self.history=collections.OrderedDict()
        self.tx_times=collections.deque(); self.rx_times=collections.deque()
        self.count=collections.Counter(); self.last_rx=None; self.rx_at=0.; self.rx_seq=None
        self.last_tx=None; self.tx_raw=''; self.rx_raw=''; self.error=''; self.connected=False
        self.ack=None; self.pending=None; self.log=None; self.writer=None
    def command(self, operation, **args): self.commands.put((operation,args))
    def close_link(self):
        if self.link:
            try: self.link.close()
            except Exception: pass
        self.link=None; self.connected=False; self.pending=None
    def _command(self, action, args):
        if action=='disconnect': self.close_link()
        elif action=='connect':
            self.close_link(); self.uart=args['kind']=='UART'
            self.link=SerialTransport(args['port']) if self.uart else UDPTransport(args['host'],args['remote'],args['local'])
            self.hz=args['hz']; self.connected=True; self.error=''
            self.rx_seq=None; self.last_rx=None; self.rx_at=0.; self.framer=Framer(); self.history.clear()
            self.tx_times.clear(); self.rx_times.clear()
        elif action=='feedback':
            self.two_way=args['enabled']; self.last_rx=None; self.rx_at=0.; self.rx_seq=None
        elif action=='control':
            if not self.link: raise ValueError('Сначала подключитесь')
            if self.pending: raise ValueError('Дождитесь подтверждения предыдущей команды')
            p=dict(protocol='ANTENNA_CONTROL',version=1,seq=self.control_seq,**args)
            self.control_seq=(self.control_seq+1)&MASK
            self.pending=[p,0.,0]; self.ack=None
        elif action=='log':
            if self.log: self.log.close(); self.log=None; self.writer=None
            if args.get('path'):
                self.log=open(args['path'],'w',newline='',encoding='utf-8')
                self.writer=csv.writer(self.log)
                self.writer.writerow('host_ms sim_ms source_seq target_az target_el cmd_az cmd_el act_az act_el act_az_valid act_el_valid rx_age_ms'.split())
    def snapshot(self):
        with self.lock:
            now=time.monotonic()
            for d in (self.tx_times,self.rx_times):
                while d and d[0]<now-5: d.popleft()
            return dict(connected=self.connected,error=self.error,count=dict(self.count),
                        tx_hz=len(self.tx_times)/5,rx_hz=len(self.rx_times)/5,
                        tx=self.last_tx,rx=self.last_rx,rx_age=(now-self.rx_at) if self.rx_at else None,
                        tx_raw=self.tx_raw,rx_raw=self.rx_raw,ack=self.ack,pending=bool(self.pending),
                        matched=self.history.get(self.last_rx.get('source_seq')) if self.last_rx else None,
                        two_way=self.two_way)
    def _receive(self, data, now):
        frames=self.framer.feed(data) if self.uart else [data]
        self.count['rx_overflow']=self.framer.overflow
        for frame in frames:
            try:
                p=decode(frame,self.uart)
                if p['protocol']=='ANTENNA_ACK':
                    if self.pending and p['seq']==self.pending[0]['seq']:
                        self.ack=p; self.pending=None
                    continue
                if p['protocol']!='ANTENNA_STATE': raise ValueError('Unexpected response')
                if not self.two_way: continue
                if not newer(p['seq'],self.rx_seq): self.count['rx_old']+=1; continue
                if self.rx_seq is not None: self.count['rx_lost']+=((p['seq']-self.rx_seq)&MASK)-1
                self.rx_seq=p['seq']; self.last_rx=p; self.rx_at=now
                self.rx_raw=frame.decode().strip(); self.count['rx_valid']+=1
                self.count['rx_bytes']=len(frame); self.rx_times.append(now)
            except ValueError: self.count['rx_invalid']+=1
    def run(self):
        now=time.monotonic(); model_at=now; tx_at=now
        try:
            while not self.quit.is_set():
                now=time.monotonic()
                # Fixed 50 Hz model; bounded catch-up prevents a freeze after suspend.
                steps=min(25,int((now-model_at)/.02))
                for _ in range(steps): self.model.step(.02)
                model_at+=steps*.02
                if now-model_at>.5: model_at=now
                with self.lock:
                    while not self.commands.empty():
                        action,args=self.commands.get_nowait()
                        try: self._command(action,args)
                        except Exception as e: self.error=str(e)
                    if self.link:
                        try:
                            if self.pending and now-self.pending[1]>.15:
                                if self.pending[2]>=4:
                                    self.error='Нет ACK команды; состояние не подтверждено'; self.pending=None
                                else:
                                    self.link.send(encode(self.pending[0],self.uart)); self.pending[1]=now; self.pending[2]+=1
                            if now>=tx_at:
                                self.count['tx_skipped']+=max(0,int((now-tx_at)*self.hz))
                                tx_at=now+1/self.hz
                                self.count['tx_attempted']+=1
                                p=self.model.packet(self.seq,int((now-self.start_time)*1000))
                                try: data=encode(p,self.uart)
                                except ValueError as e:
                                    self.count['tx_failed']+=1; self.error=str(e)
                                else:
                                    self.link.send(data)
                                    self.seq=(self.seq+1)&MASK
                                    self.last_tx=decode(data,self.uart); self.tx_raw=data.decode().strip()
                                    self.history[p['seq']]=self.last_tx
                                    if len(self.history)>512: self.history.popitem(last=False)
                                    self.tx_times.append(now); self.count['tx_sent']+=1; self.count['tx_bytes']=len(data)
                                    if self.writer:
                                        r=self.last_rx or {}
                                        self.writer.writerow([p['host_ms'],p['sim_ms'],r.get('source_seq'),p['az_deg'],p['el_deg'],r.get('cmd_az_deg'),r.get('cmd_el_deg'),r.get('act_az_deg'),r.get('act_el_deg'),r.get('act_az_valid',False),r.get('act_el_valid',False),(now-self.rx_at)*1000 if self.rx_at else None])
                                        self.log.flush()
                            for _ in range(8):
                                data=self.link.receive()
                                if not data: break
                                self._receive(data,now)
                        except Exception as e:
                            self.error=str(e); self.count['io_failed']+=1; self.close_link()
                    else: tx_at=now
                self.quit.wait(.001)
        finally:
            self.close_link()
            if self.log: self.log.close()
