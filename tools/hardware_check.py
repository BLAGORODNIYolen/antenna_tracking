"""Read/write physical UART for 60s, WITHOUT arming motors.
Usage: python tools/hardware_check.py COM5 --seconds 60
Install python/requirements.txt first. Close GUI/Serial Monitor.
This script never sends CALIBRATE. Reset Mega first if an earlier host used this session.
"""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from drone_sim.simulation import Simulation
from drone_sim.transport import Engine

p=argparse.ArgumentParser(description=__doc__); p.add_argument('port'); p.add_argument('--seconds',type=float,default=60); args=p.parse_args()
model=Simulation(); engine=Engine(model); engine.start()
engine.command('connect',kind='UART',port=args.port,host='',remote=0,local=0,hz=25)
time.sleep(3)  # allow Mega bootloader/reset; not included in measured interval
start=engine.snapshot()['count']; begin=time.monotonic()
try:
    while time.monotonic()-begin<args.seconds:
        time.sleep(.2)
        if not engine.snapshot()['connected']: raise RuntimeError(engine.snapshot()['error'])
    end=engine.snapshot()
finally:
    engine.quit.set(); engine.join(2)
counts={k:end['count'].get(k,0)-start.get(k,0) for k in ('tx_sent','rx_valid','rx_invalid','rx_overflow','io_failed')}
result=dict(seconds=args.seconds,counts=counts,tx_hz=counts['tx_sent']/args.seconds,rx_hz=counts['rx_valid']/args.seconds,last_state=end['rx'])
print(json.dumps(result,ensure_ascii=False,indent=2))
if counts['tx_sent']<20*args.seconds or counts['rx_valid']<20*args.seconds: sys.exit(1)
