"""Strict v2 wire contract and optional ANTENNA_CONTROL/ACK v1."""
import json
import math

MAX_PACKET = 384
MASK = 0xffffffff

def newer(a, b):
    return b is None or 0 < ((a-b) & MASK) < 0x80000000

def uint(v):
    return type(v) is int and 0 <= v <= MASK

def number(v):
    return type(v) in (int, float) and math.isfinite(v)

def validate(p):
    def require(ok, message):
        if not ok:
            raise ValueError(message)
    require(type(p) is dict, 'Expected object')
    def finite_tree(value):
        if type(value) is float: return math.isfinite(value)
        if isinstance(value, dict): return all(finite_tree(v) for v in value.values())
        if isinstance(value, list): return all(finite_tree(v) for v in value)
        return True
    require(finite_tree(p), 'Non-finite number')
    name = p.get('protocol')
    require(uint(p.get('seq')), 'seq must be uint32')
    if name == 'DRONE_SIM':
        required = 'version sim_ms host_ms mode x y z vx vy vz speed az_deg el_deg az_valid el_valid'.split()
        require(all(k in p for k in required), 'Missing DRONE_SIM field')
        require(type(p['version']) is int and p['version'] == 2, 'Version must be 2')
        require(uint(p['sim_ms']) and uint(p['host_ms']), 'Invalid time')
        require(p['mode'] in ('RUN', 'PAUSE', 'STOP'), 'Invalid mode')
        require(all(number(p[k]) for k in ('x','y','z','vx','vy','vz','speed')), 'Invalid number')
        require(p['speed'] >= 0, 'Negative speed')
        pairs = [('az_deg','az_valid',0,360), ('el_deg','el_valid',-90,90)]
    elif name == 'ANTENNA_STATE':
        required = 'version source_seq mcu_ms cmd_az_deg cmd_el_deg act_az_deg act_el_deg act_az_valid act_el_valid status'.split()
        require(all(k in p for k in required), 'Missing ANTENNA_STATE field')
        require(type(p['version']) is int and p['version'] == 2, 'Version must be 2')
        require(uint(p['mcu_ms']) and (p['source_seq'] is None or uint(p['source_seq'])), 'Invalid time/source_seq')
        require(p['status'] in ('TRACKING','NO_FEEDBACK','STALE','FAULT'), 'Invalid status')
        for k, lo, hi in [('cmd_az_deg',0,360),('cmd_el_deg',-90,90)]:
            v = p[k]
            require(v is None or (number(v) and lo <= v <= hi and (hi != 360 or v < hi)), k)
        pairs = [('act_az_deg','act_az_valid',0,360),('act_el_deg','act_el_valid',-90,90)]
    elif name == 'ANTENNA_CONTROL':
        require(type(p.get('version')) is int and p['version'] == 1, 'Control version')
        require(p.get('action') in ('CALIBRATE','DISARM'), 'Invalid action')
        if p['action'] == 'CALIBRATE':
            require(number(p.get('az_mech_deg')), 'Invalid calibration')
        pairs = []
    elif name == 'ANTENNA_ACK':
        require(type(p.get('version')) is int and p['version'] == 1, 'ACK version')
        require(type(p.get('ok')) is bool and type(p.get('reason')) is str, 'Invalid ACK')
        pairs = []
    else:
        raise ValueError('Unknown protocol')
    for key, flag, lo, hi in pairs:
        require(type(p[flag]) is bool, flag)
        v = p[key]
        require((v is None and not p[flag]) or (p[flag] and number(v) and lo <= v <= hi and (hi != 360 or v < hi)), key)
    return p

def encode(p, uart=True):
    validate(p)
    q = {k: (round(v, 3) if type(v) is float else v) for k,v in p.items()}
    # 359.9999 rounds to 360; retain the canonical azimuth interval.
    for k in ('az_deg','cmd_az_deg','act_az_deg'):
        if q.get(k) == 360:
            q[k] = 0.0
    validate(q)
    data = json.dumps(q, separators=(',', ':'), allow_nan=False, ensure_ascii=True).encode('ascii') + (b'\n' if uart else b'')
    if len(data) > MAX_PACKET:
        raise ValueError('Packet exceeds 384 bytes')
    return data

def decode(data, uart=True):
    if len(data) > MAX_PACKET or (uart and not data.endswith(b'\n')):
        raise ValueError('Invalid framing/length')
    def pairs(items):
        d = {}
        for k,v in items:
            if k in d: raise ValueError('Duplicate key')
            d[k] = v
        return d
    try:
        p = json.loads(data.decode('ascii'), object_pairs_hook=pairs,
                       parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
    except (UnicodeError, json.JSONDecodeError) as e:
        raise ValueError(str(e)) from e
    return validate(p)

class Framer:
    def __init__(self):
        self.buffer = bytearray()
        self.dropping = False
        self.overflow = 0
    def feed(self, data):
        out = []
        for b in data:
            if self.dropping:
                if b == 10: self.dropping = False
                continue
            self.buffer.append(b)
            if len(self.buffer) > MAX_PACKET or (len(self.buffer) == MAX_PACKET and b != 10):
                self.buffer.clear(); self.dropping = b != 10; self.overflow += 1
            elif b == 10:
                out.append(bytes(self.buffer)); self.buffer.clear()
        return out
