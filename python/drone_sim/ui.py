import math
import sys
from collections import deque
from PySide6 import QtCore, QtGui, QtWidgets as W
from .simulation import Simulation, geometry, wrap180
from .transport import Engine

class MapView(W.QGraphicsView):
    def __init__(self):
        super().__init__()
        self.setScene(W.QGraphicsScene(self)); self.setSceneRect(-2000,-2000,4000,4000)
        self.setDragMode(W.QGraphicsView.DragMode.ScrollHandDrag)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        self.setBackgroundBrush(QtGui.QColor('#101c2c')); self.scale(1.2,1.2)
        self.trail=deque(maxlen=3000)
    def wheelEvent(self,event):
        f=1.2 if event.angleDelta().y()>0 else 1/1.2
        s=self.transform().m11()*f
        if .05<s<20: self.scale(f,f)
    def draw_state(self,m,p,r,age):
        s=self.scene(); s.clear()
        for i in range(-2000,2001,100):
            pen=QtGui.QPen(QtGui.QColor('#2b3d50' if i else '#63768c'),0)
            s.addLine(i,-2000,i,2000,pen); s.addLine(-2000,i,2000,i,pen)
        if not self.trail or self.trail[-1]!=(m.x,-m.y): self.trail.append((m.x,-m.y))
        path=QtGui.QPainterPath()
        for i,(x,y) in enumerate(self.trail):
            if i==0: path.moveTo(x,y)
            else: path.lineTo(x,y)
        s.addPath(path,QtGui.QPen(QtGui.QColor('#547e99'),0))
        if m.flight=='circle':
            cx,cy,rad,*_=m.circle
            s.addEllipse(cx-rad,-cy-rad,2*rad,2*rad,QtGui.QPen(QtGui.QColor('#475c78'),0,QtCore.Qt.PenStyle.DashLine))
        if m.flight=='line':
            s.addLine(m.x,-m.y,m.end[0],-m.end[1],QtGui.QPen(QtGui.QColor('#475c78'),0,QtCore.Qt.PenStyle.DashLine))
        s.addLine(0,0,p['x'],-p['y'],QtGui.QPen(QtGui.QColor('#ffd369'),0))
        s.addEllipse(-5,-5,10,10,QtGui.QPen(QtGui.QColor('white')),QtGui.QBrush(QtGui.QColor('#ffffff')))
        s.addEllipse(m.x-5,-m.y-5,10,10,QtGui.QPen(QtGui.QColor('#5adcc1')),QtGui.QBrush(QtGui.QColor('#5adcc1')))
        if r and age is not None and age<=.25:
            for key,color in [('cmd_az_deg','#66b3ff'),('act_az_deg','#e29bfa')]:
                a=r.get(key)
                if a is not None:
                    a=math.radians(a); s.addLine(0,0,140*math.sin(a),-140*math.cos(a),QtGui.QPen(QtGui.QColor(color),2))
        label=s.addText('N ↑   E →   сетка 100 м'); label.setDefaultTextColor(QtGui.QColor('#c5d3e5')); label.setPos(10,-30)

class Window(W.QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('Drone Simulator · Arduino Mega 2560'); self.resize(1320,860)
        self.model=Simulation(); self.engine=Engine(self.model); self.engine.start()
        self.fields={}; self.keys=set(); self.low_since=None
        root=W.QWidget(); self.setCentralWidget(root); layout=W.QVBoxLayout(root)
        top=W.QHBoxLayout(); layout.addLayout(top)
        self.kind=W.QComboBox(); self.kind.addItems(['UART','UDP (тестовый mock)']); top.addWidget(self.kind)
        self.port=W.QComboBox(); self.port.setEditable(True); top.addWidget(self.port)
        self.button(top,'Обновить порты',self.scan)
        self.host=W.QLineEdit('127.0.0.1'); self.host.setMaximumWidth(120); top.addWidget(self.host)
        self.remote=W.QSpinBox(); self.remote.setRange(1,65535); self.remote.setValue(5005); top.addWidget(self.remote)
        self.local=W.QSpinBox(); self.local.setRange(1,65535); self.local.setValue(5006); top.addWidget(self.local)
        self.rate=W.QSpinBox(); self.rate.setRange(20,25); self.rate.setValue(25); self.rate.setSuffix(' Гц'); top.addWidget(self.rate)
        self.button(top,'Подключить',self.connect_link); self.button(top,'Отключить',lambda:self.engine.command('disconnect'))
        self.feedback=W.QCheckBox('Двусторонний обмен'); self.feedback.setChecked(True); top.addWidget(self.feedback)
        self.feedback.toggled.connect(lambda v:self.engine.command('feedback',enabled=v))
        split=W.QSplitter(); layout.addWidget(split,1)
        self.map=MapView(); split.addWidget(self.map)
        tabs=W.QTabWidget(); split.addWidget(tabs); split.setSizes([770,510])
        sim=W.QWidget(); f=W.QFormLayout(sim); scroll=W.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(sim); tabs.addTab(scroll,'Дрон')
        self.flight=W.QComboBox(); self.flight.addItems(['Ручной','Прямая','Окружность']); f.addRow('Траектория',self.flight)
        for name,label,value,lo,hi in [('x','X, м',0,-100000,100000),('y','Y, м',100,-100000,100000),('z','Z, м',50,0,100000),('speed','Скорость, м/с',10,0,1000),('heading','Курс, °',0,0,359.999),('vz','Вертикальная скорость',0,-1000,1000),('ex','Конечная X',100,-100000,100000),('ey','Конечная Y',100,-100000,100000),('ez','Конечная Z',50,0,100000),('cx','Центр X',0,-100000,100000),('cy','Центр Y',0,-100000,100000),('radius','Радиус, м',100,.1,100000),('cz','Высота окружности',50,0,100000),('phi','Начальный угол, °',0,-360,360)]: self.field(f,name,label,value,lo,hi)
        self.clockwise=W.QCheckBox('По часовой стрелке'); f.addRow(self.clockwise)
        self.button(f,'Установить координаты',self.teleport)
        self.button(f,'Применить траекторию / скорость',self.configure)
        row=W.QHBoxLayout()
        for label,state in [('Start','RUN'),('Pause','PAUSE'),('Resume','RUN'),('Stop','STOP')]: self.button(row,label,lambda _,s=state:self.set_state(s))
        f.addRow(row); self.button(f,'Reset',self.reset)
        self.scale=W.QComboBox(); self.scale.addItems(['0.25','0.5','1','2','5']); self.scale.setCurrentText('1')
        self.scale.currentTextChanged.connect(self.change_scale); f.addRow('Скорость времени ×',self.scale)
        self.keyboard=W.QCheckBox('Клавиатура W/A/S/D, Q↑ / E↓ (в фокусе карта)'); f.addRow(self.keyboard)
        self.map.installEventFilter(self); self.map.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        ant=W.QWidget(); af=W.QFormLayout(ant); tabs.addTab(ant,'Антенна')
        info=W.QLabel('1. Настройте config.h и проверьте механику.\n2. Установите исходное положение вручную.\n3. Отправьте калибровку и дождитесь ACK.\nБез датчиков реальные углы неизвестны.'); info.setWordWrap(True); af.addRow(info)
        self.field(af,'cal','Исходный механический AZ, °',0,-720,720)
        self.confirm=W.QCheckBox('Исходное положение физически проверено'); af.addRow(self.confirm)
        self.button(af,'Калибровать / разрешить приводы',self.calibrate)
        self.button(af,'Блокировать приводы (DISARM)',lambda:self.engine.command('control',action='DISARM'))
        self.field(af,'az','Целевой AZ, °',0,0,359.999); self.field(af,'el','Целевой EL, °',0,-90,90)
        self.button(af,'Навести вручную',self.manual)
        jog=W.QHBoxLayout()
        for label,axis,step in [('AZ −5°','az',-5),('AZ +5°','az',5),('EL −5°','el',-5),('EL +5°','el',5)]: self.button(jog,label,lambda _,a=axis,d=step:self.jog(a,d))
        af.addRow(jog); self.button(af,'Вернуть сопровождение дрона',self.track)
        self.ant_status=W.QLabel(); self.ant_status.setWordWrap(True); af.addRow(self.ant_status)
        af.addRow(W.QLabel('DISARM — программная блокировка.\nАппаратный выключатель питания нужен отдельно.'))
        diag=W.QWidget(); df=W.QVBoxLayout(diag); tabs.addTab(diag,'Диагностика')
        self.tx=W.QPlainTextEdit(); self.rx=W.QPlainTextEdit()
        for label,view in [('Последний TX',self.tx),('Последний RX',self.rx)]: df.addWidget(W.QLabel(label)); view.setReadOnly(True); df.addWidget(view)
        self.button(df,'Начать CSV…',self.start_log); self.button(df,'Закончить CSV',lambda:self.engine.command('log',path=None))
        self.stats=W.QLabel(); self.stats.setWordWrap(True); df.addWidget(self.stats)
        self.telemetry=W.QLabel(); self.telemetry.setWordWrap(True); self.telemetry.setMinimumHeight(42); layout.addWidget(self.telemetry)
        self.angles=W.QLabel(); self.angles.setWordWrap(True); self.angles.setMinimumHeight(60); layout.addWidget(self.angles)
        self.status=W.QLabel(); self.status.setWordWrap(True); self.status.setMinimumHeight(40); layout.addWidget(self.status)
        self.scan(); self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(40)
    def field(self,form,name,label,value,low,high):
        box=W.QDoubleSpinBox(); box.setDecimals(3); box.setRange(low,high); box.setValue(value); form.addRow(label,box); self.fields[name]=box
    def val(self,key): return self.fields[key].value()
    def button(self,layout,label,fn):
        b=W.QPushButton(label); b.clicked.connect(fn)
        if isinstance(layout,W.QFormLayout): layout.addRow(b)
        else: layout.addWidget(b)
        return b
    def scan(self):
        from serial.tools.list_ports import comports
        old=self.port.currentText(); self.port.clear(); self.port.addItems([p.device for p in comports()]); self.port.setEditText(old)
    def connect_link(self):
        self.engine.command('connect',kind='UART' if self.kind.currentIndex()==0 else 'UDP',port=self.port.currentText(),host=self.host.text(),remote=self.remote.value(),local=self.local.value(),hz=self.rate.value())
    def teleport(self):
        with self.model.lock: self.model.x,self.model.y,self.model.z=(self.val(k) for k in ('x','y','z'))
    def configure(self):
        flight=['manual','line','circle'][self.flight.currentIndex()]
        a=math.radians(self.val('heading')); v=self.val('speed')
        self.model.configure(flight,velocity=v,end=tuple(self.val(k) for k in ('ex','ey','ez')),
            circle=tuple(self.val(k) for k in ('cx','cy','radius','cz','phi'))+(-1 if self.clockwise.isChecked() else 1,),
            vx=v*math.sin(a),vy=v*math.cos(a),vz=self.val('vz'))
    def set_state(self,state):
        with self.model.lock: self.model.state=state
    def change_scale(self,s):
        with self.model.lock: self.model.scale=float(s)
    def reset(self): self.model.reset(); self.map.trail.clear(); self.scale.setCurrentText('1'); self.flight.setCurrentIndex(0)
    def calibrate(self):
        if not self.confirm.isChecked():
            W.QMessageBox.information(self,'Калибровка','Сначала физически установите и подтвердите исходный угол.'); return
        self.engine.command('control',action='CALIBRATE',az_mech_deg=self.val('cal')); self.confirm.setChecked(False)
    def manual(self):
        with self.model.lock:
            self.model.manual_az,self.model.manual_el=self.val('az'),self.val('el'); self.model.manual_antenna=True
    def jog(self,axis,delta):
        v=self.val(axis)+delta
        self.fields[axis].setValue(v%360 if axis=='az' else v); self.manual()
    def track(self):
        with self.model.lock: self.model.manual_antenna=False
    def start_log(self):
        path,_=W.QFileDialog.getSaveFileName(self,'Сохранить журнал','telemetry.csv','CSV (*.csv)')
        if path: self.engine.command('log',path=path)
    def eventFilter(self,obj,event):
        keys={QtCore.Qt.Key.Key_W,QtCore.Qt.Key.Key_A,QtCore.Qt.Key.Key_S,QtCore.Qt.Key.Key_D,QtCore.Qt.Key.Key_Q,QtCore.Qt.Key.Key_E}
        if event.type()==QtCore.QEvent.Type.FocusOut:
            self.keys.clear(); self.keyboard_velocity()
        if event.type() in (QtCore.QEvent.Type.KeyPress,QtCore.QEvent.Type.KeyRelease) and event.key() in keys and self.keyboard.isChecked():
            if not event.isAutoRepeat():
                if event.type()==QtCore.QEvent.Type.KeyPress: self.keys.add(event.key())
                else: self.keys.discard(event.key())
                self.keyboard_velocity()
            return True
        return super().eventFilter(obj,event)
    def keyboard_velocity(self):
        if self.model.flight!='manual': return
        k=QtCore.Qt.Key; v=self.val('speed')
        x=int(k.Key_D in self.keys)-int(k.Key_A in self.keys); y=int(k.Key_W in self.keys)-int(k.Key_S in self.keys); z=int(k.Key_Q in self.keys)-int(k.Key_E in self.keys)
        n=math.sqrt(x*x+y*y+z*z) or 1
        with self.model.lock: self.model.vx,self.model.vy,self.model.vz=v*x/n,v*y/n,v*z/n
    def refresh(self):
        snap=self.engine.snapshot(); r=snap['rx']; age=snap['rx_age']
        with self.model.lock:
            m=self.model; p=m.packet(0,0); az,el,h,d=geometry(m.x,m.y,m.z)
            self.map.draw_state(m,p,r,age)
            speed=math.sqrt(m.vx*m.vx+m.vy*m.vy+m.vz*m.vz) if m.state=='RUN' else 0
            vh=math.hypot(m.vx,m.vy) if m.state=='RUN' else 0
            heading=math.degrees(math.atan2(m.vx,m.vy))%360 if vh else None
            self.telemetry.setText(f'{m.state} · t={m.t:.2f} с · XYZ=({m.x:.2f}, {m.y:.2f}, {m.z:.2f}) м · V={speed:.2f}, Vh={vh:.2f} м/с · vXYZ=({m.vx if m.state=='RUN' else 0:.2f}, {m.vy if m.state=='RUN' else 0:.2f}, {m.vz if m.state=='RUN' else 0:.2f}) · курс={fmt(heading)}\nДальности: горизонтальная {h:.2f} м, наклонная {d:.2f} м · AZ/EL на дрон: {fmt(az)} / {fmt(el)}')
            mode='РУЧНОЕ НАВЕДЕНИЕ' if m.manual_antenna else 'СОПРОВОЖДЕНИЕ ДРОНА'
        fresh=r is not None and age is not None and age<=.25
        state=(r['status'] if fresh else 'RX STALE') if snap['two_way'] else 'Приём отключён'
        self.angles.setText(f'{mode} · {state} · Target AZ/EL: {fmt(p["az_deg"])} / {fmt(p["el_deg"])}\n'+self.angle_text(r,snap['matched'],p,fresh))
        self.tx.setPlainText(snap['tx_raw']); self.rx.setPlainText(snap['rx_raw'])
        self.stats.setText(str(snap['count'])+f'\nRX age: {age:.3f} с' if age is not None else str(snap['count']))
        self.ant_status.setText('Ожидание ACK…' if snap['pending'] else f'Последний ACK: {snap["ack"] or "нет"}')
        import time
        now=time.monotonic()
        if snap['connected'] and snap['tx_hz']<20:
            if self.low_since is None: self.low_since=now
        else: self.low_since=None
        warning=' · TX <20 Гц более 5 с' if self.low_since and now-self.low_since>5 else ''
        self.status.setText(f'{"Подключено" if snap["connected"] else "Отключено"} · 115200 8N1 · TX {snap["tx_hz"]:.1f} / RX {snap["rx_hz"]:.1f} Гц (окно 5 с){warning}\n{snap["error"]}')
    def angle_text(self,r,matched,p,fresh):
        if not r: return 'Commanded: — / — · Actual: нет данных'
        cmd=f'Commanded: {fmt(r["cmd_az_deg"])} / {fmt(r["cmd_el_deg"])}'
        actual=' / '.join(fmt(r['act_'+a+'_deg']) if r['act_'+a+'_valid'] else 'нет датчика' for a in ('az','el'))
        if not fresh: return cmd+' · Actual: '+actual+' · УСТАРЕЛО (ошибки не вычисляются)'
        def error(target,actual,az):
            if target is None or actual is None: return '—'
            return fmt(wrap180(target-actual) if az else target-actual)
        ec=' / '.join(error((matched or {}).get(a+'_deg'),r['cmd_'+a+'_deg'],a=='az') for a in ('az','el'))
        ea=' / '.join(error(p[a+'_deg'],r['act_'+a+'_deg'],a=='az') for a in ('az','el'))
        return cmd+' · Actual: '+actual+f'\nОшибка команды (source_seq={r["source_seq"]}): {ec} · Текущая ошибка сопровождения: {ea} · seq={r["seq"]} · {r.get("reason", "")}'
    def closeEvent(self,event):
        self.timer.stop(); self.engine.quit.set(); self.engine.join(2); event.accept()

def fmt(v): return '—' if v is None else f'{v:.3f}°'

def main():
    app=W.QApplication(sys.argv); app.setStyle('Fusion'); w=Window(); w.show(); sys.exit(app.exec())
