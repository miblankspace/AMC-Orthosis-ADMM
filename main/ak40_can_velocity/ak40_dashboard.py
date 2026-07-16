import sys,time,csv,re
from collections import deque
import serial
from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
PORT="COM15"; BAUD=115200; MAXPTS=500
class Dash(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle("AK40 Dashboard"); self.resize(1200,900); self.ser=None; self.start=time.time()
        self.d={k:deque(maxlen=MAXPTS) for k in["t","target","cmd","meas","pos","cur","torque","temp"]}
        w=QWidget(); self.setCentralWidget(w); v=QVBoxLayout(w)
        h=QHBoxLayout(); self.status=QLabel("Disconnected"); self.box=QLineEdit(); self.box.setPlaceholderText("v 500"); b1=QPushButton("Connect"); b2=QPushButton("Send"); h.addWidget(self.status); h.addWidget(self.box); h.addWidget(b2); h.addWidget(b1); v.addLayout(h)
        self.lab={}; g=QGridLayout()
        for i,k in enumerate(["state","target","current","measured","position","currentA","torque","temp","err"]):
            g.addWidget(QLabel(k),i,0); self.lab[k]=QLabel("-"); g.addWidget(self.lab[k],i,1)
        v.addLayout(g)
        self.p1=pg.PlotWidget(title="Speed"); self.p2=pg.PlotWidget(title="Position"); self.p3=pg.PlotWidget(title="Current/Torque"); self.p4=pg.PlotWidget(title="Temperature")
        v.addWidget(self.p1);v.addWidget(self.p2);v.addWidget(self.p3);v.addWidget(self.p4)
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); v.addWidget(self.log)
        self.l1=self.p1.plot(pen='r'); self.l2=self.p1.plot(pen='g'); self.l3=self.p1.plot(pen='y'); self.l4=self.p2.plot(); self.l5=self.p3.plot(); self.l6=self.p3.plot(pen='y'); self.l7=self.p4.plot()
        self.csv=open("telemetry.csv","w",newline=""); self.wr=csv.writer(self.csv); self.wr.writerow(["t","state","target","cmd","measured","pos","temp","current","torque","err"])
        b1.clicked.connect(self.connect); b2.clicked.connect(self.send); self.box.returnPressed.connect(self.send)
        self.tm=QTimer(); self.tm.timeout.connect(self.tick); self.tm.start(20)
    def connect(self):
        self.ser=serial.Serial(PORT,BAUD,timeout=0.01); self.status.setText("Connected "+PORT)
    def send(self):
        if self.ser: self.ser.write((self.box.text().strip()+"\n").encode()); self.box.clear()
    def tick(self):
        if self.ser:
            while self.ser.in_waiting:
                s=self.ser.readline().decode(errors="ignore").strip()
                if not s: break
                self.log.appendPlainText(s)
                d=dict(re.findall(r'(\w+)=([-\w\.]+)',s)); 
                if not d: continue
                t=time.time()-self.start
                vals={"target":float(d.get("target_erpm",0)),"cmd":float(d.get("current_erpm",0)),"meas":float(d.get("measured_erpm",0)),"pos":float(d.get("pos",0)),"temp":float(d.get("temp",0)),"cur":float(d.get("current",0)),"torque":float(d.get("torque_est",d.get("torque",0)))}
                self.d["t"].append(t)
                for k,v in vals.items(): self.d[k].append(v)
                self.wr.writerow([t,d.get("state",""),vals["target"],vals["cmd"],vals["meas"],vals["pos"],vals["temp"],vals["cur"],vals["torque"],d.get("err","0")])
                self.lab["state"].setText(d.get("state","")); self.lab["target"].setText(str(vals["target"])); self.lab["current"].setText(str(vals["cmd"])); self.lab["measured"].setText(str(vals["meas"])); self.lab["position"].setText(str(vals["pos"])); self.lab["currentA"].setText(str(vals["cur"])); self.lab["torque"].setText(str(vals["torque"])); self.lab["temp"].setText(str(vals["temp"])); self.lab["err"].setText(d.get("err","0"))
        if self.d["t"]:
            x=list(self.d["t"]); self.l1.setData(x,list(self.d["target"])); self.l2.setData(x,list(self.d["cmd"])); self.l3.setData(x,list(self.d["meas"])); self.l4.setData(x,list(self.d["pos"])); self.l5.setData(x,list(self.d["cur"])); self.l6.setData(x,list(self.d["torque"])); self.l7.setData(x,list(self.d["temp"]))
app=QApplication(sys.argv); pg.setConfigOptions(antialias=True); d=Dash(); d.show(); sys.exit(app.exec())