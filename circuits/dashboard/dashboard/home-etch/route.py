from pathlib import Path
import pcbnew as p
ROOT=Path(__file__).resolve().parent.parent;OUT=ROOT/'home-etch'
def mm(x):return p.FromMM(x)
def v(x,y):return p.VECTOR2I(mm(x),mm(y))
b=p.LoadBoard(str(ROOT/'dashboard.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
def track(net,pts,w=.3,layer=p.B_Cu):
 for a,c in zip(pts,pts[1:]):
  t=p.PCB_TRACK(b);t.SetStart(v(*a));t.SetEnd(v(*c));t.SetWidth(mm(w));t.SetLayer(layer);t.SetNet(b.FindNet(net));t.SetLocked(True);b.Add(t)
def via(net,x,y):
 q=p.PCB_VIA(b);q.SetPosition(v(x,y));q.SetWidth(mm(2));q.SetDrill(mm(1));q.SetViaType(p.VIATYPE_THROUGH);q.SetLayerPair(p.F_Cu,p.B_Cu);q.SetNet(b.FindNet(net));q.SetLocked(True);b.Add(q)
track('Net-(D7-K)',[(65.3,103.35),(65.3,97.85)],1)
track('Net-(D7-K)',[(65.3,99.6),(62.3,99.6),(59.7,102.2),(59.7,103.35)],1)
track('5V',[(63.6,103.35),(60.7,106.25),(60.7,112.5),(56.3,116.9),(55.9,116.9)],.8)
track('GND',[(67,103.35),(67,112.5)],.8)
track('GND',[(70.4,103.35),(73,105.95),(73,112.5),(67,112.5)],.6)
track('3.3V',[(65.3,88.15),(68,90.85),(75.3,90.85),(77.5,93.05)],1)
track('3.3V',[(77.5,93.05),(75.5,95.05),(75.5,98),(70.7,98),(68.7,100),(68.7,103.35)],.3)
# Capacitor grounds and thermal tab connect into the rear ground pour.
track('GND',[(52.9,103.35),(52.9,106.5),(55.9,109.5)],1)
for x in []:
 track('GND',[(x,114),(x,118)],.8);via('GND',x,118)
# Supply decoupling next to the wider-pitch SOIC devices.
def pp(ref,num):
 q=next(q for q in fps[ref].Pads() if q.GetNumber()==str(num));return p.ToMM(q.GetPosition().x),p.ToMM(q.GetPosition().y)
for u,c in [('U1','C1'),('U2','C2')]:
 a=pp(u,16);d=pp(c,1);track('5V',[a,(d[0],a[1]),d],.4,p.F_Cu)
# References kept readable on assembly prints; no silkscreen process assumed.
pos={'U1':(60,55.5),'U2':(91,56.3),'BZ1':(76,62),'D7':(56.3,99.1),'C3':(55.9,118.7),'C4':(82.4,90),'L1':(65.3,93),'U4':(75.5,110),'J1':(67,71.5),'J2':(94,69),'J3':(93,95),'U3':(89,105),'D6':(79.7,52.5),'C1':(54.5,52.5),'C2':(86.6,52.5),'R17':(65.5,68.2),'R16':(94,69.8)}
for ref,f in fps.items():
 if ref.startswith('R') and int(ref[1:])<=15:pos[ref]=(p.ToMM(f.GetPosition().x),90.6)
 if ref in ['D1','D2','D3','D4','D5']:pos[ref]=(p.ToMM(f.GetPosition().x)-3.4,74)
 if ref in pos:f.Reference().SetPosition(v(*pos[ref]))
b.BuildConnectivity();p.SaveBoard(str(ROOT/'dashboard.kicad_pcb'),b)
assert p.ExportSpecctraDSN(b,str(OUT/'dashboard.dsn'))
# Large hand-wired vias carry a high routing cost; avoid gratuitous fanout.
s=(OUT/'dashboard.dsn').read_text();s=s.replace('(via_costs 50)','(via_costs 200)');(OUT/'dashboard.dsn').write_text(s)
print('Home-etch power paths saved.')
