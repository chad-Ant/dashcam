import pcbnew as p
from pathlib import Path
r=Path(__file__).resolve().parent.parent;b=p.LoadBoard(str(r/'dashboard.kicad_pcb'))
def v(x,y):return p.VECTOR2I(p.FromMM(x),p.FromMM(y))
def tr(n,pts,layer=p.F_Cu):
 for a,c in zip(pts,pts[1:]):
  t=p.PCB_TRACK(b);t.SetStart(v(*a));t.SetEnd(v(*c));t.SetLayer(layer);t.SetNet(b.FindNet(n));t.SetWidth(p.FromMM(.3));t.SetLocked(True);b.Add(t)
def via(n,x,y):
 q=p.PCB_VIA(b);q.SetPosition(v(x,y));q.SetWidth(p.FromMM(2));q.SetDrill(p.FromMM(1));q.SetViaType(p.VIATYPE_THROUGH);q.SetLayerPair(p.F_Cu,p.B_Cu);q.SetNet(b.FindNet(n));q.SetLocked(True);b.Add(q)
for n,pts in [('Net-(U1-QD)',[(57.525,60.095),(53.5,60.095)]),('Net-(U1-QH)',[(57.525,65.175),(53.5,65.175)]),("Net-(U1-QH')",[(62.475,66.445),(66,66.445)]),('Net-(U1-SER)',[(62.475,60.095),(64.595,60.095),(66.5,62)]),('Net-(U1-QA)',[(62.475,58.825),(64.2,58.825),(64.2,53)])]:
 tr(n,pts);via(n,*pts[-1])
for n,x in [('Net-(U1-SRCLK)',84.82),('Net-(U1-SER)',89.9)]:
 tr(n,[(x,101.5),(x,98)],p.B_Cu);via(n,x,98)
for f in b.GetFootprints():
 if f.GetReference()=='C4':f.Reference().SetPosition(v(83.5,90))
 if f.GetReference()=='C3':f.Reference().SetPosition(v(55.9,113.2))
b.BuildConnectivity();p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(r/'dashboard.kicad_pcb'),b)
