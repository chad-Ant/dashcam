from pathlib import Path
import pcbnew as p
ROOT=Path(__file__).resolve().parent.parent; OUT=ROOT/'revision-b'
def mm(x):return p.FromMM(x)
def v(x,y):return p.VECTOR2I(mm(x),mm(y))
b=p.LoadBoard(str(ROOT/'dashboard.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
def track(net,pts,w=.2,layer=p.B_Cu):
 for a,c in zip(pts,pts[1:]):
  t=p.PCB_TRACK(b);t.SetStart(v(*a));t.SetEnd(v(*c));t.SetWidth(mm(w));t.SetLayer(layer);t.SetNet(b.FindNet(net));t.SetLocked(True);b.Add(t)
def via(net,x,y):
 q=p.PCB_VIA(b);q.SetPosition(v(x,y));q.SetWidth(mm(.6));q.SetDrill(mm(.3));q.SetViaType(p.VIATYPE_THROUGH);q.SetLayerPair(p.F_Cu,p.B_Cu);q.SetNet(b.FindNet(net));q.SetLocked(True);b.Add(q)
def T(pt):return 136-pt[1],pt[0]+33
def oldtrack(net,pts,w):track(net,[T(pt) for pt in pts],w)
oldtrack('Net-(D7-K)',[(70.35,70.7),(64.85,70.7)],1)
oldtrack('Net-(D7-K)',[(66.6,70.7),(66.6,73.7),(69.2,76.3),(70.35,76.3)],1)
oldtrack('5V',[(70.35,72.4),(73.6,72.4),(73.6,76.1),(77,79.5)],.8)
oldtrack('GND',[(70.35,83.1),(76.4,83.1),(80,79.5),(80.5,79.5)],1.2)
oldtrack('GND',[(80.5,79.5),(80.5,75),(79.5,74),(79.5,69)],1.2)
oldtrack('GND',[(70.35,69),(79.5,69)],.8)
oldtrack('GND',[(70.35,65.6),(73.5,65.6),(75.2,63.9),(79.5,63.9)],.6)
for y in [65,69,73]:
 oldtrack('GND',[(83.5,y),(85,y)],.8);via('GND',*T((85,y)))
for x in [69,71.7]:
 oldtrack('GND',[(70.35,83.1),(x,83.1)],.8);via('GND',*T((x,83.1)))
oldtrack('3.3V',[(55.15,70.7),(53,68.55),(53,61.5),(54,60.5)],1)
oldtrack('3.3V',[(54,60.5),(57.3,63.8),(63.5,63.8),(67,67.3),(70.35,67.3)],.3)
oldtrack('GND',[(56.5,60.5),(56.5,59)],.6);via('GND',*T((56.5,59)))
# Accessory supply runs on the rear, outside the logic pin banks.
track('3.3V',[(75.5,87),(80,82.5),(82.8,79.7),(82.8,56.5),(84.8,54.5),(87.388,54.5),(87.388,56.5)],.8)
# Wide input feed on the front, with branches served by the remaining routing.
track('5V',[(56.5,110),(60,106.5),(60,91.5),(92,91.5)],.8,p.F_Cu)
via('5V',92,91.5)
track('5V',[(92,91.5),(98,85.5),(98,73.8),(100.3,71.5)],.8)
for x,y,cx,cy in [(63.4125,60.725,65.8,61.275),(94.4125,58.225,97.2,59.275)]:
 track('5V',[(x,y),(cx-(cy-y),y),(cx,cy)],.3,p.F_Cu)
 track('GND',[(cx,cy-1.55),(cx,cy-2.55)],.4,p.F_Cu);via('GND',cx,cy-2.55)
# Put references clear of the fixed LED row and component bodies.
positions={'U1':(60,58.7),'U2':(91,56.2),'BZ1':(76,62),'D7':(56.3,99.1),'C3':(56.5,117.2),'C4':(79.8,89),'L1':(65.3,93),'U4':(75,110),'J1':(76,70.5),'J2':(94,69),'J3':(94,94),'U3':(89,105),'D6':(79.7,52.5),'C1':(65.8,57.5),'C2':(97.2,55.5),'R17':(65.5,67.5),'R16':(94,71.8)}
for ref,f in fps.items():
 if ref.startswith('R') and int(ref[1:])<=15:positions[ref]=(p.ToMM(f.GetPosition().x),87)
 if ref in ['D1','D2','D3','D4','D5']:positions[ref]=(p.ToMM(f.GetPosition().x)-3.4,75.5)
 if ref=='R8':positions[ref]=(75.5,85.8)
 if ref in positions:f.Reference().SetPosition(v(*positions[ref]))
# Terminal body overhang is on Fab; omit silk segments outside the board.
j=fps['J3']
for g in list(j.GraphicalItems()):
 if g.GetLayer()==p.B_SilkS and max(g.GetStart().x,g.GetEnd().x)>mm(101.8):g.SetLayer(p.B_Fab)
copy=p.FOOTPRINT(j);copy.Flip(copy.GetPosition(),False);copy.SetOrientationDegrees(0);copy.SetPosition(v(0,0));copy.SetReference('REF**');copy.SetFPID(p.LIB_ID('dashboard','TerminalBlock_Right_10pin_2p54'));p.FootprintSave(str(ROOT/'dashboard.pretty'),copy)
j.SetFPID(p.LIB_ID('dashboard','TerminalBlock_Right_10pin_2p54'))
sch=ROOT/'dashboard.kicad_sch';s=sch.read_text();s=s.replace('TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-10-2.54_1x10_P2.54mm_Horizontal','dashboard:TerminalBlock_Right_10pin_2p54');sch.write_text(s)
b.BuildConnectivity();p.SaveBoard(str(ROOT/'dashboard.kicad_pcb'),b)
assert p.ExportSpecctraDSN(b,str(OUT/'dashboard.dsn'))
print('Critical routes and labels ready; signal routing exported.')
