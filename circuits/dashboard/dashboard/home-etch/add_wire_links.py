from pathlib import Path
import pcbnew as p,json,math
r=Path(__file__).resolve().parent.parent;b=p.LoadBoard(str(r/'dashboard.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
def v(x,y):return p.VECTOR2I(p.FromMM(x),p.FromMM(y))
links=[('U2','4','R5','1'),('U2','12','U1','12'),('U2','14','U1','9'),('U2','15','R8','1'),('J2','2','J3','3')]
rows=[]
for idx,(ar,ap,br,bp) in enumerate(links,1):
 a=next(q for q in fps[ar].Pads() if q.GetNumber()==ap);c=next(q for q in fps[br].Pads() if q.GetNumber()==bp)
 assert a.GetNetname()==c.GetNetname()
 f=p.FOOTPRINT(b);b.Add(f);f.SetReference('WL'+str(idx));f.SetValue('INSULATED WIRE - FIT');f.SetAttributes(p.FP_BOARD_ONLY|p.FP_EXCLUDE_FROM_POS_FILES);f.SetDuplicatePadNumbersAreJumpers(True)
 f.Reference().SetVisible(False);f.Value().SetVisible(False)
 pos=a.GetPosition();f.SetPosition(pos)
 endpoints=[]
 for original in [a,c]:
  q=p.PAD(f);q.SetNumber('1');q.SetAttribute(p.PAD_ATTRIB_SMD);q.SetShape(p.PAD_SHAPE_RECT);q.SetNet(original.GetNet());ls=p.LSET();ls.AddLayer(p.F_Cu if idx<5 else p.B_Cu);q.SetLayerSet(ls)
  q.SetSize(v(.3,.3));qp=original.GetPosition()
  if original.GetAttribute()==p.PAD_ATTRIB_PTH:qp=p.VECTOR2I(qp.x+p.FromMM(.75),qp.y)
  q.SetPosition(qp);f.Add(q);endpoints.append([round(p.ToMM(qp.x)-50,4),round(p.ToMM(qp.y)-50,4)])
 # Drawing only: the actual electrical connection is the fitted insulated wire.
 g=p.PCB_SHAPE(f);g.SetShape(p.SHAPE_T_SEGMENT);g.SetStart(v(endpoints[0][0]+50,endpoints[0][1]+50));g.SetEnd(v(endpoints[1][0]+50,endpoints[1][1]+50));g.SetWidth(p.FromMM(.2));g.SetLayer(p.User_1);f.Add(g)
 name='Wire_Link_'+str(idx);f.SetFPID(p.LIB_ID('dashboard',name));f.SetLibDescription(f'Fit insulated wire from {ar} pin {ap} to {br} pin {bp}; overlapped landing pads mark existing solder joints, not new holes.')
 clone=p.FOOTPRINT(f);clone.SetPosition(v(0,0));clone.SetReference('REF**');p.FootprintSave(str(r/'dashboard.pretty'),clone)
 rows.append({'reference':f.GetReference(),'from':ar+'.'+ap,'to':br+'.'+bp,'net':a.GetNetname(),'face':'Front' if idx<5 else 'Rear','endpoints_mm_from_front_top_left':endpoints,'straight_distance_mm':round(math.dist(*endpoints),1)})
b.BuildConnectivity();p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(r/'dashboard.kicad_pcb'),b)
(r/'home-etch/wire-links.json').write_text(json.dumps(rows,indent=2))
print('Five fitted wire links modelled using KiCad jumper connectivity.')


