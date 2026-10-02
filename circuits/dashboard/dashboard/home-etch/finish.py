from pathlib import Path
import pcbnew as p
import sys,json
r=Path(__file__).resolve().parent.parent;b=p.LoadBoard(str(r/'dashboard.kicad_pcb'))
if len(sys.argv)>1:assert p.ImportSpecctraSES(b,str(r/'home-etch'/sys.argv[1]))
def v(x,y):return p.VECTOR2I(p.FromMM(x),p.FromMM(y))
if not any(not z.GetIsRuleArea() for z in b.Zones()):
 for layer in [p.F_Cu,p.B_Cu]:
  z=p.ZONE(b);z.SetLayer(layer);z.SetNet(b.FindNet('GND'));z.SetLocalClearance(p.FromMM(.4));z.SetMinThickness(p.FromMM(.3));z.SetPadConnection(p.ZONE_CONNECTION_THERMAL);z.SetThermalReliefGap(p.FromMM(.4));z.SetThermalReliefSpokeWidth(p.FromMM(.5));z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)
  poly=z.Outline();poly.NewOutline()
  for x,y in [(50.5,50.5),(101.5,50.5),(101.5,119.5),(50.5,119.5)]:poly.Append(p.FromMM(x),p.FromMM(y))
  b.Add(z)
for f in b.GetFootprints():
 if f.GetReference()=='C4':f.Reference().SetPosition(v(83.5,90))
 if f.GetReference()=='C3':f.Reference().SetPosition(v(55.9,113.2))
b.BuildConnectivity();p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(r/'dashboard.kicad_pcb'),b)
print('Vias:',sum(isinstance(t,p.PCB_VIA) for t in b.GetTracks()))
