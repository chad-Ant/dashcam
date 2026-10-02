from pathlib import Path
import pcbnew as p
import sys,json
ROOT=Path(__file__).resolve().parent.parent;OUT=ROOT/'revision-b'
def mm(x):return p.FromMM(x)
def v(x,y):return p.VECTOR2I(mm(x),mm(y))
b=p.LoadBoard(str(ROOT/'dashboard.kicad_pcb'));fps={f.GetReference():f for f in b.GetFootprints()}
if '--refill' not in sys.argv:
 assert p.ImportSpecctraSES(b,str(OUT/'dashboard.ses'))
 fps['R8'].Reference().SetPosition(v(76.9,85.5));fps['R8'].Reference().SetTextAngle(p.EDA_ANGLE(90,p.DEGREES_T))
 def text(s,x,y,size=.8,layer=p.F_SilkS,angle=0):
  t=p.PCB_TEXT(b);t.SetText(s);t.SetPosition(v(x,y));t.SetTextSize(v(size,size));t.SetTextThickness(mm(.12));t.SetLayer(layer);t.SetMirrored(layer==p.B_SilkS);t.SetTextAngle(p.EDA_ANGLE(angle,p.DEGREES_T));b.Add(t)
 text('DASHBOARD REV B',76,56,1)
 text('52 x 70 mm / 2 layers',76,57.5,.8)
 text('3V3 ONLY',94,65,.9,p.B_SilkS)
 text('CUSTOM PORT',94,63.5,.7,p.B_SilkS)
 for x,s in zip([70.92,73.46,76,78.54,81.08],['VI2C','5V','G','SDA','SCL']):text(s,x,75.2,.7)
 for y,s in zip([71.5,74.04,76.58,79.12,81.66,84.2,86.74,89.28,91.82,94.36],['5V','D+','D-','G','NC','NC','NC','NC','RX','TX']):text(s,98,y,.7,p.B_SilkS)
 for layer in [p.F_Cu,p.B_Cu]:
  z=p.ZONE(b);z.SetLayer(layer);z.SetNet(b.FindNet('GND'));z.SetLocalClearance(mm(.2));z.SetMinThickness(mm(.2));z.SetPadConnection(p.ZONE_CONNECTION_THT_THERMAL);z.SetThermalReliefGap(mm(.25));z.SetThermalReliefSpokeWidth(mm(.4));z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)
  poly=z.Outline();poly.NewOutline()
  for x,y in [(50.3,50.3),(101.7,50.3),(101.7,119.7),(50.3,119.7)]:poly.Append(mm(x),mm(y))
  b.Add(z)
 for q in fps['C3'].Pads():
  if q.GetNumber()=='2':q.SetLocalZoneConnection(p.ZONE_CONNECTION_FULL)
b.BuildConnectivity();p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(ROOT/'dashboard.kicad_pcb'),b)
stats={'board_mm':[52,70],'copper_layers':2,'board_thickness_mm':1.6,'footprints':len(list(b.GetFootprints())),'tracks':sum(not isinstance(t,p.PCB_VIA) for t in b.GetTracks()),'vias':sum(isinstance(t,p.PCB_VIA) for t in b.GetTracks())}
(OUT/'board-summary.json').write_text(json.dumps(stats,indent=2));print(stats)
