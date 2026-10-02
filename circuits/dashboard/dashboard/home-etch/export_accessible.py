from pathlib import Path
import pcbnew as p
r=Path(__file__).resolve().parent.parent;b=p.LoadBoard(str(r/'dashboard.kicad_pcb'))
for f in b.GetFootprints():
 if f.GetReference().startswith('D') and f.GetReference()!='D7':continue
 if f.GetReference() in ['BZ1','J1']:continue
 f.BuildCourtyardCaches();poly=f.GetCourtyard(p.B_CrtYd if f.GetLayer()==p.B_Cu else p.F_CrtYd)
 if not poly.OutlineCount():continue
 box=poly.BBox();z=p.ZONE(b);z.SetIsRuleArea(True);z.SetLayerSet(p.LSET.AllCuMask(2));z.SetDoNotAllowVias(True);z.SetDoNotAllowTracks(False);z.SetDoNotAllowPads(False);z.SetDoNotAllowFootprints(False);z.SetDoNotAllowZoneFills(False)
 outline=z.Outline();outline.NewOutline()
 for x,y in [(box.GetLeft(),box.GetTop()),(box.GetRight(),box.GetTop()),(box.GetRight(),box.GetBottom()),(box.GetLeft(),box.GetBottom())]:outline.Append(x,y)
 b.Add(z)
p.SaveBoard(str(r/'dashboard.kicad_pcb'),b);assert p.ExportSpecctraDSN(b,str(r/'home-etch/accessible.dsn'))
f=r/'home-etch/accessible.dsn';s=f.read_text();s=s.replace('(class kicad_default 3.3V 5V GND ','(class kicad_default GND ');idx=s.index('    (class kicad_default');s=s[:idx]+'''    (class accessory 3.3V (circuit (use_via "Via[0-1]_2000:1000_um")) (rule (width 800) (clearance 300)))
    (class supply 5V (circuit (use_via "Via[0-1]_2000:1000_um")) (rule (width 500) (clearance 300)))
'''+s[idx:];f.write_text(s)
