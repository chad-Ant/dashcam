"""Build revision B placement from the updated schematic netlist."""
from pathlib import Path
import pcbnew as p
import xml.etree.ElementTree as E
import re,json
ROOT=Path(__file__).resolve().parent.parent;OUT=ROOT/'home-etch'
LIB=Path(r'C:\Program Files\KiCad\10.0\share\kicad\footprints')
def mm(x):return p.FromMM(x)
def v(x,y):return p.VECTOR2I(mm(x),mm(y))
r=E.parse(OUT/'source.xml').getroot()
a=p.LoadBoard(str(ROOT/'revision-b/revision-a-source.kicad_pcb'))
af={f.GetReference():f for f in a.GetFootprints()}
b=p.BOARD();b.SetCopperLayerCount(2);b.GetDesignSettings().SetBoardThickness(mm(1.6))
ds=b.GetDesignSettings();ds.m_MinClearance=mm(.3);ds.m_TrackMinWidth=mm(.3);ds.m_CopperEdgeClearance=mm(.3)
dc=ds.m_NetSettings.GetDefaultNetclass();dc.SetClearance(mm(.3));dc.SetTrackWidth(mm(.3));dc.SetViaDiameter(mm(2));dc.SetViaDrill(mm(1))
nets={}
for n in r.findall('./nets/net'):
    ni=p.NETINFO_ITEM(b,n.get('name'));b.Add(ni);nets[n.get('name')]=ni
pn={(n.get('ref'),n.get('pin')):nets[net.get('name')] for net in r.findall('./nets/net') for n in net.findall('node')}
rootuuid=re.search(r'\(uuid "([^"]+)"',(ROOT/'dashboard.kicad_sch').read_text())[1]
placement={'J2':(94.6,60,0,True),'U3':(79.4,115.5,90,True),'J3':(100.3,71.5,270,True),
           'U1':(60,61.5,90,False),'U2':(91,60.5,90,False),'BZ1':(71,62,0,False),
           'C1':(54.5,54.5,0,False),'C2':(86.6,54.5,0,False),'C3':(55.9,113.2,90,True),'C4':(77.5,90,90,True),'R17':(65.5,70,0,False),
           'J1':(70.92,71.5,90,False),'D6':(74.73,52.5,0,False),'R16':(94,71.5,0,False)}
for i,x in enumerate([56,66,76,86,96],1):
    placement[f'D{i}']=(x,75.19,270,False)
    for j,ref in enumerate([f'R{i}',f'R{i+5}',f'R{i+10}']):placement[ref]=(x-3+j*2.5,88,90,False)
fps={}
for c in r.findall('./components/comp'):
    ref=c.get('ref');lib,name=c.findtext('footprint').split(':',1)
    if ref in ['U4','L1','D7']:
        f=p.FOOTPRINT(af[ref]);f.Rotate(v(0,0),p.EDA_ANGLE(-90,p.DEGREES_T));f.Move(v(136,33));b.Add(f)
    else:
        f=p.FootprintLoad(str(ROOT/'dashboard.pretty' if lib=='dashboard' else LIB/(lib+'.pretty')),name)
        assert f,ref
        b.Add(f);x,y,angle,back=placement[ref]
        f.SetPosition(v(x,y));f.SetOrientationDegrees(angle)
        if back:f.Flip(f.GetPosition(),False)
    f.SetReference(ref);f.SetValue(c.findtext('value'));f.SetFPID(p.LIB_ID(lib,name))
    path=p.KIID_PATH();path.push_back(p.KIID(rootuuid));path.push_back(p.KIID(c.findtext('tstamps')));f.SetPath(path)
    for pad in f.Pads():
        if (ref,pad.GetNumber()) in pn:pad.SetNet(pn[(ref,pad.GetNumber())])
        else:pad.SetNetCode(0)
    for field in c.findall('./fields/field'):
        if field.get('name')=='Footprint':continue
        f.SetField(field.get('name'),field.text or '');f.GetField(field.get('name')).SetVisible(False)
    f.SetSheetname('Root');f.SetSheetfile('dashboard.kicad_sch')
    f.Value().SetVisible(False);f.Reference().SetVisible(True)
    back=f.GetLayer()==p.B_Cu
    f.Reference().SetLayer(p.B_SilkS if back else p.F_SilkS);f.Reference().SetMirrored(back)
    f.Reference().SetTextSize(v(.8,.8));f.Reference().SetTextThickness(mm(.12));f.Reference().SetTextAngle(p.EDA_ANGLE(0,p.DEGREES_T))
    pos=f.GetPosition();f.Reference().SetPosition(v(p.ToMM(pos.x),p.ToMM(pos.y)-3))
    if ref in ['D1','D2','D3','D4','D5']:f.SetLocked(True)
    fps[ref]=f
def line(start,end,layer=p.Dwgs_User):
    s=p.PCB_SHAPE(b);s.SetShape(p.SHAPE_T_SEGMENT);s.SetStart(v(*start));s.SetEnd(v(*end));s.SetWidth(mm(.3));s.SetLayer(layer);b.Add(s)
def rect(x,y,w,h,layer):
    for start,end in [((x,y),(x+w,y)),((x+w,y),(x+w,y+h)),((x+w,y+h),(x,y+h)),((x,y+h),(x,y))]:line(start,end,layer)
def text(s,x,y,size=1,layer=p.Dwgs_User):
    t=p.PCB_TEXT(b);t.SetText(s);t.SetPosition(v(x,y));t.SetTextSize(v(size,size));t.SetTextThickness(mm(.3));t.SetLayer(layer);b.Add(t)
rect(50,50,52,70,p.Edge_Cuts);rect(50.5,92,51,28,p.Dwgs_User)
text('51 x 28 DISPLAY - FRONT',76,106,1.3);text('Wired header / mounting holes not drilled',76,110,.85)
text('HOME ETCH - HAND-SOLDERED THROUGH CONNECTIONS',76,124,.9)
b.BuildConnectivity();p.SaveBoard(str(ROOT/'dashboard.kicad_pcb'),b);p.SaveBoard(str(OUT/'dashboard-layout-draft.kicad_pcb'),b)
(OUT/'placed-pads.json').write_text(json.dumps({ref:[{'n':q.GetNumber(),'x':round(p.ToMM(q.GetPosition().x),4),'y':round(p.ToMM(q.GetPosition().y),4),'net':q.GetNetname()} for q in f.Pads() if q.GetNumber()] for ref,f in fps.items()},indent=2))
print('Placed 37 components on 52 x 70 mm revision B board.')
