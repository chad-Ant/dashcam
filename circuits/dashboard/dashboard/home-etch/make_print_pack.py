from pathlib import Path
from io import BytesIO
import json,math
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.colors import HexColor,black,white
from pypdf import PdfReader,PdfWriter,Transformation
from pypdf.generic import RectangleObject

R=Path(__file__).resolve().parent.parent;O=R/'release/print';O.mkdir(exist_ok=True)
links=json.loads((R/'home-etch/wire-links.json').read_text())
drills=json.loads((R/'release/checks/drill-map.json').read_text())
W,H=A4;ink=HexColor('#17334A');muted=HexColor('#526574')
def newpage():
 stream=BytesIO();return stream,canvas.Canvas(stream,pagesize=A4)
def page(stream,c):
 c.showPage();c.save();stream.seek(0);return PdfReader(stream).pages[0]
def heading(c,title,subtitle):
 c.setFillColor(ink);c.setFont('Helvetica-Bold',17);c.drawString(18*mm,277*mm,title)
 c.setFillColor(muted);c.setFont('Helvetica',9);c.drawString(18*mm,269*mm,subtitle)
 c.setStrokeColor(ink);c.setLineWidth(.6);c.line(18*mm,264*mm,192*mm,264*mm)
def lines(c,items,x,y,size=10,leading=5):
 c.setFillColor(black);c.setFont('Helvetica',size)
 for s in items:c.drawString(x*mm,y*mm,s);y-=leading
 return y
def board_merge(dest,filename,mirror,x,y,scale=1):
 src=PdfReader(R/'home-etch'/filename).pages[0];sw=float(src.mediabox.width);sh=float(src.mediabox.height)
 left=sw-102*mm if mirror else 50*mm;bottom=sh-120*mm
 # The raw KiCad PDF is explicitly plotted at 1:1. Clip only; never fit to page.
 src.cropbox=RectangleObject([left-.5*mm,bottom-.5*mm,left+52.5*mm,bottom+70.5*mm])
 dest.merge_transformed_page(src,Transformation().scale(scale).translate(x*mm-left*scale,y*mm-bottom*scale),expand=False)
def footer(c,s):
 c.setFillColor(muted);c.setFont('Helvetica',8);c.drawString(18*mm,16*mm,s)

writer=PdfWriter()
for front in [True,False]:
 stream,c=newpage();heading(c,'FRONT COPPER' if front else 'REAR COPPER','TONER TRANSFER | 52 x 70 mm | Print at Actual size / 100%')
 lines(c,['Front artwork is already mirrored.' if front else 'Rear artwork is intentionally not mirrored.',
          'Place the TONER side against copper. Do not mirror again in the printer dialog.',
          'Black = copper to retain. Small white hole centres are drilling pilots.'],18,254,9,5)
 x,y=79,150
 c.setStrokeColor(black);c.setLineWidth(.2*mm)
 for cx,cy in [(x-3,y-3),(x+55,y-3),(x-3,y+73),(x+55,y+73)]:
  c.line((cx-1.5)*mm,cy*mm,(cx+1.5)*mm,cy*mm);c.line(cx*mm,(cy-1.5)*mm,cx*mm,(cy+1.5)*mm)
 c.setFont('Helvetica',9);c.drawCentredString(105*mm,139*mm,'Board outline: 52.00 x 70.00 mm')
 # Independent X and Y scale checks, both exactly 50 mm.
 c.line(35*mm,60*mm,85*mm,60*mm);c.line(35*mm,59*mm,35*mm,61*mm);c.line(85*mm,59*mm,85*mm,61*mm)
 c.drawCentredString(60*mm,55*mm,'50.00 mm horizontal')
 c.line(165*mm,55*mm,165*mm,105*mm);c.line(164*mm,55*mm,166*mm,55*mm);c.line(164*mm,105*mm,166*mm,105*mm)
 c.saveState();c.translate(170*mm,80*mm);c.rotate(90);c.drawCentredString(0,0,'50.00 mm vertical');c.restoreState()
 lines(c,['Measure BOTH scale bars before transfer. Disable Fit / Shrink / Borderless scaling.',
          'Use the corner crosses on scrap margin to align the two faces before trimming.',
          'Check actual LED lead forming and component footprints before etching.',
          'This board requires 30 wire-filled vias and 5 insulated jumpers; see the guide.'],18,126,9,5)
 footer(c,'Home-etch prototype | Insulated display mounting remains to be measured.')
 dest=page(stream,c);board_merge(dest,'front-raw.pdf' if front else 'back-raw.pdf',front,x,y);writer.add_page(dest)
with (O/'toner-transfer.pdf').open('wb') as f:writer.write(f)

guide=PdfWriter();stream,c=newpage();heading(c,'Assembly and wire connections','HOME-ETCH PROTOTYPE | Front-view coordinates measured from the top-left corner')
lines(c,['Two copper faces; no plated holes or solder mask are assumed.',
         'Fit surface-mount electronics. RGB LEDs, piezo and the display header are through-hole.',
         'Install 30 short via wires and solder BOTH faces. Do not rely on an unplated hole.',
         'Fit the five insulated links below. They land on existing component solder joints.',
         'Link lines in the drawings identify endpoints; route the wires clear of LED/display faces.',
         'Fit vias before the piezo; leave clearance beneath its body for solder joints.'],18,254,9,6)
c.setFont('Helvetica-Bold',11);c.drawString(18*mm,210*mm,'Required insulated wire links')
ys=199
for row in links:
 c.setFillColor(ink);c.setFont('Helvetica-Bold',10);c.drawString(18*mm,ys*mm,row['reference'])
 c.setFillColor(black);c.setFont('Helvetica',10);c.drawString(36*mm,ys*mm,f"{row['from']}  to  {row['to']}   |   {row['face']}   |   direct span {row['straight_distance_mm']:.1f} mm")
 ys-=9
lines(c,['These are real assembly wires, not copper traces or optional links.',
         'KiCad models their fitted connectivity as board-only jumper footprints.',
         'The connection checks pass only for the assembly with all five links fitted.'],18,151,9,5)
c.setFont('Helvetica-Bold',11);c.drawString(18*mm,129*mm,'Drilling and mechanical fit')
lines(c,['LEDs, piezo, display header and vias: 1.0 mm holes.',
         'J3 terminal: 1.1 mm holes. USB-A shell tabs: 2.3114 mm holes; fit the actual tabs.',
         'RGB lead holes: 2.54 mm pitch, 2.0 mm copper pads; form leads if necessary.',
         'Piezo lead spacing remains the selected footprint\'s 10.0 mm; verify the actual part.',
         'Display: 51 x 28 mm envelope, five-wire connection, no guessed mounting holes.',
         'Support and insulate the display above solder joints. Check standoff height first.',
         'J3 body overhangs the right board edge by about 1.4 mm, plus wire clearance.'],18,119,9,6)
c.setFont('Helvetica-Bold',11);c.drawString(18*mm,68*mm,'Power and first checks')
lines(c,['J3.1 = regulated 5 V input; J3.4 = GND. Power XIAO separately through USB-C.',
         'USB-A output is custom 3.3 V (100-200 mA normal, 500 mA transient).',
         'Display header left-to-right: VI2C (3.3 V), 5 V, GND, SDA, SCL.',
         'Inspect both faces for bridges and continuity before applying current-limited power.',
         'D+/D- wiring is not a qualified controlled-impedance USB link.'],18,58,9,5)
footer(c,'1 / 5 | Confirm unspecified USB-A/buzzer parts and LED/display measurements before fabrication.')
guide.add_page(page(stream,c))

for front in [True,False]:
 stream,c=newpage();heading(c,'Front assembly' if front else 'Rear assembly','2:1 inspection drawing - NOT toner artwork. Connector openings face right from the front.')
 footer(c,('2 / 5' if front else '3 / 5')+' | Coloured lines show fitted insulated-wire endpoints, not etched copper.')
 dest=page(stream,c);board_merge(dest,'front-assembly-raw.pdf' if front else 'back-assembly-raw.pdf',not front,53,103,2)
 overlay,oc=newpage();colors=['#D73A49','#0066AA','#8E44AD','#008060','#C75E00']
 for i,row in enumerate(links):
  if (row['face']=='Front')!=front:continue
  coords=[((53+2*(x if front else 52-x))*mm,(103+2*(70-y))*mm) for x,y in row['endpoints_mm_from_front_top_left']]
  oc.setStrokeColor(HexColor(colors[i]));oc.setFillColor(HexColor(colors[i]));oc.setLineWidth(1)
  
  offset=(11 if i==1 else -4 if i==2 else 0)*mm
  if offset:
   path=oc.beginPath();path.moveTo(*coords[0]);path.curveTo(coords[0][0],coords[0][1]+offset,coords[1][0],coords[1][1]+offset,*coords[1]);oc.drawPath(path)
  else:oc.line(*coords[0],*coords[1])
  for x,y in coords:oc.circle(x,y,1.1*mm,stroke=1,fill=0)
  mx=(coords[0][0]+coords[1][0])/2;my=(coords[0][1]+coords[1][1])/2+offset*.75
  oc.setFillColor(white);oc.rect(mx-5*mm,my-2*mm,10*mm,4*mm,stroke=0,fill=1)
  oc.setFillColor(HexColor(colors[i]));oc.setFont('Helvetica-Bold',8);oc.drawCentredString(mx,my-.9*mm,row['reference'])
 lines(oc,['All component references are shown for assembly; there is no assumed silkscreen.',
           'Front LED body centres: x = 6, 16, 26, 36, 46 mm; y = 29 mm.',
           'RGB physical lead order from top to bottom: A, R, G, B.',
           'Display envelope starts at x = 0.5 mm, y = 42 mm; mounting holes are omitted.'
           if front else 'Fit the XIAO and connectors only after the nearby via wires are soldered and inspected.'],18,82,9,6)
 dest.merge_page(page(overlay,oc));guide.add_page(dest)

stream,c=newpage();heading(c,'Drill and via map','Front view | 2.5:1 inspection drawing - use the copper print sheets for transfer')
x0,y0,sc=40,61,2.5
c.setStrokeColor(black);c.setLineWidth(.5);c.rect(x0*mm,y0*mm,52*sc*mm,70*sc*mm)
for hole in drills['component_holes']:
 x=(x0+sc*hole['x'])*mm;y=(y0+sc*(70-hole['y']))*mm
 c.circle(x,y,hole['drill']*sc*mm/2,stroke=1,fill=0)
c.setStrokeColor(HexColor('#B44A00'));c.setFillColor(HexColor('#B44A00'));c.setFont('Helvetica',6)
for q in drills['vias']:
 x=(x0+sc*q['x'])*mm;y=(y0+sc*(70-q['y']))*mm;c.circle(x,y,sc*mm/2,stroke=1,fill=0)
 c.drawString(x+1.5*mm,y+1.1*mm,q['id'])
lines(c,['Black circles: component holes. Orange numbered circles: 1 mm wire-filled vias.',
         'Vias have 2 mm copper pads. Solder a wire on both faces at every orange location.',
         'The next page gives exact via coordinates. No mounting holes are included.'],18,45,9,6)
footer(c,'4 / 5 | This guide is enlarged. Do not use it as a 1:1 drilling template.')
guide.add_page(page(stream,c))

stream,c=newpage();heading(c,'Via-wire checklist','All positions in mm from the front top-left corner. All via drills are 1.00 mm.')
for col in range(2):
 x=18+col*90;y=251;c.setFillColor(ink);c.setFont('Helvetica-Bold',10)
 c.drawString(x*mm,y*mm,'Via');c.drawString((x+22)*mm,y*mm,'X');c.drawString((x+44)*mm,y*mm,'Y');c.drawString((x+65)*mm,y*mm,'Done')
 y-=9
 for q in drills['vias'][col*15:(col+1)*15]:
  c.setFillColor(black);c.setFont('Helvetica',10);c.drawString(x*mm,y*mm,q['id']);c.drawRightString((x+37)*mm,y*mm,f"{q['x']:.3f}");c.drawRightString((x+59)*mm,y*mm,f"{q['y']:.3f}")
  c.setStrokeColor(black);c.setLineWidth(.4);c.rect((x+68)*mm,(y-1)*mm,3*mm,3*mm);y-=9
lines(c,['Trim the wire close after soldering, without cutting into the copper pad.',
         'Check continuity from the front trace to the rear trace; visual solder alone is not proof.',
         'Also solder through-hole component leads on each face that carries a connection.',
         'Do not substitute a bare hole for a via or omit a required insulated jumper.'],18,83,9,6)
footer(c,'5 / 5 | Machine-readable coordinates and nets: release/via-wire-list.csv')
guide.add_page(page(stream,c))
with (O/'assembly-guide.pdf').open('wb') as f:guide.write(f)

# Geometry checks independent of the printer: two A4 pages and exact-scale merge matrices.
qa={'toner_pages':len(PdfReader(O/'toner-transfer.pdf').pages),'assembly_pages':len(PdfReader(O/'assembly-guide.pdf').pages),'board_mm':[52,70],'toner_merge_scale':1,'horizontal_scale_bar_mm':50,'vertical_scale_bar_mm':50,'via_wires':len(drills['vias']),'insulated_links':len(links)}
assert qa['toner_pages']==2 and qa['assembly_pages']==5
(R/'release/checks/print-geometry.json').write_text(json.dumps(qa,indent=2))
print(qa)
