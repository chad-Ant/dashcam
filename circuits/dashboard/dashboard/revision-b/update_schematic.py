from pathlib import Path
import re,uuid
ROOT=Path(__file__).resolve().parent.parent
WORK=Path(r'C:\Users\THOMAS\.codex\visualizations\2026\09\26\01a0dbd7-cc36-73a0-b412-1c99219f180f')
exec((WORK/'fix_schematic.py').read_text().split('edits=[]; template=None')[0])
LIB=Path(r'C:\Program Files\KiCad\10.0\share\kicad\footprints')
led=(LIB/'LED_THT.pretty'/'LED_D5.0mm-4_RGB.kicad_mod').read_text().replace('LED_D5.0mm-4_RGB','LED_RGB_5mm_ARGB_P1.27')
led=re.sub(r'\(pad "([1234])"',lambda m:'(pad "'+{'1':'1','2':'4','3':'3','4':'2'}[m[1]]+'"',led)
(ROOT/'dashboard.pretty'/'LED_RGB_5mm_ARGB_P1.27.kicad_mod').write_text(led)
header=(LIB/'Connector_PinHeader_2.54mm.pretty'/'PinHeader_1x05_P2.54mm_Vertical.kicad_mod').read_text().replace('PinHeader_1x05_P2.54mm_Vertical','Display_Header_5pin')
header=re.sub(r'\(pad "([12345])"',lambda m:'(pad "'+{'1':'2','2':'5','3':'1','4':'3','5':'4'}[m[1]]+'"',header)
(ROOT/'dashboard.pretty'/'Display_Header_5pin.kicad_mod').write_text(header)

oldname='SparkFun_Connector_I2CStandard_1x04_P2_54mm';newname='Display_5pin'
edits=[];newsym=None
for a,b in spans(s):
    t=s[a:b]
    if t.startswith('(lib_symbols'):
        for c,d in spans(t):
            old=t[c:d]
            if not old.startswith('(symbol "dashboard:'+oldname+'"'):continue
            newsym=old.replace(oldname,newname).replace('(start -1.27 3.81)','(start -1.27 6.35)')
            newsym=prop(newsym,'Value','DISPLAY');newsym=prop(newsym,'Footprint','dashboard:Display_Header_5pin')
            newsym=prop(newsym,'Description','Display interface: GND, 3V3 I2C level, SDA, SCL and 5V display supply')
            body=next(newsym[e:f] for e,f in spans(newsym) if newsym[e:f].startswith('(symbol "'+newname+'_1_1"'))
            pin=next(body[e:f] for e,f in spans(body) if body[e:f].startswith('(pin ') and '(number "4"' in body[e:f])
            pin=pin.replace('(at -5.08 2.54 0)','(at -5.08 5.08 0)').replace('Pin_4','Pin_5').replace('(number "4"','(number "5"')
            extra='(text "5V" (at 1.27 5.08 0) (effects (font (size 1 1))))\n'+pin
            newbody=body[:body.rfind(')')]+extra+body[body.rfind(')'):]
            newsym=newsym.replace(body,newbody)
            t=t[:c]+newsym+t[d:];break
        edits.append((a,b,t))
    elif t.startswith('(symbol\n'):
        ref=re.search(r'\(property "Reference" "([^"]+)"',t)[1]
        if ref in ['D1','D2','D3','D4','D5']:
            t=prop(t,'Footprint','dashboard:LED_RGB_5mm_ARGB_P1.27')
        elif ref=='J3':
            t=prop(t,'Footprint','TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-10-2.54_1x10_P2.54mm_Horizontal')
            t=prop(t,'Description','Phoenix Contact MPT 0,5/10-2,54; part 1725737; 10-position 2.54mm screw terminal')
        elif ref=='D6':t=prop(t,'Footprint','LED_THT:LED_D3.0mm')
        elif ref=='J1':
            t=t.replace('dashboard:'+oldname,'dashboard:'+newname)
            t=prop(t,'Value','DISPLAY');t=prop(t,'Footprint','dashboard:Display_Header_5pin')
            t=prop(t,'Description','Display interface: GND, 3V3 I2C level, SDA, SCL and 5V display supply')
            t=t[:t.rfind(')')]+f'(pin "5" (uuid "{uuid.uuid4()}"))\n'+t[t.rfind(')'):]
        if t!=s[a:b]:edits.append((a,b,t))
assert newsym
for a,b,t in reversed(edits):s=s[:a]+t+s[b:]
extra=f'''(wire (pts (xy 38.1 29.21) (xy 40.64 29.21)) (stroke (width 0) (type default)) (uuid "{uuid.uuid4()}"))
(label "5V" (at 40.64 29.21 0) (effects (font (size 1.0 1.0)) (justify left bottom)) (uuid "{uuid.uuid4()}"))
'''
s=s[:s.rfind(')')]+extra+s[s.rfind(')'):]
assert src.read_bytes()==raw
src.write_text(s)
libfile=ROOT/'dashboard.kicad_sym';lib=libfile.read_text()
lib=lib[:lib.rfind(')')]+newsym.replace('(symbol "dashboard:'+newname+'"','(symbol "'+newname+'"',1)+'\n'+lib[lib.rfind(')'):]
libfile.write_text(lib)
print('Updated RGB pad mapping, smaller main terminal, LED indicator and five-pin display interface.')
