#!/usr/bin/env python3
"""Deterministic vector A4 calibration pack. No camera or ISP access.

Host dependencies: Python 3, pycairo, Pillow. All dimensions are millimetres.
Outputs stay under this script's directory; legacy chart files are untouched.
"""
import argparse
import json
import math
from pathlib import Path

import cairo
from PIL import Image

HERE = Path(__file__).resolve().parent
PT_MM = 72 / 25.4
PAPER = (297, 210)
MARKER_MM = 26
MARKER_XY = [(10, 10), (261, 10), (261, 174), (10, 174)]
# OpenCV DICT_4X4_50, upright inner 4x4 bits, white=1. Black one-cell border.
# Checked against OpenCV in validate_chart.py; no OpenCV needed to generate.
BITS = {
    10: ['1111', '1001', '1001', '0001'],
    11: ['0001', '0001', '1010', '0111'],
    12: ['0000', '1110', '1011', '0111'],
    13: ['0010', '1010', '0000', '1111'],
    14: ['0010', '0100', '1011', '0001'],
    15: ['0010', '0110', '0011', '1110'],
    16: ['0100', '0110', '0110', '0101'],
    17: ['0110', '0110', '0000', '0000'],
    18: ['0110', '1100', '0101', '1110'],
    19: ['0111', '0110', '1010', '1111'],
    20: ['1000', '0110', '1000', '1011'],
    21: ['1011', '0000', '0010', '1011'],
}
COLOURS = [
    (115,82,68),(194,150,130),(98,122,157),(87,108,67),(133,128,177),(103,189,170),
    (214,126,44),(80,91,166),(193,90,99),(94,60,108),(157,188,64),(224,163,46),
    (56,61,150),(70,148,73),(175,54,60),(231,199,31),(187,86,149),(8,133,161),
    (243,243,242),(200,200,200),(160,160,160),(122,122,121),(85,85,85),(52,52,52),
]


def rect(ctx, x, y, w, h, rgb):
    ctx.set_source_rgb(*(v / 255 for v in rgb))
    ctx.rectangle(x, y, w, h)
    ctx.fill()


def text(ctx, x, y, message, size=2.5, bold=False, centre=False):
    ctx.select_font_face('DejaVu Sans', cairo.FONT_SLANT_NORMAL,
                         cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    ctx.set_font_size(size)
    ctx.set_source_rgb(.12, .12, .12)
    if centre:
        ext = ctx.text_extents(message)
        x -= ext.width / 2 + ext.x_bearing
    ctx.move_to(x, y)
    ctx.show_text(message)


def patch(name, rgb, box, kind='flat', inset=.2):
    x, y, w, h = box
    return dict(id=name, kind=kind, nominal_srgb=list(rgb), rect_mm=list(box),
                roi_mm=[x + w*inset, y + h*inset, w*(1-2*inset), h*(1-2*inset)])


def geometry(sheet, title, first_id):
    return dict(schema='imx296-chart-v2', sheet=sheet, title=title,
                paper_mm=list(PAPER), coordinate_system='top-left; x right, y down; mm',
                colour_reference='nominal sRGB artwork only, NOT measured print reflectance',
                dictionary='DICT_4X4_50', marker_order='TL, TR, BR, BL; upright',
                markers=[dict(id=first_id+i, rect_mm=[x,y,MARKER_MM,MARKER_MM])
                         for i,(x,y) in enumerate(MARKER_XY)],
                scale_bar_mm=[98.5,193,100], patches=[], edges=[], textures=[])


def build_geometry():
    colour = geometry('01_colour', 'COLOUR / NEUTRAL BALANCE', 10)
    for i,rgb in enumerate(COLOURS):
        colour['patches'].append(patch(f'C{i+1:02}', rgb,
                                      [42.5+(i%6)*36, 39+(i//6)*36,32,32]))
    for side,x in [('L',12),('R',269)]:
        for i,(v,y) in enumerate(zip([96,160,224],[65,100,135])):
            colour['patches'].append(patch(f'{side}{i+1}',(v,v,v),[x,y,16,20]))
    noise = geometry('02_noise_tone', 'NOISE / SHADOW RESPONSE', 14)
    for i,v in enumerate([16,32,64,96,160,224]):
        noise['patches'].append(patch(f'N{v:03}',(v,v,v),
                                     [12+(i%3)*96.5,48+(i//3)*65,80,50]))
    for i,v in enumerate([0,4,8,12,16,24,32,40,48,64]):
        noise['patches'].append(patch(f'S{v:03}',(v,v,v),[49.5+i*20,177,18,8],
                                     kind='shadow_step'))
    detail = geometry('03_detail', 'DETAIL / DENOISING TRADE-OFF', 18)
    for i,(orientation,polarity) in enumerate([('vertical',1),('horizontal',1),
                                             ('vertical',-1),('horizontal',-1)]):
        detail['edges'].append(dict(id=f'E{i+1}', orientation=orientation,
            angle_from_axis_deg=5, polarity=polarity, nominal_srgb=[96,180],
            rect_mm=[48+(i%2)*113,43+(i//2)*62,88,48],
            roi_mm=[48+(i%2)*113+22,43+(i//2)*62+10,44,28]))
    for i,period in enumerate([4,2,1,.5]):
        detail['textures'].append(dict(id=f'T{i+1}', kind='checker',
            period_mm=period, nominal_srgb=[96,180], rect_mm=[48+i*51,167,45,15]))
    return [colour,noise,detail]


def draw_marker(ctx, marker):
    x,y,w,h = marker['rect_mm']
    # A white quiet zone of at least one cell is reserved in layout.
    rect(ctx,x,y,w,h,(0,0,0))
    cell = w / 6
    for row,bits in enumerate(BITS[marker['id']]):
        for col,bit in enumerate(bits):
            if bit == '1':
                rect(ctx,x+(col+1)*cell,y+(row+1)*cell,cell,cell,(255,255,255))


def draw_common(ctx, page, number):
    rect(ctx,0,0,*PAPER,(255,255,255))
    for marker in page['markers']:
        draw_marker(ctx,marker)
    text(ctx,48,18,f'IMX296  /  {page["title"]}',4.0,True)
    text(ctx,48,25,f'V2   |   SHEET {number} / 3   |   A4 LANDSCAPE   |   PRINT AT 100%',2.5)
    text(ctx,48,31,'Nominal RGB artwork - not a certified colour or reflectance reference.',2.25)
    x,y,length = page['scale_bar_mm']
    ctx.set_source_rgb(.15,.15,.15)
    ctx.set_line_width(.2)
    ctx.move_to(x,y)
    ctx.line_to(x+length,y)
    for i in range(11):
        ctx.move_to(x+i*10,y-1)
        ctx.line_to(x+i*10,y+(2 if i%5==0 else 1))
    ctx.stroke()
    text(ctx,148.5,199,'100 mm - measure after printing',2.3,centre=True)
    text(ctx,148.5,205,'Matte stock / no auto-enhance / no fit-to-page / all four markers visible',2.2,centre=True)


def draw_edge(ctx, edge):
    x,y,w,h = edge['rect_mm']
    dark,light = edge['nominal_srgb']
    if edge['polarity'] < 0:
        dark,light = light,dark
    rect(ctx,x,y,w,h,(light,)*3)
    t = math.tan(math.radians(edge['angle_from_axis_deg']))
    if edge['orientation'] == 'vertical':
        points = [(x,y),(x+w/2-h*t/2,y),(x+w/2+h*t/2,y+h),(x,y+h)]
    else:
        points = [(x,y),(x+w,y),(x+w,y+h/2+w*t/2),(x,y+h/2-w*t/2)]
    ctx.set_source_rgb(*(dark/255,)*3)
    ctx.move_to(*points[0])
    for pt in points[1:]:
        ctx.line_to(*pt)
    ctx.close_path()
    ctx.fill()


def draw_page(ctx,page,number):
    draw_common(ctx,page,number)
    for p in page['patches']:
        rect(ctx,*p['rect_mm'],p['nominal_srgb'])
        x,y,w,h = p['rect_mm']
        if p['kind'] == 'shadow_step':
            text(ctx,x+w/2,y+h+2.8,str(p['nominal_srgb'][0]),1.8,centre=True)
        elif p['id'].startswith('N'):
            text(ctx,x,y+h+4,f'{p["id"]}   /   RGB {p["nominal_srgb"][0]}   /   80 x 50 mm',2.3)
        else:
            text(ctx,x+w/2,y+h+2.9,p['id'],2.0,centre=True)
    for edge in page['edges']:
        draw_edge(ctx,edge)
        x,y,w,h = edge['rect_mm']
        text(ctx,x,y+h+4,f'{edge["id"]}   /   {edge["orientation"]}   /   5-degree slant',2.4)
    for tile in page['textures']:
        x,y,w,h = tile['rect_mm']
        cell = tile['period_mm']/2
        rect(ctx,x,y,w,h,(180,)*3)
        ctx.save()
        ctx.rectangle(x,y,w,h)
        ctx.clip()
        for row in range(math.ceil(h/cell)):
            for col in range(math.ceil(w/cell)):
                if (row+col)%2 == 0:
                    rect(ctx,x+col*cell,y+row*cell,cell,cell,(96,)*3)
        ctx.restore()
        text(ctx,x,y+h+3.2,f'{tile["id"]} / {tile["period_mm"]:g} mm period',2.1)
    if page['sheet'] == '01_colour':
        text(ctx,148.5,187,'L1-R1 / L2-R2 / L3-R3: repeated neutrals for field consistency',2.1,centre=True)
    elif page['sheet'] == '02_noise_tone':
        text(ctx,148.5,42,'Lock exposure, gain and white balance. Capture 64 stationary frames.',2.6,centre=True)
        text(ctx,148.5,173,'Shadow steps: nominal RGB code, not measured luminance',2.2,centre=True)
    else:
        text(ctx,148.5,162,'RGB 96 / 180 edges and texture: judge noise together with retained detail',2.2,centre=True)


def validate_layout(pages):
    ids=[]
    for page in pages:
        objects = page['patches']+page['edges']+page['textures']
        for obj in objects+page['markers']:
            x,y,w,h = obj['rect_mm']
            if w <= 0 or h <= 0 or x < 0 or y < 0 or x+w > PAPER[0] or y+h > PAPER[1]:
                raise ValueError(f'Out of paper bounds: {obj}')
        for marker in page['markers']:
            ids.append(marker['id'])
            x,y,w,h = marker['rect_mm']
            q = w/6
            for obj in objects:
                a,b,c,d = obj['rect_mm']
                if a < x+w+q and a+c > x-q and b < y+h+q and b+d > y-q:
                    raise ValueError(f'Marker quiet-zone collision: {obj["id"]}')
        for i,a in enumerate(objects):
            x,y,w,h = a['rect_mm']
            for b in objects[i+1:]:
                u,v,s,t = b['rect_mm']
                if x < u+s and x+w > u and y < v+t and y+h > v:
                    raise ValueError(f'Overlapping patterns: {a["id"]}, {b["id"]}')
    if len(set(ids)) != 12 or any(i < 10 for i in ids):
        raise ValueError('Marker IDs must be unique and distinct from legacy 0-3')


def build(out):
    pages = build_geometry()
    validate_layout(pages)
    # Refuse replacement; regenerating into a new directory is explicit.
    out.mkdir(parents=True,exist_ok=False)
    pack = cairo.PDFSurface(str(out/'imx296_chart_v2_a4.pdf'),PAPER[0]*PT_MM,PAPER[1]*PT_MM)
    previews=[]
    for n,page in enumerate(pages,1):
        stem=page['sheet']
        ctx=cairo.Context(pack)
        ctx.scale(PT_MM,PT_MM)
        draw_page(ctx,page,n)
        ctx.show_page()
        for extension,surface_type in [('pdf',cairo.PDFSurface),('svg',cairo.SVGSurface)]:
            surface=surface_type(str(out/f'{stem}.{extension}'),PAPER[0]*PT_MM,PAPER[1]*PT_MM)
            ctx=cairo.Context(surface)
            ctx.scale(PT_MM,PT_MM)
            draw_page(ctx,page,n)
            surface.finish()
        scale=300/25.4
        raster=cairo.ImageSurface(cairo.FORMAT_RGB24,round(PAPER[0]*scale),round(PAPER[1]*scale))
        ctx=cairo.Context(raster)
        ctx.scale(scale,scale)
        draw_page(ctx,page,n)
        raster.write_to_png(str(out/f'{stem}_300dpi.png'))
        (out/f'{stem}_geometry.json').write_text(json.dumps(page,indent=2)+'\n')
        with Image.open(out/f'{stem}_300dpi.png') as im:
            # Pillow shipped on this host predates the Resampling enum.
            previews.append(im.convert('RGB').resize((1000,707),Image.LANCZOS))
    pack.finish()
    overview=Image.new('RGB',(1040,3*747+20),(226,229,232))
    for i,preview in enumerate(previews):
        overview.paste(preview,(20,20+i*747))
    overview.save(out/'preview.png')
    (out/'manifest.json').write_text(json.dumps(dict(schema='imx296-chart-v2',
        pages=[p['sheet'] for p in pages],print_pdf='imx296_chart_v2_a4.pdf',
        source_rgb='nominal sRGB, PDF DeviceRGB; printed output must be characterized'),indent=2)+'\n')
    print(f'Created 3 vector sheets, pack, geometry, lossless PNGs and preview: {out}')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=HERE/'output',help='New output directory (must not exist)')
    args=parser.parse_args()
    build(args.out.resolve())
