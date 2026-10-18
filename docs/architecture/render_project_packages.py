#!/usr/bin/env python3
"""Update the existing editable package scene and export its SVG preview."""
from copy import deepcopy
from pathlib import Path
import html
import json
import xml.etree.ElementTree as ET
from PIL import ImageFont

ROOT = Path(__file__).resolve().parent
path = ROOT / 'project_packages.excalidraw'
scene = json.loads(path.read_text())
elements = scene['elements']
by_id = {e['id']: e for e in elements}
old_svg = ET.parse(ROOT / 'project_packages.svg')
weights = {e.text: e.get('font-weight', '400')
           for e in old_svg.iter('{http://www.w3.org/2000/svg}text')}
for e in elements:
    if e['type'] == 'text':
        e.setdefault('customData', {}).setdefault('svgFontWeight', weights.get(e['text'], '400'))


def label(eid, value, *, y=None, size=None):
    e = by_id[eid]
    e.update(text=value, originalText=value)
    if y is not None:
        e['y'] = y
    if size is not None:
        e['fontSize'] = size


def add(eid, template, value, x, y, size):
    if eid not in by_id:
        e = deepcopy(by_id[template])
        e.update(id=eid, x=x, y=y, boundElements=None, containerId=None)
        elements.append(e)
        by_id[eid] = e
    label(eid, value, y=y, size=size)


label('e0048', 'Prior cố định → A* 3D, 5 Hz')
label('e0044', 'RGB → cloud cho MPPI')
add('p_node_launches', 'e0033', 'launch/*.launch.xml — launch riêng từng node', 710, 293, 18)
label('e0023', 'runtime.yaml · sitl.parm · forest.rviz', size=17)
label('e0024', 'Profile navigation / autopilot / RViz', size=17)
add('p_visualization', 'e0025', 'visualization/ — RViz + SSH relay', 96, 878, 17)
label('e0025', 'sitl_control.py', y=915, size=20)
label('e0026', 'Takeoff / land / status', y=945, size=18)
add('p_orin_helper', 'e0025', 'orin_runtime.sh', 96, 985, 20)
add('p_orin_helper_role', 'e0026', 'Docker: build / start / observe / stop', 96, 1015, 17)
add('p_observer', 'e0025', 'observe_run.py', 96, 1050, 20)
add('p_observer_role', 'e0026', 'Goal, theo dõi lượt chạy, summary.json', 96, 1080, 17)
label('e0027', 'Build: CMakeLists.txt · Dockerfile', y=1110, size=17)
label('e0028', 'Dockerfile.build: image dependency', y=1140, size=17)
label('e0077', 'metric_depth.cpp', y=885)
add('p_static_planner', 'e0078', 'static_octomap_planner.cpp', 1845, 919, 19)
add('p_static_planner_role', 'e0048', 'Cache clearance cho A* trên prior cố định', 1845, 948, 16)
by_id['p_static_planner_role']['groupIds'] = ['core']
label('e0078', 'octomap_planner.cpp', y=980)
label('e0079', 'command_conditioner.cpp', y=1010)
label('e0080', 'trajectory_safety_checker.cpp', y=1040)
label('e0081', 'stabilization.cpp', y=1070)
add('p_run_forest', 'e0107', 'run_forest.sh', 846, 1496, 22)
label('e0108', 'prepare_octomap.sh', y=1532, size=22)
label('e0109', 'Build/test, chuẩn bị map, điều phối Mac + Orin', y=1568, size=18)

svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="2560" height="1720" '
       'viewBox="0 0 2560 1720" role="img" aria-labelledby="title desc">',
       '<title id="title">uav_project — Package and dependency diagram</title>',
       '<desc id="desc">Cấu trúc source, prior cố định cho A* 5 Hz, depth cho local MPPI.</desc>',
       '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" '
       'markerWidth="9" markerHeight="9" orient="auto-start-reverse">'
       '<path d="M1 1 L9 5 L1 9" fill="none" stroke="#64748b" stroke-width="1.5"/>'
       '</marker></defs>']
for e in elements:
    if e.get('isDeleted'):
        continue
    x, y, w, h = (e[k] for k in ('x', 'y', 'width', 'height'))
    stroke, fill = e['strokeColor'], e['backgroundColor']
    if e['type'] == 'rectangle':
        radius = 12 if e.get('roundness') else 0
        svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" '
                   f'fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>')
    elif e['type'] == 'text':
        size = e['fontSize']
        mono = e['fontFamily'] == 3
        font_file = '/System/Library/Fonts/Menlo.ttc' if mono else '/System/Library/Fonts/Supplemental/Arial.ttf'
        font = ImageFont.truetype(font_file, size)
        e['width'] = font.getlength(e['text'])
        e['height'] = size * e.get('lineHeight', 1.25)
        family = 'Menlo, Consolas, monospace' if mono else 'Arial, Helvetica, sans-serif'
        weight = e.get('customData', {}).get('svgFontWeight', '400')
        svg.append(f'<text x="{x}" y="{y+size}" font-family="{family}" font-size="{size}" '
                   f'font-weight="{weight}" fill="{stroke}">{html.escape(e["text"])}</text>')
    elif e['type'] == 'arrow':
        points = [(x+a, y+b) for a, b in e['points']]
        d = 'M' + ' L'.join(f'{a} {b}' for a, b in points)
        dash = ' stroke-dasharray="8 6"' if e.get('strokeStyle') == 'dashed' else ''
        svg.append(f'<path d="{d}" fill="none" stroke="{stroke}" stroke-width="1.5"'
                   f'{dash} marker-end="url(#arrow)"/>')
    else:
        raise ValueError(f'Unsupported element type: {e["type"]}')
svg.append('</svg>')
path.write_text(json.dumps(scene, ensure_ascii=False, separators=(',', ':'))+'\n')
(ROOT / 'project_packages.svg').write_text('\n'.join(svg)+'\n')
print(f'{len(elements)} editable elements; updated project_packages.svg')
