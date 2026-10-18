#!/usr/bin/env python3
"""Render editable file-call diagram; standard library only. SVG -> PNG via rsvg."""
from pathlib import Path
import html
import json
import random

ROOT = Path(__file__).resolve().parent
W, H = 2400, 2200
INK, MUTED, BLUE, GREEN, ORANGE = '#172b4d', '#53647a', '#3269a8', '#2d8050', '#b66b10'
svg, elements = [], []
rng = random.Random(75)


def base(kind, x, y, w, h, stroke=INK, fill='transparent'):
    return dict(id=f'f{len(elements):04}', type=kind, x=x, y=y, width=w, height=h,
                angle=0, strokeColor=stroke, backgroundColor=fill, fillStyle='solid',
                strokeWidth=1.5, strokeStyle='solid', roughness=0, opacity=100,
                groupIds=[], frameId=None, roundness=None, seed=rng.randrange(2**30),
                version=1, versionNonce=rng.randrange(2**30), isDeleted=False,
                boundElements=None, updated=1, link=None, locked=False)


def rect(x, y, w, h, fill='#ffffff', stroke='#cbd5e1', radius=12):
    svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill}" stroke="{stroke}"/>')
    e = base('rectangle', x, y, w, h, stroke, fill)
    if radius:
        e['roundness'] = {'type': 3}
    elements.append(e)


def text(x, y, value, size=22, color=INK, mono=False):
    family = 'Menlo, Consolas, monospace' if mono else 'Arial, Helvetica, sans-serif'
    svg.append(f'<text x="{x}" y="{y+size}" font-family="{family}" font-size="{size}" fill="{color}">{html.escape(value)}</text>')
    e = base('text', x, y, len(value)*size*(.61 if mono else .57), size*1.3, color)
    e.update(fontSize=size, fontFamily=3 if mono else 2, text=value, originalText=value,
             textAlign='left', verticalAlign='top', containerId=None, autoResize=True, lineHeight=1.3)
    elements.append(e)


def arrow(points, dashed=False, color=MUTED):
    x, y = points[0]
    relative = [[a-x, b-y] for a, b in points]
    path = 'M' + ' L'.join(f'{a} {b}' for a, b in points)
    dash = ' stroke-dasharray="8 6"' if dashed else ''
    svg.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2"{dash} marker-end="url(#arrow)"/>')
    # Points remain relative to their first point, including negative coordinates.
    e = base('arrow', x, y, max(a for a, _ in points)-min(a for a, _ in points),
             max(b for _, b in points)-min(b for _, b in points), color)
    e.update(points=relative, startBinding=None, endBinding=None, startArrowhead=None,
             endArrowhead='arrow', elbowed=False, lastCommittedPoint=None,
             strokeStyle='dashed' if dashed else 'solid')
    elements.append(e)


def card(x, y, w, h, title, lines, color, fill):
    first = len(elements)
    rect(x, y, w, h, fill, color)
    text(x+22, y+16, title, 25, color)
    for i, line in enumerate(lines):
        text(x+22, y+61+i*34, line, 20, INK, mono=True)
    for e in elements[first:]:
        e['groupIds'] = [f'card_{first}']


rect(0, 0, W, H, '#ffffff', '#ffffff', 0)
card(700, 35, 1000, 115, 'ĐIỂM BẮT ĐẦU — chạy trên Mac',
     ['UAV_DURATION=120 UAV_RVIZ=1 bash scripts/run_forest.sh'], BLUE, '#eef5ff')
text(55, 180, 'Mũi tên liền: gọi / khởi chạy / dùng file', 22, MUTED)
arrow([(600, 198), (690, 198)])
text(760, 180, 'Nét đứt: truyền dữ liệu khi chạy', 22, MUTED)
arrow([(1200, 198), (1290, 198)], True)
text(1390, 180, 'Số 1–7 là thứ tự khởi động; các process sau đó chạy đồng thời.', 21, MUTED)
rect(40, 240, 1000, 1620, '#fffbf4', '#e4c596')
text(70, 255, 'MAC — mô phỏng + điều phối', 28, ORANGE)
rect(1140, 240, 1220, 1620, '#f5f9ff', '#b7cce6')
text(1170, 255, 'ORIN — Docker / ROS 2 Humble / TensorRT GPU', 28, BLUE)
arrow([(700, 90), (25, 90), (25, 400), (70, 400)])
arrow([(1700, 90), (2380, 90), (2380, 370), (2330, 370)])

card(70, 310, 940, 180, '1  BUILD + PRIOR MAP — trước khi bay', [
    'simulation/CMakeLists.txt → plugin + gazebo_sender',
    'scripts/prepare_octomap.sh → tools/octomap/CMakeLists.txt',
    'build_octomap_world + worlds/forest.sdf → maps/prior.bt',
], ORANGE, '#fff4df')
card(70, 535, 940, 230, '3  GAZEBO SERVER + GUI — hai process riêng', [
    'gz sim -s -r ...  /  gz sim -g ...',
    'simulation/worlds/forest.sdf → simulation/models/',
    'build/simulation/plugins/libArduPilotPlugin.dylib',
    'Source plugin: simulation/vendor/ardupilot_plugin/src/',
], ORANGE, '#fff4df')
card(70, 805, 940, 230, '3–4  SITL + TAKEOFF — chỉ tiếp tục khi HOVER READY', [
    'state/simulation/bin/arducopter',
    '  └─ simulation/config/sitl.parm',
    'simulation/sitl_control.py takeoff --altitude 3',
    'MAVLink localhost:5760; không force-arm',
], ORANGE, '#fff4df')
card(70, 1080, 940, 200, '6  STREAM CAMERA + STATE + CLOCK', [
    'build/simulation/gazebo_sender',
    '  ← simulation/bridge/gazebo_sender.cpp',
    '  ← bridge/include/{sim_wire.hpp,jpeg_rgb.hpp}',
], ORANGE, '#fff4df')
card(70, 1320, 940, 230, 'RVIZ — Mac, ROS domain 175 riêng', [
    'simulation/visualization/start_mac.sh → rviz2',
    'relay_client.py + simulation/config/forest.rviz',
    'Orin relay_server.py (Docker) → SSH TCP 7448 → Mac',
    'Path A* / MPPI / cloud / đường bay thực + lý do dừng',
    'Bật/tắt bằng UAV_RVIZ=1/0; chỉ nhận dữ liệu',
], GREEN, '#eef8f1')
card(70, 1590, 940, 210, '7  MAC GỌI OBSERVER TRÊN ORIN QUA SSH', [
    'run_forest.sh → SSH: orin_runtime.sh observe',
    'Truyền UAV_DURATION và XYZ goal cho observer',
    'Chờ observer kết thúc; hiển thị log qua tee',
    'Sau đó chạy cleanup: LAND → dừng → thu log',
], GREEN, '#eef8f1')
arrow([(540, 490), (540, 535)])
arrow([(540, 765), (540, 805)])
arrow([(540, 1035), (540, 1080)])
arrow([(540, 1280), (540, 1320)])
arrow([(540, 1550), (540, 1590)])

card(1170, 310, 1160, 125, '2  ĐỒNG BỘ + BUILD — USB SSH / rsync', [
    'simulation/orin_runtime.sh build   (được gọi từ run_forest.sh)',
], BLUE, '#eaf2ff')
card(1170, 480, 1160, 160, 'DOCKER BUILD CHAIN', [
    'simulation/Dockerfile.build → simulation-build (dependency, một lần)',
    'deploy/Dockerfile.gpu → humble-gpu (CMake/colcon build src/)',
    'simulation/Dockerfile → simulation (build receiver + launch)',
], BLUE, '#eaf2ff')
arrow([(1750, 435), (1750, 480)])
card(1170, 685, 1160, 140, '5  KHỞI ĐỘNG GPU RUNTIME — sau takeoff', [
    'orin_runtime.sh start → docker run --runtime nvidia',
    'Docker CMD → ros2 launch uav_simulation forest.launch.py',
], BLUE, '#eaf2ff')
arrow([(1750, 640), (1750, 685)])
card(1170, 880, 440, 205, 'RECEIVER LAUNCH', [
    'gazebo_receiver.launch.py',
    '  → gazebo_receiver.cpp',
    'Trong simulation/ros/',
    'TCP → RGB / state / clock',
], BLUE, '#eaf2ff')
card(1660, 880, 670, 205, 'APP LAUNCH — composition', [
    'src/uav_navigation_bringup/',
    '  launch/app.launch.py',
    'Params: /simulation/runtime.yaml',
    '  ← simulation/config/runtime.yaml',
], '#7751ad', '#f4f0ff')
arrow([(1540, 825), (1540, 850), (1390, 850), (1390, 880)])
arrow([(1980, 825), (1980, 880)])
arrow([(1980, 1085), (1980, 1120)])
card(1170, 1120, 1160, 470, 'APP INCLUDE → LAUNCH RIÊNG → NODE / EXECUTABLE', [
    'odometry_tf.launch.py              → odometry_tf_node',
    'monocular_depth.launch.py          → monocular_depth_node',
    'octomap_route_replanner.launch.py  → octomap_route_replanner_node',
    'local_navigation.launch.py        → local_navigation_node',
    'autopilot_adapter.launch.py       → autopilot_adapter_node',
    'mavros.launch.py → MAVROS apm.launch → mavros_node',
    'Node C++: src/uav_navigation_ros/src/ → uav_navigation_core',
    'Global: prior cố định → static_octomap_planner.cpp → A* 5 Hz',
    'Local: Depth Anything → cloud → MPPI; không cập nhật prior',
    'Định vị: Gazebo GT pose + vận tốc → odometry_tf → A* / MPPI',
    'Mount: depth_fp16.plan · prior.bt · output/ · driver NVIDIA',
    'RViz opt.: orin_runtime.sh visualize → relay_server.py (Docker)',
], BLUE, '#eaf2ff')
card(1170, 1620, 1160, 220, '7  OBSERVER — thực thi trong Docker trên Orin', [
    'orin_runtime.sh observe → docker exec → simulation/observe_run.py',
    'Chờ cloud/state mới + FCU connected/armed/GUIDED → publish /goal_pose',
    'Ghi vị trí, cloud/path/lệnh → summary.json; dừng sớm khi tới goal',
    'Hạn UAV_DURATION: mặc định 60 s; lệnh minh họa ở trên dùng 120 s',
], GREEN, '#eef8f1')
arrow([(1010, 1690), (1120, 1690), (1120, 1710), (1170, 1710)])
arrow([(1170, 1565), (1110, 1565), (1110, 1440), (1010, 1440)], True, GREEN)
# Runtime data connectors are distinct from file calls.
arrow([(1010, 1170), (1100, 1170), (1100, 980), (1170, 980)], True, GREEN)
text(1038, 925, 'USB', 19, GREEN)
text(1038, 950, '7447', 19, GREEN)
arrow([(1170, 1530), (1080, 1530), (1080, 1885), (30, 1885), (30, 925), (70, 925)], True, GREEN)
text(100, 1860, 'Lệnh quay về SITL: adapter → MAVROS → USB TCP 5762', 20, GREEN)
card(40, 1910, 2320, 190, 'KẾT THÚC / Ctrl+C — run_forest.sh điều phối', [
    'sitl_control.py land → chờ disarmed → orin_runtime.sh stop → docker stop → lưu runtime.log',
    'rsync log về Mac → dừng sender / GUI / SITL / Gazebo do script tạo',
    'Nếu chưa xác nhận LAND: giữ physics/SITL hoạt động và báo lỗi; không tắt mô phỏng giữa không trung.',
], GREEN, '#eef8f1')
text(60, 2130, 'Kết quả: state/simulation/run_<timestamp>_<pid>/  •  prior.bt + runtime.log + summary.json + log từng process', 22, MUTED)
text(60, 2164, 'Prior được tạo offline ở bước 1; camera chỉ cấp vật cản cho MPPI, không cập nhật map của A*.', 20, MUTED)
head = f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
head += '<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="9" refY="4" orient="auto"><path d="M0,0 L9,4 L0,8" fill="none" stroke="#53647a" stroke-width="1.5"/></marker></defs>'
(ROOT/'forest_run.svg').write_text(head+'\n'+'\n'.join(svg)+'\n</svg>\n')
scene = dict(type='excalidraw', version=2, source='uav_project', elements=elements,
             appState=dict(viewBackgroundColor='#ffffff', gridSize=None), files={})
(ROOT/'forest_run.excalidraw').write_text(json.dumps(scene, ensure_ascii=False, separators=(',', ':'))+'\n')
print(f'{len(elements)} editable elements; {ROOT}/forest_run.svg')
