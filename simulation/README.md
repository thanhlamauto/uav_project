# Một ca rừng độc lập: Mac + Orin qua USB

Toàn bộ world, model, plugin, bridge và cấu hình giả lập nằm trong folder này.
Không gọi runner, source, model hoặc image experimental của repo cha.
Các lệnh dưới chạy từ root **uav_project**.

## Chạy toàn bộ bằng một lệnh trên Mac

Sau khi đã có dependency Mac và base image `uav-navigation:humble` trên Orin:

```bash
bash scripts/run_forest.sh
```

Script tự build Mac, tạo prior đầy đủ, rsync source qua USB `192.168.55.1`,
build image GPU/simulation từ source hiện tại, mở Gazebo GUI + server và SITL.
Sau khi UAV hover ở 3 m, script chạy ROS trên Orin, stream RGB/state/clock,
chờ input mới và FCU connected/armed/GUIDED rồi gửi một goal `(20,4,4)`.
Không cần tự mở sáu terminal. Lần đầu build image có thể mất vài phút;
những lần sau dùng Docker cache. Mọi dependency Orin vẫn ở trong Docker.
Image compiler `uav-navigation:simulation-build` được chuẩn bị một lần, tách
khỏi image runtime để sửa source navigation không kéo theo tải lại dependency.
Máy mới cần Internet lúc chuẩn bị dependency; USB SSH riêng không cấp Internet.

Mặc định quan sát **60 giây theo đồng hồ thực** sau khi gửi goal, sau đó LAND,
chờ disarm, dừng container và các process do script tạo. Ctrl+C cũng đi qua
bước LAND. Container dùng stop signal SIGINT để ROS launch đóng các node.
Nếu không xác nhận được LAND, script báo lỗi và giữ physics/SITL
để bạn xử lý; không tự giết một lượt khác đang chiếm port/container.

Log in ra đầu lượt, lưu riêng trong `state/simulation/run_<timestamp>_<pid>/`:
`gazebo.log`, `sitl.log`, `takeoff.log`, `sender.log`, `observe.log`, `land.log`,
`build_orin.log` và `orin/{runtime.log,output/summary.json,output/map.bt}`.
`summary.json` ghi vị trí, số cloud/path/lệnh, diagnostics và kết quả quan sát.
Hết giờ hoặc có lệnh khác 0 **không đồng nghĩa đã đạt đích hay không va chạm**.
Runner giữ nguyên thuật toán và mọi safety gate, kể cả hold do camera hazard.

```bash
# Xem các bước, không chạy/copy/build gì:
bash scripts/run_forest.sh --dry-run
# Ví dụ đổi một ca: không GUI, goal khác, quan sát 30 s:
UAV_GUI=0 UAV_DURATION=30 UAV_GOAL_X=12 UAV_GOAL_Y=2 UAV_GOAL_Z=4 bash scripts/run_forest.sh
```

Sơ đồ gọi file: [xem SVG](../docs/architecture/forest_run.svg),
[mở/chỉnh trong Excalidraw](../docs/architecture/forest_run.excalidraw).
Entry point gọi `simulation/orin_runtime.sh` qua SSH; helper này quản lý Docker
và gọi `simulation/observe_run.py` **bên trong container**, không chạy ROS trên Mac.
Các mục dưới giữ nguyên lệnh thủ công để bạn đọc hoặc chạy từng bước.

```text
simulation/
├── worlds/forest.sdf           Rừng, một Iris, RGB 640×360 @ 10 Hz
├── models/                    Iris/gimbal, mesh; resource nằm trong project
├── vendor/ardupilot_plugin/    Plugin physics ↔ ArduPilot JSON, source/license
├── bridge/                    C++ Gazebo → TCP/JPEG; clock + pose + body twist
├── ros/                       Package uav_simulation: TCP → ROS + launch
├── config/sitl.parm            ArduCopter không tự tránh vật cản
├── config/runtime.yaml         Profile mô phỏng, mount camera và bounds map
├── sitl_control.py             Chuẩn bị takeoff/land; chỉ nối SITL localhost
├── CMakeLists.txt              Build plugin và sender trên Mac
└── Dockerfile                 Thêm simulator ingress vào image GPU product
```

**Luồng chạy:**

```text
Mac: Gazebo RGB + ideal state + clock ── TCP/JPEG, USB ──► Orin Docker
                                                    gazebo_receiver
                                                         ↓
                 app.launch.py: odometry_tf → depth → OctoMap/A* → MPPI
                                                                         ↓
Mac: ArduPilot SITL ◄── MAVLink TCP 5762 qua USB ── MAVROS ◄── adapter
```

MAVROS là MAVROS thật, đọc trạng thái armed/mode/connected từ SITL. Bridge
không giả trạng thái autopilot và không truyền lệnh. Định vị là Gazebo GT,
không phải EKF; depth chỉ nhận RGB, không có depth GT/LiDAR trong luồng này.
JPEG quality 85 giữ cách truyền ảnh hiện có; đây không phải ảnh lossless.

`forest.launch.py` ghép launch riêng của receiver với `app.launch.py`.
Bạn có thể tắt/thay từng thành phần bằng `start_*` và `*_launch_file`, ví dụ
`start_depth:=false` hoặc `depth_launch_file:=/absolute/path/new_depth.launch.py`.
Các argument này truyền khi gọi `ros2 launch uav_simulation forest.launch.py`
trong container đã source workspace. Hướng dẫn: [launch và thay node](../src/uav_navigation_bringup/README.md).

## 1. Chuẩn bị Mac, làm một lần

Mac cần Gazebo Harmonic/Homebrew, CMake, Qt5, RapidJSON, libjpeg, OctoMap,
tinyxml2; Python conda `ardupilot-rviz` hiện có pymavlink. ROS không bắt buộc
trên Mac cho phần mô phỏng này. Gazebo và ArduPilot SITL là dependency cài
sẵn, không phải source của project. Dependency/plugin được phân biệt ở
[PROVENANCE.md](PROVENANCE.md).

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
cmake -S simulation -B build/simulation -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5
cmake --build build/simulation -j4
ctest --test-dir build/simulation --output-on-failure
```

Đưa binary SITL đã cài vào vùng dependency local của project. Lệnh `cp` này
là bước nhập dependency một lần; các lệnh chạy sau không dùng đường dẫn nguồn.
Binary đang dùng trên máy này đã được chuẩn bị tại `state/simulation/bin`.
Trên máy mới, build/cài ArduPilot SITL riêng rồi copy binary tương ứng.

```bash
mkdir -p state/simulation/bin
cp "$HOME/Projects/ardupilot/build/sitl/bin/arducopter" state/simulation/bin/arducopter
```

Tạo prior map đầy đủ cho đúng một ca; `full.bt` dành đối chiếu, chỉ mount
`prior.bt` vào runtime. Start sau takeoff gần `(0,0,3)`, goal `(20,4,4)` mét.

```bash
cmake -S tools/octomap -B build/octomap-tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/octomap-tools -j4
build/octomap-tools/build_octomap_world \
  --world simulation/worlds/forest.sdf --output state/simulation/maps \
  --min -4 -12 1 --max 26 12 9 --resolution 0.5 --hidden-fraction 0 --seed 7
```

## 2. Chép và build trên Orin, khi source thay đổi

Mac và Orin nối USB-C data, Mac `192.168.55.100`, Orin `192.168.55.1`.
Orin cần Docker + NVIDIA runtime đã cài, engine TensorRT phù hợp JetPack.
Không cài apt/pip/dependency lên host Orin; build tất cả trong Docker.

**Mac:**

```bash
ssh nvidia@192.168.55.1 'mkdir -p ~/uav_project/state/simulation/maps'
rsync -az --exclude build --exclude install --exclude log --exclude state \
  --exclude weights --exclude .git ./ nvidia@192.168.55.1:~/uav_project/
rsync -az state/simulation/maps/prior.bt \
  nvidia@192.168.55.1:~/uav_project/state/simulation/maps/
ssh nvidia@192.168.55.1
```

**Orin:**

```bash
cd ~/uav_project
# Máy này đã có base image; build CPU chỉ cần ở lần đầu hoặc khi Dockerfile.cpu đổi.
# docker build -f deploy/Dockerfile.cpu -t uav-navigation:humble .
docker build -f deploy/Dockerfile.gpu -t uav-navigation:humble-gpu .
docker build -f simulation/Dockerfile.build -t uav-navigation:simulation-build .
docker build -f simulation/Dockerfile -t uav-navigation:simulation .
```

## 3. Chạy từng terminal, theo thứ tự

Đóng Gazebo/SITL lượt cũ trước. Port 9002/5760/5762 phải trống. Mỗi lượt mới
đặt lại `UAV_RUN`; không ghi đè dữ liệu/EEPROM của lượt đang chạy.

**Mac terminal 1 — server Gazebo:**

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
export UAV_PROJECT="$PWD"
export GZ_PARTITION=uav_forest
export GZ_SIM_RESOURCE_PATH="$UAV_PROJECT/simulation/models:$UAV_PROJECT/simulation/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$UAV_PROJECT/build/simulation/plugins"
/opt/homebrew/bin/gz sim -s -r -v2 simulation/worlds/forest.sdf
```

**Mac terminal 2 — GUI**, cùng partition; không cần ROS:

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
export GZ_PARTITION=uav_forest
export GZ_SIM_RESOURCE_PATH="$PWD/simulation/models:$PWD/simulation/worlds"
/opt/homebrew/bin/gz sim -g -v2
```

**Mac terminal 3 — ArduPilot SITL:**

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
export UAV_PROJECT="$PWD"
export UAV_RUN="$UAV_PROJECT/state/simulation/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$UAV_RUN"
cd "$UAV_RUN"
"$UAV_PROJECT/state/simulation/bin/arducopter" -w --model JSON --speedup 1 \
  --slave 0 --serial1=tcp:2 --defaults "$UAV_PROJECT/simulation/config/sitl.parm" \
  --sim-address=127.0.0.1 -I0 --home=-35.363262,149.165237,584,0
```

**Mac terminal 4 — takeoff**, chờ `HOVER READY` trước bước tiếp theo.
Chưa khởi động Orin launch lúc takeoff: adapter phát lệnh hold có thể ngắt
chế độ takeoff. Tool này kiểm tra EKF ready, arm bình thường và hover ổn định;
không điều khiển đường bay, không force-arm.

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
/opt/miniconda3/envs/ardupilot-rviz/bin/python simulation/sitl_control.py takeoff --altitude 3
```

**Orin terminal 1 — product runtime**, sau `HOVER READY`:

```bash
cd ~/uav_project
mkdir -p state/simulation/output
docker run --rm --name uav-forest --runtime nvidia --network host \
  -e ROS_DOMAIN_ID=75 \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v /usr/lib/aarch64-linux-gnu/nvidia:/jetson-driver:ro \
  -v /home/nvidia/uav_deploy/depth_fp16.plan:/models/depth_fp16.plan:ro \
  -v "$PWD/state/simulation/maps/prior.bt:/maps/prior.bt:ro" \
  -v "$PWD/state/simulation/output:/state" \
  uav-navigation:simulation
```

Mount `/jetson-driver` cung cấp thư viện driver L4T của host (gồm
`libnvdla_compiler.so` mà TensorRT cần khi load), không ghi/cài gì lên host.

**Mac terminal 4 — phát RGB/state/clock**, sau khi Orin hiện ingress listening:

```bash
export GZ_PARTITION=uav_forest
build/simulation/gazebo_sender 192.168.55.1 7447
```

**Orin terminal 2 — kiểm tra, rồi gửi đúng một goal:**

```bash
docker exec uav-forest bash -c \
  'source /opt/ros/humble/setup.bash; ros2 topic echo /mavros/state --once'
docker exec uav-forest bash -c \
  'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic hz /perception/obstacles_camera'
# Lệnh hz tự dừng sau 8 s; timeout có thể trả mã 124. Tiếp tục gửi goal:
docker exec uav-forest bash -c \
  'source /opt/ros/humble/setup.bash; ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 20.0, y: 4.0, z: 4.0}, orientation: {w: 1.0}}}"'
```

Tất cả process trong `docker exec` thừa hưởng domain 75 từ container.
Các topic cần thấy: `/camera/image_rect_color`, `/camera/camera_info`,
`/iris/odometry`, `/localization/odometry`, `/planning/global_path`,
`/planning/mppi/predicted_path`, `/control/safe_velocity_command`.
Theo dõi `/diagnostics` và `/perception/monocular_depth/diagnostics` để biết
hold, stale, deadline hay đang chạy. Có topic/lệnh không có nghĩa đạt đích.

### Nếu UAV vẫn hover

- Sau takeoff, UAV giữ vị trí cho đến khi nhận `/goal_pose`. Khi chưa gửi goal,
  diagnostics của local navigation báo `WAITING_FOR_PATH: no global path received`.
- `ros2 topic hz` chạy liên tục nếu không có `timeout` hoặc Ctrl+C; khi copy các
  lệnh vào cùng terminal, phải kết thúc lệnh này để chạy tiếp bước gửi goal.
- Sau khi gửi goal, cần thấy log `Static A* attempt=...` và `accepted global path`.
  A* dùng prior cố định, chạy theo `replan_rate_hz: 5.0`; depth chỉ vào MPPI.
  Nếu A* thất bại vì state không đủ clearance, nó phát path rỗng và MPPI giữ UAV.
  Các lượt tìm lại đường qua cùng voxel không publish lại để tránh reset MPPI.
- `MAVROS FCU state is stale` là một chặn riêng ở adapter: chưa được chuyển
  lệnh khi heartbeat cũ. Kiểm tra link/telemetry, giữ giới hạn timeout hiện có.

## 4. Dừng

**Mac terminal mới — LAND trước:**

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
/opt/miniconda3/envs/ardupilot-rviz/bin/python simulation/sitl_control.py land
```

Chờ disarmed, rồi **Orin**:

```bash
docker stop -t 15 uav-forest
```

Sau đó Ctrl+C sender, GUI, SITL và Gazebo ở các terminal tương ứng.
Nếu sender đứt kết nối, dừng launch rồi mới khởi động lại; không tiếp tục
với clock cũ. Nhánh hiện tại không cập nhật hoặc xuất map từ camera.
Folder `state/` và binary SITL không đưa vào Git.

## 5. Chỉnh thêm ca

- **Đổi đích:** đổi XYZ trong lệnh `/goal_pose`; phải nằm trong bounds map,
  có clearance và đường free. Giữ một goal cho mỗi lượt đầu tiên.
- **Prior thiếu:** tạo map vào folder mới với `--hidden-fraction 0.6 --seed 11`,
  rsync đúng `prior.bt`, thay mount. Phần bị giấu được assumed-free; chưa phải
  policy unknown-space. Với nhánh map cố định hiện tại, camera chỉ giúp MPPI
  tránh cục bộ; A* không biết những vật cản bị giấu và có thể tiếp tục chọn đường bị chặn.
- **Đổi nơi xuất phát:** sửa pose của include Iris cuối `worlds/forest.sdf`,
  giữ spawn Z=0.35; map/state vẫn cùng world origin, không trừ offset spawn.
- **Tốc độ/horizon:** profile demo nằm `simulation/config/runtime.yaml`;
  giữ 0.5 m/s ban đầu. Sửa profile rồi rebuild image simulation.
- **Đổi world:** giữ đúng model/camera, sửa world đầu vào Gazebo và map exporter,
  đổi bounds trong profile cho khớp rồi tạo lại prior.

Đây là rừng primitive/texture hiện có, chưa phải cảnh quang thực. Bản setup
không tự chạy campaign hoặc bảo đảm thành công trên mọi tuyến.

## Kiểm tra setup ngày 2026-10-15

### Gazebo UI + RViz bằng một lệnh

```bash
cd /Users/nguyenthanhlam/Projects/ardupilot_gazebo/uav_project
UAV_GUI=1 UAV_RVIZ=1 UAV_DURATION=120 bash scripts/run_forest.sh
```

RViz trên Mac dùng `simulation/config/forest.rviz`:

- Vàng: global path A*; cyan: dự đoán MPPI; trắng: đường bay thực tế.
- Điểm cam: vật cản từ Depth Anything. Rừng xanh/nâu và tường xám là geometry
  tham chiếu của mô phỏng, không phải map dựng từ camera. Đích màu hồng.
- Bảng trạng thái trong cảnh hiển thị MPPI, adapter và depth. Nếu MPPI `ACTIVE`
  nhưng adapter `MAVROS FCU state is stale`, lệnh bị chặn ở adapter.
- Ảnh RGB chỉ được chuyển cho RViz ở tối đa 2 Hz, point cloud 5 Hz;
  các giới hạn này không đổi tần suất camera, depth hay planner gốc.

`simulation/visualization/start_mac.sh` mở tunnel SSH loopback 7448 và RViz.
`relay_server.py` subscribe trong Docker Orin; `relay_client.py` phát bản sao
topic trong ROS domain 175 trên Mac. Không chuyển goal hoặc lệnh điều khiển
từ RViz về Orin, không đổi cấu hình DDS/mạng của runtime domain 75.
Chỉ dùng các ROS message chuẩn; receiver kiểm tra độ dài dữ liệu trước giải nén.
Wrapper dùng môi trường ROS/RViz hiện có ở `/opt/miniconda3/envs/ardupilot-rviz`;
đổi bằng `UAV_MAC_ROS_PREFIX` nếu cần. Không cài thêm dependency trên Orin host.

Tắt RViz bằng `UAV_RVIZ=0`, tắt riêng Gazebo UI bằng `UAV_GUI=0`.
Log thêm ở folder lượt chạy: `rviz.log`, `rviz_relay.log`, `visualization.json`
và `visualization_events.jsonl`, `orin/output/visualization_server.log`.
`visualization.json` ghi số message
nhận được, subscriber RViz và trạng thái các node để phân biệt thiếu dữ liệu
hiển thị với việc bộ điều khiển thực sự dừng. Ctrl+C vẫn LAND rồi đóng các process.

Trong kiểm tra trên Mac hiện tại, RViz còn in `mutex lock failed` khi nhận tín
hiệu đóng cửa sổ; relay Python đã xử lý shutdown riêng. Runner vẫn xác nhận
LAND/disarm và dừng container. Lỗi đóng viewer này không phải nguyên nhân UAV
dừng giữa lượt; xem `rviz.log` riêng với diagnostics MPPI/adapter.

### Prior cố định + A* 5 Hz + depth chỉ cho MPPI

Chạy cùng ca rừng và goal `(20,4,4)`, giới hạn 0,5 m/s:

```bash
UAV_GUI=1 UAV_DURATION=120 bash scripts/run_forest.sh
```

Phiên bản này đã chạy trên Mac + Orin GPU qua USB: tới goal sau 80,9 s,
sai số vị trí cuối 0,19 m; 405 lượt A* trong khoảng 80,9 s, đạt 5,00 Hz.
Các lượt tìm đường thành công: trung vị 6,79 ms, P95 24,62 ms; tạo cache ban đầu 607 ms.
Node A* không subscribe cloud. MPPI vẫn nhận cloud Depth Anything; thông số MPPI giữ nguyên.
Chỉ publish đường khi chuỗi voxel thay đổi để hạn chế reset controller.
Đã LAND/disarm và dừng container sau lượt chạy.

Đây là một lượt tới đích, chưa phải kiểm chứng an toàn: có 9 lượt A* thất bại
tạm thời do state không đủ clearance, hai lần path bị thu hồi sau khởi tạo,
và vẫn có cảnh báo telemetry MAVROS. Không tăng tốc từ kết quả đơn lẻ này.

Các kiểm tra dưới đây là lịch sử trước khi đổi sang prior cố định.

- Copy `simulation/` và binary SITL sang `/tmp/uav_project_standalone_20261015`,
  build plugin/sender mới tại đó; Gazebo chỉ dùng resource/plugin của bản copy.
  Server, cất cánh và hover hoạt động, không đọc source/model/build của repo cha.
- Test transport (packet phân mảnh, mất kết nối, timestamp, JPEG) và ba test
  OctoMap exporter đều pass. Source/config text mới đều không quá 400 dòng.
- Docker Orin build đủ ba package product + simulator ingress. Đã sửa namespace
  MAVROS bị lặp và bổ sung mount driver/biến NVIDIA cho GPU.
- Quan sát 16 s qua USB: nhận 79 cloud, 16 MPPI predicted path, 35 setpoint
  khác 0; FCU báo connected/armed/GUIDED thật. A* xuất route 45–46 điểm.
  Vị trí GT đổi từ khoảng `(0.007,-0.010,3.190)` sang `(1.204,0.259,3.550)` m.
- Có `INVALID_INPUT`/hold/stale trong lượt này; **chưa xác nhận đạt đích**.
  Lượt kiểm tra đồng thời có Docker build, không dùng làm benchmark tốc độ.
  UAV giả lập đã LAND/disarm và process/container thử được dừng.

Log và JSON kiểm tra nằm trong `state/simulation/` (không đưa vào Git).

Runner một lệnh đã được chạy 15 s trên Mac không GUI + Orin GPU qua USB:
`state/simulation/run_20261015_153241_39928/`. Nhận 101 cloud, 381 state,
4 MPPI predicted path và 7 lệnh khác 0; A* ban đầu có 46 điểm. Sau đó
replanner hold vì `Unconfirmed camera hazard on route`, chưa đạt goal.
Runner đã LAND/disarm, lấy `summary.json`/map/log về Mac và dừng các process.
Đã kiểm tra riêng shutdown SIGINT trong container cô lập, không nối SITL.

Sau sửa resize đầu ra depth từ bicubic sang bilinear, lượt
`run_20261015_160154_92498` (15 s sau goal, không GUI) dịch ngang 3,52 m,
có 48 predicted path và 97 lệnh khác 0. Cuối lượt navigation `ACTIVE`, adapter
`STREAMING`; đã LAND/disarm và dừng container/process. Chưa xác nhận đạt goal.
