# Monocular UAV navigation

Runtime C++17/ROS 2 Humble cho Orin:
```text
Prior OctoMap cố định → A* 3D → global path ──┐
                 └── vùng hợp lệ / phanh ────┤
RGB → Depth Anything → point cloud vật cản ──┴→ MPPI → MAVROS → ArduPilot
```

A* lập lại đường ở **5 Hz theo thời gian thực**, dùng cache clearance trên prior.
Depth chỉ cung cấp vật cản cho MPPI, **không cập nhật map hoặc thu hồi path A***.
A* vẫn tính ứng viên ở 5 Hz; giữ path hiện hành nếu UAV còn nối vào được trong
vùng hợp lệ và cách path không quá `route_retention_distance_m` (0,75 m).
MPPI kiểm tra cứng cả rollout lẫn đoạn phanh trên prior, đồng thời giữ kiểm tra cloud.
`navigation/prior_region.cpp` nạp cùng prior/ROI/clearance với A*; thiếu map thì node không khởi động.
Phanh 3D dùng `safety.braking_acceleration_m_s2`, không vượt giới hạn của trục yếu nhất.
Prior phải có bounds khớp lưới voxel; vật cản thiếu trong prior chỉ được tránh cục bộ.

| Folder | Nội dung |
|---|---|
| `src/uav_navigation_core` | MPPI, dynamics, safety, OctoMap/A* 3D, camera geometry |
| `src/uav_navigation_ros` | Năm node runtime, interface `.hpp/.cpp` |
| `src/uav_navigation_bringup` | Launch và `config/runtime.yaml`: topic, giới hạn, geofence, camera extrinsics |
| `deploy` | Build/run Docker GPU trên Orin |
| `scripts` | Build/test, chuẩn bị map và chạy một ca mô phỏng |
| `simulation` | Setup rừng độc lập: world/model/plugin, bridge C++, SITL config và Docker; [từng lệnh chạy](simulation/README.md) |
| `tools/octomap` | CLI C++ chuẩn bị `full.bt`/`prior.bt` từ collision geometry SDF |

Mở `uav_project.code-workspace`. Model weights để ngoài source control.
`test/` chỉ chứa unit test phần mềm. Demo rừng được để riêng trong `simulation/`,
không đưa vào ba package runtime.

Chạy một ca rừng Mac + Orin GPU qua USB, kèm Gazebo GUI và RViz trên Mac:
`bash scripts/run_forest.sh`. Script build/sync, takeoff, chạy pipeline, gửi goal,
quan sát 60 s rồi LAND và thu log. Xem [hướng dẫn](simulation/README.md) và
[sơ đồ gọi file để mở trong Excalidraw](docs/architecture/forest_run.excalidraw).
Để quan sát tối đa 180 s: `UAV_DURATION=180 UAV_RVIZ=1 bash scripts/run_forest.sh`.
Tắt riêng từng cửa sổ bằng `UAV_GUI=0` (Gazebo) hoặc `UAV_RVIZ=0` (RViz).
RViz có path A* vàng, MPPI cyan, đường bay thực trắng, điểm depth cam và trạng thái
MPPI/adapter. Kênh xem chỉ truyền Orin → Mac qua SSH, ROS domain 175 tách khỏi runtime.

Các node có launch riêng, được ghép trong `app.launch.xml`. Có thể bật/tắt
từng thành phần bằng `start_*:=false` hoặc thay launch bằng `*_launch_file`;
xem [launch và thay node](src/uav_navigation_bringup/README.md).

Chuẩn bị prior từ Gazebo world bằng công cụ độc lập trong project:

```bash
bash scripts/prepare_octomap.sh \
  --world /absolute/path/world.sdf --output state/maps/my_world \
  --min -30 -30 0 --max 30 30 15 --resolution 0.5
```

Xuất `full.bt` và `prior.bt`; mặc định prior đầy đủ. Bounds phải khớp runtime.
Tùy chọn giấu collision, dependency và giới hạn SDF: [tools/octomap](tools/octomap/README.md).
Đây là bước chuẩn bị offline, không thêm ROS node hoặc dependency vào runtime.

## Input/output

| Input/output | Contract |
|---|---|
| Camera | `/camera/image_rect_color`, RGB/BGR uint8 **640×360**, đã rectify, timestamp lúc chụp |
| Calibration | `/camera/camera_info`, cùng kích thước và optical frame với ảnh; K/P hợp lệ |
| Định vị | `/localization/odometry`: pose `map` ENU, twist `base_link` FLU; mặc định lấy Gazebo GT từ `/iris/odometry` |
| Goal | `/goal_pose`, `geometry_msgs/PoseStamped`, XYZ **mét**, frame `map` |
| Prior map | OctoMap `.bt`, cùng gốc/hướng ENU với nguồn định vị đang chọn |
| Output | `/mavros/setpoint_velocity/cmd_vel`, vận tốc ENU và yaw rate |
| Diagnostics | `/diagnostics`, `/perception/monocular_depth/diagnostics` |

Camera driver là thành phần phụ thuộc phần cứng: USB/CSI có thể dùng driver ROS
khác nhau, miễn xuất đúng contract trên cùng `ROS_DOMAIN_ID`. Project không
khởi động camera driver cụ thể hay giả định camera giống trong Gazebo.

Chỉnh `src/uav_navigation_bringup/config/runtime.yaml` cho UAV: topic/frame, `camera_translation_m`,
`camera_quaternion_xyzw`, `map_min/map_max`, giới hạn vận tốc và khoảng hở.
Camera cloud dùng ROS optical axes (x phải, y xuống, z trước); extrinsics được
áp dụng bằng TF `map → base_link → camera_optical_frame`.
Giá trị mount mặc định minh họa camera nhìn thẳng, offset bằng 0; cần thay bằng
calibration thực tế. Không dùng FOV hoặc offset camera của mô phỏng.

Trong thí nghiệm hiện tại, **Gazebo ground truth đóng vai trò GPS/state lý tưởng**:
`/iris/odometry → odometry_tf → /localization/odometry → A* và MPPI`.
`odometry_tf` cũng phát TF `map → base_link` để đổi point cloud camera sang frame của MPPI.
`Odometry` là kiểu message ROS chứa pose/vận tốc, không có nghĩa dùng visual odometry.
Depth Anything vẫn chỉ nhận RGB; không dùng depth GT để sửa dự đoán.

Setup Mac → Orin có sẵn trong [simulation/README.md](simulation/README.md),
chạy chính `app.launch.xml` này và MAVROS thật, không gọi runtime repo cha.
Nếu tự dùng bridge khác, contract là ROS `/iris/odometry`, pose frame
`odom`, child `base_link`, và `/clock`. `odom` trong mô phỏng phải cùng gốc/hướng
world ENU với `map` và prior `.bt`; node chỉ đổi nhãn frame, không ước lượng offset.
Không trừ vị trí spawn. Timestamp mô phỏng được giữ nguyên; mọi node dùng
`use_sim_time=true`. Không bridge TF Gazebo song song với `odometry_tf`.
Ví dụ bridge chạy trên máy có Gazebo và `ros_gz_bridge` (cùng ROS domain):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/iris/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Gazebo và MAVROS đều xuất twist trong trục thân; MPPI đổi sang ENU đúng một lần.
Theo [Gazebo OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)
và [MAVROS local_position](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/local_position.cpp).
MPPI nhận angular velocity trong odometry để tính **yaw rate đo được**, tách
khỏi yaw-rate command. Rollout dự đoán đáp ứng quay bằng `response.yaw_enabled`
và `response.yaw_tau_s` (mặc định runtime: bật, 0,5 s). Hằng số này cần được
nhận dạng/kiểm chứng khi thử bay; cost hướng bay hiện có được giữ nguyên.
Roll/pitch vẫn do FCU điều khiển. Diagnostics có `measured_yaw_rate_rad_s`.

Không tự fallback về EKF khi Gazebo hết dữ liệu. MAVROS vẫn truyền lệnh vận tốc
và trạng thái FCU connected/armed/GUIDED; nó không cấp localization trong chế độ này.

Khi đưa lên UAV thật, chọn rõ `localization_source:=mavros use_sim_time:=false`
(hoặc `UAV_LOCALIZATION_SOURCE=mavros` với Docker wrapper). Khi đó input định vị
là `/mavros/local_position/odom`, pose `map` ENU và twist `base_link`.
Prior phải đăng ký lại với origin FCU. Goal là XYZ mét local ENU, không phải lat/lon.

## Build/run trên Orin

Chép folder source này lên Orin, rồi:

```bash
cd ~/uav_project
bash deploy/build_orin.sh
export UAV_DEPTH_ENGINE=/home/nvidia/uav_deploy/depth_fp16.plan
export UAV_MAP_FILE=/home/nvidia/maps/prior.bt
# SITL MAVLink endpoint; thay IP bằng địa chỉ máy chạy SITL mà Orin truy cập được.
export UAV_FCU_URL='tcp://192.168.55.100:5762'
export UAV_LOCALIZATION_SOURCE=gazebo
ROS_DOMAIN_ID=75 bash deploy/run_orin.sh
```

Image: `uav-navigation:humble-gpu`. Model/prior mount read-only;
ROS logs lưu trong `state/`. Chỉ build/cài dependency bên trong Docker.
GPU image dùng JetPack userspace `r36.4.0` theo recipe hiện có; phải khớp ABI
TensorRT/CUDA với host và engine `.plan` trên Orin. Có thể đổi
`JETPACK_IMAGE` qua Docker build arg khi chuẩn bị runtime tương ứng.
Runner mount read-only thư viện driver từ `/usr/lib/aarch64-linux-gnu/nvidia`
vào `/jetson-driver` để TensorRT tìm thấy thư viện L4T như `libnvdla_compiler.so`;
carrier/rootfs khác có thể đặt `UAV_JETSON_DRIVER_DIR`.

Nếu đã có MAVROS ngoài container, đặt `UAV_START_MAVROS=false`. MAVROS ngoài
container cần cùng ROS domain và frame/topic contract. FCU serial hoặc UDP
được cấu hình qua `UAV_FCU_URL`; UART device chỉ được mount khi dùng serial.

Gửi goal bằng terminal đã có ROS và cùng `ROS_DOMAIN_ID`:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}'
```

Goal phải nằm trong vùng plan và có đường known-free trong prior cố định.
Camera chỉ cung cấp cloud cho local planner. A* và bộ safety của MPPI dùng
cùng prior; `map_min`, `map_max`, `clearance_m` ở hai node phải khớp nhau.

Autopilot adapter chỉ chuyển lệnh khi FCU connected, armed và ở `GUIDED`.
Project không tự arm, takeoff hoặc chuyển flight mode. Khi lệnh hết hạn sau khi
đã điều khiển, adapter gửi velocity 0 nếu FCU còn healthy/GUIDED. Mất FCU link
phải do failsafe đã cấu hình trên flight controller xử lý.

## Workspace và build

Cấu trúc source gồm ba package ROS 2; launch/config được đóng gói trong bringup.
Build chuẩn bằng colcon tạo `build/`, `install/`, `log/` ở workspace root:

```bash
# Trong bash, sau khi source môi trường ROS 2 phù hợp:
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv5
colcon test --packages-select uav_navigation_core uav_navigation_ros
colcon test-result --verbose
source install/setup.bash
ros2 launch uav_navigation_bringup app.launch.xml
```

Launch mặc định cần model, prior map, RGB/CameraInfo, Gazebo state + clock và FCU theo contract phía trên.
Trên Mac, script chọn ROS conda và OpenCV Homebrew, build cả ba package rồi test:

```bash
bash scripts/build.sh
```

Nếu Mac chưa có colcon, cài công cụ build vào môi trường riêng (không sửa ROS conda):

```bash
/opt/miniconda3/envs/ardupilot-rviz/bin/python -m venv --system-site-packages \
  "$HOME/.cache/uav_project/colcon"
"$HOME/.cache/uav_project/colcon/bin/python" -m pip install colcon-common-extensions
```

Script tìm colcon trong PATH hoặc môi trường cache trên; có thể đặt `UAV_COLCON_BIN`.
Khi gọi colcon trực tiếp trên Mac, thêm cache đó vào PATH và dùng
`-DOpenCV_DIR=/opt/homebrew/lib/cmake/opencv5`.

Dùng ROS conda hiện có trên Mac cho compile/unit test; runtime triển khai là
Humble Docker. `bash scripts/build.sh --core-only` chỉ build core C++.
Khi chạy ảnh ROS trên Mac, dùng `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` cho các
process: kiểm tra ảnh lớn đã qua với middleware này.
Depth có một interface `IDepthPredictor` (`depth_predictor.hpp`) và một file
triển khai `depth_predictor.cpp`, gồm ONNX, TensorRT và factory.
Depth đầu ra được resize bằng bilinear trước khi chiếu sang 3D. Bicubic ở
bước này từng tạo undershoot tại biên cây/nền xa và sinh vật cản ảo rất gần;
preprocessing RGB vẫn giữ bộ resize/normalization của checkpoint.
`depth_backend:=auto` là mặc định: `.onnx` chọn OpenCV CPU, `.plan`/`.engine`
chọn TensorRT GPU. Ví dụ `model_file:=/models/depth.onnx` hoặc
`model_file:=/models/depth_fp16.plan`; node vẫn dùng cùng lời gọi `Predict()`.
Có thể chọn rõ `depth_backend:=cpu|cuda|cuda_fp16|tensorrt`.
Weight phải tương thích contract Depth Anything metric hiện tại: input float
NCHW 1×3×294×518, output depth 294×518 tính bằng mét với preprocessing hiện có.
Đổi sang kiến trúc/relative depth khác cần adapter phù hợp; `.pt`/`.pth` phải
export trước. Runtime không có PyTorch/Python thuật toán.

Đã kiểm tra trên Orin L4T R36.5.2: Docker Humble build đủ ba package ARM64,
smoke test ban đầu chạy TensorRT → OctoMap/A* → MPPI → adapter.
Bài kiểm tra phát lại cùng một ảnh RGB 100 lần ở 10 Hz, dùng calibration,
prior map và state EKF/FCU giả lập; nhận 99 cloud, map được cập nhật và có lệnh
MPPI. Đây là smoke test lịch sử; runtime hiện tại không cập nhật prior từ camera.
Watchdog dừng khi ảnh hết hạn và chặn output khi disarm đều đạt.

Cấu hình hiện giữ giới hạn 0,5 m/s và horizon 3 s. Chưa có xác nhận bay thật
hoặc kiểm tra toàn pipeline với camera/FCU vật lý trên UAV.
