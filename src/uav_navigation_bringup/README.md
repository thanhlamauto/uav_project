# Launch theo từng thành phần

`app.launch.xml` ghép các launch con bằng `include`. Mỗi launch con cũng chạy
độc lập được. Cấu hình mặc định giữ các node, topic, tên node và thuật toán hiện có.
Các include được đặt trong group có scope riêng để `node_package` /
`node_executable` của một thành phần không lan sang thành phần khác.

| Thành phần | Launch riêng | Bật/tắt trong app | Chọn launch thay thế |
|---|---|---|---|
| Localization + TF | `odometry_tf.launch.xml` | `start_localization` | `localization_launch_file` |
| Depth | `monocular_depth.launch.xml` | `start_depth` | `depth_launch_file` |
| OctoMap + A* | `octomap_route_replanner.launch.xml` | `start_global_planner` | `global_planner_launch_file` |
| MPPI + safety | `local_navigation.launch.xml` | `start_local_planner` | `local_planner_launch_file` |
| Adapter | `autopilot_adapter.launch.xml` | `start_adapter` | `adapter_launch_file` |
| MAVROS | `mavros.launch.xml` | `start_mavros` | `mavros_launch_file` |

Mặc định các thành phần đều bật. Launch dùng lại `config/runtime.yaml`;
`params_file` chọn profile khác. Tham số truyền trực tiếp qua launch có ưu tiên
cao hơn cùng tham số trong YAML.

## Chạy riêng một node

Các lệnh ROS dưới đây dùng trong workspace đã build và source ROS 2. Đường dẫn
`/models`, `/maps`, `/simulation` trong ví dụ là đường dẫn bên trong Docker Orin.
Chạy riêng depth:

```bash
ros2 launch uav_navigation_bringup monocular_depth.launch.xml \
  params_file:=/simulation/runtime.yaml model_file:=/models/depth_fp16.plan
```

Chạy riêng MPPI + safety:

```bash
ros2 launch uav_navigation_bringup local_navigation.launch.xml \
  params_file:=/simulation/runtime.yaml
```

Node vẫn cần các đầu vào của nó: local navigation cần state, path, cloud và TF.
Chỉ mở launch riêng không tự sinh dữ liệu hay arm/takeoff UAV.
Xem argument của một launch mà chưa chạy node:

```bash
ros2 launch uav_navigation_bringup monocular_depth.launch.xml --show-args
```

## Thay một thành phần trong cả hệ thống

Tạo launch mới, có thể copy launch tương ứng rồi sửa package/executable và
cấu hình cho node mới. Chọn file đó khi chạy app:

```bash
ros2 launch uav_navigation_bringup app.launch.xml \
  depth_launch_file:=/absolute/path/new_depth.launch.xml
```

App truyền các argument sau cho launch thay thế; file mới phải nhận chúng:

| Thành phần | Argument được truyền vào launch con |
|---|---|
| Localization | `params_file`, `use_sim_time`, `localization_source` |
| Depth | `params_file`, `use_sim_time`, `model_file`, `depth_backend` |
| Global planner | `params_file`, `use_sim_time`, `prior_map` |
| Local planner, adapter | `params_file`, `use_sim_time` |
| MAVROS | `fcu_url` |

Node mới phải cung cấp cùng contract message/topic/frame/timestamp/QoS cho
các thành phần còn lại (xem bảng input/output ở README root). Nếu node mới có
parameter khác, launch thay thế chịu trách nhiệm ánh xạ cấu hình cho nó.
Launch không tự chuyển đổi thuật toán, định dạng model hay message.

## Tắt node mặc định để chạy bản khác ở terminal riêng

```bash
ros2 launch uav_navigation_bringup app.launch.xml start_depth:=false
```

Ở terminal thứ hai đã source cùng workspace, cùng ROS domain:

```bash
ros2 launch uav_navigation_bringup monocular_depth.launch.xml \
  node_package:=my_depth_package node_executable:=my_depth_node \
  params_file:=/absolute/path/new_profile.yaml
```

`my_depth_package` / `my_depth_node` là ví dụ cho node mới đã build/install.
Năm launch runtime đều hỗ trợ `node_package` và `node_executable`; tên node
ROS vẫn cố định để khớp section trong YAML. Dùng một nguồn output cho mỗi
thành phần: tắt bản mặc định trước khi bật bản thay thế.

Nếu chỉ đổi weight/backend hiện được hỗ trợ, dùng `model_file` và
`depth_backend`; không cần tạo node khác.

## Dùng với mô phỏng

`uav_simulation/forest.launch.xml` chuyển tiếp các argument của app, gồm
`start_*` và `*_launch_file`. Ví dụ:

```bash
ros2 launch uav_simulation forest.launch.xml \
  depth_launch_file:=/absolute/path/new_depth.launch.xml
```

Receiver cũng có launch riêng `uav_simulation/gazebo_receiver.launch.xml`,
argument `bind_address`, `port`, `node_package`, `node_executable`.
Trong forest, dùng `start_receiver`, `receiver_launch_file`,
`receiver_bind_address`, `receiver_port` để chọn/tắt/cấu hình receiver.

## Build và cập nhật

Chỉ đổi launch/config của bringup, trong workspace đã source các dependency:

```bash
colcon build --base-paths src --packages-select uav_navigation_bringup
source install/setup.bash
```

`CMakeLists.txt` đã cài toàn bộ folder `launch/` và `config/`.
Với Docker, source thay đổi chưa tự cập nhật container/image cũ. Đồng bộ project
lên Orin rồi build lại image theo README root và `simulation/README.md`.
