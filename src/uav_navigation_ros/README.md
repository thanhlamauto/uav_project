# ROS 2 runtime

- `monocular_depth_node`: calibrated RGB → metric depth/cloud.
- `octomap_route_replanner_node`: prior cố định + localization + goal → A* 3D, 5 Hz; không nhận cloud.
- `local_navigation_node`: localization + path + cloud depth + prior guard → MPPI velocity.
- `odometry_tf_node`: Gazebo ideal state hoặc MAVROS ENU state → normalized odometry + TF; camera extrinsics.
- `autopilot_adapter_node`: safe velocity → MAVROS; connection/mode/watchdog.

Launch và cấu hình nằm trong package `uav_navigation_bringup`.
[Integration guide (English)](../../docs/INTEGRATION.md): build, Python launch and data contract.
Camera driver bên ngoài cung cấp rectified RGB 640×360 và CameraInfo.
MAVROS cung cấp odometry chuẩn: pose ENU, twist trong base_link.

Depth dùng một interface `IDepthPredictor` trong `include/uav_navigation_ros/depth_predictor.hpp`;
ONNX, TensorRT và factory được gộp trong `src/depth_predictor.cpp`.
Node chỉ gọi `MakeDepthPredictor(model_path, backend)` và `Predict(...)`.
`backend=auto`: `.onnx` chạy OpenCV CPU, `.plan`/`.engine` chạy TensorRT;
có thể chọn rõ `cpu`, `cuda`, `cuda_fp16` hoặc `tensorrt`.
Các weight phải theo contract Depth Anything metric: preprocessing hiện tại,
input float NCHW 1×3×294×518, output 294×518 depth dọc trục camera tính bằng mét.
Định dạng chung không tự chuyển relative/inverse depth sang mét hay hỗ trợ mọi kiến trúc.

`navigation/prior_region.cpp` nạp prior cho safety; không cập nhật map từ depth.
A* giữ route khi connector còn free và gần path; đổi goal hoặc lệch xa mới thay.
`prior_map_file`, ROI và clearance phải giống nhau ở global/local node.
