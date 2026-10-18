# Nguồn và dependency

- World `forest.sdf`, model RGB/Iris và tham số SITL được nhập từ snapshot
  dự án cũ ngày 2026-10-15. Các file này là bản sao trong project, không phải
  symlink hoặc đường dẫn runtime tới repo cũ.
- `forest_geometry.json` là proxy để đối chiếu offline; không mount vào runtime.
- Iris/gimbal mesh và plugin dựa trên ArduPilot/ardupilot_gazebo (Apache-2.0).
  Model metadata được giữ lại. License nằm `vendor/ardupilot_plugin/LICENSE`.
- Plugin lấy từ source local tại commit `2fce562716e3ff3ce4c3935e1551fbf5be3c1d2d`
  của repo cũ, gồm thay đổi có sẵn trước lần tách này (ví dụ FDM health logs).
  `ArduPilotPlugin.cc` được chia theo nhóm hàm, giữ hành vi:
  lifecycle, controls, sensors, updates, forces, servo, state. Các lớp private
  dùng chung nằm `plugin_internal.hpp`; đăng ký plugin chỉ ở lifecycle.
  Không kéo các plugin GStreamer, parachute hoặc zoom không dùng trong demo.
- XML model/mesh dài được gộp whitespace giữa tag; không đổi hình học/texture.
  Mỗi file source/config mới không quá 400 dòng.
- JPEG codec được nhập từ transport đã chạy thử trước đây; protocol simulator
  mới có magic riêng, không tương thích bridge experimental. Không có nhánh
  depth GT, planner, truyền lệnh MAVLink hoặc trạng thái FCU giả trong bridge.

Các dependency cài ngoài source: Gazebo Harmonic/Homebrew, ArduPilot SITL,
CMake/compiler, Qt5/RapidJSON/libjpeg, OctoMap/tinyxml2 và Python+pymavlink.
Trên Orin chỉ cần Docker/NVIDIA runtime + engine hiện có; thư viện ROS/compiler
bổ sung được cài trong Docker. Source ba package navigation nằm `src/`.

`state/simulation/bin/arducopter` là bản binary dependency local, không commit
vào Git và không phải firmware cho UAV thật. Máy mới cần tự cung cấp binary
SITL tương thích (lượt kiểm tra: ArduCopter V4.8.0-dev, commit f808f78c).
Source ArduPilot là dependency riêng, giống cách project
không đóng gói source Gazebo/ROS; khi phân phối binary phải giữ nghĩa vụ license
tương ứng của dependency. Không cần checkout repo experimental để chạy.
