# Chuẩn bị OctoMap từ Gazebo SDF

CLI C++ độc lập, không phải ROS node. Build chỉ dùng source trong project này,
OctoMap, TinyXML2, CMake và pkg-config; không cần chạy ROS/Gazebo.
Mac: `brew install octomap tinyxml2 cmake pkg-config` nếu chưa có dependency.
Ubuntu: `sudo apt-get install build-essential cmake pkg-config liboctomap-dev libtinyxml2-dev`.
Không cần cài các dependency chuẩn bị map lên UAV nếu tạo map trên Mac.

Từ root `uav_project`:

```bash
bash scripts/prepare_octomap.sh \
  --world /absolute/path/world.sdf --output state/maps/my_world \
  --min -30 -30 0 --max 30 30 15 --resolution 0.5
```

- `full.bt`: map đầy đủ trong ROI.
- `prior.bt`: map đầu vào cho A*, mặc định giống full.
- Có thể thêm `--hidden-fraction 0.25 --seed 17` để giấu một phần collision
  khỏi prior. Chọn bằng hash tên collision và seed, tái lập giữa các lần chạy;
  fraction là xác suất chọn, không bảo đảm đúng 25% trên world nhỏ.
- Vùng được giấu là **assumed free**, không phải unknown. Full luôn giữ đủ
  hình học được hỗ trợ; không đưa full cho planner khi đang kiểm tra prior thiếu.
- Mỗi lần chạy ghi lại `full.bt`/`prior.bt` trong output directory đã chọn.

`--min/--max` phải khớp vùng nhiệm vụ `map_min/map_max` trong runtime YAML.
Các biên phải là bội số của resolution; tối đa 5 triệu voxel trong ROI.
Tọa độ giữ nguyên world XYZ, đơn vị mét; không trừ vị trí spawn UAV.
Không tự đọc hoặc sửa `runtime.yaml`, không nhúng start/goal vào `.bt`.

Hỗ trợ static inline model, collision box/cylinder/sphere, model/link/collision
pose chỉ có yaw, radian. Voxelization có bù nửa voxel để bảo thủ ở biên.
Không dùng visual/texture. Model động và include `model://iris_with_*` được bỏ qua.
Mesh, plane, named frames, relative_to, roll/pitch, nested model và các include
khác phải được chuyển thành collision primitives inline trước; tool báo lỗi
thay vì âm thầm xuất vùng đó là free. Đây không phải converter SDF tổng quát.

Dùng trên Orin với Docker wrapper hiện tại:

```bash
export UAV_MAP_FILE=/absolute/path/prior.bt
```

Wrapper mount file thành `/maps/prior.bt`. Build/run runtime vẫn là 5 node.
File chính để đọc: `build_octomap_world.cpp` (CLI + voxelization + ghi map),
`sdf_geometry.cpp` (đọc SDF và biến đổi tọa độ).

Kiểm tra phần mềm sau khi build:

```bash
ctest --test-dir build/octomap-tools --output-on-failure
```
