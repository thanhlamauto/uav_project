# Planning core

C++17: MPPI, dynamics, safety, command conditioner và OctoMap/A* 3D.
Public interface ở `include/`; implementation ở `src/`.
Phụ thuộc OctoMap, không phụ thuộc ROS hay camera driver.

`test/test_runtime.cpp` kiểm tra camera geometry, frame convention và safety.
Build: `bash scripts/build.sh --core-only` từ project root.

`IFlightRegion` là ràng buộc vùng đã inflation, tách khỏi distance field.
`StaticOctomapPlanner::SegmentAllowed` kiểm tra mọi voxel cắt qua;
`TrajectorySafetyChecker` áp dụng cho rollout và từng đoạn phanh, không bỏ cloud.
`static_route.hpp` kiểm tra connector để giữ đường prior còn hợp lệ.
`test/test_prior_guard.cpp` kiểm tra swept collision, phanh, unknown và giữ route.
