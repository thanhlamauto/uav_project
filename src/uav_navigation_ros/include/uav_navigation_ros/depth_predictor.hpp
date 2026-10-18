#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace uav_navigation_ros {
// RGB/BGR 640x360 uint8 input, camera axial depth in metres, same resolution.
// Both formats use the same Depth Anything metric preprocessing/output contract:
// float NCHW 1x3x294x518 input, one 294x518 axial-depth output in metres.
// Implementations retain ownership of ONNX/TensorRT state. No pose/map inputs.
class IDepthPredictor {
public:
  virtual ~IDepthPredictor() = default;
  virtual std::vector<float> Predict(const std::uint8_t *image, int width,
      int height, std::size_t row_step, bool bgr_input) = 0;
};
// auto: .onnx -> OpenCV CPU; .plan/.engine -> TensorRT. Explicit backends:
// cpu, cuda, cuda_fp16 (ONNX), tensorrt (serialized engine). No silent fallback.
std::unique_ptr<IDepthPredictor> MakeDepthPredictor(
    const std::string &model_path, const std::string &backend = "auto");
}
