#include "uav_navigation_ros/depth_predictor.hpp"
#include "uav_navigation_core/metric_depth.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#ifdef UAV_DEPTH_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace uav_navigation_ros {
namespace {
constexpr int kCameraWidth = 640, kCameraHeight = 360;
constexpr int kInputWidth = 518, kInputHeight = 294;
constexpr std::size_t kDepthElements = kInputWidth * kInputHeight;

// One preprocessing and resizing path keeps ONNX/TensorRT camera geometry equal.
std::vector<float> PrepareInput(const std::uint8_t *image, int width, int height,
                               std::size_t row_step, bool bgr_input) {
  if (width != kCameraWidth || height != kCameraHeight)
    throw std::invalid_argument("depth predictor requires 640x360 RGB/BGR images");
  return uav_navigation_core::PrepareDepthAnythingInput(
      image, width, height, row_step, bgr_input, kInputWidth, kInputHeight);
}

std::vector<float> ResizeDepth(const float *depth, int width, int height) {
  cv::Mat small(kInputHeight, kInputWidth, CV_32F, const_cast<float *>(depth));
  cv::Mat full;
  // Linear interpolation stays within its input samples. Cubic undershoot at
  // tree/sky boundaries created 0.77 m obstacles from roughly 5 m native depth.
  // This is depth postprocessing; RGB preprocessing remains unchanged.
  cv::resize(small, full, cv::Size(width, height), 0.0, 0.0, cv::INTER_LINEAR);
  return {full.ptr<float>(), full.ptr<float>() + full.total()};
}

// Backend classes are private; callers only depend on IDepthPredictor.
class OnnxDepthPredictor final : public IDepthPredictor {
public:
  OnnxDepthPredictor(const std::string &path, const std::string &backend)
      : network_(cv::dnn::readNetFromONNX(path)) {
    if (network_.empty()) throw std::runtime_error("ONNX depth model is empty");
    if (backend == "cpu") {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } else {
      const auto target = backend == "cuda_fp16" ? cv::dnn::DNN_TARGET_CUDA_FP16
                                                  : cv::dnn::DNN_TARGET_CUDA;
      const auto targets = cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA);
      if (std::find(targets.begin(), targets.end(), target) == targets.end())
        throw std::runtime_error("requested OpenCV DNN CUDA target is unavailable");
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      network_.setPreferableTarget(target);
    }
  }

  std::vector<float> Predict(const std::uint8_t *image, int width, int height,
                            std::size_t row_step, bool bgr_input) override {
    const auto tensor = PrepareInput(image, width, height, row_step, bgr_input);
    const int shape[]{1, 3, kInputHeight, kInputWidth};
    cv::Mat blob(4, shape, CV_32F);
    std::copy(tensor.begin(), tensor.end(), blob.ptr<float>());
    network_.setInput(blob);
    const cv::Mat predicted = network_.forward();
    if (predicted.type() != CV_32F || predicted.total() != kDepthElements ||
        !predicted.isContinuous())
      throw std::runtime_error("unexpected depth output; expected 294x518 float depth");
    return ResizeDepth(predicted.ptr<float>(), width, height);
  }

private:
  cv::dnn::Net network_;
};

#ifdef UAV_DEPTH_TENSORRT
void CheckCuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
}

std::size_t Elements(const nvinfer1::Dims &dims) {
  std::size_t elements = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("TensorRT engine requires fixed I/O shapes");
    elements *= static_cast<std::size_t>(dims.d[i]);
  }
  return elements;
}

class TensorRtDepthPredictor final : public IDepthPredictor {
public:
  explicit TensorRtDepthPredictor(const std::string &path) : small_(kDepthElements) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open TensorRT engine: " + path);
    const std::vector<char> plan{std::istreambuf_iterator<char>(input),
                                 std::istreambuf_iterator<char>()};
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(plan.data(), plan.size()));
    if (!engine_) throw std::runtime_error("deserializeCudaEngine failed");
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("createExecutionContext failed");
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char *name = engine_->getIOTensorName(i);
      if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT)
        throw std::runtime_error("TensorRT engine must have FP32 I/O");
      switch (engine_->getTensorIOMode(name)) {
      case nvinfer1::TensorIOMode::kINPUT:
        if (!input_name_.empty()) throw std::runtime_error("expected one engine input");
        input_name_ = name;
        break;
      case nvinfer1::TensorIOMode::kOUTPUT:
        if (!output_name_.empty()) throw std::runtime_error("expected one engine output");
        output_name_ = name;
        break;
      default: throw std::runtime_error("invalid TensorRT tensor mode");
      }
    }
    if (input_name_.empty() || output_name_.empty() ||
        Elements(context_->getTensorShape(input_name_.c_str())) != 3 * kDepthElements ||
        Elements(context_->getTensorShape(output_name_.c_str())) != kDepthElements)
      throw std::runtime_error("unexpected depth engine shape; expected 1x3x294x518 input");
    try {
      CheckCuda(cudaMalloc(&input_buffer_, 3 * kDepthElements * sizeof(float)), "cudaMalloc input");
      CheckCuda(cudaMalloc(&output_buffer_, kDepthElements * sizeof(float)), "cudaMalloc output");
      CheckCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
      if (!context_->setTensorAddress(input_name_.c_str(), input_buffer_) ||
          !context_->setTensorAddress(output_name_.c_str(), output_buffer_))
        throw std::runtime_error("setTensorAddress failed");
    } catch (...) {
      ReleaseDeviceMemory();
      throw;
    }
  }

  ~TensorRtDepthPredictor() override { ReleaseDeviceMemory(); }
  TensorRtDepthPredictor(const TensorRtDepthPredictor &) = delete;
  TensorRtDepthPredictor &operator=(const TensorRtDepthPredictor &) = delete;

  std::vector<float> Predict(const std::uint8_t *image, int width, int height,
                            std::size_t row_step, bool bgr_input) override {
    const auto tensor = PrepareInput(image, width, height, row_step, bgr_input);
    CheckCuda(cudaMemcpyAsync(input_buffer_, tensor.data(), tensor.size() * sizeof(float),
                              cudaMemcpyHostToDevice, stream_), "H2D");
    if (!context_->enqueueV3(stream_)) throw std::runtime_error("enqueueV3 failed");
    CheckCuda(cudaMemcpyAsync(small_.data(), output_buffer_, small_.size() * sizeof(float),
                              cudaMemcpyDeviceToHost, stream_), "D2H");
    CheckCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    return ResizeDepth(small_.data(), width, height);
  }

private:
  void ReleaseDeviceMemory() noexcept {
    if (stream_) cudaStreamDestroy(stream_);
    if (output_buffer_) cudaFree(output_buffer_);
    if (input_buffer_) cudaFree(input_buffer_);
    stream_ = nullptr;
    output_buffer_ = input_buffer_ = nullptr;
  }

  struct Logger final : nvinfer1::ILogger {
    void log(Severity severity, const char *message) noexcept override {
      if (severity <= Severity::kWARNING) std::cerr << "TensorRT: " << message << '\n';
    }
  } logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string input_name_, output_name_;
  void *input_buffer_{nullptr}, *output_buffer_{nullptr};
  cudaStream_t stream_{nullptr};
  std::vector<float> small_;
};
#endif
} // namespace

std::unique_ptr<IDepthPredictor> MakeDepthPredictor(
    const std::string &model_path, const std::string &backend) {
  if (backend != "auto" && backend != "cpu" && backend != "cuda" &&
      backend != "cuda_fp16" && backend != "tensorrt")
    throw std::invalid_argument("depth backend must be auto, cpu, cuda, cuda_fp16 or tensorrt");
  auto extension = std::filesystem::path(model_path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const bool onnx = extension == ".onnx";
  const bool engine = extension == ".plan" || extension == ".engine";
  if (!onnx && !engine)
    throw std::invalid_argument("unsupported depth weights; use .onnx, .plan or .engine");
  const auto selected = backend == "auto" ? (onnx ? "cpu" : "tensorrt") : backend;
  if ((onnx && selected == "tensorrt") || (engine && selected != "tensorrt"))
    throw std::invalid_argument("depth backend does not match model file format");
  if (!std::filesystem::is_regular_file(model_path))
    throw std::invalid_argument("depth model file does not exist: " + model_path);
  if (selected == "tensorrt") {
#ifdef UAV_DEPTH_TENSORRT
    return std::make_unique<TensorRtDepthPredictor>(model_path);
#else
    throw std::runtime_error("TensorRT backend was not built into this image");
#endif
  }
  return std::make_unique<OnnxDepthPredictor>(model_path, selected);
}
} // namespace uav_navigation_ros
