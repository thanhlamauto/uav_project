// RGB JPEG quality 85; imported from the tested simulator transport.
#pragma once
#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <jpeglib.h>
#include <vector>
namespace remote {
struct JpegError {
  jpeg_error_mgr base;
  jmp_buf jump;
};
inline void JpegFailure(j_common_ptr context) {
  longjmp(reinterpret_cast<JpegError *>(context->err)->jump, 1);
}
inline std::vector<unsigned char> EncodeRgb(const char *rgb) {
  jpeg_compress_struct context{};
  jpeg_error_mgr error{};
  context.err = jpeg_std_error(&error);
  jpeg_create_compress(&context);
  unsigned char *memory = nullptr;
  unsigned long length = 0;
  jpeg_mem_dest(&context, &memory, &length);
  context.image_width = 640;
  context.image_height = 360;
  context.input_components = 3;
  context.in_color_space = JCS_RGB;
  jpeg_set_defaults(&context);
  jpeg_set_quality(&context, 85, TRUE);
  jpeg_start_compress(&context, TRUE);
  while (context.next_scanline < context.image_height) {
    auto *row = reinterpret_cast<JSAMPLE *>(const_cast<char *>(rgb)) +
                context.next_scanline * 1920;
    jpeg_write_scanlines(&context, &row, 1);
  }
  jpeg_finish_compress(&context);
  std::vector<unsigned char> output(memory, memory + length);
  std::free(memory);
  jpeg_destroy_compress(&context);
  return output;
}
inline bool DecodeRgb(const std::vector<unsigned char> &encoded,
                      std::vector<unsigned char> &rgb) {
  jpeg_decompress_struct context{};
  JpegError error{};
  context.err = jpeg_std_error(&error.base);
  error.base.error_exit = JpegFailure;
  if (setjmp(error.jump)) {
    jpeg_destroy_decompress(&context);
    return false;
  }
  jpeg_create_decompress(&context);
  jpeg_mem_src(&context, encoded.data(), encoded.size());
  jpeg_read_header(&context, TRUE);
  if (context.image_width != 640 || context.image_height != 360) {
    jpeg_destroy_decompress(&context);
    return false;
  }
  context.out_color_space = JCS_RGB;
  jpeg_start_decompress(&context);
  rgb.resize(640 * 360 * 3);
  while (context.output_scanline < context.output_height) {
    auto *row = rgb.data() + context.output_scanline * 1920;
    jpeg_read_scanlines(&context, &row, 1);
  }
  jpeg_finish_decompress(&context);
  jpeg_destroy_decompress(&context);
  return true;
}
} // namespace remote
