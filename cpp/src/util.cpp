#include "camintrinsics/util.hpp"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>

#include <cstdarg>
#include <cstdio>
#include <iostream>

namespace ci {

std::string fmt(const char* format, ...) {
  char buf[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  return std::string(buf);
}

bool writeImage(const std::string& path, const cv::Mat& img) {
  try {
    // Match the Python twin: a default --out into data/generated/ must work
    // from a fresh checkout, not fail because the directory is missing.
    const std::filesystem::path parent =
        std::filesystem::absolute(path).parent_path();
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (cv::imwrite(path, img)) {
      std::cout << "wrote " << path << "  (" << img.cols << "x" << img.rows
                << ")\n";
      return true;
    }
    std::cerr << "could not write " << path
              << " -- does the directory exist and is it writable?\n";
  } catch (const cv::Exception& e) {
    std::cerr << "could not write " << path << " -- " << e.what() << "\n";
  }
  return false;
}

}  // namespace ci
