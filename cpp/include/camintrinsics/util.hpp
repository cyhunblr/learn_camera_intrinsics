// Small helpers with no dependencies beyond the standard library.
#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace ci {

/// printf into a std::string -- used everywhere to build HUD lines and tables.
std::string fmt(const char* format, ...);

/// Write an image, reporting honestly. cv::imwrite returns false for a path it
/// cannot write and throws for an extension it does not know; either way the
/// caller must not go on to claim success.
bool writeImage(const std::string& path, const cv::Mat& img);

}  // namespace ci
