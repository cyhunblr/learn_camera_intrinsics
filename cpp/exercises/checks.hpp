#pragma once

#include <functional>
#include <string>
#include <vector>

#include "exercises.hpp"

namespace exlab {

/// Thrown by a check when the answer is wrong, carrying a message that says
/// *what* went wrong rather than merely that something did.
struct CheckFailed : std::runtime_error {
  explicit CheckFailed(const std::string& what) : std::runtime_error(what) {}
};

struct Check {
  int number;
  const char* title;
  void (*run)(const Impl&);
};

const std::vector<Check>& checks();

}  // namespace exlab
