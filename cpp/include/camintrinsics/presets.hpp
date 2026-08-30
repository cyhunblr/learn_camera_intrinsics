// Named lens presets and the on-screen summary of a camera model.
//
// Shared by the figure renderers here and by the web viewer in web/, so a lens
// called "action cam" means the same thing everywhere.
#pragma once

#include <string>
#include <vector>

#include "camintrinsics/intrinsics.hpp"

namespace ci {

struct Preset {
  const char* name;
  double f, k1, k2, p1, p2, k3;
};

/// Named starting points, roughly matching real hardware classes.
const std::vector<Preset>& presets();

/// Build (K, D) for a preset at the given image width.
void presetModel(const Preset& p, int width, int height, Mat33* K, Dist* D);

/// Human-readable summary of a camera model, for the figure HUD.
std::vector<std::string> kdHudLines(const Mat33& K, const Dist& D, int width,
                                    int height);

}  // namespace ci
