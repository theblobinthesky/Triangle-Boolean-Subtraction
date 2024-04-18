#pragma once
#include "util.h"
#include <vector>
#include <glm/vec2.hpp>

// Returns in clockwise order.
void inplace_convex_hull(std::vector<glm::vec2>& pts);