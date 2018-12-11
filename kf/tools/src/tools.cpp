#include "../include/tools.h"

double normalizeAngle(double angle) {
  return (std::fmod(angle + PI, 2 * PI) - PI);
}


