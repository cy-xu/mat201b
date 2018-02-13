#ifndef __COMMON__
#define __COMMON__

#include "allocore/io/al_App.hpp"
using namespace al;

// Common definition of application state
//
struct State {
  vector<Color> particleColors;
  vector<Vec3f> particlePositions;
  Vec3f targetPosition;
};

#endif
