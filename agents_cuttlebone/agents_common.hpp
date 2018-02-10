#ifndef __COMMON__
#define __COMMON__

#include "allocore/io/al_App.hpp"
using namespace al;

// Common definition of application state
//
struct State {
  Vec3f particle_position;
  int index;

  Vec3f target_position;

  // vector<Particle> particle_list;
};

#endif
