#ifndef __COMMON__
#define __COMMON__

#include "allocore/io/al_App.hpp"
using namespace al;

#include <iostream>
#include <type_traits>
#include <vector>
#include "allocore/math/al_Random.hpp"
#include "allocore/math/al_Vec.hpp"
using namespace std;
using namespace al;

#define MAX (2000)
struct FakeVector {
  unsigned n;
  Vec3f stuff[MAX];

  void fill_stuff(const vector<Vec3f>& v) {
    n = v.size();
    for (unsigned i = 0; i < v.size(); ++i) stuff[i] = v[i];
  }
};

// Common definition of application state
//
struct State {
  FakeVector fishZeroPosComm;
  FakeVector planktonPosComm;
  Vec3f userFishPosition;
  Pose ghostNetComm;
  float parNearTargetTemp;

  // FakeVector colors;
  // vector<Color> particleColors;
  // vector<Pose> particlePoses;
};

#endif
