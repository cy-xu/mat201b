#ifndef __COMMON__
#define __COMMON__

#include <iostream>
#include <type_traits>
#include <vector>
#include "allocore/io/al_App.hpp"
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

struct FakePoseVector {
  unsigned n;
  Pose stuff[MAX];

  void fill_stuff(const vector<Pose>& v) {
    n = v.size();
    for (unsigned i = 0; i < v.size(); ++i) stuff[i] = v[i];
  }
};

struct FakeBoolVector {
  unsigned n;
  bool stuff[MAX];

  void fill_stuff(const vector<bool>& v) {
    n = v.size();
    for (unsigned i = 0; i < v.size(); ++i) stuff[i] = v[i];
  }
};

struct FakeColorVector {
  unsigned n;
  Color stuff[MAX];

  void fill_stuff(const vector<Color>& v) {
    n = v.size();
    for (unsigned i = 0; i < v.size(); ++i) stuff[i] = v[i];
  }
};

// Common definition of application state
//
struct State {
  FakePoseVector fishZeroPosComm;
  FakeBoolVector fishZeroAliveComm;
  FakeColorVector fishZeroColorComm;

  FakePoseVector planktonPosesComm;
  FakeBoolVector planktonAliveComm;

  // Nav userFishNav;
  Pose userFishPose;

  // Nav ghostNetNav;
  Pose ghostNetPose;
  int ranNumsComm[4];
  FakeVector ghostNetVertsComm;

  al::Mesh::Vertices ghostNetVerts;

  float parNearTargetTemp;

  // FakeVector colors;
  // vector<Color> particleColors;
  // vector<Pose> particlePoses;
};

#endif
