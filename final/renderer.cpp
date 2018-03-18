/******************************************************************
MAT 201B, 2018 Winter
Author: Chengyuan Xu, cxu@ucsb.edu

Reference:
Based on starter code by Karl Yerkes
Flocking based on The Nature of Code by Daniel Shiffman
textured_background.cpp by Tim Wood

License: GPL-3.0
******************************************************************/

#include <float.h>
#include <math.h>
#include <cassert>  // gets you assert()
#include "Cuttlebone/Cuttlebone.hpp"
#include "Gamma/Oscillator.h"
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"
#include "allocore/math/al_Vec.hpp"
#include "alloutil/al_OmniStereoGraphicsRenderer.hpp"
#include "common.hpp"

using namespace al;
using namespace std;

#define TAIL_LENGTH 20
#define OUT_BOUND 200

// some of these must be carefully balanced; i spent some time turning them.
// change them however you like, but make a note of these settings.
unsigned fishCount = 300;      // try 2, 5, 50, and 5000
double maxAcceleration = 300;  // prevents explosion, loss of particles
double maxSpeed = 100;         // mock number
double initRadius = 50;        // initial condition
double initSpeed = 100;        // initial condition
double timeStep = 0.01;        // keys change this value for effect
double scaleFactor = 0.1;      // resizes the entire scene
double sphereRadius = 2;       // increase this to make collisions more frequent
int targetFishID = 0;
int ranNums[4];

// global variables for sound
float targetToNav;
float nearbyFish;
float myFrameRate;

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }
Vec3f circle() {
  return Vec3f(
      initRadius * sin(rnd::uniformS(2 * M_PI)) * cos(rnd::uniformS(2 * M_PI)),
      initRadius * sin(rnd::uniformS(2 * M_PI)) * sin(rnd::uniformS(2 * M_PI)),
      initRadius * cos(rnd::uniformS(2 * M_PI)));
}

// global mesh variables
Mesh userFishMesh;
Mesh fishMeshS;
Mesh fishMeshM;
Mesh fishMeshL;
Mesh planktonMesh;

// customized random func
int myRnd(int seed, int max, int min) {
  rnd::Random<> rig;
  rig.seed(seed);
  return rig.uniform(max, min);
}

// Plankton
struct Plankton {
  float lifespan, mass;
  Vec3f velocity, acceleration;
  Pose pose, target;
  Color color;
  bool alive;
  int id;

  // *particles is the pointer to the actual particleList
  vector<Plankton> *planktons;

  Plankton() {}

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Plankton(vector<Plankton> *p, int i) {
    // pose.pos() = circle() * 10;
    pose.pos() = rnd::ball<Vec3f>() * 10.0f * initRadius;
    pose.quat().set(float(rnd::uniform()), float(rnd::uniform()),
                    float(rnd::uniform()), float(rnd::uniform()));
    velocity = Vec3f(0, 0, 0);
    acceleration = Vec3f(0, 0, 0);
    // color = RGB(150);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    planktons = p;
    alive = true;
    id = i;
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(pose.quat());
    g.color(color);
    g.draw(planktonMesh);
    g.popMatrix();
  }
};

// NormalFish
struct Fish {
  float lifespan, mass, targetP_diff;
  Vec3f velocity, acceleration;
  Pose pose, target;
  Color color;
  bool alive;
  int id;

  // for target plankton
  int targetID;

  // *particles is the pointer to the actual particleList
  vector<Fish> *fishes;

  Fish() {}

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Fish(vector<Fish> *f, int i) {
    // pose.pos() = circle() * 5;
    pose.pos() = (rnd::ball<Vec3f>() * 5 * initRadius);
    velocity = Vec3f(0, 0, 0).cross(pose.pos()).normalize(initSpeed);
    // velocity = Vec3f(0, 0, 0);
    // acceleration = r() * initSpeed;
    acceleration = Vec3f(0, 0, 0);
    color = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    fishes = f;
    alive = true;
    id = i;
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(pose.quat());
    g.color(color);
    if (id % 5 == 0) {
      g.draw(fishMeshS);
    } else if (id % 2 == 0) {
      g.draw(fishMeshM);
    } else {
      g.draw(fishMeshL);
    }
    g.popMatrix();
  }
};

// UserFish
struct UserFish {
  Vec3f velocity, acceleration, lastPos;
  Pose pose;
  Color color;
  Mesh tentacles;
  bool autoMode;
  // Quatf targetQuat;
  // Vec3f targetPos;

  UserFish() {
    pose.pos() = Vec3f(0, 0, 0) * initRadius * 2;
    velocity = Vec3f(0, 0, 0);
    velocity = Vec3f(0, 0, 0);
    color = RGB(1);

    tentacles.primitive(Graphics::LINES);
    for (int i = 0; i < TAIL_LENGTH; i++) {
      tentacles.vertex(Vec3f());
      tentacles.color(RGB(1));
    }
    // id = 0;
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(pose.quat());
    g.color(color);
    g.draw(userFishMesh);
    g.popMatrix();
    g.draw(tentacles);
  }

  void update() {
    velocity += acceleration * timeStep;
    pose.pos() += velocity * timeStep;
    acceleration.zero();  // reset acceleration after each update
  }
};

// Ghost net
struct GhostNet {
  Mesh ghostNetMesh, boundingBoxMesh;
  Vec3f velocity, acceleration, center;
  Pose pose;
  Color color;
  double timePast;
  int total;
  vector<Vec3f> vertices;

  GhostNet() {
    pose.pos() =
        Vec3f(rnd::uniform(100), rnd::uniform(400, 100), rnd::uniform(100));
    // nav.quat().set(float(rnd::uniform()), float(rnd::uniform()),
    //                float(rnd::uniform()), float(rnd::uniform()));
    velocity = Vec3f(0, 0, 0);
    color = RGB(0.9f);

    // generate the shape
    addSurface(ghostNetMesh, 20, 40, 40, 80);

    // randomize the vertices
    for (int i = 0; i < ghostNetMesh.vertices().size(); i++) {
      ghostNetMesh.vertices()[i] += myRnd(i, 5, -5);
      vertices.push_back(ghostNetMesh.vertices()[i]);
    }

    ghostNetMesh.primitive(Graphics::LINE_LOOP);
    ghostNetMesh.generateNormals();

    total = ghostNetMesh.vertices().size();
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(pose.quat());
    g.color(color);
    g.draw(ghostNetMesh);
    g.popMatrix();
    // g.draw(tentacles);
  }

  void wiggle(double dt) {
    float topY = -FLT_MAX, bottomY = FLT_MAX, leftX = -FLT_MAX,
          rightX = FLT_MAX, nearZ = -FLT_MAX, farZ = FLT_MAX;
    float record = 0.f;
    // add random offset to vertices to make them wiggle and deform
    for (int i = 0; i < total - 1; i++) {
      Vec3f *VertPointer;
      VertPointer = &(ghostNetMesh.vertices()[i]);
      Vec3f VertV, VertA, steer;

      // calculate ghost net's top vertices
      // if (*VertPointer.y > topY) topY = *VertPointer.y;
      // if (*VertPointer.y < bottomY) bottomY = *VertPointer.y;
      // if (*VertPointer.x > leftX) leftX = *VertPointer.x;
      // if (*VertPointer.x < rightX) rightX = *VertPointer.x;
      // if (*VertPointer.z > nearZ) nearZ = *VertPointer.z;
      // if (*VertPointer.z < farZ) farZ = *VertPointer.z;

      Vec3f diff = ghostNetMesh.vertices()[i] - ghostNetMesh.vertices()[i + 1];
      Vec3f midd =
          (ghostNetMesh.vertices()[i] + ghostNetMesh.vertices()[i + 1]) / 2;
      float d = diff.mag();
      if (d > 0 && d < 5) {
        diff.normalize(maxSpeed);
        steer = diff - VertV;
      } else {
        midd -= midd - ghostNetMesh.vertices()[i];
        midd.normalize(maxAcceleration);
        steer = midd;
      }
      VertA += steer;

      VertV += VertA * timeStep;
      *VertPointer += VertV * timeStep;
    }
    ghostNetMesh.generateNormals();
  }
};

string fullPathOrDie(string fileName, string whereToLook = ".") {
  SearchPaths searchPaths;
  searchPaths.addSearchPath(whereToLook);
  string filePath = searchPaths.find(fileName).filepath();
  assert(filePath != "");
  return filePath;
}

// MyApp
/////////////////////////////
struct MyApp : OmniStereoGraphicsRenderer {
  // Cuttlebone
  State appState;
  cuttlebone::Taker<State> taker;

  // for text and image
  Mesh bgMesh;
  Texture bgTexture;
  Texture planktonTexture;

  // general environment setting
  Material material;
  Light light;
  bool simulate = true;

  // creating the real particleList, it's now empty
  vector<Fish> fishZeroList;
  vector<Plankton> planktonList;
  UserFish userFishZero;

  GhostNet ghostNet0;

  // for sound
  gam::SineD<> sined;
  gam::Accum<> timer;

  MyApp() {
    light.pos(0, 0, 0);  // place the light
    pose.pos(0, 0, 0);   // place the viewer
    // background(Color(0.1));
    omni().clearColor() = Color(0.15);

    // set near/far clip
    lens().near(0.1);
    lens().far(1000);  // increase far clip to 1000 GL Units

    // generate sphere with texture coordinates
    {
      addSphereWithTexcoords(bgMesh);

      // load image into texture print out error and exit if failure
      Image image;
      SearchPaths searchPaths;
      searchPaths.addSearchPath(".");
      string filename =
          searchPaths.find("DSC06307_Panorama_dark.jpg").filepath();
      if (image.load(filename)) {
        cout << "Read image from " << filename << endl;
      } else {
        cout << "Failed to read image from " << filename << "!!!" << endl;
        exit(-1);
      }
      bgTexture.allocate(image.array());

      // load image for plankton
      addSphereWithTexcoords(planktonMesh);

      Image planktonImage;
      string filename2 = searchPaths.find("dot_plankton.png").filepath();
      if (planktonImage.load(filename2)) {
        cout << "Read image from " << filename << endl;
      } else {
        cout << "Failed to read image from " << filename << "!!!" << endl;
        exit(-1);
      }
      planktonTexture.allocate(planktonImage.array());
    }

    // add vertices to empty Meshes
    {
      addCone(userFishMesh, sphereRadius * 3, Vec3f(0, 0, sphereRadius * 12),
              16, 1);
      addCone(fishMeshS, sphereRadius * 0.5, Vec3f(0, 0, sphereRadius * 1.5),
              16, 1);
      addCone(fishMeshM, sphereRadius * 1, Vec3f(0, 0, sphereRadius * 3), 16,
              1);
      addCone(fishMeshL, sphereRadius * 1.5, Vec3f(0, 0, sphereRadius * 4.5),
              16, 1);
      // addSphere(planktonMesh, 0.5, 8, 8);
      userFishMesh.generateNormals();
      fishMeshS.generateNormals();
      fishMeshM.generateNormals();
      fishMeshL.generateNormals();
      // planktonMesh.generateNormals();
    }

    // pushing every Particle instance into the actual list
    for (int i = 0; i < fishCount; i++) {
      Fish newFish(&fishZeroList, i);
      fishZeroList.push_back(newFish);
    }

    for (int i = 0; i < fishCount * 5; i++) {
      Plankton newPlankton(&planktonList, i);
      planktonList.push_back(newPlankton);
    }

    initWindow();
    // initAudio();
  }

  void onAnimate(double dt) {
    // reset two variable for sound
    nearbyFish = 0;
    myFrameRate = 1 / dt;
    // cout << "Current Frame Rate: " << myFrameRate << endl;

    // get data from common, skip all until we heard at least once from the sim
    taker.get(appState);
    // static bool hasNeverHeardFromSim = true;
    // if (taker.get(appState) > 0) {
    //   hasNeverHeardFromSim = false;
    // }
    // if (hasNeverHeardFromSim) return;

    if (!simulate)
      // skip the rest of this function
      return;

    // light position
    light.pos(userFishZero.pose.pos());

    // userFish animation
    userFishZero.pose = appState.userFishPose;  // cuttlebone

    // ghost net animation
    ghostNet0.pose = appState.ghostNetPose;
    for (int i = 0; i < ghostNet0.total; i++) {
      ghostNet0.vertices[i] = appState.ghostNetVertsComm.stuff[i];
      ghostNet0.ghostNetMesh.vertices()[i] = ghostNet0.vertices[i];
    }
    // ghostNet0.wiggle(dt);

    // fish animation
    for (int i = 0; i < fishZeroList.size(); ++i) {
      // cuttlebone fish
      fishZeroList[i].pose = appState.fishZeroPosComm.stuff[i];
      fishZeroList[i].alive = appState.fishZeroAliveComm.stuff[i];
      fishZeroList[i].color = appState.fishZeroColorComm.stuff[i];
    }

    // plankton animation
    for (int i = 0; i < planktonList.size(); ++i) {
      planktonList[i].pose = appState.planktonPosesComm.stuff[i];
      planktonList[i].alive = appState.planktonAliveComm.stuff[i];
    }

    // how close is the target to viewer
    Vec3f diff_nav = pose.pos() - userFishZero.pose.pos();
    targetToNav = diff_nav.mag();  // it ranges from 50 - 100
    timer.freq(nearbyFish);
  }

  void onDraw(Graphics &g) {
    shader().uniform("texture", 1.0);
    shader().uniform("lighting", 1.0);

    // draw background textured sphere centered at nav
    // turn off lighting
    g.lighting(false);
    // disable depth buffer, so that background will be drawn over
    g.depthMask(false);

    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(180, 0, 0, 1);
    bgTexture.bind();
    g.color(1, 1, 1);
    g.draw(bgMesh);
    bgTexture.unbind();
    g.popMatrix();

    g.depthMask(true);  // turn depth mask back on

    // draw original scene here
    light();  // enable light
    material();
    g.pushMatrix();
    g.scale(scaleFactor);
    for (auto fish : fishZeroList) {
      if (fish.alive == true) {
        fish.draw(g);
      }
    }

    // draw plankton with texture
    g.pushMatrix();
    planktonTexture.bind();
    for (auto p : planktonList) {
      if (p.alive == true) {
        p.draw(g);
      }
    }
    planktonTexture.unbind();
    g.popMatrix();

    userFishZero.draw(g);
    ghostNet0.draw(g);
    g.popMatrix();
  }

  // virtual void onSound(AudioIOData &io) {
  //   gam::Sync::master().spu(audioIO().fps());
  //   while (io()) {
  //     if (timer()) {
  //       // sined.set(rnd::uniform(220.0f, 880.0f), 0.5f, 1.0f);
  //       sined.set(1000.0f - targetToNav * 6, 0.5f, 1.0f);
  //     }
  //     float s = sined();
  //     io.out(0) = s;
  //     io.out(1) = s;
  //   }
  // }
};

int main() {
  MyApp app;
  app.taker.start();
  app.start();
}
