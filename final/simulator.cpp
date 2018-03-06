/******************************************************************
MAT 201B, 2018 Winter
Author: Chengyuan Xu, cxu@ucsb.edu

Reference:
Based on starter code by Karl Yerkes
Flocking based on The Nature of Code by Daniel Shiffman
textured_background.cpp by Tim Wood

License: GPL-3.0
******************************************************************/

// struct boid;
// struct clan (boids);
// struct target;
// struct targetGroup;
// struct myApp (clan, targetGroup)

#include <cassert>  // gets you assert()
#include "Cuttlebone/Cuttlebone.hpp"
#include "Gamma/Oscillator.h"
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"
#include "allocore/math/al_Vec.hpp"
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
  Plankton(vector<Plankton> *p) {
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
  }

  void update() {
    if (alive == false) {
      pose.pos() = Vec3f(0, 0, 10000);
    } else {
      // since *particles is inside class, no need to bring into the
      // functions.
      Vec3f sep = separate();
      Vec3f stay = stayInCircle();

      sep = sep * 1.5f;
      stay = stay * 0.1f;

      applyForce(sep);
      applyForce(stay);

      velocity += acceleration * timeStep;
      pose.pos() += velocity * timeStep;

      acceleration.zero();  // reset acceleration after each update
    }
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    g.rotate(pose.quat());
    g.color(color);
    g.draw(planktonMesh);
    g.popMatrix();
  }

  Vec3f separate() {
    int count = 0;
    Vec3f steer;

    for (auto other : *planktons) {
      // this difference is a vector from b to a, put this force on a, so
      // push away.
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      // if agents is getting closer, push away
      if (d > 0 && d < 20 * sphereRadius) {
        steer += difference.normalize() / d;
        count++;
      }
    }
    if (count > 0) {
      steer = steer / count;
    }
    if (steer.mag() > 0) {
      steer = steer.normalize() * maxSpeed;
      steer -= velocity;
    }
    return steer;
  }

  Vec3f seek(Vec3f target) {
    Vec3f desired = target - pose.pos();
    Vec3f steer = desired.normalize(maxSpeed);
    steer -= velocity;
    return steer;
  }

  void eaten() { alive = false; }

  Vec3f stayInCircle() { return seek(rnd::ball<Vec3f>() * 20.0f * initRadius); }

  void applyForce(Vec3f force) { acceleration += force; }
};

// NormalFish
struct Fish {
  float lifespan, mass, targetPlankton = 10000;
  Vec3f velocity, acceleration;
  Pose pose, target;
  Color color;
  bool alive;
  int id;
  int myTargetID;
  Plankton targetP;

  // *particles is the pointer to the actual particleList
  vector<Fish> *fishes;

  Fish() {}

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Fish(vector<Fish> *f) {
    pose.pos() = circle() * 5;
    pose.pos() = (rnd::ball<Vec3f>() * 10 * initRadius);
    velocity = Vec3f(0, 0, 0);
    // acceleration = r() * initSpeed;
    acceleration = Vec3f(0, 0, 0);
    color = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    fishes = f;
    alive = true;
  }

  void update() {
    if (alive == false) {
      pose.pos() = Vec3f(0, 0, 10000);
    } else {
      // since *particles is inside class, no need to bring into the
      // functions.
      Vec3f sep = separate();
      Vec3f ali = align();
      Vec3f coh = cohesion();
      Vec3f stay = stayInCircle();

      // 4 * sphereRadius, 10 * sphereRadius, 30 * sphereRadius
      // 1.0 , 1.0 , 1.0 is an interesting stable combination

      sep = sep * 1.5f;
      ali = ali * 1.0f;
      coh = coh * 1.0f;
      stay = stay * 1.0f;

      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
      applyForce(stay);

      velocity += acceleration * timeStep;
      pose.pos() += velocity * timeStep;

      acceleration.zero();  // reset acceleration after each update
    }
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

  Vec3f separate() {
    int count = 0;
    Vec3f steer;

    for (auto other : *fishes) {
      // this difference is a vector from b to a, put this force on a, so
      // push away.
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      // if agents is getting closer, push away
      if (d > 0 && d < 6 * sphereRadius) {
        steer += difference.normalize() / d;
        count++;
      }
    }
    if (count > 0) {
      steer = steer / count;
    }
    if (steer.mag() > 0) {
      steer = steer.normalize() * maxSpeed;
      steer -= velocity;
    }
    return steer;
  }

  Vec3f align() {
    int count = 0;
    Vec3f steer;
    Vec3f sum;

    for (auto other : *fishes) {
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      if (d > 0 && d < 30 * sphereRadius) {
        sum += other.acceleration;
        count++;
      }
    }
    if (count > 0) {
      sum = sum / count;
      sum.normalize(maxSpeed);
      steer = sum - velocity;
      return steer;
    } else {
      return Vec3f(0, 0, 0);
    }
  }

  Vec3f cohesion() {
    int count = 0;
    Vec3f sum;

    for (auto other : *fishes) {
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      if (d > 0 && d < 30 * sphereRadius) {
        sum += other.pose.pos();
        count++;
      }
    }
    if (count > 0) {
      sum = sum / count;
      sum.normalize(maxSpeed);
      return seek(sum);
    } else {
      return Vec3f(0, 0, 0);
    }
  }

  Vec3f seek(Vec3f target) {
    Vec3f desired = target - pose.pos();
    Vec3f steer = desired.normalize(maxSpeed);
    steer -= velocity;
    return steer;
  }

  void eaten() { alive = false; }

  // void seekTarget(Vec3f target) {
  //   Vec3f steer = seek(target).normalize();
  //   steer = steer * maxAcceleration;
  //   applyForce(steer);
  // }
  void seekTarget(vector<Plankton> planktons) {
    Quatf targetQuat;
    Vec3f targetPos;

    ///////// this is causing a bug
    for (int ii = 0; ii < planktons.size(); ii++) {
      float d = (pose.pos() - planktons[ii].pose.pos()).mag();
      if (d < targetPlankton) {
        targetPlankton = d;
        myTargetID = ii;
        targetP = planktons[ii];
        targetQuat = planktons[ii].pose.quat();
        targetPos = planktons[ii].pose.pos();
      }
    }
    pose.quat().slerpTo(targetQuat, 0.06);
    // Vec3f desired = targetPos - pose.pos();
    // Vec3f steer = desired.normalize() * maxSpeed * 1.0f;
    // Vec3f steer = seek(targetPos) * 10.0f;
    Vec3f steer = seek(targetPos) * 3.0f;
    // applyForce(steer);
  }

  void runAway(Vec3f target) {
    Vec3f desired = target - pose.pos();
    Vec3f steer = -(desired.normalize() * maxSpeed);
    steer = (steer - velocity) * 5.0f;
    applyForce(steer);
  }

  Vec3f stayInCircle() {
    float d = pose.pos().mag() / 3;
    if (d > OUT_BOUND) {
      return seek(Vec3f(0, 0, 0));
    } else
      // return seek(Vec3f(pose.pos().x + 10 * velocity.x,
      //                   rnd::uniform(initRadius),
      //                   pose.pos().z + 10 * velocity.z));
      return seek(rnd::ball<Vec3f>() * 10 * initRadius);
  }

  void applyForce(Vec3f force) { acceleration += force; }
};

// UserFish
struct UserFish {
  Vec3f velocity, acceleration, lastPos;
  Nav nav;
  Color color;
  Mesh tentacles;
  int idx;
  bool autoMode;

  UserFish() {
    nav.pos() = Vec3f(0, 0, 0) * initRadius * 2;
    velocity = Vec3f(0, 0, 0);
    velocity = Vec3f(0, 0, 0);
    color = RGB(1);

    tentacles.primitive(Graphics::LINES);
    for (int i = 0; i < TAIL_LENGTH; i++) {
      tentacles.vertex(Vec3f());
      tentacles.color(RGB(1));
    }
    idx = 0;
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(nav.pos());
    g.rotate(nav.quat());
    g.color(color);
    g.draw(userFishMesh);
    g.popMatrix();
    g.draw(tentacles);
  }

  void update() {
    velocity += acceleration * timeStep;
    nav.pos() += velocity * timeStep;
    acceleration.zero();  // reset acceleration after each update
  }

  void seekTarget(vector<Fish> fishes) {
    Quatf targetQuat;
    Vec3f targetPos;
    float nearestFish = 10000;
    for (auto fish : fishes) {
      float d = (nav.pos() - fish.pose.pos()).mag();
      if (d < nearestFish) {
        nearestFish = d;
        targetQuat = fish.pose.quat();
        targetPos = fish.pose.pos();
      }
    }
    nav.quat().slerpTo(targetQuat, 0.06);

    Vec3f desired = targetPos - nav.pos();
    Vec3f steer = desired.normalize() * maxAcceleration;
    steer = steer - velocity;
    applyForce(steer);
  }

  void applyForce(Vec3f force) { acceleration += force; }
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
struct MyApp : App {
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

  // for sound
  gam::SineD<> sined;
  gam::Accum<> timer;

  MyApp() {
    light.pos(0, 0, 0);   // place the light
    nav().pos(0, 0, 50);  // place the viewer
    background(Color(0.1));

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
      addCone(userFishMesh, sphereRadius * 3, Vec3f(0, 0, sphereRadius * 9), 16,
              1);
      addCone(fishMeshS, sphereRadius * 1, Vec3f(0, 0, sphereRadius * 3), 16,
              1);
      addCone(fishMeshM, sphereRadius * 2, Vec3f(0, 0, sphereRadius * 6), 16,
              1);
      addCone(fishMeshL, sphereRadius * 3, Vec3f(0, 0, sphereRadius * 9), 16,
              1);
      // addSphere(planktonMesh, 0.5, 8, 8);
      userFishMesh.generateNormals();
      fishMeshS.generateNormals();
      fishMeshM.generateNormals();
      fishMeshL.generateNormals();
      // planktonMesh.generateNormals();
    }

    // pushing every Particle instance into the actual list
    for (int i = 0; i < fishCount; i++) {
      Fish newFish(&fishZeroList);
      fishZeroList.push_back(newFish);
      newFish.id = i;
    }
    for (int i = 0; i < fishCount * 2; i++) {
      Plankton newPlankton(&planktonList);
      planktonList.push_back(newPlankton);
      newPlankton.id = i;
    }

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    // reset two variable for sound
    nearbyFish = 0;
    myFrameRate = 1 / dt;
    // cout << "Current Frame Rate: " << myFrameRate << endl;

    if (!simulate)
      // skip the rest of this function
      return;

    // light position
    // light.pos(userFishZero.nav.pos());

    // userFish animation
    userFishZero.update();
    if (userFishZero.autoMode) {
      userFishZero.seekTarget(fishZeroList);
    }

    // fish animation
    for (int i = 0; i < fishZeroList.size(); ++i) {
      fishZeroList[i].update();
      fishZeroList[i].pose.faceToward(userFishZero.nav.pos());
      // fishZeroList[i].pose.faceToward(fishZeroList[i].targetP.pose.pos());

      // get distance
      Vec3f diff_predator = fishZeroList[i].pose.pos() - userFishZero.nav.pos();
      float d2 = diff_predator.mag();

      // mark nearby plankton dead
      if (d2 > 50 * sphereRadius) {
        fishZeroList[i].seekTarget(planktonList);
        if (fishZeroList[i].targetPlankton < 3 * sphereRadius) {
          planktonList[fishZeroList[i].myTargetID].eaten();
        }
      }

      // run away from predator
      if (d2 < 20 * sphereRadius) {
        fishZeroList[i].runAway(userFishZero.nav.pos());
      }
      if (d2 < 30 * sphereRadius) {
        nearbyFish += 1;
      }

      // mark nearby fish dead
      if (d2 < 5 * sphereRadius) {
        fishZeroList[i].eaten();
      }
    }

    // plankton animation
    for (int i = 0; i < planktonList.size(); ++i) {
      planktonList[i].update();
    }

    // how close is the target to viewer
    Vec3f diff_nav = nav().pos() - userFishZero.nav.pos();
    targetToNav = diff_nav.mag();  // it ranges from 50 - 100
    timer.freq(nearbyFish);
  }

  void onDraw(Graphics &g, const Viewpoint &v) {
    // draw background textured sphere centered at nav
    // turn off lighting
    g.lighting(false);
    // disable depth buffer, so that background will be drawn over
    g.depthMask(false);

    g.pushMatrix();
    g.translate(nav().pos());
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
    g.popMatrix();
  }

  void onKeyDown(const ViewpointWindow &, const Keyboard &k) {
    float targetPosition = 10;
    switch (k.key()) {
      default:
      case 'u':
        userFishZero.nav.pos().y += targetPosition;
        break;
      case 'j':
        userFishZero.nav.pos().y -= targetPosition;
        break;
      case 'h':
        userFishZero.nav.pos().x -= targetPosition;
        break;
      case 'k':
        userFishZero.nav.pos().x += targetPosition;
        break;
      case '1':
        // reverse time
        timeStep *= -1;
        break;
      case '2':
        // speed up time
        if (timeStep < 1) timeStep *= 2;
        break;
      case '3':
        // slow down time
        if (timeStep > 0.0005) timeStep /= 2;
        break;
      case '4':
        // pause the simulation
        //   simulate = !simulate;
        break;
      case '5':
        // change auto/manual control
        if (userFishZero.autoMode)
          userFishZero.autoMode = false;
        else
          userFishZero.autoMode = true;
        break;
    }
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
  app.start();
}