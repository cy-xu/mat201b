/*
MAT 201B, 2018 Winter
Chengyuan Xu, cxu@ucsb.edu
Based on starter code by Karl Yerkes
MIT License (details omitted)
*/

// struct boid;
// struct clan (boids);
// struct target;
// struct targetGroup;
// struct myApp (clan, targetGroup)

#include "Cuttlebone/Cuttlebone.hpp"
#include "Gamma/Oscillator.h"
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"
#include "allocore/math/al_Vec.hpp"
#include "common.hpp"

using namespace al;
using namespace std;

#define TAIL_LENGTH 20

// some of these must be carefully balanced; i spent some time turning them.
// change them however you like, but make a note of these settings.
unsigned fishCount = 300;      // try 2, 5, 50, and 5000
double maxAcceleration = 300;  // prevents explosion, loss of particles
double maxSpeed = 100;         // mock number
double initialRadius = 50;     // initial condition
double initialSpeed = 100;     // initial condition
double timeStep = 0.01;        // keys change this value for effect
double scaleFactor = 0.1;      // resizes the entire scene
double sphereRadius = 2;       // increase this to make collisions more frequent

Mesh yuanqiu;  // global prototype; leave this alone
Mesh mubiao;   // global target
Mesh lieren;   // global predator

Mesh fishMesh;
Mesh userFishMesh;

// global variables for sound
float targetToNav;
float nearbyFish;
float myFrameRate;

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }

// All Marine Creature
struct MarineCreature {
  float lifespan, mass;
  Vec3f velocity, acceleration;
  Pose pose, target;
  Color color;
  bool alive;

  // *particles is the pointer to the actual particleList
  vector<MarineCreature> *marineCreatures;

  MarineCreature() {}

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  MarineCreature(vector<MarineCreature> *p) {
    pose.pos() = r() * initialRadius * 2;
    velocity = Vec3f(0, 200, 0).cross(pose.pos()).normalize(initialSpeed);
    acceleration = r() * initialSpeed;
    // acceleration = Vec3f(0, 0, 0);
    color = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    marineCreatures = p;
  }

  void update() {
    // since *particles is inside class, no need to bring into the
    // functions.
    Vec3f sep = separate();
    Vec3f ali = align();
    Vec3f coh = cohesion();

    // 4 * sphereRadius, 10 * sphereRadius, 30 * sphereRadius
    // 1.0 , 1.0 , 1.0 is an interesting stable combination

    sep = sep * 6.0f;
    ali = ali * 1.f;
    coh = coh * 1.f;

    applyForce(sep);
    applyForce(ali);
    applyForce(coh);

    velocity += acceleration * timeStep;
    pose.pos() += velocity * timeStep;

    acceleration.zero();  // reset acceleration after each update
  }

  void applyForce(Vec3f force) { acceleration += force; }

  Vec3f separate() {
    int count = 0;
    Vec3f steer;

    for (auto other : *marineCreatures) {
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
      steer.normalize() * maxAcceleration;
      steer -= velocity;
    }
    return steer;
  }

  Vec3f align() {
    int count = 0;
    Vec3f steer;
    Vec3f sum;

    for (auto other : *marineCreatures) {
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      if (d > 0 && d < 20 * sphereRadius) {
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

    for (auto other : *marineCreatures) {
      Vec3f difference = (pose.pos() - other.pose.pos());
      float d = difference.mag();
      if (d > 0 && d < 10 * sphereRadius) {
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
    desired.normalize(maxSpeed);
    Vec3f steer = desired - velocity;
    return steer;
  }

  void seekTarget(Vec3f target) {
    Vec3f desired = target - pose.pos();
    Vec3f steer = desired - velocity;
    steer.normalize(maxSpeed * 1);
    applyForce(steer);
  }

  void runAway(Vec3f target) {
    Vec3f desired = target - pose.pos();
    Vec3f steer = -(desired - velocity);
    steer = steer.normalize() * maxSpeed * 0.005;
    applyForce(steer);
  }
};

// NormalFish
struct Fish : MarineCreature {
  // *particles is the pointer to the actual particleList
  vector<Fish> *fishes;

  Fish() {}

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Fish(vector<Fish> *f) {
    pose.pos() = r() * initialRadius;
    velocity = Vec3f(0, 0, 0).cross(pose.pos()).normalize(initialSpeed);
    acceleration = r() * initialSpeed;
    // acceleration = Vec3f(0, 0, 0);
    color = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    fishes = f;
    alive = true;
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
      steer.normalize(maxSpeed);
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
      if (d > 0 && d < 50 * sphereRadius) {
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
      if (d > 0 && d < 10 * sphereRadius) {
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

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    // g.rotate(pose.quat().inverse());
    g.rotate(pose.quat());
    g.color(color);
    g.draw(fishMesh);
    g.popMatrix();
  }

  void eaten() { alive = false; }

  void update() {
    if (alive == false) {
      pose.pos() = Vec3f(100, 100, 100);
    } else {
      // since *particles is inside class, no need to bring into the
      // functions.
      Vec3f sep = separate();
      Vec3f ali = align();
      Vec3f coh = cohesion();

      // 4 * sphereRadius, 10 * sphereRadius, 30 * sphereRadius
      // 1.0 , 1.0 , 1.0 is an interesting stable combination

      sep = sep * 6.0f;
      ali = ali * 1.f;
      coh = coh * 1.f;

      applyForce(sep);
      applyForce(ali);
      applyForce(coh);

      velocity += acceleration * timeStep;
      pose.pos() += velocity * timeStep;
      cohesion();

      acceleration.zero();  // reset acceleration after each update
    }
  }
};

// UserFish
struct UserFish {
  Vec3f velocity, acceleration, lastPos;
  Nav nav;
  Color color;
  Mesh tentacles;
  int idx;

  UserFish() {
    nav.pos() = Vec3f(0, 0, 0) * initialRadius * 2;
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
    // remember previous position and step nav
    lastPos.set(nav.pos());
    // nav.step(dt);

    // animate tail
    int size = tentacles.vertices().size();
    if ((nav.pos() - lastPos).mag() > 5)
      // dont draw line segment if position was wraped
      tentacles.vertices()[idx] = nav.pos();
    else
      tentacles.vertices()[idx] = lastPos;
    idx = wrap(idx + 1, size, 0);
    tentacles.vertices()[idx] = nav.pos();
    idx = wrap(idx + 1, size, 0);

    // set tail colors to fade older segments out
    int c = idx - 1;
    c = wrap(c, size, 0);
    for (int i = 0; i < size; i++) {
      tentacles.colors()[c--] = RGB((size - i) / (float)size);
      c = wrap(c, size, 0);
    }

    velocity += acceleration * timeStep;
    nav.pos() += velocity * timeStep;
    acceleration.zero();  // reset acceleration after each update
  }

  void seekTarget(vector<Fish> fishes) {
    Quatf targetQuat;
    Vec3f targetPos;
    float nearestFish = 10000;
    for (auto fish : fishes) {
      Vec3f difference = (nav.pos() - fish.pose.pos());
      float d = difference.mag();
      if (d < nearestFish) {
        nearestFish = d;
        targetQuat = fish.pose.quat();
        targetPos = fish.pose.pos();
      }
    }
    // cout << "current nearest Fish " << nearestFish << endl;
    nav.quat().slerpTo(targetQuat, 0.01);
    // nav.move(targetPos);

    Vec3f desired = targetPos - nav.pos();
    Vec3f steer = desired - velocity;
    steer = steer.normalize() * maxSpeed * 10;
    applyForce(steer);
  }

  void applyForce(Vec3f force) { acceleration += force; }
};

// MyApp
/////////////////////////////
struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  // creating the real particleList, it's now empty
  vector<Fish> fishZeroList;
  Fish fish;
  UserFish userFishZero;

  // for sound
  gam::SineD<> sined;
  gam::Accum<> timer;

  MyApp() {
    addCone(fishMesh, sphereRadius, Vec3f(0, 0, sphereRadius * 3), 16, 1);
    fishMesh.generateNormals();

    addCone(userFishMesh, sphereRadius * 3, Vec3f(0, 0, sphereRadius * 9), 16,
            1);
    userFishMesh.generateNormals();

    light.pos(0, 10, 10);  // place the light
    // light.diffuse();
    nav().pos(0, 0, 30);  // place the viewer
    lens().far(400);      // set the far clipping plane
    background(Color(0.07));

    // pushing every Particle instance into the actual list
    for (int i = 0; i < fishCount; i++) {
      Fish newFish(&fishZeroList);
      fishZeroList.push_back(newFish);
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
    light.pos(userFishZero.nav.pos());

    // userFish animation
    userFishZero.update();
    userFishZero.seekTarget(fishZeroList);

    // fish animation
    for (int i = 0; i < fishZeroList.size(); ++i) {
      fishZeroList[i].update();
      fishZeroList[i].pose.faceToward(userFishZero.nav.pos());

      // run away from predator
      Vec3f diff_predator = fishZeroList[i].pose.pos() - userFishZero.nav.pos();
      float d2 = diff_predator.mag();

      if (d2 < 50 * sphereRadius) {
        nearbyFish += 1;
        // particleList[i].seekTarget(Vec3d(mubiaoOne.position));
      }
      if (d2 < 20 * sphereRadius) {
        fishZeroList[i].runAway(userFishZero.nav.pos());
      }
      if (d2 < 5 * sphereRadius) {
        fishZeroList[i].eaten();
      }
    }

    // how close is the target to viewer
    Vec3f diff_nav = nav().pos() - userFishZero.nav.pos();
    targetToNav = diff_nav.mag();  // it ranges from 50 - 100
    timer.freq(nearbyFish);
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto fish : fishZeroList) {
      if (fish.alive == true) {
        fish.draw(g);
      }
    }
    userFishZero.draw(g);
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