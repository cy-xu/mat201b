/*
MAT 201B, 2018 Winter
Chengyuan Xu, cxu@ucsb.edu
Based on starter code by Karl Yerkes
MIT License (details omitted)
*/

#include "Cuttlebone/Cuttlebone.hpp"
#include "allocore/io/al_App.hpp"

#include "agents_common.hpp"

using namespace al;
using namespace std;

// some of these must be carefully balanced; i spent some time turning them.
// change them however you like, but make a note of these settings.
unsigned particleCount = 100;  // try 2, 5, 50, and 5000
double maxAcceleration = 300;  // prevents explosion, loss of particles
double maxSpeed = 50;          // mock number
double initialRadius = 50;     // initial condition
double initialSpeed = 100;     // initial condition
double timeStep = 0.01;        // keys change this value for effect
double scaleFactor = 0.1;      // resizes the entire scene
double sphereRadius = 2;       // increase this to make collisions more frequent
float steerFactor = -1e1;      // Seperation steer constant

Mesh yuanqiu;  // global prototype; leave this alone
Mesh mubiao;   // global target

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }

struct Particle {
  Vec3f position, velocity, acceleration, target;
  Pose pose;
  Color c;

  // *particles is the pointer to the actual particleList
  vector<Particle> *particles;

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Particle(vector<Particle> *p) {
    pose.pos() = r() * initialRadius;
    // this will tend to spin stuff around the y axis
    velocity = Vec3f(100, 100, 0).cross(pose.pos()).normalize(initialSpeed);
    acceleration = Vec3f(0, 0, 0);
    // acceleration = r() * initialSpeed;
    c = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    particles = p;
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(pose.pos());
    // g.rotate(pose.quat().inverse());
    g.rotate(pose.quat());
    g.color(c);
    g.draw(yuanqiu);
    g.popMatrix();
  }
};

struct Target {
  Vec3f position, velocity, acceleration;
  Color c;

  Target() {
    position = Vec3f(0, 0, 0);
    velocity = Vec3f(0, 0, 0);
    acceleration = Vec3f(0, 0, 0);
    Color c = HSV(rnd::uniform(), 1, 1);
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(mubiao);
    g.popMatrix();
  }
};

struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  State appState;
  cuttlebone::Taker<State> taker;

  // creating the real particleList, it's now empty
  vector<Particle> particleList;
  vector<Pose> particlePoses;
  vector<Color> particleColors;

  Target mubiaoOne;

  MyApp() {
    taker.get(appState);

    addCone(yuanqiu, sphereRadius, Vec3f(0, 0, sphereRadius * 3), 16, 1);
    yuanqiu.generateNormals();

    addSphere(mubiao, sphereRadius, 16);
    // addWireBox(mubiao, 10, 10, 10);
    // addCube(mubiao, false, 10);
    mubiao.generateNormals();

    light.pos(0, 10, 20);  // place the light
    nav().pos(0, 0, 30);   // place the viewer
    lens().far(400);       // set the far clipping plane
    background(Color(0.07));

    // pushing every Particle instance into the actual list
    for (int i = 0; i < particleCount; i++) {
      Particle par(&particleList);
      particleList.push_back(par);
      particleList[i].c = appState.particleColors[i];
      particleList[i].pose = appState.particlePoses[i];
    }

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    if (!simulate)
      // skip the rest of this function
      return;

    // taker.get(appState);

    mubiaoOne.position = appState.targetPosition;

    for (int i = 0; i < particleList.size(); ++i) {
      particleList[i].pose = appState.particlePoses[i];
    }
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto par : particleList) {
      par.draw(g);
    }
    mubiaoOne.draw(g);
  }
};

int main() {
  MyApp app;
  app.taker.start();
  app.start();
}