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
#include "agents_common.hpp"
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"
#include "allocore/math/al_Vec.hpp"

using namespace al;
using namespace std;

// some of these must be carefully balanced; i spent some time turning them.
// change them however you like, but make a note of these settings.
unsigned particleCount = 300;  // try 2, 5, 50, and 5000
double maxAcceleration = 300;  // prevents explosion, loss of particles
double maxSpeed = 50;          // mock number
double initialRadius = 50;     // initial condition
double initialSpeed = 300;     // initial condition
double timeStep = 0.01;        // keys change this value for effect
double scaleFactor = 0.1;      // resizes the entire scene
double sphereRadius = 2;       // increase this to make collisions more frequent
float steerFactor = -1e1;      // Seperation steer constant

Mesh yuanqiu;  // global prototype; leave this alone
Mesh mubiao;   // global target
Mesh lieren;

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }

// Particle
/////////////////////////////
struct Particle {
  Vec3f position, velocity, acceleration, target;
  Pose pose;
  Color c;

  // *particles is the pointer to the actual particleList
  vector<Particle> *particles;

  Particle(vector<Particle> *p) {
    pose.pos() = r() * initialRadius * 2;
    velocity = Vec3f(0, 0, 0);
    acceleration = Vec3f(0, 0, 0);
    c = HSV(rnd::uniform(), 1, 1);
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

// Target
/////////////////////////////
struct Target {
  Vec3f position, velocity, acceleration, desired, steer;
  Color c;
  double lifetime = 0, i = 0;

  Target() {
    position = r() * initialRadius;
    Color c = RGB(0, 1, 0);
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(mubiao);
    g.popMatrix();
  }
};

struct Predator : Target {
  Predator() {
    position = r() * initialRadius;
    Color c = RGB(1, 0, 0);
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(lieren);
    g.popMatrix();
  }
};

// MyApp
/////////////////////////////
struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  State appState;
  cuttlebone::Taker<State> taker;

  // creating the real particleList, it's now empty
  vector<Particle> particles;

  Target mubiaoOne;
  Predator lierenOne;

  MyApp() {
    addCone(yuanqiu, sphereRadius, Vec3f(0, 0, sphereRadius * 3), 16, 1);
    yuanqiu.generateNormals();

    addSphere(mubiao, sphereRadius, 16);
    mubiao.generateNormals();

    addSphere(lieren, sphereRadius * 2, 16);
    lieren.generateNormals();

    light.pos(0, 10, 10);  // place the light
    nav().pos(0, 0, 30);   // place the viewer
    lens().far(400);       // set the far clipping plane
    background(Color(0.07));

    // pushing every Particle instance into the actual list
    for (int i = 0; i < particleCount; i++) {
      Particle par(&particles);
      particles.push_back(par);
    }

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    // get data from common
    taker.get(appState);

    // light position
    light.pos(mubiaoOne.position);

    // Target animation
    mubiaoOne.position = appState.targetPosition;

    // Predator animation
    lierenOne.position = appState.predatorPosition;

    // Particle animation
    for (int i = 0; i < particles.size(); ++i) {
      particles[i].pose.pos().x = appState.parPositions.stuff[i].x;
      particles[i].pose.pos().y = appState.parPositions.stuff[i].y;
      particles[i].pose.pos().z = appState.parPositions.stuff[i].z;
    }

    // nav().faceToward(mubiaoOne.position);
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto par : particles) {
      par.draw(g);
    }
    mubiaoOne.draw(g);
    lierenOne.draw(g);
  }
};

int main() {
  MyApp app;
  app.taker.start();
  app.start();
}