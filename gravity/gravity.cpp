/*
MAT 201B, 2018 Winter
Chengyuan Xu, cxu@ucsb.edu
Based on starter code by Karl Yerkes
MIT License (details omitted)
*/

#include "allocore/io/al_App.hpp"
using namespace al;
using namespace std;

// some of these must be carefully balanced; i spent some time turning them.
// change them however you like, but make a note of these settings.
unsigned particleCount = 2;        // try 2, 5, 50, and 5000
double maximumAcceleration = 600;  // prevents explosion, loss of particles
double initialRadius = 50;         // initial condition
double initialSpeed = 0;           // initial condition
double gravityFactor = 1e5;        // see Gravitational Constant
// double timeStep = 0.0625;         // keys change this value for effect
double timeStep = 0.01;      // keys change this value for effect
double scaleFactor = 0.1;    // resizes the entire scene
double sphereRadius = 8;     // increase this to make collisions more frequent
float springConstant = 200;  // spring characteristic constant
float dragConstant = 0.001;

Mesh sphere;  // global prototype; leave this alone

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }

struct Particle {
  Vec3f position, velocity, acceleration;
  Color c;
  Particle() {
    position = r() * initialRadius;
    velocity =
        // this will tend to spin stuff around the y axis
        Vec3f(0, 1, 0).cross(position).normalize(initialSpeed);
    c = HSV(rnd::uniform(), 0.7, 1);
  }
  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(sphere);
    g.popMatrix();
  }
};

struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  vector<Particle> particle;

  MyApp() {
    addSphere(sphere, sphereRadius);
    sphere.generateNormals();
    light.pos(0, 0, 0);              // place the light
    nav().pos(0, 0, 30);             // place the viewer
    lens().far(400);                 // set the far clipping plane
    particle.resize(particleCount);  // make all the particles
    background(Color(0.07));

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    if (!simulate)
      // skip the rest of this function
      return;

    for (unsigned i = 0; i < particle.size(); ++i)
      for (unsigned j = 1 + i; j < particle.size(); ++j) {
        Particle &a = particle[i];
        Particle &b = particle[j];
        Vec3f difference = (b.position - a.position);
        float d = difference.mag();
        // F = ma where m=1
        Vec3f acceleration;

        if (d > 2 * sphereRadius) {
          acceleration = difference / (d * d * d) * gravityFactor;
        }
        // equal and opposite force (symmetrical)

        // if two sphere contact, using hooks's law
        if (d < 2 * sphereRadius) {
          float hookeRatio = (-springConstant) * abs(2 * sphereRadius - d);
          // float hookeRatio = (-springConstant) * (d - sphereRadius);
          acceleration = difference * hookeRatio;
        }

        // only similar color balls will attract each other
        // double colorEpsilon = 0.2;
        // if (abs(a.c[1] - b.c[1]) < colorEpsilon) {
        a.acceleration += acceleration;
        b.acceleration -= acceleration;
        // }

        // give drag to all sphere
        Vec3f drag = a.velocity * abs(a.velocity) * -dragConstant;
        a.acceleration += drag;

        // else {
        //   // based on Elastic collision formula, if the mass of two spheres
        //   are the same, then they simply exchange velocity. Vec3f
        //   tempVelocity = a.velocity; a.velocity = b.velocity; b.velocity =
        //   tempVelocity;
        // }
      }

    // Limit acceleration
    unsigned limitCount = 0;
    for (auto &p : particle) {
      if (p.acceleration.mag() > maximumAcceleration) {
        p.acceleration.normalize(maximumAcceleration);
        limitCount++;
      }
    }
    printf("%u of %u limited\n", limitCount, particle.size());

    // Euler's Method; Keep the time step small
    for (auto &p : particle) {
      p.position += p.velocity * timeStep;
    }

    for (auto &p : particle) {
      p.velocity += p.acceleration * timeStep;
    }

    for (auto &p : particle) {
      p.acceleration.zero();
    }
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto p : particle) p.draw(g);
  }

  void onSound(AudioIO &io) {
    io();
    io.out(0) = 1;  // left
    io.out(1) = 1;  // right
    while (io()) {
      io.out(0) = 0;
      io.out(1) = 0;
    }
  }

  void onKeyDown(const ViewpointWindow &, const Keyboard &k) {
    switch (k.key()) {
      default:
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
        simulate = !simulate;
        break;
    }
  }
};

int main() { MyApp().start(); }