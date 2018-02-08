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
unsigned particleCount = 2;    // try 2, 5, 50, and 5000
double maxAcceleration = 300;  // prevents explosion, loss of particles
double maxSpeed = 100;         // mock number
double initialRadius = 50;     // initial condition
double initialSpeed = 100;     // initial condition
double gravityFactor = 1e1;    // see Gravitational Constant
// double timeStep = 0.0625;         // keys change this value for effect
double timeStep = 0.01;      // keys change this value for effect
double scaleFactor = 0.1;    // resizes the entire scene
double sphereRadius = 8;     // increase this to make collisions more frequent
float springConstant = 500;  // spring characteristic constant
float dragConstant = 0.001;  // drag constant
float steerFactor = -1e1;    // Seperation steer constant

Mesh sphere;  // global prototype; leave this alone

// helper function: makes a random vector
Vec3f r() { return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()); }

struct Particle {
  Vec3f position, velocity, acceleration;
  Color c;
  vector<Particle> *particles;

  Particle(vector<Particle> *p) {
    position = r() * initialRadius;
    // this will tend to spin stuff around the y axis
    velocity = Vec3f(0, 1, 0).cross(position).normalize(initialSpeed);
    // acceleration = Vec3f(0, 0, 0);
    acceleration = r() * initialSpeed;
    c = HSV(rnd::uniform(), 1, 1);
    particles = p;
  }

  void flock() {
    Vec3f sep = separate();
    Vec3f ali = align();
    Vec3f coh = cohesion();

    sep = sep * 1.5f;
    ali = ali * 1.0f;
    coh = coh * 1.0f;

    applyForce(sep);
    applyForce(ali);
    applyForce(coh);

    velocity += acceleration * timeStep;
    position += velocity * timeStep;
    acceleration.zero();
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(sphere);
    g.popMatrix();
  }

  void applyForce(Vec3f force) { acceleration += force; }

  Vec3f separate() {
    int count = 0;
    Vec3f steer;

    for (auto other : *particles) {
      // this difference is a vector from b to a, put this force on a, so push
      // away.
      Vec3f difference = (position - other.position);
      float d = difference.mag();
      // if agents is getting closer, push away
      if (d > 0 && d < 4 * sphereRadius) {
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

    for (auto other : *particles) {
      Vec3f difference = (position - other.position);
      float d = difference.mag();
      if (d > 0 && d < 8 * sphereRadius) {
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

    for (auto other : *particles) {
      Vec3f difference = (position - other.position);
      float d = difference.mag();
      if (d > 0 && d < 8 * sphereRadius) {
        sum += other.position;
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
    Vec3f desired = target - position;
    desired.normalize(maxSpeed);
    Vec3f steer = desired - velocity;
    return steer;
  }
};

struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  vector<Particle> particleList;

  MyApp() {
    addSphere(sphere, sphereRadius);
    sphere.generateNormals();
    light.pos(0, 0, 0);   // place the light
    nav().pos(0, 0, 30);  // place the viewer
    lens().far(400);      // set the far clipping plane
    background(Color(0.07));

    for (int i = 0; i < particleCount; i++) {
      Particle par(&particleList);
      particleList.push_back(par);
    }
    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    if (!simulate)
      // skip the rest of this function
      return;
    for (auto p : particleList) {
      p.flock();
      cout << "current p's acceleration is " << p.acceleration << endl;
      cout << "current p's velocity is " << p.velocity << endl;
    }
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto p : particleList) {
      p.draw(g);
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
        //   simulate = !simulate;
        break;
    }
  }
};

int main() { MyApp().start(); }