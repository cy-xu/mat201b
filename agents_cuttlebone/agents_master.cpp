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
double maxSpeed = 100;         // mock number
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

  // using *p here because we don't want to copy the actual particleList every
  // time creating an instance, so using a pointer here
  Particle(vector<Particle> *p) {
    pose.pos() = r() * initialRadius * 2;
    // this will tend to spin stuff around the y axis
    velocity = Vec3f(0, 200, 0).cross(pose.pos()).normalize(initialSpeed);
    // acceleration = Vec3f(rnd::uniform(), rnd::uniform(), rnd::uniform());
    // acceleration = r() * initialSpeed;
    acceleration = Vec3f(0, 0, 0);
    c = HSV(rnd::uniform(), 1, 1);
    // pointing the *p to *particles so we can access the actual vector
    // via *p, by accessing *particles
    particles = p;
  }

  void update() {
    // since *particles is in the Particle class, no need to bring into the
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

    // printf("in update: position %f %f %f\n", position.x, position.y,
    //        position.z);
    acceleration.zero();
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

  void applyForce(Vec3f force) { acceleration += force; }

  Vec3f separate() {
    int count = 0;
    Vec3f steer;

    for (auto other : *particles) {
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

    for (auto other : *particles) {
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

    for (auto other : *particles) {
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
    steer.normalize(maxSpeed * 6);
    applyForce(steer);
  }
};

// Target
/////////////////////////////
struct Target {
  Vec3f position;
  Color c;
  double lifetime = 0, i = 0;

  Target() {
    position = r() * initialRadius;
    Color c = RGB(0, 1, 0);
  }

  void run(double dt) {
    lifetime += dt;
    position += Vec3f(sin(M_PI * i) * 2, cos(M_PI * i), sin(M_PI * i * 2) * 2);
    i += 0.002;
    position.normalize(initialRadius * 1.5);
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
  float i = 2;
  void run(double dt) {
    lifetime += dt;
    position += Vec3f(cos(M_PI * i), sin(M_PI * i), cos(M_PI * i) * 2);
    i += 0.003;
    position.normalize(initialRadius * 1.3);
  }

  void draw(Graphics &g) {
    g.pushMatrix();
    g.translate(position);
    g.color(c);
    g.draw(lieren);
    g.popMatrix();
  }
};

// Audio
/////////////////////////////

// MyApp
/////////////////////////////
struct MyApp : App {
  Material material;
  Light light;
  bool simulate = true;

  State appState;
  cuttlebone::Maker<State> maker;

  // creating the real particleList, it's now empty
  vector<Particle> particleList;
  vector<Vec3f> particlePositions;
  FakeVector parPositions;

  // vector<Vec3f> particleColors;

  Target mubiaoOne;
  Predator lierenOne;

  MyApp() : maker("127.0.0.1") {
    addCone(yuanqiu, sphereRadius, Vec3f(0, 0, sphereRadius * 3), 16, 1);
    yuanqiu.generateNormals();

    addSphere(mubiao, sphereRadius, 16);
    mubiao.generateNormals();

    addSphere(lieren, sphereRadius * 2, 16);
    lieren.generateNormals();

    light.pos(0, 10, 10);  // place the light
    // light.diffuse();
    nav().pos(0, 0, 30);  // place the viewer
    lens().far(400);      // set the far clipping plane
    background(Color(0.07));

    // pushing every Particle instance into the actual list
    for (int i = 0; i < particleCount; i++) {
      Particle par(&particleList);
      particleList.push_back(par);
      // particleColors.push_back(par.c);
      particlePositions.push_back(par.pose.pos());
    }

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    if (!simulate)
      // skip the rest of this function
      return;

    // light position
    light.pos(mubiaoOne.position);

    // Target animation
    mubiaoOne.run(dt);
    // send target position
    appState.targetPosition = mubiaoOne.position;

    // Predator animation
    lierenOne.run(dt);
    // send Predator position
    appState.predatorPosition = lierenOne.position;

    // Particle animation
    for (int i = 0; i < particleList.size(); ++i) {
      particleList[i].update();
      particleList[i].pose.faceToward(mubiaoOne.position);
      particlePositions[i] = particleList[i].pose.pos();

      // only sense target nearby
      Vec3f difference = particleList[i].pose.pos() - mubiaoOne.position;
      float d = difference.mag();
      if (d < 50 * sphereRadius) {
        particleList[i].seekTarget(Vec3d(mubiaoOne.position));
      }

      // run away from predator
      Vec3f difference2 = particleList[i].pose.pos() - lierenOne.position;
      float d2 = difference2.mag();
      if (d2 < 20 * sphereRadius) {
        particleList[i].runAway(lierenOne.position);
      }
    }
    // send all particle position
    appState.parPositions.fill_stuff(particlePositions);

    // limiting paritcle acceleration
    // unsigned limitCount = 0;
    // for (int i = 0; i < particleList.size(); ++i) {
    //   if (particleList[i].acceleration.mag() > maxAcceleration) {
    //     particleList[i].acceleration.normalize(maxAcceleration);
    //     limitCount++;
    //   }
    // }
    // printf("%u of %lu limited\n", limitCount, particleList.size());

    // send data to common
    maker.set(appState);

    // nav().faceToward(mubiaoOne.position);
  }

  void onDraw(Graphics &g) {
    material();
    light();
    g.scale(scaleFactor);
    for (auto par : particleList) {
      par.draw(g);
    }
    mubiaoOne.draw(g);
    lierenOne.draw(g);
  }

  void onKeyDown(const ViewpointWindow &, const Keyboard &k) {
    float targetPosition = 10;
    switch (k.key()) {
      default:
      case 'u':
        mubiaoOne.position.y += targetPosition;
        break;
      case 'j':
        mubiaoOne.position.y -= targetPosition;
        break;
      case 'h':
        mubiaoOne.position.x -= targetPosition;
        break;
      case 'k':
        mubiaoOne.position.x += targetPosition;
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
};

int main() {
  MyApp app;
  app.maker.start();
  app.start();
}