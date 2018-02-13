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
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"

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

  void update() {
    // since *particles is in the Particle class, no need to bring into the
    // functions.
    Vec3f sep = separate();
    Vec3f ali = align();
    Vec3f coh = cohesion();

    // 4 * sphereRadius, 10 * sphereRadius, 30 * sphereRadius
    // 1.0 , 1.0 , 1.0 is an interesting stable combination

    sep = sep * 6.0f;
    ali = ali * 1.0f;
    coh = coh * 0.1f;

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
    g.rotate(pose.quat().inverse());
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
      if (d > 0 && d < 12 * sphereRadius) {
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
      if (d > 0 && d < 12 * sphereRadius) {
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
    steer.normalize(maxSpeed * 3);
    applyForce(steer);
  }
};

struct Target {
  Vec3f position, velocity, acceleration, desired, steer;
  Color c;
  double lifetime = 0, i = 0;

  Target() {
    position = r() * initialRadius;
    velocity = Vec3f(0, 0, 0);
    acceleration = Vec3f(0, 0, 0);
    Color c = HSV(1, 1, 1);
  }

  void update() {
    velocity += acceleration * timeStep;
    position += velocity * timeStep;
    // acceleration.zero();
  }

  void applyForce(Vec3f force) { acceleration += force; }

  void run(double dt) {
    lifetime += dt;
    position += Vec3f(sin(M_PI * i), cos(M_PI * i), sin(M_PI * i * 2));
    i += 0.005;
    position.normalize(initialRadius);
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
  int targetNo = 1;

  State appState;
  cuttlebone::Maker<State> maker;

  // creating the real particleList, it's now empty
  vector<Particle> particleList;
  vector<Vec3f> particlePositions;
  vector<Color> particleColors;

  Target mubiaoOne;

  MyApp() : maker("127.0.0.1") {
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
      particleColors.push_back(par.c);
      particlePositions.push_back(par.pose.pos());
    }
    // push the initial particles status to state
    appState.particleColors = particleColors;
    appState.particlePositions = particlePositions;

    // mouse control
    navControl().useMouse(false);

    initWindow();
    initAudio();
  }

  void onAnimate(double dt) {
    if (!simulate)
      // skip the rest of this function
      return;

    // send data to common
    maker.set(appState);

    // Target animation
    mubiaoOne.run(dt);
    mubiaoOne.update();
    if (mubiaoOne.lifetime > 5) {
      mubiaoOne.position = r() * initialRadius;
      mubiaoOne.run(dt);
      mubiaoOne.update();
      mubiaoOne.lifetime = 0;
      cout << "New Target generated." << endl;
    }

    appState.targetPosition = mubiaoOne.position;

    for (int i = 0; i < particleList.size(); ++i) {
      // maker.set(appState);
      particleList[i].pose.faceToward(mubiaoOne.position);
      particleList[i].update();
      particleList[i].seekTarget(mubiaoOne.position);
      particlePositions[i] = particleList[i].pose.pos();
    }

    appState.particlePositions = particlePositions;

    unsigned limitCount = 0;
    for (int i = 0; i < particleList.size(); ++i) {
      if (particleList[i].acceleration.mag() > maxAcceleration) {
        particleList[i].acceleration.normalize(maxAcceleration);
        limitCount++;
      }
    }
    // printf("%u of %lu limited\n", limitCount, particleList.size());

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
  }

  void onKeyDown(const ViewpointWindow &, const Keyboard &k) {
    float targetPosition = 5;
    switch (k.key()) {
      default:
      case ';':
        mubiaoOne.position.y += targetPosition;
        break;
      case '"':
        mubiaoOne.position.y -= targetPosition;
        break;
      case '.':
        mubiaoOne.position.x -= targetPosition;
        break;
      case '/':
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