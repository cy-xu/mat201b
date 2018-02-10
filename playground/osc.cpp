#include "allocore/io/al_App.hpp"
using namespace al;
using namespace std;

struct MyApp : App, osc::PacketHandler {
  Mesh theMesh;
  Pose meshPose;
  Material material;
  Light light;

  // void onMessage(osc::Message& m) {
  //   m.print();

  //   // the message we'll receive consists of a single float
  //   float v;
  //   m >> v;

  //   printf("v = %f\n", v);
  //   // 192.168.1.138
  // }

  void onMessage(osc::Message& m) {
    if (m.addressPattern() == "/XYZ") {
      double x, y, z;
      m >> x;
      m >> y;
      m >> z;
      meshPose.pos(x, y, z);
    } else if (m.addressPattern() == "/XYZW") {
      double x, y, z, w;
      m >> x;
      m >> y;
      m >> z;
      m >> w;
      meshPose.quat() = Quatd(x, y, z, w);
    } else
      m.print();
  }

  MyApp() {
    initWindow();
    initAudio();
    // I listen on 60777
    oscRecv().open(60777, "", 0.016, Socket::UDP);
    oscRecv().handler(*this);
    oscRecv().start();
    // You better be listening on 60778
    oscSend().open(60778, "192.168.1.138", 0.016, Socket::UDP);

    addCone(theMesh, 2, Vec3f(0, 0, 9));
    theMesh.generateNormals();
    light.pos(0, 0, 0);
    nav().pos(0, 0, 20);
  }

  void onAnimate(double dt) {
    oscSend().send("/xyz", nav().pos().x, nav().pos().y, nav().pos().z);
    oscSend().send("/xyzw", nav().quat().x, nav().quat().y, nav().quat().z,
                   nav().quat().w);
    oscSend().send("/name", "from CY Xu");
    // cout << t << endl;
    // }
  }

  void onDraw(Graphics& g) {
    g.pushMatrix();
    g.translate(meshPose.pos());
    g.rotate(meshPose.quat());
    g.color(RGB(0, 1, 0));
    g.draw(theMesh);
    g.popMatrix();
  }

  void onSound(AudioIO& io) {
    while (io()) {
      io.out(0) = 0;
      io.out(1) = 0;
    }
  }

  void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
    switch (k.key()) {
      default:
      case '1':
        break;
      case '2':
        break;
      case '3':
        break;
      case '4':
        break;
    }
  }
};

int main() { MyApp().start(); }
