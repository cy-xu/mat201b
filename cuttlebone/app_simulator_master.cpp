// MAT201B
// Fall 2015
// Author: Karl Yerkes
//
// Cuttlebone "App Simulator/Master and Renderer"
//

#include "Cuttlebone/Cuttlebone.hpp"
#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Ray.hpp"

#include "common.hpp"

using namespace al;

struct MyApp : App {
  Mesh ball_mesh;

  State state;
  cuttlebone::Maker<State> maker;

  MyApp() : maker("127.0.0.1") {
    addSphere(ball_mesh);
    navControl().useMouse(false);
    initWindow();
  }

  virtual void onAnimate(double dt) { maker.set(state); }

  virtual void onDraw(Graphics& g, const Viewpoint& v) {
    g.translate(state.ball_position);
    g.draw(ball_mesh);
  }

  virtual void onMouseMove(const ViewpointWindow& w, const Mouse& ball_mesh) {
    Rayd mouse_ray = getPickRay(w, ball_mesh.x(), ball_mesh.y());
    state.ball_position = mouse_ray(10.0);
  }
};

int main() {
  MyApp app;
  app.maker.start();
  app.start();
}
