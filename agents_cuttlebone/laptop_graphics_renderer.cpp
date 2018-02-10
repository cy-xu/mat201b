// MAT201B
// Fall 2015
// Author: Karl Yerkes
//
// Cuttlebone "Laptop Graphics Renderer"
//

#include "Cuttlebone/Cuttlebone.hpp"
#include "allocore/io/al_App.hpp"

#include "common.hpp"

using namespace al;

struct MyApp : App {
  Mesh ball_mesh;

  State state;
  cuttlebone::Taker<State> taker;

  MyApp() {
    addSphere(ball_mesh);
    navControl().useMouse(false);
    initWindow();
  }

  virtual void onAnimate(double dt) { taker.get(state); }

  virtual void onDraw(Graphics& g, const Viewpoint& v) {
    g.translate(state.ball_position);
    g.draw(ball_mesh);
  }
};

int main() {
  MyApp app;
  app.taker.start();
  app.start();
}
