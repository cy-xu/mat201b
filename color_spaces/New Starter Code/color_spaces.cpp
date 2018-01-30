// MAT 201B
// Color Spaces Homework
// <put your name and email here>
//
//

#include "allocore/io/al_App.hpp"
using namespace al;
using namespace std;

struct MyApp : App {
  // You might name these differently. These are just my suggestions. You could
  // probably get away with using just one mesh, but why? Always write a program
  // the simplest way first. Only try to improve a program that works. Never try
  // to improve a program that does not yet exists and work.
  //
  Mesh plane, cube, cylinder, current, target, last;

  bool shouldReset = false;

  MyApp() {
    // Choose your own image
    //
    const char* filename = "mat201b/color_spaces/photoshop.png";

    // We're putting image in the constructor because we don't need it after we
    // exctract the pixel colors and positions.
    //
    Image image;

    if (!image.load(filename)) {
      cerr << "ERROR" << endl;
      exit(1);
    }

    Array& array(image.array());
    cout << "Array.print: " << endl << "   ";
    array.print();
    assert(array.type() == AlloUInt8Ty);

    //

    Image::RGBAPix<uint8_t> pixel;

    int W = array.width();
    int H = array.height();
    for (size_t row = 0; row < H; ++row) {
      for (size_t col = 0; col < W; ++col) {
        array.read(&pixel.r, col, row);
        Color color(pixel.r / 256.0f, pixel.g / 256.0f, pixel.b / 256.0f, 0.6);
        current.color(color);
        current.vertex(col / (float)W, row / (float)H, 0);
        plane.vertex(col / (float)W, row / (float)H, 0);

        // TODO
        // right now you know everything you need to know about the image to
        // build *all* the meshes you'll need.
      }
    }

    // place the viewer back a bit, so she can see the meshes
    nav().pos(0, 0, 7);
  }

  double t = 0;
  void onAnimate(double dt) {
    // Randomly wiggles vertices; This is just to show you how to manipulate
    // mesh vertices. You should not leave this in place except MAYBE in your
    // version of the 4th state.
    //
    for (unsigned v = 0; v < current.vertices().size(); ++v)
      current.vertices()[v] +=
          Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * 0.007f;

    // shouldReset is what I might call a signal or a flag. It is a message sent
    // in the onKeyDown callback to the onAnimate callback (here). This shows
    // how you might set all the vertices in a mesh to be the same and another
    // given mesh.
    //
    if (shouldReset) {
      shouldReset = false;
      for (unsigned v = 0; v < current.vertices().size(); ++v)
        current.vertices()[v] = plane.vertices()[v];
    }

    // TODO
    //
    // use linear interpolation to animate mesh points from the last state to
    // the target state. Add code here.

    // t is what time it is now
    //
    t += dt;
  }

  void onDraw(Graphics& g) {
    // I would leave this alone, but you don't have to
    //
    g.draw(current);
  }

  void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
    // Set a flag. Use this if you want. Or delete it.
    //
    shouldReset = true;

    // Set the new target state based on which key the user presses; Do not try
    // to animate anything here. Do animation in onAnimate.
    //
    switch (k.key()) {
      default:  // always have a default case; this one "falls through" to '1'
      case '1':
        // Add code here
        break;
      case '2':
        // Add code here
        break;
      case '3':
        // Add code here
        break;
      case '4':
        // Add code here
        break;
    }
  }
};

int main() {
  MyApp app;
  app.initWindow();
  app.start();
}
