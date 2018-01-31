// MAT 201B
// Color Spaces Homework
// Chengyuan Xu, cxu@ucsb.edu
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
  Mesh plane, cube, cylinder, current, target, last, personal, temp;
  bool shouldReset = false;
  double elapsedTime = 0;

  MyApp() {
    // Choose your own image
    //
    const char* filename = "mat201b/color_spaces/office_window.jpg";

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
        last.vertex(col / (float)W, row / (float)H, 0);
        target.vertex(col / (float)W, row / (float)H, 0);
        temp.vertex(col / (float)W, row / (float)H, 0);

        // The CUBE
        // cube.primitive(Graphics::POINTS);
        cube.vertex(pixel.r / 255.f, pixel.g / 255.f, pixel.b / 255.f);

        // The Cylinder
        // // cylinder.primitive(Graphics::POINTS);
        HSV pixelHSV = RGB(pixel.r /255.f, pixel.g/255.f, pixel.b/255.f);
        cylinder.vertex(pixelHSV[1] * sin(M_PI * 2.f * pixelHSV[0]), pixelHSV[1] * cos(M_PI * 2.f * pixelHSV[0]), pixelHSV[2]);
        // cylinder.color(pixel.r /255.f, pixel.g/255.f, pixel.b/255.f);
        
        // The Personal
        personal.vertex( 4.f * sin(M_PI * 2.f * col / array.width()), row * 7.f / array.height(), 4.f * cos(M_PI * 2.f * col / array.width()));
        // target.color(pixel.r / 255.f, pixel.g / 255.f, pixel.b / 255.f);

        // TODO
        // right now you know everything you need to know about the image to
        // build *all* the meshes you'll need.
      }
    }

    // place the viewer back a bit, so she can see the meshes
    nav().pos(0, 0, 7);
  }

  double t = 0;
  double startTime = 0;
  void onAnimate(double dt) {
    // Randomly wiggles vertices; This is just to show you how to manipulate
    // mesh vertices. You should not leave this in place except MAYBE in your
    // version of the 4th state.
    //
    // for (unsigned v = 0; v < current.vertices().size(); ++v)
    //   current.vertices()[v] +=
    //       Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * 0.007f;

    // shouldReset is what I might call a signal or a flag. It is a message sent
    // in the onKeyDown callback to the onAnimate callback (here). This shows
    // how you might set all the vertices in a mesh to be the same and another
    // given mesh.
    //

    // t is what time it is now
    //
  
    t += dt;
    elapsedTime = t - startTime;

      if (shouldReset) {
        // t += dt;
          for (unsigned v = 0; v < current.vertices().size(); ++v) {
            temp.vertices()[v] = last.vertices()[v];
            temp.vertices()[v].lerp(target.vertices()[v], elapsedTime);
            current.vertices()[v] = temp.vertices()[v];
          } 
        if (elapsedTime > 1) {shouldReset = false;}
      }

    // TODO
    //
    // use linear interpolation to animate mesh points from the last state to
    // the target state. Add code here.

  }

  void onDraw(Graphics& g) {
    // I would leave this alone, but you don't have to
    //
    g.draw(current);
  }

  void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
    // Set the new target state based on which key the user presses; Do not try
    // to animate anything here. Do animation in onAnimate.
    //
    switch (k.key()) {
      default:  // always have a default case; this one "falls through" to '1'
      case '1':
        for (unsigned v = 0; v < current.vertices().size(); ++v) {
            target.vertices()[v] = plane.vertices()[v];
            last.vertices()[v] = current.vertices()[v];
        }
        startTime = t;
        shouldReset = true;
        break;
      case '2':
        for (unsigned v = 0; v < current.vertices().size(); ++v) {
          target.vertices()[v] = cube.vertices()[v];
          last.vertices()[v] = current.vertices()[v];
        }
        startTime = t;
        shouldReset = true;
        break;
      case '3':
        for (unsigned v = 0; v < current.vertices().size(); ++v) {
          target.vertices()[v] = cylinder.vertices()[v];
          last.vertices()[v] = current.vertices()[v];
        }
        startTime = t;
        shouldReset = true;
        break;
      case '4':
        for (unsigned v = 0; v < current.vertices().size(); ++v) {
          target.vertices()[v] = personal.vertices()[v];
          last.vertices()[v] = current.vertices()[v];
        }
        startTime = t;
        shouldReset = true;
        break;
    }
  }
};

int main() {
  MyApp app;
  app.initWindow();
  app.start();
}
