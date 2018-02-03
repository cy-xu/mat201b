#include <cassert>
#include <iostream>
#include "allocore/io/al_App.hpp"
using namespace al;
using namespace std;
static Array *start, *curr;

struct Frame : Mesh {
  Frame() {
    primitive(Graphics::LINES);
    // x-axis
    vertex(0, 0, 0);
    color(1, 1, 1);
    vertex(1, 0, 0);
    color(1, 0, 0);
    // y-axis
    vertex(0, 0, 0);
    color(1, 1, 1);
    vertex(0, 1, 0);
    color(0, 1, 0);
    // z-axis
    vertex(0, 0, 0);
    color(1, 1, 1);
    vertex(0, 0, 1);
    color(0, 0, 1);
  }
};

struct RgbMesh : public Mesh {
  Array current_Pos;
  RgbMesh() {}  
  RgbMesh(Array &array) {
    primitive(Graphics::POINTS);
    Image::RGBAPix<uint8_t> pixel;
    for (size_t row = 0; row < array.height(); ++row) {
      for (size_t col = 0; col < array.width(); ++col) {
        array.read(&pixel, col, row);
        vertex(pixel.r / 255.f, pixel.g / 255.f, pixel.b / 255.f);
        color(pixel.r / 255.f, pixel.g / 255.f, pixel.b / 255.f);
      }
    }
  }
};

struct HsvMesh : public Mesh {
  HsvMesh() {}  
  HsvMesh(Array &array) {
    primitive(Graphics::POINTS);
    Image::RGBAPix<uint8_t> pixel;
    // HSV::
    for (size_t row = 0; row < array.height(); ++row) {
      for (size_t col = 0; col < array.width(); ++col) {
        array.read(&pixel, col, row);
        // RGBtoHSV(pixel.r, pixel.g, pixel.b, h, s, v);
        HSV pixelHSV = RGB(pixel.r /255.f, pixel.g/255.f, pixel.b/255.f);
        // cout << pixelHSV[0] << " " << pixelHSV[1] << " " << pixelHSV[2] << endl;3
        vertex(pixelHSV[1] * sin(M_PI * 2.f * pixelHSV[0]), pixelHSV[1] * cos(M_PI * 2.f * pixelHSV[0]), pixelHSV[2]);
        color(pixel.r /255.f, pixel.g/255.f, pixel.b/255.f);
      }
    }
    
    // the Standard Cylinder guidance
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 20; j++){
        vertex(sin(M_PI * 2.f * i / 100.f), cos(M_PI * 2.f * i / 100.f), j / 20.f);
        color(1, 1, 1);
      }
    }
  }
};

struct DiyMesh : public Mesh {
  DiyMesh() {} 
  DiyMesh(Array &array) {
    primitive(Graphics::POINTS);
    Image::RGBAPix<uint8_t> pixel;
    for (size_t row = 0; row < array.height(); ++row) {
      for (size_t col = 0; col < array.width(); ++col) {
        array.read(&pixel, col, row);
        vertex( 4.f * sin(M_PI * 2.f * col / array.width()), row * 7.f / array.height(), 4.f * cos(M_PI * 2.f * col / array.width()));
        color(pixel.r / 255.f, pixel.g / 255.f, pixel.b / 255.f);
      }
    }
  }
};


class MyApp : public App {
  // c++ 公有继承
 public:
  Image image;
  Texture texture;
  RgbMesh cube;
  HsvMesh hsvCylinder;
  Frame frame;
  DiyMesh ring;
  int keyDown;

  MyApp() {
    // Load a .jpg file
    //
    const char* filename = "mat201b/color_spaces/office_window.jpg";
    
    //Q: Why Const? Why char not String? Why pointer?
    // Because doc says - bool al::Image::load	(	const std::string & 	filePath	)	?

    if (image.load(filename)) {
      printf("Read image from %s\n", filename);
    } else {
      printf("Failed to read image from %s!  Quitting.\n", filename);
      exit(-1);
    }
    
    // Here we copy the pixels from the image to the texture
    texture.allocate(image.array());

    cube = RgbMesh(image.array());

    ring = DiyMesh(image.array());

    hsvCylinder = HsvMesh(image.array());

    // Don't bother trying to print the image or the image's array directly
    // using C++ syntax.  This won't work:
    //
    // cout << "Image " << image << endl;
    // cout << "   Array: " << image.array() << endl;

    // Make a reference to our image's array so we can just say "array" instead
    // of "image.array()":
    //
    Array& array(image.array());

    // The "components" of the array are like "planes" in Jitter: the number of
    // data elements in each cell.  In our case three components would
    // represent R, G, B.
    //
    cout << "array has " << (int)array.components() << " components" << endl;

    // Each of these data elements is represented by the same numeric type:
    //
    cout << "Array's type (as enum) is " << array.type() << endl;

    // But that type is represented as an enum (see al_Array.h), so if you want
    // to read it use this function:
    //
    printf("Array's type (human readable) is %s\n",
           allo_type_name(array.type()));

    // The array itself also has a print method:
    //
    cout << "Array.print: " << endl << "   ";
    array.print();

    // Code below assumes this type is 8-bit unsigned integer, so this line
    // guarantees that's the case, or else crashes the program if not:
    //
    assert(array.type() == AlloUInt8Ty);

    // AlloCore's image class provides a type for an RGBA pixel, which is of
    // course templated on the numeric type used to represent each value in the
    // pixel.  Since templating happens at compile time we can't just ask the
    // array at runtime what type to put in here (hence the "assert" above):
    //
    Image::RGBAPix<uint8_t> pixel;

    // Loop through all the pixels.  Note that the columns go from left to
    // right and the rows go from bottom to top.  (So the "row" and "column"
    // are like X and Y coordinates on the Cartesian plane, with the entire
    // image living in the quadrant with positive X and positive Y --- in other
    // words the origin is in the lower left of the image.)
    //
    cout << "Display ALL the pixels !!! " << endl;

    for (size_t row = 0; row < array.height(); ++row) {
      // Q: What is size_t here?
      for (size_t col = 0; col < array.width(); ++col) {
        // read the pixel at (row, col) and print
        //
        // array.read(&pixel, row, col);
        array.read(&pixel, col, row);
        // you can ask pixel.g? (0, 255)
        
      }
    }
  }

	void onKeyDown(const Keyboard& k){

		// Use a switch to do something when a particular key is pressed
		switch(k.key()){
		// For printable keys, we just use its character symbol:
		case '1': keyDown = 1; break;
		case '2': keyDown = 2; break;
		case '3': keyDown = 3; break;
		case '4': keyDown = 4; break;
    case '5': keyDown = 5; break;

		// For non-printable keys, we have to use the enums described in the
		// Keyboard class:
		case Keyboard::RETURN: printf("Pressed return.\n"); break;
		case Keyboard::DELETE: printf("Pressed delete.\n"); break;
		case Keyboard::F1: printf("Pressed F1.\n"); break;
		}
	}

  void onDraw(Graphics& g) {

    g.pushMatrix();
    // Push the texture/quad back 5 units (away from the camera)
    g.translate(0, 0, -5);

    switch(keyDown){
      case 1: 
      {
          texture.quad(g, image.width()/200.f, image.height()/200.f, -(image.width()/400.f), -(image.height()/400.f),0); 
          g.draw(frame); 
          break;
      }
      case 2: 
      {
          g.draw(cube); 
          g.draw(frame); 
          break;
      }
      case 3: g.draw(hsvCylinder); break;
      case 4: g.translate(0, -4, 5); g.draw(ring); g.draw(frame); break;
      // case 5: cout << array. ; break;
    }

    g.popMatrix();
  }


	void onAnimate(double dt){
		// The phase will ramp from 0 to 1 over 10 seconds. We will use it to
		// animate the sphere.
		double period = 10;
		phase += dt / period;
		if(phase >= 1.) phase -= 1.
  }
};

int main() {
  MyApp app;
  app.initWindow(Window::Dim(600, 400), "imageTexture");
  app.start();

}
