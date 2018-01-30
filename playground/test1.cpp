#include "allocore/io/al_App.hpp"
#include <cmath>
#include <iostream> // cout, cin

// using namespace std; (look in std:: for any unkonwn identifiers that follows)
using namespace al;

// definiiton
struct MyApp : App {
	// this is a "constructor" ...
	Mesh sphere;
	Material material;
	Light light;

	MyApp() {
		// ^ same name as the class we're defining
		// gets called when a "MyApp" is created..
		// auto* w = initWindow();
		initWindow();
		addSphere(sphere);
		sphere.generateNormals();
		light.pos(0, 0, 10);
		initAudio();
		// sphere.primitive(Graphics::LINES);
	}

		// this function will be called each frame
		void onDraw(Graphics& g, const Viewpoint& vp) {
			g.draw(sphere);
			light();
			material();
			// printf("GOT HERE");
		}

		// this function will be called each audio frame
		void onSound(AudioIOData& io) {
			io();
			io.out(0) = 1;
			io.out(1) = 1;
			while (io()){
				io.out(0) = 0;
				io.out(1) = 0;
			}
		}
	};

//#include <iostream>

int main(void) {
	// std::vector<int> v;
	// std::vector<int>::iterator -> auto
	// int i = 0;
	// instatiation
	MyApp app;
	app.start(); // start "blocks"
	std::cout << "sin(2) == " << std::sin(2.0) << std::endl;
	printf("%s\n", );

}
