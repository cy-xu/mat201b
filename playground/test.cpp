#include "allocore/io/al_App.hpp"

using namespace al;

struct MyApp : App {
	// this is a "constructor" ...
	MyApp() {
		// ^ same name as the class we're defining
		// gets called when a "MyApp" is created..
	auto* w = initWindow();
	auto* x = initWindow();
	}

	// tis is a ""destructor"...
	~MyApp() {
		// ^ smae name as the class we are defining
		// gets called when a "MyApp" is being destroyed..
		printf("MyApp destroyed");
	}
};

//#include <iostream>

int main(void) {
	//instatiation
	MyApp app;
	app.start(); // start "blocks"
}
