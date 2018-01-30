#include "allocore/io/al_App.hpp"

using namespace al;
using namespace std;

struct TriApp : App {
    Mesh triangle;
    TriApp() {
        initWindow();

        triangle.vertex(-1,0,0);
        triangle.vertex(1,0,0);
        triangle.vertex(0,1,0);
        triangle.primitive(Graphics::TRIANGLES);
        nav().pos(0,0,10);

        cout << "this is a sentence." << endl;
    }
    void onAnimate(double dt){
        //
        nav().faceToward(Vec3f(2,2,2), 0.01);
        // nav().nudgeToward(Vec3f(0,0,-10), 0.01);
    }
    void onDraw(Graphics& g) { 
        g.draw(triangle);
     }
    
};

int main(){
    TriApp a;
    a.start();
}