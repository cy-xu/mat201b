#include "allocore/io/al_App.hpp"
// #include <iostream> // secretly included
using namespace al;
using namespace std;

int main() {
  Vec3f a(0, 0, 1), b(0, 1, 0);

  // test the dot and cross product
  //
  cout << a << endl;
  cout << b << endl;

  cout << a.dot(b) << endl;
  cout << cross(a, b) << endl; // cross product right hand rule, output (-1, 0, 0)

  Vec3f c(1, 1, 1);

  cout << a.dot(c) << endl;
  cout << cross(a, c) << endl;
  cout << a.cross(c) << endl;

  // use lerp to move 'a' half way between 'a' and 'b'
  //
  a.lerp(b, 0.5);
  cout << a << endl;
}
