#include <vector>
#include <iostream>

using namespace std;

// struct
// arrays

int main(){
  float floatArray[]{1,2,3,4,5,6};
  for (auto e : floatArray) printf("%f ", e);
  printf("\n");

  // resizable, generic
  vector<float> floatList{1,2,3,4,5,6};
  floatVecotr.push_back(100.00);
  for (auto e : floatList) printf("%f \n", 2*e);
}
