// C++ syntax
#include <iostream>

// C syntax
// #include <stdio.h>
// #include <cstdio>
#include <stdlib.h>
#include <time.h> 

int main() {
    int d1,d2;

    srand(time(0));

    d1 = 1 + rand() % 6;
    d2 = 1 + rand() % 6;

    std::cout << d1 << d2 << std::endl;

    return 0;
}