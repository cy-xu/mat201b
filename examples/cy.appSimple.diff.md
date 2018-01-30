**cyxu$ diff appSimple.cpp cy.appSimple.cpp**

``` c++
49a50,52
> 
> Modified by CY Xu for practice purpose
> Jan 22, 2017
60a64,65
> 	Mesh mesh2;
> //	Mesh mesh3;
67c72,74
< 		addSphere(mesh);
---
> 		addSphere(mesh, 1.0, 64, 64);
> 		addSphere(mesh2, 0.6, 16, 16);
> //		addCube(mesh3); // I find these functions in al_Shapes.hpp
118c125,126
< 		double period = 10;
---
> 		dt = 0.1; // Question: What is dt? And at which setp do we assign the value? So I force assign it's value here to see the difference.
> 		double period = 20; // Change value to 20 make rotation slower 
134c142
< 		g.polygonMode(Graphics::LINE); // wireframe mode
---
> 		g.polygonMode(Graphics::POINT); // wireframe mode // found this function in al_Graphics.hpp, changed to point mode, but couldn't find .cpp file and couldn't understand how it functions. 
136c144
< 		g.rotate(phase*360, 0,1,0);
---
> 		g.rotate(phase*360, 180,180,180);// changed rotation angles
137a146,151
> 		g.popMatrix(); // starting to understand why using stack here, so all mesh points are pushed and popped between each frame?
> 
> 		g.polygonMode(Graphics::LINE); // Added second polygon g, why can they have the same name yet still function?
> 		g.pushMatrix();
> 		g.rotate(phase*360, 0,-180,0);// changed rotation angles
> 		g.draw(mesh2); // draw the second Mesh, mesh2
188c202
< }
---
> }
\ No newline at end of file
```