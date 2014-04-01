//
//  Fluid.h
//  slow
//
//  Created by Thomas van den Berg on 01/04/14.
//
//

#ifndef SLOW_FLUID_H_
#define SLOW_FLUID_H_

#include <iostream>
#include <vector>

#define IX(i,j) ((i) + (nx) * (j))

using std::vector;

class FluidSolver {
    
public:
    int nx, ny;
    int cells;
    
    vector<float> u, v, u_prev, v_prev, d, d_prev;
    
    void setup(int width, int height);
    void update();
    
    void diffuse(vector<float>& x, vector<float>& x0, int boundary, float diff, float dt);
    void advect (vector<float>& m, vector<float>& m0, vector<float>& u, vector<float>& v, int boundary, float dt )
    
    void set_boundary(int boundary, vector<float>& m);
};

#endif /* defined(__slow__Fluid__) */
