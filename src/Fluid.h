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

// This indexing is different from Jos Stam's paper, here
// i = row or 'y' and j = column or 'x'.
#define IX(i,j) ((i) * (nx+2) + (j))

using std::vector;

class FluidSolver {
    
public:
    int nx, ny;
    int cells;
    
    double diffusion, viscosity, delta;
    vector<double> u, v, u_prev, v_prev, d, d_prev;
    
    void setup(int width, int height);
    void update();
    
    void diffuse(vector<double>& m, const vector<double>& m0, double diff, int boundary, double dt);
    void advect(vector<double>& m, const vector<double>& m0, const vector<double>& fu, const vector<double>& fv, int boundary, double dt);
    void project(vector<double>& u, vector<double>& v, vector<double>& p, vector<double>& div);
    
    void density_step(vector<double>& m, vector<double>& m0, double diff, double dt);
    void velocity_step(double dt);
    
    void set_boundary(int boundary, vector<double>& m);
    
    void fill_texture(unsigned char * pixels);
};

#endif /* defined(__slow__Fluid__) */
