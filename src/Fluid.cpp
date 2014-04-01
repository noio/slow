//
//  Fluid.cpp
//  slow
//
//  Created by Thomas van den Berg on 01/04/14.
//
//

#include "Fluid.h"

void FluidSolver::setup(int width, int height) {
    nx = width + 2;
    ny = height + 2;
    cells = nx * ny;
    
    u.clear(); u.resize(cells);
    v.clear(); v.resize(cells);
    u_prev.clear(); u_prev.resize(cells);
    v_prev.clear(); v_prev.resize(cells);
    d.clear(); d.resize(cells);
    d_prev.clear(); d_prev.resize(cells);
    
    
}

void FluidSolver::diffuse(vector<float>& m, vector<float>& m0, int boundary, float diff, float dt){
    int i, j, k;
    float a = dt * diff * cells;
    
    for (k = 0; k < 20; k++) {
        for (i = 1; i < nx; i++) {
            for (j = 1; j < ny; j++) {
                m[IX(i,j)] = (m0[IX(i,j)] + a*(m[IX(i-1,j)]+m[IX(i+1,j)]+
                                               m[IX(i,j-1)]+m[IX(i,j+1)]))/(1+4*a);
            } 
        } 
        set_bnd(boundary, m);
    }
}

void FluidSolver::advect(vector<float>& m, vector<float>& m0, vector<float>& u, vector<float>& v, int boundary, float dt )
{
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;
    
    for (i = 1; i < nx; i++ ) {
        for (j = 1; j < ny; j++ ) {
            x = i - dt * nx * u[IX(i,j)];
            y = j - dt * ny * v[IX(i,j)];
            if (x < 0.5) x = 0.5;
            if (x > nx - 1.5) x = nx - 1.5;
            i0 = (int)x; i1 = i0 + 1;
            if (y < 0.5) y = 0.5;
            if (y > ny - 1.5) y = ny - 1.5;
            j0 = (int)y; j1 = j0 + 1;
            s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
            m[IX(i,j)] = s0 * (t0 * m0[IX(i0,j0)] + t1 * m0[IX(i0,j1)])+
                         s1 * (t0 * m0[IX(i1,j0)] + t1 * m0[IX(i1,j1)]);
        }
    }
    set_bnd(boundary, m);
}

void FluidSolver::set_boundary(int boundary, vector<float>& m){
    
}
