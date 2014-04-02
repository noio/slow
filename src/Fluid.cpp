//
//  Fluid.cpp
//  slow
//
//  Created by Thomas van den Berg on 01/04/14.
//  Following the famous Jos Stam paper.
//

#include "Fluid.h"

using std::cout;
using std::endl;

void FluidSolver::setup(int width, int height) {
    nx = width;
    ny = height;
    cells = (nx + 2) * (ny + 2);
    
    delta = 1/30.0;
    diffusion = 0.01;
    viscosity = 0.001;
    
    cout << "Initialized " << nx << "x" << ny << " fluid (" << cells << " cells)." << endl;
    
    u.clear(); u.resize(cells, 0.0);
    v.clear(); v.resize(cells, 0.0);
    u_prev.clear(); u_prev.resize(cells, 0.0);
    v_prev.clear(); v_prev.resize(cells, 0.0);
    d.clear(); d.resize(cells, 0.0);
    d_prev.clear(); d_prev.resize(cells, 0.0);
    
}

void FluidSolver::update(){
    d[IX(50,50)] += 1.0;
    velocity_step(delta);
    density_step(d, d_prev, diffusion, delta);
}

void FluidSolver::density_step(vector<double>& m, vector<double>& m0, double diff, double dt){
    m.swap(m0);
    diffuse(m, m0, diff, 0, dt);
    m.swap(m0);
//    advect(m, m0, u, v, 0, dt);
}

void FluidSolver::velocity_step(double dt){
    // Diffuse and fix mass-preservation
    v.swap(v_prev);
    u.swap(u_prev);
    diffuse(v, v_prev, viscosity, 1, dt);
    diffuse(u, u_prev, viscosity, 2, dt);
    project(u, v, u_prev, v_prev);
    // Advect and fix mass-preservation
    u.swap(u_prev);
    v.swap(v_prev);
    advect(u, u_prev, u_prev, v_prev, 1, dt); // TODO Make constants out of these magic 1's and 2's
    advect(v, v_prev, u_prev, v_prev, 2, dt);
    project(u, v, u_prev, v_prev);
}

void FluidSolver::diffuse(vector<double>& m, const vector<double>& m0, double diff, int boundary,  double dt){
    int i, j, k;
    double a = dt * diff * nx * ny;
    
    // TODO optimize this loop: don't use IX all over the place.
    for (k = 0; k < 20; k++) {
        for (i = 1; i <= ny; i++) {
            for (j = 1; j <= nx; j++) {
                m[IX(i,j)] = (m0[IX(i,j)] + a * (m[IX(i-1,j)] + m[IX(i+1,j)]+
                                                 m[IX(i,j-1)] + m[IX(i,j+1)])) / (1.0 + 4.0 * a);
            } 
        } 
        set_boundary(boundary, m);
    }
}

void FluidSolver::advect(vector<double>& m, const vector<double>& m0, const vector<double>& fu, const vector<double>& fv, int boundary, double dt )
{
    int i, j, i0, j0, i1, j1;
    double x, y, s0, t0, s1, t1, dt0;
    
    for (i = 1; i <= ny; i++ ) {
        for (j = 1; j <= nx; j++ ) {
            // TODO take out the multiplication by nx and ny here.
            // Make velocities in "cells/second" because this is FUCKED UP YO.
            x = j - dt * nx * fu[IX(i,j)];
            y = i - dt * ny * fv[IX(i,j)];
            if (x < 0.5) x = 0.5;
            if (x > nx - 1.5) x = nx - 1.5;
            if (y < 0.5) y = 0.5;
            if (y > ny - 1.5) y = ny - 1.5;
            i0 = (int)y; i1 = i0 + 1;
            j0 = (int)x; j1 = j0 + 1;
            s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
            m[IX(i,j)] = s0 * (t0 * m0[IX(i0,j0)] + t1 * m0[IX(i0,j1)])+
                         s1 * (t0 * m0[IX(i1,j0)] + t1 * m0[IX(i1,j1)]);
        }
    }
    set_boundary(boundary, m);
}

void FluidSolver::project(vector<double>& u, vector<double>& v, vector<double>& p, vector<double>& div){
    int i, j, k;
    
    double h = 1.0 / nx; // TODO THIS IS FUCKED UP YO!
    
    for (i = 1; i <= ny; i++) {
        for (j = 1; j <= nx; j++) {
            div[IX(i,j)] = -0.5 * h * (u[IX(i+1,j)] - u[IX(i-1,j)]+
                                       v[IX(i,j+1)] - v[IX(i,j-1)]);
            p[IX(i,j)] = 0;
        } 
    }
    set_boundary(0, div);
    set_boundary(0, p);
    
    for (k = 0; k < 20; k++ ) {
        for (i = 1; i <= ny; i++ ) { // TODO loop optimizing
            for (j = 1; j <= nx; j++ ) {
                p[IX(i,j)] = (div[IX(i,j)] + p[IX(i-1,j)] + p[IX(i+1,j)] +
                                             p[IX(i,j-1)] + p[IX(i,j+1)]) / 4;
            } 
        } 
        set_boundary(0, p);
    }
    
    for (i = 1; i <= ny; i++) {
        for (j = 1; j <= nx; j++ ) {
            u[IX(i,j)] -= 0.5 * (p[IX(i+1,j)] - p[IX(i-1,j)])/h;
            v[IX(i,j)] -= 0.5 * (p[IX(i,j+1)] - p[IX(i,j-1)])/h;
        }
    }
    set_boundary(1, u);
    set_boundary(2, v);
}

void FluidSolver::set_boundary(int boundary, vector<double>& m){
    int i, j;
    for (j = 1; j <= nx; j++){
        m[IX(0,j)]    = (boundary == 1) ? -m[IX(1,j)] : m[IX(1,j)];
        m[IX(ny+1,j)] = (boundary == 1) ? -m[IX(ny,j)] : m[IX(ny,j)];
    }
    for (i = 1; i <= ny; i++){
        m[IX(i,0)]    = (boundary == 2) ? -m[IX(i,1)] : m[IX(i,1)];
        m[IX(i,nx+1)] = (boundary == 2) ? -m[IX(i,nx)] : m[IX(i,nx)];
    }
    m[IX(0,0)]    = 0.5 * (m[IX(1, 0)] + m[IX(0, 1)]);
    m[IX(0,nx+1)] = 0.5 * (m[IX(1, nx+1)] + m[IX(0, nx)]);
    m[IX(ny+1,0)] = 0.5 * (m[IX(ny+1, 1)] + m[IX(ny, 0)]);
    m[IX(ny+1,nx+1)] = 0.5 * (m[IX(ny,nx+1)] + m[IX(ny+1,nx)]);
}

void FluidSolver::fill_texture(unsigned char * pixels){
    int i, j, idx;
    idx = 0;
    for (i = 1; i <= ny; i++ ) {
        for (j = 1; j <= nx; j++ ) {
//            if (d[IX(i, j)] != 0)
//                cout << d[IX(i, j)] << endl;
            pixels[idx++] = (int)(std::min(255.0, d[IX(i,j)] * 255.0));
        }
    }
}