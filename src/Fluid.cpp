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
    diffusion = 0.0001;
    viscosity = 0.0008;
    density_decay = 0.998;
    iterations = 10;
    
    cout << "Initialized " << nx << "x" << ny << " fluid (" << cells << " cells)." << endl;
    
    u.clear(); u.resize(cells, 0.0);
    v.clear(); v.resize(cells, 0.0);
    u_prev.clear(); u_prev.resize(cells, 0.0);
    v_prev.clear(); v_prev.resize(cells, 0.0);
    d.clear(); d.resize(cells, 0.0);
    d_prev.clear(); d_prev.resize(cells, 0.0);
    
}

void FluidSolver::update(){

//    v[IX(50,50)] += 500.1;
//    v[IX(50,51)] += 500.1;
    velocity_step(delta);
    density_step(d, d_prev, diffusion, delta);
//    cout << "[50,50] " << d[IX(50,50)] << endl;
//    cout << "[50,51] " << d[IX(50,51)] << endl;
}

void FluidSolver::add_velocity(int at_x, int at_y, double to_u, double to_v){
    u[IX(at_y+1,at_x+1)] += to_u;
    v[IX(at_y+1,at_x+1)] += to_v;
}
void FluidSolver::add_density(int at_x, int at_y, double to_d){
    d[IX(at_y+1,at_x+1)] += to_d;
}

void FluidSolver::density_step(vector<double>& m, vector<double>& m0, double diff, double dt){
    m.swap(m0);
    diffuse(m, m0, diff, 0, dt);
    m.swap(m0);
    advect(m, m0, u, v, 0, dt);
    decay(m, density_decay);
}

void FluidSolver::velocity_step(double dt){
    // Diffuse and fix mass-preservation
    v.swap(v_prev);
    u.swap(u_prev);
    diffuse(u, u_prev, viscosity, kBoundaryHorizontal, dt);
    diffuse(v, v_prev, viscosity, kBoundaryVertical, dt);
    project(u, v, u_prev, v_prev);
    // Advect and fix mass-preservation
    u.swap(u_prev);
    v.swap(v_prev);
    advect(u, u_prev, u_prev, v_prev, kBoundaryHorizontal, dt);
    advect(v, v_prev, u_prev, v_prev, kBoundaryVertical, dt);
    project(u, v, u_prev, v_prev);
}

void FluidSolver::diffuse(vector<double>& m, const vector<double>& m0, double diff, int boundary,  double dt){
    int i, j, k, idx;
    double a = dt * diff * nx * ny;
    int row = nx + 2;
    
    for (k = 0; k < iterations; k++) {
        for (i = 1; i <= ny; i++){
            idx = IX(i,1);
            for (j = 1; j <= nx; j++) {
                m[idx] = (m0[idx] + a * (m[idx-row] + m[idx+row] + m[idx-1] + m[idx+1])) / (1.0 + 4.0 * a);
                idx++;
            }
        }
        set_boundary(boundary, m);
    }
}

void FluidSolver::decay(vector<double>& m, double d){
    int i, j, idx;
    for (i = 1; i <= ny; i++){
        idx = IX(i,1);
        for (j = 1; j <= nx; j++) {
            m[idx] = d * m[idx];
            idx++;
        }
    }
}

void FluidSolver::advect(vector<double>& m, const vector<double>& m0, const vector<double>& fu, const vector<double>& fv, int boundary, double dt )
{
    int i, j, i0, j0, i1, j1;
    double x, y, s0, t0, s1, t1, dt0;
    
    for (i = 1; i <= ny; i++ ) {
        for (j = 1; j <= nx; j++ ) {
            x = j - dt * fu[IX(i,j)];
            y = i - dt * fv[IX(i,j)];
            if (x < 0.5) x = 0.5;
            if (x > nx + 0.5) x = nx + 0.5;
            if (y < 0.5) y = 0.5;
            if (y > ny + 0.5) y = ny + 0.5;
            i0 = (int)y; i1 = i0 + 1;
            j0 = (int)x; j1 = j0 + 1;
            s1 = y - i0; s0 = 1 - s1; t1 = x - j0; t0 = 1 - t1;
            m[IX(i,j)] = s0 * (t0 * m0[IX(i0,j0)] + t1 * m0[IX(i0,j1)])+
                         s1 * (t0 * m0[IX(i1,j0)] + t1 * m0[IX(i1,j1)]);
//            if (i > 48 && i < 52 && j > 48 && j < 52){
//                cout << "i0[" << i << "," << j << "] = " << i0 << endl;
//                cout << "x[" << i << "," << j << "] = " << x << endl;
//                cout << "y[" << i << "," << j << "] = " << y << endl;
//            }
        }
    }
    set_boundary(boundary, m);
}

void FluidSolver::project(vector<double>& u, vector<double>& v, vector<double>& p, vector<double>& div){
    int i, j, k, idx;
    int row = nx + 2;
    
    float h = 1.0 / nx;
    
    for (i = 1; i <= ny; i++){
        idx = IX(i,1);
        for (j = 1; j <= nx; j++) {
            div[idx] = -0.5 * h * (u[idx+1] - u[idx-1] + v[idx+row] - v[idx-row]);
            p[IX(i,j)] = 0;
            idx ++;
        } 
    }
    set_boundary(kBoundaryDensity, div);
    set_boundary(kBoundaryDensity, p);
    
    for (k = 0; k < iterations; k++ ) {
        for (i = 1; i <= ny; i++){
            idx = IX(i,1);
            for (j = 1; j <= nx; j++) {
                p[idx] = (div[idx] + p[idx-row] + p[idx+row] + p[idx-1] + p[idx+1]) / 4;
                idx++;
            } 
        } 
        set_boundary(kBoundaryDensity, p);
    }
    
    for (i = 1; i <= ny; i++){
        idx = IX(i,1);
        for (j = 1; j <= nx; j++) {
            u[idx] -= 0.5 * (p[idx+1] - p[idx-1])/h;
            v[idx] -= 0.5 * (p[idx+row] - p[idx-row])/h;
            idx++;
        }
    }
    set_boundary(kBoundaryHorizontal, u);
    set_boundary(kBoundaryVertical, v);
}

void FluidSolver::set_boundary(int boundary, vector<double>& m){
    int i, j;
    for (j = 1; j <= nx; j++){
        m[IX(0,j)]    = (boundary == kBoundaryVertical) ? -2 * m[IX(1,j)] : m[IX(1,j)];
        m[IX(ny+1,j)] = (boundary == kBoundaryVertical) ? -2 * m[IX(ny,j)] : m[IX(ny,j)];
    }
    for (i = 1; i <= ny; i++){
        m[IX(i,0)]    = (boundary == kBoundaryHorizontal) ? -m[IX(i,1)] : m[IX(i,1)];
        m[IX(i,nx+1)] = (boundary == kBoundaryHorizontal) ? -m[IX(i,nx)] : m[IX(i,nx)];
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