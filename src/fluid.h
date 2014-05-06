
#ifndef SLOW_FLUID_H_
#define SLOW_FLUID_H_

#include <iostream>
#include <vector>

// This indexing is different from Jos Stam's paper, here
// i = row or 'y' and j = column or 'x'.
#define IX(i,j) ((i) * (nx+2) + (j))

typedef struct Velocity
{
    double u;
    double v;
} Velocity;

using std::vector;

const int kBoundaryDensity = 0;
const int kBoundaryHorizontal = 1;
const int kBoundaryVertical = 2;


class FluidSolver
{

public:
    int nx, ny;
    int cells;

    int iterations;
    double diffusion, viscosity, delta, density_decay;
    vector<double> u, v, u_prev, v_prev, d, d_prev;

    void setup(int width, int height);
    void update();

    void add_velocity(int at_x, int at_y, double to_u, double to_v);
    void add_velocity(float at_x, float at_y, double to_u, double to_v);
    void add_density(int at_x, int at_y, double to_d);
    void add_density(float at_x, float at_y, double to_d);

    Velocity velocity_at(float x, float y);
    double density_at(float x, float y);

    void diffuse(vector<double>& m, const vector<double>& m0, double diff, int boundary, double dt);
    void advect(vector<double>& m, const vector<double>& m0, const vector<double>& fu, const vector<double>& fv, int boundary, double dt);
    void project(vector<double>& u, vector<double>& v, vector<double>& p, vector<double>& div);
    void decay(vector<double>& m, double d);

    void density_step(vector<double>& m, vector<double>& m0, double diff, double dt);
    void velocity_step(double dt);

    void set_boundary(int boundary, vector<double>& m);

    void fill_texture(unsigned char* pixels);
};

#endif /* defined(__slow__Fluid__) */
