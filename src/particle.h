#pragma once

#include "ofGraphics.h"

class Particle
{
public:
    float x, y, a;
    float xv, yv, av;
    float xf, yf;
    float damping;
    float life;
    ofColor color;
    bool alive;
    char layer;

    Particle(float _x = 0, float _y = 0, float _a = 0,
             float _xv = 0, float _yv = 0, float _av = 0,
             float _damping = 1.0,
             float _life = 10.0, char _layer = 0) :
        x(_x), y(_y), a(_a),
        xv(_xv), yv(_yv), av(_av),
        damping(_damping),
        life(_life), alive(true), layer(_layer)
    {
        resetForce();
    }
    void update(float timeStep)
    {
        // f = ma, m = 1, f = a, v = int(a)
        xv += xf;
        yv += yf;
        xv *= damping;
        yv *= damping;
        av *= damping;
        x += xv * timeStep;
        y += yv * timeStep;
        a += av * timeStep;
        life -= timeStep;
        if (life < 0)
        {
            alive = false;
        }
    }
    void resetForce()
    {
        xf = 0;
        yf = 0;
    }
    void bounceOffWalls(float left, float top, float right, float bottom, float damping = .3)
    {
        bool collision = false;
        if (x > right)
        {
            x = right;
            xv *= -1;
            collision = true;
        }
        else if (x < left)
        {
            x = left;
            xv *= -1;
            collision = true;
        }
        if (y > bottom)
        {
            y = bottom;
            yv *= -1;
            collision = true;
        }
        else if (y < top)
        {
            y = top;
            yv *= -1;
            collision = true;
        }
        if (collision == true)
        {
            xv *= damping;
            yv *= damping;
        }
    }
    void addDampingForce(float damping = .01)
    {
        xf = xf - xv * damping;
        yf = yf - yv * damping;
    }
    void draw()
    {
        glVertex2f(x, y);
    }
};
