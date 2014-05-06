
////////// INCLUDES //////////
#include "squid.h"

#include <algorithm>

#include "ofxCv.h"

#include "constants.h"
#include "utilities.h"

using std::cout;
using std::endl;

////////// SQUID CLASS //////////

void Squid::setup(ofxBox2d& box2d)
{
    // Set up physics body
    body.setPhysics(3.0, 0.5, 0.1);
    body.setup(box2d.getWorld(), kGameWidth / 2, kGameHeight / 2, 20);
}

/*
 This function consists of the following steps:
 - Check & update the current goal
 - Plan a path to goal
 - Move to goal (low level)
 */
void Squid::update(cv::Mat flow_high, bool draw_debug)
{
    ofPoint pos = body.getPosition();
    ofPoint pos_grid = pos / kGameSize * kPathGridSize;
    //
    // Subsample the flow grid to get a pathfinding grid
    cv::resize(flow_high, grid, kPathGridSize, 0, 0, CV_INTER_AREA);
    grid.convertTo(grid, CV_32F, 1/255.0f);
    grid = 1.0f - grid;
    cv::threshold(grid, grid, 0.2, 1.0, CV_THRESH_TOZERO);
    cv::resize(grid, target_grid, kPathGridSize, 0, 0,  CV_INTER_AREA);
    //
    // Pick a goal, first check if current area or goal is crowded.
    cv::Rect local_area = cv::Rect(pos_grid.x - 1, pos_grid.y - 1, 2, 2);
    local_area = local_area & cv::Rect(cv::Point(0,0), kTargetGridSize);
    
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    minMaxLoc( target_grid, &minVal, &maxVal, &minLoc, &maxLoc );
    
    cout << "min val : " << minVal << endl;
    cout << "max val: " << maxVal << endl;
    
    // Flip the grid and turn it into an ofPixels to do pathfinding
    ofxCv::copy(grid, grid_im);
    // Do the actual pathfinding
    pathfinder.setup(grid_im);
    pathfinder.find(pos_grid.x, pos_grid.y, 15, 7);
    pathfinder.path.simplify(1.0f);
    
    
    if (target_path.size())
    {
        body.setDamping(0.2);
        body.addForce(target_path[0] - pos, 10.0);
        if (target_path[0].distance(pos) < 30)
        {
            target_path.clear();
        }
    }
    else
    {
        ofPoint target;
        int attempts = 5;
        while (attempts--)
        {
            int x = ofRandom(path_region) - (int)(path_region / 2);
            int y = ofRandom(path_region) - (int)(path_region / 2);
            x += pos_grid.x;
            y += pos_grid.y;
            if (x >= 0 && x < grid.cols && y >= 0 && y < grid.rows)
            {
                target.x = x;
                target.y = y;
            }
            if (grid.at<uchar>(x, y) == 0)
            {
                break;
            }
        }
        target_path.push_back(target / kPathGridSize * kGameSize);
    }
    if (draw_debug)
    {
        // Draw the grids
        ofSetColor(255, 0, 255, 128);
        ofxCv::drawMat(grid, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofSetColor(0,255,0,128);
        ofxCv::drawMat(target_grid, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
//        grid_im.draw(0,0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofCircle(target_path[0], 20);
        // Draw the path
        ofSetColor(0, 255, 255, 255);
        ofSetLineWidth(4);
        ofPushMatrix();
        ofScale(kScreenWidth / kPathGridSize.width, kScreenHeight / kPathGridSize.height);
        ofTranslate(0.5, 0.5);
        pathfinder.path.draw();
        ofPopMatrix();
    }
}


void Squid::draw()
{
    ofPoint pos = body.getPosition();
    ofSetColor(255, 255, 255, 255);
    body.draw();
    ofNoFill();
    for (int i = 0; i < target_path.size(); i ++)
    {
        ofSetColor(255, 0, 0, 255);
        ofCircle(target_path[i].x, target_path[i].y, 5.0);
    }
}