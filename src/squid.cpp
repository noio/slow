
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
    grid.convertTo(grid, CV_32F, 1 / 255.0f);
    ofxCv::dilate(grid, 1);
    grid = 1.0f - grid;
    cv::threshold(grid, grid, 0.2, 1.0, CV_THRESH_TOZERO);
    cv::resize(grid, sections, kSectionsSize, 0, 0,  CV_INTER_AREA);
    ofxCv::blur(sections, 1);
    //
    // Pick a goal, first check if current area or goal is crowded.
    cv::Rect local_area = cv::Rect(pos_grid.x - 2, pos_grid.y - 2, 5, 5);
    local_area = local_area & cv::Rect(cv::Point(0, 0), kPathGridSize);
    float local_motion = cv::sum(1.0f - grid(local_area))[0];
    if (local_motion > 0.0f)
    {
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        minMaxLoc( sections, &minVal, &maxVal, &minLoc, &maxLoc );
        if (goal_section != maxLoc)
        {
            goal_section = maxLoc;
            ofPoint goal = (ofxCv::toOf(goal_section) + ofPoint(ofRandomuf(), ofRandomuf())) / kSectionsSize * kPathGridSize;
            goal_x = goal.x;
            goal_y = goal.y;
        }
    }
    //
    // Do the actual pathfinding
    // Flip the grid and turn it into an ofPixels
    ofxCv::copy(grid, grid_im);
    pathfinder.setup(grid_im);
    float path_length = pathfinder.find(pos_grid.x, pos_grid.y, goal_x, goal_y);
    pathfinder.path.simplify(0.5f);
    //
    // Low level planning on path
    if (pathfinder.path.size() > 1)
    {
        body.setDamping(0.2);
        ofPoint next = (pathfinder.path[pathfinder.path.size() - 2] + 0.5) / kPathGridSize * kGameSize;
        body.addForce((next - pos).normalized(), 2000.0);
    }
    if (draw_debug)
    {
        // Draw the grids
        ofSetColor(255, 0, 255, 255);
        ofxCv::drawMat(grid, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofSetColor(0, 255, 0, 255);
//        ofxCv::drawMat(sections, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        // Draw the path
        ofSetColor(0, 255, 255, 255);
        ofSetLineWidth(4);
        ofPushMatrix();
        ofScale(kScreenWidth / kPathGridSize.width, kScreenHeight / kPathGridSize.height);
        ofRectangle debug_local_area = ofxCv::toOf(local_area);
        ofRect(debug_local_area);
        ofTranslate(0.5, 0.5);
        pathfinder.path.draw();
        if (path_length == 0)
        {
            ofSetColor(255, 0, 0);
        }
        ofCircle(goal_x, goal_y, 0.2f);
        ofPopMatrix();
    }
}


void Squid::draw()
{
    ofPoint pos = body.getPosition();
    ofSetColor(255, 255, 255, 255);
    body.draw();
}