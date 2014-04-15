#include "squid.h"

#include "constants.h"

#include <algorithm>

using std::cout;
using std::endl;

void Squid::setup(ofxBox2d& box2d){
    body.setPhysics(3.0, 0.5, 0.1);
    body.setup(box2d.getWorld(), kGameWidth / 2, kGameHeight / 2, 20);
}


void Squid::update(cv::Mat flow_high, bool draw_debug){
    ofPoint pos = body.getPosition();

    cv::Mat grid;
    cv::resize(flow_high, grid, cv::Size(path_grid_size.x, path_grid_size.y));
    ofPoint pos_grid = pos / kGameSize * path_grid_size;
    
    if (target_path.size()){
        body.setDamping(0.2);
        body.addForce(target_path[0] - pos, 10.0);
        if (target_path[0].distance(pos) < 30){
            target_path.clear();
        }
    } else {
        ofPoint target;
        int attempts = 5;
        while (attempts--) {
            int x = ofRandom(path_region) - (int)(path_region / 2);
            int y = ofRandom(path_region) - (int)(path_region / 2);
            x += pos_grid.x;
            y += pos_grid.y;
            if (x >= 0 && x < grid.cols && y >= 0 && y < grid.rows){
                target.x = x;
                target.y = y;
            }
            if (grid.at<uchar>(x, y) == 0){
                break;
            }
        }
        target_path.push_back(target / path_grid_size * kGameSize);
    }
    
    if (draw_debug){
        ofSetColor(255,0,0);
        ofxCv::drawMat(grid, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofCircle(target_path[0], 20);
    }
    
}


void Squid::draw(){
    ofPoint pos = body.getPosition();
    ofSetColor(255,255,255,255);
    body.draw();
    ofNoFill();

    for (int i = 0; i < target_path.size(); i ++) {
        ofSetColor(255,0,0,255);
        ofCircle(target_path[i].x, target_path[i].y, 5.0);
    }
}