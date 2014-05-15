
////////// INCLUDES //////////
#include "squid.h"
#include "constants.h"

#include "ofxCv.h"

#include <algorithm>



using std::cout;
using std::endl;

////////// SQUID CLASS //////////

void Squid::setup(ofPtr<b2World> phys_world)
{
    // Set up physics body
    ofPoint pos = ofPoint(kGameWidth / 2, kGameHeight / 2);
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = ofToB2(pos);
    bodyDef.linearDamping = 2.0;
    body = phys_world->CreateBody(&bodyDef);
    b2CircleShape shape;
    shape.m_radius = kBodyRadius / kPhysicsScale;
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.density = 1.0f;
    fixture.friction = 0.3;
    body->CreateFixture(&fixture);
    //
    // Create tentacles
    for (int i = 0; i < kNumTentacles; i ++)
    {
        // Add the knee
        double angle = TWO_PI * (i / (double)kNumTentacles);
        ofPoint offset = ofPoint(cos(angle) * kTentacleSegmentLength, sin(angle) * kTentacleSegmentLength);
        ofPoint body_offset = ofPoint(cos(angle) * kBodyRadius, sin(angle) * kBodyRadius);
        ofPoint attach_at = body_offset + pos;
        b2Body* previous = body;
        for (int j = 0; j < kNumSegments; j ++)
        {
            bodyDef.position = ofToB2(attach_at + 0.5 * offset);
            bodyDef.angle = angle;
            bodyDef.angularDamping = 2.0f;
            bodyDef.linearDamping = 3.0;
            b2Body* tentacle = phys_world->CreateBody(&bodyDef);
            b2PolygonShape box;
            box.SetAsBox(kTentacleSegmentLength * 0.5 / kPhysicsScale, 3 / kPhysicsScale);
            fixture.shape = &box;
            fixture.density = 0.01f;
            fixture.filter.maskBits = 0x0;
            tentacle->CreateFixture(&fixture);
            tentacles.push_back(tentacle);
            b2RevoluteJointDef jointDef;
            jointDef.Initialize(previous, tentacle, ofToB2(attach_at));
            jointDef.maxMotorTorque = 100.0f;
            b2RevoluteJoint* joint = (b2RevoluteJoint*) phys_world->CreateJoint(&jointDef);
            tentacle_joints.push_back(joint);
            previous = tentacle;
            attach_at += offset;
        }
//
//
//        ofPtr<ofxBox2dCircle> circle = ofPtr<ofxBox2dCircle>(new ofxBox2dCircle);
//        circle.get()->setPhysics(1.0, 0.5, 0.0);
//        circle.get()->setup(box2d.getWorld(), pos.x + offset.x, pos.y + offset.y, 2);
//        circle.get()->body->SetLinearDamping(5.0);
//        tentacles.push_back(circle);
//        // Add the joint
//        ofPtr<ofxBox2dJoint> joint = ofPtr<ofxBox2dJoint>(new ofxBox2dJoint);
//        joint.get()->setup(box2d.getWorld(), body.body, circle.get()->body);
//        joint.get()->setLength(40);
//        tentacle_joints.push_back(joint);
    }
}

/*
 This function consists of the following steps:
 - Check & update the current goal
 - Plan a path to goal
 - Move to goal (low level)
 */
void Squid::update(double delta_t, cv::Mat flow_high, ofxCv::ObjectFinder objectfinder, cv::Mat frame)
{
    pos_game = b2ToOf(body->GetPosition());
    pos_grid = pos_game / kGameSize * kPathGridSize;
    //
    // Subsample the flow grid to get a pathfinding grid
    cv::resize(flow_high, grid, kPathGridSize, 0, 0, CV_INTER_AREA);
    grid.convertTo(grid, CV_32F, 1 / 255.0f);
    ofxCv::dilate(grid, 1); // Dilation creates a margin around movement.
    grid = 1.0f - grid;
    cv::threshold(grid, grid, 0.2, 1.0, CV_THRESH_TOZERO);
    cv::resize(grid, sections, kSectionsSize, 0, 0,  CV_INTER_AREA);
    ofxCv::blur(sections, 2); // Blurring favors quiet sections that neighbor quiet sections.
    //
    // Check if current area or goal is crowded.
    local_area = cv::Rect(pos_grid.x - 2, pos_grid.y - 2, 5, 5);
    local_area = local_area & cv::Rect(cv::Point(0, 0), kPathGridSize);
    float local_sum = cv::sum(1.0f - grid(local_area))[0] / local_area.area();
    int local_flow_level = 0;
    // Any nearby motion scares the squid; causing it to re-select a goal.
    if (local_sum > local_flow_high) {
        local_flow_level = 2;
    } else if (local_sum > 0.0f) {
        local_flow_level = 1;
    }
    // Define some shorthands
    float going_slow = b2ToOf(body->GetLinearVelocity()).length() < kMinVelocity;
    bool near_goal = (ofPoint(goal_x, goal_y) - pos_grid).length() > kMaxGoalDistance;
    //
    // Behavior State machine
    switch (behavior_state) {
        case IDLE:
            if (local_flow_level == 2) {
                selectQuietGoal();
                behavior_state = PANIC;
            } else if (local_flow_level == 1){
                selectCloseGoal();
            }
            break;
            
        case PANIC:
            if (local_flow_level == 0){
                behavior_state = IDLE;
            }
            break;
            
        case FACE:
            break;
            
        case SWIM:
            break;
    }
    
    //
    // Motion state machine
    switch (motion_state) {
        case STILL:
            if (going_slow && !near_goal) {
                motion_time = 0;
                motion_state = PREP;
            }
            break;
        
        case PREP:
            motion_time += delta_t;
            tentaclePrep();
            if (motion_time > kMotionTimePrep){
                findWaypoint();
                motion_state = PUSH;
                motion_time = 0;
            }
            break;
            
        case PUSH:
            motion_time += delta_t;
            tentaclePush();
            body->ApplyForceToCenter(ofToB2(waypoint_direction * kPushForce * delta_t), true);
            if (motion_time > kMotionTimePush){
                motion_state = GLIDE;
                motion_time = 0;
            }
            break;
            
        case GLIDE:
            tentacleGlide();
            if (going_slow){
                motion_state = STILL;
            }
            break;
    }

}

void Squid::selectQuietGoal()
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

void Squid::selectCloseGoal()
{
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    minMaxLoc( grid(local_area), &minVal, &maxVal, &minLoc, &maxLoc );
    goal_x = pos_grid.x;
    goal_y = pos_grid.y;
}

void Squid::findWaypoint(){
    // Do the actual pathfinding
    ofxCv::copy(grid, grid_im);
    pathfinder.setup(grid_im);
    float path_length = pathfinder.find(pos_grid.x, pos_grid.y, goal_x, goal_y);
    pathfinder.path.simplify(0.5f);
    ofPoint waypoint;
    if (pathfinder.path.size() > 1){
        waypoint = pathfinder.path[pathfinder.path.size() - 2];
    } else {
        waypoint = ofPoint(goal_x, goal_y);
    }
    waypoint = (waypoint + 0.5) / kPathGridSize * kGameSize;
    waypoint_direction = (waypoint - pos_game).normalized();
}

/*
 * Moves the tentacles outward to prepare for a push
 */
void Squid::tentaclePrep(){
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        if (i % kNumTentacles < 2)
        {
            b2Vec2 outward = tentacle->GetPosition() - body->GetPosition();
            outward.Normalize();
            tentacle->ApplyForceToCenter(0.7f * outward, true);
        }
    }
}
            
/*
 * Moves the tentacles back to push
 */
void Squid::tentaclePush(){
    // Limb muscles
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        if (i % kNumTentacles < 2)
        {
            tentacle->ApplyForceToCenter(b2Vec2(-0.1f * waypoint_direction.x, -0.1f * waypoint_direction.y), true);
        }
    }
}

/*
 * Applies tensor friction to the tentacles to make them trail nicely
 */
void Squid::tentacleGlide(){
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        // Apply tensor friction
        b2Vec2 velocity = tentacle->GetLinearVelocity();
        b2Vec2 forward = b2Mul(tentacle->GetTransform().q, b2Vec2(1, 0));
        forward *= b2Dot(velocity, forward);
        b2Vec2 sideways = velocity - forward;
        velocity = 0.5 * sideways + forward;
        tentacle->SetLinearVelocity(velocity);
    }
}

void Squid::draw(bool draw_debug)
{
    ofPoint pos = b2ToOf(body->GetPosition());
    ofSetColor(255, 255, 255, 255);
    ofCircle(pos, kBodyRadius);
    for (int i = 0; i < tentacles.size(); i ++)
    {
        ofSetColor(255, 255 * (1.0 - ((float)i / (kNumSegments * kNumTentacles))), 255 * (float)i / (kNumSegments * kNumTentacles));
        ofPushMatrix();
        ofTranslate(b2ToOf(tentacles[i]->GetPosition()));
        ofRotateZ(tentacles[i]->GetAngle() * RAD_TO_DEG);
        ofRect(-kTentacleSegmentLength / 2, -3, kTentacleSegmentLength, 6);
//        ofRotate(float degrees)(b2ToOf(tentacles[i]->GetPosition()), 3);
        ofPopMatrix();
    }
    if (draw_debug)
    {
        // Draw the grids
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofSetColor(0, 255, 0, 64);
        ofxCv::drawMat(grid, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofSetColor(0, 0, 255, 64);
        ofxCv::drawMat(sections, 0, 0, kScreenWidth, kScreenHeight, GL_NEAREST);
        ofDisableBlendMode();
        ofSetColor(0, 255, 0, 255);
        // Draw the path and local area
        ofSetColor(0, 255, 255, 255);
        ofPushMatrix();
        ofScale(kScreenWidth / kPathGridSize.width, kScreenHeight / kPathGridSize.height);
        ofRectangle debug_local_area = ofxCv::toOf(local_area);
        ofNoFill();
        ofRect(debug_local_area);
        ofTranslate(0.5, 0.5);
        pathfinder.path.draw();
        ofCircle(goal_x, goal_y, 0.2f);
        ofPopMatrix();
    }
}