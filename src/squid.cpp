
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
    clean();
    // Set up physics body
    pos_game = ofPoint(ofGetWidth() / 2, ofGetHeight() / 2);
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = ofToB2(pos_game);
    bodyDef.angularDamping = 5.0f;
    bodyDef.linearDamping = 2.0f;
    body = phys_world->CreateBody(&bodyDef);
    b2CircleShape shape;
    shape.m_radius = body_radius / kPhysicsScale;
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.density = body_density;
    fixture.friction = 0.3;
    body->CreateFixture(&fixture);
    //
    // Create tentacles
    for (int i = 0; i < num_tentacles; i ++)
    {
        // Add the knee
        double angle = TWO_PI * (i / (double)num_tentacles);
        ofPoint offset = ofPoint(cos(angle) * segment_length, sin(angle) * segment_length);
        ofPoint body_offset = ofPoint(cos(angle) * body_radius, sin(angle) * body_radius);
        ofPoint attach_at = body_offset + pos_game;
        b2Body* previous = body;
        for (int j = 0; j < num_segments; j ++)
        {
            bodyDef.position = ofToB2(attach_at + 0.5 * offset);
            bodyDef.angle = angle;
            bodyDef.angularDamping = 2.0f;
            bodyDef.linearDamping = tentacle_damping;
            b2Body* tentacle = phys_world->CreateBody(&bodyDef);
            b2PolygonShape box;
            box.SetAsBox(segment_length * 0.5 / kPhysicsScale, 3 / kPhysicsScale);
            fixture.shape = &box;
            fixture.density = 0.05f;
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
    }
}

void Squid::clean()
{
    if (body != NULL)
    {
        body->GetWorld()->DestroyBody(body);
        body = NULL;
        for (int i = 0; i < tentacles.size(); i++)
        {
            tentacles[i]->GetWorld()->DestroyBody(tentacles[i]);
        }
        tentacles.clear();
        tentacle_joints.clear();
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
    ofPoint game_size(ofGetWidth(), ofGetHeight(), 1);
    pos_game = b2ToOf(body->GetPosition());
    pos_grid = pos_game / game_size * kPathGridSize;
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
    if (local_sum > local_flow_high)
    {
        local_flow_level = 2;
    }
    else if (local_sum > 0.0f)
    {
        local_flow_level = 1;
    }
    // Define some shorthands
    float going_slow = b2ToOf(body->GetLinearVelocity()).length() < min_velocity;
    bool near_goal = (ofPoint(goal_x, goal_y) - pos_grid).length() < max_goal_distance;
    //
    // Behavior State machine
    switch (behavior_state)
    {
    case IDLE:
        if (local_flow_level == 2)
        {
            selectQuietGoal();
            behavior_state = PANIC;
        }
        break;
    case PANIC:
        if (local_flow_level == 0)
        {
            behavior_state = IDLE;
        }
        else
        {
            selectQuietGoal();
        }
        break;
    case FACE:
        break;
    case SWIM:
        break;
    }
    //
    // Motion state machine
    switch (motion_state)
    {
    case STILL:
        if (!near_goal)
        {
            motion_time = 0;
            findWaypoint();
            motion_state = PREP;
        }
        idleMotion(delta_t);
        break;
    case PREP:
        motion_time += delta_t;
        tentaclePrep();
        bodyPrep(delta_t);
        if (motion_time > motion_time_prep)
        {
            findWaypoint();
            motion_state = PUSH;
            motion_time = 0;
        }
        break;
    case PUSH:
        motion_time += delta_t;
        tentaclePush();
        bodyPush(delta_t);
        if (motion_time > motion_time_push)
        {
            motion_state = GLIDE;
            motion_time = 0;
        }
        break;
    case GLIDE:
        tentacleGlide();
        if (going_slow)
        {
            motion_state = STILL;
        }
        if (behavior_state == PANIC)
        {
            motion_time = 0;
            findWaypoint();
            motion_state = PREP;
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

void Squid::findWaypoint()
{
    // Do the actual pathfinding
    ofxCv::copy(grid, grid_im);
    pathfinder.setup(grid_im);
    float path_length = pathfinder.find(pos_grid.x, pos_grid.y, goal_x, goal_y);
    pathfinder.path.simplify(0.5f);
    ofPoint waypoint;
    if (pathfinder.path.size() > 1)
    {
        waypoint = pathfinder.path[pathfinder.path.size() - 2];
    }
    else
    {
        waypoint = ofPoint(goal_x, goal_y);
    }
    waypoint = (waypoint + 0.5) / kPathGridSize * ofPoint(ofGetWidth(), ofGetHeight(), 1);
    waypoint_direction = (waypoint - pos_game).normalized();
    waypoint_distance = (waypoint - pos_game).length();
}

/*
 * Rotates the body towards the goal
 */
void Squid::bodyPrep(double delta_t)
{
    b2Vec2 forward = b2Mul(body->GetTransform().q, b2Vec2(1, 0));
    float angle = atan2(b2Cross(forward, ofToB2(waypoint_direction)), b2Dot(forward, ofToB2(waypoint_direction)));
    if (angle < -0.3)
    {
        body->ApplyAngularImpulse(20 * delta_t, true);
    }
    else if (angle > 0.3)
    {
        body->ApplyAngularImpulse(-20 * delta_t, true);
    }
}

/*
 * Applies a force to the body to propel it to goal
 */
void Squid::bodyPush(double delta_t)
{
    double force = push_force * body->GetMass() * delta_t;
    force = MIN(force, force * (waypoint_distance / (2 * max_goal_distance)));
    body->ApplyForceToCenter(ofToB2(waypoint_direction * force), true);
}
/*
 * Moves the tentacles outward to prepare for a push
 */
void Squid::tentaclePrep()
{
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        if (i % num_segments < 2)
        {
            b2Vec2 outward = tentacle->GetPosition() - body->GetPosition();
            outward.Normalize();
            tentacle->ApplyForceToCenter(tentacle->GetMass() * tentacle_prep_force * outward, true);
        }
    }
}

/*
 * Moves the tentacles back to push
 */
void Squid::tentaclePush()
{
    // Limb muscles
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        if (i % num_segments < 2)
        {
            tentacle->ApplyForceToCenter(b2Vec2(-0.1f * waypoint_direction.x, -0.1f * waypoint_direction.y), true);
        }
    }
}

/*
 * Applies tensor friction to the tentacles to make them trail nicely
 */
void Squid::tentacleGlide()
{
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

void Squid::idleMotion(double delta_t){
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];
        if (i % num_segments < 2)
        {
            b2Vec2 outward = tentacle->GetPosition() - body->GetPosition();
            outward.Normalize();
            tentacle->ApplyForceToCenter(tentacle->GetMass() * 10 * outward, true);
        }
    }
    body->ApplyAngularImpulse(3 * delta_t, true);
}

void Squid::draw(bool draw_debug)
{
    ofFill();
    ofSetColor(255, 255, 255, 255);
    ofCircle(pos_game, body_radius);
    for (int i = 0; i < tentacles.size(); i ++)
    {
        ofSetColor(255, 255 * (1.0 - ((float)i / (num_segments * num_tentacles))), 255 * (float)i / (num_segments * num_tentacles));
        ofPushMatrix();
        ofTranslate(b2ToOf(tentacles[i]->GetPosition()));
        ofRotateZ(tentacles[i]->GetAngle() * RAD_TO_DEG);
        ofRect(-segment_length / 2, -3, segment_length, 6);
//        ofRotate(float degrees)(b2ToOf(tentacles[i]->GetPosition()), 3);
        ofPopMatrix();
    }
    if (draw_debug)
    {
        // Draw state
        std::string state = "";
        switch (behavior_state)
        {
        case IDLE:
            state += "IDLE";
            break;
        case PANIC:
            state += "PANIC";
            break;
        default:
            break;
        }
        state += " / ";
        switch (motion_state)
        {
        case STILL:
            state += "STILL";
            break;
        case PREP:
            state += "PREP";
            break;
        case PUSH:
            state += "PUSH";
            break;
        case GLIDE:
            state += "GLIDE";
            break;
        }
        ofDrawBitmapStringHighlight(state, pos_game + kLabelOffset);
        // Draw the grids
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofSetColor(0, 255, 0, 64);
        ofxCv::drawMat(grid, 0, 0, ofGetWidth(), ofGetHeight(), GL_NEAREST);
        ofSetColor(0, 0, 255, 64);
        ofxCv::drawMat(sections, 0, 0, ofGetWidth(), ofGetHeight(), GL_NEAREST);
        ofDisableBlendMode();
        ofSetColor(0, 255, 0, 255);
        // Draw the path and local area
        ofSetColor(0, 255, 255, 255);
        ofPushMatrix();
        ofScale(ofGetWidth() / kPathGridSize.width, ofGetHeight() / kPathGridSize.height);
        ofRectangle debug_local_area = ofxCv::toOf(local_area);
        ofNoFill();
        ofRect(debug_local_area);
        ofTranslate(0.5, 0.5);
        pathfinder.path.draw();
        ofCircle(goal_x, goal_y, 0.2f);
        ofPopMatrix();
    }
}