
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
    goal = ofPoint(ofGetWidth() / 2, ofGetHeight() / 2);
    // Setup the physics
    setupPhysics(phys_world);
    // Load textures
    body_outer_im.loadImage("assets/body_outer.png");
    body_inner_im.loadImage("assets/body_inner.png");
    // Set up face detection
    objectfinder.setup("haarcascades/haarcascade_frontalface_alt2.xml");
    //    objectfinder.setup("haarcascades/haarcascade_profileface.xml");
    objectfinder.setRescale(1.0); // Don't rescale internally because we'll feed it a small frame
    objectfinder.setMinNeighbors(2);
    objectfinder.setMultiScaleFactor(1.2);
    objectfinder.setFindBiggestObject(true);
    has_face = false;
}

void Squid::setupPhysics(ofPtr<b2World> phys_world)
{
    tentacles = vector<b2Body*>();
    tentacle_joints = vector<b2RevoluteJoint*>();
    // Set up physics body
    pos_game = ofPoint(ofGetWidth() / 2, ofGetHeight() / 2);
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = ofToB2(pos_game);
    bodyDef.angularDamping = 5.0f;
    bodyDef.linearDamping = 2.0f;
    body = phys_world->CreateBody(&bodyDef);
    b2CircleShape shape;
    shape.m_radius = scale * body_radius / kPhysicsScale;
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.density = body_density;
    fixture.friction = 0.3;
    body->CreateFixture(&fixture);

    // Create tentacles
    for (int i = 0; i < num_tentacles; i ++)
    {
        // Add the knee
        double angle = TWO_PI * (i / (double)num_tentacles);
        ofPoint offset = ofPoint(cos(angle) * segment_length * scale, sin(angle) * segment_length * scale);
        ofPoint body_offset = ofPoint(cos(angle) * body_radius * scale, sin(angle) * body_radius * scale);
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
            box.SetAsBox(0.5 * scale * segment_length / kPhysicsScale, scale * segment_length * 0.25 / kPhysicsScale);
            fixture.shape = &box;
            fixture.density = 0.05f;
            fixture.filter.maskBits = 0x0;
            tentacle->CreateFixture(&fixture);
            tentacles.push_back(tentacle);
            b2RevoluteJointDef jointDef;
            jointDef.Initialize(previous, tentacle, ofToB2(attach_at));
            b2RevoluteJoint* joint = (b2RevoluteJoint*) phys_world->CreateJoint(&jointDef);
            tentacle_joints.push_back(joint);
            previous = tentacle;
            attach_at += offset;
        }
    }
}


/*
 This function consists of the following steps:
 - Check & update the current goal
 - Plan a path to goal
 - Move to goal (low level)
 */
void Squid::update(double delta_t, cv::Mat flow_high, cv::Mat frame, ofxFluid& fluid)
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
    float going_slow = b2ToOf(body->GetLinearVelocity()).length() < (min_velocity * scale);
    bool near_goal = (goal - pos_game).length() < max_goal_distance;
    bool sees_face = objectfinder.size() > 0;
    bool near_face = (found_face.getCenter() - pos_game).length() < max_face_distance;
    // Update Cooldowns
    face_cooldown_timer -= delta_t;

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
            else if (sees_face && face_cooldown_timer < 0)
            {
                selectFaceGoal();
                behavior_state = FACE;
            }

            break;

        case PANIC:
            fluid.addTemporalForce(getPosition(), -waypoint_direction*10, ofColor::red,body_radius * kFluidScale);
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
            if (local_flow_level == 2)
            {
                selectQuietGoal();
                behavior_state = PANIC;
            }

            if (near_face && face_cooldown_timer < 0)
            {
                grabFace(frame);
                face_cooldown_timer = face_cooldown;
                selectQuietGoal();
                behavior_state = IDLE;
            }

            break;
    }

    //
    // Motion state machine
    switch (motion_state)
    {
        case STILL:
            if (!near_goal || behavior_state == FACE)
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

void Squid::updateObjectFinder(cv::Mat frame)
{
    // Only compute a width because the face search window is square.
    frame_scale = ofGetWidth() / (double)frame.cols;
    int face_search_width = MIN(frame.cols, frame.rows) * face_search_window;
    face_roi = cv::Rect(0, 0, face_search_width, face_search_width);
    face_roi += cv::Point(MAX(0, MIN(frame.cols - face_search_width, (pos_game.x / frame_scale - face_search_width / 2))),
                          MAX(0, MIN(frame.rows - face_search_width, (pos_game.y / frame_scale - face_search_width / 2))));
    // Face size is relative to frame, not face search window, so rescale and cap at 1.0
    objectfinder.setMinSizeScale(MIN(1.0, face_size_min / face_search_window));
    objectfinder.setMaxSizeScale(MIN(1.0, face_size_max / face_search_window));
    // The call below uses 2 arguments including a switch "preprocess" that
    // was added to ObjectFinder::update to disable the resize and BGR2GRAY calls.
    objectfinder.update(frame(face_roi), true);

    if (objectfinder.size() > 0)
    {
        found_face = objectfinder.getObject(0);
        found_face.scale(frame_scale);
        found_face.x += face_roi.x * frame_scale;
        found_face.y += face_roi.y * frame_scale;
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
        goal = (ofxCv::toOf(goal_section) + ofPoint(ofRandomuf(), ofRandomuf())) / kSectionsSize * ofPoint(ofGetWidth(), ofGetHeight());
    }
}

void Squid::selectFaceGoal()
{
    goal = found_face.getCenter();
}

void Squid::grabFace(cv::Mat frame)
{
    frame_scale = ofGetWidth() / (float)frame.cols;
    cv::Rect face_region(found_face.x / frame_scale, found_face.y / frame_scale, found_face.width / frame_scale, found_face.height / frame_scale);
    face_region &= cv::Rect(0, 0, frame.cols, frame.rows);
    cv::Mat cutout = frame(face_region).clone();
    cv::cvtColor(cutout, cutout, CV_RGB2GRAY);
    printMatrixInfo(cutout);
    cv::equalizeHist(cutout, cutout);
//    cv::threshold(cutout, cutout, 128, 255, cv::THRESH_BINARY);
    //Build alpha channel
    cv::Mat alpha(cutout.size(), CV_8UC1);
    alpha.setTo(0);
    cv::circle(alpha, cv::Point(alpha.cols / 2, alpha.rows / 2), alpha.rows / 2, cv::Scalar(255), -1);
    vector<cv::Mat> channels;
    cv::split(cutout, channels);
    channels.push_back(cutout);
    channels.push_back(cutout);
    channels.push_back(alpha);
    cv::merge(channels, face_mat);
    printMatrixInfo(face_mat);
    ofxCv::toOf(face_mat, face_im);
    face_im.update();
//    ofxCv::toOf(face_mat, face_im);
    has_face = true;
}


void Squid::findWaypoint()
{
    // Do the actual pathfinding
    ofxCv::copy(grid, grid_im);
    pathfinder.setup(grid_im);
    ofPoint goal_grid = goal / ofPoint(ofGetWidth(), ofGetHeight()) * kPathGridSize;
    float path_length = pathfinder.find(pos_grid.x, pos_grid.y, round(goal_grid.x), round(goal_grid.y));
    pathfinder.path.simplify(0.5f);
    ofPoint waypoint;

    if (pathfinder.path.size() > 2)
    {
        waypoint = pathfinder.path[pathfinder.path.size() - 2];
        waypoint = (waypoint + 0.5) / kPathGridSize * ofPoint(ofGetWidth(), ofGetHeight());
    }
    else
    {
        waypoint = goal;
    }

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
        body->ApplyTorque(100.0 * body->GetInertia(), true);
    }
    else if (angle > 0.3)
    {
        body->ApplyTorque(-100.0 * body->GetInertia(), true);
    }
}

/*
 * Applies a force to the body to propel it to goal
 */
void Squid::bodyPush(double delta_t)
{
    double force = push_force * body->GetMass() * scale;
    if (behavior_state == PANIC){
        force *= panic_force_multiplier;
    }
    force = MIN(force, force * (waypoint_distance / (2 * max_goal_distance)));
    body->ApplyLinearImpulse(ofToB2(waypoint_direction * force), body->GetPosition(), true);
//    body->ApplyForceToCenter(ofToB2(waypoint_direction * force), true);
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
    b2Vec2 force = tentacles[0]->GetMass() * -1 * b2Vec2(waypoint_direction.x, waypoint_direction.y);
    // Limb muscles
    for (int i = 0; i < tentacles.size(); i++)
    {
        b2Body* tentacle = tentacles[i];

        if (i % num_segments < 2)
        {
            tentacle->ApplyForceToCenter(force, true);
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

void Squid::idleMotion(double delta_t)
{
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

    body->ApplyTorque(3.0, true);
}

void Squid::draw(bool draw_debug)
{
    ofSetColor(255, 255, 255, 255);
    float body_radius_s = body_radius * scale;
    // Trasnform to body position
    ofPushMatrix();
    ofEnableAlphaBlending();
    ofTranslate(pos_game);
    ofRotate(body->GetAngle() * RAD_TO_DEG);
    ofRectangle body_draw_rect(-body_radius_s, -body_radius_s, body_radius_s * 2, body_radius_s * 2);
    body_outer_im.draw(body_draw_rect);
    // Draw face
    if (has_face)
    {
        face_im.draw(body_draw_rect);
    }
    body_inner_im.draw(body_draw_rect);
    ofPopMatrix();
    // Draw rest of squid
    ofNoFill();

//    for (int i = 0; i < tentacles.size(); i ++)
//    {
//        ofSetColor(255, 255 * (1.0 - ((float)i / (num_segments * num_tentacles))), 255 * (float)i / (num_segments * num_tentacles));
//        ofPushMatrix();
//        ofTranslate(b2ToOf(tentacles[i]->GetPosition()));
//        ofRotateZ(tentacles[i]->GetAngle() * RAD_TO_DEG);
//        ofRect(-segment_length / 2, -3, segment_length, 6);
////        ofRotate(float degrees)(b2ToOf(tentacles[i]->GetPosition()), 3);
//        ofPopMatrix();
//    }
    for (int i = 0; i < num_tentacles; i ++)
    {
        ofPolyline path;
        path.curveTo(b2ToOf(body->GetPosition()));
        ofPoint p = b2ToOf(tentacles[i * num_segments]->GetWorldPoint(b2Vec2(-0.5 * segment_length / kPhysicsScale, 0)));
        path.curveTo(p);

        for (int j = 1; j < num_segments; j ++)
        {
            ofPoint mid = b2ToOf(tentacles[i * num_segments + j]->GetPosition());
            path.curveTo(mid);

            if (j == num_segments - 1)
            {
                ofPoint end = b2ToOf(tentacles[i * num_segments + j]->GetWorldPoint(b2Vec2(segment_length * 0.75 / kPhysicsScale, 0.0)));
                path.curveTo(end);
            }
        }

        ofSetLineWidth(10);
        path.draw();
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

            case FACE:
                state += "FACE";
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
        ofPopMatrix();
        ofCircle(goal, max_goal_distance);
        // Draw face search window
        ofRectangle face_roi_rect = ofxCv::toOf(face_roi);
        face_roi_rect.scale(frame_scale);
        face_roi_rect.x *= frame_scale;
        face_roi_rect.y *= frame_scale;
        ofPushStyle();
        ofSetLineWidth(2.0);
        ofSetColor(ofColor::orange);
        ofRect(face_roi_rect);
        ofDrawBitmapStringHighlight("face search window", face_roi_rect.getPosition() + kLabelOffset, ofColor::orange, ofColor::black);
        ofSetColor(255, 255, 255);
        ofRect(face_roi_rect.getPosition(), face_size_min * ofGetHeight(), face_size_min * ofGetHeight());
        ofRect(face_roi_rect.getPosition(), face_size_max * ofGetHeight(), face_size_max * ofGetHeight());

        // Draw detected faces
        if(objectfinder.size())
        {
            ofSetColor(ofColor::green);
            ofRect(found_face);
        }
    }
}