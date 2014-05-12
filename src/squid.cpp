
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
    for (int i = 0; i < kNumTentacles; i ++){
        // Add the knee
        double angle = TWO_PI * (i / (double)kNumTentacles);
        ofPoint offset = ofPoint(cos(angle) * kTentacleSegmentLength, sin(angle) * kTentacleSegmentLength);
        ofPoint body_offset = ofPoint(cos(angle) * kBodyRadius, sin(angle) * kBodyRadius);
        ofPoint attach_at = body_offset + pos;
        b2Body* previous = body;
        
        for (int j = 0; j < kNumSegments; j ++){
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
void Squid::update(double delta_t, cv::Mat flow_high)
{
    ofPoint pos = b2ToOf(body->GetPosition());
    ofPoint pos_grid = pos / kGameSize * kPathGridSize;
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
    // Pick a goal, first check if current area or goal is crowded.
    local_area = cv::Rect(pos_grid.x - 2, pos_grid.y - 2, 5, 5);
    local_area = local_area & cv::Rect(cv::Point(0, 0), kPathGridSize);
    float local_motion = cv::sum(1.0f - grid(local_area))[0];
    // Any nearby motion scares the squid; causing it to re-select a goal.
    if (local_motion > 0.0f)
    {
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
//        cv::Mat random;
//        ofxCv::imitate(random, sections);
//        cv::randu(random, 0, 0.1);
//        sections += random;
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
    ofxCv::copy(grid, grid_im);
    pathfinder.setup(grid_im);
    float path_length = pathfinder.find(pos_grid.x, pos_grid.y, goal_x, goal_y);
    pathfinder.path.simplify(0.5f);
    //
    // Low level planning on path
    if (pathfinder.path.size() > 1)
    {
        ofPoint next = (pathfinder.path[pathfinder.path.size() - 2] + 0.5) / kPathGridSize * kGameSize;
        ofPoint diff = (next - pos);
        if (pushing <= 0 && b2ToOf(body->GetLinearVelocity()).length() < kMinVelocity && (diff.length() > kMaxGoalDistance || pathfinder.path.size() > 2)) {
            push_direction = diff.normalized();
            pushing = kPushTime + kPrepTime;
        }
    }
    //
    // Muscle movement
    if (0 < pushing && pushing < kPushTime){
        body->ApplyForceToCenter(ofToB2(push_direction * kPushForce * delta_t), true);
//        double direction_angle = ofPoint(0,0).angleRad(push_direction);
//        body->ApplyTorque((direction_angle - body->GetAngle()) * 20, true);
    }
    pushing -= delta_t;
    //
    // Limb muscles
    for (int i = 0; i < tentacles.size(); i++){
        b2Body* tentacle = tentacles[i];
        // Apply tensor friction
        if (pushing < kPushTime){
            b2Vec2 velocity = tentacle->GetLinearVelocity();
            b2Vec2 forward = b2Mul(tentacle->GetTransform().q, b2Vec2(1,0));
            forward *= b2Dot(velocity, forward);
            b2Vec2 sideways = velocity - forward;
            velocity = 0.5 * sideways + forward;
            tentacle->SetLinearVelocity(velocity);
        }
        // Move tentacles out
        if (pushing > kPushTime){
            if (i % kNumTentacles < 2){
                b2Vec2 outward = tentacle->GetPosition() - body->GetPosition();
                outward.Normalize();
                tentacle->ApplyForceToCenter(0.7f * outward, true);
            }
        // Move tentacles in
        }
        else if (pushing > 0) {
            if (i % kNumTentacles < 2){
                tentacle->ApplyForceToCenter(b2Vec2(-0.1f * push_direction.x, -0.1f * push_direction.y), true);
            }
        }

    }


    // Keep limbs straight
//    for (int i = 0; i < tentacle_joints.size(); i ++){
//        b2RevoluteJoint* joint = tentacle_joints[i];
//        double angle = joint->GetJointAngle();
//        if (abs(angle) > 90 * DEG_TO_RAD) {
//            joint->SetMotorSpeed((angle > 0) ? -0.1 : 0.1);
//            joint->EnableMotor(true);
//        } else {
//            joint->EnableMotor(false);
//        }
//
//    }
}


void Squid::draw(bool draw_debug)
{
    ofPoint pos = b2ToOf(body->GetPosition());
    ofSetColor(255, 255, 255, 255);
    ofCircle(pos, kBodyRadius);
    
    for (int i = 0; i < tentacles.size(); i ++){
        ofSetColor(255, 255 * (1.0 - ((float)i / (kNumSegments*kNumTentacles))), 255 * (float)i / (kNumSegments*kNumTentacles));
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
        // Draw push force
        if (pushing > 0){
            ofLine(pos.x, pos.y, pos.x + push_direction.x * 20, pos.y + push_direction.y * 20);
        }
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