
#include "squid.h"
#include "constants.h"
#include "ofxCv.h"
#include <algorithm>

using namespace ofxCv;
using namespace Playlist;

using std::cout;
using std::endl;

////////// SQUID CLASS //////////

void Squid::setup(ofPtr<b2World> in_phys_world, ofxFluid* in_fluid, FlowCam* in_flowcam)
{
    phys_world = in_phys_world;
    fluid = in_fluid;
    flowcam = in_flowcam;
    squish = 1.0f;
    goal = ofPoint(ofGetWidth() / 2, ofGetHeight() / 2);
    // Setup the physics
    setupPhysics();
    setupTextures();
    // Set up face detection
    objectfinder.setup("haarcascades/haarcascade_frontalface_alt2.xml");
    //    objectfinder.setup("haarcascades/haarcascade_profileface.xml");
    objectfinder.setRescale(1.0); // Don't rescale internally because we'll feed it a small frame
    objectfinder.setMinNeighbors(2);
    objectfinder.setMultiScaleFactor(1.2);
    objectfinder.setFindBiggestObject(false);
    objectfinder.getTracker().setSmoothingRate(0.1);
    has_face = false;
    main_color = ofColor::red;
}

void Squid::setupPhysics()
{
    // Clean up old physics if any
    if (body != NULL){
        body->GetWorld()->DestroyBody(body);
        for (int i = 0; i < tentacles.size(); i ++) {
            tentacles[i]->GetWorld()->DestroyBody(tentacles[i]);
        }
    }
    tentacles = vector<b2Body*>();
    tentacle_joints = vector<b2RevoluteJoint*>();
    // Set up physics body
    pos_game = ofPoint(ofGetWidth() / 2, ofGetHeight() / 2);
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = ofToB2(pos_game);
    bodyDef.angularDamping = 5.0f;
    bodyDef.linearDamping = body_damping;
    body = phys_world->CreateBody(&bodyDef);
    b2CircleShape shape;
    shape.m_radius = scale * body_radius / kPhysicsScale;
    b2FixtureDef fixture;
    fixture.shape = &shape;
    fixture.density = body_density;
    fixture.friction = 0.3;
    body->CreateFixture(&fixture);

    // Create tentacles
    for (int i = 0; i < num_tentacles; i ++) {
        // Add the knee
        double angle = TWO_PI * (i / (double)num_tentacles) - PI / 2;
        ofPoint direction = ofPoint(cos(angle), sin(angle));
        ofPoint offset = segment_join_length * segment_length * scale * direction;
        ofPoint body_offset = body_radius * scale * (tentacle_attach_scale * direction + tentacle_attach_offset);
        ofPoint attach_at = body_offset + pos_game;
        b2Body* previous = body;

        for (int j = 0; j < num_segments; j ++) {
            bodyDef.position = ofToB2(attach_at + 0.5 * offset);
            bodyDef.angle = angle;
            bodyDef.angularDamping = 1.0f;
            bodyDef.linearDamping = tentacle_damping;
            b2Body* tentacle = phys_world->CreateBody(&bodyDef);
            b2PolygonShape box;
            box.SetAsBox(0.5 * scale * segment_length / kPhysicsScale, 0.5 * scale * segment_width / kPhysicsScale);
            fixture.shape = &box;
            fixture.density = tentacle_density;
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

void Squid::setupTextures(){
    // Load textures
    body_back_im.loadImage("assets/body_back.png");
    body_front_im.loadImage("assets/body_front.png");
    body_accent_im.loadImage("assets/body_accent.png");
    hint_im.loadImage("assets/hint.png");
    tentacle_back_im.loadImage("assets/tentacle_back.png");
    tentacle_front_im.loadImage("assets/tentacle_front.png");
    face_mask_im.loadImage("assets/face_mask.png");
    face_mask_mat = toCv(face_mask_im);
    cv::cvtColor(face_mask_mat, face_mask_mat, CV_RGB2GRAY);
    cv::resize(face_mask_mat, face_mask_mat, cv::Size(body_radius * scale * 2, body_radius * scale * 2));
}

/*
 This function consists of the following steps:
 - Check & update the current goal
 - Plan a path to goal
 - Move to goal (low level)
 */
void Squid::update(double delta_t)
{
    updateFlow();
    updateObjectFinder();
    playlist.update();
    // Some position and direction helpers
    ofPoint game_size(ofGetWidth(), ofGetHeight(), 1);
    pos_game = b2ToOf(body->GetPosition());
    pos_section = pos_game / game_size * ofPoint(kSectionsSize.width, kSectionsSize.height);
    ofPoint to_goal = (goal - pos_game);
    goal_distance = to_goal.length();
    goal_direction = to_goal.normalized();
    goal_angle = atan2(goal_direction.y, goal_direction.x);
    // Define some shorthands
    going_slow = b2ToOf(body->GetLinearVelocity()).length() < (min_velocity * scale);
    on_goal = (goal - pos_game).length() < max_goal_distance_close;
    facing_goal = abs(ofWrapRadians(goal_angle - body->GetAngle() + PI / 2)) < DEG_TO_RAD * 30;
    // Update State machines
    updateBehaviorState(delta_t);
    updateMotionState(delta_t);
    // Display mood
    main_color.setHue(main_color.getHue() + 2);
    main_color.setSaturation(16 + 255 * local_flow);
}

void Squid::updateFlow(){
    // Subsample the flow grid to get "sections"
    cv::Mat flow_high_float;
    flowcam->flow_high.convertTo(flow_high_float, CV_32F, 1 / 255.0f);
    cv::resize(flow_high_float, sections, kSectionsSize, 0, 0,  CV_INTER_AREA);
    ofxCv::blur(sections, 2); // Blurring favors quiet sections that neighbor quiet sections.
    cv::Mat noise(kSectionsSize, CV_32F); // Add some noise to avoid (0,0) bias when looking for minimum
    cv::randu(noise, 0, 0.1);
    sections += noise;
    local_area = cv::Rect(pos_section.x - 1, pos_section.y - 1, 3, 3);
    local_area = local_area & cv::Rect(cv::Point(0, 0), kSectionsSize);
    float local_flow_sum = cv::sum(sections(local_area))[0] / local_area.area();
    local_flow = 0.9 * local_flow + 0.1 * ofMap(local_flow_sum, local_flow_min, local_flow_max, 0, 1);
}

void Squid::updateObjectFinder()
{
    // Only compute a width because the face search window is square.
    cv::Mat frame = flowcam->frame;
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
    
    if (objectfinder.size() > 0) {
        face_detection_count ++;
        if (face_detection_count > face_detection_threshold){
            int label = objectfinder.getLabel(0);
            found_face = toOf(objectfinder.getTracker().getSmoothed(label));
            found_face.scale(frame_scale);
            found_face.x += face_roi.x * frame_scale;
            found_face.y += face_roi.y * frame_scale;
            sees_face = true;
        }
    } else {
        face_detection_count = 0;
        sees_face = false;
    }
}

void Squid::updateBehaviorState(double delta_t)
{
    time_in_behavior_state += delta_t;

    //
    // Behavior State machine
    switch (behavior_state) {
        case IDLE:
            if (local_flow >= 1) {
                switchBehaviorState(PANIC);
                break;
            }

            if (sees_face) {
                switchBehaviorState(FACE);
                break;
            }

            if (time_in_behavior_state > idle_move_cooldown) {
                selectQuietGoalInRegion(local_area);
                switchBehaviorState(IDLE);
                break;
            }

            break;

        case PANIC:
            if (!currentGoalIsQuiet()) {
                selectQuietGoal();
            }

            if (on_goal) {
                switchBehaviorState(IDLE);
            }

            break;

        case FACE:
            if (sees_face) {
                goal = found_face.getCenter();
            }

            if (time_in_behavior_state > face_pose_time) {
                grabFace(true);
                switchBehaviorState(PANIC);
                break;
            }

            grabFace(false);
            break;
    }
}
void Squid::updateMotionState(double delta_t)
{
    time_in_motion_state += delta_t;

    switch (motion_state) {
        case STILL:
            if (!on_goal) {
                switchMotionState(PREP);
            } else {
                turnUpright(delta_t);
            }

            break;

        case PREP:
            tentaclePrep();
            turnToGoal(delta_t);
            squishPrep(delta_t);

            if (time_in_motion_state > motion_time_prep && (facing_goal || behavior_state == PANIC)) {
                switchMotionState(PUSH);
            }

            break;

        case PUSH:
            squishPush(delta_t);
            bodyPush(delta_t);

            if (time_in_motion_state > motion_time_push) {
                switchMotionState(GLIDE);
            }
            
            if (behavior_state == PANIC){
                squirt();
            }

            break;

        case GLIDE:
            tentacleGlide();

            if (going_slow) {
                switchMotionState(STILL);
            }

            break;

        case LOCK:
            bodyPush(delta_t);
            turnUpright(delta_t);
            break;
    }
}
void Squid::switchBehaviorState(BehaviorState next)
{
    time_in_behavior_state = 0;

    switch (behavior_state) {
        case IDLE:
            break;

        case PANIC:
            break;

        case FACE:
            break;
    }

    switch (next) {
        case IDLE:
            break;

        case PANIC:
            switchMotionState(STILL);
            selectQuietGoal();
            break;

        case FACE:
            switchMotionState(LOCK);
            showCaptureHint();
            clearFace();
            break;
    }

    behavior_state = next;
}
void Squid::switchMotionState(MotionState next)
{
    time_in_motion_state = 0;

    switch (motion_state) {
        case STILL:
            break;

        case PREP:
            break;

        case PUSH:
            break;

        case GLIDE:
            break;

        case LOCK:
            break;
    }

    switch (next) {
        case STILL:
            break;

        case PREP:
            break;

        case PUSH:
            break;

        case GLIDE:
            break;

        case LOCK:
            break;
    }

    motion_state = next;
}


void Squid::selectQuietGoalInRegion(cv::Rect bounds)
{
    double minVal, maxVal;
    cv::Point2i minLoc, maxLoc;
    minMaxLoc( sections(bounds), &minVal, &maxVal, &minLoc, &maxLoc );
    minLoc.x += bounds.x;
    minLoc.y += bounds.y;

    if (goal_section != minLoc) {
        goal_section = minLoc;
        goal = (toOf(goal_section) + ofPoint(ofRandomuf(), ofRandomuf())) / kSectionsSize * ofPoint(ofGetWidth(), ofGetHeight());
    }

    goal.set(ofClamp(goal.x, body_radius * scale, ofGetWidth() - body_radius * scale),
             ofClamp(goal.y, body_radius * scale, ofGetHeight() - body_radius * scale * goal_bottom_margin));
}

void Squid::selectQuietGoal()
{
    selectQuietGoalInRegion(cv::Rect(0, 0, sections.cols, sections.rows));
}

bool Squid::currentGoalIsQuiet()
{
    float flow_in_goal_section = sections.at<float>(goal_section.y , goal_section.x);
    return flow_in_goal_section <= 0.2;
}

void Squid::selectFaceGoal()
{
    goal = found_face.getCenter();
}

void Squid::clearFace(){
    has_face = false;
    face_anim.clear();
}

void Squid::grabFace(bool do_cut)
{
    frame_scale = ofGetWidth() / (float)flowcam->frame.cols;
    ofRectangle extract = ofRectangle(found_face);
    extract.scaleFromCenter(face_grab_padding);
    cv::Rect face_region(extract.x / frame_scale, extract.y / frame_scale, extract.width / frame_scale, extract.height / frame_scale);
    face_region &= cv::Rect(0, 0, flowcam->frame.cols, flowcam->frame.rows);
    cv::Mat cutout = flowcam->frame(face_region).clone();
    int padding = cutout.cols / 8;
    cv::copyMakeBorder(cutout, cutout, padding, padding, padding, padding, cv::BORDER_REPLICATE);
    cv::resize(cutout, cutout, cv::Size(body_radius * scale * 2, body_radius * scale * 2));
    cv::Mat mask;
    cv::Mat bgd, fgd;

    if (do_cut) {
        cv::grabCut(cutout, mask, cv::Rect(1, 1, cutout.cols - 2, cutout.rows - 2), bgd, fgd, 1, cv::GC_INIT_WITH_RECT);
        mask = ((mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD)) & face_mask_mat;
        ofxCv::dilate(mask, 16);
        ofxCv::erode(mask, 16);
    } else {
        mask = face_mask_mat;
    }

    vector<cv::Mat> channels;
    cv::split(cutout, channels);
    channels.push_back(mask);
    cv::merge(channels, cutout);
    face_anim.push_back(cutout);
    face_im.update();
    has_face = true;
}

void Squid::showCaptureHint(){
    hint_progress = 0;
    playlist.addKeyFrame(Action::tween(500.f, &hint_alpha, 255.0));
    playlist.addKeyFrame(Action::tween(4000.f, &hint_progress, 1.0));
    playlist.addKeyFrame(Action::tween(500.f, &hint_alpha, 0.0));
}

/*
 * Applies a force to the body to propel it to goal
 */
void Squid::bodyPush(double delta_t)
{
    double force = push_force * body->GetMass() * scale;
    force *= MIN(1.0, goal_distance / (2 * body_radius * scale));

    if (behavior_state == PANIC) {
        force *= panic_force_multiplier;
    }

    body->ApplyLinearImpulse(ofToB2(goal_direction * force), body->GetPosition(), true);
}

void Squid::turnToAngle(float target_angle, double delta_t)
{
    float goal_angle_relative = ofWrapRadians(target_angle - body->GetAngle() + PI / 2);
    float multiplier = MIN(1.0, abs(goal_angle_relative) / PI);

    if (behavior_state == PANIC) {
        multiplier *= panic_force_multiplier;
    }

    if (goal_angle_relative > 0.2) {
        body->ApplyTorque(300.0 * multiplier, true);
    } else if (goal_angle_relative < -0.2) {
        body->ApplyTorque(-300.0 * multiplier, true);
    }
}

void Squid::turnToGoal(double delta_t)
{
    turnToAngle(goal_angle, delta_t);
}

void Squid::turnUpright(double delta_t)
{
    turnToAngle(-PI / 2, delta_t);
}

void Squid::squirt(){
    ofPoint dir = ofVec2f(0,1).rotateRad(body->GetAngle());
    fluid->addTemporalForce(pos_game, dir * scale * 25, main_color, 0.1 * body_radius * scale,  1.0f);
}

void Squid::squishPrep(double delta_t)
{
    squish += (0.8 - squish) * 0.1;
}

void Squid::squishPush(double delta_t)
{
    squish += (1.0 - squish) * 0.2;
}

/*
 * Moves the tentacles outward to prepare for a push
 */
void Squid::tentaclePrep()
{
    b2Vec2 tentacle_center = body->GetWorldPoint( ofToB2( tentacle_attach_offset * body_radius * scale) );

    for (int i = 0; i < tentacles.size(); i++) {
        b2Body* tentacle = tentacles[i];

        if (i % num_segments < 2) {
            b2Vec2 outward = tentacle->GetPosition() - tentacle_center;
            outward.Normalize();
            tentacle->ApplyForceToCenter(tentacle->GetMass() * tentacle_prep_force * outward, true);
        }
    }
}



/*
 * Applies tensor friction to the tentacles to make them trail nicely
 */
void Squid::tentacleGlide()
{
    for (int i = 0; i < tentacles.size(); i++) {
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
    drawTentacles();
    drawBody();
    
    // Draw Hint
    if (hint_alpha > 0){
        ofPushMatrix();
        ofRectangle hint_draw_rect(-body_radius, -body_radius, body_radius * 2, body_radius * 2);
        hint_draw_rect.scaleFromCenter(scale * 1.5);
        if (sees_face){
            ofTranslate(found_face.getCenter());
        } else {
            ofTranslate(pos_game);
        }
        ofPolyline p;
        ofSetLineWidth(4*scale);
        p.arc(0, 0, hint_draw_rect.width/2, hint_draw_rect.width/2, hint_progress * 360, true, 100);
        p.draw();
        ofRotate(-360.0 * hint_progress);
        ofSetColor(255,255,255,hint_alpha);
        hint_im.draw(hint_draw_rect);
        ofPopMatrix();
    }

    if (draw_debug) {
        drawDebug();
    }
}

void Squid::drawBody(){
    // Draw body
    float body_radius_s = body_radius * scale;
    // Trasnform to body position
    ofPushMatrix();
    ofTranslate(pos_game);
    ofRotate(body->GetAngle() * RAD_TO_DEG);
    //    ofRectangle body_draw_rect(-body_radius_s / squish, -body_radius_s, body_radius_s * 2 / squish, body_radius_s * 2 * squish);
    ofRectangle body_draw_rect(-body_radius_s, -body_radius_s, body_radius_s * 2, body_radius_s * 2);
    body_back_im.draw(body_draw_rect);
    ofSetColor(255, 255, 255, 255);
    
    // Draw face
    if (has_face) {
        toOf(face_anim[face_anim_current_frame], face_im);
        face_anim_current_frame = (face_anim_current_frame + 1) % face_anim.size();
        face_im.update();
        face_im.draw(body_draw_rect);
    }
    
    body_draw_rect.scaleFromCenter(1 / squish, 1.0);
    body_draw_rect.height *= squish;
    body_front_im.draw(body_draw_rect);
    ofSetColor(main_color);
    body_accent_im.draw(body_draw_rect);
    ofPopMatrix();
}

void Squid::drawTentacles(){
    ofEnableAlphaBlending();
    // Draw Tentacles
    ofRectangle tentacle_draw_rect(-segment_length * scale * 0.5, -segment_width * scale * 0.5, segment_length * scale, segment_width * scale);
    
    for (int i = 0; i < num_tentacles; i ++) {
        ofPolyline p;
        
        for (int j = 0; j < num_segments; j ++) {
            b2Body* tentacle = tentacles[i * num_segments + j];
            
            // Tentacle start
            if (j == 0) {
                p.curveTo(b2ToOf(tentacle->GetWorldPoint(ofToB2(ofPoint(- segment_length / 2, 0)))));
            }
            
            p.curveTo(b2ToOf(tentacle->GetWorldPoint(ofToB2(ofPoint(segment_length / 2, 0)))));
            
            // Tentacle end
            if (j == num_segments - 1) {
                p.curveTo(b2ToOf(tentacle->GetWorldPoint(ofToB2(ofPoint(segment_length, 0)))));
            }
            
            for (int layer = 0; layer < 2; layer ++) {
                ofPushMatrix();
                ofTranslate(b2ToOf(tentacle->GetPosition()));
                ofRotate(tentacle->GetAngle() * RAD_TO_DEG);
                int alpha = j / (float)num_segments * 255;
                
                if (layer == 0) {
                    ofSetColor(255, 255, 255, alpha);
                    tentacle_back_im.draw(tentacle_draw_rect);
                } else if (layer == 1) {
                    ofSetColor(main_color, alpha);
                    tentacle_front_im.draw(tentacle_draw_rect);
                }
                
                ofPopMatrix();
            }
        }
        
        ofSetColor(ofColor::white);
        ofSetLineWidth(2.0f * scale);
        p.draw();
    }

}

void Squid::drawDebug(){
    // Draw state
    std::string state = "";
    
    switch (behavior_state) {
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
    
    switch (motion_state) {
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
            
        case LOCK:
            state += "LOCK";
            break;
    }
    
    ofSetLineWidth(1.0f);
    ofDrawBitmapStringHighlight(state, pos_game + kLabelOffset);
    ofCircle(goal, max_goal_distance_close);
    // Draw the grids
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofSetColor(255, 0, 255, 128);
    drawMat(sections, 0, 0, ofGetWidth(), ofGetHeight(), GL_NEAREST);
    ofDisableBlendMode();
    ofSetColor(0, 255, 0, 255);
    // Draw the local area
    ofSetColor(0, 255, 255, 255);
    ofPushMatrix();
    ofScale(ofGetWidth() / kSectionsSize.width, ofGetHeight() / kSectionsSize.height);
    ofRectangle debug_local_area = toOf(local_area);
    ofNoFill();
    ofRect(debug_local_area);
    ofPopMatrix();
    ofSetColor(255, 0, 0);
    ofNoFill();
    ofRect(pos_game + ofPoint(0, 20), body_radius * scale, 12);
    ofFill();
    ofRect(pos_game + ofPoint(0, 20), ofClamp(local_flow, 0, 1) * body_radius * scale, 12);
    // Draw face search window
    ofRectangle face_roi_rect = toOf(face_roi);
    face_roi_rect.scale(frame_scale);
    face_roi_rect.x *= frame_scale;
    face_roi_rect.y *= frame_scale;
    ofPushStyle();
    ofNoFill();
    ofSetColor(ofColor::orange);
    ofRect(face_roi_rect);
    ofFill();
    ofDrawBitmapStringHighlight("face search window", face_roi_rect.getPosition() + kLabelOffset, ofColor::orange, ofColor::black);
    ofSetColor(255, 255, 255);
    ofNoFill();
    ofRect(face_roi_rect.getPosition(), face_size_min * ofGetHeight(), face_size_min * ofGetHeight());
    ofRect(face_roi_rect.getPosition(), face_size_max * ofGetHeight(), face_size_max * ofGetHeight());
    
    // Draw detected faces
    if(objectfinder.size()) {
        ofSetColor(ofColor::green);
        ofRect(found_face);
    }
}

void Squid::setScale(float in_scale){
    scale = in_scale;
    setupPhysics();
    setupTextures();
}