
#include "squid.h"
#include "constants.h"
#include "ofxCv.h"
#include <algorithm>

using namespace ofxCv;
using namespace Playlist;

using std::cout;
using std::endl;

////////// SQUID CLASS //////////

void Squid::setup(ofPtr<b2World> in_phys_world, FlowCam* in_flowcam, MotionVisualizer* in_visualizer, HighscoreTable* in_highscores)
{
    phys_world = in_phys_world;
    flowcam = in_flowcam;
    visualizer = in_visualizer;
    highscores = in_highscores;
    squish = 1.0f;
    setGoal(ofGetWidth() / 2, ofGetHeight() / 2);
    // Tentacle attach points
    tentacle_attach.clear();
    tentacle_out_direction.clear();
    tentacle_attach.push_back(ofPoint(0.17, 0.87));
    tentacle_out_direction.push_back(ofPoint(-1, 0));
    tentacle_attach.push_back(ofPoint(0.83, 0.87));
    tentacle_out_direction.push_back(ofPoint(1, 0));
    tentacle_attach.push_back(ofPoint(0.30, 0.93));
    tentacle_out_direction.push_back(ofPoint(-1, 0));
    tentacle_attach.push_back(ofPoint(0.70, 0.93));
    tentacle_out_direction.push_back(ofPoint(1, 0));
    tentacle_attach.push_back(ofPoint(0.50, 0.95));
    tentacle_out_direction.push_back(ofPoint(0, 0));
    // Setup the physics
    setupPhysics();
    setupTextures();
    // Set up face detection
    objectfinder.setup("haarcascades/haarcascade_frontalface_alt2.xml");
    //    objectfinder.setup("haarcascades/haarcascade_profileface.xml");
    objectfinder.setRescale(1.0); // Don't rescale internally because we'll feed it a small frame
    objectfinder.setMinNeighbors(2);
    objectfinder.setMultiScaleFactor(1.3);
    objectfinder.setFindBiggestObject(true);
    objectfinder.getTracker().setSmoothingRate(0.2);
    has_face = false;
    main_color = ofColor(247, 0, 207);
}


void Squid::setupPhysics()
{
    // Clean up old physics if any
    if (body != NULL) {
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
    for (int i = 0; i < tentacle_attach.size(); i ++) {
        // Add the knee
        ofPoint attach_point = (2 * tentacle_attach[i] - 1) * body_radius * scale;
        ofPoint direction = attach_point.normalized();
        float angle = atan2(direction.y, direction.x);
        ofPoint offset = segment_join_length * segment_length * scale * direction;
        ofPoint anchor = attach_point + pos_game;
        b2Body* attach_body = body;
        float w = 0.5 * scale * segment_width / kPhysicsScale;
        float l = 0.5 * scale * segment_length / kPhysicsScale;

        for (int seg = 0; seg < num_segments; seg ++) {
            bodyDef.position = ofToB2(anchor + 0.5 * offset);
            bodyDef.angle = angle;
            bodyDef.angularDamping = 2.0f;
            bodyDef.linearDamping = tentacle_damping;
            b2Body* tentacle = phys_world->CreateBody(&bodyDef);
            b2PolygonShape box;
            box.SetAsBox(l, w * pow(0.7, seg));
            fixture.shape = &box;
            fixture.density = tentacle_density;
            fixture.filter.maskBits = 0x0;
            tentacle->CreateFixture(&fixture);
            tentacles.push_back(tentacle);
            b2RevoluteJointDef jointDef;
            jointDef.Initialize(attach_body, tentacle, ofToB2(anchor));
            jointDef.enableLimit = true;
            jointDef.upperAngle = 0.3 * PI;
            jointDef.lowerAngle = -0.3 * PI;
            b2RevoluteJoint* joint = (b2RevoluteJoint*) phys_world->CreateJoint(&jointDef);
            tentacle_joints.push_back(joint);
            // Shift
            attach_body = tentacle;
            anchor += offset;
        }
    }
}

void Squid::setupTextures()
{
    // Load textures
    body_base_back_im.loadImage("assets/body_base_back.png");
    body_base_front_im.loadImage("assets/body_base_front.png");
    body_bubble_im.loadImage("assets/body_bubble.png");
    hint_im.loadImage("assets/hint.png");
    tentacle_im.loadImage("assets/tentacle.png");
    markings_bubble_im.loadImage("assets/markings_bubble.png");
    tentacle_shine_im.loadImage("assets/tentacle_shine.png");
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
    // Some position and direction helpers
    ofPoint game_size(ofGetWidth(), ofGetHeight(), 1);
    pos_game = b2ToOf(body->GetPosition());
    pos_section = pos_game / game_size * ofPoint(kSectionsSize.width, kSectionsSize.height);
    ofPoint to_goal = (goal - pos_game);
    goal_distance = to_goal.length();
    goal_direction = to_goal.normalized();
    goal_angle = atan2(goal_direction.y, goal_direction.x);
    // Update data
    updateFlow();
//    updateObjectFinder();
    playlist.update();
    // Define some shorthands
    going_slow = b2ToOf(body->GetLinearVelocity()).length() < (min_velocity * scale);
    on_goal = (goal - pos_game).length() < max_goal_distance_close * scale;
    facing_goal = abs(ofWrapRadians(goal_angle - body->GetAngle() + PI / 2)) < DEG_TO_RAD * 30;
    // Update State machines
    updateBehaviorState(delta_t);
    updateMotionState(delta_t);
    // Display mood

    // Update face animation
    if (has_face) {
        face_time += delta_t;
        face_anim->update(delta_t);
    }
}

void Squid::updateFlow()
{
    // Subsample the flow grid to get "sections"
    cv::Mat flow_high_float;
    flowcam->flow_high.convertTo(flow_high_float, CV_32F, 1 / 255.0f);
    cv::resize(flow_high_float, sections, kSectionsSize, 0, 0,  CV_INTER_AREA);
    ofxCv::blur(sections, 2); // Blurring favors quiet sections that neighbor quiet sections.
    cv::Mat noise(kSectionsSize, CV_32F); // Add some noise to avoid (0,0) bias when looking for minimum
    cv::randu(noise, 0, 0.1);
    sections += noise;
    // Compute local area flow
    local_area = ofRectangle(pos_game - local_area_radius * scale, pos_game + local_area_radius * scale);
    core_area = ofRectangle(pos_game - core_area_radius * scale, pos_game + core_area_radius * scale);
    float scale_game_to_flow = (float)flowcam->flow_high.cols / ofGetWidth();
    cv::Rect local_area_rect(local_area.x * scale_game_to_flow,
                             local_area.y * scale_game_to_flow,
                             local_area.width * scale_game_to_flow,
                             local_area.height * scale_game_to_flow);
    cv::Rect core_area_rect(core_area.x * scale_game_to_flow,
                            core_area.y * scale_game_to_flow,
                            core_area.width * scale_game_to_flow,
                            core_area.height * scale_game_to_flow);
    cv::Rect screen_rect(0, 0, flowcam->flow_high.cols, flowcam->flow_high.rows);
    local_area_rect &= screen_rect;
    core_area_rect &= screen_rect;
//    local_area_rect
    float local_flow_sum = cv::sum(flowcam->flow_high(local_area_rect))[0] / local_area_rect.area() / 255.0;
    float core_flow_sum = cv::sum(flowcam->flow_high(core_area_rect))[0] / core_area_rect.area() / 255.0;
    local_flow = 0.8 * local_flow + 0.2 * ofMap(local_flow_sum, local_flow_min, local_flow_max, 0, 1);
    core_flow = 0.8 * core_flow + 0.2 * ofMap(core_flow_sum, local_flow_min, local_flow_max, 0, 1);
}

void Squid::updateObjectFinder()
{
    // Only compute a width because the face search window is square.
    int frame_width = flowcam->frame.cols;
    int frame_height = flowcam->frame.rows;
    scale_frame_to_game = (double)ofGetWidth() / (double)frame_width;
    int face_search_width = MIN(frame_width, frame_height) * face_search_window;
    face_roi = cv::Rect(MAX(0, MIN(frame_width - face_search_width, (pos_game.x / scale_frame_to_game - face_search_width / 2))),
                        MAX(0, MIN(frame_height - face_search_width, (pos_game.y / scale_frame_to_game - face_search_width / 2))),
                        face_search_width, face_search_width);
    // Face size is relative to frame, not face search window, so rescale and cap at 1.0
    objectfinder.setMinSizeScale(MIN(1.0, face_size_min / face_search_window));
    objectfinder.setMaxSizeScale(MIN(1.0, face_size_max / face_search_window));
    // The call below uses 2 arguments including a switch "preprocess" that
    // was added to ObjectFinder::update to disable the resize and BGR2GRAY calls.
    cv::Mat frame = flowcam->frame(face_roi);
    objectfinder.update(frame, true);

    if (objectfinder.size() > 0) {
        face_detection_count ++;

        if (face_detection_count > face_detection_threshold) {
            int label = objectfinder.getLabel(0);
            found_face = toOf(objectfinder.getTracker().getSmoothed(label));
            found_face.scale(scale_frame_to_game);
            found_face.x = (found_face.x + face_roi.x) * scale_frame_to_game;
            found_face.y = (found_face.y + face_roi.y) * scale_frame_to_game;
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
            if (core_flow >= 1) {
                switchBehaviorState(GRABBED);
                break;
            }

            if (local_flow >= 1) {
                switchBehaviorState(PANIC);
                break;
            }

            if (time_in_behavior_state > idle_move_cooldown) {
                selectQuietGoalAdjacent();
                switchBehaviorState(IDLE);
                break;
            }

            break;

        case PANIC:
            if (on_goal) {
                switchBehaviorState(IDLE);
                break;
            }

            if (!currentGoalIsQuiet()) {
                selectQuietGoal();
            }

            break;

        case FACE:
            setGoal(found_face.getCenter());

            if (time_in_behavior_state > face_pose_time) {
                grabFace();
                switchBehaviorState(PANIC);
                break;
            }

            grabFace();
            break;

        case GRABBED:
            if (time_in_behavior_state > grab_time) {
                switchBehaviorState(PANIC);
                break;
            }

            if (sees_face) {
                switchBehaviorState(FACE);
                break;
            }

            moveGoalWithFlow();
            break;
    }
}
void Squid::updateMotionState(double delta_t)
{
    time_in_motion_state += delta_t;

    switch (motion_state) {
        case STILL:
            if (!on_goal) {
                switchMotionState(PREP1);
            } else {
                turnUpright(delta_t);
            }

            break;

        case PREP1:
            tentaclePrep();
            turnToGoal(delta_t);
            squishPrep(delta_t);

            if (time_in_motion_state > motion_time_prep1 && (facing_goal || behavior_state == PANIC)) {
                switchMotionState(PREP2);
            }

            break;

        case PREP2:
            tentaclePush();
            turnToGoal(delta_t);
            squishPrep(delta_t);

            if (time_in_motion_state > motion_time_prep2) {
                switchMotionState(PUSH);
            }

            break;

        case PUSH:
            tentaclePush();
            squishPush(delta_t);
            bodyPush(delta_t);
            squirt();

            if (time_in_motion_state > motion_time_push) {
                switchMotionState(GLIDE);
            }

            break;

        case GLIDE:
            tentacleGlide();
            squirt();

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

    switch (next) {
        case IDLE:
            main_color = idle_color;
            break;

        case PANIC:
            main_color = panic_color;
            switchMotionState(STILL);
            selectQuietGoal();
            break;

        case FACE:
            main_color = face_color;
            switchMotionState(LOCK);
            showCaptureHint();
            clearFace();
            break;

        case GRABBED:
            main_color = grab_color;
            switchMotionState(LOCK);
            setGoal(pos_game);
            break;
    }

    behavior_state = next;
    ofLogVerbose("Squid") << "Switched to: " << getState();
}
void Squid::switchMotionState(MotionState next)
{
    time_in_motion_state = 0;

    switch (next) {
        case STILL:
            break;

        case PREP1:
            break;

        case PREP2:
            break;

        case PUSH:
            break;

        case GLIDE:
            break;

        case LOCK:
            break;
    }

    motion_state = next;
    ofLogVerbose("Squid") << "Switched to: " << getState();
}


void Squid::selectQuietGoalInRegion(cv::Rect bounds)
{
    double minVal, maxVal;
    cv::Point2i minLoc, maxLoc;
    minMaxLoc( sections(bounds), &minVal, &maxVal, &minLoc, &maxLoc );
    minLoc.x += bounds.x;
    minLoc.y += bounds.y;
    goal_section = minLoc;
    setGoal((toOf(goal_section) + ofPoint(ofRandomuf(), ofRandomuf())) / kSectionsSize * ofPoint(ofGetWidth(), ofGetHeight()));
    ofLogVerbose("Squid") << "goal section " << goal_section.x << "," << goal_section.y;
}

void Squid::selectQuietGoalAdjacent()
{
    selectQuietGoalInRegion(cv::Rect(pos_section.x - 1, pos_section.y - 1, 3, 3) & cv::Rect(0, 0, sections.cols, sections.rows));
}

void Squid::selectQuietGoal()
{
    selectQuietGoalInRegion(cv::Rect(0, 0, sections.cols, sections.rows));
}

void Squid::moveGoalWithFlow()
{
    ofPoint scale_flow_to_game = ofPoint(ofGetWidth() / (float)flowcam->flow.cols, ofGetHeight() / (float)flowcam->flow.rows);
    ofPoint closest;
    float min_distance = ofGetWidth() * ofGetHeight();

    for (int i = 0; i < flowcam->contourfinder_high.size(); i++) {
        ofPoint center = toOf(flowcam->contourfinder_high.getCenter(i)) * scale_flow_to_game;
        float distance = pos_game.distance(center);

        if (distance < min_distance) {
            min_distance = distance;
            closest = center;
        }
    }

    ofPoint pos_flow = goal / scale_flow_to_game;
    ofPoint direction = toOf(flowcam->flow.at<cv::Vec2f>(pos_flow.y, pos_flow.x));

    if (direction.length() < flowcam->flow_threshold_low && min_distance < local_area_radius * scale) {
        setGoal(closest);
    } else {
        setGoal(goal + direction * scale_flow_to_game);
    }
}

bool Squid::currentGoalIsQuiet()
{
    float flow_in_goal_section = sections.at<float>(goal_section.y , goal_section.x);
    return flow_in_goal_section <= 0.1;
}

void Squid::selectFaceGoal()
{
    goal = found_face.getCenter();
}

void Squid::setGoal(float x, float y)
{
    goal = ofPoint(ofClamp(x, 0, ofGetWidth() - 1), ofClamp(y, 0, ofGetHeight() - 1));
}

void Squid::setGoal(ofPoint in_goal)
{
    setGoal(in_goal.x, in_goal.y);
}

void Squid::clearFace()
{
    if (has_face) {
        highscores->add(face_time, face_anim);
        has_face = false;
    }

    face_time = 0.0;
    face_anim = ofPtr<FrameRecord>(new FrameRecord(face_mask_mat));
}

void Squid::grabFace()
{
    scale_frame_to_game = ofGetWidth() / (float)flowcam->frame.cols;
    ofRectangle extract = ofRectangle(found_face);
    extract.scaleFromCenter(face_grab_padding);
    cv::Rect face_region(extract.x / scale_frame_to_game, extract.y / scale_frame_to_game, extract.width / scale_frame_to_game, extract.height / scale_frame_to_game);
    face_region &= cv::Rect(0, 0, flowcam->frame.cols, flowcam->frame.rows);
    face_anim->grab(flowcam->frame, face_region, 0.2);
    has_face = true;
}

void Squid::showCaptureHint()
{
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
    if (goal_distance < 1.0) return;

    double force = push_force * body->GetMass() * scale;
    force *= MIN(1.0, goal_distance / (2 * body_radius * scale));

    if (behavior_state == PANIC) {
        force *= panic_force_multiplier;
    }

    body->ApplyLinearImpulse(ofToB2(goal_direction * force), body->GetWorldPoint(ofToB2(ofPoint(0.0, -0.5) * scale * body_radius)), true);
}

void Squid::turnToAngle(float target_angle, double delta_t)
{
    float goal_angle_relative = ofWrapRadians(target_angle - body->GetAngle() + PI / 2);
    float multiplier = MIN(1.0, abs(goal_angle_relative) / PI);

    if (behavior_state == PANIC) {
        multiplier *= panic_force_multiplier;
    }

    if (goal_angle_relative > 0.2) {
        body->ApplyTorque(100.0 * multiplier * body->GetMass(), true);
    } else if (goal_angle_relative < -0.2) {
        body->ApplyTorque(-100.0 * multiplier * body->GetMass(), true);
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

void Squid::squirt()
{
    ofPoint dir = ofVec2f(0, 1).rotateRad(body->GetAngle());
    visualizer->trail(getPosition(), dir, body_radius * scale);
}

void Squid::squishPrep(double delta_t)
{
    squish += (0.7 - squish) * 0.1;
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
    b2Vec2 tentacle_center = body->GetWorldPoint( ofToB2( ofPoint(0.0, 1.0) * body_radius * scale) );

    for (int i = 0; i < tentacle_attach.size(); i++) {
        b2Body* tentacle = tentacles[i * num_segments];
        b2Vec2 outward = body->GetWorldVector(ofToB2(tentacle_out_direction[i]));
        outward.Normalize();
        tentacle->ApplyForceToCenter(tentacle->GetMass() * tentacle_prep_force * outward, true);
    }
}


void Squid::tentaclePush()
{
    for (int i = 0; i < tentacle_attach.size(); i++) {
        b2Body* tentacle = tentacles[i * num_segments];
        float current = tentacle_joints[i]->GetJointAngle();
//            tentacle->ApplyTorque(-current *tentacle_push_force*tentacle->GetMass(), true);
        b2Vec2 inward = - body->GetWorldVector(ofToB2(tentacle_out_direction[i / 2]));
        inward.Normalize();
        tentacle->ApplyForceToCenter(tentacle->GetMass() * tentacle_push_force * inward, true);
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
    if (hint_alpha > 0) {
        ofPushMatrix();
        ofRectangle hint_draw_rect(-body_radius, -body_radius, body_radius * 2, body_radius * 2);
//        ofRectangle hint_draw_rect(-found_face.width / 2, -found_face.height / 2, found_face.width, found_face.height);
        hint_draw_rect.scaleFromCenter(scale * 1.5);
        ofTranslate(found_face.getCenter());
        ofPolyline p;
        ofSetLineWidth(4 * scale);
        p.arc(0, 0, hint_draw_rect.width / 2, hint_draw_rect.width / 2, hint_progress * 360, true, 100);
        p.draw();
        ofRotate(-360.0 * hint_progress);
        ofSetColor(255, 255, 255, hint_alpha);
        hint_im.draw(hint_draw_rect);
        ofPopMatrix();
    }

    if (draw_debug) {
        drawDebug();
    }
}

void Squid::drawBody()
{
    // Draw body
    float body_radius_s = body_radius * scale;
    // Trasnform to body position
    ofPushMatrix();
    ofTranslate(pos_game);
    ofRotate(body->GetAngle() * RAD_TO_DEG);
    ofRectangle body_draw_rect(-body_radius_s, -body_radius_s, body_radius_s * 2, body_radius_s * 2);
    ofRectangle body_draw_rect_squished(body_draw_rect);
    body_draw_rect_squished.height *= squish;
    body_draw_rect_squished.y += body_draw_rect_squished.height * (1 - squish);
    body_base_back_im.draw(body_draw_rect);
    ofSetColor(255, 255, 255, 255);

    // Draw face
    if (has_face) {
        face_anim->draw(body_draw_rect_squished);
    }

    body_base_front_im.draw(body_draw_rect);
    body_bubble_im.draw(body_draw_rect_squished);
    ofSetColor(main_color);
    markings_bubble_im.draw(body_draw_rect_squished);
    ofPopMatrix();
}

void Squid::drawTentacles()
{
    ofEnableAlphaBlending();
    // Draw Tentacles
    ofRectangle tentacle_draw_rect(-segment_length * scale * 0.5, -segment_width * scale * 0.5, segment_length * scale, segment_width * scale);

    for (int i = 0; i < tentacles.size(); i ++) {
        ofPolyline p;
        b2Body* tentacle = tentacles[i];
        ofPushMatrix();
        ofTranslate(b2ToOf(tentacle->GetPosition()));
        ofRotate(tentacle->GetAngle() * RAD_TO_DEG);
//        tentacle_im.draw(tentacle_draw_rect);
        ofPoint shine_offset = 0.14 * ofVec2f(tentacle_draw_rect.width, tentacle_draw_rect.height) * ofVec2f(-1, 1).rotateRad(-tentacle->GetAngle());
//        shine_offset.rotateRad(-tentacle->GetAngle());
        ofTranslate(shine_offset);
//        tentacle_shine_im.draw(tentacle_draw_rect);
        ofPopMatrix();
    }

    float w = 0.5 * segment_width / kPhysicsScale;
    float l = 0.5 * segment_length / kPhysicsScale;

    // Alternate method: splines
    for (int i = 0; i < tentacle_attach.size(); i ++) {
        b2Body* seg1 = tentacles[i * num_segments];
        b2Body* seg2 = tentacles[i * num_segments + 1];
        ofPath p;
        p.setCurveResolution(4);
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(-l, 0.6 * -w))));
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(-l, 0.6 * w))));
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(0, w))));
        p.curveTo(b2ToOf(seg2->GetWorldPoint(b2Vec2(0, 0.7 * w))));
        p.curveTo(b2ToOf(seg2->GetWorldPoint(b2Vec2(l, 0.2 * w))));
        p.curveTo(b2ToOf(seg2->GetWorldPoint(b2Vec2(l, 0.2 * -w))));
        p.curveTo(b2ToOf(seg2->GetWorldPoint(b2Vec2(0, 0.7 * -w))));
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(0, -w))));
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(-l, 0.6 * -w))));
        p.curveTo(b2ToOf(seg1->GetWorldPoint(b2Vec2(-l, 0.6 * w))));
//        p.close();
        p.setColor(body_color);
        p.draw();
        ofSetColor(ofColor::white);
        ofSetLineWidth(2.0 * scale);
        p.getOutline()[0].draw();
    }
}

void Squid::drawDebug()
{
    std::string state = getState();
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
    ofNoFill();
    ofSetColor(ofColor::green);
    ofRect(local_area);
    ofSetColor(ofColor::red);
    ofRect(core_area);
    // Local flow meter
    ofNoFill();
    ofSetColor(ofColor::red);
    ofRect(pos_game + ofPoint(0, 20), body_radius * scale, 12);
    ofFill();
    ofRect(pos_game + ofPoint(0, 20), ofClamp(local_flow, 0, 1) * body_radius * scale, 12);
    // Draw face search window
    ofRectangle face_roi_rect = toOf(face_roi);
    face_roi_rect.scale(scale_frame_to_game);
    face_roi_rect.x *= scale_frame_to_game;
    face_roi_rect.y *= scale_frame_to_game;
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
        ofPushStyle();
        ofSetLineWidth(2.0);
        ofSetColor(ofColor::blue);
        ofRect(found_face);
        ofPopStyle();
    }
}

ofPoint Squid::getPosition() const
{
    return b2ToOf(body->GetPosition());
};
float Squid::getBodyAngle() const
{
    return body->GetAngle();
}
ofPoint Squid::getGoalDirection() const
{
    return ofPoint(goal_direction);
}

std::string Squid::getState()
{
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

        case GRABBED:
            state += "GRABBED";
            break;
    }

    state += " / ";

    switch (motion_state) {
        case STILL:
            state += "STILL";
            break;

        case PREP1:
            state += "PREP1";
            break;

        case PREP2:
            state += "PREP2";
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

    return state;
}

void Squid::setScale(float in_scale)
{
    scale = in_scale;
    setupPhysics();
    setupTextures();
}