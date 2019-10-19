#include "ofApp.h"

#include <rsutil.h>

#include "pose_util.h"

//--------------------------------------------------------------
void ofApp::setup() {
    initializeFramewok();

    if (!initializeT265()) {
        return;
    }

    fisheye_texture_.allocate(intrinsics.width, intrinsics.height, GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    box_drawer_.setup();

    is_start_ = true;
}

void ofApp::initializeFramewok() {
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    ofSetBackgroundColor(0);

    ofSetCircleResolution(60);
}

bool ofApp::initializeT265() {
    try {
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 2);

        pipe_profile = pipe.start(cfg);

        fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, FISH_EYE_SENSOR_USE_INDEX);
        intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
        extrinsics = fisheye_stream.get_extrinsics_to(pipe_profile.get_stream(RS2_STREAM_POSE));
    }
    catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return false;
    }

    return true;
}

//--------------------------------------------------------------
void ofApp::update() {
    if (!updateT265()) {
        return;
    }
    box_drawer_.update();
}

bool ofApp::updateT265() {
    if (!is_start_) {
        return false;
    }

    auto frames = pipe.wait_for_frames();
    rs2::video_frame fisheye_frame  = frames.get_fisheye_frame(FISH_EYE_SENSOR_USE_INDEX);
    rs2::pose_frame pose_frame      = frames.get_pose_frame();
    device_pose_in_world            = pose_frame.get_pose_data();

    fisheye_texture_.loadData(static_cast<const uint8_t*>(fisheye_frame.get_data()),
                              intrinsics.width,
                              intrinsics.height,
                              GL_LUMINANCE);

    return true;
}

//--------------------------------------------------------------
void ofApp::draw() {
    drawFisheye();

    arBigin();

    ofSetColor(0);
    ofDrawGrid(100, 50, true, true, true, false);

    box_drawer_.draw();

    arEnd();
}

void ofApp::drawFisheye() {
    auto w = ofGetWidth();
    auto h = ofGetHeight();

    auto w_offset = (w - intrinsics.width) / 2;
    auto h_offset = (h - intrinsics.height) / 2;

    ofSetColor(255);
    fisheye_texture_.draw(w_offset, h_offset);
}

void ofApp::arBigin() {
    ofPushMatrix();
    ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);

    float x = -device_pose_in_world.translation.x * 1000;
    float y = -device_pose_in_world.translation.y * 1000;
    float z = -device_pose_in_world.translation.z * 1000;

    float x2 = extrinsics.rotation[0] * x + extrinsics.rotation[3] * y + extrinsics.rotation[6] * z + extrinsics.translation[0] * 1000;
    float y2 = extrinsics.rotation[1] * x + extrinsics.rotation[4] * y + extrinsics.rotation[7] * z + extrinsics.translation[1] * 1000;
    float z2 = extrinsics.rotation[2] * x + extrinsics.rotation[5] * y + extrinsics.rotation[8] * z + extrinsics.translation[2] * 1000;

    cam_.setPosition(-x2,
                     y2,
                     z2);

    ofQuaternion q(device_pose_in_world.rotation.x,
                   device_pose_in_world.rotation.y,
                   device_pose_in_world.rotation.z,
                   device_pose_in_world.rotation.w);

    cam_.setOrientation(q);
    cam_.begin();
}

void ofApp::arEnd() {
    cam_.end();
    ofPopMatrix();
}
