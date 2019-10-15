#pragma once

#include "ofMain.h"

#include <rs.hpp>

#include "box_drawer.h"

class ofApp : public ofBaseApp {

public:
    void setup();
    void update();
    void draw();

private:
    static const int FISH_EYE_SENSOR_USE_INDEX = 1;


    void initializeFramewok();
    bool initializeT265();
    bool updateT265();

    void drawFisheye();
    void arBigin();
    void arEnd();


    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile pipe_profile;
    rs2::stream_profile fisheye_stream;
    rs2_intrinsics intrinsics;
    rs2_extrinsics extrinsics;

    rs2_pose device_pose_in_world;

    ofTexture fisheye_texture_;

    bool is_start_{ false };

    ofCamera cam_;
    BoxDrawer box_drawer_;
};
