#include <cmath>

#include "box_drawer.h"

//--------------------------------------------------------------
void BoxDrawer::setup() {
    boxes_.resize(kBoxNum);
}

void BoxDrawer::update() {
    updateRotateAngle();
    updateTranslateDistance();
    updateBoxSize();
}

void BoxDrawer::updateRotateAngle() {
    //float ratio   = 0.05f;
    //rotate_angle_ = std::fmod(ofGetElapsedTimeMillis() * ratio, 360.f);
    rotate_angle_ = 2.f;
}

void BoxDrawer::updateTranslateDistance() {
    float angle_deg = std::fmod(360.f * translate_count_, 360.f);
    float angle_rad = ofDegToRad(angle_deg);
    float distance_ratio = (sin(angle_rad) + 1.0f) * 0.5f;

    translate_distance_ = kBoxDistanceBase - kBoxDistanceBase * distance_ratio;
    updateTranslateCount();
}

void BoxDrawer::updateTranslateCount() {
    float ratio = 2.f;
    float threshold = kBoxDistanceBase * 0.5f;
    if (translate_distance_ > threshold) {
        float ratio_2 = ofMap(translate_distance_, threshold, kBoxDistanceBase, 1.0f, 0.4f);
        ratio *= ratio_2;
    }

    translate_count_ += (kTranslateCountBase * ratio);
}

void BoxDrawer::updateBoxSize() {
    float ratio = 1.0f;
    float threshold = kBoxDistanceBase * 0.4f;
    if (translate_distance_ < threshold) {
        float ratio_2 = ofMap(translate_distance_, 0.f, threshold, 0.9f, 1.0f);
        ratio *= ratio_2;
    }

    box_size_ = (kBoxSize * ratio);
}

void BoxDrawer::draw() {
    ofSetColor(230, 0, 200, 128);
    ofNoFill();

    float x_offset = -translate_distance_;
    for (auto& box : boxes_) {
        box.setPosition(x_offset, 0.0, 500.0);
        box.rotate(rotate_angle_, 1.0, 1.0, 1.0);
        box.drawFaces();

        x_offset += translate_distance_;
    }
}
