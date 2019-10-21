#pragma once

#include "ofMain.h"


class BoxDrawer {
public:
    BoxDrawer() = default;
    virtual ~BoxDrawer() = default;

    virtual void setup();
    virtual void update();
    virtual void draw();

private:
    static constexpr size_t   kBoxNum  = 3;
    static constexpr float    kBoxSize = 10.f;
    static constexpr float    kBoxDistanceBase = kBoxSize * 5.f;

    static constexpr float kTranslateCountBase = (1.0f / 30.f) * 0.1f;

    void updateRotateAngle();
    void updateTranslateDistance();
    void updateTranslateCount();
    void updateBoxSize();

    float rotate_angle_{0.f};
    float translate_distance_{0.f};
    float translate_count_{0.f};
    float box_size_{ kBoxSize };

    std::vector<ofBoxPrimitive> boxes_;

};
