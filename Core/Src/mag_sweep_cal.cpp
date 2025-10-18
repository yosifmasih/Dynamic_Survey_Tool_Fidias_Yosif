#include "mag_sweep_cal.hpp"
#include <algorithm>

namespace dst {

    void MagSweepCalibration::reset() {
        count_ = 0;
        sum_ = {0,0,0};
        min_ = {500, 500, 500};
        max_ = {-500, -500, -500};
    }

    void MagSweepCalibration::push(const Vec3& m){
        count_++;

        sum_.x += m.x;
        sum_.y += m.y;
        sum_.z += m.z;
        min_.x = std::min(min_.x, m.x);
        min_.y = std::min(min_.y, m.y);
        min_.z = std::min(min_.z, m.z);
        max_.x = std::max(max_.x, m.x);
        max_.y = std::max(max_.y, m.y);
        max_.z = std::max(max_.z, m.z);
    }

    void MagSweepCalibration::finalize(CalResults& r) {
        if (count_ <= 0) return;

        Vec3 mid { (max_.x+min_.x)*0.5f, (max_.y+min_.y)*0.5f, (max_.z+min_.z)*0.5f };
        Vec3 field { (max_.x-min_.x)*0.5f, (max_.y-min_.y)*0.5f, (max_.z-min_.z)*0.5f };

        r.mag.bias = mid;       //hard iron offsets
        r.mag.scale = field;    //soft scaling factor

    }

}   //namespace dst
