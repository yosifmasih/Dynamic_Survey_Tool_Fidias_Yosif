#pragma once
#include "dst/types.hpp"
#include "dst/calibration.hpp"
#include <cfloat>
#include <cmath>

namespace dst {

//phase B of calibration, we will sweep device through many orientations to hit min/max per axis
class MagSweepCalibration {
    public:
    void reset();
    void push(const Vec3& mag);     //one raw sample
    void finalize(CalResults& results); //we will use this to fill results of the mag bias/scale and b0
    
    bool any() const {return count_ > 0;}
    Vec3 min() const {return min_;}
    Vec3 max() const {return max_;}

    private:
    int count_ = 0;
    Vec3 sum_{0,0,0};
    Vec3 min_ {500, 500, 500};
    Vec3 max_ {-500, -500, -500};
};

}   //namespace dst