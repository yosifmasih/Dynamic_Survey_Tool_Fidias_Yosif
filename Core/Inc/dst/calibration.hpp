#pragma once
#include "dst/types.hpp"
#include "dst/features.hpp"
#include <cstdint>

namespace dst {

struct AxisCal {
    Vec3 bias {0,0,0};  //subtract from raw
    Vec3 scale {1,1,1}; //divide raw bias by scale
};

struct CalResults {
    AxisCal acc_lo, acc_med, acc_hi;    //all 3 accels to be calibrated
    AxisCal mag;                        //magnetometer

    //KF seeding
    float g0 = 9.81f;   //this will not change
    float b0 = 56.0f;   //mag field norm that will be changed after calobration

    //initialize the noise per sample
    Vec3 acc_var_lo {0,0,0}, acc_var_med {0,0,0}, acc_var_hi {0,0,0};
    Vec3 mag_var {0,0,0};
};

struct CalConfig {
    int samples = 500;  // seconds * sampling rate (5*100)
};

struct AxisWin {
    RollingWindow x, y, z;
    explicit AxisWin(int n) : x(n), y(n), z(n) {}
    void reset() {x.reset(); y.reset(), z.reset();}
    void push(const Vec3& v) {x.push(v.x); y.push(v.y); z.push(v.z);}
    Vec3 mean() const {return {x.mean(), y.mean(), z.mean()};}
    Vec3 var() const {return{x.var_sample(),y.var_sample(),z.var_sample()};}
};

// Phase A: standstill collector for the accelerometers mainly
class StandstillCalibration {
public:
    explicit StandstillCalibration (const CalConfig& cfg)
    : cfg_(cfg), lo_(cfg.samples),med_(cfg.samples), hi_(cfg.samples), mag_(cfg.samples) {}
    void reset() {
        lo_.reset(); 
        med_.reset(); 
        hi_.reset(); 
        mag_.reset(); 
        count_=0;
    }
    void push(const MultiSample& s){    //feed one sample
        lo_.push(s.acc_lo); 
        med_.push(s.acc_med); 
        hi_.push(s.acc_hi); 
        mag_.push(s.mag); 
        count_++;
    }    
    CalResults finalize(const AxisCal* mag_cal=nullptr) const;        //acccel biases and variances(accel/mag)

private:
    CalConfig cfg_;
    int count_ = 0;
    AxisWin lo_, med_, hi_, mag_;
};

}   //namespace dst