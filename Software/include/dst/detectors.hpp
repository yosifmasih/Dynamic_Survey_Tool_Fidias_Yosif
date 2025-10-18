#pragma once
#include <cstdint>
#include <utility>
#include <vector>
#include <deque>
#include "dst/types.hpp"

namespace dst {

struct DetectorConfig {
    int rot_window_n = 100;       //# of samples to examine for rotation (can change based on polling req)
    int rot_min_zero_cross = 6;   
    float rot_min_amp = 0.01f;    //helps guard the amplitude on mx/my

    float flow_mean_window_s = 1.0f;    //rolling var window
    float flow_hi_thresh = 1000.0f;     // Flow ON if var > Hi for hold time
    float flow_low_thresh = 500.0f;     // Flow OFF is var < LO for hold time
    float flow_hi_hold_s = 3.0f;
    float flow_lo_hold_s = 3.0f;
};

//rotation and flow detector 
class RotationFlowDetector {
public:
    explicit RotationFlowDetector(const DetectorConfig& cfg);
    void reset();

    std::pair<uint8_t,uint8_t> update(const MultiSample& s);     //feed one sample and update detectors

    //inspectors for logging
    float current_flow_mean_g() const {return flow_mean_;}
    float current_flow_var() const {return flow_var_;}
    uint8_t rot_flag() const {return rot_flag_;}
    uint8_t flow_flag() const {return flow_flag_;}

private:
    DetectorConfig cfg_; //copy of config

    //rotation detection
    int rot_n_ = 0, rot_head_ = 0, zero_cross_count_ = 0;   //counters  + circ buf index
    float last_mx_sign_ = 0.0f, last_my_sign_ = 0.0f;        //to track mag sign flips
    std::vector<float> mx_buf_, my_buf_;                    //stores mag samples
    uint8_t rot_flag_ = 0;

    //flow detection
    struct TimedG { float g; uint64_t t_us;};      //val and timestamp
    std::deque<TimedG> g_win_;                     // sliding window of |g| vals with timestamps
    double g_sum_ = 0.0, g_sumsq_ = 0.0;                 // running sums for mean/var
    float flow_mean_ = 0.0f, flow_var_ = 0.0f;          //current flow stats
    uint8_t flow_flag_ = 0;              

    //hold timers (only update in us while condition is true)
    uint64_t hi_accum_us_ = 0;  //time above HI threshold
    uint64_t lo_accum_us_ = 0;  //time below LO threshold
    uint64_t last_t_us_ = 0;     // last timestamp

    //helpers
    static inline float signf(float x) {return (x>0) - (x<0);}  //fast sign function
    void update_rotation_(float mx, float my);
    void update_flow_(float gmag, uint64_t t_us);
};

} // namespace dst
