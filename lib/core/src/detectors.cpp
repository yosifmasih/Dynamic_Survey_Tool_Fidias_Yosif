#include "dst/detectors.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <deque>

namespace dst {


// RotationFlowDetector: gives us rotation, flow, and allows us to update after eachsample and reset the internals

// reset
RotationFlowDetector::RotationFlowDetector(const DetectorConfig& cfg)
:   cfg_(cfg),                                        //copy config
    mx_buf_(std::max(1, cfg.rot_window_n), 0.0f),     //presize mx and my buffers
    my_buf_(std::max(1, cfg.rot_window_n), 0.0f) {
    reset();                                        // start with a clean state
  }

void RotationFlowDetector::reset(){
    //rotating state, here we will reset everything
    std::fill(mx_buf_.begin(), mx_buf_.end(),0.0f);     //clear mx and my buffer
    std::fill(my_buf_.begin(), my_buf_.end(),0.0f);
    rot_n_ = rot_head_ = zero_cross_count_ = 0;         //reset counters
    last_mx_sign_ = last_my_sign_ = 0.0f;                //unkown sign initially
    rot_flag_ = 0;                                      //rot output low

    //flow state resets
    g_win_.clear();                                 //clear |g| time window
    g_sum_ = g_sumsq_ = 0.0;                        //reset sums and stats
    flow_mean_ = flow_var_ = 0.0f;
    flow_flag_ = 0;                                 //flow output low
    hi_accum_us_ = lo_accum_us_ = 0;                //reset these hold timers too
    last_t_us_ = 0;                                 // zero means "not intitialized"

}


// public update of the rotation and flow flags
std::pair<uint8_t,uint8_t> RotationFlowDetector::update(const MultiSample& s){
    //rotation via mag
    float mx = (std::fabs(s.mag.x) >= cfg_.rot_min_amp) ? s.mag.x : 0.0f;     //logic ignores tiny changes in mx and my 
    float my = (std::fabs(s.mag.y) >= cfg_.rot_min_amp) ? s.mag.y : 0.0f;
    update_rotation_(mx,my);        //calls rot update function

    //flow via variance of |g| around rolling mean 
    float gmag = std::sqrt(s.acc_med.x*s.acc_med.x + s.acc_med.y*s.acc_med.y +s.acc_med.z*s.acc_med.z);       //gives up |g|
    update_flow_(gmag, s.t_us);     //calls flow update function

    return {rot_flag_, flow_flag_};
}

//rotation function
void RotationFlowDetector::update_rotation_(float mx, float my){
    //push mx/my into circular buffer
    if(rot_n_ < (int)mx_buf_.size()){
        mx_buf_[rot_head_] = mx;
        my_buf_[rot_head_] = my;
        rot_head_ = (rot_head_ + 1) % mx_buf_.size();       //advance head 
        rot_n_++;                                           //increase count
    } else { //if else executes we will overwrite oldest
        int idx = rot_head_;                                //our current head is the oldest
        mx_buf_[idx] = mx;                                  // we now replace with our nex mx/my
        my_buf_[idx] = my;
        rot_head_ = (rot_head_ + 1) % mx_buf_.size();       // advance head
    }

    //we now detect zero crossings on mx/my (any sign flips excluding zeros)
    float sgn_mx = signf(mx);
    float sgn_my = signf(my);

    if(rot_n_ > 1){     //this ensures we have previous sign
        if(last_mx_sign_ != 0.0f && sgn_mx != 0.0f && sgn_mx != last_mx_sign_)  //checks for mx sign flip
            zero_cross_count_++;
        if(last_my_sign_ != 0.0f && sgn_my != 0.0f && sgn_my != last_my_sign_)  //checks for my sign flip
            zero_cross_count_++;

        zero_cross_count_ = std::min(zero_cross_count_, 2 * (int)mx_buf_.size());   //this ensures this doesnt grow unbounded due to overwrites
    }

    //here we will update to last observed signs 
    if (sgn_mx != 0.0f) last_mx_sign_ = sgn_mx;
    if (sgn_my != 0.0f) last_my_sign_ = sgn_my;

    //we can now flag rotation to be HI or LO
    rot_flag_ = (rot_n_ >= cfg_.rot_window_n && zero_cross_count_ >= cfg_.rot_min_zero_cross) ? 1:0;
}

//flow function
void RotationFlowDetector::update_flow_(float gmag, uint64_t t_us) {
    // initialize delta-t tracking for first call only
    if(last_t_us_ == 0) last_t_us_ = t_us;
    uint64_t dt_us = (t_us >= last_t_us_) ? (t_us - last_t_us_) : 0;
    last_t_us_ = t_us;

    //push new |g| with timestamp and update running sums
    g_win_.push_back({gmag, t_us});
    g_sum_ += gmag;
    g_sumsq_ += double(gmag) * double(gmag);

    //we can now get rid of samples older than the time window
    const uint64_t win_us = static_cast<uint64_t>(cfg_.flow_mean_window_s * 1e6f);      //convert s to us
    while (!g_win_.empty() && (t_us - g_win_.front().t_us) > win_us) {
        float old = g_win_.front().g;
        g_sum_ -= old;                              //subs old value from sum and sumsq
        g_sumsq_ -= double(old) * double(old);
        g_win_.pop_front();                         //physically remove from deque
    }

    // compute rolling mean/var over time window 
    const int n = (int)g_win_.size();
    if (n >= 1){
        double mean = g_sum_/double(n);                     //E[x]
        double mean_sq = g_sumsq_/double(n);                //E[x^2]
        double var = std::max(0.0, mean_sq - mean*mean);    //var = E[x^2] - (E[x])^2
        flow_mean_ = float(mean);
        flow_var_ = float(var);
    } else {
        flow_mean_ = 0.0f;
        flow_var_ = 0.0f;
    }

    //here we check if flow should be ON or OFF using our hold thresholds
    if(flow_var_ > cfg_.flow_hi_thresh) {       //ON if var > HI
        hi_accum_us_ += dt_us;
        lo_accum_us_ = 0;
    } else if (flow_var_ < cfg_.flow_low_thresh) {      //OFF if var < LO
        lo_accum_us_ += dt_us;
        hi_accum_us_ = 0;
    } else {                    //between threshold so neither counters accumulate
        hi_accum_us_ = 0;
        lo_accum_us_ = 0;
    }
    
    // now we actually compare accumulators to rewuired hold durations
    const uint64_t hi_need = static_cast<uint64_t>(cfg_.flow_hi_hold_s * 1e6f);
    const uint64_t lo_need = static_cast<uint64_t>(cfg_.flow_lo_hold_s * 1e6f);

    if(!flow_flag_ && hi_accum_us_ >= hi_need) {        //assert Flow ON
        flow_flag_ = 1;
        hi_accum_us_ = 0;
    } else if (flow_flag_ && lo_accum_us_ >= lo_need){  //assert Flow OFF
        flow_flag_ = 0;
        lo_accum_us_ = 0;
    }
}

}   // namespace dst