#include "dst/features.hpp"
#include <cmath>
#include <algorithm>

namespace dst{

// any variable with a _ denotes it is a member variable

// Rolling Window
RollingWindow::RollingWindow(int window) : win_(std::max(1, window)), buf_(win_, 0.0f) {}

void RollingWindow::reset(){
    std::fill(buf_.begin(), buf_.end(), 0.0f);      // this clears buffer to all zeros
    n_ = 0;
    head_ = 0;
    sum_ = 0.0;
    sumsq_ = 0.0;
    
}

void RollingWindow::push(float x){
    // this runs if buffer is not yet full
    if (n_ < win_) {        
        buf_[head_] = x;
        head_ = (head_ + 1) % win_;
        n_++;
        sum_ += x;
        sumsq_ += double(x) * double(x);
    } else { // else runs if buffer is full meaning we overwrite oldest val
        int idx = head_;
        float old = buf_[idx];
        buf_[idx] = x;
        head_ = (head_ + 1) % win_;
        sum_ += x - old;
        sumsq_ += double(x) *  double(x) - double(old) * double(old);
    }
}

float RollingWindow::mean() const {     // simple mean calculation
    if (n_ == 0) return 0.0f;
    return float(sum_ / double(n_));
}

float RollingWindow::var_sample() const {
    if (n_ < 2) return 0.0f;

    double mu = sum_/double(n_);
    double var_pop = (sumsq_/double(n_)) - mu*mu;
    double var_samp = var_pop * double(n_) / double(std::max(1,n_-1));

    return float(std::max(0.0, var_samp));
}

// feature extractor for ML inputs
// converts raw samples to ML ready inputs, uses external analog vib channels when provided, maintains rolling window state/stats
FeatureExtractor::FeatureExtractor(const FeatureConfig& cfg) 
: cfg_(cfg), 
axial_win_(cfg.var_window), 
lateral_win_(cfg.var_window),
total_win_(cfg.var_window) {}

void FeatureExtractor::reset() {    //simple reset function to reset states
    axial_win_.reset();
    lateral_win_.reset();
    total_win_.reset();
}

StateInputs FeatureExtractor::update(const MultiSample& s){
    const Vec3 a = pick_accel_(s);
    float axial = std::fabs(a.x);         //compute axial and lat vibs
    float lateral = std::sqrt(a.y*a.y + a.z*a.z);
    float total = std::sqrt(axial*axial + lateral*lateral);     
    
    axial_win_.push(axial);                             //update rolling stats
    lateral_win_.push(lateral);
    total_win_.push(total);

    StateInputs out{};                                  //declare output bundle
    out.axial_vib = axial;
    out.axial_var = axial_win_.var_sample();
    out.lateral_vib = lateral;
    out.lateral_var = lateral_win_.var_sample();
    out.vib_total = total;
    out.vib_total_var = total_win_.var_sample();
    return out;                                         //ML ready fatures
}

}   // namespace dst