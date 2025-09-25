#pragma once
#include <cstddef>      //defines common c++ types
#include <cstdint>
#include <vector>
#include "types.hpp"

// we will use structs to contain related data
// we will use classes to help with abstraction, not just data

namespace dst{

// rolling window with sample variance (implementation manually for effic, portab, and minimal dependencies)
class RollingWindow {
public:
    explicit RollingWindow(int window);
    void reset();
    void push(float x);
    int size()          const {return n_;}
    int capacity()      const {return win_;}
    float mean ()       const;
    float var_sample()  const; // returns 0 if n<2

private:
    int win_;
    int n_ = 0;
    int head_ = 0;
    std::vector<float> buf_;
    double sum_ = 0.0;
    double sumsq_ = 0.0;
};

// here we convert raw samples into ML inps (axial/lateral vib/var)
struct FeatureConfig {
    int var_window = 10;            // can adjust as we need
    enum class VibSource : uint8_t {low, mid, high} vib_source = VibSource::mid;
};

class FeatureExtractor {
public:
    explicit FeatureExtractor(const FeatureConfig& cfg);    // initialize with feature settings
    void reset();
    StateInputs update(const MultiSample& s);   //derive vib from chosen accel

private:
    FeatureConfig cfg_;         //config
    RollingWindow axial_win_;   //rolling buffer for axial vib 
    RollingWindow lateral_win_; // rolling buyffer for lateral vib
    RollingWindow total_win_;

    inline Vec3 pick_accel_(const MultiSample& s) const {
        switch (cfg_.vib_source){
            case FeatureConfig::VibSource::low: 
                return s.acc_lo;
            case FeatureConfig::VibSource::mid: 
                return s.acc_med;
            case FeatureConfig::VibSource::high: 
                return s.acc_hi;    
            default: 
                return s.acc_med;
        }
    }
};

}   // namespace dst