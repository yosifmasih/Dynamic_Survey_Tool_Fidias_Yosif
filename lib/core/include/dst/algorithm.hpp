#pragma once
#include <cstdint>
#include <memory>
#include "types.hpp"
#include "features.hpp"
#include "detectors.hpp"
#include "ml.hpp"
#include "ml_postproc.hpp"

namespace dst{

struct AlgoConfig {
    uint32_t sample_rate_hz = 100;  //sample rate can be changed 

    FeatureConfig feat{};
    DetectorConfig det{};
    
    //needed for pipeline 2
    struct FlowRotConfig {
        int acc_ra_n = 100;
        int mag_ra_n = 50;
    } flowrot;
};

//anytime this is called in main(every input sample), we will return this
struct AlgoOutputs {
    //SM state and confidence
    DrillClass ml_pred = DrillClass::PumpsOff;
    DrillClass ml_final = DrillClass::PumpsOff;

    //detectors, features, and heading packet
    uint8_t rot = 0;
    uint8_t flow = 0;
    StateInputs input{};
    Heading heading{};

};

class Algorithm {
public:
    Algorithm() = default;

    void init(const AlgoConfig& cfg);
    AlgoOutputs update(const MultiSample& s);

    void reset();

private:
    AlgoConfig cfg_;
    FeatureExtractor fe_{FeatureConfig{}};
    RotationFlowDetector det_{DetectorConfig{}};
    StateML ml_{};
    MLPostProcessor post_{};

    std::unique_ptr<RollingWindow> ra_lo_x_, ra_lo_y_,ra_lo_z_;
    std::unique_ptr<RollingWindow> ra_mid_x_, ra_mid_y_,ra_mid_z_;
    std::unique_ptr<RollingWindow> ra_hi_x_, ra_hi_y_,ra_hi_z_;
    std::unique_ptr<RollingWindow> ra_mx_, ra_my_,ra_mz_;

    static Heading compute_heading_(const Vec3& a, const Vec3& m);
};

}   //namespace dst