#include "dst/algorithm.hpp"
#include <cmath>
#include <algorithm>

#ifndef DST_PI
constexpr float DST_PI = 3.14159265358979323846f;
#endif

namespace dst {

Heading Algorithm::compute_heading_(const Vec3& a, const Vec3& m) {
    Heading heading{};

    // total field strengths
    float gTotal = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    float mTotal = std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    const float eps = 1e-6; //avoid dividing by zero

    gTotal = std::max(gTotal,eps);
    mTotal = std::max(mTotal,eps);
    //hello
    // inclination (degrees)
    float incl = std::acos(a.z / gTotal) * (180.0f / float(DST_PI));

    // azimuth (degrees)
    float az_numerator = ((a.x*m.y) - (a.y*m.x)) * gTotal;
    float az_denom1 = m.z*(a.x*a.x + a.y*a.y);
    float az_denom2 = a.z*(a.x*m.x + a.y*m.y);
    float azim = (std::atan2(az_numerator, az_denom1-az_denom2) * (180.0f / float(DST_PI)));
    if (azim < 0.0f) {
        azim += 360.0f;
    }

    // magnetic dip angle (degrees)
    float dip_num = a.x*m.x + a.y*m.y + a.z*m.z;
    float dip = std::asin((dip_num) / (gTotal*mTotal)) * (180.0f / float(DST_PI));

    // fill in the Heading struct
    heading.inclination_deg = incl;
    heading.azimuth_deg = azim;
    heading.mag_dip_deg = dip;

    return heading;
}

void Algorithm::init(const AlgoConfig& cfg){   
    cfg_ = cfg;
    fe_ = FeatureExtractor(cfg_.feat);
    det_ = RotationFlowDetector(cfg_.det);
    post_.reset(DrillClass::PumpsOff);

    const int A = std::max(1,cfg_.flowrot.acc_ra_n);
    const int M = std::max(1,cfg_.flowrot.mag_ra_n);

    ra_lo_x_.reset(new RollingWindow(A)); ra_lo_y_.reset(new RollingWindow(A)); ra_lo_z_.reset(new RollingWindow(A));
    ra_mid_x_.reset(new RollingWindow(A)); ra_mid_y_.reset(new RollingWindow(A)); ra_mid_z_.reset(new RollingWindow(A));
    ra_hi_x_.reset(new RollingWindow(A)); ra_hi_y_.reset(new RollingWindow(A)); ra_hi_z_.reset(new RollingWindow(A));
    ra_mx_.reset(new RollingWindow(M)); ra_my_.reset(new RollingWindow(M)); ra_mz_.reset(new RollingWindow(M));

    //Kalman filter setup, later these values will be tweaked by calibration
    kf_cfg_.q_acc = 1.0f;
    kf_cfg_.q_mag = 1.0f;
    kf_cfg_.r_acc = 1.0f;
    kf_cfg_.r_mag = 1.0f;
    kf_cfg_.p0_acc = 10.0f * kf_cfg_.r_acc;
    kf_cfg_.P0_mag = 10.0f * kf_cfg_.r_mag;
    kf_cfg_.lambda_norm = 0.1f;     //lambda will be tuned through testing

    kf_cfg_.g0 = 1.0f;  //1g
    kf_cfg_.b0 = 56.0f; //56uT, this will also be set through calibration

    kf_.init(kf_cfg_);
}

void Algorithm::reset(){
    fe_.reset();
    det_.reset();
    post_.reset(DrillClass::PumpsOff);

    if (ra_lo_x_) {ra_lo_x_->reset();ra_lo_y_->reset();ra_lo_z_->reset();}
    if (ra_mid_x_) {ra_mid_x_->reset();ra_mid_y_->reset();ra_mid_z_->reset();}
    if (ra_hi_x_) {ra_hi_x_->reset();ra_hi_y_->reset();ra_hi_z_->reset();}
    if (ra_mx_) {ra_mx_->reset();ra_my_->reset();ra_mz_->reset();}
}


AlgoOutputs Algorithm::update(const MultiSample& s){
    AlgoOutputs output{};

    // 1) Feature extraction (ML Inputs)
    StateInputs ml_inputs = fe_.update(s);

    //2) Detectors (rotation and flow)
    auto [rot_flag, flow_flag] = det_.update(s);
    ml_inputs.rot = rot_flag;
    ml_inputs.flow = flow_flag;

    //3) ML prediction
    MLPrediction prediction = ml_.predict(ml_inputs);
    output.ml_pred = prediction.label;

    //4) ML post process and correction
    DrillClass final_label = post_.update(prediction, ml_inputs);
    output.ml_final = final_label;

    //5) filtering pipelines 

    //we will have a running average of all accels and magnetometer for further testing
    ra_lo_x_->push(s.acc_lo.x); ra_lo_y_->push(s.acc_lo.y); ra_lo_z_->push(s.acc_lo.z);
    ra_mid_x_->push(s.acc_med.x); ra_mid_y_->push(s.acc_med.y); ra_mid_z_->push(s.acc_med.z);
    ra_hi_x_->push(s.acc_hi.x); ra_hi_y_->push(s.acc_hi.y); ra_hi_z_->push(s.acc_hi.z);
    ra_mx_->push(s.mag.x); ra_my_->push(s.mag.y); ra_mz_->push(s.mag.z);
    
    //we will also run KF for every sample, but only use it for heading if we are in pipeline 3
    float z[6];
    kf_.pack_meas(s.acc_lo.x,s.acc_lo.y,s.acc_lo.z,s.mag.x,s.mag.y,s.mag.z, z);
    kf_.predict();
    kf_.update(z);
    kf_.apply_norm_const();

    //5.1) Pumps Off. low range accel for G and little to no filtering (can change once we have the accels)
    if (final_label == DrillClass::PumpsOff){
        output.heading = compute_heading_(s.acc_lo, s.mag);
    } 
      //5.2) Pumps On and Rotating. 
      else if (final_label == DrillClass::PumpsOn || final_label == DrillClass::Rotating){
        Vec3 acc_ra = {ra_lo_x_->mean(), ra_lo_y_->mean(),ra_lo_z_->mean()};
        Vec3 mag_ra = {ra_mx_->mean(),ra_my_->mean(),ra_mz_->mean()};
        output.heading = compute_heading_(acc_ra, mag_ra);

    } else if (final_label == DrillClass::RotatingDrilling || final_label == DrillClass::SlidingDrilling){
        float ax, ay, az, mx, my, mz;
        kf_.get_accel(ax, ay, az);
        kf_.get_mag(mx, my, mz);
        output.heading = compute_heading_({ax,ay,az},{mx,my,mz});
    }

    //6) heading and logging

    
    output.input = ml_inputs;
    output.flow = flow_flag;
    output.rot = rot_flag;

    return output;
}

} //namespace dst