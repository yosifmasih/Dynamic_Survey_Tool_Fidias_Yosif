#pragma once            // includes this header only once per compilation (prevents redef)
#include <cstdint>
#include <cstdint>


namespace dst {

struct KFConfig {

    //Q = process noise sets how much we will allow accel and mag to wander sample to sample
    float q_acc = 0.0f; //for all 3 axies
    float q_mag = 0.0f;

    //R = measurment noise per axis, model to tell us how noisy sensors are
    float r_acc = 0.0f;
    float r_mag = 0.0f;

    //P covariance matrix
    float p0_acc = 0.0f;
    float P0_mag = 0.0f;

    //Norm Constraints targets and blend
    float g0 = 1.0f;    //expected 1g
    float b0 = 56.0f;   //expected 56uT 
    float lambda_norm = 0.1f;   //can be fine tunes
};

class Kalman6 {
public:
    Kalman6();

    void init(const KFConfig& cfg);
    void reset();

    void predict();     
    void update(const float z[6]);
    void apply_norm_const();

    void get_accel(float& ax, float& ay, float& az) const;
    void get_mag(float& mx, float& my, float& mz) const;

    void pack_meas(float ax, float ay, float az, float mx, float my, float mz, float out[6]) const;


private:
    KFConfig cfg_{};

    float X_[6]{};  //state per axis
    float P_[6]{};  //covariance per axis
    float Q_[6]{};  //process noise per axis
    float R_[6]{};  //neasurment noise per axis
    
};

}   //namespace dst