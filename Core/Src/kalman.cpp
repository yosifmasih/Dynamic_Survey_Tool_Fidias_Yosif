#include "kalman.hpp"
#include <algorithm>
#include <cmath>

namespace dst{

Kalman6::Kalman6() {reset();}

void Kalman6::init(const KFConfig& cfg){

    cfg_ = cfg;

    //initialize covars and noises
    for(int i=0; i<3; ++i){     //accel
        P_[i] = cfg_.p0_acc;
        Q_[i] = cfg_.q_acc;
        R_[i] = cfg_.r_acc;
        X_[i] = 0.0f;           //fisrt updates are empty
    }
    for(int i=3; i<6; ++i){     //mag
        P_[i] = cfg_.P0_mag;
        Q_[i] = cfg_.q_mag;
        R_[i] = cfg_.r_mag;
        X_[i] = 0.0f;
    }
}

void Kalman6::reset(){
    cfg_ = KFConfig{};
    for(int i=0; i<6; ++i){     
        P_[i] = 1.0f;
        Q_[i] = 1e-3f;
        R_[i] = 1e-3f;
        X_[i] = 0.0f;
    }

}

void Kalman6::predict(){
    for(int i=0; i<6; ++i){     
        P_[i] += Q_[i];         //A=I, P = P+Q
    }
}

void Kalman6::update(const float z[6]){

    for(int i=0; i<6; ++i){
        
        const float K = P_[i] / (P_[i] + R_[i]);     //kalman gain

        X_[i] += K * (z[i] - X_[i]);                //state update

        P_[i] = (1-K) * P_[i];                      //cov update
    }

}

void Kalman6::get_accel(float& ax, float& ay, float& az) const{
    ax = X_[0]; 
    ay = X_[1];
    az = X_[2];
}

void Kalman6::get_mag(float& mx, float& my, float& mz) const{
    mx = X_[3]; 
    my = X_[4];
    mz = X_[5];
}

void Kalman6::pack_meas(float ax, float ay, float az, float mx, float my, float mz, float out[6]) const{
    out[0] = ax;
    out[1] = ay;
    out[2] = az;
    out[3] = mx;
    out[4] = my;
    out[5] = mz;
}

void Kalman6::apply_norm_const(){
    const float lambda = cfg_.lambda_norm;

    float v_ax=X_[0], v_ay=X_[1], v_az=X_[2];   //accel vals
    float v_mx=X_[3], v_my=X_[4], v_mz=X_[5];   //mag vals

    float V_a = std::sqrt(v_ax*v_ax + v_ay*v_ay + v_az*v_az);   //accel magnitude
    float V_m = std::sqrt(v_mx*v_mx + v_my*v_my + v_mz*v_mz);   //mag magnitude

    V_a = std::max(V_a, 1e-6f);
    V_m = std::max(V_m, 1e-6f);

    X_[0] = v_ax*((1.0f-lambda) + (lambda * (cfg_.g0 / V_a)));
    X_[1] = v_ay*((1.0f-lambda) + (lambda * (cfg_.g0 / V_a)));
    X_[2] = v_az*((1.0f-lambda) + (lambda * (cfg_.g0 / V_a)));

    X_[3] = v_mx*((1.0f-lambda) + (lambda * (cfg_.b0 / V_m)));
    X_[4] = v_my*((1.0f-lambda) + (lambda * (cfg_.b0 / V_m)));
    X_[5] = v_mz*((1.0f-lambda) + (lambda * (cfg_.b0 / V_m)));
}

}   //namespace dst
