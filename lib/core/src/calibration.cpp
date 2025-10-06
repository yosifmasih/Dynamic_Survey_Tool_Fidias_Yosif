#include "dst/calibration.hpp"
#include <cmath>

namespace dst {

CalResults StandstillCalibration::finalize() const{
    CalResults out{};
    if (count_ <= 0) return out;

    //means and averages
    const Vec3 m_lo = lo_.mean();
    const Vec3 m_med = med_.mean();
    const Vec3 m_hi = hi_.mean();
    const Vec3 m_mag = mag_.mean();

    out.acc_var_lo = lo_.var();
    out.acc_var_med = med_.var();
    out.acc_var_hi = hi_.var();
    out.mag_var = mag_.var();

    //accel biases at standstill
    constexpr float G = 9.81f;
    out.acc_lo.bias = {m_lo.x, m_lo.y, m_lo.z - G};
    out.acc_med.bias = {m_med.x, m_med.y, m_med.z - G};
    out.acc_hi.bias = {m_hi.x, m_hi.y, m_hi.z - G};

    return out;
}

}   //namespace dst