#include "calibration.hpp"
#include <cmath>

namespace dst {

CalResults StandstillCalibration::finalize(const AxisCal* mag_cal) const{
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
    const float gnorm = std::sqrt(m_lo.x*m_lo.x + m_lo.y*m_lo.y + m_lo.z*m_lo.z);
    out.g0 = (std::isfinite(gnorm) && gnorm > 0.1f) ? gnorm : 9.80665f;

    //since we will calibrate mag before this, we can also compute b0 (earths mag field) at a standstill
    if(mag_cal) {
        Vec3 m = mag_.mean();
        const float mx = (m.x - mag_cal->bias.x) / mag_cal->scale.x;
        const float my = (m.y - mag_cal->bias.y) / mag_cal->scale.y;
        const float mz = (m.z - mag_cal->bias.z) / mag_cal->scale.z;
        out.b0 = std::sqrt(mx*mx + my*my + mz*mz);
    }

    return out;
}

}   //namespace dst
