#include "dst/types.hpp"
#include "dst/features.hpp"
#include "dst/calibration.hpp"      // StandstillCalibration
#include "dst/mag_sweep_cal.hpp"    // MagSweepCalibration
#include <cstdio>
#include <cmath>

using namespace dst;

//purpose of this test is to smoke test the calibration files, for accel and mag
//g++ -std=c++17 -I Software/lib/core/include \
  Software/dev/tests/src/test_calibration.cpp \
  Software/lib/core/src/features.cpp \
  Software/lib/core/src/calibration.cpp \
  Software/lib/core/src/mag_sweep_cal.cpp \
  -o /tmp/test_calibration && /tmp/test_calibration

static void test_near(const char* name, float got, float want, float eps=1e-3f){
    float err = std::fabs(got-want);
    if (err > eps) std::printf("[FAIL] %s: got=%.6f want=%.6f (|err|=%.6f)\n", name, got, want, err);
    else           std::printf("[PASS] %s: got=%.6f want=%.6f\n", name, got, want);
}

int main(){
    // Phase B first → mag bias/scale
    CalResults cal{};
    MagSweepCalibration sweep;
    sweep.push({-50,  0,  0}); sweep.push({+50,  0,  0});
    sweep.push({  0,-30,  0}); sweep.push({  0,+30,  0});
    sweep.push({  0,  0,-40}); sweep.push({  0,  0,+40});
    sweep.finalize(cal);

    test_near("mag.bias.x",  cal.mag.bias.x, 0.0f);
    test_near("mag.scale.x", cal.mag.scale.x, 50.0f);
    test_near("mag.scale.y", cal.mag.scale.y, 30.0f);
    test_near("mag.scale.z", cal.mag.scale.z, 40.0f);

    // Phase A standstill → accel biases, variances, g0 + b0 from calibrated mag mean
    StandstillCalibration A({.samples=500});
    MultiSample s{};
    s.acc_lo  = {0.10f, -0.20f, 10.10f};
    s.acc_med = s.acc_lo; s.acc_hi = s.acc_lo;
    s.mag     = {5, 0, 0}; // constant at standstill

    for (int i=0;i<500;++i) A.push(s);
    CalResults R = A.finalize(&cal.mag); // compute b0 with bias+scale

    test_near("acc_lo.bias.x",  R.acc_lo.bias.x,  0.10f);
    test_near("acc_lo.bias.y",  R.acc_lo.bias.y, -0.20f);
    test_near("acc_lo.bias.z",  R.acc_lo.bias.z,  10.10f - 9.81f);

    float gnorm = std::sqrt(0.10f*0.10f + 0.20f*0.20f + 10.10f*10.10f);
    test_near("g0", R.g0, gnorm);

    float mx = (5.0f - cal.mag.bias.x) / cal.mag.scale.x;
    float b0_expected = std::sqrt(mx*mx + 0 + 0);
    test_near("b0", R.b0, b0_expected);
}