#include "dst/types.hpp"
#include "dst/detectors.hpp"
#include <cmath>
#include <vector>
#include <cstdio>

//simple sanity tests for rotaion and flow
using namespace dst;

// Build (example):
// g++ -std=c++17 -I Software/lib/core/include Software/dev/tests/src/test_detectors.cpp Software/lib/core/src/detectors.cpp -o /tmp/test_detectors && /tmp/test_detectors

static void test_bool(const char* name, bool got, bool want){
    if (got != want) std::printf("[FAIL] %s: got=%d want=%d\n", name, got, want);
    else             std::printf("[PASS] %s: got=%d want=%d\n", name, got, want);
}

int main(){

    DetectorConfig cfg{};
    cfg.rot_window_n = 20;
    cfg.rot_min_zero_cross = 6;
    cfg.rot_min_amp = 1e-4f;

    cfg.flow_mean_window_s = 1.0f;
    cfg.flow_hi_thresh = 5.0f;
    cfg.flow_low_thresh = 1.0f;
    cfg.flow_hi_hold_s = 0.2f;
    cfg.flow_lo_hold_s = 0.2f;

    RotationFlowDetector det(cfg);
    det.reset();

    const int sample_rate_hz = 100;
    const uint64_t dt_us = 10000;

    auto push = [&](int n, const Vec3& acc_med, const Vec3& mag, uint64_t& t){
        for(int i=0;i<n;++i){
            MultiSample s{};
            s.acc_med = acc_med;
            s.mag = mag;
            t += dt_us; s.t_us = t;
            det.update(s);
        }
    };

    // Drive rotation: many sign flips in mx/my
    uint64_t t = 0;
    for (int k=0; k<40; ++k){
        float th = 2.0f*float(M_PI)*(k/5.0f);
        MultiSample s{};
        s.mag = { std::sin(th), std::cos(th), 0.0f };
        s.acc_med = {0,0,9.81f};
        t += dt_us; s.t_us = t;
        det.update(s);
    }
    test_bool("rot_flag", det.rot_flag(), 1);

    // Flow: low variance → high variance → low variance

    // 1) Low variance for 0.5 s
    push(50, {0,0,9.81f}, {0,0,0}, t);
    test_bool("flow low var", det.flow_flag()!=0, false);

    // 2) High variance (alternate |g|) for 0.5 s
    for (int k=0; k<50; ++k){
        float g = (k%2==0)? 0.0f : 20.0f;
        MultiSample s{};
        s.acc_med = {g,0,0};
        s.mag = {0,0,0};
        t += dt_us; s.t_us = t;
        det.update(s);
    }
    test_bool("flow high var", det.flow_flag()!=0, true);

    // 3) Back to low variance long enough to flush the 1.0 s window + satisfy the 0.2 s low-hold
    const int tail_needed = int((cfg.flow_mean_window_s + cfg.flow_lo_hold_s) * sample_rate_hz) + 10; // cushion
    push(tail_needed, {0,0,9.81f}, {0,0,0}, t);

    test_bool("flow back to low", det.flow_flag()!=0, false);

    return 0;
}