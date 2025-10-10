#include "dst/algorithm.hpp"
#include <cstdio>
#include <cmath>

using namespace dst;

// simple smoke test to prove we can move through the algorithm as we intend for it to
//g++ -std=c++17 -I Software/lib/core/include \
  Software/dev/tests/src/test_algo_integration.cpp \
  Software/lib/core/src/features.cpp \
  Software/lib/core/src/detectors.cpp \
  Software/lib/core/src/ml.cpp \
  Software/lib/core/src/ml_postproc.cpp \
  Software/lib/core/src/statemachine_ML.cpp \
  Software/lib/core/src/kalman.cpp \
  Software/lib/core/src/algorithm.cpp \
  -o /tmp/test_algorithm && /tmp/test_algorithm

static void test_bool(const char* name, uint8_t got, uint8_t want){
    if (got != want) std::printf("[FAIL] %s: got=%d want=%d\n", name, got, want);
    else             std::printf("[PASS] %s: got=%d want=%d\n", name, got, want);
}
static void test_in_range(const char* name, float v, float lo, float hi){
    bool ok = (v >= lo && v <= hi);
    if (!ok) std::printf("[FAIL] %s: got=%.3f want in [%.3f, %.3f]\n", name, v, lo, hi);
    else     std::printf("[PASS] %s: got=%.3f in [%.3f, %.3f]\n", name, v, lo, hi);
}

int main() {

  AlgoConfig cfg{};
  cfg.sample_rate_hz = 100;
  cfg.feat.var_window = 8;
  cfg.det.rot_window_n = 20;
  cfg.det.rot_min_zero_cross = 6;
  cfg.det.rot_min_amp = 1e-4f;
  cfg.flowrot.acc_ra_n = 16;
  cfg.flowrot.mag_ra_n = 16;

  Algorithm algo;
  algo.init(cfg);

  // Fixed pose
  uint64_t t = 0;
  AlgoOutputs last{};
  for(int i=0;i<200;++i){
    MultiSample s{};
    s.acc_lo  = {0,0,9.81f};
    s.acc_med = s.acc_lo;
    s.acc_hi  = s.acc_lo;
    s.mag     = {50,0,0};
    s.temp_c  = 25.0f;
    t += 10000; s.t_us = t;
    last = algo.update(s);
  }

  test_in_range("inclination_deg", last.heading.inclination_deg, 0.0f, 5.0f);
  test_in_range("mag_dip_deg",     last.heading.mag_dip_deg,    -10.0f, 10.0f);
  test_in_range("azimuth_deg",     last.heading.azimuth_deg,     -1.0f, 361.0f);
  test_bool("rot flag",  last.rot, 0);
  test_bool("flow flag", last.flow, 0);
  std::printf("Final ml_final = %d\n", int(last.ml_final));

}