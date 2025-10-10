#include "dst/kalman.hpp"
#include <cstdio>
#include <cmath>

using namespace dst;

//purpose of this test is to smoke test kalman.cpp, making sure we can initialize, predict, update, and apply norm constraint the same way algorithm.cpp does
//g++ -std=c++17 -I Software/lib/core/include \
  Software/dev/tests/src/test_kalman.cpp \
  Software/lib/core/src/kalman.cpp \
  -o /tmp/test_kalman && /tmp/test_kalman

static void test_near(const char* name, float got, float want, float eps=1e-2f){
    float err = std::fabs(got-want);
    if (err > eps) std::printf("[FAIL] %s: got=%.6f want=%.6f (|err|=%.6f)\n", name, got, want, err);
    else           std::printf("[PASS] %s: got=%.6f want=%.6f\n", name, got, want);

}

int main() {

    KFConfig cfg{};
    cfg.q_acc = 0.01f; cfg.q_mag = 0.01f;
    cfg.r_acc = 0.05f; cfg.r_mag = 0.05f;
    cfg.p0_acc = 1.0f; cfg.P0_mag = 1.0f;
    cfg.g0 = 9.81f; cfg.b0 = 50.0f;
    cfg.lambda_norm = 0.1f;

    Kalman6 kf; 
    kf.init(cfg);

    float z[6];
    for (int i=0;i<200;++i){
        kf.pack_meas(0,0,9.81f, 50.0f,0,0, z);
        kf.predict();
        kf.update(z);
        kf.apply_norm_const();
    }

    float ax,ay,az, mx,my,mz;
    kf.get_accel(ax,ay,az); 
    kf.get_mag(mx,my,mz);

    float gN = std::sqrt(ax*ax+ay*ay+az*az);
    float bN = std::sqrt(mx*mx+my*my+mz*mz);

    test_near("accel norm -> g0", gN, cfg.g0, 1e-2f);
    test_near("mag norm   -> b0", bN, cfg.b0, 1e-2f);

}