#include "dst/ml.hpp"
#include "dst/ml_postproc.hpp"
#include "dst/types.hpp"
#include <cstdio>
#include <cmath>

using namespace dst;

//purpose of this test file is to smoke test the ml pipeline, this incluides ml, statemachine_ML, and ml_postproc cpp files
//g++ -std=c++17 -I Software/lib/core/include \
  Software/dev/tests/src/test_ml_pipeline.cpp \
  Software/lib/core/src/ml.cpp \
  Software/lib/core/src/ml_postproc.cpp \
  Software/lib/core/src/statemachine_ML.cpp \
  -o /tmp/test_ml && /tmp/test_ml

static void test_bool(const char* name, bool got, bool want){
    if (got != want) std::printf("[FAIL] %s: got=%d want=%d\n", name, got, want);
    else             std::printf("[PASS] %s: got=%d want=%d\n", name, got, want);
}
static void test_near(const char* name, float got, float want, float eps=1e-3f){
    float err = std::fabs(got-want);
    if (err > eps) std::printf("[FAIL] %s: got=%.6f want=%.6f (|err|=%.6f)\n", name, got, want, err);
    else           std::printf("[PASS] %s: got=%.6f want=%.6f\n", name, got, want);
}

int main() {
    // Softmax sum sanity
    StateML model;
    StateInputs in{};
    in.axial_vib = 0.5f; in.lateral_vib = 0.8f;
    in.axial_var = 0.1f; in.lateral_var = 0.2f;
    in.vib_total = std::sqrt(in.axial_vib*in.axial_vib + in.lateral_vib*in.lateral_vib);
    in.vib_total_var = in.axial_var + in.lateral_var;
    in.rot=0; in.flow=0;

    MLPrediction pred = model.predict(in);
    float sum=0; for(float p: pred.probs) sum += p;
    test_near("softmax sum", sum, 1.0f, 5e-3f);

    // Postproc transitions
    MLPostProcessor post;
    post.reset(DrillClass::PumpsOff);

    // Illegal jump blocked (PumpsOff -> Rotating)
    MLPrediction fake{}; StateInputs in2{};
    fake.label = DrillClass::Rotating; in2.flow = 0; in2.rot = 0;
    DrillClass c1 = post.update(fake, in2);
    test_bool("illegal jump blocked", c1==DrillClass::PumpsOff, true);

    // Legal (PumpsOff -> PumpsOn when flow=1)
    fake.label = DrillClass::PumpsOn; in2.flow = 1; in2.rot = 0;
    DrillClass c2 = post.update(fake, in2);
    test_bool("legal transition", c2==DrillClass::PumpsOn, true);
}