#include "dst/types.hpp"
#include "dst/features.hpp"
#include <cmath>
#include <vector>
#include <cstdio>

using namespace dst;

//purpose of this test file is to test feature extractor and rolling window math(mean and variance)
//to run: g++ -std=c++17 -I./lib/core/include     dev/tests/src/test_features.cpp     lib/core/src/features.cpp     -o /tmp/test_features
//then /tmp/test_features

static void test_output(const char* name, float got, float want){
    if(got != want){
        std::printf("[FAIL] %s: got=%.6f want=%.6f\n", name,got,want);
    }
    if(got == want){
        std::printf("[PASS] %s: got=%.6f want=%.6f\n", name,got,want);

    }
}

int main() {
    
    // test 1 rolling window basic math functionality
    //data = 1,2,3,4,5. mean = 3, samp var = 2.5

    RollingWindow rw(5);
    rw.reset();

    std::vector<float> data = {1,2,3,4,5};
    for (float x : data){
        rw.push(x);
    }
    
    float mean_got = rw.mean();
    float var_got = rw.var_sample();

    test_output("RW.mean", mean_got, 3);
    test_output("RW.var_sample", var_got, 2.5);
    std::printf("INFO Before reset: size=%d\n", rw.size());
    rw.reset();
    std::printf("INFO After reset: size=%d\n", rw.size());


    //test 2 feature extractor, testing update, and axial/lat vib
    FeatureConfig cfg;
    cfg.var_window = 4;
    cfg.vib_source = FeatureConfig::VibSource::mid;
    FeatureExtractor fx(cfg);
    fx.reset();

    std::vector<MultiSample> test_samp(4); 
    //i=0, {1.0f,2.0f,2.0f,0,0,0,1000}
    test_samp[0].acc_lo = {0,0,0};
    test_samp[0].acc_med = {1,2,2};
    test_samp[0].acc_hi = {0,0,0};
    test_samp[0].mag = {0,0,0};
    test_samp[0].temp_c = 0.0f;
    test_samp[0].t_us = 1000;

    //i=1, {-2.0f,3.0f,1.0f,0,0,0,2000},    
    test_samp[1].acc_med = {-2,3,1};
    test_samp[1].t_us = 2000;
        
    //i=2, {3.0f,0.0f,4.0f,0,0,0,3000},
    test_samp[2].acc_med = {3,0,4};
    test_samp[2].t_us = 3000;

    //i=3, {-4.0f,1.0f,1.0f,0,0,0,4000},
    test_samp[3].acc_med = {-4,1,1};
    test_samp[3].t_us = 4000;
    

    for (int i=0; i<test_samp.size(); ++i){
        StateInputs out = fx.update(test_samp[i]);
        float axial_expected = std::fabs(test_samp[i].acc_med.x);
        float lateral_expected = std::sqrt(test_samp[i].acc_med.y*test_samp[i].acc_med.y + test_samp[i].acc_med.z*test_samp[i].acc_med.z);

        test_output("FE.axial_vib", out.axial_vib, axial_expected);
        test_output("FE.lateral_vib", out.lateral_vib, lateral_expected);

        std::printf("Index = %d, Axial Var = %f, Lateral Var = %f\n", i,out.axial_var,out.lateral_var);
    }
}