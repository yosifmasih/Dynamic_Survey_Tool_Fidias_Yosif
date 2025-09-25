#include "ml.hpp"
#include "statemachine_ML.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace dst {

static inline void softmax_inplace(std::vector<float>& v){
    float mx = *std::max_element(v.begin(),v.end());
    double sum = 0.0;

    for (float &x : v) {
        x = std::exp(double(x - mx));
        sum += x;
    }

    if(sum <= 0.0)
        return;

    for (float &x : v){
        x = float(x/sum);
    }
        
}

static inline std::vector<float> make_sample_vector(const StateInputs& s) {
    std::vector<float> sample(8);
    sample[0] = s.axial_vib;        //MR_VIBA.MR
    sample[1] = s.lateral_vib;      //MR_VIBL.MR
    sample[2] = s.vib_total;        //Vib_Total
    sample[3] = s.axial_var;        //MR_VIBA.MR_var
    sample[4] = s.lateral_var;      //MR_VIBL.MR_var
    sample[5] = s.vib_total_var;    //Vib_Total_var
    sample[6] = float(s.flow != 0); //MR_Flow.M model reads as 0/1
    sample[7] = float(s.rot != 0);  //MR_Rotation model also reads as 0/1
    return sample;
}

MLPrediction StateML::predict(const StateInputs& in)const {
    std::vector<float> sample = make_sample_vector(in);
    std::vector<float> logits = xgb_classify(sample);       //sends our sample to ML
    softmax_inplace(logits);

    size_t arg = std::distance(logits.begin(), std::max_element(logits.begin(),logits.end()));

    MLPrediction out;
    out.label = static_cast<DrillClass>(arg);
    for (int i=0; i<5; ++i){
        out.probs[i] = logits[i];
    }

    return out;
}

}// namespace dst