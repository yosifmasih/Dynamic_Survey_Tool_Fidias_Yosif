#pragma once
#include <array>
#include <vector>
#include "dst/types.hpp"

namespace dst{

struct MLPrediction{
    DrillClass label;
    std::array<float,5> probs;
};

class StateML {
public:
    StateML() = default;
    MLPrediction predict(const StateInputs& in) const;    
};

} //namespace dst