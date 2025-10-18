#pragma once
#include "ml.hpp"
#include "dst/types.hpp"

namespace dst{

class MLPostProcessor {
public:
    MLPostProcessor() = default;

    //in update. we will feed prediction+ rot and flow, will then return corrected label
    DrillClass update(const MLPrediction& pred, const StateInputs& in);
    //we also need the current state
    DrillClass current() const {return current_;}

    //always good to have a reset
    void reset(DrillClass start = DrillClass::PumpsOff) {current_ = start;}

private:
    DrillClass current_ = DrillClass::PumpsOff;

    static bool is_legal(DrillClass from, DrillClass to);
};

} // namespace dst