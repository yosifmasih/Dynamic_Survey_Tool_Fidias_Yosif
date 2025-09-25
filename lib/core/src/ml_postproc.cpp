#include "ml_postproc.hpp"

//here we will define all LEGAL transitions for S.M.
//then fix edge cases we have found through testing

namespace dst{

bool MLPostProcessor::is_legal(DrillClass from, DrillClass to){
    if (from == to){
        return true;
    }

    switch (from)
    {
    case DrillClass::PumpsOff:
        return (to == DrillClass::PumpsOn);

    case DrillClass::PumpsOn:
        return (to == DrillClass::PumpsOn || 
                to == DrillClass::SlidingDrilling ||
                to == DrillClass::Rotating );

    case DrillClass::SlidingDrilling:
        return (to == DrillClass::PumpsOn || 
                to == DrillClass::Rotating );  

    case DrillClass::Rotating:
        return (to == DrillClass::PumpsOn ||
                to == DrillClass::RotatingDrilling );      
    
    case DrillClass::RotatingDrilling:
        return (to == DrillClass::Rotating );

    default:
        return false;
    }
}

DrillClass MLPostProcessor::update(const MLPrediction& pred, const StateInputs& in){

    DrillClass predicted = pred.label;
    DrillClass next = current_;

    //edge case found in field data: allow fallback to Pumps On if both flow and rotation drop
    if((in.flow == 0 && in.rot == 0) && (current_ == DrillClass::Rotating || current_ == DrillClass::RotatingDrilling)) {
        next = DrillClass::PumpsOn;
    }
    //edge case 2: if flow==1 and rot==1 and prev==SlidingDrilling -> force Rotating
    else if ((in.flow == 1 && in.rot == 1) && current_ == DrillClass::SlidingDrilling){
        next = DrillClass::Rotating;
    }
    //finally we check if the ML predicts a legal transition
    else if (is_legal(current_, predicted)) {
        next = predicted;
    }
    //if none of these trigger next to change, then it is an illegal jump and we hold current state
    current_ = next;
    return current_;
}

}   //namespace dst