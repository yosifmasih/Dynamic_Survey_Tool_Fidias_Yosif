#pragma once            // includes this header only once per compilation (prevents redef)
#include <cstdint>      // fixed-width int types

namespace dst{          // this will help avoid conflicts

//Struct of 3D vector for each sensor
struct Vec3 {
    float x=0, y=0, z=0;
};

// raw sample from sensors
struct MultiSample  {
    Vec3 acc_lo;        //low-g accel
    Vec3 acc_med;       //medium-g accel
    Vec3 acc_hi;        //high-g accel
    Vec3 mag;
    float temp_c = 0.0f;
    uint64_t t_us = 0;      // timestamp (microsecs)
};

// SM/ML state classes (post process final label)
enum class DrillClass : uint8_t {
    PumpsOff = 0,
    PumpsOn = 1,
    SlidingDrilling = 2,
    Rotating = 3,
    RotatingDrilling = 4
};

// 1Hz log record(logging to memory)
struct LogRecord {
    uint64_t t_us = 0;
    Vec3 accel_lo_mean{};
    Vec3 accel_mid_mean{};
    Vec3 accel_hi_mean{};
    Vec3 mag_mean{};
    float temp_c_mean = 0.0f;
    float azimuth_mean = 0.0f;
    float mag_dip_mean = 0.0f;
    float inc_mean = 0.0f;
    DrillClass drilling_state = DrillClass::PumpsOff;
    float rpm = 0.0f;   //we might want this
};

// inputs for SM/ML model
struct StateInputs {
    float axial_vib = 0.0f;
    float lateral_vib = 0.0f;
    float axial_var = 0.0f;
    float lateral_var = 0.0f;
    float vib_total = 0.0f;
    float vib_total_var = 0.0f;
    uint8_t rot = 0;        // 0 or 1 flag from detector
    uint8_t flow = 0;       // 0 or 1 flag from detector
};



// heading outputs we compute in each pipeline
struct Heading {
    float inclination_deg = 0.0f;
    float mag_dip_deg = 0.0f;
    float azimuth_deg = 0.0f;
};

} // namespace dst