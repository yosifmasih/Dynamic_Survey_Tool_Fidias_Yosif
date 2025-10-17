/*
 * orientation.h
 *
 *  Created on: Aug 10, 2025
 *      Author: yosif
 */

#ifndef INC_ORIENTATION_H_
#define INC_ORIENTATION_H_

#include <stdint.h>

float heading_from_mag_level(float mx_uT, float my_uT, float decl_deg);
float magnetic_dip_from_mag(float mx_uT, float my_uT, float mz_uT);

typedef struct {
    float azimuth_deg;      // 0..360 (yaw, compass heading)
    float inclination_deg;  // pitch from gravity (0..180 or -90..+90 style)
    float dip_deg;          // magnetic dip angle
    float tot_grav_g;       // |accel|
    float tot_mag_uT;       // |mag|
} Orientation;

void compute_orientation(
    float ax_g, float ay_g, float az_g,          // calibrated accel (g)
    float mx_uT, float my_uT, float mz_uT,       // calibrated mag (µT)
    float declination_deg,                       // local magnetic declination
    Orientation* out);

// Tilt-compensate magnetometer using gravity unit vector ghat (axially aligned).
// Outputs horizontal components (mxh, myh) and vertical component (mzh) in the leveled frame.
void orientation_tilt_compensate_mag(const float ghat[3],
                                     float mx, float my, float mz,
                                     float* mxh, float* myh, float* mzh);


#endif

