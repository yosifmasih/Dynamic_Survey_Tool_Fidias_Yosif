/*
 * orientation.c
 *
 *  Created on: Aug 10, 2025
 *      Author: yosif
 */


#include "orientation.h"
#include <math.h>

static float rad2deg(float r){ return r * 57.295779513f; } // avoid M_PI portability issues

// Adjust this 3x3 to map LIS2MDL sensor-frame -> body-frame (accel frame).
// Start with identity; change signs/swaps until heading/dip make sense.
static inline void mag_remap(float mx, float my, float mz,
                             float *bx, float *by, float *bz)
{
    // IDENTITY (current)
    *bx =  mx;  *by =  my;  *bz =  mz;

    // EXAMPLES you might try (one at a time) if heading/dip are off by 90°/sign:
    // Swap X/Y (sensor rotated 90° about Z):
    // *bx =  my;  *by =  mx;  *bz =  mz;

    // Swap + negate one (sensor rotated + mirrored):
    // *bx =  my;  *by = -mx;  *bz =  mz;

    // Flip Z (sensor Z points opposite accel Z):
    // *bx =  mx;  *by =  my;  *bz = -mz;

    // etc.
}


void orientation_tilt_compensate_mag(const float ghat[3],
                                     float mx, float my, float mz,
                                     float* mxh, float* myh, float* mzh)
{
    // roll & pitch from gravity unit vector
    float roll  = atan2f(ghat[1], ghat[2]);
    float pitch = atan2f(-ghat[0], sqrtf(ghat[1]*ghat[1] + ghat[2]*ghat[2]));

    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);

    // Rotate magnetometer into the leveled frame (Xh, Yh in horizontal plane; Zh vertical)
    float Xh =  mx*cp + mz*sp;
    float Yh =  mx*sr*sp + my*cr - mz*sr*cp;
    float Zh = -mx*cr*sp + my*sr + mz*cr*cp;

    if (mxh) *mxh = Xh;
    if (myh) *myh = Yh;
    if (mzh) *mzh = Zh;
}


// If LIS2MDL axes don’t match the accel board axes, remap here:
static void remap_mag_to_body(float mx, float my, float mz,
                              float* bx, float* by, float* bz){
    *bx = mx; *by = my; *bz = mz; // identity mapping
}

void compute_orientation(float ax_g, float ay_g, float az_g,
                         float mx_uT, float my_uT, float mz_uT,
                         float decl_deg,
                         Orientation* out)
{
    // 1) normalize gravity (guard against divide-by-zero)
    float g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (g < 1e-6f) {
        out->azimuth_deg = 0.0f;
        out->inclination_deg = 0.0f;
        out->dip_deg = 0.0f;
        out->tot_grav_g = g;
        out->tot_mag_uT = sqrtf(mx_uT*mx_uT + my_uT*my_uT + mz_uT*mz_uT);
        return;
    }

    float nx = ax_g / g, ny = ay_g / g, nz = az_g / g;

    // 2) roll & pitch from accel
    float roll  = atan2f(ny, nz);                            // rad
    float pitch = atan2f(-nx, sqrtf(ny*ny + nz*nz));         // rad

    // 3) remap/tilt-compensate magnetometer
    float mx_b, my_b, mz_b;
    remap_mag_to_body(mx_uT, my_uT, mz_uT, &mx_b, &my_b, &mz_b);

    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);

    float mxh = mx_b*cp + mz_b*sp;
    float myh = mx_b*sr*sp + my_b*cr - mz_b*sr*cp;

    // 4) azimuth (yaw), add declination, wrap 0..360
    float azimuth_deg = rad2deg(atan2f(-myh, mxh)) + decl_deg;
    if (azimuth_deg < 0)   azimuth_deg += 360.0f;
    if (azimuth_deg >= 360.0f) azimuth_deg -= 360.0f;

    // 5) inclination from gravity & magnetic dip
    float inc_deg = rad2deg(acosf(nz));                           // 0..180 from gravity
    float tot_mag = sqrtf(mx_uT*mx_uT + my_uT*my_uT + mz_uT*mz_uT);
    float dip_deg = rad2deg(atan2f(mz_uT, sqrtf(mx_uT*mx_uT + my_uT*my_uT)));

    out->azimuth_deg     = azimuth_deg;
    out->inclination_deg = inc_deg;
    out->dip_deg         = dip_deg;
    out->tot_grav_g      = g;
    out->tot_mag_uT      = tot_mag;
}

float heading_from_mag_level(float mx, float my, float decl_deg)
{
    float hdg = (180.0f/3.14159265f) * atan2f(-my, mx) + decl_deg; // magnetic north if decl=0
    if (hdg < 0)   hdg += 360.0f;
    if (hdg >= 360.0f) hdg -= 360.0f;
    return hdg;
}

float magnetic_dip_from_mag(float mx, float my, float mz)
{
    float dip = (180.0f/3.14159265f) * atan2f(mz, sqrtf(mx*mx + my*my));
    return dip;
}

