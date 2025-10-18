/*
 * dst_app_bridge_corrected.cpp
 *
 *  Created on: Oct 18, 2025
 *      Author: fidia
 */

// Core/Src/dst_app_bridge.cpp
#include <dst_app_bridge.h>
#include <cstdint>
#include "algorithm.hpp"
#include "types.hpp"

static dst::Algorithm g_algo;

extern "C" {

void dst_app_init(void) {
    dst::AlgoConfig cfg{};      // defaults OK to start
    g_algo.init(cfg);
}

void dst_app_step(const DstRawSample* s, DstAlgoOut* out) {
    if (!s || !out) return;

    dst::MultiSample ms{};
    ms.acc_lo  = { s->ax_lo,  s->ay_lo,  s->az_lo };
    ms.acc_med = { s->ax_mid, s->ay_mid, s->az_mid };
    ms.acc_hi  = { s->ax_hi,  s->ay_hi,  s->az_hi };
    ms.mag     = { s->mx,     s->my,     s->mz    };
    ms.temp_c  = s->temp_c;
    ms.t_us    = s->t_us;  // ok if MultiSample uses uint64_t

    const dst::AlgoOutputs a = g_algo.update(ms);

    out->incl_deg  = a.heading.inclination_deg;
    out->azim_deg  = a.heading.azimuth_deg;
    out->dip_deg   = a.heading.mag_dip_deg;
    out->ml_pred   = static_cast<uint8_t>(a.ml_pred);
    out->ml_final  = static_cast<uint8_t>(a.ml_final);
    out->rot_flag  = a.rot;
    out->flow_flag = a.flow;

}

} // extern "C"



