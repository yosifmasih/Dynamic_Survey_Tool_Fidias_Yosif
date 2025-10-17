/*
 * accel_selector.c
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#include "accel_selector.h"
#include <math.h>
#include <string.h>

// ----------- TEST THRESHOLDS (in g) -------------
#ifndef T12_UP_G
#define T12_UP_G        3.0f   // switch 312 -> 356MID at or above 3 g
#endif
#ifndef T12_DOWN_G
#define T12_DOWN_G      2.0f   // switch 356MID -> 312 at or below 2 g
#endif

// We are not using the HI path in this test; keep these huge to “never”
#ifndef T23_UP_G
#define T23_UP_G        1.0e9f
#endif
#ifndef T23_DOWN_G
#define T23_DOWN_G      9.0e8f
#endif

#ifndef ACCEL_SWITCH_DWELL
#define ACCEL_SWITCH_DWELL  4  // require 2 consecutive votes (at 100 Hz ~20 ms)
#endif
// -------------------------------------------------

static inline float vmag3(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void accel_selector_init(AccelSelectState* st, AccelSource start)
{
  memset(st, 0, sizeof(*st));
  st->current       = start;    // ACCEL_SRC_312 recommended at boot
  st->pending       = start;
  st->pending_count = 0;
}

// Decide which source we *prefer* based on the magnitude of the *current* source
static AccelSource decide_preferred_twolevel(AccelSource cur, float mag_g)
{
  switch (cur) {
    case ACCEL_SRC_312:
      // Up-switch only when clearly >= 3 g
      return (mag_g >= T12_UP_G) ? ACCEL_SRC_356B : ACCEL_SRC_312;

    case ACCEL_SRC_356B:
      // Down-switch only when clearly <= 2 g
      return (mag_g <= T12_DOWN_G) ? ACCEL_SRC_312 : ACCEL_SRC_356B;

    case ACCEL_SRC_356C:
    default:
      // HI path not used in this test; fold it into 356MID behavior
      return (mag_g <= T12_DOWN_G) ? ACCEL_SRC_312 : ACCEL_SRC_356B;
  }
}

AccelSource accel_selector_update(const float a312[3],
                                  const float a356b[3],
                                  const float a356c[3],   // unused in test; may be NULL
                                  AccelSelectState* st,
                                  float out_g[3])
{
  (void)a356c;

  // Safety: if 356MID is absent, force 312
  if (!a356b) {
    st->current = ACCEL_SRC_312;
    st->pending = ACCEL_SRC_312;
    st->pending_count = 0;
    if (out_g) { out_g[0] = a312 ? a312[0] : 0.f; out_g[1] = a312 ? a312[1] : 0.f; out_g[2] = a312 ? a312[2] : 0.f; }
    return st->current;
  }

  // Measure magnitude using the CURRENT source (stable behavior)
  const float* cur_v = (st->current == ACCEL_SRC_312) ? a312 :
                       (st->current == ACCEL_SRC_356B) ? a356b : a356b; // fold 356C->356B in test
  const float mag_g = cur_v ? vmag3(cur_v) : 0.f;

  // Preferred source by threshold/hysteresis
  const AccelSource preferred = decide_preferred_twolevel(st->current, mag_g);

  // Dwell: need N consecutive "votes" before committing to new source
  if (preferred != st->current) {
    if (st->pending != preferred) {
      st->pending = preferred;
      st->pending_count = 1;
    } else {
      if (st->pending_count < 255) st->pending_count++;
      if (st->pending_count >= ACCEL_SWITCH_DWELL) {
        st->current = preferred;
        st->pending_count = 0;
      }
    }
  } else {
    st->pending = st->current;
    st->pending_count = 0;
  }

  // Output selected vector
  const float* sel_v = (st->current == ACCEL_SRC_356B) ? a356b : a312;
  if (out_g) {
    if (sel_v) { out_g[0] = sel_v[0]; out_g[1] = sel_v[1]; out_g[2] = sel_v[2]; }
    else       { out_g[0] = out_g[1] = out_g[2] = 0.f; }
  }
  return st->current;
}

