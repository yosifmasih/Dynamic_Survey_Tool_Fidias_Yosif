#ifndef DST_APP_BRIDGE_H
#define DST_APP_BRIDGE_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float ax_lo, ay_lo, az_lo;
  float ax_mid, ay_mid, az_mid;
  float ax_hi, ay_hi, az_hi;
  float mx, my, mz;
  float temp_c;
  uint32_t t_us;   // microseconds
} DstRawSample;

typedef struct {
  float   incl_deg;
  float   azim_deg;
  float   dip_deg;
  uint8_t ml_pred;
  uint8_t ml_final;
  uint8_t rot_flag;
  uint8_t flow_flag;
} DstAlgoOut;

void dst_app_init(void);
void dst_app_step(const DstRawSample* s, DstAlgoOut* out);

#ifdef __cplusplus
}
#endif
#endif
