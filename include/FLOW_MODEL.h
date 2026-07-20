#ifndef FLOW_MODEL_H
#define FLOW_MODEL_H

#include <math.h>

#define FLOW_K        132.4799f
#define FLOW_B        -6.5691f
 
#define FLOW_DP_MAX    1.260f
#define FLOW_MAX       200.0f

inline float flowFromDP(float dp) {
  if (dp < 0.0f) dp = 0.0f;
  //if (dp > FLOW_DP_MAX) dp = FLOW_DP_MAX;

  //float q = (95.9039f * powf(dp, 3)) - (250.419 * powf(dp, 2)) + (269.603 * dp) + 7.18236;
  float q = FLOW_K * sqrtf(dp) + FLOW_B;

  if (q < 0.0f) return 0.0f;
  if (q > FLOW_MAX) return FLOW_MAX;

  return q;
}

#endif