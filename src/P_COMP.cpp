#include "P_COMP.h"

typedef struct {
    float Pset;
    float Pint;
} LUT_PSET_t;

static const LUT_PSET_t lut_pset[] = {
    {4.0f, 4.4f},
    {5.0f, 5.5f},
    {6.0f, 6.6f},
    {7.0f, 7.7f},
    {8.0f, 9.0f},
    {9.0f, 10.0f},
    {10.0f, 11.2f},
    {11.0f, 12.1f},
    {12.0f, 13.2f}
};

#define LUT_PSET_SIZE (sizeof(lut_pset)/sizeof(lut_pset[0]))

float get_internal_ref(float Pset) {
    if (Pset <= lut_pset[0].Pset)
        return lut_pset[0].Pint;

    if (Pset >= lut_pset[LUT_PSET_SIZE - 1].Pset)
        return lut_pset[LUT_PSET_SIZE - 1].Pint;

    for (int i = 0; i < LUT_PSET_SIZE - 1; i++) {

        if (Pset >= lut_pset[i].Pset && Pset <= lut_pset[i+1].Pset) {

            float x1 = lut_pset[i].Pset;
            float x2 = lut_pset[i+1].Pset;
            float y1 = lut_pset[i].Pint;
            float y2 = lut_pset[i+1].Pint;

            return y1 + (y2 - y1) * (Pset - x1) / (x2 - x1);
        }
    }

    return Pset; // fallback
}