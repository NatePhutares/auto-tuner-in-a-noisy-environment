/*
 * at_ring.h — ring buffers addressed by absolute sample index.
 *
 * The Python prototype keeps whole-signal arrays (audio[n], residual[n], ...). On the MCU the same
 * arrays are rings of AT_RING samples: element n lives at [n & (AT_RING-1)]. Negative n reads as 0,
 * exactly like the zero padding at the start of the Python arrays.
 */
#ifndef AT_RING_H
#define AT_RING_H

#include <stdint.h>
#include "at_config.h"

#define AT_RING_MASK (AT_RING - 1)

static inline float at_ring_getf(const float *ring, int32_t n)
{
    return n < 0 ? 0.0f : ring[n & AT_RING_MASK];
}

static inline uint8_t at_ring_getb(const uint8_t *ring, int32_t n)
{
    return n < 0 ? 0u : ring[n & AT_RING_MASK];
}

#endif /* AT_RING_H */
