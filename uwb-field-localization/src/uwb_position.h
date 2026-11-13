#ifndef UWB_POSITION_H
#define UWB_POSITION_H

#include <stdbool.h>

#define UWB_ANCHOR_COUNT 4

typedef struct {
    double x;
    double y;
} UwbPoint;

typedef struct {
    UwbPoint position;
    double distance;
    bool received;
} UwbAnchorReading;

/* Estimate a tag position from at least three readings.
 * Anchor 0 and anchor 3 are opposite corners of the test field.
 * Returns false for invalid readings or a non-finite estimate.
 */
bool uwb_estimate_position(const UwbAnchorReading readings[UWB_ANCHOR_COUNT],
                           UwbPoint *position);

#endif
