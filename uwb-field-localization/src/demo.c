#include "uwb_position.h"

#include <math.h>
#include <stdio.h>

int main(void)
{
    UwbAnchorReading readings[UWB_ANCHOR_COUNT] = {
        {{0.0, 0.0}, 0.0, true},
        {{50.0, 0.0}, 0.0, true},
        {{0.0, 50.0}, 0.0, true},
        {{50.0, 50.0}, 0.0, true}
    };
    const UwbPoint expected = {12.0, 18.0};
    for (int i = 0; i < UWB_ANCHOR_COUNT; ++i) {
        readings[i].distance = hypot(expected.x - readings[i].position.x,
                                    expected.y - readings[i].position.y);
    }

    UwbPoint estimate;
    if (!uwb_estimate_position(readings, &estimate)) {
        fprintf(stderr, "Position estimate failed.\n");
        return 1;
    }
    printf("Estimated position: (%.2f, %.2f) m\n", estimate.x, estimate.y);
    return hypot(expected.x - estimate.x, expected.y - estimate.y) < 0.01 ? 0 : 1;
}
