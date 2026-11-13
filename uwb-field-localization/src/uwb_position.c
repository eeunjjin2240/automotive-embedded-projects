#include "uwb_position.h"

#include <math.h>
#include <stddef.h>

bool uwb_estimate_position(const UwbAnchorReading readings[UWB_ANCHOR_COUNT],
                           UwbPoint *position)
{
    const double epsilon = 1e-6;
    const double max_step = 1e-5;
    const double damping = 3.0;
    const int max_iterations = 1000;
    size_t valid_count = 0;

    if (readings == NULL || position == NULL) {
        return false;
    }

    for (size_t i = 0; i < UWB_ANCHOR_COUNT; ++i) {
        const UwbAnchorReading *anchor = &readings[i];
        if (!anchor->received) {
            continue;
        }
        if (!isfinite(anchor->distance) || anchor->distance <= 0.0 ||
            anchor->distance > 1000.0 || !isfinite(anchor->position.x) ||
            !isfinite(anchor->position.y)) {
            return false;
        }
        ++valid_count;
    }
    if (valid_count < 3) {
        return false;
    }

    UwbPoint estimate = {
        (readings[0].position.x + readings[3].position.x) / 2.0,
        (readings[0].position.y + readings[3].position.y) / 2.0
    };

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        double cost = 0.0;
        double gradient_x = 0.0;
        double gradient_y = 0.0;

        for (size_t i = 0; i < UWB_ANCHOR_COUNT; ++i) {
            const UwbAnchorReading *anchor = &readings[i];
            if (!anchor->received) {
                continue;
            }

            const double dx = estimate.x - anchor->position.x;
            const double dy = estimate.y - anchor->position.y;
            const double residual = dx * dx + dy * dy -
                                    anchor->distance * anchor->distance;
            cost += residual * residual;
            gradient_x += 4.0 * residual * dx;
            gradient_y += 4.0 * residual * dy;
        }

        if (gradient_x * gradient_x + gradient_y * gradient_y < epsilon * epsilon) {
            break;
        }

        const double error = sqrt(cost);
        const double step = fmax(epsilon, max_step * error / (error + damping));
        estimate.x -= step * gradient_x;
        estimate.y -= step * gradient_y;
    }

    if (!isfinite(estimate.x) || !isfinite(estimate.y)) {
        return false;
    }
    *position = estimate;
    return true;
}
