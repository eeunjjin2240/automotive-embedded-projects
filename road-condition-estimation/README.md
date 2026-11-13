# Road Condition Estimation

In a separate team project, we explored estimating road conditions from wheel-speed and inertial signals. The MATLAB workflow derived slip and vibration indicators, considered grade and estimated friction, and visualized the results during recorded-data replay.

The replay illustrated the behavior of the estimator across the available runs. A field-accuracy claim would require independent road-condition labels and validation data.

The replay used wheel speeds and vehicle speed to derive slip, and filtered vertical acceleration to estimate vibration. The dashboard plotted these indicators alongside the road-condition index and a vehicle-position view.
