# Digital Filter Verification

Compared MATLAB/Simulink models and C implementations of low-pass, high-pass, all-pass, band-pass, and band-stop filters. Checked STM32G474 outputs at selected input frequencies against expected amplitude and phase behavior.

- [lab3.m](lab3.m) calculates discrete coefficients at a 10 kHz sample rate and uses MATLAB with Control System Toolbox.
- [Filters.h](Filters.h) contains the original C routines used by the lab models. Include it in one translation unit to exercise the sample-by-sample functions.

The lab report records 2.24 V measured versus 2.15 V expected for the low-pass filter at its 100 Hz cutoff. The high-pass check at the same cutoff recorded 2.24 V measured versus 2.21 V expected.
