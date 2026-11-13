# 디지털 필터 검증

MATLAB/Simulink에서 설계한 필터를 STM32G474에 구현하고, 입력 주파수별 출력 파형을 이론값과 비교했습니다.

- [LPF·HPF·APF 펌웨어](firmware/lpf-hpf-apf-g474/)
- [BPF·BSF 펌웨어](firmware/bpf-bsf-g474/)
- [Filters.h](Filters.h) · 샘플 단위 C 필터 계산
- [lab3.m](lab3.m) · MATLAB Control System Toolbox를 사용한 10 kHz 이산화 및 계수 산출

실습 보고서에서 100 Hz 저역통과 출력은 이론값 2.15 V, 측정값 2.24 V였습니다. 같은 주파수의 고역통과 출력은 이론값 2.21 V, 측정값 2.24 V였습니다.

---

## English

## Digital Filter Verification

Compared MATLAB/Simulink models and C implementations of low-pass, high-pass, all-pass, band-pass, and band-stop filters. Checked STM32G474 outputs at selected input frequencies against expected amplitude and phase behavior.

- [lab3.m](lab3.m) calculates discrete coefficients at a 10 kHz sample rate and uses MATLAB with Control System Toolbox.
- [Filters.h](Filters.h) contains the original C routines used by the lab models. Include it in one translation unit to exercise the sample-by-sample functions.

The lab report records 2.24 V measured versus 2.15 V expected for the low-pass filter at its 100 Hz cutoff. The high-pass check at the same cutoff recorded 2.24 V measured versus 2.21 V expected.

The STM32 projects are [LPF/HPF/APF](firmware/lpf-hpf-apf-g474/) and [BPF/BSF](firmware/bpf-bsf-g474/).
