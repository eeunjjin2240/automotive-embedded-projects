# 노면 상태 추정

팀 프로젝트에서 MATLAB으로 휠 속도와 관성 센서 기록을 분석했습니다. 슬립과 수직 가속도 기반 진동 지표를 산출하고, 경사도와 마찰 추정치를 함께 고려했습니다. 기록 데이터 재생 화면에서 지표와 차량 위치를 확인할 수 있었습니다.

팀 소스와 제공 데이터의 공개 범위를 확인 중입니다. 실제 노면 상태에 대한 정확도를 평가하려면 독립 기준값과 검증 데이터가 필요합니다.

---

## English

## Road Condition Estimation

In a separate team project, we explored estimating road conditions from wheel-speed and inertial signals. The MATLAB workflow derived slip and vibration indicators, considered grade and estimated friction, and visualized the results during recorded-data replay.

The replay illustrated the behavior of the estimator across the available runs. A field-accuracy claim would require independent road-condition labels and validation data.

The replay used wheel speeds and vehicle speed to derive slip, and filtered vertical acceleration to estimate vibration. The dashboard plotted these indicators alongside the road-condition index and a vehicle-position view.

Release of the team source and supplied data is still to be confirmed.
