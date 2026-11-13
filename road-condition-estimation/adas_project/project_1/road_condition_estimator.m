%% =====================================================================
%  도로 상태 추정 시스템 (Road Condition Estimator)
%  - 차량 데이터 기반 실시간 노면 상태 분류
%  - 입력: 4륜 휠속도, IMU(AccelZ, AngRate), 경사각(AnglePitch, VelDown)
%  - 출력: 도로 상태 지수 → 좋음 / 양호 / 나쁨
%  
%  플로우:
%   1) 타이어 슬립    → 슬립 지수
%   2) IMU Z축 진동   → BPF + RMS
%   3) 도로 경사도    → 경사 가중치
%   4) μ 추정         → 동적 하중이동 고려
%   5) 도로상태 지수  → 임계값 기반 등급화
% =====================================================================

clear; clc; close all;

%% ------------------------------------------------------------------
%  1. 데이터 로드 및 기본 정보
%% ------------------------------------------------------------------
fprintf('[1/6] 데이터 로딩 중...\n');
load('testset1.mat');

% 길이가 다른 변수들 → 짧은 쪽 기준으로 통일
N_imu = length(AccelZ);    % 243766
N_can = length(WHL_SPD_FL);% 243771
N = min(N_imu, N_can);     % 243766 사용

% 시간축
t = Time(1:N);
fs = 100;                  % 샘플링 주파수 100 Hz
dt = 1/fs;

fprintf('  - 총 샘플: %d, 길이: %.1f분 (%.1f초)\n', N, t(end)/60, t(end));

%% ------------------------------------------------------------------
%  2. 원본 데이터 추출 (필요한 변수만)
%% ------------------------------------------------------------------
fprintf('[2/6] 변수 추출 및 사전처리...\n');

% --- 타이어 슬립 라인 ---
whl_FL = WHL_SPD_FL(1:N);     % 4륜 휠 속도 [km/h]
whl_FR = WHL_SPD_FR(1:N);
whl_RL = WHL_SPD_RL(1:N);
whl_RR = WHL_SPD_RR(1:N);
vs     = double(VS(1:N));     % 차량 속도 [km/h]
yaw    = YAW_RATE(1:N);       % YAW rate [deg/s]

% --- IMU Z축 진동 라인 ---
accZ   = AccelZ(1:N);         % 수직 가속도 [m/s²]  (중력 ~ -9.8 포함)
angRY  = AngRateY(1:N);       % Pitch rate [deg/s]

% --- 도로 경사도 라인 ---
pitch    = AnglePitch(1:N);   % 피치각 [deg]  (이미 계산됨)
velDown  = VelDown(1:N);      % 수직 속도 [m/s]
velFwd   = VelForward(1:N);   % 전진 속도 [m/s]

% --- 시각화용 ---
posLat = PosLat(1:N);
posLon = PosLon(1:N);
speed2D = Speed2D(1:N);

%% ------------------------------------------------------------------
%  3. 다운샘플링 (시각화 효율을 위해 10Hz로)
%% ------------------------------------------------------------------
DS = 10;                       % 10개 중 1개 → 10Hz
idx_ds = 1:DS:N;
N_ds = length(idx_ds);
t_ds = t(idx_ds);

fprintf('  - 다운샘플링: 100Hz → 10Hz, 샘플 수: %d\n', N_ds);

%% ------------------------------------------------------------------
%  4. 사전처리 단계: 유효 주행 구간 추출
%% ------------------------------------------------------------------
% 유효 GPS + 차량 움직임 조건
valid_mask = (posLat > 0) & (posLon > 0) & (vs > 5);

% 유효 구간이 충분히 긴 영역만 선택 (연속 5초 이상)
fprintf('  - 유효 주행 구간 추출 중...\n');

%% ------------------------------------------------------------------
%  5. 핵심 계산: 플로우차트의 각 블록 구현
%% ------------------------------------------------------------------
fprintf('[3/6] 슬립 지수 계산...\n');

%% -- (A) 타이어 슬립 라인 --------------------------------------------
% 코너링 YAW 보정: 선회 시 좌우 휠 속도 차이는 자연스러우므로 보정 필요
% 트레드(좌우 휠 거리, 약 1.6m) 기반 이론 차이 계산
TREAD = 1.6;                                  % [m] 좌우 휠 간격 (가정)
WHEELBASE = 2.7;                              % [m] 휠베이스 (가정)
yaw_rad = yaw * pi/180;                       % deg/s → rad/s

% 선회 보정: 좌측 휠은 v - (TREAD/2)*yaw_rate, 우측은 v + (TREAD/2)*yaw_rate (km/h 기준)
v_corr = (TREAD/2) * yaw_rad * 3.6;           % m/s → km/h

% 보정된 휠속도 (선회 영향 제거)
whl_FL_c = whl_FL + v_corr;
whl_FR_c = whl_FR - v_corr;
whl_RL_c = whl_RL + v_corr;
whl_RR_c = whl_RR - v_corr;

% 4륜 평균 (보정 후)
whl_avg = (whl_FL_c + whl_FR_c + whl_RL_c + whl_RR_c) / 4;

% 슬립율: (휠속도 - 차량속도) / 차량속도
% vs가 0인 경우 회피
vs_safe = max(vs, 0.1);
slip_ratio = (whl_avg - vs) ./ vs_safe;

% 절댓값으로 변환 (음/양 모두 슬립으로 간주)
slip_ratio = abs(slip_ratio);

% 4륜 분산: 4륜이 서로 다르게 움직이면 슬립 발생 의심
whl_arr = [whl_FL_c, whl_FR_c, whl_RL_c, whl_RR_c];
whl_var = var(whl_arr, 0, 2) ./ (vs_safe.^2 + 0.1);  % 정규화

% 슬립 지수 (0~1로 정규화) - 단순 가중합
slip_index_raw = 0.7 * slip_ratio + 0.3 * sqrt(whl_var);

% 1초 이동평균 (Kalman 대용으로 단순 평활화)
WIN_1s = fs;  % 100 샘플 = 1초
slip_index = movmean(slip_index_raw, WIN_1s);

% 0~1 범위로 클리핑
slip_index = min(max(slip_index, 0), 1);

%% -- (B) IMU Z축 진동 라인 -------------------------------------------
fprintf('[4/6] IMU 진동 RMS 계산...\n');

% Bandpass filter: 0.5 ~ 10 Hz 통과 (도로 노면 진동 대역)
% 중력 성분(DC)과 고주파 노이즈 제거
[b, a] = butter(4, [0.5 10]/(fs/2), 'bandpass');
accZ_bpf = filtfilt(b, a, accZ);

% 1초 이동 윈도우 RMS
accZ_sq = accZ_bpf.^2;
rms_z = sqrt(movmean(accZ_sq, WIN_1s));

% 속도 의존성 정규화 (빠를수록 진동 커짐 → 보정)
% v_norm = max(vs, 10) / 50;  % 50km/h 기준 정규화
v_norm = sqrt(max(vs, 5) / 50);
rms_z_norm = rms_z ./ v_norm;

%% -- (C) 도로 경사도 라인 --------------------------------------------
fprintf('[5/6] 경사도 및 가중치 계산...\n');

% 경사도 계산: AnglePitch가 이미 있으나, VelDown/VelForward로 검증
% theta = atan2(VelDown, VelForward) 가 더 동적인 경사 반영
% 단, VelForward가 0에 가까우면 부정확
velFwd_safe = max(abs(velFwd), 0.5);
slope_dyn = atan2(-velDown, velFwd_safe) * 180/pi;  % [deg]

% AnglePitch와 동적 경사도의 가중평균 (안정성 위해)
slope = 0.7 * pitch + 0.3 * slope_dyn;

% 경사 가중치: 경사가 클수록 슬립이 커지므로 슬립 영향을 감소시킴
% w_grade = f(|theta|) → 경사 절댓값이 클수록 가중치 작아짐 (슬립 보정)
w_grade = 1 ./ (1 + (abs(slope)/10).^2);  % 10° 기준 1/2로 감쇠
% 결과: 평지 → 1, 10도 경사 → 0.5, 20도 경사 → 0.2

%% -- (D) 마찰계수 μ 추정 ---------------------------------------------
% 동적 하중이동 고려한 μ 추정
% F_x = μ * F_z * f(slip)
% 종방향 가속도 → 하중이동 계산
% μ ≈ a_x / (g * cos(pitch)) 근사 (단순 모델)
g = 9.81;
% AccelX 사용 (종방향 가속도)
accX = AccelX(1:N);
mu_est_raw = abs(accX) ./ (g * cosd(slope) + 0.1);
mu_est_raw = min(mu_est_raw, 1.2);  % 최대 1.2 클리핑

% 평활화
mu_est = movmean(mu_est_raw, WIN_1s);

% μ 자체는 "그립이 좋을수록 큰 값"이므로
% 도로 상태 지수에서는 (1 - μ_norm) 형태로 활용
% 또는 슬립이 일어났을 때만 의미가 있으므로 슬립과 결합
mu_score = mu_est;  % 그대로 사용

%% -- (E) 도로 상태 지수 산출 -----------------------------------------
% road_index = 슬립 + RMS + μ × w_grade
% 각 항목을 0~1로 정규화 후 가중합

% 유효 주행 구간 마스크 (정규화 기준점 추출용)
valid_mask = (posLat > 0) & (posLon > 0) & (vs > 5);

% [개선] 백분위수 기반 정규화 - 데이터 분포에 자동 적응
% 유효 구간의 95% 분위수를 1.0의 기준으로 사용
slip_p95 = prctile(slip_index(valid_mask), 95);
rms_p95  = prctile(rms_z_norm(valid_mask), 95);

slip_norm = min(slip_index ./ max(slip_p95, 0.01), 1);
rms_norm  = min(rms_z_norm ./ max(rms_p95, 0.1), 1);

% μ 역수: 그립이 약할수록(μ가 작을수록) 도로 의심
% 이 데이터셋의 μ는 정상적으로 작으므로 0.3을 기준으로 반전
mu_inv = max(0, 1 - mu_est / 0.3);

% 도로 상태 지수 (0~1, 클수록 나쁨)
% 가중치: 진동 55%, 슬립 30%, μ역수 15%
road_index = 0.55 * rms_norm + 0.30 * slip_norm + 0.15 * mu_inv;

% 경사 보정 적용
% 경사가 크면 슬립/μ가 자연발생적이므로 진동 RMS만 신뢰
road_index_corrected = w_grade .* road_index + (1 - w_grade) .* rms_norm;

% 최종 평활화 (3초)
road_index_final = movmean(road_index_corrected, 3*WIN_1s);
road_index_final = min(max(road_index_final, 0), 1);

%% -- (F) 적응형 임계값 기반 등급 분류 --------------------------------
fprintf('[6/6] 도로 상태 등급 분류...\n');

% 유효 구간 분포 기반 적응형 임계값
% TH1 = 60% 분위수 (하위 60%는 좋음)
% TH2 = 90% 분위수 (상위 10%는 나쁨)
TH1 = prctile(road_index_final(valid_mask), 60);
TH2 = prctile(road_index_final(valid_mask), 90);

fprintf('  - 적응형 임계값: TH1 = %.4f, TH2 = %.4f\n', TH1, TH2);

% 등급: 1=좋음, 2=양호, 3=나쁨
road_grade = ones(N, 1);
road_grade(road_index_final >= TH1 & road_index_final < TH2) = 2;
road_grade(road_index_final >= TH2) = 3;

% 통계 (유효 주행 구간 기준)
g_v = road_grade(valid_mask);
fprintf('\n=== 도로 상태 분포 (유효 주행 구간) ===\n');
fprintf('  좋음 (Good): %.1f%%\n', 100*sum(g_v==1)/length(g_v));
fprintf('  양호 (Fair): %.1f%%\n', 100*sum(g_v==2)/length(g_v));
fprintf('  나쁨 (Bad) : %.1f%%\n', 100*sum(g_v==3)/length(g_v));

%% ------------------------------------------------------------------
%  6. 결과 저장 (실시간 시각화용)
%% ------------------------------------------------------------------
% 다운샘플링된 데이터로 시각화 (효율성)
results.t           = t(idx_ds);
results.vs          = vs(idx_ds);
results.whl_FL      = whl_FL(idx_ds);
results.whl_FR      = whl_FR(idx_ds);
results.whl_RL      = whl_RL(idx_ds);
results.whl_RR      = whl_RR(idx_ds);
results.accZ        = accZ(idx_ds);
results.accZ_bpf    = accZ_bpf(idx_ds);
results.pitch       = pitch(idx_ds);
results.slope       = slope(idx_ds);
results.slip_index  = slip_norm(idx_ds);   % 정규화된 값으로 변경
results.rms_z       = rms_norm(idx_ds);    % 정규화된 값으로 변경
results.mu_est      = mu_est(idx_ds);
results.w_grade     = w_grade(idx_ds);
results.road_index  = road_index_final(idx_ds);
results.road_grade  = road_grade(idx_ds);
results.posLat      = posLat(idx_ds);
results.posLon      = posLon(idx_ds);
results.valid       = (posLat(idx_ds) > 0) & (posLon(idx_ds) > 0) & (vs(idx_ds) > 5);
results.TH1         = TH1;   % 임계값 저장
results.TH2         = TH2;

save('road_results.mat', 'results', '-v7.3');
fprintf('\n결과 저장 완료: road_results.mat\n');
fprintf('이제 visualize_realtime.m 을 실행하세요.\n');
