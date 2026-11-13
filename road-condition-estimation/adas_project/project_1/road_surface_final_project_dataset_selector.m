%% ========================================================================
%  Road Surface / Grip State Estimation Final Project
%  실차 CAN/IMU 데이터 + Bicycle Model 기반 노면 접지/거칠기 상태 판단
%
%  Input  : testset1.mat
%  Output : road_surface_final_project_result.mat
%
%  Main functions:
%   1) Vehicle / IMU / GPS signal preprocessing
%   2) Road bank angle estimation
%   3) Grade / bank acceleration correction
%   4) Wheel slip ratio approximation + Kalman slip index fusion
%   5) Tire slip angle estimation using bicycle model
%   6) Tire force / vertical load estimation
%   7) Required friction coefficient estimation + revised confidence based mu_road
%   8) AccelZ 3D correction + BPF RMS roughness / lateral consistency index
%   9) Road surface / grip state classification
%  10) Map-based dashboard visualization
%
%  Author note:
%  본 코드는 실제 노면 최대 마찰계수 mu_max를 직접 계산하는 것이 아니라,
%  차량 거동을 설명하기 위한 요구 마찰계수 mu_required와 보조 지표를 이용해
%  노면 접지 상태를 간접 판단하는 프로젝트 코드입니다.
%% ========================================================================

clear all
close all
clc

%% ========================================================================
%  0. User Options
%% ========================================================================

% Dashboard에서 선택 가능한 데이터셋 목록
DATASET_LIST = {'testset1.mat', 'testset3.mat'};

% 마지막으로 선택한 데이터셋을 임시 파일에 저장해두고, 재실행 시 그대로 사용
DATASET_CHOICE_FILE = fullfile(tempdir, 'road_surface_dataset_choice.mat');
DATA_FILE = DATASET_LIST{1};
if isfile(DATASET_CHOICE_FILE)
    try
        choiceData = load(DATASET_CHOICE_FILE, 'DATA_FILE_CHOICE');
        if isfield(choiceData, 'DATA_FILE_CHOICE') && any(strcmp(choiceData.DATA_FILE_CHOICE, DATASET_LIST))
            DATA_FILE = choiceData.DATA_FILE_CHOICE;
        end
    catch
        DATA_FILE = DATASET_LIST{1};
    end
end

[~, DATA_BASENAME, ~] = fileparts(DATA_FILE);
SAVE_FILE = sprintf('road_surface_final_project_result_%s.mat', DATA_BASENAME);
SCRIPT_PATH = mfilename('fullpath');
if isempty(SCRIPT_PATH)
    SCRIPT_PATH = [mfilename '.m'];
end

RUN_DASHBOARD = true;       % 지도 대시보드 실행 여부
ANIMATION_STEP = 100;       % 애니메이션 샘플 간격. 작을수록 느리고 부드러움
ANIMATION_PAUSE = 0.03;     % 애니메이션 pause 시간

USE_MAP_BASE = true;        % geobasemap 사용. 인터넷/툴박스 문제 시 false로 변경

%% ========================================================================
%  1. Load Data
%% ========================================================================

if ~isfile(DATA_FILE)
    error('데이터 파일을 찾을 수 없습니다: %s', DATA_FILE);
end

load(DATA_FILE)

fprintf('\n============================================================\n');
fprintf(' Road Surface / Grip State Estimation Final Project\n');
fprintf('============================================================\n');
fprintf('Loaded data file: %s\n', DATA_FILE);

%% ========================================================================
%  2. Vehicle Parameters
%% ========================================================================
% 사용자가 정한 차량 파라미터

g = 9.81;

m = 1600;              % 차량 질량 [kg]
L = 2.8;               % 축거 [m]
Wf_ratio = 0.60;       % 전륜 정적하중 비율

lr = Wf_ratio * L;     % CG에서 후륜축까지 거리 [m]
lf = L - lr;           % CG에서 전륜축까지 거리 [m]

h = 0.55;              % CG height [m]
Iz = m * (0.4 * L)^2;  % yaw inertia approximation [kg*m^2]
steering_ratio = 15;   % steering wheel angle / road wheel angle
track_w = 1.635;       % track width [m] - yaw corrected wheel slip용

CdA = 0.65;            % Drag coefficient * frontal area [m^2]
Crr = 0.012;           % Rolling resistance coefficient
rho = 1.225;           % Air density [kg/m^3]
R_tire = 0.32;         % Tire effective radius [m]

lambda_x = 0.5;        % Front/rear longitudinal force distribution assumption

%% ========================================================================
%  3. Signal Selection
%% ========================================================================

% -----------------------------
% Time
% -----------------------------
t = Time(:);

% -----------------------------
% Longitudinal vehicle speed Vx [m/s]
% Priority: VelForward -> CF_Clu_VehicleSpeed -> Speed2D
% -----------------------------
if exist('VelForward', 'var')
    Vx = VelForward(:);
elseif exist('CF_Clu_VehicleSpeed', 'var')
    Vx = CF_Clu_VehicleSpeed(:) / 3.6;
elseif exist('Speed2D', 'var')
    Vx = Speed2D(:);
else
    error('차량 속도 신호를 찾을 수 없습니다. VelForward, CF_Clu_VehicleSpeed, Speed2D 중 하나가 필요합니다.');
end

% VelForward가 0이 너무 많으면 CAN speed 사용
if exist('CF_Clu_VehicleSpeed', 'var') && mean(Vx == 0) > 0.3
    Vx = CF_Clu_VehicleSpeed(:) / 3.6;
end

% -----------------------------
% Lateral speed Vy [m/s]
% -----------------------------
if exist('VelLateral', 'var')
    Vy = VelLateral(:);
else
    Vy = zeros(size(Vx));
end

% -----------------------------
% Acceleration signals [m/s^2]
% Priority: AccelForward/AccelLateral -> LONG_ACCEL/LAT_ACCEL
% -----------------------------
if exist('AccelForward', 'var')
    ax = AccelForward(:);
elseif exist('LONG_ACCEL', 'var')
    ax = LONG_ACCEL(:);
else
    error('종방향 가속도 신호를 찾을 수 없습니다.');
end

if exist('AccelLateral', 'var')
    ay = AccelLateral(:);
elseif exist('LAT_ACCEL', 'var')
    ay = LAT_ACCEL(:);
else
    error('횡방향 가속도 신호를 찾을 수 없습니다.');
end

if exist('AccelDown', 'var')
    az = AccelDown(:);
elseif exist('AccelZ', 'var')
    az = AccelZ(:);
else
    az = zeros(size(ax));
end

% -----------------------------
% Yaw rate [rad/s]
% YAW_RATE is assumed deg/s
% -----------------------------
if exist('YAW_RATE', 'var')
    r = deg2rad(YAW_RATE(:));
elseif exist('CAN_TX_YAW_RATE_CALC', 'var')
    r = deg2rad(CAN_TX_YAW_RATE_CALC(:));
else
    error('Yaw rate 신호를 찾을 수 없습니다.');
end

% -----------------------------
% Steering angle
% SAS_Angle is assumed steering wheel angle [deg]
% -----------------------------
if exist('SAS_Angle', 'var')
    sas = SAS_Angle(:);
else
    sas = zeros(size(Vx));
end

delta = deg2rad(sas) / steering_ratio;  % road wheel angle [rad]

% -----------------------------
% Optional INS slip / control intervention signals
% - AngleSlip : INS/IMU lateral slip angle [deg] assumed
% - TCS_CTL / ABS_ACT / ESP_CTL : control intervention flags
% -----------------------------
if exist('AngleSlip', 'var')
    angle_slip_deg = AngleSlip(:);
else
    angle_slip_deg = zeros(size(Vx));
end

if exist('TCS_CTL', 'var')
    TCS_CTL_sig = double(TCS_CTL(:));
else
    TCS_CTL_sig = zeros(size(Vx));
end

if exist('ABS_ACT', 'var')
    ABS_ACT_sig = double(ABS_ACT(:));
else
    ABS_ACT_sig = zeros(size(Vx));
end

if exist('ESP_CTL', 'var')
    ESP_CTL_sig = double(ESP_CTL(:));
else
    ESP_CTL_sig = zeros(size(Vx));
end

% -----------------------------
% Road slope / grade angle selection
% 1) SLOPE_TCU: grade percent [%] assumed
% 2) Pitch_Angle: rad or deg depending on range
% If SLOPE_TCU is nearly fixed or too biased, Pitch_Angle is used instead.
% -----------------------------
theta_tcu = [];
if exist('SLOPE_TCU', 'var')
    theta_tcu = atan(double(SLOPE_TCU(:)) / 100);  % [rad]
end

theta_pitch = [];
if exist('Pitch_Angle', 'var')
    pitch_raw = double(Pitch_Angle(:));
    if max(abs(pitch_raw(isfinite(pitch_raw)))) < 1.5
        theta_pitch = pitch_raw;                  % likely rad
    else
        theta_pitch = deg2rad(pitch_raw);         % likely deg
    end
    % remove installation offset using median
    theta_pitch = theta_pitch - median(theta_pitch(isfinite(theta_pitch)), 'omitnan');
end

if ~isempty(theta_tcu)
    grade_tcu_deg = rad2deg(theta_tcu);
    grade_tcu_std = std(grade_tcu_deg(isfinite(grade_tcu_deg)), 'omitnan');
    grade_tcu_med = median(grade_tcu_deg(isfinite(grade_tcu_deg)), 'omitnan');

    % Use TCU only when it has enough variation and not an unrealistic constant bias.
    if grade_tcu_std > 0.05 && abs(grade_tcu_med) < 8
        theta = theta_tcu;
        grade_source = 'SLOPE_TCU';
    elseif ~isempty(theta_pitch)
        theta = theta_pitch;
        grade_source = 'Pitch_Angle';
    else
        theta = theta_tcu - median(theta_tcu(isfinite(theta_tcu)), 'omitnan');
        grade_source = 'SLOPE_TCU_offset_removed';
    end
elseif ~isempty(theta_pitch)
    theta = theta_pitch;
    grade_source = 'Pitch_Angle';
else
    theta = zeros(size(Vx));
    grade_source = 'zero';
end

% -----------------------------
% Roll / bank related signal
% AngleRoll is assumed deg
% -----------------------------
if exist('AngleRoll', 'var')
    roll_meas_deg = AngleRoll(:);
else
    roll_meas_deg = zeros(size(Vx));
end

% -----------------------------
% Wheel speed [km/h]
% -----------------------------
if exist('WHL_SPD_FL', 'var') && exist('WHL_SPD_FR', 'var') && ...
   exist('WHL_SPD_RL', 'var') && exist('WHL_SPD_RR', 'var')
    whl_FL = WHL_SPD_FL(:);
    whl_FR = WHL_SPD_FR(:);
    whl_RL = WHL_SPD_RL(:);
    whl_RR = WHL_SPD_RR(:);
else
    error('4륜 wheel speed 신호가 필요합니다: WHL_SPD_FL/FR/RL/RR');
end

% -----------------------------
% GPS position
% -----------------------------
if exist('PosLat', 'var') && exist('PosLon', 'var')
    pos_lat = PosLat(:);
    pos_lon = PosLon(:);
else
    pos_lat = zeros(size(t));
    pos_lon = zeros(size(t));
end

%% ========================================================================
%  4. Length Matching
%% ========================================================================

N = min([length(t), length(Vx), length(Vy), length(ax), length(ay), ...
         length(az), length(r), length(delta), length(angle_slip_deg), ...
         length(TCS_CTL_sig), length(ABS_ACT_sig), length(ESP_CTL_sig), ...
         length(theta), length(roll_meas_deg), length(whl_FL), length(whl_FR), ...
         length(whl_RL), length(whl_RR)]);

t = t(1:N);
Vx = Vx(1:N);
Vy = Vy(1:N);
ax = ax(1:N);
ay = ay(1:N);
az = az(1:N);
r = r(1:N);
delta = delta(1:N);
sas = sas(1:N);
angle_slip_deg = angle_slip_deg(1:N);
TCS_CTL_sig = TCS_CTL_sig(1:N);
ABS_ACT_sig = ABS_ACT_sig(1:N);
ESP_CTL_sig = ESP_CTL_sig(1:N);
theta = theta(1:N);
roll_meas_deg = roll_meas_deg(1:N);
whl_FL = whl_FL(1:N);
whl_FR = whl_FR(1:N);
whl_RL = whl_RL(1:N);
whl_RR = whl_RR(1:N);

if length(pos_lat) >= N && length(pos_lon) >= N
    pos_lat = pos_lat(1:N);
    pos_lon = pos_lon(1:N);
else
    pos_lat = zeros(N,1);
    pos_lon = zeros(N,1);
end

%% ========================================================================
%  5. Basic Preprocessing
%% ========================================================================

if length(t) < 2
    error('Time 데이터 길이가 너무 짧습니다.');
end

dt = mean(diff(t), 'omitnan');
if isnan(dt) || dt <= 0
    dt = 0.01;
end
Fs = 1 / dt;

win_smooth = 21;

Vx_s = movmedian(Vx, win_smooth, 'omitnan');
Vy_s = movmedian(Vy, win_smooth, 'omitnan');
ax_s = movmedian(ax, win_smooth, 'omitnan');
ay_s = movmedian(ay, win_smooth, 'omitnan');
az_s = movmedian(az, win_smooth, 'omitnan');
r_s = movmedian(r, win_smooth, 'omitnan');
delta_s = movmedian(delta, win_smooth, 'omitnan');
sas_s = movmedian(sas, win_smooth, 'omitnan');
angle_slip_s_deg = movmedian(angle_slip_deg, win_smooth, 'omitnan');
TCS_CTL_s = TCS_CTL_sig > 0;
ABS_ACT_s = ABS_ACT_sig > 0;
ESP_CTL_s = ESP_CTL_sig > 0;
theta_s = movmedian(theta, win_smooth, 'omitnan');
roll_s_deg = movmedian(roll_meas_deg, win_smooth, 'omitnan');

valid_speed = Vx_s > 5;   % 5 m/s 이상만 주요 계산 신뢰

%% ========================================================================
%  6. Road Bank Angle Estimation
%% ========================================================================
% AngleRoll = road bank + vehicle body roll + sensor offset
% road bank ≈ AngleRoll - offset - vehicle roll

straight_idx = ...
    Vx_s > 5 & ...
    abs(r_s) < deg2rad(1.0) & ...
    abs(ay_s) < 0.2 & ...
    abs(sas_s) < 5 & ...
    ~isnan(roll_s_deg);

if sum(straight_idx) < 100
    straight_idx = ...
        Vx_s > 3 & ...
        abs(r_s) < deg2rad(2.0) & ...
        abs(ay_s) < 0.4 & ...
        abs(sas_s) < 10 & ...
        ~isnan(roll_s_deg);
end

if sum(straight_idx) < 10
    roll_offset_deg = mean(roll_s_deg, 'omitnan');
else
    roll_offset_deg = mean(roll_s_deg(straight_idx), 'omitnan');
end

if isnan(roll_offset_deg)
    roll_offset_deg = 0;
end

roll_zeroed_deg = roll_s_deg - roll_offset_deg;

K_roll_deg_per_g = 3.0;  % 승용차 roll gradient 가정 [deg/g]
vehicle_roll_deg = K_roll_deg_per_g * (ay_s / g);

bank_roll_deg = roll_zeroed_deg - vehicle_roll_deg;

bank_arg = (ay_s - Vx_s .* r_s) / g;
bank_arg(bank_arg > 1) = 1;
bank_arg(bank_arg < -1) = -1;
bank_dyn_deg = rad2deg(asin(bank_arg));

bank_fused_deg = 0.5 * bank_roll_deg + 0.5 * bank_dyn_deg;
bank_diff = abs(bank_roll_deg - bank_dyn_deg);

valid_bank = ...
    Vx_s > 8 & ...
    abs(sas_s) < 45 & ...
    abs(bank_diff) < 5 & ...
    ~isnan(bank_fused_deg);

bank_final_deg = bank_fused_deg;
bank_final_deg(~valid_bank) = NaN;
bank_final_deg = movmedian(bank_final_deg, 101, 'omitnan');

% invalid bank 구간은 bank 보정하지 않음
phi = deg2rad(bank_final_deg);
phi(isnan(phi)) = 0;

%% ========================================================================
%  7. Grade / Bank Acceleration Correction
%% ========================================================================
% 코드 1 방식 이식:
% - pitch(theta_s) + bank/roll(phi)를 함께 사용한 3D 중력 보정
% - ax/ay뿐 아니라 az_corr까지 생성하여 이후 BPF RMS 진동 지수에 사용

ax_gravity =  g .* sin(theta_s);
ay_gravity =  g .* sin(phi) .* cos(theta_s);
az_gravity = -g .* cos(theta_s) .* cos(phi);  % 센서 축 정의에 따라 AccelZ가 +g 또는 -g면 데이터 확인 필요

ax_corr = ax_s - ax_gravity;
ay_corr = ay_s - ay_gravity;
az_corr = az_s - az_gravity;

%% ========================================================================
%  8. Wheel Slip Ratio Approximation + Kalman Slip Index Fusion
%% ========================================================================
% 코드 1 방식 이식:
% - yaw rate에 의한 좌/우 휠속도 차이를 보정
% - wheel slip과 INS AngleSlip을 Kalman filter로 fusion
% - 직선 구간에서는 AngleSlip이 거의 0이므로 wheel slip 위주
% - 코너링 구간에서는 AngleSlip도 보조 측정값으로 사용

% Prefer independent vehicle speed for slip calculation.
% Speed2D is independent from wheel speed; fallback to Vx_s if unavailable/invalid.
if exist('Speed2D', 'var')
    Vveh_kph = Speed2D(:) * 3.6;
    Vveh_kph = Vveh_kph(1:N);
else
    Vveh_kph = Vx_s * 3.6;
end

% Fallback for invalid GPS/INS speed or tunnel-like missing sections
invalid_vveh = ~isfinite(Vveh_kph) | Vveh_kph < 5;
Vveh_kph(invalid_vveh) = Vx_s(invalid_vveh) * 3.6;

VS_ms = Vveh_kph / 3.6;
denom_kph = max(Vveh_kph, 1);

% Yaw corrected wheel slip ratio [dimensionless]
V_left_kph  = (VS_ms - r_s .* track_w ./ 2) * 3.6;
V_right_kph = (VS_ms + r_s .* track_w ./ 2) * 3.6;

slip_FL = (whl_FL - V_left_kph)  ./ denom_kph;
slip_FR = (whl_FR - V_right_kph) ./ denom_kph;
slip_RL = (whl_RL - V_left_kph)  ./ denom_kph;
slip_RR = (whl_RR - V_right_kph) ./ denom_kph;

slip_FL(~valid_speed) = NaN;
slip_FR(~valid_speed) = NaN;
slip_RL(~valid_speed) = NaN;
slip_RR(~valid_speed) = NaN;

slip_front = mean([slip_FL, slip_FR], 2, 'omitnan');
slip_rear  = mean([slip_RL, slip_RR], 2, 'omitnan');

slip_abs_each = abs([slip_FL, slip_FR, slip_RL, slip_RR]);
slip_abs_max = max(slip_abs_each, [], 2);
slip_abs_med = median(slip_abs_each, 2, 'omitnan');

% Unrealistic spikes 제거: 속도 소스 mismatch / GPS dropout 방어
slip_abs_med(slip_abs_med > 0.50) = NaN;
slip_abs_max(slip_abs_max > 0.80) = NaN;

% Wheel-slip measurement for Kalman update
% 수정: 4개 바퀴 중 최대값(slip_abs_max)은 한 바퀴 노이즈에 과민하므로
% Kalman 입력에는 median 기반 slip_abs_med를 사용한다.
% NaN/저속 구간은 아래 Kalman loop에서 update를 건너뛰게 유지한다.
slip_wheel = slip_abs_med;

% INS lateral slip angle normalization: 5 deg 이상이면 최대 위험으로 간주
% NaN/저속 구간은 아래 Kalman loop에서 update를 건너뛰게 유지한다.
slip_ins = min(abs(angle_slip_s_deg) / 5.0, 1);

straight  = abs(sas_s) < 5 & abs(rad2deg(r_s)) < 3;
cornering = ~straight;

% Kalman fusion parameters from code 1
R_wheel = 0.05;
R_ins   = 0.10;
Q_kf    = 1e-3;
x_kf = 0;
P_kf = 1;
slip_kf = zeros(N,1);

for i = 1:N
    P_pred = P_kf + Q_kf;

    % Update 1: wheel speed slip
    % 수정: 저속/NaN 구간을 0 측정값처럼 넣지 않고, 측정 update를 건너뛴다.
    if valid_speed(i) && isfinite(slip_wheel(i))
        K1 = P_pred / (P_pred + R_wheel);
        x_kf = x_kf + K1 * (slip_wheel(i) - x_kf);
        P_kf = (1 - K1) * P_pred;
    else
        P_kf = P_pred;
    end

    % Update 2: INS AngleSlip, only meaningful while cornering
    % 수정: INS slip도 valid_speed + finite 조건에서만 반영한다.
    if valid_speed(i) && cornering(i) && isfinite(slip_ins(i))
        K2 = P_kf / (P_kf + R_ins);
        x_kf = x_kf + K2 * (slip_ins(i) - x_kf);
        P_kf = (1 - K2) * P_kf;
    end

    slip_kf(i) = x_kf;
end

% Display / classification index: 0.3 slip ratio를 1.0으로 normalize
slip_index = min(slip_kf / 0.3, 1);
slip_index(~valid_speed) = NaN;

%% ========================================================================
%  9. Tire Slip Angle Estimation using Bicycle Model
%% ========================================================================
% alpha_f = delta - atan((Vy + lf*r) / Vx)
% alpha_r = -atan((Vy - lr*r) / Vx)

Vx_safe = max(Vx_s, 0.1);

alpha_f = delta_s - atan2(Vy_s + lf .* r_s, Vx_safe);
alpha_r = -atan2(Vy_s - lr .* r_s, Vx_safe);

alpha_f(~valid_speed) = NaN;
alpha_r(~valid_speed) = NaN;

alpha_f_deg = rad2deg(alpha_f);
alpha_r_deg = rad2deg(alpha_r);

slip_angle_max = max(abs([alpha_f, alpha_r]), [], 2);
slip_angle_diff = abs(alpha_f - alpha_r);

%% ========================================================================
%  10. Bicycle Model Tire Force Estimation
%% ========================================================================

r_dot = gradient(r_s, dt);

Fy_f = (lr * m .* ay_corr + Iz .* r_dot) / L;
Fy_r = (lf * m .* ay_corr - Iz .* r_dot) / L;

Fx_total = m .* ax_corr;
Fx_f = lambda_x .* Fx_total;
Fx_r = (1 - lambda_x) .* Fx_total;

%% ========================================================================
%  11. Vertical Load Estimation
%% ========================================================================

Fz_f = m * g * lr / L - m .* ax_corr * h / L;
Fz_r = m * g * lf / L + m .* ax_corr * h / L;

Fz_f(Fz_f <= 0) = NaN;
Fz_r(Fz_r <= 0) = NaN;

%% ========================================================================
%  12. Required Friction Coefficient Estimation
%% ========================================================================

mu_f = sqrt(Fx_f.^2 + Fy_f.^2) ./ Fz_f;
mu_r = sqrt(Fx_r.^2 + Fy_r.^2) ./ Fz_r;
mu_required = max([mu_f, mu_r], [], 2);

mu_f(~valid_speed) = NaN;
mu_r(~valid_speed) = NaN;
mu_required(~valid_speed) = NaN;

mu_demand = sqrt(ax_corr.^2 + ay_corr.^2) / g;
mu_demand(~valid_speed) = NaN;

% ------------------------------------------------------------------------
% Confidence based mu_road estimation - revised
% ------------------------------------------------------------------------
% 기존 조건(accel_mag > 3.0)만 사용하면 일반 주행에서는 confidence가 거의
% 발생하지 않아 mu_road가 갱신되지 않는 문제가 있었다. 따라서 다음 조건을
% 추가해 일반 주행 중 의미 있는 가감속/코너링/슬립 조짐에서도 갱신되도록 한다.
%
% 1) ABS/TCS/ESP 개입
% 2) 일정 수준 이상의 가감속/횡가속도: accel_mag > 1.5
% 3) Kalman slip index가 일정 이상: slip_index > 0.05
% 4) bicycle model slip angle이 일정 이상: slip_angle_max > 3 deg
%
% 주의: mu_required는 실제 노면 최대 마찰계수(mu_max)가 아니라, 현재 차량이
% 요구한 마찰계수이다. mu_road는 confidence 구간에서 관측된 mu_required의
% 이동 최대값을 이용한 간접 추정값이다.
accel_mag = sqrt(ax_corr.^2 + ay_corr.^2);
confidence = (TCS_CTL_s | ABS_ACT_s | ESP_CTL_s | ...
              accel_mag > 1.5 | ...
              slip_index > 0.05 | ...
              slip_angle_max > deg2rad(3)) & valid_speed;

mu_candidate = nan(N,1);
mu_candidate(confidence & isfinite(mu_required)) = mu_required(confidence & isfinite(mu_required));

window_mu = max(1, round(5 / dt));  % approximately 5 seconds
mu_road = zeros(N,1);
mu_road(1) = 0.8;  % dry asphalt initial assumption

for i = 2:N
    seg = mu_candidate(max(1, i-window_mu):i);
    seg = seg(isfinite(seg));
    if ~isempty(seg)
        % confidence 구간에서는 최근 5초 내 최대 요구마찰계수로 갱신
        mu_road(i) = max(seg);
    else
        % confidence 구간이 없다고 해서 마찰계수가 0으로 감소하는 것은 아님.
        % 따라서 이전 추정값을 유지한다.
        mu_road(i) = mu_road(i-1);
    end
end

% 대시보드 표시 안정화를 위해 비현실적인 0 근처 고정 방지.
% 0.2는 '판단 보류/저자극 주행'의 하한값 성격이며, 실제 mu_max 측정값이 아님.
mu_road = min(max(mu_road, 0.2), 1.2);
mu_road(~valid_speed) = NaN;

%% ========================================================================
%  13. Equivalent Drive / Brake Torque Estimation
%% ========================================================================
% 실제 ECU 토크가 아니라, 차량 거동을 설명하기 위한 등가 요구 토크

F_aero = 0.5 * rho * CdA .* Vx_s.^2;
F_roll = Crr * m * g * ones(size(Vx_s));
F_grade = m * g .* sin(theta_s);

Fx_required_resist = m .* ax_s + F_aero + F_roll + F_grade;

T_wheel_equiv = Fx_required_resist .* R_tire;
T_drive_equiv = max(T_wheel_equiv, 0);
T_brake_equiv = max(-T_wheel_equiv, 0);

%% ========================================================================
%  14. IMU Roughness Index - AccelZ 3D Correction + BPF RMS
%% ========================================================================
% 코드 1 방식 이식:
% - 7장에서 계산한 az_corr를 사용
% - 0.5~10 Hz band-pass filter 후 1초 RMS 계산
% - 필터 적용이 불가능한 짧은 데이터/툴박스 문제는 high-pass 대체 계산 사용

try
    [b_bp, a_bp] = butter(4, [0.5 10] / (Fs/2), 'bandpass');
    az_filt = filtfilt(b_bp, a_bp, az_corr);
catch
    warning('BPF 적용 실패: movmean 기반 high-pass roughness로 대체합니다.');
    win_hp = max(5, round(0.5 / dt));
    az_filt = az_corr - movmean(az_corr, win_hp, 'omitnan');
end

win_rms = max(1, round(1 / dt));  % 1 second window
roughness_z = sqrt(movmean(az_filt.^2, win_rms, 'omitnan'));
roughness_xy = movstd(ax_corr, win_rms, 'omitnan') + ...
               movstd(ay_corr, win_rms, 'omitnan');

vib_norm = prctile(roughness_z(isfinite(roughness_z)), 99);
if isempty(vib_norm) || ~isfinite(vib_norm) || vib_norm <= 0
    vib_norm = 1;
end
vib_index = min(roughness_z / (vib_norm + eps), 1);
roughness_norm = vib_index;

roughness_z(~valid_speed) = NaN;
roughness_xy(~valid_speed) = NaN;
roughness_norm(~valid_speed) = NaN;
vib_index(~valid_speed) = NaN;

%% ========================================================================
%  15. Lateral Consistency Index
%% ========================================================================

ay_pred = Vx_s .* r_s;
lat_error = abs(ay_corr - ay_pred);
lat_error(~valid_speed) = NaN;

%% ========================================================================
%  16. Control Intervention Suspicion Indices
%% ========================================================================
% 실제 ABS/TCS/ESC 상태가 아니라, 데이터 기반 의심 지표

ABS_suspect = ax_corr < -3.0 & slip_abs_max > 0.08;
TCS_suspect = ax_corr > 1.5 & slip_abs_max > 0.08;
ESC_suspect = lat_error > 1.2 & abs(rad2deg(r_s)) > 5;

%% ========================================================================
%  17. Road Surface / Grip State Classification
%% ========================================================================

road_state = strings(N, 1);
road_state_code = zeros(N, 1);

% State code
% 0: INVALID_LOW_SPEED
% 1: LOW_EXCITATION_UNKNOWN
% 2: MEDIUM_GRIP_CONFIRMED
% 3: HIGH_GRIP_CONFIRMED
% 4: LOW_STABILITY_SUSPECTED
% 5: ROUGH_SURFACE

for i = 1:N

    if ~valid_speed(i) || isnan(mu_required(i))
        road_state(i) = "INVALID_LOW_SPEED";
        road_state_code(i) = 0;

    elseif vib_index(i) > 0.60
        road_state(i) = "ROUGH_SURFACE";
        road_state_code(i) = 5;

    elseif slip_index(i) > 0.08 || ...
           slip_angle_max(i) > deg2rad(6) || ...
           lat_error(i) > 1.2 || ...
           ESC_suspect(i)
        road_state(i) = "LOW_STABILITY_SUSPECTED";
        road_state_code(i) = 4;

    elseif confidence(i) && mu_required(i) > 0.55
        % 수정: mu_road는 과거 confidence 구간의 이동 최대값이라 고접지 확정 판정에 쓰면 과장될 수 있음.
        % 현재 샘플에서 confidence가 있고 실제 계산값 mu_required가 충분히 클 때만 고접지 확인.
        road_state(i) = "HIGH_GRIP_CONFIRMED";
        road_state_code(i) = 3;

    elseif confidence(i) && mu_required(i) > 0.35
        % 수정: 중간 접지도 현재 confidence + 현재 mu_required 기준으로 판정.
        road_state(i) = "MEDIUM_GRIP_CONFIRMED";
        road_state_code(i) = 2;

    else
        road_state(i) = "LOW_EXCITATION_UNKNOWN";
        road_state_code(i) = 1;
    end
end

%% ========================================================================
%  18. Summary Result
%% ========================================================================

valid_idx = road_state ~= "INVALID_LOW_SPEED";

fprintf('\n[Vehicle Parameters]\n');
fprintf('m                 : %.1f kg\n', m);
fprintf('L                 : %.2f m\n', L);
fprintf('lf                : %.2f m\n', lf);
fprintf('lr                : %.2f m\n', lr);
fprintf('h                 : %.2f m\n', h);
fprintf('Iz                : %.1f kg*m^2\n', Iz);
fprintf('steering ratio    : %.1f\n', steering_ratio);
fprintf('roll offset       : %.3f deg\n', roll_offset_deg);

fprintf('\n[Result Summary]\n');
fprintf('Total samples     : %d\n', N);
fprintf('Valid samples     : %d\n', sum(valid_idx));

if any(valid_idx)
    fprintf('Mean mu_required  : %.3f\n', mean(mu_required(valid_idx), 'omitnan'));
    fprintf('Max mu_required   : %.3f\n', max(mu_required(valid_idx), [], 'omitnan'));
    fprintf('Mean mu_demand    : %.3f\n', mean(mu_demand(valid_idx), 'omitnan'));
    fprintf('Max mu_demand     : %.3f\n', max(mu_demand(valid_idx), [], 'omitnan'));
    fprintf('Max slip index    : %.3f\n', max(slip_index(valid_idx), [], 'omitnan'));
    fprintf('Max wheel-slip ratio diagnostic : %.3f\n', max(slip_abs_max(valid_idx), [], 'omitnan'));
    fprintf('Max slip angle    : %.3f deg\n', max(rad2deg(slip_angle_max(valid_idx)), [], 'omitnan'));
    fprintf('Max bank angle    : %.3f deg\n', max(abs(bank_final_deg(valid_idx)), [], 'omitnan'));
    fprintf('Max roughness     : %.5f\n', max(roughness_norm(valid_idx), [], 'omitnan'));
end

fprintf('\n[Road State Ratio]\n');
state_list = ["LOW_EXCITATION_UNKNOWN", ...
              "MEDIUM_GRIP_CONFIRMED", ...
              "HIGH_GRIP_CONFIRMED", ...
              "LOW_STABILITY_SUSPECTED", ...
              "ROUGH_SURFACE"];

for s = 1:length(state_list)
    ratio = sum(road_state == state_list(s)) / max(sum(valid_idx), 1) * 100;
    fprintf('%-30s : %.2f %%\n', state_list(s), ratio);
end

fprintf('============================================================\n');

%% ========================================================================
%  19. Save Results
%% ========================================================================

result.Time = t;

result.VehicleParameter.m = m;
result.VehicleParameter.L = L;
result.VehicleParameter.lf = lf;
result.VehicleParameter.lr = lr;
result.VehicleParameter.h = h;
result.VehicleParameter.Iz = Iz;
result.VehicleParameter.steering_ratio = steering_ratio;
result.VehicleParameter.CdA = CdA;
result.VehicleParameter.Crr = Crr;
result.VehicleParameter.R_tire = R_tire;
result.VehicleParameter.lambda_x = lambda_x;

result.Vx = Vx_s;
result.Vy = Vy_s;
result.ax_raw = ax_s;
result.ay_raw = ay_s;
result.az = az_s;
result.yaw_rate_rad = r_s;
result.SAS_Angle_deg = sas_s;

result.theta_grade_rad = theta_s;
result.grade_source = grade_source;
result.roll_offset_deg = roll_offset_deg;
result.roll_zeroed_deg = roll_zeroed_deg;
result.vehicle_roll_deg = vehicle_roll_deg;
result.bank_roll_deg = bank_roll_deg;
result.bank_dyn_deg = bank_dyn_deg;
result.bank_final_deg = bank_final_deg;
result.valid_bank = valid_bank;

result.ax_corr = ax_corr;
result.ay_corr = ay_corr;
result.az_corr = az_corr;

result.Vveh_slip_kph = Vveh_kph;
result.slip_FL = slip_FL;
result.slip_FR = slip_FR;
result.slip_RL = slip_RL;
result.slip_RR = slip_RR;
result.slip_front = slip_front;
result.slip_rear = slip_rear;
result.slip_abs_max = slip_abs_max;
result.slip_abs_med = slip_abs_med;
result.slip_wheel = slip_wheel;
result.slip_ins = slip_ins;
result.slip_kf = slip_kf;
result.slip_index = slip_index;

result.alpha_f_deg = alpha_f_deg;
result.alpha_r_deg = alpha_r_deg;
result.slip_angle_max_deg = rad2deg(slip_angle_max);
result.slip_angle_diff_deg = rad2deg(slip_angle_diff);

result.Fx_f = Fx_f;
result.Fx_r = Fx_r;
result.Fy_f = Fy_f;
result.Fy_r = Fy_r;
result.Fz_f = Fz_f;
result.Fz_r = Fz_r;

result.mu_f = mu_f;
result.mu_r = mu_r;
result.mu_required = mu_required;
result.mu_demand = mu_demand;
result.mu_display = mu_required;  % dashboard raw calculated value
result.confidence = confidence;
result.mu_candidate = mu_candidate;
result.mu_road = mu_road;

result.roughness_z = roughness_z;
result.roughness_xy = roughness_xy;
result.roughness_norm = roughness_norm;
result.vib_index = vib_index;
result.az_filt = az_filt;
result.lat_error = lat_error;

result.T_wheel_equiv = T_wheel_equiv;
result.T_drive_equiv = T_drive_equiv;
result.T_brake_equiv = T_brake_equiv;

result.ABS_suspect = ABS_suspect;
result.TCS_suspect = TCS_suspect;
result.ESC_suspect = ESC_suspect;

result.road_state = road_state;
result.road_state_code = road_state_code;
result.PosLat = pos_lat;
result.PosLon = pos_lon;

save(SAVE_FILE, 'result');
fprintf('\nSaved result file: %s\n', SAVE_FILE);


%% ========================================================================
%  20. Stable Single-Window Dashboard
%  - app/guidata 기반으로 재생/정지/스텝/슬라이더 연동
%  - 값이 NaN인 구간은 자동으로 '-' 처리
%% ========================================================================
if RUN_DASHBOARD

    speed_kph = Vx_s * 3.6;
    grade_deg = rad2deg(theta_s);  % selected grade source: SLOPE_TCU or Pitch_Angle
    slip_angle_max_deg = rad2deg(slip_angle_max);
    rough_rms = roughness_z;
    mu_display = mu_required;  % dashboard에서는 실제 계산값인 요구 마찰계수 표시

    % Object / CIPV availability check.
    % BEV vehicles are displayed only when object signals are actually valid.
    has_front_obj = false(N,1);
    front_obj_x = nan(N,1);
    front_obj_y = nan(N,1);

    if exist('SOW_x_pos_01','var') && exist('SOW_y_pos_cor_01','var')
        tmpx = SOW_x_pos_01(:); tmpy = SOW_y_pos_cor_01(:);
        Mobj = min([N, length(tmpx), length(tmpy)]);
        front_obj_x(1:Mobj) = tmpx(1:Mobj);
        front_obj_y(1:Mobj) = tmpy(1:Mobj);
        has_front_obj = isfinite(front_obj_x) & front_obj_x > 0 & front_obj_x < 80 & abs(front_obj_y) < 3.5;
    elseif exist('PosX01','var') && exist('PosY01','var')
        tmpx = PosX01(:); tmpy = PosY01(:);
        Mobj = min([N, length(tmpx), length(tmpy)]);
        front_obj_x(1:Mobj) = tmpx(1:Mobj);
        front_obj_y(1:Mobj) = tmpy(1:Mobj);
        has_front_obj = isfinite(front_obj_x) & front_obj_x > 0 & front_obj_x < 80 & abs(front_obj_y) < 3.5;
    end

    has_side_left = false(N,1); has_side_right = false(N,1);
    side_left_x = nan(N,1); side_right_x = nan(N,1);
    side_left_y = nan(N,1); side_right_y = nan(N,1);
    for objNo = 1:8
        xName1 = sprintf('SOW_x_pos_%02d', objNo);
        yName1 = sprintf('SOW_y_pos_cor_%02d', objNo);
        xName2 = sprintf('PosX%02d', objNo);
        yName2 = sprintf('PosY%02d', objNo);
        if exist(xName1,'var') && exist(yName1,'var')
            ox = eval([xName1 '(:)']); oy = eval([yName1 '(:)']);
        elseif exist(xName2,'var') && exist(yName2,'var')
            ox = eval([xName2 '(:)']); oy = eval([yName2 '(:)']);
        else
            continue
        end
        Mobj = min([N, length(ox), length(oy)]);
        ox = ox(1:Mobj); oy = oy(1:Mobj);
        leftCand = isfinite(ox) & isfinite(oy) & ox > -15 & ox < 35 & oy > 2.0 & oy < 6.0;
        rightCand = isfinite(ox) & isfinite(oy) & ox > -15 & ox < 35 & oy < -2.0 & oy > -6.0;
        idxL = find(leftCand & ~has_side_left(1:Mobj));
        idxR = find(rightCand & ~has_side_right(1:Mobj));
        has_side_left(idxL) = true; side_left_x(idxL)=ox(idxL); side_left_y(idxL)=oy(idxL);
        has_side_right(idxR) = true; side_right_x(idxR)=ox(idxR); side_right_y(idxR)=oy(idxR);
    end

    frame_valid = valid_speed(:) & isfinite(speed_kph(:));
    frame_valid_mu = frame_valid & isfinite(mu_display(:)) & isfinite(slip_index(:));
    start_idx = find(frame_valid_mu, 1, 'first');
    if isempty(start_idx)
        start_idx = find(frame_valid, 1, 'first');
    end
    if isempty(start_idx)
        start_idx = 1;
    end
    end_idx = N;

    gps_valid = pos_lat ~= 0 & pos_lon ~= 0 & isfinite(pos_lat) & isfinite(pos_lon);

    % Convert GPS latitude/longitude to local XY coordinates for route view.
    % This removes the dependency on map/GPS UI while preserving the driving path shape.
    route_valid = gps_valid(:);
    route_x = nan(N,1);
    route_y = nan(N,1);
    if any(route_valid)
        lat0 = median(pos_lat(route_valid), 'omitnan');
        lon0 = median(pos_lon(route_valid), 'omitnan');
        route_x(route_valid) = (pos_lon(route_valid) - lon0) .* 111320 .* cosd(lat0);
        route_y(route_valid) = (pos_lat(route_valid) - lat0) .* 110540;
    else
        % Fallback: dead-reckoning-like straight path from speed when GPS is not valid.
        route_valid = true(N,1);
        route_x = zeros(N,1);
        route_y = cumtrapz(t, Vx_s);
    end

    fig = figure('Name', 'Road Surface Dashboard', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.965 0.965 0.955], ...
                 'Position', [60 60 1450 820], ...
                 'MenuBar', 'none');

    ui.headerAx = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.92 0.96 0.07]);
    localPanelBase(ui.headerAx, [0.98 0.98 0.98]);
    text(ui.headerAx, 0.025, 0.58, '주행 데이터 분석 대시보드', 'Units', 'normalized', ...
        'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'none', 'Color', [0.08 0.08 0.08]);
    text(ui.headerAx, 0.025, 0.26, sprintf('Road Surface Project · %s · %.0f samples', datestr(now,'yyyy-mm-dd'), N), ...
        'Units', 'normalized', 'FontSize', 11, 'Interpreter', 'none', 'Color', [0.38 0.38 0.38]);
    ui.headerLive = text(ui.headerAx, 0.37, 0.22, '', 'Units', 'normalized', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Interpreter', 'none', 'Color', [0.20 0.20 0.20], ...
        'Visible', 'off');

    datasetValue = find(strcmp(DATA_FILE, DATASET_LIST), 1);
    if isempty(datasetValue), datasetValue = 1; end
    ui.datasetPopup = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
        'Position', [0.34 0.925 0.105 0.045], 'String', DATASET_LIST, 'Value', datasetValue, ...
        'FontSize', 10.5, 'BackgroundColor', [0.98 0.98 0.98], ...
        'Callback', @(src,~)localCbDataset(fig, src));

    ui.btnReset = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.46 0.925 0.08 0.045], 'String', '◀◀', 'FontSize', 12, ...
        'Callback', @(~,~)localCbReset(fig));
    ui.btnPlay = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.555 0.925 0.09 0.045], 'String', '▶ 재생', 'FontSize', 11, ...
        'Callback', @(~,~)localCbPlay(fig));
    ui.btnStep = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.655 0.925 0.08 0.045], 'String', '▶▶', 'FontSize', 12, ...
        'Callback', @(~,~)localCbStep(fig));
    ui.btnSpeed = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.77 0.925 0.095 0.045], 'String', '속도 ×1', 'FontSize', 11, ...
        'Callback', @(~,~)localCbSpeed(fig));
    ui.btnExport = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.875 0.925 0.10 0.045], 'String', '내보내기', 'FontSize', 11, ...
        'Callback', @(~,~)localCbExport(fig));

    ui.card(1) = localCreateCard(fig, [0.02 0.75 0.23 0.12], '차속', 'kph');
    ui.card(2) = localCreateCard(fig, [0.265 0.75 0.23 0.12], '슬립 지수', '');
    ui.card(3) = localCreateCard(fig, [0.51 0.75 0.23 0.12], '요구 마찰계수 μ', '');
    ui.card(4) = localCreateCard(fig, [0.755 0.75 0.225 0.12], '도로 경사', 'deg');

    % visible panel borders
    localAddBorder(fig, [0.02 0.92 0.96 0.07]);
    localAddBorder(fig, [0.02 0.75 0.23 0.12]);
    localAddBorder(fig, [0.265 0.75 0.23 0.12]);
    localAddBorder(fig, [0.51 0.75 0.23 0.12]);
    localAddBorder(fig, [0.755 0.75 0.225 0.12]);

    ui.axGps = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04 0.265 0.56 0.40]);
    localPanelBase(ui.axGps, [0.99 0.99 0.985]);
    title(ui.axGps, '주행 경로 + 노면 상태', 'FontSize', 12, 'FontWeight', 'bold');
    hold(ui.axGps, 'on'); axis(ui.axGps, 'equal'); axis(ui.axGps, 'off');

    if any(route_valid)
        plot(ui.axGps, route_x(route_valid), route_y(route_valid), '-', 'Color', [0.72 0.72 0.70], 'LineWidth', 2.0);
        idxGood = route_valid & road_state_code == 3;
        idxOkay = route_valid & (road_state_code == 1 | road_state_code == 2);
        idxBad  = route_valid & (road_state_code == 4 | road_state_code == 5);
        scatter(ui.axGps, route_x(idxGood), route_y(idxGood), 34, [0.40 0.65 0.15], 'filled');
        scatter(ui.axGps, route_x(idxOkay), route_y(idxOkay), 34, [0.79 0.50 0.10], 'filled');
        scatter(ui.axGps, route_x(idxBad),  route_y(idxBad),  34, [0.90 0.30 0.30], 'filled');

        validStart = start_idx;
        if ~route_valid(validStart)
            tmpIdx = find(route_valid, 1, 'first');
            if ~isempty(tmpIdx), validStart = tmpIdx; end
        end
        % Current position marker: use a single compact blue marker so it does not
        % hide the route and does not look like a duplicated double circle.
        ui.currentGps = scatter(ui.axGps, route_x(validStart), route_y(validStart), 95, ...
            'o', 'MarkerFaceColor', [0.20 0.52 0.90], 'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.4);

        % Fit route with margin
        rx = route_x(route_valid); ry = route_y(route_valid);
        rx = rx(isfinite(rx)); ry = ry(isfinite(ry));
        if ~isempty(rx) && ~isempty(ry)
            xpad = max(20, (max(rx)-min(rx))*0.08); ypad = max(20, (max(ry)-min(ry))*0.08);
            xlim(ui.axGps, [min(rx)-xpad max(rx)+xpad]);
            ylim(ui.axGps, [min(ry)-ypad max(ry)+ypad]);
        end

        ui.gpsInfo = text(ui.axGps, 0.98, 0.96, 'Local XY route', 'Units', 'normalized', ...
            'HorizontalAlignment', 'right', 'FontSize', 9, 'Color', [0.42 0.42 0.42], 'Interpreter', 'none');
    else
        text(ui.axGps, 0.5, 0.5, '주행 경로 데이터 없음', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', [0.45 0.45 0.45]);
        ui.currentGps = gobjects(1);
        ui.gpsInfo = text(ui.axGps, 0.98, 0.96, 'Route unavailable', 'Units', 'normalized', ...
            'HorizontalAlignment', 'right', 'FontSize', 9, 'Color', [0.42 0.42 0.42]);
    end

    % Unified legend / surface-index card for the route panel.
    legendPos = [0.485 0.33 0.115 0.27];
    ui.legendPanel = annotation(fig, 'rectangle', legendPos, ...
        'FaceColor', [1 1 1], 'Color', [0.86 0.86 0.84], 'LineWidth', 1.0);
    ui.legendTitle = annotation(fig, 'textbox', [legendPos(1)+0.010 legendPos(2)+legendPos(4)-0.045 legendPos(3)-0.020 0.030], ...
        'String', '범례', 'LineStyle', 'none', 'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', [0.18 0.18 0.18], 'HorizontalAlignment', 'left');
    legendColors = {[0.40 0.65 0.15], [0.79 0.50 0.10], [0.90 0.30 0.30], [0.20 0.52 0.90]};
    legendLabels = {'좋음','양호','나쁨','현재 위치'};
    legendRowY = [0.175 0.130 0.085 0.040];
    for kk = 1:4
        ui.legendDot(kk) = annotation(fig, 'ellipse', [legendPos(1)+0.012 legendPos(2)+legendRowY(kk) 0.010 0.018], ...
            'FaceColor', legendColors{kk}, 'Color', legendColors{kk}, 'LineWidth', 1.0);
        ui.legendText(kk) = annotation(fig, 'textbox', [legendPos(1)+0.028 legendPos(2)+legendRowY(kk)-0.004 legendPos(3)-0.038 0.026], ...
            'String', legendLabels{kk}, 'LineStyle', 'none', 'FontSize', 9.5, ...
            'Color', [0.20 0.20 0.20], 'HorizontalAlignment', 'left');
    end
    ui.legendSep = annotation(fig, 'line', [legendPos(1)+0.010 legendPos(1)+legendPos(3)-0.010], ...
        [legendPos(2)+0.032 legendPos(2)+0.032], 'Color', [0.88 0.88 0.86], 'LineWidth', 1.0);
    ui.legendIndex = annotation(fig, 'textbox', [legendPos(1)+0.010 legendPos(2)+0.004 legendPos(3)-0.020 0.024], ...
        'String', '양호 (index 0.43)', 'LineStyle', 'none', 'FontSize', 10.5, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'BackgroundColor', [0.93 0.89 0.80], 'Color', [0.45 0.33 0.10]);

    ui.axBev = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.63 0.53 0.33 0.18]);
    localPanelBase(ui.axBev, [0.99 0.99 0.985]);
    title(ui.axBev, 'Camera Bird''s-Eye View', 'FontSize', 12, 'FontWeight', 'bold');
    hold(ui.axBev, 'on'); grid(ui.axBev, 'on');
    xlim(ui.axBev, [-10 80]); ylim(ui.axBev, [-10 10]); axis(ui.axBev, 'equal');
    xlabel(ui.axBev, 'Forward X [m]'); ylabel(ui.axBev, 'Lateral Y [m]');
    set(ui.axBev, 'XTick', 0:20:80, 'YTick', -10:5:10, 'FontSize', 8);
    patch(ui.axBev, [-10 80 80 -10], [-10 -10 10 10], [0.96 0.96 0.95], 'EdgeColor','none');
    plot(ui.axBev, [0 80], [0 0], ':', 'Color', [0.72 0.72 0.72], 'LineWidth', 1.0);
    ui.laneLAll = plot(ui.axBev, nan, nan, '-', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.0);
    ui.laneRAll = plot(ui.axBev, nan, nan, '-', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.0);
    ui.laneLValid = plot(ui.axBev, nan, nan, 'g-', 'LineWidth', 2.0);
    ui.laneRValid = plot(ui.axBev, nan, nan, 'g-', 'LineWidth', 2.0);
    ui.bevDyn = gobjects(0);
    ui.bevInfo = text(ui.axBev, 0.98, 0.96, 'Camera objects', 'Units', 'normalized', ...
        'HorizontalAlignment','right', 'FontSize', 9, 'Color', [0.35 0.35 0.35]);
    egoW=2.0; egoL=4.5; objW=2.0; objL=4.5; sideL=4.5;

    ui.axIdx = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.63 0.265 0.33 0.22]);
    localPanelBase(ui.axIdx, [0.99 0.99 0.985]);
    title(ui.axIdx, '노면 상태 지수', 'FontSize', 13, 'FontWeight', 'bold');
    hold(ui.axIdx, 'on'); xlim(ui.axIdx, [0 1]); ylim(ui.axIdx, [0 4]); axis(ui.axIdx,'off');
    idxNames = {'슬립 지수','진동 RMS','마찰계수 μ','경사 보정'};
    idxY = [3.15 2.25 1.35 0.45];
    idxColor = [0.90 0.28 0.28; 0.78 0.49 0.08; 0.42 0.66 0.11; 0.22 0.53 0.88];
    for q=1:4
        text(ui.axIdx, 0.04, idxY(q)+0.26, idxNames{q}, 'FontSize', 11, 'Color', [0.30 0.30 0.30]);
        patch(ui.axIdx, [0.04 0.92 0.92 0.04], [idxY(q) idxY(q) idxY(q)+0.16 idxY(q)+0.16], [0.92 0.92 0.90], 'EdgeColor','none');
        ui.idxFg(q) = patch(ui.axIdx, [0.04 0.20 0.20 0.04], [idxY(q) idxY(q) idxY(q)+0.16 idxY(q)+0.16], idxColor(q,:), 'EdgeColor','none');
        ui.idxVal(q) = text(ui.axIdx, 0.94, idxY(q)+0.24, '-', 'HorizontalAlignment','right', 'FontSize', 11, 'Color',[0.25 0.25 0.25]);
    end
    % Surface-index summary is shown next to the route legend, so the old badge in
    % this panel is hidden.
    ui.summaryBadge = text(ui.axIdx, 0.50, 0.08, '양호 (index 0.43)', 'HorizontalAlignment','center', ...
        'FontSize', 12, 'FontWeight','bold', 'BackgroundColor',[0.93 0.89 0.80], ...
        'Color',[0.45 0.33 0.10], 'Visible', 'off');

    ui.axTrend(1) = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.04 0.075 0.28 0.105]);
    ui.axTrend(2) = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.36 0.075 0.28 0.105]);
    ui.axTrend(3) = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.68 0.075 0.28 0.105]);
    trendNames = {'슬립 지수','진동 RMS (AccelZ)','요구 마찰계수 μ_{required}'};
    trendData = {slip_index, rough_rms, mu_display};
    trendColor = [0.90 0.28 0.28; 0.78 0.49 0.08; 0.05 0.64 0.46];
    trendThresh = [0.10, localNanMean(rough_rms), 0.35];
    for q=1:3
        ax = ui.axTrend(q);
        localPanelBase(ax, [0.99 0.99 0.985]);
        title(ax, trendNames{q}, 'FontSize', 11, 'FontWeight','bold');
        hold(ax,'on');
        plot(ax, t, trendData{q}, 'Color', trendColor(q,:), 'LineWidth', 1.4);
        if isfinite(trendThresh(q))
            yline(ax, trendThresh(q), '--', 'Color', trendColor(q,:), 'LineWidth', 1.0);
        end
        ui.trendMarker(q) = xline(ax, t(start_idx), 'Color', [0.20 0.40 0.90], 'LineWidth', 1.5);
        grid(ax,'on'); ax.XTick=[]; ax.YTick=[];
    end

    ui.axTime = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02 0.015 0.96 0.035]);
    localPanelBase(ui.axTime, [0.99 0.99 0.99]); hold(ui.axTime,'on'); xlim(ui.axTime,[0 1]); ylim(ui.axTime,[0 1]); axis(ui.axTime,'off');
    ui.timeLeft = text(ui.axTime, 0.015, 0.5, '00:00:00', 'VerticalAlignment','middle', 'FontSize', 10, 'Color',[0.45 0.45 0.45]);
    ui.timeRight = text(ui.axTime, 0.965, 0.5, localSecToHMS(t(end)-t(start_idx)), 'HorizontalAlignment','right', 'VerticalAlignment','middle', 'FontSize', 10, 'Color',[0.30 0.30 0.30]);
    % Timeline display bar. Time movement is controlled by the ◀◀ / ▶▶ buttons.
    ui.timeTrackBg = plot(ui.axTime, [0.08 0.92], [0.5 0.5], '-', 'Color', [0.88 0.88 0.88], ...
        'LineWidth', 6, 'HitTest', 'off');
    ui.timeTrackFg = plot(ui.axTime, [0.08 0.08], [0.5 0.5], '-', 'Color', [0.76 0.76 0.76], ...
        'LineWidth', 6, 'HitTest', 'off');
    ui.timeDot = scatter(ui.axTime, 0.08, 0.5, 180, [0.10 0.48 0.95], 'filled', ...
        'MarkerEdgeColor', [1 1 1], 'LineWidth', 1.2, 'HitTest', 'off');
    ui.timeSlider = gobjects(1);

    localAddBorder(fig, [0.04 0.265 0.56 0.40]);
    localAddBorder(fig, [0.63 0.53 0.33 0.18]);
    localAddBorder(fig, [0.63 0.265 0.33 0.22]);
    localAddBorder(fig, [0.04 0.075 0.28 0.105]);
    localAddBorder(fig, [0.36 0.075 0.28 0.105]);
    localAddBorder(fig, [0.68 0.075 0.28 0.105]);
    localAddBorder(fig, [0.02 0.015 0.96 0.035]);

    % Camera lane model arrays from test_plot_cam.m style
    lane = struct();
    lane.has = exist('LaneMarkPosition_Lh_ME','var') && exist('LaneMarkPosition_Rh_ME','var');
    if lane.has
        lane.c0_l = LaneMarkPosition_Lh_ME(:); lane.c1_l = LaneMarkHeadAngle_Lh_ME(:);
        lane.c2_l = LaneMarkModelA_Lh_ME(:); lane.c3_l = LaneMarkModelDerivA_Lh_ME(:);
        lane.vr_l = LaneMarkViewRange_Lh_ME(:); lane.qual_l = LaneMarkQuality_Lh_ME(:);
        lane.c0_r = LaneMarkPosition_Rh_ME(:); lane.c1_r = LaneMarkHeadAngle_Rh_ME(:);
        lane.c2_r = LaneMarkModelA_Rh_ME(:); lane.c3_r = LaneMarkModelDerivA_Rh_ME(:);
        lane.vr_r = LaneMarkViewRange_Rh_ME(:); lane.qual_r = LaneMarkQuality_Rh_ME(:);
    end

    % Camera object arrays from PosX01~PosX08 / PosY01~PosY08
    camObjX = nan(N,8); camObjY = nan(N,8);
    for objNo = 1:8
        xName = sprintf('PosX%02d', objNo);
        yName = sprintf('PosY%02d', objNo);
        if exist(xName,'var') && exist(yName,'var')
            ox = eval([xName '(:)']); oy = eval([yName '(:)']);
            Mobj = min([N, length(ox), length(oy)]);
            camObjX(1:Mobj,objNo) = ox(1:Mobj);
            camObjY(1:Mobj,objNo) = oy(1:Mobj);
        end
    end

    app = struct();
    app.ui = ui; app.N = N; app.t = t; app.startIdx = start_idx; app.endIdx = end_idx; app.idx = start_idx;
    app.dataFile = DATA_FILE; app.datasetList = DATASET_LIST; app.choiceFile = DATASET_CHOICE_FILE; app.scriptPath = SCRIPT_PATH;
    app.stepBase = max(1, ANIMATION_STEP); app.speedChoices = [1 2 4]; app.speedChoiceIdx = 1; app.isPlaying = false; app.isSeekingTime = false;
    app.timer = timer('ExecutionMode','fixedSpacing', 'Period', max(0.02, ANIMATION_PAUSE), 'TimerFcn', @(~,~)localTimerStep(fig));
    app.speed_kph = speed_kph; app.grade_deg = grade_deg; app.grade_source = grade_source; app.bank_final_deg = bank_final_deg; app.slip_abs_max = slip_index; app.slip_diag_max = slip_abs_max;
    app.has_front_obj = has_front_obj; app.front_obj_x = front_obj_x; app.front_obj_y = front_obj_y;
    app.has_side_left = has_side_left; app.side_left_x = side_left_x; app.side_left_y = side_left_y;
    app.has_side_right = has_side_right; app.side_right_x = side_right_x; app.side_right_y = side_right_y;
    app.slip_angle_max_deg = slip_angle_max_deg; app.mu_required = mu_required; app.mu_demand = mu_demand; app.mu_road = mu_road; app.mu_display = mu_display;
    app.rough_rms = rough_rms; app.roughness_norm = roughness_norm; app.lat_error = lat_error;
    app.road_state = road_state; app.road_state_code = road_state_code;
    app.ABS_suspect = ABS_suspect; app.TCS_suspect = TCS_suspect; app.ESC_suspect = ESC_suspect;
    app.pos_lat = pos_lat; app.pos_lon = pos_lon; app.gps_valid = gps_valid;
    app.route_x = route_x; app.route_y = route_y; app.route_valid = route_valid;
    app.maxRough = max(localFinite(rough_rms));
    if isempty(app.maxRough) || app.maxRough <= 0 || ~isfinite(app.maxRough), app.maxRough = 1; end
    app.egoW=egoW; app.egoL=egoL; app.objW=objW; app.objL=objL; app.sideL=sideL;
    app.lane = lane; app.camObjX = camObjX; app.camObjY = camObjY;

    guidata(fig, app);
    set(fig, 'CloseRequestFcn', @(src,~)localCloseDashboard(src));
    localUpdateDashboard(fig, start_idx);
end

fprintf('\nFinal project script finished.\n');

%% ========================================================================
%  Local functions for dashboard
%% ========================================================================
function localAddBorder(fig, pos)
annotation(fig, 'rectangle', pos, 'Color', [0.74 0.74 0.74], 'LineWidth', 0.8);
end

function localPanelBase(ax, bg)
set(ax, 'Color', bg, 'Box', 'on', 'XTick', [], 'YTick', []);
end

function card = localCreateCard(fig, pos, titleStr, unitStr)
card.ax = axes('Parent', fig, 'Units', 'normalized', 'Position', pos);
localPanelBase(card.ax, [0.94 0.94 0.92]); hold(card.ax, 'on'); xlim(card.ax,[0 1]); ylim(card.ax,[0 1]); axis(card.ax,'off');
card.title = text(card.ax, 0.06, 0.80, titleStr, 'FontSize', 10, 'Color', [0.35 0.35 0.35], 'Interpreter','none');
card.value = text(card.ax, 0.06, 0.46, '-', 'FontSize', 18, 'FontWeight','bold', 'Color',[0.08 0.08 0.08], 'Interpreter','none');
card.unit = text(card.ax, 0.34, 0.47, unitStr, 'FontSize', 10, 'Color',[0.45 0.45 0.45], 'Interpreter','none');
card.badge = text(card.ax, 0.06, 0.15, '-', 'FontSize', 11, 'FontWeight','bold', 'Color',[0.28 0.45 0.18], 'BackgroundColor',[0.87 0.91 0.82], 'Interpreter','none');
end

function localSetCard(card, valueStr, badgeStr, bg, fg)
set(card.value, 'String', valueStr); set(card.badge, 'String', ['  ' badgeStr '  '], 'BackgroundColor', bg, 'Color', fg);
end

function localLegendDot(ax, x, y, c, label)
% Draw legend items in normalized coordinates so they are independent from
% route scale and always visible.
text(ax, x, y, '●', 'Units', 'normalized', 'Color', c, 'FontSize', 18, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'Interpreter', 'none', ...
    'BackgroundColor', [1 1 1]);
text(ax, x+0.035, y, label, 'Units', 'normalized', 'VerticalAlignment', 'middle', ...
    'FontSize', 10, 'Color', [0.18 0.18 0.18], 'Interpreter', 'none', ...
    'BackgroundColor', [1 1 1]);
end

function localUpdateDashboard(fig, idx)
app = guidata(fig); idx = max(app.startIdx, min(app.endIdx, round(idx))); app.idx = idx; ui = app.ui; i = idx;
speed=app.speed_kph(i); slip=app.slip_abs_max(i); mu=app.mu_display(i); grade=app.grade_deg(i); rough=app.rough_rms(i); stateCode=app.road_state_code(i);
[roadLabel,roadBg,roadFg]=localRoadBadge(stateCode); [slipLabel,slipBg,slipFg]=localSlipBadge(slip); [muLabel,muBg,muFg]=localMuBadge(mu); [gradeLabel,gradeBg,gradeFg]=localGradeBadge(grade);
localSetCard(ui.card(1), localNum(speed,'%.0f'), roadLabel, roadBg, roadFg); localSetCard(ui.card(2), localNum(slip,'%.2f'), slipLabel, slipBg, slipFg); localSetCard(ui.card(3), localNum(mu,'%.2f'), muLabel, muBg, muFg); localSetCard(ui.card(4), localNum(grade,'%.1f'), gradeLabel, gradeBg, gradeFg);
if isgraphics(ui.headerLive), set(ui.headerLive,'String',''); end
if any(app.route_valid) && i<=numel(app.route_x) && app.route_valid(i) && isgraphics(ui.currentGps)
    set(ui.currentGps,'XData',app.route_x(i),'YData',app.route_y(i));
    set(ui.gpsInfo,'String',sprintf('Local path · X %.1f m, Y %.1f m',app.route_x(i),app.route_y(i)));
end
switch stateCode, case 5, egoFace=[0.78 0.52 0.86]; case 4, egoFace=[0.97 0.58 0.40]; case 3, egoFace=[0.58 0.80 0.94]; case 2, egoFace=[0.62 0.78 0.94]; otherwise, egoFace=[0.70 0.70 0.70]; end
% Ego vehicle color is applied when drawing the BEV vehicle below.
% ui.egoPatch is not used because ego vehicle is redrawn every update.
% BEV update using camera lane model and object list from test_plot_cam.m
if isfield(ui,'bevDyn')
    try
        delete(ui.bevDyn(isgraphics(ui.bevDyn)));
    catch
    end
end
ui.bevDyn = gobjects(0);

xTemp = 0:0.5:80;
if isfield(app,'lane') && app.lane.has && i <= numel(app.lane.c0_l)
    yl = app.lane.c0_l(i) + app.lane.c1_l(i).*xTemp + app.lane.c2_l(i).*xTemp.^2 + app.lane.c3_l(i).*xTemp.^3;
    yr = app.lane.c0_r(i) + app.lane.c1_r(i).*xTemp + app.lane.c2_r(i).*xTemp.^2 + app.lane.c3_r(i).*xTemp.^3;
    set(ui.laneLAll, 'XData', xTemp, 'YData', -yl);
    set(ui.laneRAll, 'XData', xTemp, 'YData', -yr);

    vrL = app.lane.vr_l(i); vrR = app.lane.vr_r(i);
    if isfinite(vrL) && vrL > 0
        xlv = 0:0.5:min(vrL,80);
        ylv = app.lane.c0_l(i) + app.lane.c1_l(i).*xlv + app.lane.c2_l(i).*xlv.^2 + app.lane.c3_l(i).*xlv.^3;
        set(ui.laneLValid, 'XData', xlv, 'YData', -ylv);
    else
        set(ui.laneLValid, 'XData', nan, 'YData', nan);
    end
    if isfinite(vrR) && vrR > 0
        xrv = 0:0.5:min(vrR,80);
        yrv = app.lane.c0_r(i) + app.lane.c1_r(i).*xrv + app.lane.c2_r(i).*xrv.^2 + app.lane.c3_r(i).*xrv.^3;
        set(ui.laneRValid, 'XData', xrv, 'YData', -yrv);
    else
        set(ui.laneRValid, 'XData', nan, 'YData', nan);
    end
else
    set([ui.laneLAll ui.laneRAll ui.laneLValid ui.laneRValid], 'XData', nan, 'YData', nan);
end

% ego vehicle from uploaded draw_veh.m style
ui.bevDyn(end+1:end+5) = localDrawVeh(ui.axBev, 0, 0, -pi/2, app.egoW, app.egoL, [0 0 0], 1.5, egoFace);
textH = text(ui.axBev, 0, 0, 'EGO', 'HorizontalAlignment','center', 'FontWeight','bold', 'Color',[0.05 0.15 0.35], 'FontSize',8);
ui.bevDyn(end+1) = textH;

objCnt = 0; frontDist = inf;
if isfield(app,'camObjX') && i <= size(app.camObjX,1)
    for oo = 1:size(app.camObjX,2)
        ox = app.camObjX(i,oo); oy = app.camObjY(i,oo);
        if isfinite(ox) && isfinite(oy) && ox > 0.5 && ox < 80 && abs(oy) < 10 && ~(abs(ox)<0.1 && abs(oy)<0.1)
            objCnt = objCnt + 1;
            if abs(oy) < 2.2 && ox < frontDist
                frontDist = ox;
                col = [0.90 0.20 0.20];
                label = 'CIPV';
            else
                col = [0.80 0.80 0.80];
                label = sprintf('O%d',oo);
            end
            ui.bevDyn(end+1:end+5) = localDrawVeh(ui.axBev, ox, -oy, -pi/2, app.objW, app.objL, col, 1.2, col);
            th = text(ui.axBev, ox, -oy, label, 'HorizontalAlignment','center', 'FontWeight','bold', 'Color',[0.1 0.1 0.1], 'FontSize',7);
            ui.bevDyn(end+1) = th;
        end
    end
end
if isfinite(frontDist)
    set(ui.bevInfo,'String',sprintf('CIPV %.0fm / Obj %d',frontDist,objCnt));
elseif objCnt > 0
    set(ui.bevInfo,'String',sprintf('Obj %d / CIPV 없음',objCnt));
else
    set(ui.bevInfo,'String','객체 없음');
end
app.ui = ui;
vals=[localClamp(localZero(slip)/0.35,0,1), localClamp(localZero(rough)/app.maxRough,0,1), localClamp(localZero(mu)/1.0,0,1), localClamp(abs(localZero(grade))/10,0,1)]; valText={localNum(slip,'%.2f'), localNum(rough,'%.2f'), localNum(mu,'%.2f'), [localNum(grade,'%.1f') '°']};
for q=1:4, xEnd=0.04+0.88*vals(q); set(ui.idxFg(q),'XData',[0.04 xEnd xEnd 0.04]); set(ui.idxVal(q),'String',valText{q}); end
surfaceIndex=0.35*localClamp(localZero(mu),0,1)+0.25*(1-localClamp(localZero(slip)/0.25,0,1))+0.25*(1-localClamp(localZero(rough)/app.maxRough,0,1))+0.15*(1-localClamp(abs(localZero(grade))/10,0,1));
if surfaceIndex>0.70
    idxStr = sprintf('좋음 (index %.2f)', surfaceIndex);
    idxBg = [0.86 0.92 0.80]; idxFg = [0.24 0.46 0.14];
elseif surfaceIndex>0.45
    idxStr = sprintf('양호 (index %.2f)', surfaceIndex);
    idxBg = [0.93 0.89 0.80]; idxFg = [0.45 0.33 0.10];
else
    idxStr = sprintf('주의 (index %.2f)', surfaceIndex);
    idxBg = [0.96 0.83 0.82]; idxFg = [0.60 0.18 0.18];
end
if isfield(ui,'legendIndex') && isgraphics(ui.legendIndex)
    set(ui.legendIndex, 'String', idxStr, 'BackgroundColor', idxBg, 'Color', idxFg);
end
if isfield(ui,'summaryBadge') && isgraphics(ui.summaryBadge)
    set(ui.summaryBadge, 'String', idxStr, 'BackgroundColor', idxBg, 'Color', idxFg, 'Visible', 'off');
end
for q=1:3, set(ui.trendMarker(q),'Value',app.t(i)); end
p = (i-app.startIdx)/max(app.endIdx-app.startIdx,1);
xPos = 0.08 + 0.84*p;
if isfield(ui,'timeTrackFg') && isgraphics(ui.timeTrackFg)
    set(ui.timeTrackFg, 'XData', [0.08 xPos], 'YData', [0.5 0.5]);
end
if isfield(ui,'timeDot') && isgraphics(ui.timeDot)
    set(ui.timeDot, 'XData', xPos, 'YData', 0.5);
end
set(ui.timeLeft,'String',localSecToHMS(app.t(i)-app.t(app.startIdx)));
guidata(fig,app); drawnow limitrate
end


function h = localDrawVeh(ax, xi, yi, yaw, width, length, color, line_w, faceColor)
% Draw vehicle using geometry compatible with uploaded draw_veh.m.
if nargin < 9, faceColor = color; end
w = width; l = length;
P1 = [ w/2; -l/2]; P2 = [ w/2;  l/2]; P3 = [-w/2;  l/2]; P4 = [-w/2; -l/2];
D1 = [ w/2; l/4]; D2 = [0; l/2]; D3 = [-w/2; l/4];
T = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
body = T*[P1 P2 P3 P4]; body(1,:) = body(1,:) + xi; body(2,:) = body(2,:) + yi;
dirp = T*[D1 D2 D3]; dirp(1,:) = dirp(1,:) + xi; dirp(2,:) = dirp(2,:) + yi;
h = gobjects(5,1);
h(1) = patch(ax, body(1,:), body(2,:), faceColor, 'EdgeColor', color, 'LineWidth', line_w, 'FaceAlpha', 0.65);
h(2) = plot(ax, [dirp(1,:) dirp(1,1)], [dirp(2,:) dirp(2,1)], '-', 'Color', color, 'LineWidth', max(1.2,line_w));
% tires: simple four black rectangles
for k = 1:3
    h(k+2) = plot(ax, xi, yi, '.', 'Color', color, 'MarkerSize', 1); % placeholders kept for deletion handle consistency
end
end

function localCbDataset(fig, src)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app), return; end
list = get(src, 'String');
val = get(src, 'Value');
if ischar(list)
    selectedFile = strtrim(list(val,:));
else
    selectedFile = list{val};
end
if isfield(app, 'dataFile') && strcmp(selectedFile, app.dataFile)
    return;
end

% 선택한 데이터셋을 저장한 뒤 같은 스크립트를 재실행한다.
DATA_FILE_CHOICE = selectedFile; %#ok<NASGU>
try
    save(app.choiceFile, 'DATA_FILE_CHOICE');
catch ME
    warning('데이터셋 선택 저장 실패: %s', ME.message);
end

try
    if isfield(app, 'timer') && isa(app.timer, 'timer')
        stop(app.timer);
        delete(app.timer);
    end
catch
end

scriptPath = app.scriptPath;
try
    delete(fig);
catch
end

if ~isempty(scriptPath) && isfile(scriptPath)
    run(scriptPath);
else
    warning('스크립트 경로를 찾지 못했습니다. 선택한 DATA_FILE=%s 로 스크립트를 다시 실행해 주세요.', selectedFile);
end
end

function localCbPlay(fig), app=guidata(fig); if app.isPlaying, app.isPlaying=false; try, stop(app.timer); catch, end; set(app.ui.btnPlay,'String','▶ 재생'); else, app.isPlaying=true; set(app.ui.btnPlay,'String','⏸ 정지'); try, start(app.timer); catch, end; end; guidata(fig,app); end
function localTimerStep(fig), if ~ishandle(fig), return; end; app=guidata(fig); if ~app.isPlaying, return; end; mult=app.speedChoices(app.speedChoiceIdx); nextIdx=app.idx+app.stepBase*mult; if nextIdx>app.endIdx, nextIdx=app.startIdx; end; app.idx=nextIdx; guidata(fig,app); localUpdateDashboard(fig,nextIdx); end
function localCbReset(fig), localJumpSeconds(fig,-10); end
function localCbStep(fig), localJumpSeconds(fig,10); end
function localJumpSeconds(fig, deltaSec)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app), return; end

curT = app.t(app.idx);
targetT = curT + deltaSec;

if targetT <= app.t(app.startIdx)
    newIdx = app.startIdx;
elseif targetT >= app.t(app.endIdx)
    newIdx = app.endIdx;
else
    searchRange = app.startIdx:app.endIdx;
    [~, relIdx] = min(abs(app.t(searchRange) - targetT));
    newIdx = searchRange(relIdx);
end

app.idx = newIdx;
guidata(fig, app);
localUpdateDashboard(fig, newIdx);
end
function localCbSpeed(fig), app=guidata(fig); app.speedChoiceIdx=app.speedChoiceIdx+1; if app.speedChoiceIdx>numel(app.speedChoices), app.speedChoiceIdx=1; end; set(app.ui.btnSpeed,'String',sprintf('속도 ×%d',app.speedChoices(app.speedChoiceIdx))); guidata(fig,app); end
function localCbTimeBarDown(fig, ~, ~)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app), return; end
app.isSeekingTime = true;
guidata(fig, app);
set(fig, 'WindowButtonMotionFcn', @(src,evt)localCbTimeBarMove(src, evt), ...
         'WindowButtonUpFcn', @(src,evt)localCbTimeBarUp(src, evt));
localTimeSeekFromPointer(fig);
end

function localCbTimeBarMove(fig, ~)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app) || ~app.isSeekingTime, return; end
localTimeSeekFromPointer(fig);
end

function localCbTimeBarUp(fig, ~)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app), return; end
app.isSeekingTime = false;
guidata(fig, app);
set(fig, 'WindowButtonMotionFcn', '', 'WindowButtonUpFcn', '');
end

function localTimeSeekFromPointer(fig)
if ~ishandle(fig), return; end
app = guidata(fig);
if isempty(app) || ~isstruct(app) || ~isfield(app,'ui') || ~isgraphics(app.ui.axTime), return; end
cp = get(app.ui.axTime, 'CurrentPoint');
x = cp(1,1);
p = (x - 0.08) / 0.84;
p = max(0, min(1, p));
newIdx = round(app.startIdx + p * max(app.endIdx - app.startIdx, 1));
newIdx = max(app.startIdx, min(app.endIdx, newIdx));
app.idx = newIdx;
guidata(fig, app);
localUpdateDashboard(fig, newIdx);
end
function localCbExport(fig), try, fname=sprintf('road_surface_dashboard_%s.png',datestr(now,'yyyymmdd_HHMMSS')); exportgraphics(fig,fname,'Resolution',150); fprintf('Dashboard exported: %s\n',fname); catch, fname=sprintf('road_surface_dashboard_%s.png',datestr(now,'yyyymmdd_HHMMSS')); saveas(fig,fname); fprintf('Dashboard exported: %s\n',fname); end; end
function localCloseDashboard(fig), try, app=guidata(fig); if isfield(app,'timer') && isa(app.timer,'timer'), stop(app.timer); delete(app.timer); end; catch, end; delete(fig); end
function [label,bg,fg]=localRoadBadge(code), switch code, case 3, label='정상 주행'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; case 2, label='양호 노면'; bg=[0.93 0.89 0.80]; fg=[0.45 0.33 0.10]; case 4, label='안정성 저하'; bg=[0.96 0.83 0.82]; fg=[0.60 0.18 0.18]; case 5, label='거친 노면'; bg=[0.89 0.83 0.95]; fg=[0.40 0.18 0.52]; otherwise, label='저자극 주행'; bg=[0.91 0.91 0.91]; fg=[0.35 0.35 0.35]; end; end
function [label,bg,fg]=localSlipBadge(val), if ~isfinite(val), val=0; end; if val<0.05, label='안정'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif val<0.12, label='경미한 슬립'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; else, label='슬립 주의'; bg=[0.96 0.83 0.82]; fg=[0.60 0.18 0.18]; end; end
function [label,bg,fg]=localMuBadge(val), if ~isfinite(val), val=0; end; if val>0.60, label='고접지 확인'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif val>0.35, label='중간 접지'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; else, label='판단 보류'; bg=[0.91 0.91 0.91]; fg=[0.35 0.35 0.35]; end; end
function [label,bg,fg]=localGradeBadge(val), if ~isfinite(val), val=0; end; a=abs(val); if a<2, label='평탄 도로'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif a<5, label='완만한 경사'; bg=[0.90 0.93 0.84]; fg=[0.30 0.45 0.18]; else, label='급경사 구간'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; end; end
function out=localClamp(x,lo,hi), out=min(max(x,lo),hi); end
function out=localZero(x), if ~isfinite(x), out=0; else, out=x; end; end
function s=localNum(x,fmt), if ~isfinite(x), s='-'; else, s=sprintf(fmt,x); end; end
function arr=localFinite(x), arr=x(isfinite(x)); end
function m=localNanMean(x), x=x(isfinite(x)); if isempty(x), m=NaN; else, m=mean(x); end; end
function txt=localSecToHMS(sec), if ~isfinite(sec), sec=0; end; sec=max(sec,0); h=floor(sec/3600); m=floor(mod(sec,3600)/60); s=floor(mod(sec,60)); txt=sprintf('%02d:%02d:%02d',h,m,s); end
