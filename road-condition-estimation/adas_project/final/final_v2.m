%% ========================================================================
%  Road Surface / Grip State Estimation Final Project
%  실차 CAN/IMU 데이터 + Bicycle Model 기반 노면 접지/거칠기 상태 판단
%
%  Input  : testset1.mat
%  Output : road_surface_final_project_result.mat
%
%  [경사 수정]
%  빠른 추정: AnglePitch (INS 직접 측정, 반응 빠름)
%  느린 보정: Altitude_CUR (GPS 고도 기반, 장기 drift 보정)
%  퓨전:     Complementary Filter (alpha=0.95)
%            없으면 AnglePitch 단독 사용
%
%  [거칠기 정규화 변경]
%  99% 퍼센타일 정규화 → 속도 정규화 (roughness_z / Vx)
%  - 차량 의존성을 제거하면서 단위 거리당 진동 진폭의 물리적 의미 유지
%  - 분류 임계값: roughness_norm > 0.10
%% ========================================================================

global ROAD_SURFACE_REUSE_FIG ROAD_SURFACE_REUSE_REQUEST
if isempty(ROAD_SURFACE_REUSE_REQUEST)
    ROAD_SURFACE_REUSE_REQUEST = false;
end

reuseFig = [];
if ROAD_SURFACE_REUSE_REQUEST && ~isempty(ROAD_SURFACE_REUSE_FIG) && isgraphics(ROAD_SURFACE_REUSE_FIG, 'figure')
    reuseFig = ROAD_SURFACE_REUSE_FIG;
end

clearvars -except reuseFig ROAD_SURFACE_REUSE_FIG ROAD_SURFACE_REUSE_REQUEST
global ROAD_SURFACE_REUSE_FIG ROAD_SURFACE_REUSE_REQUEST

if ~isempty(reuseFig) && isgraphics(reuseFig, 'figure')
    ROAD_SURFACE_REUSE_FIG = reuseFig;
    ROAD_SURFACE_REUSE_REQUEST = true;
else
    ROAD_SURFACE_REUSE_FIG = [];
    ROAD_SURFACE_REUSE_REQUEST = false;
    try
        oldFigs = findall(0, 'Type', 'figure');
        for kk = 1:numel(oldFigs)
            try; set(oldFigs(kk), 'CloseRequestFcn', ''); catch; end
        end
        delete(oldFigs);
    catch
    end
end
clc

%% ========================================================================
%  0. User Options
%% ========================================================================
DATASET_LIST = {'testset1.mat', 'testset3.mat'};
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
SCRIPT_NAME = mfilename;
if isempty(SCRIPT_NAME); SCRIPT_NAME = 'road_surface_final_project_v3'; end
SCRIPT_PATH = mfilename('fullpath');
if isempty(SCRIPT_PATH) || ~isfile(SCRIPT_PATH); SCRIPT_PATH = which(SCRIPT_NAME); end
if isempty(SCRIPT_PATH) || ~isfile(SCRIPT_PATH); SCRIPT_PATH = fullfile(pwd, [SCRIPT_NAME '.m']); end

RUN_DASHBOARD  = true;
ANIMATION_STEP = 100;
ANIMATION_PAUSE= 0.03;
USE_MAP_BASE   = true;

% [거칠기 임계값] 속도 정규화 후 분류용 임계값 [m/s]
% testset 분포에 따라 0.05~0.15 사이에서 조정
ROUGH_THRESHOLD = 0.05;

%% ========================================================================
%  1. Load Data
%% ========================================================================
if ~isfile(DATA_FILE); error('데이터 파일 없음: %s', DATA_FILE); end
load(DATA_FILE)

fprintf('\n============================================================\n');
fprintf(' Road Surface / Grip State Estimation Final Project\n');
fprintf('============================================================\n');
fprintf('Loaded: %s\n', DATA_FILE);

%% ========================================================================
%  2. Vehicle Parameters
%% ========================================================================
g  = 9.81;
m  = 1600;
L  = 2.8;
Wf_ratio = 0.60;
lr = Wf_ratio * L;
lf = L - lr;
h  = 0.55;
Iz = m * (0.4*L)^2;
steering_ratio = 15;
track_w = 1.635;
CdA  = 0.65;
Crr  = 0.012;
rho  = 1.225;
R_tire  = 0.32;
lambda_x= 0.5;

%% ========================================================================
%  3. Signal Selection
%% ========================================================================
t = Time(:);

% 차속 Vx [m/s]
if exist('VelForward','var')
    Vx = double(VelForward(:));
elseif exist('CF_Clu_VehicleSpeed','var')
    Vx = double(CF_Clu_VehicleSpeed(:))/3.6;
elseif exist('Speed2D','var')
    Vx = double(Speed2D(:));
else
    error('차속 없음');
end
if exist('VelForward','var') && exist('CF_Clu_VehicleSpeed','var') && mean(Vx==0)>0.3
    Vx = double(CF_Clu_VehicleSpeed(:))/3.6;
end

% Vy
if exist('VelLateral','var'); Vy=VelLateral(:); else; Vy=zeros(size(Vx)); end

% 가속도
if exist('AccelForward','var');  ax=double(AccelForward(:));
elseif exist('LONG_ACCEL','var'); ax=double(LONG_ACCEL(:));
else; error('종방향 가속도 없음'); end

if exist('AccelLateral','var');  ay=double(AccelLateral(:));
elseif exist('LAT_ACCEL','var'); ay=double(LAT_ACCEL(:));
else; error('횡방향 가속도 없음'); end

if exist('AccelDown','var');  az=double(AccelDown(:));
elseif exist('AccelZ','var'); az=double(AccelZ(:));
else; az=zeros(size(ax)); end

% 요레이트 [rad/s]
if exist('YAW_RATE','var')
    r = deg2rad(double(YAW_RATE(:)));
elseif exist('CAN_TX_YAW_RATE_CALC','var')
    r = deg2rad(double(CAN_TX_YAW_RATE_CALC(:)));
else; error('요레이트 없음'); end

% 조향각
if exist('SAS_Angle','var'); sas=SAS_Angle(:); else; sas=zeros(size(Vx)); end
delta = deg2rad(sas)/steering_ratio;

% 슬립각 / 제어 신호
if exist('AngleSlip','var'); angle_slip_deg=AngleSlip(:); else; angle_slip_deg=zeros(size(Vx)); end
if exist('TCS_CTL','var');   TCS_CTL_sig=double(TCS_CTL(:)); else; TCS_CTL_sig=zeros(size(Vx)); end
if exist('ABS_ACT','var');   ABS_ACT_sig=double(ABS_ACT(:)); else; ABS_ACT_sig=zeros(size(Vx)); end
if exist('ESP_CTL','var');   ESP_CTL_sig=double(ESP_CTL(:)); else; ESP_CTL_sig=zeros(size(Vx)); end

%% ─────────────────────────────────────────────────────────────────────
%  경사각 [1단계] AnglePitch 빠른 추정
%  - AnglePitch 우선, 없으면 Pitch_Angle
%  - 전체 median 제거 X → 정차 구간 오프셋만 제거
%% ─────────────────────────────────────────────────────────────────────
theta_pitch = [];
pitch_signal_name = 'none';

if exist('AnglePitch','var')
    pitch_raw = double(AnglePitch(:));
    pitch_signal_name = 'AnglePitch';
elseif exist('Pitch_Angle','var')
    pitch_raw = double(Pitch_Angle(:));
    pitch_signal_name = 'Pitch_Angle';
else
    pitch_raw = [];
end

if ~isempty(pitch_raw)
    finite_pitch = isfinite(pitch_raw);
    if any(finite_pitch)
        if max(abs(pitch_raw(finite_pitch))) < 1.5
            theta_pitch = pitch_raw;           % rad
        else
            theta_pitch = deg2rad(pitch_raw);  % deg → rad
        end
        % 전체 중앙값 제거 (롤백: 상대 경사 변화량만 표현)
        install_offset = median(theta_pitch(isfinite(theta_pitch)), 'omitnan');
        if ~isfinite(install_offset); install_offset = 0; end
        theta_pitch = theta_pitch - install_offset;
        fprintf('[경사] %s 로드 완료 (전체 median %.4f rad 제거)\n', ...
            pitch_signal_name, install_offset);
    end
end

% AnglePitch 없으면 zeros → GPS 보정 후 덮어씌워짐
if ~isempty(theta_pitch)
    theta = theta_pitch;
    grade_source = pitch_signal_name;
else
    theta = zeros(size(Vx));
    grade_source = 'zero';
    fprintf('[경사] pitch 신호 없음 → 0 초기화\n');
end

%% ─────────────────────────────────────────────────────────────────────
%  경사각 [2단계] GPS 고도 준비 (Altitude_CUR)
%  - dt가 Section 5에서 계산되므로 여기서는 raw 신호만 저장
%  - 실제 퓨전은 Section 5.5에서 수행
%% ─────────────────────────────────────────────────────────────────────
alt_raw = [];
has_altitude = false;

if exist('Altitude_CUR','var')
    alt_tmp = double(Altitude_CUR(:));
    alt_valid_mask = isfinite(alt_tmp) & alt_tmp ~= 0;
    if sum(alt_valid_mask) > 100
        alt_raw = alt_tmp;
        has_altitude = true;
        fprintf('[경사] Altitude_CUR 유효: %.1f%%\n', ...
            sum(alt_valid_mask)/numel(alt_tmp)*100);
    else
        fprintf('[경사] Altitude_CUR 유효값 부족 → GPS 보정 생략\n');
    end
end

% Roll/Bank
if exist('AngleRoll','var'); roll_meas_deg=AngleRoll(:); else; roll_meas_deg=zeros(size(Vx)); end

% 휠속도 [kph]
if exist('WHL_SPD_FL','var') && exist('WHL_SPD_FR','var') && ...
   exist('WHL_SPD_RL','var') && exist('WHL_SPD_RR','var')
    whl_FL=WHL_SPD_FL(:); whl_FR=WHL_SPD_FR(:);
    whl_RL=WHL_SPD_RL(:); whl_RR=WHL_SPD_RR(:);
else; error('휠속도 없음'); end

% GPS
if exist('PosLat','var') && exist('PosLon','var')
    pos_lat=PosLat(:); pos_lon=PosLon(:);
else
    pos_lat=zeros(size(t)); pos_lon=zeros(size(t));
end

%% ========================================================================
%  4. Length Matching
%% ========================================================================
N = min([length(t),length(Vx),length(Vy),length(ax),length(ay), ...
         length(az),length(r),length(delta),length(angle_slip_deg), ...
         length(TCS_CTL_sig),length(ABS_ACT_sig),length(ESP_CTL_sig), ...
         length(theta),length(roll_meas_deg), ...
         length(whl_FL),length(whl_FR),length(whl_RL),length(whl_RR)]);

tr=@(v)v(1:N);
t=tr(t); Vx=tr(Vx); Vy=tr(Vy); ax=tr(ax); ay=tr(ay); az=tr(az);
r=tr(r); delta=tr(delta); sas=tr(sas);
angle_slip_deg=tr(angle_slip_deg);
TCS_CTL_sig=tr(TCS_CTL_sig); ABS_ACT_sig=tr(ABS_ACT_sig); ESP_CTL_sig=tr(ESP_CTL_sig);
theta=tr(theta); roll_meas_deg=tr(roll_meas_deg);
whl_FL=tr(whl_FL); whl_FR=tr(whl_FR); whl_RL=tr(whl_RL); whl_RR=tr(whl_RR);

if length(pos_lat)>=N && length(pos_lon)>=N
    pos_lat=pos_lat(1:N); pos_lon=pos_lon(1:N);
else
    pos_lat=zeros(N,1); pos_lon=zeros(N,1);
end

if has_altitude && length(alt_raw) >= N
    alt_raw = alt_raw(1:N);
elseif has_altitude
    % 길이 부족 시 보간으로 맞춤
    alt_tmp2 = nan(N,1);
    M_alt = length(alt_raw);
    alt_tmp2(1:M_alt) = alt_raw;
    alt_raw = alt_tmp2;
end

%% ========================================================================
%  5. Basic Preprocessing
%% ========================================================================
if length(t)<2; error('Time 너무 짧음'); end
dt = mean(diff(t),'omitnan');
if isnan(dt)||dt<=0; dt=0.01; end
Fs = 1/dt;

win_smooth=21;
Vx_s  = movmedian(Vx, win_smooth,'omitnan');
Vy_s  = movmedian(Vy, win_smooth,'omitnan');
ax_s  = movmedian(ax, win_smooth,'omitnan');
ay_s  = movmedian(ay, win_smooth,'omitnan');
az_s  = movmedian(az, win_smooth,'omitnan');
r_s   = movmedian(r,  win_smooth,'omitnan');
delta_s = movmedian(delta, win_smooth,'omitnan');
sas_s   = movmedian(sas,   win_smooth,'omitnan');
angle_slip_s_deg = movmedian(angle_slip_deg, win_smooth,'omitnan');
TCS_CTL_s = TCS_CTL_sig>0;
ABS_ACT_s = ABS_ACT_sig>0;
ESP_CTL_s = ESP_CTL_sig>0;
theta_s   = movmedian(theta, win_smooth,'omitnan');
roll_s_deg= movmedian(roll_meas_deg, win_smooth,'omitnan');

valid_speed = Vx_s > 5;

%% ========================================================================
%  5.5 경사각 [2단계+3단계] GPS 고도 보정 및 Complementary Filter 퓨전
%  - dt가 확정된 뒤 GPS 고도 미분 → theta_gps 계산
%  - AnglePitch + GPS고도 → Complementary Filter
%  - GPS고도 없으면 AnglePitch 단독 사용
%% ========================================================================
if has_altitude && ~isempty(alt_raw)

    %--- GPS 고도 강하게 스무딩 (노이즈 억제) ---
    % 201샘플 ≈ 2초 윈도우 (100Hz 기준)
    alt_smooth = movmedian(double(alt_raw), 201, 'omitnan');

    % NaN 보간 (GPS dropout 구간 처리)
    alt_valid_idx = find(isfinite(alt_smooth) & alt_smooth ~= 0);
    if numel(alt_valid_idx) > 10
        alt_interp = alt_smooth;
        nan_alt = isnan(alt_smooth) | alt_smooth == 0;
        alt_interp(nan_alt) = interp1(alt_valid_idx, ...
            alt_smooth(alt_valid_idx), find(nan_alt), 'linear', 'extrap');
    else
        alt_interp = alt_smooth;
    end

    %--- 고도 미분 → GPS 기반 경사각 [rad] ---
    % dAlt/dDist = dAlt/dt ÷ Vx
    dAlt = gradient(alt_interp, dt);
    Vx_clamped = max(abs(Vx_s), 0.5);
    theta_gps_raw = atan2(dAlt, Vx_clamped);

    % GPS 경사는 매우 노이즈 → 강하게 스무딩
    % 501샘플 ≈ 5초 윈도우
    theta_gps = movmedian(theta_gps_raw, 501, 'omitnan');

    %--- Complementary Filter 퓨전 ---
    % alpha 높을수록 AnglePitch 비중 크고 반응 빠름
    % alpha 낮을수록 GPS 비중 크고 장기 drift 보정
    alpha_cf = 0.95;

    if ~isempty(theta_pitch)
        % AnglePitch + GPS고도 퓨전
        theta_fused = alpha_cf * theta_s + (1-alpha_cf) * theta_gps;
        theta_s = movmedian(theta_fused, win_smooth, 'omitnan');
        grade_source = sprintf('%s+Altitude_CF(%.2f)', pitch_signal_name, alpha_cf);
        fprintf('[경사] Complementary Filter 퓨전: %s\n', grade_source);
    else
        % AnglePitch 없음 → GPS고도만으로 경사 추정
        theta_s = movmedian(theta_gps, win_smooth, 'omitnan');
        grade_source = 'Altitude_CUR_only';
        fprintf('[경사] GPS 고도 단독 사용 (AnglePitch 없음)\n');
    end

    fprintf('[경사] theta_s 범위: %.2f ~ %.2f deg\n', ...
        min(rad2deg(theta_s)), max(rad2deg(theta_s)));

else
    % GPS 고도 없음 → AnglePitch 단독 (theta_s는 이미 설정됨)
    if ~isempty(theta_pitch)
        fprintf('[경사] %s 단독 사용 (Altitude_CUR 없음)\n', pitch_signal_name);
    else
        fprintf('[경사] 모든 경사 신호 없음 → 0\n');
    end
    fprintf('[경사] theta_s 범위: %.2f ~ %.2f deg\n', ...
        min(rad2deg(theta_s)), max(rad2deg(theta_s)));
end

%% ========================================================================
%  6. Road Bank Angle Estimation
%% ========================================================================
straight_idx = Vx_s>5 & abs(r_s)<deg2rad(1.0) & abs(ay_s)<0.2 & abs(sas_s)<5 & ~isnan(roll_s_deg);
if sum(straight_idx)<100
    straight_idx = Vx_s>3 & abs(r_s)<deg2rad(2.0) & abs(ay_s)<0.4 & abs(sas_s)<10 & ~isnan(roll_s_deg);
end
if sum(straight_idx)<10
    roll_offset_deg=mean(roll_s_deg,'omitnan');
else
    roll_offset_deg=mean(roll_s_deg(straight_idx),'omitnan');
end
if isnan(roll_offset_deg); roll_offset_deg=0; end

roll_zeroed_deg = roll_s_deg - roll_offset_deg;
K_roll_deg_per_g = 3.0;
vehicle_roll_deg = K_roll_deg_per_g*(ay_s/g);
bank_roll_deg = roll_zeroed_deg - vehicle_roll_deg;

bank_arg = (ay_s - Vx_s.*r_s)/g;
bank_arg = min(max(bank_arg,-1),1);
bank_dyn_deg = rad2deg(asin(bank_arg));

bank_fused_deg = 0.5*bank_roll_deg + 0.5*bank_dyn_deg;
bank_diff = abs(bank_roll_deg - bank_dyn_deg);

valid_bank = Vx_s>8 & abs(sas_s)<45 & bank_diff<5 & ~isnan(bank_fused_deg);
bank_final_deg = bank_fused_deg;
bank_final_deg(~valid_bank) = NaN;
bank_final_deg = movmedian(bank_final_deg,101,'omitnan');

phi = deg2rad(bank_final_deg);
phi(isnan(phi)) = 0;

%% ========================================================================
%  7. Grade / Bank Acceleration Correction (3D)
%% ========================================================================
ax_gravity =  g.*sin(theta_s);
ay_gravity =  g.*sin(phi).*cos(theta_s);
az_gravity = -g.*cos(theta_s).*cos(phi);

ax_corr = ax_s - ax_gravity;
ay_corr = ay_s - ay_gravity;
az_corr = az_s - az_gravity;

%% ========================================================================
%  8. Wheel Slip Ratio
%% ========================================================================
if exist('Speed2D','var')
    Vveh_kph=Speed2D(:)*3.6; Vveh_kph=Vveh_kph(1:N);
else
    Vveh_kph=Vx_s*3.6;
end

steady = valid_speed & abs(sas_s)<deg2rad(5) & abs(ax_corr)<0.2;
whl_avg = mean([whl_FL,whl_FR,whl_RL,whl_RR],2,'omitnan');
bias_kph = mean(whl_avg(steady)-Vveh_kph(steady),'omitnan');
Vveh_kph = Vveh_kph + bias_kph;

invalid_vveh = ~isfinite(Vveh_kph)|Vveh_kph<5;
Vveh_kph(invalid_vveh) = Vx_s(invalid_vveh)*3.6;

slip_FL=(whl_FL-Vveh_kph)./max(max(whl_FL,Vveh_kph),1); slip_FL(~valid_speed)=NaN;
slip_FR=(whl_FR-Vveh_kph)./max(max(whl_FR,Vveh_kph),1); slip_FR(~valid_speed)=NaN;
slip_RL=(whl_RL-Vveh_kph)./max(max(whl_RL,Vveh_kph),1); slip_RL(~valid_speed)=NaN;
slip_RR=(whl_RR-Vveh_kph)./max(max(whl_RR,Vveh_kph),1); slip_RR(~valid_speed)=NaN;

slip_front=mean([slip_FL,slip_FR],2,'omitnan');
slip_rear =mean([slip_RL,slip_RR],2,'omitnan');
slip_abs_each=abs([slip_FL,slip_FR,slip_RL,slip_RR]);
slip_abs_max=max(slip_abs_each,[],2);
slip_abs_med=median(slip_abs_each,2,'omitnan');
slip_abs_med(slip_abs_med>0.50)=NaN;
slip_abs_max(slip_abs_max>0.80)=NaN;
slip_index=slip_abs_med;

%% ========================================================================
%  9. Tire Slip Angle (Bicycle Model)
%% ========================================================================
Vx_safe=max(Vx_s,0.1);
alpha_f=delta_s-atan2(Vy_s+lf.*r_s,Vx_safe); alpha_f(~valid_speed)=NaN;
alpha_r=-atan2(Vy_s-lr.*r_s,Vx_safe);         alpha_r(~valid_speed)=NaN;
alpha_f_deg=rad2deg(alpha_f); alpha_r_deg=rad2deg(alpha_r);
slip_angle_max=max(abs([alpha_f,alpha_r]),[],2);
slip_angle_diff=abs(alpha_f-alpha_r);

%% ========================================================================
%  10. Bicycle Model Tire Force
%% ========================================================================
r_dot=gradient(r_s,dt);
Fy_f=(lr*m.*ay_corr+Iz.*r_dot)/L;
Fy_r=(lf*m.*ay_corr-Iz.*r_dot)/L;
Fx_total=m.*ax_corr;

lambda_x_vec=zeros(N,1);
lambda_x_vec(ax_corr>=0)=0.90;
lambda_x_vec(ax_corr< 0)=0.65;
Fx_f=lambda_x_vec.*Fx_total;
Fx_r=(1-lambda_x_vec).*Fx_total;

%% ========================================================================
%  11. Vertical Load
%% ========================================================================
Fz_f=m*g*lr/L-m.*ax_corr*h/L; Fz_f(Fz_f<=0)=NaN;
Fz_r=m*g*lf/L+m.*ax_corr*h/L; Fz_r(Fz_r<=0)=NaN;

%% ========================================================================
%  12. Required Friction Coefficient + mu_road
%% ========================================================================
mu_f=sqrt(Fx_f.^2+Fy_f.^2)./Fz_f;
mu_r=sqrt(Fx_r.^2+Fy_r.^2)./Fz_r;
mu_required=max([mu_f,mu_r],[],2);
mu_f(~valid_speed)=NaN; mu_r(~valid_speed)=NaN; mu_required(~valid_speed)=NaN;
mu_demand=sqrt(ax_corr.^2+ay_corr.^2)/g; mu_demand(~valid_speed)=NaN;

accel_mag=sqrt(ax_corr.^2+ay_corr.^2);
confidence=(TCS_CTL_s|ABS_ACT_s|ESP_CTL_s|accel_mag>1.5|slip_index>0.05|slip_angle_max>deg2rad(3))&valid_speed;

mu_candidate=nan(N,1);
mu_candidate(confidence&isfinite(mu_required))=mu_required(confidence&isfinite(mu_required));

window_mu=max(1,round(5/dt));
mu_road=zeros(N,1); mu_road(1)=0.8;
for i=2:N
    seg=mu_candidate(max(1,i-window_mu):i); seg=seg(isfinite(seg));
    if ~isempty(seg); mu_road(i)=max(seg); else; mu_road(i)=mu_road(i-1); end
end
mu_road=min(max(mu_road,0.2),1.2); mu_road(~valid_speed)=NaN;

%% ========================================================================
%  13. Equivalent Torque
%% ========================================================================
F_aero=0.5*rho*CdA.*Vx_s.^2;
F_roll=Crr*m*g*ones(size(Vx_s));
F_grade=m*g.*sin(theta_s);
Fx_resist=m.*ax_s+F_aero+F_roll+F_grade;
T_wheel_equiv=Fx_resist.*R_tire;
T_drive_equiv=max(T_wheel_equiv,0);
T_brake_equiv=max(-T_wheel_equiv,0);

%% ========================================================================
%  14. IMU Roughness Index (3D 보정 az_corr + BPF RMS + 속도 정규화)
%
%  변경 사항:
%  - 99% 퍼센타일 정규화 → 속도 정규화로 변경
%  - 단위 거리당 진동 진폭 [m/s]의 물리적 의미 유지
%  - 차량/센서 의존성은 BPF 대역 선택과 RMS 윈도우로 흡수
%% ========================================================================
fs=1/dt; win_rms=max(1,round(1/dt));
Wn_bp=min([0.5 10]/(fs/2),[0.99 0.99]);
try
    [b_bp,a_bp]=butter(4,Wn_bp,'bandpass'); bpf_ok=true;
catch
    warning('BPF 실패 → HPF 대체'); bpf_ok=false; win_hp=max(5,round(0.5/dt));
end

az_interp=az_corr; nan_idx_z=isnan(az_corr);
if any(~nan_idx_z)
    az_interp(nan_idx_z)=interp1(find(~nan_idx_z),az_corr(~nan_idx_z),find(nan_idx_z),'linear','extrap');
end
if bpf_ok; az_filt=filtfilt(b_bp,a_bp,az_interp);
else; az_filt=az_interp-movmean(az_interp,win_hp,'omitnan'); end
az_filt(nan_idx_z)=NaN;
roughness_z=sqrt(movmean(az_filt.^2,win_rms,'omitnan'));

ax_interp=ax_corr; ax_interp(isnan(ax_corr))=0;
ay_interp=ay_corr; ay_interp(isnan(ay_corr))=0;
if bpf_ok; ax_filt=filtfilt(b_bp,a_bp,ax_interp); ay_filt=filtfilt(b_bp,a_bp,ay_interp);
else; ax_filt=ax_interp-movmean(ax_interp,win_hp,'omitnan'); ay_filt=ay_interp-movmean(ay_interp,win_hp,'omitnan'); end
ax_filt(isnan(ax_corr))=NaN; ay_filt(isnan(ay_corr))=NaN;
roughness_xy=sqrt(movmean(ax_filt.^2,win_rms,'omitnan'))+sqrt(movmean(ay_filt.^2,win_rms,'omitnan'));

% [핵심 변경] 속도 정규화: roughness_z [m/s²] / Vx [m/s] = [m/s]
% 단위 거리당 진동 진폭 → 노면 자체의 거칠기 표현
roughness_norm = roughness_z ./ max(Vx_s, 1);

% vib_index는 호환성을 위해 roughness_norm을 가리키도록 alias 처리
% 분류 임계값은 ROUGH_THRESHOLD (기본 0.10) 사용
vib_index = roughness_norm;

roughness_z(~valid_speed)=NaN; roughness_xy(~valid_speed)=NaN;
roughness_norm(~valid_speed)=NaN; vib_index(~valid_speed)=NaN;

% 거칠기 분포 출력 (임계값 튜닝 참고용)
if any(isfinite(roughness_norm))
    rn_valid = roughness_norm(isfinite(roughness_norm));
    fprintf('[거칠기] roughness_norm 분포: median=%.3f, p75=%.3f, p90=%.3f, p95=%.3f, max=%.3f [m/s]\n', ...
        median(rn_valid), prctile(rn_valid,75), prctile(rn_valid,90), ...
        prctile(rn_valid,95), max(rn_valid));
    fprintf('[거칠기] 임계값 %.3f 이상 비율: %.2f%%\n', ...
        ROUGH_THRESHOLD, sum(rn_valid > ROUGH_THRESHOLD)/numel(rn_valid)*100);
end

%% ========================================================================
%  15. Lateral Consistency Index
%% ========================================================================
ay_pred=Vx_s.*r_s;
lat_error=abs(ay_corr-ay_pred); lat_error(~valid_speed)=NaN;

%% ========================================================================
%  16. Control Intervention Suspicion
%% ========================================================================
ABS_suspect=ax_corr<-3.0 & slip_abs_max>0.08;
TCS_suspect=ax_corr> 1.5 & slip_abs_max>0.08;
ESC_suspect=lat_error>1.2 & abs(rad2deg(r_s))>5;

%% ========================================================================
%  17. 5단계 노면 분류
%
%  변경 사항: vib_index > 0.60 → roughness_norm > ROUGH_THRESHOLD (기본 0.10)
%% ========================================================================
road_state=strings(N,1); road_state_code=zeros(N,1);
for i=1:N
    if ~valid_speed(i)||isnan(mu_required(i))
        road_state(i)="INVALID_LOW_SPEED"; road_state_code(i)=0;
    elseif roughness_norm(i) > ROUGH_THRESHOLD
        road_state(i)="ROUGH_SURFACE"; road_state_code(i)=5;
    elseif slip_index(i)>0.08||slip_angle_max(i)>deg2rad(6)||lat_error(i)>1.2||ESC_suspect(i)
        road_state(i)="LOW_STABILITY_SUSPECTED"; road_state_code(i)=4;
    elseif confidence(i)&&mu_required(i)>0.55
        road_state(i)="HIGH_GRIP_CONFIRMED"; road_state_code(i)=3;
    elseif confidence(i)&&mu_required(i)>0.35
        road_state(i)="MEDIUM_GRIP_CONFIRMED"; road_state_code(i)=2;
    else
        road_state(i)="LOW_EXCITATION_UNKNOWN"; road_state_code(i)=1;
    end
end

%% ========================================================================
%  18. Summary
%% ========================================================================
valid_idx=road_state~="INVALID_LOW_SPEED";
fprintf('\n[Vehicle Parameters]\n');
fprintf('m=%.0fkg  L=%.2fm  lf=%.2fm  lr=%.2fm  h=%.2fm\n',m,L,lf,lr,h);
fprintf('Iz=%.0f  steering_ratio=%.0f  roll_offset=%.3fdeg\n',Iz,steering_ratio,roll_offset_deg);
fprintf('\n[Result Summary]\n');
fprintf('Total=%d  Valid=%d  Grade: %s\n',N,sum(valid_idx),grade_source);
fprintf('Rough threshold (roughness_norm) = %.3f [m/s]\n', ROUGH_THRESHOLD);
if any(valid_idx)
    fprintf('mu_required: mean=%.3f  max=%.3f\n',mean(mu_required(valid_idx),'omitnan'),max(mu_required(valid_idx),[],'omitnan'));
    fprintf('slip_index:  max=%.3f\n',max(slip_index(valid_idx),[],'omitnan'));
    fprintf('bank_angle:  max=%.3f deg\n',max(abs(bank_final_deg(valid_idx)),[],'omitnan'));
    fprintf('grade(theta):max=%.2f deg\n',max(abs(rad2deg(theta_s)),[],'omitnan'));
    fprintf('roughness_norm: max=%.3f [m/s]\n',max(roughness_norm(valid_idx),[],'omitnan'));
end
fprintf('\n[Road State Ratio]\n');
for s=["LOW_EXCITATION_UNKNOWN","MEDIUM_GRIP_CONFIRMED","HIGH_GRIP_CONFIRMED","LOW_STABILITY_SUSPECTED","ROUGH_SURFACE"]
    fprintf('%-30s: %.2f%%\n',s,sum(road_state==s)/max(sum(valid_idx),1)*100);
end
fprintf('============================================================\n');

%% ========================================================================
%  19. Save Results
%% ========================================================================
result.Time=t;
result.VehicleParameter=struct('m',m,'L',L,'lf',lf,'lr',lr,'h',h,'Iz',Iz,...
    'steering_ratio',steering_ratio,'CdA',CdA,'Crr',Crr,'R_tire',R_tire,'lambda_x',lambda_x);
result.grade_source=grade_source;
result.has_altitude_gps=has_altitude;
result.rough_threshold=ROUGH_THRESHOLD;
result.Vx=Vx_s; result.Vy=Vy_s;
result.ax_raw=ax_s; result.ay_raw=ay_s; result.az=az_s;
result.yaw_rate_rad=r_s; result.SAS_Angle_deg=sas_s;
result.theta_grade_rad=theta_s;
result.roll_offset_deg=roll_offset_deg; result.roll_zeroed_deg=roll_zeroed_deg;
result.vehicle_roll_deg=vehicle_roll_deg; result.bank_roll_deg=bank_roll_deg;
result.bank_dyn_deg=bank_dyn_deg; result.bank_final_deg=bank_final_deg;
result.valid_bank=valid_bank;
result.ax_corr=ax_corr; result.ay_corr=ay_corr; result.az_corr=az_corr;
result.slip_FL=slip_FL; result.slip_FR=slip_FR; result.slip_RL=slip_RL; result.slip_RR=slip_RR;
result.slip_front=slip_front; result.slip_rear=slip_rear;
result.slip_abs_max=slip_abs_max; result.slip_abs_med=slip_abs_med;
result.slip_index=slip_index; result.angle_slip_deg=angle_slip_s_deg;
result.alpha_f_deg=alpha_f_deg; result.alpha_r_deg=alpha_r_deg;
result.slip_angle_max_deg=rad2deg(slip_angle_max);
result.Fx_f=Fx_f; result.Fx_r=Fx_r; result.Fy_f=Fy_f; result.Fy_r=Fy_r;
result.Fz_f=Fz_f; result.Fz_r=Fz_r;
result.mu_f=mu_f; result.mu_r=mu_r; result.mu_required=mu_required;
result.mu_demand=mu_demand; result.mu_display=mu_required;
result.confidence=confidence; result.mu_candidate=mu_candidate; result.mu_road=mu_road;
result.roughness_z=roughness_z; result.roughness_xy=roughness_xy;
result.roughness_norm=roughness_norm; result.vib_index=vib_index;
result.az_filt=az_filt; result.lat_error=lat_error;
result.T_drive_equiv=T_drive_equiv; result.T_brake_equiv=T_brake_equiv;
result.ABS_suspect=ABS_suspect; result.TCS_suspect=TCS_suspect; result.ESC_suspect=ESC_suspect;
result.road_state=road_state; result.road_state_code=road_state_code;
result.PosLat=pos_lat; result.PosLon=pos_lon;
save(SAVE_FILE,'result');
fprintf('\nSaved: %s\n',SAVE_FILE);

%% ========================================================================
%  20. Dashboard (document 6 동일)
%% ========================================================================
if ~RUN_DASHBOARD; return; end

speed_kph=Vx_s*3.6; grade_deg=rad2deg(theta_s);
slip_angle_max_deg=rad2deg(slip_angle_max);
rough_rms=roughness_z;            % 절대 RMS (참고용 저장)
rough_display=roughness_norm;      % 대시보드 표시용 (분류 기준과 동일)
mu_display=mu_required;

has_front_obj=false(N,1); front_obj_x=nan(N,1); front_obj_y=nan(N,1);
if exist('SOW_x_pos_01','var')&&exist('SOW_y_pos_cor_01','var')
    tmpx=SOW_x_pos_01(:); tmpy=SOW_y_pos_cor_01(:);
    Mobj=min([N,length(tmpx),length(tmpy)]);
    front_obj_x(1:Mobj)=tmpx(1:Mobj); front_obj_y(1:Mobj)=tmpy(1:Mobj);
    has_front_obj=isfinite(front_obj_x)&front_obj_x>0&front_obj_x<80&abs(front_obj_y)<3.5;
elseif exist('PosX01','var')&&exist('PosY01','var')
    tmpx=PosX01(:); tmpy=PosY01(:);
    Mobj=min([N,length(tmpx),length(tmpy)]);
    front_obj_x(1:Mobj)=tmpx(1:Mobj); front_obj_y(1:Mobj)=tmpy(1:Mobj);
    has_front_obj=isfinite(front_obj_x)&front_obj_x>0&front_obj_x<80&abs(front_obj_y)<3.5;
end

has_side_left=false(N,1); has_side_right=false(N,1);
side_left_x=nan(N,1); side_right_x=nan(N,1); side_left_y=nan(N,1); side_right_y=nan(N,1);
for objNo=1:8
    xN1=sprintf('SOW_x_pos_%02d',objNo); yN1=sprintf('SOW_y_pos_cor_%02d',objNo);
    xN2=sprintf('PosX%02d',objNo);       yN2=sprintf('PosY%02d',objNo);
    if exist(xN1,'var')&&exist(yN1,'var'); ox=eval([xN1 '(:)']); oy=eval([yN1 '(:)']);
    elseif exist(xN2,'var')&&exist(yN2,'var'); ox=eval([xN2 '(:)']); oy=eval([yN2 '(:)']);
    else; continue; end
    Mobj=min([N,length(ox),length(oy)]); ox=ox(1:Mobj); oy=oy(1:Mobj);
    Lc=isfinite(ox)&isfinite(oy)&ox>-15&ox<35&oy>2&oy<6;
    Rc=isfinite(ox)&isfinite(oy)&ox>-15&ox<35&oy<-2&oy>-6;
    iL=find(Lc&~has_side_left(1:Mobj)); iR=find(Rc&~has_side_right(1:Mobj));
    has_side_left(iL)=true; side_left_x(iL)=ox(iL); side_left_y(iL)=oy(iL);
    has_side_right(iR)=true; side_right_x(iR)=ox(iR); side_right_y(iR)=oy(iR);
end

frame_valid=valid_speed(:)&isfinite(speed_kph(:));
frame_valid_mu=frame_valid&isfinite(mu_display(:))&isfinite(slip_index(:));
start_idx=find(frame_valid_mu,1,'first');
if isempty(start_idx); start_idx=find(frame_valid,1,'first'); end
if isempty(start_idx); start_idx=1; end
end_idx=N;

gps_valid=pos_lat~=0&pos_lon~=0&isfinite(pos_lat)&isfinite(pos_lon);
route_valid=gps_valid(:); route_x=nan(N,1); route_y=nan(N,1);
if any(route_valid)
    lat0=median(pos_lat(route_valid),'omitnan'); lon0=median(pos_lon(route_valid),'omitnan');
    route_x(route_valid)=(pos_lon(route_valid)-lon0).*111320.*cosd(lat0);
    route_y(route_valid)=(pos_lat(route_valid)-lat0).*110540;
else
    route_valid=true(N,1); route_x=zeros(N,1); route_y=cumtrapz(t,Vx_s);
end

global ROAD_SURFACE_REUSE_FIG ROAD_SURFACE_REUSE_REQUEST
if ROAD_SURFACE_REUSE_REQUEST&&~isempty(ROAD_SURFACE_REUSE_FIG)&&isgraphics(ROAD_SURFACE_REUSE_FIG,'figure')
    fig=ROAD_SURFACE_REUSE_FIG;
    try; oldApp=guidata(fig); if isstruct(oldApp)&&isfield(oldApp,'timer')&&isa(oldApp.timer,'timer'); stop(oldApp.timer); delete(oldApp.timer); end; catch; end
    try; set(fig,'CloseRequestFcn',''); clf(fig,'reset'); catch; end
    set(fig,'Name','Road Surface Dashboard','NumberTitle','off','Color',[0.965 0.965 0.955],'Position',[60 60 1450 820],'MenuBar','none','Visible','on');
else
    fig=figure('Name','Road Surface Dashboard','NumberTitle','off','Color',[0.965 0.965 0.955],'Position',[60 60 1450 820],'MenuBar','none');
end
ROAD_SURFACE_REUSE_FIG=fig; ROAD_SURFACE_REUSE_REQUEST=false;

ui.headerAx=axes('Parent',fig,'Units','normalized','Position',[0.02 0.92 0.96 0.07]);
localPanelBase(ui.headerAx,[0.98 0.98 0.98]);
text(ui.headerAx,0.025,0.58,'주행 데이터 분석 대시보드','Units','normalized','FontSize',18,'FontWeight','bold','Interpreter','none','Color',[0.08 0.08 0.08]);
text(ui.headerAx,0.025,0.26,sprintf('Road Surface Project · %s · %.0f samples · %s',datestr(now,'yyyy-mm-dd'),N,grade_source),'Units','normalized','FontSize',10,'Interpreter','none','Color',[0.38 0.38 0.38]);
ui.headerLive=text(ui.headerAx,0.37,0.22,'','Units','normalized','FontSize',10,'FontWeight','bold','Interpreter','none','Color',[0.20 0.20 0.20],'Visible','off');

datasetValue=find(strcmp(DATA_FILE,DATASET_LIST),1); if isempty(datasetValue); datasetValue=1; end
ui.datasetPopup=uicontrol(fig,'Style','popupmenu','Units','normalized','Position',[0.34 0.925 0.105 0.045],'String',DATASET_LIST,'Value',datasetValue,'FontSize',10.5,'BackgroundColor',[0.98 0.98 0.98],'Callback',@(src,~)localCbDataset(fig,src));
ui.btnReset =uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.46 0.925 0.08 0.045],'String','◀◀','FontSize',12,'Callback',@(~,~)localCbReset(fig));
ui.btnPlay  =uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.555 0.925 0.09 0.045],'String','▶ 재생','FontSize',11,'Callback',@(~,~)localCbPlay(fig));
ui.btnStep  =uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.655 0.925 0.08 0.045],'String','▶▶','FontSize',12,'Callback',@(~,~)localCbStep(fig));
ui.btnSpeed =uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.77 0.925 0.095 0.045],'String','속도 ×1','FontSize',11,'Callback',@(~,~)localCbSpeed(fig));
ui.btnExport=uicontrol(fig,'Style','pushbutton','Units','normalized','Position',[0.875 0.925 0.10 0.045],'String','내보내기','FontSize',11,'Callback',@(~,~)localCbExport(fig));

ui.card(1)=localCreateCard(fig,[0.02 0.75 0.23 0.12],'차속','kph');
ui.card(2)=localCreateCard(fig,[0.265 0.75 0.23 0.12],'슬립 지수','');
ui.card(3)=localCreateCard(fig,[0.51 0.75 0.23 0.12],'요구 마찰계수 μ','');
ui.card(4)=localCreateCard(fig,[0.755 0.75 0.225 0.12],'도로 경사','deg');

for p_={[0.02 0.92 0.96 0.07],[0.02 0.75 0.23 0.12],[0.265 0.75 0.23 0.12],[0.51 0.75 0.23 0.12],[0.755 0.75 0.225 0.12]}
    localAddBorder(fig,p_{1});
end

ui.axGps=axes('Parent',fig,'Units','normalized','Position',[0.04 0.265 0.56 0.40]);
localPanelBase(ui.axGps,[0.99 0.99 0.985]);
title(ui.axGps,'주행 경로 + 노면 상태','FontSize',12,'FontWeight','bold');
hold(ui.axGps,'on'); axis(ui.axGps,'equal'); axis(ui.axGps,'off');
if any(route_valid)
    plot(ui.axGps,route_x(route_valid),route_y(route_valid),'-','Color',[0.72 0.72 0.70],'LineWidth',2);
    scatter(ui.axGps,route_x(route_valid&road_state_code==3),route_y(route_valid&road_state_code==3),34,[0.40 0.65 0.15],'filled');
    scatter(ui.axGps,route_x(route_valid&(road_state_code==1|road_state_code==2)),route_y(route_valid&(road_state_code==1|road_state_code==2)),34,[0.79 0.50 0.10],'filled');
    scatter(ui.axGps,route_x(route_valid&(road_state_code==4|road_state_code==5)),route_y(route_valid&(road_state_code==4|road_state_code==5)),34,[0.90 0.30 0.30],'filled');
    vS=start_idx; if ~route_valid(vS); tmp=find(route_valid,1,'first'); if ~isempty(tmp); vS=tmp; end; end
    ui.currentGps=scatter(ui.axGps,route_x(vS),route_y(vS),95,'o','MarkerFaceColor',[0.20 0.52 0.90],'MarkerEdgeColor',[1 1 1],'LineWidth',1.4);
    rx=route_x(route_valid); ry=route_y(route_valid); rx=rx(isfinite(rx)); ry=ry(isfinite(ry));
    if ~isempty(rx)
        xR=max(max(rx)-min(rx),eps); yR=max(max(ry)-min(ry),eps);
        xp=max(20,xR*0.08); yp=max(20,yR*0.08);
        xlim(ui.axGps,[min(rx)-xp max(rx)+xp+max(40,xR*0.38)]);
        ylim(ui.axGps,[min(ry)-yp max(ry)+yp]);
    end
    ui.gpsInfo=text(ui.axGps,0.98,0.96,'Local XY route','Units','normalized','HorizontalAlignment','right','FontSize',9,'Color',[0.42 0.42 0.42],'Interpreter','none');
else
    text(ui.axGps,0.5,0.5,'주행 경로 없음','Units','normalized','HorizontalAlignment','center','FontSize',12);
    ui.currentGps=gobjects(1);
    ui.gpsInfo=text(ui.axGps,0.98,0.96,'Route unavailable','Units','normalized','HorizontalAlignment','right','FontSize',9);
end

lPos=[0.485 0.33 0.115 0.27];
annotation(fig,'rectangle',lPos,'FaceColor',[1 1 1],'Color',[0.86 0.86 0.84],'LineWidth',1);
annotation(fig,'textbox',[lPos(1)+0.01 lPos(2)+lPos(4)-0.045 lPos(3)-0.02 0.03],'String','범례','LineStyle','none','FontSize',10,'FontWeight','bold','Color',[0.18 0.18 0.18]);
lc_={{[0.40 0.65 0.15],'좋음'},{[0.79 0.50 0.10],'양호'},{[0.90 0.30 0.30],'나쁨'},{[0.20 0.52 0.90],'현재'}};
lry=[0.175 0.130 0.085 0.040];
for kk=1:4
    annotation(fig,'ellipse',[lPos(1)+0.012 lPos(2)+lry(kk) 0.010 0.018],'FaceColor',lc_{kk}{1},'Color',lc_{kk}{1},'LineWidth',1);
    annotation(fig,'textbox',[lPos(1)+0.028 lPos(2)+lry(kk)-0.004 lPos(3)-0.038 0.026],'String',lc_{kk}{2},'LineStyle','none','FontSize',9.5,'Color',[0.20 0.20 0.20]);
end
annotation(fig,'line',[lPos(1)+0.010 lPos(1)+lPos(3)-0.010],[lPos(2)+0.032 lPos(2)+0.032],'Color',[0.88 0.88 0.86]);
ui.legendIndex=annotation(fig,'textbox',[lPos(1)+0.010 lPos(2)+0.004 lPos(3)-0.020 0.024],'String','양호 (index 0.43)','LineStyle','none','FontSize',10.5,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','middle','BackgroundColor',[0.93 0.89 0.80],'Color',[0.45 0.33 0.10]);

ui.axBev=axes('Parent',fig,'Units','normalized','Position',[0.63 0.53 0.33 0.18]);
localPanelBase(ui.axBev,[0.99 0.99 0.985]);
title(ui.axBev,'Camera Bird''s-Eye View','FontSize',12,'FontWeight','bold');
hold(ui.axBev,'on'); grid(ui.axBev,'on');
xlim(ui.axBev,[-10 80]); ylim(ui.axBev,[-10 10]); axis(ui.axBev,'equal');
xlabel(ui.axBev,'Forward X [m]'); ylabel(ui.axBev,'Lateral Y [m]');
set(ui.axBev,'XTick',0:20:80,'YTick',-10:5:10,'FontSize',8);
patch(ui.axBev,[-10 80 80 -10],[-10 -10 10 10],[0.96 0.96 0.95],'EdgeColor','none');
plot(ui.axBev,[0 80],[0 0],':','Color',[0.72 0.72 0.72],'LineWidth',1);
ui.laneLAll  =plot(ui.axBev,nan,nan,'-','Color',[0.65 0.65 0.65],'LineWidth',1);
ui.laneRAll  =plot(ui.axBev,nan,nan,'-','Color',[0.65 0.65 0.65],'LineWidth',1);
ui.laneLValid=plot(ui.axBev,nan,nan,'g-','LineWidth',2);
ui.laneRValid=plot(ui.axBev,nan,nan,'g-','LineWidth',2);
ui.bevDyn=gobjects(0);
ui.bevInfo=text(ui.axBev,0.98,0.96,'Camera objects','Units','normalized','HorizontalAlignment','right','FontSize',9,'Color',[0.35 0.35 0.35]);
egoW=2; egoL=4.5; objW=2; objL=4.5; sideL=4.5;

ui.axIdx=axes('Parent',fig,'Units','normalized','Position',[0.63 0.265 0.33 0.22]);
localPanelBase(ui.axIdx,[0.99 0.99 0.985]);
title(ui.axIdx,'노면 상태 지수','FontSize',13,'FontWeight','bold');
hold(ui.axIdx,'on'); xlim(ui.axIdx,[0 1]); ylim(ui.axIdx,[0 4]); axis(ui.axIdx,'off');
idxNames={'슬립 지수','진동 RMS','마찰계수 μ','경사 보정'};
idxY=[3.15 2.25 1.35 0.45];
idxColor=[0.90 0.28 0.28; 0.78 0.49 0.08; 0.42 0.66 0.11; 0.22 0.53 0.88];
for q=1:4
    text(ui.axIdx,0.04,idxY(q)+0.26,idxNames{q},'FontSize',11,'Color',[0.30 0.30 0.30]);
    patch(ui.axIdx,[0.04 0.92 0.92 0.04],[idxY(q) idxY(q) idxY(q)+0.16 idxY(q)+0.16],[0.92 0.92 0.90],'EdgeColor','none');
    ui.idxFg(q)=patch(ui.axIdx,[0.04 0.20 0.20 0.04],[idxY(q) idxY(q) idxY(q)+0.16 idxY(q)+0.16],idxColor(q,:),'EdgeColor','none');
    ui.idxVal(q)=text(ui.axIdx,0.94,idxY(q)+0.24,'-','HorizontalAlignment','right','FontSize',11,'Color',[0.25 0.25 0.25]);
end
ui.summaryBadge=text(ui.axIdx,0.50,0.08,'양호 (index 0.43)','HorizontalAlignment','center','FontSize',12,'FontWeight','bold','BackgroundColor',[0.93 0.89 0.80],'Color',[0.45 0.33 0.10],'Visible','off');

ui.axTrend(1)=axes('Parent',fig,'Units','normalized','Position',[0.04 0.075 0.28 0.105]);
ui.axTrend(2)=axes('Parent',fig,'Units','normalized','Position',[0.36 0.075 0.28 0.105]);
ui.axTrend(3)=axes('Parent',fig,'Units','normalized','Position',[0.68 0.075 0.28 0.105]);
trendNames={'슬립 지수','노면 거칠기 (norm)','요구 마찰계수 μ_{required}'};
trendData={slip_index,rough_display,mu_display};
trendColor=[0.90 0.28 0.28; 0.78 0.49 0.08; 0.05 0.64 0.46];
trendThresh=[0.10,ROUGH_THRESHOLD,0.35];
for q=1:3
    ax=ui.axTrend(q); localPanelBase(ax,[0.99 0.99 0.985]);
    title(ax,trendNames{q},'FontSize',11,'FontWeight','bold'); hold(ax,'on');
    plot(ax,t,trendData{q},'Color',trendColor(q,:),'LineWidth',1.4);
    if isfinite(trendThresh(q)); yline(ax,trendThresh(q),'--','Color',trendColor(q,:),'LineWidth',1); end
    ui.trendMarker(q)=xline(ax,t(start_idx),'Color',[0.20 0.40 0.90],'LineWidth',1.5);
    grid(ax,'on'); ax.XTick=[]; ax.YTick=[];
end

ui.axTime=axes('Parent',fig,'Units','normalized','Position',[0.02 0.015 0.96 0.035]);
localPanelBase(ui.axTime,[0.99 0.99 0.99]); hold(ui.axTime,'on');
xlim(ui.axTime,[0 1]); ylim(ui.axTime,[0 1]); axis(ui.axTime,'off');
ui.timeLeft =text(ui.axTime,0.015,0.5,'00:00:00','VerticalAlignment','middle','FontSize',10,'Color',[0.45 0.45 0.45]);
ui.timeRight=text(ui.axTime,0.965,0.5,localSecToHMS(t(end)-t(start_idx)),'HorizontalAlignment','right','VerticalAlignment','middle','FontSize',10,'Color',[0.30 0.30 0.30]);
plot(ui.axTime,[0.08 0.92],[0.5 0.5],'-','Color',[0.88 0.88 0.88],'LineWidth',6,'HitTest','off');
ui.timeTrackFg=plot(ui.axTime,[0.08 0.08],[0.5 0.5],'-','Color',[0.76 0.76 0.76],'LineWidth',6,'HitTest','off');
ui.timeDot=scatter(ui.axTime,0.08,0.5,180,[0.10 0.48 0.95],'filled','MarkerEdgeColor',[1 1 1],'LineWidth',1.2,'HitTest','off');
ui.timeSlider=gobjects(1);

for p_={[0.04 0.265 0.56 0.40],[0.63 0.53 0.33 0.18],[0.63 0.265 0.33 0.22],[0.04 0.075 0.28 0.105],[0.36 0.075 0.28 0.105],[0.68 0.075 0.28 0.105],[0.02 0.015 0.96 0.035]}
    localAddBorder(fig,p_{1});
end

lane=struct();
lane.has=exist('LaneMarkPosition_Lh_ME','var')&&exist('LaneMarkPosition_Rh_ME','var');
if lane.has
    lane.c0_l=LaneMarkPosition_Lh_ME(:); lane.c1_l=LaneMarkHeadAngle_Lh_ME(:);
    lane.c2_l=LaneMarkModelA_Lh_ME(:);   lane.c3_l=LaneMarkModelDerivA_Lh_ME(:);
    lane.vr_l=LaneMarkViewRange_Lh_ME(:);lane.qual_l=LaneMarkQuality_Lh_ME(:);
    lane.c0_r=LaneMarkPosition_Rh_ME(:); lane.c1_r=LaneMarkHeadAngle_Rh_ME(:);
    lane.c2_r=LaneMarkModelA_Rh_ME(:);   lane.c3_r=LaneMarkModelDerivA_Rh_ME(:);
    lane.vr_r=LaneMarkViewRange_Rh_ME(:);lane.qual_r=LaneMarkQuality_Rh_ME(:);
end

camObjX=nan(N,8); camObjY=nan(N,8);
for objNo=1:8
    xN=sprintf('PosX%02d',objNo); yN=sprintf('PosY%02d',objNo);
    if exist(xN,'var')&&exist(yN,'var')
        ox=eval([xN '(:)']); oy=eval([yN '(:)']);
        Mobj=min([N,length(ox),length(oy)]);
        camObjX(1:Mobj,objNo)=ox(1:Mobj); camObjY(1:Mobj,objNo)=oy(1:Mobj);
    end
end

app=struct();
app.ui=ui; app.N=N; app.t=t; app.startIdx=start_idx; app.endIdx=end_idx; app.idx=start_idx;
app.dataFile=DATA_FILE; app.datasetList=DATASET_LIST; app.choiceFile=DATASET_CHOICE_FILE;
app.scriptPath=SCRIPT_PATH; app.scriptName=SCRIPT_NAME;
app.stepBase=max(1,ANIMATION_STEP); app.speedChoices=[1 2 4]; app.speedChoiceIdx=1;
app.isPlaying=false; app.isSeekingTime=false;
app.timer=timer('ExecutionMode','fixedSpacing','Period',max(0.02,ANIMATION_PAUSE),'TimerFcn',@(~,~)localTimerStep(fig));
app.speed_kph=speed_kph; app.grade_deg=grade_deg; app.grade_source=grade_source;
app.bank_final_deg=bank_final_deg; app.slip_abs_max=slip_index; app.slip_diag_max=slip_abs_max;
app.has_front_obj=has_front_obj; app.front_obj_x=front_obj_x; app.front_obj_y=front_obj_y;
app.has_side_left=has_side_left; app.side_left_x=side_left_x; app.side_left_y=side_left_y;
app.has_side_right=has_side_right; app.side_right_x=side_right_x; app.side_right_y=side_right_y;
app.slip_angle_max_deg=slip_angle_max_deg; app.mu_required=mu_required;
app.mu_demand=mu_demand; app.mu_road=mu_road; app.mu_display=mu_display;
app.rough_rms=rough_rms; app.roughness_norm=roughness_norm; 
app.rough_display=rough_display; app.lat_error=lat_error;
app.rough_threshold=ROUGH_THRESHOLD;
app.road_state=road_state; app.road_state_code=road_state_code;
app.ABS_suspect=ABS_suspect; app.TCS_suspect=TCS_suspect; app.ESC_suspect=ESC_suspect;
app.pos_lat=pos_lat; app.pos_lon=pos_lon; app.gps_valid=gps_valid;
app.route_x=route_x; app.route_y=route_y; app.route_valid=route_valid;
app.maxRough = max(ROUGH_THRESHOLD*2, max(localFinite(rough_display)));
if isempty(app.maxRough)||app.maxRough<=0||~isfinite(app.maxRough); app.maxRough=ROUGH_THRESHOLD*2; end
app.egoW=egoW; app.egoL=egoL; app.objW=objW; app.objL=objL; app.sideL=sideL;
app.lane=lane; app.camObjX=camObjX; app.camObjY=camObjY;

guidata(fig,app);
set(fig,'CloseRequestFcn',@(src,~)localCloseDashboard(src));
localUpdateDashboard(fig,start_idx);
fprintf('\nFinal project finished.\n');

%% ========================================================================
%  Local Functions (document 6 동일)
%% ========================================================================
function localAddBorder(fig,pos); annotation(fig,'rectangle',pos,'Color',[0.74 0.74 0.74],'LineWidth',0.8); end
function localPanelBase(ax,bg); set(ax,'Color',bg,'Box','on','XTick',[],'YTick',[]); end

function card=localCreateCard(fig,pos,titleStr,unitStr)
card.ax=axes('Parent',fig,'Units','normalized','Position',pos);
localPanelBase(card.ax,[0.94 0.94 0.92]); hold(card.ax,'on'); xlim(card.ax,[0 1]); ylim(card.ax,[0 1]); axis(card.ax,'off');
card.title=text(card.ax,0.06,0.80,titleStr,'FontSize',10,'Color',[0.35 0.35 0.35],'Interpreter','none');
card.value=text(card.ax,0.06,0.46,'-','FontSize',18,'FontWeight','bold','Color',[0.08 0.08 0.08],'Interpreter','none');
card.unit =text(card.ax,0.34,0.47,unitStr,'FontSize',10,'Color',[0.45 0.45 0.45],'Interpreter','none');
card.badge=text(card.ax,0.06,0.15,'-','FontSize',11,'FontWeight','bold','Color',[0.28 0.45 0.18],'BackgroundColor',[0.87 0.91 0.82],'Interpreter','none');
card.isGradeCard=false; card.gaugeBase=gobjects(1); card.gaugeZero=gobjects(1);
card.gaugeLine=gobjects(1); card.gaugeDot=gobjects(1);
card.gaugeLeft=gobjects(1); card.gaugeCenter=gobjects(1); card.gaugeRight=gobjects(1);
card.bankText=gobjects(1);  % 뱅크각 수치 텍스트 (비grade 카드는 미사용)
if strcmp(titleStr,'도로 경사')
    card.isGradeCard=true;
    card.gaugeBase=plot(card.ax,[0.58 0.90],[0.28 0.28],'-','Color',[0.80 0.80 0.80],'LineWidth',3);
    card.gaugeZero=plot(card.ax,[0.74 0.74],[0.22 0.34],'-','Color',[0.70 0.70 0.70],'LineWidth',1);
    card.gaugeLine=plot(card.ax,[0.58 0.90],[0.28 0.28],'-','Color',[0.30 0.45 0.18],'LineWidth',4);
    card.gaugeDot =plot(card.ax,0.74,0.28,'o','MarkerSize',5.5,'MarkerFaceColor',[0.30 0.45 0.18],'MarkerEdgeColor',[1 1 1],'LineWidth',0.8);
    card.gaugeLeft  =text(card.ax,0.56,0.17,'-','FontSize',8.5,'Color',[0.45 0.45 0.45],'HorizontalAlignment','center');
    card.gaugeCenter=text(card.ax,0.74,0.17,'0','FontSize',8.5,'Color',[0.45 0.45 0.45],'HorizontalAlignment','center');
    card.gaugeRight =text(card.ax,0.92,0.17,'+','FontSize',8.5,'Color',[0.45 0.45 0.45],'HorizontalAlignment','center');
    % 뱅크각 수치 표시 텍스트
    card.bankText=text(card.ax,0.58,0.41,'뱅크: -','FontSize',8.5,...
        'Color',[0.35 0.35 0.35],'Interpreter','none');
end
end

function localSetCard(card,valueStr,badgeStr,bg,fg,numericValue,bankDeg)
if nargin<6; numericValue=NaN; end
if nargin<7; bankDeg=NaN; end
set(card.value,'String',valueStr); set(card.badge,'String',['  ' badgeStr '  '],'BackgroundColor',bg,'Color',fg);
if isfield(card,'isGradeCard')&&card.isGradeCard&&isfield(card,'gaugeLine')&&isgraphics(card.gaugeLine)
    g=numericValue; if ~isfinite(g); g=0; end; g=max(-10,min(10,g));
    x1=0.58; x2=0.90; xc=(x1+x2)/2; y0=0.28; dy=0.18*(g/10);
    set(card.gaugeLine,'XData',[x1 x2],'YData',[y0-dy y0+dy],'Color',fg);
    set(card.gaugeDot,'XData',xc,'YData',y0,'MarkerFaceColor',fg);
end
% 뱅크각 수치 업데이트
if isfield(card,'bankText')&&isgraphics(card.bankText)
    if isfinite(bankDeg)
        set(card.bankText,'String',sprintf('뱅크: %.1f°',bankDeg));
    else
        set(card.bankText,'String','뱅크: -');
    end
end
end

function localUpdateDashboard(fig,idx)
app=guidata(fig); idx=max(app.startIdx,min(app.endIdx,round(idx))); app.idx=idx; ui=app.ui; i=idx;
speed=app.speed_kph(i); slip=app.slip_abs_max(i); mu=app.mu_display(i); grade=app.grade_deg(i); 
rough=app.rough_display(i);   % 분류와 동일한 roughness_norm 표시
stateCode=app.road_state_code(i);
[rL,rB,rF]=localRoadBadge(stateCode); [sL,sB,sF]=localSlipBadge(slip); [mL,mB,mF]=localMuBadge(mu); [gL,gB,gF]=localGradeBadge(grade);
localSetCard(ui.card(1),localNum(speed,'%.0f'),rL,rB,rF,speed); localSetCard(ui.card(2),localNum(slip,'%.2f'),sL,sB,sF,slip); localSetCard(ui.card(3),localNum(mu,'%.2f'),mL,mB,mF,mu);
bankDeg_i = app.bank_final_deg(i);
localSetCard(ui.card(4),localNum(grade,'%.1f'),gL,gB,gF,grade,bankDeg_i);
if isgraphics(ui.headerLive); set(ui.headerLive,'String',''); end
if any(app.route_valid)&&i<=numel(app.route_x)&&app.route_valid(i)&&isgraphics(ui.currentGps)
    set(ui.currentGps,'XData',app.route_x(i),'YData',app.route_y(i));
    set(ui.gpsInfo,'String',sprintf('Local path · X %.1f m, Y %.1f m',app.route_x(i),app.route_y(i)));
end
switch stateCode; case 5; egoFace=[0.78 0.52 0.86]; case 4; egoFace=[0.97 0.58 0.40]; case 3; egoFace=[0.58 0.80 0.94]; case 2; egoFace=[0.62 0.78 0.94]; otherwise; egoFace=[0.70 0.70 0.70]; end
if isfield(ui,'bevDyn'); try; delete(ui.bevDyn(isgraphics(ui.bevDyn))); catch; end; end
ui.bevDyn=gobjects(0);
xTemp=0:0.5:80;
if isfield(app,'lane')&&app.lane.has&&i<=numel(app.lane.c0_l)
    yl=app.lane.c0_l(i)+app.lane.c1_l(i).*xTemp+app.lane.c2_l(i).*xTemp.^2+app.lane.c3_l(i).*xTemp.^3;
    yr=app.lane.c0_r(i)+app.lane.c1_r(i).*xTemp+app.lane.c2_r(i).*xTemp.^2+app.lane.c3_r(i).*xTemp.^3;
    set(ui.laneLAll,'XData',xTemp,'YData',-yl); set(ui.laneRAll,'XData',xTemp,'YData',-yr);
    vrL=app.lane.vr_l(i); vrR=app.lane.vr_r(i);
    if isfinite(vrL)&&vrL>0; xlv=0:0.5:min(vrL,80); ylv=app.lane.c0_l(i)+app.lane.c1_l(i).*xlv+app.lane.c2_l(i).*xlv.^2+app.lane.c3_l(i).*xlv.^3; set(ui.laneLValid,'XData',xlv,'YData',-ylv); else; set(ui.laneLValid,'XData',nan,'YData',nan); end
    if isfinite(vrR)&&vrR>0; xrv=0:0.5:min(vrR,80); yrv=app.lane.c0_r(i)+app.lane.c1_r(i).*xrv+app.lane.c2_r(i).*xrv.^2+app.lane.c3_r(i).*xrv.^3; set(ui.laneRValid,'XData',xrv,'YData',-yrv); else; set(ui.laneRValid,'XData',nan,'YData',nan); end
else; set([ui.laneLAll ui.laneRAll ui.laneLValid ui.laneRValid],'XData',nan,'YData',nan); end
ui.bevDyn(end+1:end+5)=localDrawVeh(ui.axBev,0,0,-pi/2,app.egoW,app.egoL,[0 0 0],1.5,egoFace);
ui.bevDyn(end+1)=text(ui.axBev,0,0,'EGO','HorizontalAlignment','center','FontWeight','bold','Color',[0.05 0.15 0.35],'FontSize',8);
objCnt=0; frontDist=inf;
if isfield(app,'camObjX')&&i<=size(app.camObjX,1)
    for oo=1:size(app.camObjX,2)
        ox=app.camObjX(i,oo); oy=app.camObjY(i,oo);
        if isfinite(ox)&&isfinite(oy)&&ox>0.5&&ox<80&&abs(oy)<10&&~(abs(ox)<0.1&&abs(oy)<0.1)
            objCnt=objCnt+1;
            if abs(oy)<2.2&&ox<frontDist; frontDist=ox; col=[0.90 0.20 0.20]; lb='CIPV';
            else; col=[0.80 0.80 0.80]; lb=sprintf('O%d',oo); end
            ui.bevDyn(end+1:end+5)=localDrawVeh(ui.axBev,ox,-oy,-pi/2,app.objW,app.objL,col,1.2,col);
            ui.bevDyn(end+1)=text(ui.axBev,ox,-oy,lb,'HorizontalAlignment','center','FontWeight','bold','Color',[0.1 0.1 0.1],'FontSize',7);
        end
    end
end
if isfinite(frontDist); set(ui.bevInfo,'String',sprintf('CIPV %.0fm / Obj %d',frontDist,objCnt));
elseif objCnt>0; set(ui.bevInfo,'String',sprintf('Obj %d / CIPV 없음',objCnt));
else; set(ui.bevInfo,'String','객체 없음'); end
app.ui=ui;
vals=[localClamp(localZero(slip)/0.35,0,1),localClamp(localZero(rough)/app.maxRough,0,1),localClamp(localZero(mu)/1.0,0,1),localClamp(abs(localZero(grade))/10,0,1)];
valText={localNum(slip,'%.2f'),localNum(rough,'%.2f'),localNum(mu,'%.2f'),[localNum(grade,'%.1f') '°']};
for q=1:4; xE=0.04+0.88*vals(q); set(ui.idxFg(q),'XData',[0.04 xE xE 0.04]); set(ui.idxVal(q),'String',valText{q}); end
sIdx=0.35*localClamp(localZero(mu),0,1)+0.25*(1-localClamp(localZero(slip)/0.25,0,1))+0.25*(1-localClamp(localZero(rough)/app.maxRough,0,1))+0.15*(1-localClamp(abs(localZero(grade))/10,0,1));
if sIdx>0.70; iStr=sprintf('좋음 (%.2f)',sIdx); iBg=[0.86 0.92 0.80]; iFg=[0.24 0.46 0.14];
elseif sIdx>0.45; iStr=sprintf('양호 (%.2f)',sIdx); iBg=[0.93 0.89 0.80]; iFg=[0.45 0.33 0.10];
else; iStr=sprintf('주의 (%.2f)',sIdx); iBg=[0.96 0.83 0.82]; iFg=[0.60 0.18 0.18]; end
if isfield(ui,'legendIndex')&&isgraphics(ui.legendIndex); set(ui.legendIndex,'String',iStr,'BackgroundColor',iBg,'Color',iFg); end
if isfield(ui,'summaryBadge')&&isgraphics(ui.summaryBadge); set(ui.summaryBadge,'String',iStr,'BackgroundColor',iBg,'Color',iFg,'Visible','off'); end
for q=1:3; set(ui.trendMarker(q),'Value',app.t(i)); end
p=(i-app.startIdx)/max(app.endIdx-app.startIdx,1); xPos=0.08+0.84*p;
if isfield(ui,'timeTrackFg')&&isgraphics(ui.timeTrackFg); set(ui.timeTrackFg,'XData',[0.08 xPos],'YData',[0.5 0.5]); end
if isfield(ui,'timeDot')&&isgraphics(ui.timeDot); set(ui.timeDot,'XData',xPos,'YData',0.5); end
set(ui.timeLeft,'String',localSecToHMS(app.t(i)-app.t(app.startIdx)));
guidata(fig,app); drawnow limitrate
end

function h=localDrawVeh(ax,xi,yi,yaw,width,len,color,line_w,faceColor)
if nargin<9; faceColor=color; end; w=width; l=len;
P1=[w/2;-l/2]; P2=[w/2;l/2]; P3=[-w/2;l/2]; P4=[-w/2;-l/2];
D1=[w/2;l/4]; D2=[0;l/2]; D3=[-w/2;l/4];
T=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
body=T*[P1 P2 P3 P4]; body(1,:)=body(1,:)+xi; body(2,:)=body(2,:)+yi;
dirp=T*[D1 D2 D3]; dirp(1,:)=dirp(1,:)+xi; dirp(2,:)=dirp(2,:)+yi;
h=gobjects(5,1);
h(1)=patch(ax,body(1,:),body(2,:),faceColor,'EdgeColor',color,'LineWidth',line_w,'FaceAlpha',0.65);
h(2)=plot(ax,[dirp(1,:) dirp(1,1)],[dirp(2,:) dirp(2,1)],'-','Color',color,'LineWidth',max(1.2,line_w));
for k=1:3; h(k+2)=plot(ax,xi,yi,'.','Color',color,'MarkerSize',1); end
end

function localCbDataset(fig,src)
if ~ishandle(fig); return; end; app=guidata(fig);
if isempty(app)||~isstruct(app); return; end
list=get(src,'String'); val=get(src,'Value');
if ischar(list); selectedFile=strtrim(list(val,:)); else; selectedFile=list{val}; end
if isfield(app,'dataFile')&&strcmp(selectedFile,app.dataFile); return; end
DATA_FILE_CHOICE=selectedFile;
try; save(app.choiceFile,'DATA_FILE_CHOICE'); catch ME; warning('저장 실패: %s',ME.message); end
try; if isfield(app,'timer')&&isa(app.timer,'timer'); stop(app.timer); delete(app.timer); end; catch; end
global ROAD_SURFACE_REUSE_FIG ROAD_SURFACE_REUSE_REQUEST
ROAD_SURFACE_REUSE_FIG=fig; ROAD_SURFACE_REUSE_REQUEST=true;
try; set(fig,'Pointer','watch'); drawnow; catch; end
scriptPath=''; if isfield(app,'scriptPath'); scriptPath=app.scriptPath; end
scriptName=''; if isfield(app,'scriptName'); scriptName=app.scriptName; end
try
    if ~isempty(scriptPath)&&isfile(scriptPath); evalin('base',sprintf('run(''%s'')',strrep(scriptPath,'''','''''')));
    elseif ~isempty(scriptName)&&~isempty(which(scriptName)); evalin('base',sprintf('%s',scriptName));
    elseif ~isempty(scriptName)&&isfile(fullfile(pwd,[scriptName '.m'])); evalin('base',sprintf('run(''%s'')',strrep(fullfile(pwd,[scriptName '.m']),'''','''''')));
    else; ROAD_SURFACE_REUSE_REQUEST=false; warning('스크립트 경로 없음'); try; set(fig,'Pointer','arrow'); catch; end; end
catch ME; ROAD_SURFACE_REUSE_REQUEST=false; warning('재실행 실패: %s',ME.message); try; set(fig,'Pointer','arrow'); catch; end; end
end

function localCbPlay(fig); app=guidata(fig); if isempty(app)||~isstruct(app)||~isfield(app,'isPlaying'); return; end; if app.isPlaying; app.isPlaying=false; try; stop(app.timer); catch; end; set(app.ui.btnPlay,'String','▶ 재생'); else; app.isPlaying=true; set(app.ui.btnPlay,'String','⏸ 정지'); try; start(app.timer); catch; end; end; guidata(fig,app); end
function localTimerStep(fig); if ~ishandle(fig); return; end; app=guidata(fig); if isempty(app)||~isstruct(app)||~isfield(app,'isPlaying')||~app.isPlaying; return; end; mult=app.speedChoices(app.speedChoiceIdx); ni=app.idx+app.stepBase*mult; if ni>app.endIdx; ni=app.startIdx; end; app.idx=ni; guidata(fig,app); localUpdateDashboard(fig,ni); end
function localCbReset(fig); localJumpSeconds(fig,-10); end
function localCbStep(fig);  localJumpSeconds(fig, 10); end
function localJumpSeconds(fig,deltaSec)
if ~ishandle(fig); return; end; app=guidata(fig); if isempty(app)||~isstruct(app); return; end
targetT=app.t(app.idx)+deltaSec;
if targetT<=app.t(app.startIdx); newIdx=app.startIdx;
elseif targetT>=app.t(app.endIdx); newIdx=app.endIdx;
else; sr=app.startIdx:app.endIdx; [~,ri]=min(abs(app.t(sr)-targetT)); newIdx=sr(ri); end
app.idx=newIdx; guidata(fig,app); localUpdateDashboard(fig,newIdx);
end
function localCbSpeed(fig); app=guidata(fig); app.speedChoiceIdx=mod(app.speedChoiceIdx,numel(app.speedChoices))+1; set(app.ui.btnSpeed,'String',sprintf('속도 ×%d',app.speedChoices(app.speedChoiceIdx))); guidata(fig,app); end
function localCbExport(fig); try; fname=sprintf('road_surface_dashboard_%s.png',datestr(now,'yyyymmdd_HHMMSS')); exportgraphics(fig,fname,'Resolution',150); fprintf('Exported: %s\n',fname); catch; fname=sprintf('road_surface_dashboard_%s.png',datestr(now,'yyyymmdd_HHMMSS')); saveas(fig,fname); fprintf('Exported: %s\n',fname); end; end
function localCloseDashboard(fig); try; app=guidata(fig); if isfield(app,'timer')&&isa(app.timer,'timer'); stop(app.timer); delete(app.timer); end; catch; end; delete(fig); end
function [lb,bg,fg]=localRoadBadge(c); switch c; case 3; lb='정상 주행'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; case 2; lb='양호 노면'; bg=[0.93 0.89 0.80]; fg=[0.45 0.33 0.10]; case 4; lb='안정성 저하'; bg=[0.96 0.83 0.82]; fg=[0.60 0.18 0.18]; case 5; lb='거친 노면'; bg=[0.89 0.83 0.95]; fg=[0.40 0.18 0.52]; otherwise; lb='저자극 주행'; bg=[0.91 0.91 0.91]; fg=[0.35 0.35 0.35]; end; end
function [lb,bg,fg]=localSlipBadge(v); if ~isfinite(v); v=0; end; if v<0.05; lb='안정'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif v<0.12; lb='경미한 슬립'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; else; lb='슬립 주의'; bg=[0.96 0.83 0.82]; fg=[0.60 0.18 0.18]; end; end
function [lb,bg,fg]=localMuBadge(v); if ~isfinite(v); v=0; end; if v>0.60; lb='고접지 확인'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif v>0.35; lb='중간 접지'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; else; lb='판단 보류'; bg=[0.91 0.91 0.91]; fg=[0.35 0.35 0.35]; end; end
function [lb,bg,fg]=localGradeBadge(v); if ~isfinite(v); v=0; end; a=abs(v); if a<2; lb='평탄 도로'; bg=[0.88 0.93 0.83]; fg=[0.22 0.45 0.18]; elseif a<5; lb='완만한 경사'; bg=[0.90 0.93 0.84]; fg=[0.30 0.45 0.18]; else; lb='급경사 구간'; bg=[0.95 0.90 0.82]; fg=[0.55 0.37 0.08]; end; end
function out=localClamp(x,lo,hi); out=min(max(x,lo),hi); end
function out=localZero(x); if ~isfinite(x); out=0; else; out=x; end; end
function s=localNum(x,fmt); if ~isfinite(x); s='-'; else; s=sprintf(fmt,x); end; end
function arr=localFinite(x); arr=x(isfinite(x)); end
function m=localNanMean(x); x=x(isfinite(x)); if isempty(x); m=NaN; else; m=mean(x); end; end
function txt=localSecToHMS(sec); if ~isfinite(sec); sec=0; end; sec=max(sec,0); txt=sprintf('%02d:%02d:%02d',floor(sec/3600),floor(mod(sec,3600)/60),floor(mod(sec,60))); end