function road_state_dashboard
%% =========================================================
% road_state_dashboard.m
%
% 목적:
% testset1.mat 데이터를 이용하여
% 1) 원본 데이터 UI
% 2) 계산 결과 UI
% 3) 최종 도로 상태 UI
% 4) 차량 이동 애니메이션 UI
% 를 한 화면에서 보여준다.
%
% 사용 데이터:
% - Time
% - WHL_SPD_FL / WHL_SPD_FR / WHL_SPD_RL / WHL_SPD_RR
% - AccelZ 또는 AccelDown
% - AngRateZ 또는 YAW_RATE
% - AnglePitch 또는 Pitch_Angle
% - VelDown
% - VelForward
% - SAS_Angle (있으면 사용, 없으면 0)
%
% 특징:
% - GPS, Camera 사용 안 함
% - 가상 Slip 생성 안 함
% - 실제 데이터만 사용
% - UI를 "원본 데이터 → 계산 결과 → 최종 판단 → 차량 이동" 순서로 구성
%% =========================================================

clear; clc; close all;

%% =========================================================
% 1. 데이터 로드
%% =========================================================
dataFile = 'testset1.mat';

if ~isfile(dataFile)
    [fileName, filePath] = uigetfile('*.mat', 'testset1.mat 파일을 선택하세요');
    if isequal(fileName, 0)
        error('MAT 파일이 선택되지 않았습니다.');
    end
    dataFile = fullfile(filePath, fileName);
end

S = load(dataFile);

%% =========================================================
% 2. 필요한 신호 읽기
%% =========================================================
t = getRequiredSignal(S, {'Time'});
t = t(:);
t = t - t(1);
N = length(t);

Ts = median(diff(t), 'omitnan');
Fs = 1 / Ts;

WHL_FL = alignSignal(getRequiredSignal(S, {'WHL_SPD_FL'}), N);
WHL_FR = alignSignal(getRequiredSignal(S, {'WHL_SPD_FR'}), N);
WHL_RL = alignSignal(getRequiredSignal(S, {'WHL_SPD_RL'}), N);
WHL_RR = alignSignal(getRequiredSignal(S, {'WHL_SPD_RR'}), N);

AccelZ = alignSignal(getOptionalSignal(S, {'AccelZ','AccelDown'}, zeros(N,1)), N);
AngRate = alignSignal(getOptionalSignal(S, {'AngRateZ','YAW_RATE'}, zeros(N,1)), N);
Pitch   = alignSignal(getOptionalSignal(S, {'AnglePitch','Pitch_Angle'}, zeros(N,1)), N);
VelDown = alignSignal(getOptionalSignal(S, {'VelDown'}, zeros(N,1)), N);
VelFwd  = alignSignal(getOptionalSignal(S, {'VelForward'}, zeros(N,1)), N);
SAS_Angle = alignSignal(getOptionalSignal(S, {'SAS_Angle'}, zeros(N,1)), N);

%% =========================================================
% 3. 전처리
%% =========================================================
WHL_FL = fillmissing(double(WHL_FL), 'linear');
WHL_FR = fillmissing(double(WHL_FR), 'linear');
WHL_RL = fillmissing(double(WHL_RL), 'linear');
WHL_RR = fillmissing(double(WHL_RR), 'linear');

AccelZ = fillmissing(double(AccelZ), 'linear');
AngRate = fillmissing(double(AngRate), 'linear');
Pitch = fillmissing(double(Pitch), 'linear');
VelDown = fillmissing(double(VelDown), 'linear');
VelFwd = fillmissing(double(VelFwd), 'linear');
SAS_Angle = fillmissing(double(SAS_Angle), 'linear');

wheelMat  = [WHL_FL, WHL_FR, WHL_RL, WHL_RR];
wheelMean = mean(wheelMat, 2, 'omitnan');

% 너무 저속에서는 wheel speed 차이가 과장되어 보일 수 있으므로 제외
validSpeed = wheelMean > 5;

%% =========================================================
% 4. Slip 지수 계산
%% =========================================================
% [설명]
% GPS를 쓰지 않기 때문에 절대 Slip Ratio 대신
% "바퀴들끼리 얼마나 불일치하는가"를 기반으로 Slip 의심 지수를 계산한다.
%
% 식:
% wheelDeviation = std(4바퀴 속도) / 평균 휠속도
% frontRearDiff  = |전륜 평균 - 후륜 평균| / 평균 휠속도
% leftRightDiff  = |좌측 평균 - 우측 평균| / 평균 휠속도
%
% 코너링 중에는 좌우 바퀴 속도 차이가 자연스럽게 커질 수 있으므로
% SAS_Angle, AngRate로 좌우 차이를 보정한다.

frontMean = mean([WHL_FL, WHL_FR], 2, 'omitnan');
rearMean  = mean([WHL_RL, WHL_RR], 2, 'omitnan');
leftMean  = mean([WHL_FL, WHL_RL], 2, 'omitnan');
rightMean = mean([WHL_FR, WHL_RR], 2, 'omitnan');

wheelDeviation = std(wheelMat, 0, 2, 'omitnan') ./ max(wheelMean, 1);
frontRearDiff  = abs(frontMean - rearMean) ./ max(wheelMean, 1);
leftRightDiff  = abs(leftMean - rightMean) ./ max(wheelMean, 1);

turnCorrection = 1 ./ (1 + 0.004*abs(SAS_Angle) + 0.03*abs(AngRate));
leftRightDiffCorrected = leftRightDiff .* turnCorrection;

slipScoreRaw = ...
    0.45 * wheelDeviation + ...
    0.35 * frontRearDiff + ...
    0.20 * leftRightDiffCorrected;

% 0.08을 기준으로 정규화
slipIndex = min(slipScoreRaw / 0.08, 1);
slipIndex(~validSpeed) = 0;

%% =========================================================
% 5. IMU Z축 진동 RMS 계산
%% =========================================================
% [설명]
% 노면이 거칠면 Z축 방향 진동이 커진다.
% 0.5~10Hz 대역만 남긴 후 1초 이동 RMS 계산
%
% RMS = sqrt(moving mean of x^2)

AccelZ_clean = fillmissing(AccelZ, 'linear');

try
    if Fs > 25
        AccelZ_filt = bandpass(AccelZ_clean, [0.5 10], Fs);
    else
        base = movmean(AccelZ_clean, max(3, round(Fs)), 'omitnan');
        AccelZ_filt = AccelZ_clean - base;
    end
catch
    base = movmean(AccelZ_clean, max(3, round(Fs)), 'omitnan');
    AccelZ_filt = AccelZ_clean - base;
end

win1s = max(3, round(Fs * 1.0));
vibrationRMS = sqrt(movmean(AccelZ_filt.^2, win1s, 'omitnan'));

rmsRef = prctile(vibrationRMS(validSpeed), 95);
if isempty(rmsRef) || isnan(rmsRef) || rmsRef <= 0
    rmsRef = max(vibrationRMS, [], 'omitnan');
end
if isempty(rmsRef) || isnan(rmsRef) || rmsRef <= 0
    rmsRef = 1;
end

rmsIndex = min(vibrationRMS / rmsRef, 1);
rmsIndex(~validSpeed) = 0;

%% =========================================================
% 6. 도로 경사도 계산
%% =========================================================
% [설명]
% 경사각 = atan2(VelDown, VelFwd)
% Pitch 값도 함께 반영
%
% gradeAngle = 0.7*(속도 기반 경사각) + 0.3*(Pitch 각도)

gradeAngleVel = atan2d(VelDown, max(abs(VelFwd), 0.1));

pitchDeg = Pitch;
if max(abs(pitchDeg), [], 'omitnan') < 1
    pitchDeg = rad2deg(pitchDeg);
end

gradeAngle = 0.7 * gradeAngleVel + 0.3 * pitchDeg;

gradeIndex = min(abs(gradeAngle) / 10, 1);
gradeIndex(~validSpeed) = 0;

% 경사 가중치
wGrade = 1 + 0.5 * gradeIndex;

%% =========================================================
% 7. 마찰계수 mu 추정
%% =========================================================
% [설명]
% Slip이 커지고, 진동이 커지고, 경사가 커지면
% 유효 마찰계수가 낮아진다고 가정
%
% muEst = 0.9 - 0.45*Slip - 0.20*RMS - 0.10*Grade

muEst = 0.9 ...
      - 0.45 * slipIndex ...
      - 0.20 * rmsIndex ...
      - 0.10 * gradeIndex;

muEst = min(max(muEst, 0.1), 0.9);

% 위험도 계산용
muRisk = 1 - muEst;

%% =========================================================
% 8. 최종 도로 상태 지수
%% =========================================================
% [설명]
% roadIndex = 0.40*Slip + 0.35*RMS + 0.25*(1-mu)*wGrade

roadIndex = ...
    0.40 * slipIndex + ...
    0.35 * rmsIndex + ...
    0.25 * muRisk .* wGrade;

roadIndex = min(max(roadIndex, 0), 1);
roadIndex(~validSpeed) = 0;

TH1 = 0.30;
TH2 = 0.60;

roadStatus = cell(N,1);
for i = 1:N
    if roadIndex(i) < TH1
        roadStatus{i} = '좋음';
    elseif roadIndex(i) < TH2
        roadStatus{i} = '보통';
    else
        roadStatus{i} = '나쁨';
    end
end

%% =========================================================
% 9. 차량 이동 궤적 계산
%% =========================================================
% [설명]
% GPS 없이 차량이 움직이는 모습을 만들기 위해
% 휠 평균 속도 + 각속도를 적분해서 상대 궤적 생성
%
% v = wheelMean / 3.6
% yaw = integral(AngRate)
% x = integral(v*cos(yaw))
% y = integral(v*sin(yaw))

vehicleSpeed_mps = wheelMean / 3.6;

yawRateRad = AngRate;
if max(abs(AngRate), [], 'omitnan') > 3
    yawRateRad = deg2rad(AngRate);
end

yawAngle = cumtrapz(t, yawRateRad);

xPos = cumtrapz(t, vehicleSpeed_mps .* cos(yawAngle));
yPos = cumtrapz(t, vehicleSpeed_mps .* sin(yawAngle));

%% =========================================================
% 10. UI 표시용 다운샘플링
%% =========================================================
maxPlotN = 3000;
ds = max(1, floor(N / maxPlotN));
idxPlot = 1:ds:N;

tp = t(idxPlot);

WHL_FL_p = WHL_FL(idxPlot);
WHL_FR_p = WHL_FR(idxPlot);
WHL_RL_p = WHL_RL(idxPlot);
WHL_RR_p = WHL_RR(idxPlot);

AccelZ_p = AccelZ(idxPlot);
AngRate_p = AngRate(idxPlot);
Pitch_p = pitchDeg(idxPlot);
VelDown_p = VelDown(idxPlot);
VelFwd_p = VelFwd(idxPlot);

slipIndex_p = slipIndex(idxPlot);
rmsIndex_p = rmsIndex(idxPlot);
gradeIndex_p = gradeIndex(idxPlot);
muEst_p = muEst(idxPlot);
roadIndex_p = roadIndex(idxPlot);
gradeAngle_p = gradeAngle(idxPlot);
vibrationRMS_p = vibrationRMS(idxPlot);

wheelDeviation_p = wheelDeviation(idxPlot);
frontRearDiff_p = frontRearDiff(idxPlot);
leftRightDiffCorrected_p = leftRightDiffCorrected(idxPlot);

xPos_p = xPos(idxPlot);
yPos_p = yPos(idxPlot);
yawAngle_p = yawAngle(idxPlot);
wheelMean_p = wheelMean(idxPlot);
roadStatus_p = roadStatus(idxPlot);

M = length(tp);

%% =========================================================
% 11. 전체 UI 생성
%% =========================================================
fig = uifigure( ...
    'Name', '노면 상태 추정 대시보드', ...
    'Position', [40 30 1700 980]);

mainGrid = uigridlayout(fig, [3 1]);
mainGrid.RowHeight = {270, 280, '1x'};
mainGrid.ColumnWidth = {'1x'};
mainGrid.Padding = [10 10 10 10];
mainGrid.RowSpacing = 10;

%% ---------------------------------------------------------
% Row 1 : 원본 데이터 UI
%% ---------------------------------------------------------
rawPanel = uipanel(mainGrid, 'Title', '1. 데이터 파일에 있는 원본 데이터');
rawPanel.Layout.Row = 1;
rawGrid = uigridlayout(rawPanel, [1 4]);
rawGrid.ColumnWidth = {'1.2x', '1x', '1x', 280};
rawGrid.ColumnSpacing = 8;

% (1) Wheel speed plot
axRawWheel = uiaxes(rawGrid);
title(axRawWheel, '휠 속도 데이터');
xlabel(axRawWheel, 'Time [s]');
ylabel(axRawWheel, 'km/h');
hold(axRawWheel, 'on'); grid(axRawWheel, 'on');
plot(axRawWheel, tp, WHL_FL_p, 'LineWidth', 0.9);
plot(axRawWheel, tp, WHL_FR_p, 'LineWidth', 0.9);
plot(axRawWheel, tp, WHL_RL_p, 'LineWidth', 0.9);
plot(axRawWheel, tp, WHL_RR_p, 'LineWidth', 0.9);
legend(axRawWheel, {'FL','FR','RL','RR'}, 'Location', 'best');
rawCursor1 = xline(axRawWheel, tp(1), '-', 'Current');

% (2) IMU plot
axRawIMU = uiaxes(rawGrid);
title(axRawIMU, 'IMU 데이터');
xlabel(axRawIMU, 'Time [s]');
ylabel(axRawIMU, 'Signal');
hold(axRawIMU, 'on'); grid(axRawIMU, 'on');
plot(axRawIMU, tp, AccelZ_p, 'LineWidth', 1.0);
plot(axRawIMU, tp, AngRate_p, 'LineWidth', 1.0);
legend(axRawIMU, {'AccelZ','AngRate'}, 'Location', 'best');
rawCursor2 = xline(axRawIMU, tp(1), '-', 'Current');

% (3) Slope input plot
axRawSlope = uiaxes(rawGrid);
title(axRawSlope, '경사 계산용 데이터');
xlabel(axRawSlope, 'Time [s]');
ylabel(axRawSlope, 'Signal');
hold(axRawSlope, 'on'); grid(axRawSlope, 'on');
plot(axRawSlope, tp, Pitch_p, 'LineWidth', 1.0);
plot(axRawSlope, tp, VelDown_p, 'LineWidth', 1.0);
plot(axRawSlope, tp, VelFwd_p, 'LineWidth', 1.0);
legend(axRawSlope, {'Pitch','VelDown','VelFwd'}, 'Location', 'best');
rawCursor3 = xline(axRawSlope, tp(1), '-', 'Current');

% (4) 현재 원본 데이터 수치
rawInfoPanel = uipanel(rawGrid, 'Title', '현재 원본 값');
rawInfoGrid = uigridlayout(rawInfoPanel, [10 1]);
rawInfoGrid.RowHeight = {24,24,24,24,24,24,24,24,24,'1x'};

lblRawFL = uilabel(rawInfoGrid, 'Text', 'FL: -');
lblRawFR = uilabel(rawInfoGrid, 'Text', 'FR: -');
lblRawRL = uilabel(rawInfoGrid, 'Text', 'RL: -');
lblRawRR = uilabel(rawInfoGrid, 'Text', 'RR: -');
lblRawAccel = uilabel(rawInfoGrid, 'Text', 'AccelZ: -');
lblRawAng = uilabel(rawInfoGrid, 'Text', 'AngRate: -');
lblRawPitch = uilabel(rawInfoGrid, 'Text', 'Pitch: -');
lblRawVdown = uilabel(rawInfoGrid, 'Text', 'VelDown: -');
lblRawVfwd = uilabel(rawInfoGrid, 'Text', 'VelFwd: -');
lblRawTime = uilabel(rawInfoGrid, 'Text', 'Time: -', 'FontWeight', 'bold');

%% ---------------------------------------------------------
% Row 2 : 계산 결과 UI
%% ---------------------------------------------------------
calcPanel = uipanel(mainGrid, 'Title', '2. 원본 데이터를 이용해 계산한 값');
calcPanel.Layout.Row = 2;
calcGrid = uigridlayout(calcPanel, [1 4]);
calcGrid.ColumnWidth = {'1x','1x','1x','1x'};
calcGrid.ColumnSpacing = 8;

% Slip 계산 UI
slipPanel = uipanel(calcGrid, 'Title', '슬립 계산');
slipGrid = uigridlayout(slipPanel, [2 1]);
slipGrid.RowHeight = {'1x', 90};

axSlip = uiaxes(slipGrid);
title(axSlip, 'Slip 관련 지표');
xlabel(axSlip, 'Time [s]');
ylabel(axSlip, 'Index');
hold(axSlip, 'on'); grid(axSlip, 'on');
plot(axSlip, tp, wheelDeviation_p, 'LineWidth', 0.9);
plot(axSlip, tp, frontRearDiff_p, 'LineWidth', 0.9);
plot(axSlip, tp, leftRightDiffCorrected_p, 'LineWidth', 0.9);
plot(axSlip, tp, slipIndex_p, 'LineWidth', 1.6);
legend(axSlip, {'Wheel Dev','Front-Rear','L-R corrected','Slip Index'}, ...
    'Location', 'best');
calcCursor1 = xline(axSlip, tp(1), '-', 'Current');

slipInfoGrid = uigridlayout(slipGrid, [4 1]);
slipInfoGrid.RowHeight = {20,20,20,20};
lblSlip1 = uilabel(slipInfoGrid, 'Text', 'Wheel deviation: -');
lblSlip2 = uilabel(slipInfoGrid, 'Text', 'Front-Rear diff: -');
lblSlip3 = uilabel(slipInfoGrid, 'Text', 'Slip Index: -');
lblSlip4 = uilabel(slipInfoGrid, 'Text', 'Slip 판단: -', 'FontWeight', 'bold');

% RMS 계산 UI
rmsPanel = uipanel(calcGrid, 'Title', 'IMU Z축 진동');
rmsGrid = uigridlayout(rmsPanel, [2 1]);
rmsGrid.RowHeight = {'1x', 90};

axRMS = uiaxes(rmsGrid);
title(axRMS, '진동 RMS');
xlabel(axRMS, 'Time [s]');
ylabel(axRMS, 'Value');
hold(axRMS, 'on'); grid(axRMS, 'on');
plot(axRMS, tp, vibrationRMS_p, 'LineWidth', 1.0);
plot(axRMS, tp, rmsIndex_p, 'LineWidth', 1.6);
legend(axRMS, {'Vibration RMS','RMS Index'}, 'Location', 'best');
calcCursor2 = xline(axRMS, tp(1), '-', 'Current');

rmsInfoGrid = uigridlayout(rmsGrid, [4 1]);
rmsInfoGrid.RowHeight = {20,20,20,20};
lblRms1 = uilabel(rmsInfoGrid, 'Text', 'Filtered AccelZ: -');
lblRms2 = uilabel(rmsInfoGrid, 'Text', 'RMS: -');
lblRms3 = uilabel(rmsInfoGrid, 'Text', 'RMS Index: -');
lblRms4 = uilabel(rmsInfoGrid, 'Text', '진동 상태: -', 'FontWeight', 'bold');

% Grade 계산 UI
gradePanel = uipanel(calcGrid, 'Title', '도로 경사도 계산');
gradeGrid = uigridlayout(gradePanel, [2 1]);
gradeGrid.RowHeight = {'1x', 90};

axGrade = uiaxes(gradeGrid);
title(axGrade, '경사각 / 경사 지수');
xlabel(axGrade, 'Time [s]');
ylabel(axGrade, 'Value');
hold(axGrade, 'on'); grid(axGrade, 'on');
plot(axGrade, tp, gradeAngle_p, 'LineWidth', 1.0);
plot(axGrade, tp, gradeIndex_p, 'LineWidth', 1.6);
legend(axGrade, {'Grade Angle','Grade Index'}, 'Location', 'best');
calcCursor3 = xline(axGrade, tp(1), '-', 'Current');

gradeInfoGrid = uigridlayout(gradeGrid, [4 1]);
gradeInfoGrid.RowHeight = {20,20,20,20};
lblGrade1 = uilabel(gradeInfoGrid, 'Text', 'Grade Angle: -');
lblGrade2 = uilabel(gradeInfoGrid, 'Text', 'Grade Index: -');
lblGrade3 = uilabel(gradeInfoGrid, 'Text', 'wGrade: -');
lblGrade4 = uilabel(gradeInfoGrid, 'Text', '경사 상태: -', 'FontWeight', 'bold');

% mu/종합 계산 UI
muPanel = uipanel(calcGrid, 'Title', '마찰계수 및 종합 계산');
muGrid = uigridlayout(muPanel, [2 1]);
muGrid.RowHeight = {'1x', 90};

axMu = uiaxes(muGrid);
title(axMu, 'mu / Road Index');
xlabel(axMu, 'Time [s]');
ylabel(axMu, 'Value');
hold(axMu, 'on'); grid(axMu, 'on');
plot(axMu, tp, muEst_p, 'LineWidth', 1.0);
plot(axMu, tp, roadIndex_p, 'LineWidth', 1.8);
yline(axMu, TH1, '--', '좋음/보통');
yline(axMu, TH2, '--', '보통/나쁨');
ylim(axMu, [0 1]);
legend(axMu, {'mu','Road Index','TH1','TH2'}, 'Location', 'best');
calcCursor4 = xline(axMu, tp(1), '-', 'Current');

muInfoGrid = uigridlayout(muGrid, [4 1]);
muInfoGrid.RowHeight = {20,20,20,20};
lblMu1 = uilabel(muInfoGrid, 'Text', 'mu: -');
lblMu2 = uilabel(muInfoGrid, 'Text', 'Road Index: -');
lblMu3 = uilabel(muInfoGrid, 'Text', '종합 상태: -');
lblMu4 = uilabel(muInfoGrid, 'Text', '설명: -', 'FontWeight', 'bold');

%% ---------------------------------------------------------
% Row 3 : 최종 결과 + 차량 이동 UI
%% ---------------------------------------------------------
bottomPanel = uipanel(mainGrid, 'Title', '3. 최종 결과 및 차량 이동');
bottomPanel.Layout.Row = 3;
bottomGrid = uigridlayout(bottomPanel, [1 2]);
bottomGrid.ColumnWidth = {420, '1x'};
bottomGrid.ColumnSpacing = 8;

%% (A) 최종 결과 UI
finalPanel = uipanel(bottomGrid, 'Title', '최종 도로 상태');
finalGrid = uigridlayout(finalPanel, [10 1]);
finalGrid.RowHeight = {40, 40, 40, 40, 40, 40, 30, 30, 30, '1x'};

lblFinalTitle = uilabel(finalGrid, ...
    'Text', '현재 도로 상태', ...
    'FontSize', 22, ...
    'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

lblFinalStatus = uilabel(finalGrid, ...
    'Text', '-', ...
    'FontSize', 28, ...
    'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

lblFinalRoad = uilabel(finalGrid, ...
    'Text', 'Road Index: -', ...
    'FontSize', 18, ...
    'HorizontalAlignment', 'center');

lblFinalSlip = uilabel(finalGrid, ...
    'Text', 'Slip Index: -', ...
    'FontSize', 15);

lblFinalRms = uilabel(finalGrid, ...
    'Text', 'RMS Index: -', ...
    'FontSize', 15);

lblFinalGrade = uilabel(finalGrid, ...
    'Text', 'Grade Index: -', ...
    'FontSize', 15);

lblFinalMu = uilabel(finalGrid, ...
    'Text', 'mu: -', ...
    'FontSize', 15);

lblFinalSummary = uilabel(finalGrid, ...
    'Text', '요약: -', ...
    'FontSize', 14, ...
    'FontWeight', 'bold');

lblFinalRule = uilabel(finalGrid, ...
    'Text', '판정 기준: <0.3 좋음 / 0.3~0.6 보통 / >=0.6 나쁨', ...
    'FontSize', 12);

lblDatasetInfo = uilabel(finalGrid, ...
    'Text', sprintf('파일: %s | 샘플: %d | Fs: %.2f Hz', dataFile, N, Fs), ...
    'FontSize', 12);

%% (B) 차량 이동 UI
animPanel = uipanel(bottomGrid, 'Title', '차량이 도로를 달리는 모습');
animGrid = uigridlayout(animPanel, [2 2]);
animGrid.RowHeight = {'1x', 110};
animGrid.ColumnWidth = {'1x', 260};
animGrid.RowSpacing = 8;

% 차량 이동 그림
axVehicle = uiaxes(animGrid);
axVehicle.Layout.Row = 1;
axVehicle.Layout.Column = 1;
title(axVehicle, 'Vehicle Motion');
xlabel(axVehicle, 'X [m]');
ylabel(axVehicle, 'Y [m]');
grid(axVehicle, 'on');
axis(axVehicle, 'equal');
hold(axVehicle, 'on');

plot(axVehicle, xPos_p, yPos_p, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0);
vehicleTrace = plot(axVehicle, xPos_p(1), yPos_p(1), 'LineWidth', 1.6);
vehicleCenter = plot(axVehicle, xPos_p(1), yPos_p(1), 'o', 'MarkerSize', 6);
vehicleBody = patch(axVehicle, 0, 0, [0.2 0.5 0.9]);

margin = 10;
xMin = min(xPos_p); xMax = max(xPos_p);
yMin = min(yPos_p); yMax = max(yPos_p);

if xMin == xMax
    xMin = xMin - 1;
    xMax = xMax + 1;
end
if yMin == yMax
    yMin = yMin - 1;
    yMax = yMax + 1;
end

xlim(axVehicle, [xMin-margin, xMax+margin]);
ylim(axVehicle, [yMin-margin, yMax+margin]);

% 현재 차량 상태 정보
animInfoPanel = uipanel(animGrid, 'Title', '현재 차량 상태');
animInfoPanel.Layout.Row = 1;
animInfoPanel.Layout.Column = 2;

animInfoGrid = uigridlayout(animInfoPanel, [8 1]);
animInfoGrid.RowHeight = {24,24,24,24,24,24,24,'1x'};

lblCarTime = uilabel(animInfoGrid, 'Text', 'Time: -');
lblCarSpeed = uilabel(animInfoGrid, 'Text', 'Speed: -');
lblCarYaw = uilabel(animInfoGrid, 'Text', 'Yaw: -');
lblCarSlip = uilabel(animInfoGrid, 'Text', 'Slip: -');
lblCarRoad = uilabel(animInfoGrid, 'Text', 'Road Index: -');
lblCarStatus = uilabel(animInfoGrid, 'Text', 'Status: -');
lblCarComment = uilabel(animInfoGrid, 'Text', 'Comment: -', 'WordWrap', 'on');
lblCarGuide = uilabel(animInfoGrid, ...
    'Text', '이 차량 궤적은 휠 평균 속도와 각속도를 적분해 만든 상대 이동 경로입니다.', ...
    'WordWrap', 'on', ...
    'FontSize', 11);

% 재생 제어
controlPanel = uipanel(animGrid, 'Title', '재생 제어');
controlPanel.Layout.Row = 2;
controlPanel.Layout.Column = [1 2];
controlGrid = uigridlayout(controlPanel, [2 4]);
controlGrid.RowHeight = {35, 40};
controlGrid.ColumnWidth = {'1x', 100, 100, 100};

slider = uislider(controlGrid, ...
    'Limits', [1 M], ...
    'Value', 1, ...
    'MajorTicks', []);
slider.Layout.Row = 1;
slider.Layout.Column = [1 4];

btnPlay = uibutton(controlGrid, ...
    'Text', '▶ 재생', ...
    'ButtonPushedFcn', @(btn,event) playAnimation());
btnPlay.Layout.Row = 2; btnPlay.Layout.Column = 2;

btnPause = uibutton(controlGrid, ...
    'Text', '일시정지', ...
    'ButtonPushedFcn', @(btn,event) pauseAnimation());
btnPause.Layout.Row = 2; btnPause.Layout.Column = 3;

btnReset = uibutton(controlGrid, ...
    'Text', '처음으로', ...
    'ButtonPushedFcn', @(btn,event) resetAnimation());
btnReset.Layout.Row = 2; btnReset.Layout.Column = 4;

slider.ValueChangedFcn = @(src,event) updateFrame(round(src.Value));

%% =========================================================
% 12. 초기 표시
%% =========================================================
isPlaying = false;
currentK = 1;
updateFrame(1);

%% =========================================================
% 13. 콜백 함수
%% =========================================================
    function playAnimation()
        isPlaying = true;
        while isPlaying && currentK < M
            currentK = currentK + 1;
            slider.Value = currentK;
            updateFrame(currentK);
            drawnow limitrate;
            pause(0.01);
        end
    end

    function pauseAnimation()
        isPlaying = false;
    end

    function resetAnimation()
        isPlaying = false;
        currentK = 1;
        slider.Value = 1;
        updateFrame(1);
    end

    function updateFrame(k)
        k = max(1, min(M, k));
        currentK = k;

        % 현재 시각 커서 이동
        rawCursor1.Value = tp(k);
        rawCursor2.Value = tp(k);
        rawCursor3.Value = tp(k);

        calcCursor1.Value = tp(k);
        calcCursor2.Value = tp(k);
        calcCursor3.Value = tp(k);
        calcCursor4.Value = tp(k);

        % --------------------------------------------------
        % 1) 원본 데이터 수치 표시
        % --------------------------------------------------
        lblRawFL.Text    = sprintf('FL: %.2f km/h', WHL_FL_p(k));
        lblRawFR.Text    = sprintf('FR: %.2f km/h', WHL_FR_p(k));
        lblRawRL.Text    = sprintf('RL: %.2f km/h', WHL_RL_p(k));
        lblRawRR.Text    = sprintf('RR: %.2f km/h', WHL_RR_p(k));
        lblRawAccel.Text = sprintf('AccelZ: %.4f', AccelZ_p(k));
        lblRawAng.Text   = sprintf('AngRate: %.4f', AngRate_p(k));
        lblRawPitch.Text = sprintf('Pitch: %.3f deg', Pitch_p(k));
        lblRawVdown.Text = sprintf('VelDown: %.4f', VelDown_p(k));
        lblRawVfwd.Text  = sprintf('VelFwd: %.4f', VelFwd_p(k));
        lblRawTime.Text  = sprintf('Time: %.2f s', tp(k));

        % --------------------------------------------------
        % 2) 계산 결과 표시
        % --------------------------------------------------
        lblSlip1.Text = sprintf('Wheel deviation: %.3f', wheelDeviation_p(k));
        lblSlip2.Text = sprintf('Front-Rear diff: %.3f', frontRearDiff_p(k));
        lblSlip3.Text = sprintf('Slip Index: %.3f', slipIndex_p(k));
        if slipIndex_p(k) >= 0.60
            slipJudge = 'Slip 의심 높음';
        elseif slipIndex_p(k) >= 0.30
            slipJudge = 'Slip 의심 보통';
        else
            slipJudge = 'Slip 거의 없음';
        end
        lblSlip4.Text = ['Slip 판단: ', slipJudge];

        lblRms1.Text = sprintf('Filtered AccelZ: %.4f', AccelZ_filt(idxPlot(k)));
        lblRms2.Text = sprintf('RMS: %.4f', vibrationRMS_p(k));
        lblRms3.Text = sprintf('RMS Index: %.3f', rmsIndex_p(k));
        if rmsIndex_p(k) >= 0.60
            rmsJudge = '진동 큼';
        elseif rmsIndex_p(k) >= 0.30
            rmsJudge = '진동 보통';
        else
            rmsJudge = '진동 작음';
        end
        lblRms4.Text = ['진동 상태: ', rmsJudge];

        lblGrade1.Text = sprintf('Grade Angle: %.3f deg', gradeAngle_p(k));
        lblGrade2.Text = sprintf('Grade Index: %.3f', gradeIndex_p(k));
        lblGrade3.Text = sprintf('wGrade: %.3f', wGrade(idxPlot(k)));
        if gradeIndex_p(k) >= 0.60
            gradeJudge = '경사 큼';
        elseif gradeIndex_p(k) >= 0.30
            gradeJudge = '경사 보통';
        else
            gradeJudge = '경사 작음';
        end
        lblGrade4.Text = ['경사 상태: ', gradeJudge];

        lblMu1.Text = sprintf('mu: %.3f', muEst_p(k));
        lblMu2.Text = sprintf('Road Index: %.3f', roadIndex_p(k));
        lblMu3.Text = ['종합 상태: ', roadStatus_p{k}];
        lblMu4.Text = sprintf('설명: Slip %.2f / RMS %.2f / Grade %.2f', ...
            slipIndex_p(k), rmsIndex_p(k), gradeIndex_p(k));

        % --------------------------------------------------
        % 3) 최종 결과 표시
        % --------------------------------------------------
        lblFinalStatus.Text = roadStatus_p{k};
        lblFinalRoad.Text = sprintf('Road Index: %.3f', roadIndex_p(k));
        lblFinalSlip.Text = sprintf('Slip Index: %.3f', slipIndex_p(k));
        lblFinalRms.Text = sprintf('RMS Index: %.3f', rmsIndex_p(k));
        lblFinalGrade.Text = sprintf('Grade Index: %.3f', gradeIndex_p(k));
        lblFinalMu.Text = sprintf('mu: %.3f', muEst_p(k));

        if strcmp(roadStatus_p{k}, '좋음')
            lblFinalStatus.BackgroundColor = [0.84 0.94 0.84];
            lblFinalSummary.Text = '요약: 노면 상태가 비교적 안정적입니다.';
        elseif strcmp(roadStatus_p{k}, '보통')
            lblFinalStatus.BackgroundColor = [0.98 0.91 0.72];
            lblFinalSummary.Text = '요약: 노면 상태가 다소 불안정할 수 있습니다.';
        else
            lblFinalStatus.BackgroundColor = [0.96 0.80 0.80];
            lblFinalSummary.Text = '요약: 노면 상태가 좋지 않아 주의가 필요합니다.';
        end

        % --------------------------------------------------
        % 4) 차량 애니메이션 표시
        % --------------------------------------------------
        vehicleTrace.XData = xPos_p(1:k);
        vehicleTrace.YData = yPos_p(1:k);
        vehicleCenter.XData = xPos_p(k);
        vehicleCenter.YData = yPos_p(k);

        [carX, carY] = makeCarShape(xPos_p(k), yPos_p(k), yawAngle_p(k));
        vehicleBody.XData = carX;
        vehicleBody.YData = carY;

        lblCarTime.Text = sprintf('Time: %.2f s', tp(k));
        lblCarSpeed.Text = sprintf('Speed: %.2f km/h', wheelMean_p(k));
        lblCarYaw.Text = sprintf('Yaw: %.2f deg', rad2deg(yawAngle_p(k)));
        lblCarSlip.Text = sprintf('Slip: %.3f', slipIndex_p(k));
        lblCarRoad.Text = sprintf('Road Index: %.3f', roadIndex_p(k));
        lblCarStatus.Text = ['Status: ', roadStatus_p{k}];

        if strcmp(roadStatus_p{k}, '좋음')
            lblCarComment.Text = 'Comment: 비교적 안정적인 노면';
        elseif strcmp(roadStatus_p{k}, '보통')
            lblCarComment.Text = 'Comment: 주행 가능하지만 변화 감시 필요';
        else
            lblCarComment.Text = 'Comment: 노면 상태가 좋지 않아 주의 필요';
        end
    end

end

%% =========================================================
% Local Function 1. 필수 신호 찾기
%% =========================================================
function x = getRequiredSignal(S, candidates)
for i = 1:length(candidates)
    if isfield(S, candidates{i})
        x = S.(candidates{i});
        return;
    end
end
error('필수 변수를 찾지 못했습니다: %s', strjoin(candidates, ', '));
end

%% =========================================================
% Local Function 2. 선택 신호 찾기
%% =========================================================
function x = getOptionalSignal(S, candidates, defaultValue)
for i = 1:length(candidates)
    if isfield(S, candidates{i})
        x = S.(candidates{i});
        return;
    end
end
x = defaultValue;
end

%% =========================================================
% Local Function 3. 길이 맞추기
%% =========================================================
function y = alignSignal(x, N)
x = double(x(:));

if length(x) >= N
    y = x(1:N);
else
    y = nan(N, 1);
    y(1:length(x)) = x;
    y = fillmissing(y, 'previous');
    y = fillmissing(y, 'next');
end
end

%% =========================================================
% Local Function 4. 차량 모양
%% =========================================================
function [carX, carY] = makeCarShape(x, y, yaw)
L = 4.5;
W = 2.0;

body = [ ...
     L/2,  W/2;
     L/2, -W/2;
    -L/2, -W/2;
    -L/2,  W/2];

R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
bodyRot = (R * body')';

carX = bodyRot(:,1) + x;
carY = bodyRot(:,2) + y;
end