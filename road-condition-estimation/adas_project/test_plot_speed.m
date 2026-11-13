%% 데이터 불러오기
clear; clc; close all;
load('testset1.mat');

%% step 1) 차량 속도 그리기
figure;
plot(WHL_SPD_FL, 'LineWidth', 1); hold on;
plot(WHL_SPD_FR, 'LineWidth', 1);
plot(WHL_SPD_RL, 'LineWidth', 1);
plot(WHL_SPD_RR, 'LineWidth', 1);

% GPS 속도 그리기
plot(Speed2D, 'k--', 'LineWidth', 1.5);
% GPS 속도 겹처서 그리기
%plot(Speed2D * 3.6, 'k--', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Speed');
title('Wheel Speed vs GPS Vehicle Speed');
legend('WS_{FL}','WS_{FR}','WS_{RL}','WS_{RR}','VS (GPS Speed2D)', ...
       'Location','best');
grid on; hold off;

%% 데이터 불러오기
clear; clc; close all;
load('testset1.mat');

plot(SAS_Angle, 'LineWidth', 1);
hold on; grid on;
plot(CR_Mdps_StrAng, 'LineWidth', 1.5);
title('Steering Angle');

%% 데이터 불러오기
clear; clc; close all;
load('testset1.mat');

%% Step 3) Acceleration data 비교
figure;

% --- Longitudinal (전후방향) ---
subplot(2,1,1);
plot(AccelForward, 'b', 'LineWidth', 0.8); hold on;
plot(LONG_ACCEL, 'r-.', 'LineWidth', 1.2);
ylabel('Long. Accel [m/s^2]');
title('Longitudinal Acceleration');
legend('AccelForward (IMU)', 'LONG\_ACCEL (CAN)', 'Location', 'best');
grid on; hold off;

% --- Lateral (좌우방향) ---
subplot(2,1,2);
plot(AccelLateral, 'b', 'LineWidth', 0.8); hold on;
plot(LAT_ACCEL, 'g--', 'LineWidth', 1.2);
xlabel('Sample');
ylabel('Lat. Accel [m/s^2]');
title('Lateral Acceleration');
legend('AccelLateral (IMU)', 'LAT\_ACCEL (CAN)', 'Location', 'best');
grid on; hold off;