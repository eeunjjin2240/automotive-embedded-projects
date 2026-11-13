%% =====================================================================
%  실시간 도로 상태 시각화 v2 (Real-time Visualization with Controls)
%  - 재생 / 일시정지 / 처음으로 버튼 지원
%  - 큰 지도 영역 + 우측 도로상태 지수 + 게이지
% =====================================================================

clear; clc; close all;
load('road_results.mat');

%% ------------------------------------------------------------------
%  상태 변수 초기화
%% ------------------------------------------------------------------
state.is_playing  = false;
state.cur_idx     = 0;
state.play_speed  = 30;
state.valid_start = find(results.valid, 1, 'first');
state.valid_end   = find(results.valid, 1, 'last');
state.cur_idx     = state.valid_start;

fprintf('재생 구간: %d ~ %d\n', state.valid_start, state.valid_end);

%% ------------------------------------------------------------------
%  Figure 생성
%% ------------------------------------------------------------------
fig = figure('Name', '도로 상태 추정 시스템 (Road Condition Estimator)', ...
    'Position', [50 50 1700 1000], 'Color', [0.97 0.97 0.97], ...
    'CloseRequestFcn', @closeFigCallback);

% 메인 타이틀
annotation(fig, 'textbox', [0.3 0.96 0.4 0.03], ...
    'String', 'Real-time Road Condition Estimator', ...
    'FontSize', 16, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none');

%% ------------------------------------------------------------------
%  레이아웃 구성 (uipanel)
%% ------------------------------------------------------------------
% 상단 행: ① 휠속도 + ② IMU
panel_top_left = uipanel(fig, 'Position', [0.03 0.73 0.47 0.21], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line');
panel_top_right = uipanel(fig, 'Position', [0.51 0.73 0.46 0.21], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line');

% 중상단 행: ③ slip+rms + ④ μ+w_grade
panel_mid_left = uipanel(fig, 'Position', [0.03 0.52 0.47 0.20], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line');
panel_mid_right = uipanel(fig, 'Position', [0.51 0.52 0.46 0.20], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line');

% 큰 지도 (좌측) + 우측: 도로지수(상) + 게이지(하)
panel_map = uipanel(fig, 'Position', [0.03 0.10 0.65 0.41], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line', ...
    'Title', '🚗 도로 위 차량 위치', 'FontSize', 12, 'FontWeight', 'bold');

panel_road_idx = uipanel(fig, 'Position', [0.69 0.31 0.28 0.20], ...
    'BackgroundColor', [1 1 1], 'BorderType', 'line', ...
    'Title', '⑤ 도로 상태 지수', 'FontSize', 11, 'FontWeight', 'bold');

panel_gauge = uipanel(fig, 'Position', [0.69 0.10 0.28 0.20], ...
    'BackgroundColor', [0.98 0.98 0.98], 'BorderType', 'line', ...
    'Title', '🚦 현재 도로 상태', 'FontSize', 11, 'FontWeight', 'bold');

% 하단 컨트롤
panel_ctrl = uipanel(fig, 'Position', [0.03 0.02 0.94 0.07], ...
    'BackgroundColor', [0.92 0.92 0.95], 'BorderType', 'line');

%% ------------------------------------------------------------------
%  ① 4륜 휠 속도 + VS
%% ------------------------------------------------------------------
ax1 = axes('Parent', panel_top_left, 'Position', [0.10 0.20 0.86 0.70]);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
h_wFL = plot(ax1, NaN, NaN, '-', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.2);
h_wFR = plot(ax1, NaN, NaN, '-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2);
h_wRL = plot(ax1, NaN, NaN, '-', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.2);
h_wRR = plot(ax1, NaN, NaN, '-', 'Color', [0.6 0.4 0.8], 'LineWidth', 1.2);
h_vs  = plot(ax1, NaN, NaN, 'k--', 'LineWidth', 1.5);
title(ax1, '① 4륜 휠 속도 + VS (km/h)', 'FontWeight', 'bold');
xlabel(ax1, 'Time (s)'); ylabel(ax1, 'Speed (km/h)');
legend(ax1, {'FL','FR','RL','RR','VS'}, 'Location', 'northwest', 'FontSize', 7);
ylim(ax1, [0 160]);

%% ------------------------------------------------------------------
%  ② IMU AccelZ + Pitch
%% ------------------------------------------------------------------
ax2 = axes('Parent', panel_top_right, 'Position', [0.10 0.20 0.80 0.70]);
yyaxis(ax2, 'left');
h_accZ = plot(ax2, NaN, NaN, '-', 'Color', [0.1 0.4 0.8], 'LineWidth', 1.2);
ylabel(ax2, 'AccelZ (m/s²)');
ylim(ax2, [-15 5]);
yyaxis(ax2, 'right');
h_pitch = plot(ax2, NaN, NaN, '-', 'Color', [0.8 0.4 0.1], 'LineWidth', 1.2);
ylabel(ax2, 'Pitch (deg)');
ylim(ax2, [-15 25]);
grid(ax2, 'on'); box(ax2, 'on');
title(ax2, '② IMU 수직 가속도 / 차량 경사각', 'FontWeight', 'bold');
xlabel(ax2, 'Time (s)');
legend(ax2, {'AccelZ','Pitch'}, 'Location', 'northwest', 'FontSize', 7);

%% ------------------------------------------------------------------
%  ③ Slip + RMS
%% ------------------------------------------------------------------
ax3 = axes('Parent', panel_mid_left, 'Position', [0.10 0.22 0.86 0.68]);
hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
h_slip = plot(ax3, NaN, NaN, '-', 'Color', [0.8 0.3 0.5], 'LineWidth', 1.5);
h_rms = plot(ax3, NaN, NaN, '-', 'Color', [0.3 0.7 0.7], 'LineWidth', 1.5);
title(ax3, '③ 슬립 지수 & 진동 RMS (정규화)', 'FontWeight', 'bold');
xlabel(ax3, 'Time (s)'); ylabel(ax3, 'Index (0-1)');
legend(ax3, {'Slip Index','RMS Vibration'}, 'Location', 'northwest', 'FontSize', 7);
ylim(ax3, [0 1]);

%% ------------------------------------------------------------------
%  ④ μ + w_grade
%% ------------------------------------------------------------------
ax4 = axes('Parent', panel_mid_right, 'Position', [0.10 0.22 0.86 0.68]);
hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');
h_mu = plot(ax4, NaN, NaN, '-', 'Color', [0.6 0.2 0.7], 'LineWidth', 1.5);
h_wg = plot(ax4, NaN, NaN, '-', 'Color', [0.4 0.6 0.2], 'LineWidth', 1.5);
title(ax4, '④ 마찰계수 μ & 경사 가중치 w\_grade', 'FontWeight', 'bold');
xlabel(ax4, 'Time (s)'); ylabel(ax4, 'Value');
legend(ax4, {'μ estimate','w\_grade'}, 'Location', 'northwest', 'FontSize', 7);
ylim(ax4, [0 1.2]);

%% ------------------------------------------------------------------
%  큰 지도 영역
%% ------------------------------------------------------------------
ax_map = axes('Parent', panel_map, 'Position', [0.07 0.10 0.90 0.85]);
hold(ax_map, 'on'); grid(ax_map, 'on'); box(ax_map, 'on');
xlabel(ax_map, 'Longitude (°)'); ylabel(ax_map, 'Latitude (°)');
axis(ax_map, 'equal');

% 전체 경로 (회색 가이드)
valid_path = results.valid;
plot(ax_map, results.posLon(valid_path), results.posLat(valid_path), ...
    '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 1.5, 'HandleVisibility','off');

% 등급별 색상 점 (누적)
h_path_good = scatter(ax_map, NaN, NaN, 30, 'filled', ...
    'MarkerFaceColor', [0.2 0.7 0.3], 'MarkerEdgeColor', 'none', ...
    'MarkerFaceAlpha', 0.7, 'DisplayName', '좋음');
h_path_fair = scatter(ax_map, NaN, NaN, 30, 'filled', ...
    'MarkerFaceColor', [0.95 0.7 0.2], 'MarkerEdgeColor', 'none', ...
    'MarkerFaceAlpha', 0.7, 'DisplayName', '양호');
h_path_bad  = scatter(ax_map, NaN, NaN, 30, 'filled', ...
    'MarkerFaceColor', [0.9 0.2 0.2], 'MarkerEdgeColor', 'none', ...
    'MarkerFaceAlpha', 0.7, 'DisplayName', '나쁨');

% 차량 마커
h_car = plot(ax_map, NaN, NaN, '^', 'MarkerSize', 18, ...
    'MarkerFaceColor', [0.1 0.3 0.9], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

legend(ax_map, [h_path_good, h_path_fair, h_path_bad], ...
    'Location', 'best', 'FontSize', 10);

% 누적 경로
path_good = struct('lon', [], 'lat', []);
path_fair = struct('lon', [], 'lat', []);
path_bad  = struct('lon', [], 'lat', []);

%% ------------------------------------------------------------------
%  ⑤ 도로 상태 지수
%% ------------------------------------------------------------------
TH1 = results.TH1;
TH2 = results.TH2;

ax5 = axes('Parent', panel_road_idx, 'Position', [0.13 0.18 0.83 0.72]);
hold(ax5, 'on'); grid(ax5, 'on'); box(ax5, 'on');
fill(ax5, [0 1 1 0], [0 0 TH1 TH1], [0.7 0.95 0.7], ...
    'EdgeColor','none','FaceAlpha',0.25, 'HandleVisibility','off');
fill(ax5, [0 1 1 0], [TH1 TH1 TH2 TH2], [0.99 0.95 0.7], ...
    'EdgeColor','none','FaceAlpha',0.25, 'HandleVisibility','off');
fill(ax5, [0 1 1 0], [TH2 TH2 1.0 1.0], [0.99 0.7 0.7], ...
    'EdgeColor','none','FaceAlpha',0.25, 'HandleVisibility','off');
yline(ax5, TH1, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);
yline(ax5, TH2, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);

h_road = plot(ax5, NaN, NaN, '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 2);
h_road_now = plot(ax5, NaN, NaN, 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
xlabel(ax5, 'Time (s)'); ylabel(ax5, 'Road Index');
ylim(ax5, [0 1]);

%% ------------------------------------------------------------------
%  현재 도로 상태 게이지
%% ------------------------------------------------------------------
ax_gauge = axes('Parent', panel_gauge, 'Position', [0 0 1 1]);
axis(ax_gauge, 'off');

h_gauge_text = text(ax_gauge, 0.5, 0.65, '대기 중', ...
    'FontSize', 38, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'Color', [0.3 0.3 0.3]);

h_gauge_value = text(ax_gauge, 0.5, 0.32, 'Index: --', ...
    'FontSize', 16, 'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.2]);

h_gauge_speed = text(ax_gauge, 0.5, 0.13, 'Speed: -- km/h', ...
    'FontSize', 13, 'HorizontalAlignment', 'center', 'Color', [0.4 0.4 0.4]);

xlim(ax_gauge, [0 1]); ylim(ax_gauge, [0 1]);

%% ------------------------------------------------------------------
%  하단 컨트롤 (버튼 + 진행률)
%% ------------------------------------------------------------------
% 처음으로 버튼
btn_reset = uicontrol(panel_ctrl, 'Style', 'pushbutton', ...
    'String', '|<<  처음으로', ...
    'Units', 'normalized', 'Position', [0.02 0.20 0.10 0.60], ...
    'FontSize', 13, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.85 0.85 0.95], ...
    'Callback', @resetCallback);

% 재생/일시정지 토글 버튼
btn_play = uicontrol(panel_ctrl, 'Style', 'togglebutton', ...
    'String', '>  재생', ...
    'Units', 'normalized', 'Position', [0.13 0.20 0.10 0.60], ...
    'FontSize', 13, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.85 0.95 0.85], ...
    'Value', 0, ...
    'Callback', @playPauseCallback);

% 재생 속도
uicontrol(panel_ctrl, 'Style', 'text', ...
    'String', '재생 속도:', ...
    'Units', 'normalized', 'Position', [0.24 0.30 0.06 0.40], ...
    'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.95]);

speed_popup = uicontrol(panel_ctrl, 'Style', 'popupmenu', ...
    'String', {'10x','20x','30x','50x','100x'}, ...
    'Units', 'normalized', 'Position', [0.30 0.25 0.05 0.50], ...
    'FontSize', 11, 'Value', 3, ...
    'Callback', @speedCallback);

% 진행률 슬라이더
uicontrol(panel_ctrl, 'Style', 'text', ...
    'String', '진행률:', ...
    'Units', 'normalized', 'Position', [0.36 0.30 0.05 0.40], ...
    'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.95]);

slider_progress = uicontrol(panel_ctrl, 'Style', 'slider', ...
    'Min', state.valid_start, 'Max', state.valid_end, 'Value', state.valid_start, ...
    'Units', 'normalized', 'Position', [0.41 0.35 0.45 0.30], ...
    'Callback', @sliderCallback);

txt_progress = uicontrol(panel_ctrl, 'Style', 'text', ...
    'String', '0% (0.0s / 0.0s)', ...
    'Units', 'normalized', 'Position', [0.87 0.30 0.11 0.40], ...
    'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.95], ...
    'HorizontalAlignment', 'left');

%% ------------------------------------------------------------------
%  핸들 저장
%% ------------------------------------------------------------------
handles.fig = fig;
handles.results = results;
handles.state = state;

handles.h_wFL = h_wFL; handles.h_wFR = h_wFR;
handles.h_wRL = h_wRL; handles.h_wRR = h_wRR; handles.h_vs = h_vs;
handles.h_accZ = h_accZ; handles.h_pitch = h_pitch;
handles.h_slip = h_slip; handles.h_rms = h_rms;
handles.h_mu = h_mu; handles.h_wg = h_wg;
handles.h_road = h_road; handles.h_road_now = h_road_now;
handles.h_path_good = h_path_good;
handles.h_path_fair = h_path_fair;
handles.h_path_bad = h_path_bad;
handles.h_car = h_car;
handles.h_gauge_text = h_gauge_text;
handles.h_gauge_value = h_gauge_value;
handles.h_gauge_speed = h_gauge_speed;
handles.panel_gauge = panel_gauge;

handles.ax1 = ax1; handles.ax2 = ax2;
handles.ax3 = ax3; handles.ax4 = ax4;
handles.ax5 = ax5;

handles.path_good = path_good;
handles.path_fair = path_fair;
handles.path_bad = path_bad;

handles.btn_play = btn_play;
handles.slider_progress = slider_progress;
handles.txt_progress = txt_progress;
handles.speed_popup = speed_popup;

guidata(fig, handles);

%% ------------------------------------------------------------------
%  타이머 (재생 루프용)
%% ------------------------------------------------------------------
t_loop = timer('ExecutionMode', 'fixedRate', ...
    'Period', 0.05, ...
    'TimerFcn', @(~,~) updateFrame(fig));
handles.timer = t_loop;
guidata(fig, handles);

% 첫 프레임 그리기
updateFrameDirect(fig);

fprintf('\n=== 실행 준비 완료 ===\n');
fprintf('  ▶ 재생 버튼을 눌러 시작하세요.\n');
fprintf('  |<< 처음으로 버튼으로 리셋\n');
fprintf('  || 일시정지 후 슬라이더로 위치 조정 가능\n\n');

%% ==================================================================
%  콜백 함수들
%% ==================================================================

function playPauseCallback(src, ~)
    handles = guidata(src);
    if get(src, 'Value') == 1
        set(src, 'String', '||  일시정지', 'BackgroundColor', [0.95 0.85 0.85]);
        handles.state.is_playing = true;
        guidata(src, handles);
        if strcmp(handles.timer.Running, 'off')
            start(handles.timer);
        end
    else
        set(src, 'String', '>  재생', 'BackgroundColor', [0.85 0.95 0.85]);
        handles.state.is_playing = false;
        guidata(src, handles);
        if strcmp(handles.timer.Running, 'on')
            stop(handles.timer);
        end
    end
end

function resetCallback(src, ~)
    handles = guidata(src);
    
    if strcmp(handles.timer.Running, 'on')
        stop(handles.timer);
    end
    set(handles.btn_play, 'Value', 0, 'String', '>  재생', ...
        'BackgroundColor', [0.85 0.95 0.85]);
    handles.state.is_playing = false;
    
    handles.state.cur_idx = handles.state.valid_start;
    
    handles.path_good = struct('lon', [], 'lat', []);
    handles.path_fair = struct('lon', [], 'lat', []);
    handles.path_bad = struct('lon', [], 'lat', []);
    
    set(handles.h_path_good, 'XData', NaN, 'YData', NaN);
    set(handles.h_path_fair, 'XData', NaN, 'YData', NaN);
    set(handles.h_path_bad, 'XData', NaN, 'YData', NaN);
    
    set(handles.slider_progress, 'Value', handles.state.valid_start);
    
    guidata(src, handles);
    updateFrameDirect(handles.fig);
end

function speedCallback(src, ~)
    handles = guidata(src);
    speed_options = [10, 20, 30, 50, 100];
    handles.state.play_speed = speed_options(get(src, 'Value'));
    guidata(src, handles);
end

function sliderCallback(src, ~)
    handles = guidata(src);
    new_idx = round(get(src, 'Value'));
    
    handles.path_good = struct('lon', [], 'lat', []);
    handles.path_fair = struct('lon', [], 'lat', []);
    handles.path_bad = struct('lon', [], 'lat', []);
    
    for k = handles.state.valid_start:new_idx
        if ~handles.results.valid(k), continue; end
        cur_lon = handles.results.posLon(k);
        cur_lat = handles.results.posLat(k);
        cur_grade = handles.results.road_grade(k);
        switch cur_grade
            case 1
                handles.path_good.lon(end+1) = cur_lon;
                handles.path_good.lat(end+1) = cur_lat;
            case 2
                handles.path_fair.lon(end+1) = cur_lon;
                handles.path_fair.lat(end+1) = cur_lat;
            case 3
                handles.path_bad.lon(end+1) = cur_lon;
                handles.path_bad.lat(end+1) = cur_lat;
        end
    end
    
    set(handles.h_path_good, 'XData', handles.path_good.lon, 'YData', handles.path_good.lat);
    set(handles.h_path_fair, 'XData', handles.path_fair.lon, 'YData', handles.path_fair.lat);
    set(handles.h_path_bad, 'XData', handles.path_bad.lon, 'YData', handles.path_bad.lat);
    
    handles.state.cur_idx = new_idx;
    guidata(src, handles);
    
    updateFrameDirect(handles.fig);
end

function updateFrame(fig)
    if ~isvalid(fig), return; end
    handles = guidata(fig);
    
    if ~handles.state.is_playing, return; end
    
    step = max(1, round(handles.state.play_speed / 10));
    handles.state.cur_idx = handles.state.cur_idx + step;
    
    if handles.state.cur_idx >= handles.state.valid_end
        handles.state.cur_idx = handles.state.valid_end;
        handles.state.is_playing = false;
        set(handles.btn_play, 'Value', 0, 'String', '>  재생', ...
            'BackgroundColor', [0.85 0.95 0.85]);
        stop(handles.timer);
    end
    
    guidata(fig, handles);
    updateFrameDirect(fig);
end

function updateFrameDirect(fig)
    if ~isvalid(fig), return; end
    handles = guidata(fig);
    results = handles.results;
    
    k = handles.state.cur_idx;
    if k < 1, k = 1; end
    if k > length(results.t), k = length(results.t); end
    
    WINDOW = 30;
    fs_ds = 10;
    WIN_SIZE = WINDOW * fs_ds;
    
    i_start = max(handles.state.valid_start, k - WIN_SIZE + 1);
    idx = i_start:k;
    if isempty(idx), return; end
    
    t_now = results.t(idx);
    
    set(handles.h_wFL, 'XData', t_now, 'YData', results.whl_FL(idx));
    set(handles.h_wFR, 'XData', t_now, 'YData', results.whl_FR(idx));
    set(handles.h_wRL, 'XData', t_now, 'YData', results.whl_RL(idx));
    set(handles.h_wRR, 'XData', t_now, 'YData', results.whl_RR(idx));
    set(handles.h_vs,  'XData', t_now, 'YData', results.vs(idx));
    xlim(handles.ax1, [t_now(1) t_now(end)+0.1]);
    
    set(handles.h_accZ,  'XData', t_now, 'YData', results.accZ(idx));
    set(handles.h_pitch, 'XData', t_now, 'YData', results.pitch(idx));
    xlim(handles.ax2, [t_now(1) t_now(end)+0.1]);
    
    set(handles.h_slip, 'XData', t_now, 'YData', results.slip_index(idx));
    set(handles.h_rms,  'XData', t_now, 'YData', min(results.rms_z(idx), 1));
    xlim(handles.ax3, [t_now(1) t_now(end)+0.1]);
    
    set(handles.h_mu, 'XData', t_now, 'YData', results.mu_est(idx));
    set(handles.h_wg, 'XData', t_now, 'YData', results.w_grade(idx));
    xlim(handles.ax4, [t_now(1) t_now(end)+0.1]);
    
    set(handles.h_road,     'XData', t_now, 'YData', results.road_index(idx));
    set(handles.h_road_now, 'XData', t_now(end), 'YData', results.road_index(k));
    xlim(handles.ax5, [t_now(1) t_now(end)+0.1]);
    
    cur_lon = results.posLon(k);
    cur_lat = results.posLat(k);
    
    if cur_lon > 0 && cur_lat > 0
        set(handles.h_car, 'XData', cur_lon, 'YData', cur_lat);
        
        if handles.state.is_playing
            cur_grade = results.road_grade(k);
            switch cur_grade
                case 1
                    handles.path_good.lon(end+1) = cur_lon;
                    handles.path_good.lat(end+1) = cur_lat;
                    set(handles.h_path_good, 'XData', handles.path_good.lon, 'YData', handles.path_good.lat);
                case 2
                    handles.path_fair.lon(end+1) = cur_lon;
                    handles.path_fair.lat(end+1) = cur_lat;
                    set(handles.h_path_fair, 'XData', handles.path_fair.lon, 'YData', handles.path_fair.lat);
                case 3
                    handles.path_bad.lon(end+1) = cur_lon;
                    handles.path_bad.lat(end+1) = cur_lat;
                    set(handles.h_path_bad, 'XData', handles.path_bad.lon, 'YData', handles.path_bad.lat);
            end
            guidata(fig, handles);
        end
    end
    
    cur_road_idx = results.road_index(k);
    cur_grade = results.road_grade(k);
    cur_vs = results.vs(k);
    
    switch cur_grade
        case 1
            grade_text = '좋음 (GOOD)';
            grade_color = [0.15 0.6 0.25];
            bg_color = [0.85 0.97 0.85];
        case 2
            grade_text = '양호 (FAIR)';
            grade_color = [0.85 0.6 0.15];
            bg_color = [0.99 0.95 0.8];
        case 3
            grade_text = '나쁨 (BAD)';
            grade_color = [0.85 0.2 0.2];
            bg_color = [0.99 0.85 0.85];
    end
    set(handles.h_gauge_text, 'String', grade_text, 'Color', grade_color);
    set(handles.h_gauge_value, 'String', sprintf('Index: %.3f', cur_road_idx));
    set(handles.h_gauge_speed, 'String', sprintf('Speed: %.0f km/h', cur_vs));
    set(handles.panel_gauge, 'BackgroundColor', bg_color);
    
    set(handles.slider_progress, 'Value', k);
    total_t = results.t(handles.state.valid_end) - results.t(handles.state.valid_start);
    cur_t = results.t(k) - results.t(handles.state.valid_start);
    progress_pct = 100 * (k - handles.state.valid_start) / ...
                  (handles.state.valid_end - handles.state.valid_start);
    set(handles.txt_progress, 'String', ...
        sprintf('%.1f%% (%.0fs / %.0fs)', progress_pct, cur_t, total_t));
    
    drawnow limitrate;
end

function closeFigCallback(src, ~)
    handles = guidata(src);
    if isfield(handles, 'timer') && isvalid(handles.timer)
        stop(handles.timer);
        delete(handles.timer);
    end
    delete(src);
    fprintf('재생 종료.\n');
end
