clear;
clc; 
%  turbine protection control system - PID control vs Open-Loop
%   This Script using XSteam only
% use one strong quality-critical condenser disturbance
% Author     by : Ehab Osama Hamed

if exist('XSteam','file') ~= 2
    error('XSteam not found. Add the XSteam folder to the MATLAB path first.');
end

fprintf('   SIMPLE TURBINE PROTECTION CONTROL SYSTEM\n');
fprintf('   Open-loop vs PID on steam quality\n');
fprintf('   Simple, efficient, and visually separated response\n');
%%  1) NOMINAL PARAMETERS

p.P_rh0  = 2.00e6;    % [Pa]
p.T_rh0  = 773.15;    % [K]
p.P_c0   = 10.0e3;    % [Pa]
p.m_dot0 = 50.0;      % [kg/s]
p.eta_LP = 0.86;
p.cp_sup = 2180;      % [J/kgK]

p.T_b0   = 773.15;    % [K]
p.W_nom  = 120.0;     % [MW]

p.tau_b  = 80;        % [s]
p.tau_rh = 30;        % [s]
p.tau_c  = 90;        % [s]
p.tau_f  = 18;        % [s]

p.Q_nom = 0.80;
p.Q_lo  = 0.35;
p.Q_hi  = 1.00;

p.x_min  = 0.90;
p.x_crit = 0.885;

%%  2) TIME BASE AND METRICS SETUP

p.t_end = 1500;           % [s] = 20 min
p.dt    = 0.5;            % [s]
p.t     = (0:p.dt:p.t_end)';
p.N     = numel(p.t);
p.t_min = p.t / 60;

p.metric_start_s = 60;
p.final_window_s = 60;
p.settle_hold_s  = 45;
p.settle_band_x  = 0.0040;

p.power_scale = p.W_nom / (1e-6 * p.m_dot0 * p.cp_sup * (p.T_rh0 - 373.15) * p.eta_LP);
p.x_ref = steam_quality_xsteam(p.T_rh0, p.P_rh0, p.P_c0, p.eta_LP);

%%  3) LONG, DISCRIMINATING DISTURBANCES

pulse_boiler1 = smooth_pulse(p.t,  60, 280, 14, 45);
pulse_boiler2 = smooth_pulse(p.t, 520, 720, 18, 60);
pulse_load1   = smooth_pulse(p.t, 120, 360, 20, 50);
pulse_load2   = smooth_pulse(p.t, 640, 860, 25, 50);
pulse_cond1   = smooth_pulse(p.t, 180, 520, 70, 90);
pulse_cond2   = smooth_pulse(p.t, 420, 960, 110, 130);
pulse_flow1   = smooth_pulse(p.t, 150, 340, 20, 40);
pulse_flow2   = smooth_pulse(p.t, 660, 900, 25, 50);

p.d_boiler = clamp(1 - 0.30 * pulse_boiler1 - 0.10 * pulse_boiler2, 0.55, 1.05);
p.d_load   = clamp(1 + 0.08 * pulse_load1   + 0.10 * pulse_load2,   1.00, 1.24);
p.d_cond   = clamp(1 + 0.10 * pulse_cond1   + 0.42 * pulse_cond2,   1.00, 1.58);
p.d_flow   = clamp(1 + 0.02 * pulse_flow1   + 0.03 * pulse_flow2,   0.98, 1.08);

%%  4) CONTROLLER SETTINGS

% Open-loop: intentionally weak fixed setting
ctrl_open.name   = 'Open-loop';
ctrl_open.mode   = 'open';
ctrl_open.Q_open = 0.76;

% PID: direct single-loop quality controller
ctrl_pid.name    = 'PID-only';
ctrl_pid.mode    = 'pid';
ctrl_pid.Kp      = 6.5;
ctrl_pid.Ki      = 0.012;
ctrl_pid.Kd      = 0.050;
ctrl_pid.int_lim = 8.0;
ctrl_pid.tau_x   = 12.0;
ctrl_pid.tau_q   = 10.0;
ctrl_pid.dQ_max  = 0.020;
%%  5) SIMULATION

res_ol  = simulate_case(p, ctrl_open);
res_pid = simulate_case(p, ctrl_pid);

m_ol  = compute_metrics(p, res_ol);
m_pid = compute_metrics(p, res_pid);

fprintf('%-18s %12s %12s %12s %12s %10s\n', ...
    'Controller', 'Settling', 'IAE', 'Unsafe', 'Critical', 'Min x');
fprintf('%s\n', repmat('-', 1, 82));
print_metric_row(m_ol);
print_metric_row(m_pid);

fprintf('\nFinal steam quality:\n');
fprintf('  Open-loop : %.4f\n', m_ol.final_x);
fprintf('  PID-only  : %.4f\n', m_pid.final_x);
%%  6) SUMMARY NUMBERS

iae_gain_pct      = max(0, 100 * (m_ol.iae_x       - m_pid.iae_x)       / max(m_ol.iae_x, eps));
unsafe_gain_pct   = max(0, 100 * (m_ol.unsafe_time - m_pid.unsafe_time) / max(m_ol.unsafe_time, eps));
critical_gain_pct = max(0, 100 * (m_ol.critical_time - m_pid.critical_time) / max(m_ol.critical_time, eps));
finalx_gain_pct   = max(0, 100 * (m_ol.final_x_err - m_pid.final_x_err) / max(m_ol.final_x_err, eps));

fprintf('\n========================================================\n');
fprintf('  FINAL RESPONSE SUMMARY\n');
fprintf('========================================================\n');
fprintf('Reference quality x0           : %.4f\n', p.x_ref);
fprintf('Open-loop unsafe time          : %.1f s\n', m_ol.unsafe_time);
fprintf('PID-only unsafe time           : %.1f s\n', m_pid.unsafe_time);
fprintf('Open-loop critical time        : %.1f s\n', m_ol.critical_time);
fprintf('PID-only critical time         : %.1f s\n', m_pid.critical_time);
fprintf('Open-loop IAE                 : %.2f\n', m_ol.iae_x);
fprintf('PID-only IAE                  : %.2f\n', m_pid.iae_x);
fprintf('--------------------------------------------------------\n');
fprintf('PID IAE improvement            : %.1f %%\n', iae_gain_pct);
fprintf('PID unsafe-time improvement    : %.1f %%\n', unsafe_gain_pct);
fprintf('PID critical-time improvement  : %.1f %%\n', critical_gain_pct);
fprintf('PID final-x improvement        : %.1f %%\n\n', finalx_gain_pct);
%%  7) PLOTS

C_pid      = [0.00 0.45 0.70];
C_open     = [0.82 0.16 0.16];
C_safe     = [0.20 0.70 0.20];
C_danger   = [0.96 0.86 0.86];
C_crit     = [0.96 0.74 0.56];
C_warn     = [1.00 0.96 0.82];
C_gray1    = [0.35 0.35 0.35];
C_gray2    = [0.50 0.50 0.50];
C_gray3    = [0.65 0.65 0.65];

lw = 2.0;
fs = 10;
fs_title = 11;

summary_note = sprintf([ ...
    'Open-loop below x_{min}: %.1f s\n' ...
    'PID-only below x_{min}: %.1f s\n' ...
    'Open-loop min x = %.4f | PID min x = %.4f'], ...
    m_ol.unsafe_time, m_pid.unsafe_time, m_ol.min_x, m_pid.min_x);

fig1 = figure('Name','Steam Quality Response','Color','w','Position',[40 40 1240 620]);
ax1 = axes(fig1);
hold on;

h_crit = fill([p.t_min(1) p.t_min(end) p.t_min(end) p.t_min(1)], ...
              [0 0 p.x_crit p.x_crit], ...
              C_crit, 'EdgeColor','none', 'FaceAlpha',0.18, ...
              'DisplayName',sprintf('Critical zone (x < %.3f)', p.x_crit));

h_dng = fill([p.t_min(1) p.t_min(end) p.t_min(end) p.t_min(1)], ...
             [p.x_crit p.x_crit p.x_min p.x_min], ...
             C_danger, 'EdgeColor','none', 'FaceAlpha',0.28, ...
             'DisplayName',sprintf('Unsafe zone (%.3f <= x < %.2f)', p.x_crit, p.x_min));

h_tgt = fill([p.t_min(1) p.t_min(end) p.t_min(end) p.t_min(1)], ...
             [p.x_ref 1.02 1.02 p.x_ref], ...
             C_safe, 'EdgeColor','none', 'FaceAlpha',0.10, ...
             'DisplayName','Nominal target band');

control_gain_mask = res_ol.x < p.x_min & res_pid.x >= p.x_min;
control_gain_segments = true_segments(control_gain_mask);
for ii = 1:size(control_gain_segments, 1)
    i1 = control_gain_segments(ii, 1);
    i2 = control_gain_segments(ii, 2);
    patch([p.t_min(i1) p.t_min(i2) p.t_min(i2) p.t_min(i1)], ...
          [0.80 0.80 1.01 1.01], C_warn, ...
          'EdgeColor','none', 'FaceAlpha',0.22, 'HandleVisibility','off');
end

h_ol = plot(p.t_min, res_ol.x, '-', 'Color',C_open, 'LineWidth',lw+0.3, ...
            'DisplayName','Open-loop');
h_pid = plot(p.t_min, res_pid.x, '-', 'Color',C_pid, 'LineWidth',lw+0.2, ...
             'DisplayName','PID-only');

h_lim = yline(p.x_min, '--', 'Color',C_open, 'LineWidth',1.4, ...
      'Label','x_{min} = 0.90', 'LabelHorizontalAlignment','left', ...
      'DisplayName','Safety limit');
h_ref = yline(p.x_ref, ':', 'Color',C_safe, 'LineWidth',1.2, ...
      'Label',sprintf('x_0 = %.4f', p.x_ref), 'LabelHorizontalAlignment','left', ...
      'DisplayName','Reference quality');

xline(1.0, ':', 'Color',C_gray1, 'LineWidth',0.9, 'HandleVisibility','off');
xline(2.0, ':', 'Color',C_gray2, 'LineWidth',0.9, 'HandleVisibility','off');
xline(3.0, ':', 'Color',C_gray3, 'LineWidth',0.9, 'HandleVisibility','off');
xline(7.0, ':', 'Color',[0.45 0.45 0.45], 'LineWidth',0.9, 'HandleVisibility','off');
xline(16.0, ':', 'Color',[0.45 0.45 0.45], 'LineWidth',0.9, 'HandleVisibility','off');
text(1.03, 1.000, 'Boiler disturbance', 'Rotation',90, 'Color',C_gray1, 'FontSize',7.5, 'VerticalAlignment','top');
text(2.03, 1.000, 'Load rise',          'Rotation',90, 'Color',C_gray2, 'FontSize',7.5, 'VerticalAlignment','top');
text(3.03, 1.000, 'Condenser drift',    'Rotation',90, 'Color',C_gray3, 'FontSize',7.5, 'VerticalAlignment','top');
text(7.03, 1.000, 'Quality-critical peak', 'Rotation',90, 'Color',[0.45 0.45 0.45], 'FontSize',7.5, 'VerticalAlignment','top');
text(16.03,1.000, 'Disturbances fade',  'Rotation',90, 'Color',[0.45 0.45 0.45], 'FontSize',7.5, 'VerticalAlignment','top');

text(0.02, 0.05, summary_note, 'Units','normalized', 'FontSize',9, ...
     'BackgroundColor',[1 1 1], 'EdgeColor',[0.7 0.7 0.7]);
text(0.05, 0.18, sprintf('Open-loop final x = %.4f', m_ol.final_x), ...
     'Units','normalized', 'Color',C_open, 'FontSize',9, ...
     'FontWeight','bold', 'BackgroundColor',[1.00 0.94 0.94]);
text(0.55, 0.84, sprintf('PID final x = %.4f, back near x_0', m_pid.final_x), ...
     'Units','normalized', 'Color',C_pid, 'FontSize',9, ...
     'FontWeight','bold', 'BackgroundColor',[0.90 0.96 1.00]);

if ~isempty(control_gain_segments)
    anchor = control_gain_segments(1, 1);
    text(p.t_min(anchor) + 0.18, 0.892, ...
         'Control power appears here: PID stays safe while open-loop fails', ...
         'Color',[0.40 0.20 0.00], 'FontSize',9, 'FontWeight','bold');
end

hold off;
ylim([0.84 1.005]);
xlim([0 p.t_end/60]);
xlabel('Time [min]', 'FontSize',fs);
ylabel('LP turbine exit steam quality x [-]', 'FontSize',fs);
title('Open-loop vs PID: Steam-Quality Protection Performance', 'FontSize',fs_title, 'FontWeight','bold');
legend([h_crit h_dng h_tgt h_ol h_pid h_lim h_ref], ...
       'Location','southwest', 'FontSize',fs-1, 'NumColumns',2);
grid on; grid minor;
set(ax1, 'FontSize',fs, 'Box','on', 'GridAlpha',0.25);

fig2 = figure('Name','Control and Plant Variables','Color','w','Position',[55 55 1220 720]);

subplot(2,2,1);
hold on;
hQ_ol  = yline(ctrl_open.Q_open, '-', 'Color',C_open, 'LineWidth',1.3, ...
               'Label',sprintf('Open-loop fixed = %.2f', ctrl_open.Q_open), ...
               'LabelHorizontalAlignment','right');
hQ_pid = plot(p.t_min, res_pid.Q, '-', 'Color',C_pid, 'LineWidth',lw, 'DisplayName','PID Q_{rh}');
hold off;
ylabel('Q_{rh} [p.u.]', 'FontSize',fs);
title('Controller Action', 'FontSize',fs_title, 'FontWeight','bold');
legend([hQ_ol hQ_pid], {'Open-loop fixed','PID modulation'}, 'FontSize',fs-1, 'Location','northwest');
ylim([0.34 1.02]);
grid on; grid minor; set(gca,'FontSize',fs);

subplot(2,2,2);
hold on;
hT_ol  = plot(p.t_min, res_ol.T_rh - 273.15, '-', 'Color',C_open, 'LineWidth',1.6, 'DisplayName','Open-loop T_{rh}');
hT_pid = plot(p.t_min, res_pid.T_rh - 273.15, '-', 'Color',C_pid, 'LineWidth',1.9, 'DisplayName','PID T_{rh}');
yline(p.T_rh0 - 273.15, ':', 'Color',[0.40 0.40 0.40], 'LineWidth',1.0, 'HandleVisibility','off');
hold off;
ylabel('T_{rh} [degC]', 'FontSize',fs);
title('Reheat Temperature', 'FontSize',fs_title, 'FontWeight','bold');
legend([hT_ol hT_pid], 'FontSize',fs-1, 'Location','southwest');
grid on; grid minor; set(gca,'FontSize',fs);

subplot(2,2,3);
hold on;
hPc_ol  = plot(p.t_min, res_ol.P_c / 1e3, '-', 'Color',C_open, 'LineWidth',1.6, 'DisplayName','Open-loop P_c');
hPc_pid = plot(p.t_min, res_pid.P_c / 1e3, '-', 'Color',C_pid, 'LineWidth',1.9, 'DisplayName','PID P_c');
yline(p.P_c0 / 1e3, ':', 'Color',[0.40 0.40 0.40], 'LineWidth',1.0, 'HandleVisibility','off');
hold off;
xlabel('Time [min]', 'FontSize',fs);
ylabel('Condenser pressure [kPa]', 'FontSize',fs);
title('Quality-Critical Disturbance', 'FontSize',fs_title, 'FontWeight','bold');
legend([hPc_ol hPc_pid], 'FontSize',fs-1, 'Location','northwest');
grid on; grid minor; set(gca,'FontSize',fs);

subplot(2,2,4);
hold on;
hW_ol  = plot(p.t_min, res_ol.W, '-', 'Color',C_open, 'LineWidth',1.6, 'DisplayName','Open-loop W_{net}');
hW_pid = plot(p.t_min, res_pid.W, '-', 'Color',C_pid, 'LineWidth',1.9, 'DisplayName','PID W_{net}');
yline(p.W_nom, ':', 'Color',[0.40 0.40 0.40], 'LineWidth',1.0, 'HandleVisibility','off');
hold off;
xlabel('Time [min]', 'FontSize',fs);
ylabel('W_{net} [MW]', 'FontSize',fs);
title('Net Power', 'FontSize',fs_title, 'FontWeight','bold');
legend([hW_ol hW_pid], 'FontSize',fs-1, 'Location','southwest');
grid on; grid minor; set(gca,'FontSize',fs);

fig3 = figure('Name','Performance Dashboard','Color','w','Position',[70 70 1140 420]);

subplot(1,4,1);
settling_vals = [timeout_if_nan(m_ol.settling_s, p), timeout_if_nan(m_pid.settling_s, p)];
settling_labels = {m_ol.settling_bar, m_pid.settling_bar};
br1 = bar(1:2, settling_vals, 0.55);
br1.FaceColor = 'flat';
br1.CData = [C_open; C_pid];
xticks(1:2); xticklabels({'Open-loop','PID-only'});
ylabel('Settling time [s]', 'FontSize',fs);
title('Settling', 'FontSize',fs_title, 'FontWeight','bold');
ylim([0 max(settling_vals)*1.18 + 1]);
for ii = 1:2
    text(ii, settling_vals(ii) + max(settling_vals)*0.05 + 1, settling_labels{ii}, ...
         'HorizontalAlignment','center', 'FontSize',fs-1, 'FontWeight','bold');
end
grid on; set(gca,'FontSize',fs);

subplot(1,4,2);
iae_vals = [m_ol.iae_x, m_pid.iae_x];
br2 = bar(1:2, iae_vals, 0.55);
br2.FaceColor = 'flat';
br2.CData = [C_open; C_pid];
xticks(1:2); xticklabels({'Open-loop','PID-only'});
ylabel('IAE of |x - x_0|', 'FontSize',fs);
title('IAE', 'FontSize',fs_title, 'FontWeight','bold');
ylim([0 max(iae_vals)*1.25 + 1e-3]);
for ii = 1:2
    text(ii, iae_vals(ii) + max(iae_vals)*0.05 + 1e-3, sprintf('%.2f', iae_vals(ii)), ...
         'HorizontalAlignment','center', 'FontSize',fs-1, 'FontWeight','bold');
end
grid on; set(gca,'FontSize',fs);

%%  LOCAL FUNCTIONS

function res = simulate_case(p, ctrl)
    N = p.N;
    dt = p.dt;

    res.name = ctrl.name;
    res.T_b  = zeros(N,1); res.T_b(1)  = p.T_b0;
    res.T_rh = zeros(N,1); res.T_rh(1) = p.T_rh0;
    res.P_c  = zeros(N,1); res.P_c(1)  = p.P_c0;
    res.m_s  = zeros(N,1); res.m_s(1)  = p.m_dot0;
    res.x    = zeros(N,1); res.x(1)    = p.x_ref;
    res.W    = zeros(N,1); res.W(1)    = p.W_nom;
    res.Q    = zeros(N,1);

    if strcmp(ctrl.mode, 'open')
        res.Q(:) = ctrl.Q_open;
    else
        res.Q(1) = p.Q_nom;
    end

    x_int = 0.0;
    x_f = p.x_ref;
    x_f_prev = x_f;

    for k = 1:N-1
        x_true = steam_quality_xsteam(res.T_rh(k), p.P_rh0, res.P_c(k), p.eta_LP);
        res.x(k) = x_true;

        if strcmp(ctrl.mode, 'open')
            Q_next = ctrl.Q_open;
        else
            x_f = x_f + dt * (x_true - x_f) / max(ctrl.tau_x, dt);
            dx_f = (x_f - x_f_prev) / dt;
            x_f_prev = x_f;

            e = p.x_ref - x_f;
            if ~((res.Q(k) >= p.Q_hi - 1e-6 && e > 0) || ...
                 (res.Q(k) <= p.Q_lo + 1e-6 && e < 0))
                x_int = clamp(x_int + e * dt, -ctrl.int_lim, ctrl.int_lim);
            end

            Q_target = p.Q_nom + ctrl.Kp * e + ctrl.Ki * x_int - ctrl.Kd * dx_f;
            Q_target = clamp(Q_target, p.Q_lo, p.Q_hi);
            Q_smooth = res.Q(k) + dt * (Q_target - res.Q(k)) / ctrl.tau_q;
            Q_next = clamp(rate_limit(res.Q(k), Q_smooth, ctrl.dQ_max), p.Q_lo, p.Q_hi);
        end

        res.Q(k+1) = Q_next;

        T_b_eq = p.T_b0 ...
               + 230 * (p.d_boiler(k) - 1) ...
               - 70  * (p.d_load(k) - 1);
        T_b_eq = clamp(T_b_eq, p.T_b0 - 280, p.T_b0 + 20);
        res.T_b(k+1) = res.T_b(k) + dt * (T_b_eq - res.T_b(k)) / p.tau_b;

        T_rh_eq = p.T_rh0 ...
                + 0.78 * (res.T_b(k+1) - p.T_b0) ...
                + 260  * (res.Q(k+1) - p.Q_nom) ...
                - 25   * (p.d_load(k) - 1) ...
                - 6    * (p.d_cond(k) - 1);
        T_rh_eq = clamp(T_rh_eq, p.T_rh0 - 260, p.T_rh0 + 120);
        res.T_rh(k+1) = res.T_rh(k) + dt * (T_rh_eq - res.T_rh(k)) / p.tau_rh;

        P_c_eq = p.P_c0 * p.d_cond(k) * (1 + 0.12 * (p.d_load(k) - 1));
        res.P_c(k+1) = res.P_c(k) + dt * (P_c_eq - res.P_c(k)) / p.tau_c;

        m_eq = p.m_dot0 * p.d_flow(k) * (1 + 0.45 * (p.d_load(k) - 1));
        res.m_s(k+1) = res.m_s(k) + dt * (m_eq - res.m_s(k)) / p.tau_f;

        p_cond_penalty = 1 - 0.10 * max(res.P_c(k+1) / p.P_c0 - 1, 0);
        res.W(k+1) = p.power_scale * 1e-6 * res.m_s(k+1) * p.cp_sup ...
                   * max(res.T_rh(k+1) - 373.15, 10) * p.eta_LP * p_cond_penalty;
    end

    res.x(N) = steam_quality_xsteam(res.T_rh(N), p.P_rh0, res.P_c(N), p.eta_LP);
end

function m = compute_metrics(p, res)
    mask = p.t >= p.metric_start_s;
    tail = p.t >= max(p.metric_start_s, p.t_end - p.final_window_s);
    hold_n = max(1, round(p.settle_hold_s / p.dt));

    m.name = res.name;
    m.settling_s   = settling_time_after_departure(p.t, res.x, p.metric_start_s, p.x_ref, p.settle_band_x, hold_n);
    m.iae_x        = trapz(p.t(mask), abs(res.x(mask) - p.x_ref));
    m.unsafe_time  = sum(res.x(mask) < p.x_min)  * p.dt;
    m.critical_time = sum(res.x(mask) < p.x_crit) * p.dt;
    m.min_x        = min(res.x(mask));
    m.final_x      = mean(res.x(tail));
    m.final_W      = mean(res.W(tail));
    m.final_x_err  = abs(m.final_x - p.x_ref);
    m.final_W_err  = abs(m.final_W - p.W_nom);

    if isnan(m.settling_s)
        m.settling_label = 'No settle';
        m.settling_bar   = 'NS';
    else
        m.settling_label = sprintf('%.1f s', m.settling_s);
        m.settling_bar   = sprintf('%.0f s', m.settling_s);
    end
end

function print_metric_row(m)
    fprintf('%-18s %12s %12.2f %12.1f %12.1f %10.4f\n', ...
        m.name, m.settling_bar, m.iae_x, m.unsafe_time, m.critical_time, m.min_x);
end

function val = timeout_if_nan(ts, p)
    if isnan(ts)
        val = p.t_end - p.metric_start_s;
    else
        val = ts;
    end
end

function ts = settling_time_after_departure(t_vec, x_vec, t_start, x_ref, band, hold_n)
    ts = NaN;
    idx0 = find(t_vec >= t_start, 1, 'first');
    if isempty(idx0)
        return;
    end

    first_out = find(abs(x_vec(idx0:end) - x_ref) > band, 1, 'first');
    if isempty(first_out)
        ts = 0.0;
        return;
    end

    idx_begin = idx0 + first_out - 1;
    for idx = idx_begin:(numel(t_vec) - hold_n + 1)
        if all(abs(x_vec(idx:idx+hold_n-1) - x_ref) <= band)
            ts = t_vec(idx) - t_start;
            return;
        end
    end
end

function segs = true_segments(mask)
    edges = diff([false; mask(:); false]);
    starts = find(edges == 1);
    stops  = find(edges == -1) - 1;
    segs = [starts stops];
end

function x = steam_quality_xsteam(T_rh_K, P_rh_Pa, P_c_Pa, eta)
    p_rh_bar = P_rh_Pa / 1e5;
    p_c_bar  = P_c_Pa  / 1e5;
    T_rh_C   = T_rh_K - 273.15;

    h3  = XSteam('h_pt', p_rh_bar, T_rh_C);
    s3  = XSteam('s_pt', p_rh_bar, T_rh_C);
    h4s = XSteam('h_ps', p_c_bar, s3);
    h4  = h3 - eta * (h3 - h4s);

    hf = XSteam('hL_p', p_c_bar);
    hg = XSteam('hV_p', p_c_bar);

    if h4 >= hg
        x = 1.0;
    elseif h4 <= hf
        x = 0.0;
    else
        x = (h4 - hf) / (hg - hf);
    end

    x = clamp(x, 0, 1);
end

function y = smooth_pulse(t, t_on, t_off, tau_on, tau_off)
    rise = 0.5 * (1 + tanh((t - t_on)  / max(tau_on,  eps)));
    fall = 0.5 * (1 + tanh((t - t_off) / max(tau_off, eps)));
    y = clamp(rise - fall, 0, 1);
end

function y = rate_limit(previous, desired, delta_max)
    y = previous + min(max(desired - previous, -delta_max), delta_max);
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end