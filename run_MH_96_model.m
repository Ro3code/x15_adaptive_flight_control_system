close all;
% X-15's Longitudinal model
% H = 40000ft
% Mach = 1.2
% alpha = 2deg
% q = 395 lb/ft2
Ta      = 1.636;     % [sec]
Mdelta  = 0.38 *14.25;     % [rad/sec^2]
Chi_a   = 0.1936;    % 
Omega_a = 3.8553;    % [rad/sec]
Aircraft = tf(-Mdelta/Ta*[Ta, 1], [1, 2*Chi_a*Omega_a, Omega_a^2]);
% Actuators data
Wn_act = 90;
chi    = 0.7;
% Band-pass filter
BandPass = tf([2*chi*Wn_act, 0], [1, 2*chi*Wn_act, Wn_act^2]);
% Notch-filter for the full-wave rectifier
Notch = tf([1, 0, (2*Wn_act)^2], [1, 2*chi*(2*Wn_act), (2*Wn_act)^2]);
% MH-96's parameters
Ka    = 1;
Gamma = 40;
T     = 0.01;
% Desired Dynamics First order model
Tau = 0.5;
RefModel = tf(-1, [Tau, 1]);
% Simulation time
STOP_TIME = 100;
% Set the initial value of the Kq gain
Kq0 = 1;
% Linearize the open-loop model (negative feedback)
[a, b, c, d] = linmod('MH_96');
% Nichols plot
figure(1);
hold all;
nichols(ss(a, b, c, d, 'Name', 'MH-96 Open-Loop'), logspace(-2, 4, 1000))
grid on;
%% Compute the stability margins of the closed-loop system
[Gm, Pm] = margin(ss(a, b, c, d, 'Name', 'MH-96 Open-Loop'));
%% Simulate the closed-loop system
sim('MH_96');
%% Plot the closed-loop simulation results
figure('Color', 0.2 * [1, 1, 1]);
hs = subplot(3,1,1);
hs.Color            = 'k';
hs.GridColor        = 0.8 * [1, 1, 1];
hs.XColor           = 0.8 * [1, 1, 1];
hs.YColor           = 0.8 * [1, 1, 1];
hs.MinorGridColor   = 0.8 * [1, 1, 1];
hs.FontSize         = 16;
hold all;
plot(t, qm, 'y', 'LineWidth', 2, 'DisplayName', '$$q_m$$')
plot(t,  q, 'c', 'LineWidth', 2, 'DisplayName', '$$q$$')
grid on; box on;
legend('show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
ylabel('$$Pitch\,Rate\, [rad/s]$$', 'Interpreter', 'Latex')
hs = subplot(3,1,2);
hs.Color            = 'k';
hs.GridColor        = 0.8 * [1, 1, 1];
hs.XColor           = 0.8 * [1, 1, 1];
hs.YColor           = 0.8 * [1, 1, 1];
hs.MinorGridColor   = 0.8 * [1, 1, 1];
hs.FontSize         = 16;
hold all;
plot(t,     u, 'y', 'LineWidth', 2, 'DisplayName', '$$u$$')
plot(t, mod_u, 'c', 'LineWidth', 2, 'DisplayName', '$$||u||$$')
plot([0, t(end)], T * [1, 1], '--g', 'LineWidth', 2, 'DisplayName', '$$T$$')
grid on; box on;
legend('show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
hs = subplot(3,1,3);
hs.Color            = 'k';
hs.GridColor        = 0.8 * [1, 1, 1];
hs.XColor           = 0.8 * [1, 1, 1];
hs.YColor           = 0.8 * [1, 1, 1];
hs.MinorGridColor   = 0.8 * [1, 1, 1];
hs.FontSize         = 16;
hold all;
plot(t, Kq, 'y', 'LineWidth', 2, 'DisplayName', '$$Kq$$')
plot([0, t(end)], Gm * [1, 1], '--m', 'LineWidth', 2, 'DisplayName', '$$Kq_{Instability}$$')
grid on; box on;
legend('show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
xlabel('$$t$$', 'Interpreter', 'Latex')
ylabel('$$K$$', 'Interpreter', 'Latex')
