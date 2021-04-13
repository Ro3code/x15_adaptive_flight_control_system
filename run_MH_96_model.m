close all;
%% X-15's Longitudinal model
% H = 40000ft
% Mach = 1.2
% alpha = 2deg
% q = 395 lb/ft2
Ta      = 1.636;     % [sec]
Mdelta  = 1*14.25;     % [rad/sec^2]
Chi_a   = 0.1936;    %
Omega_a = 3.8553;    % [rad/sec]
Aircraft = tf(-Mdelta/Ta*[Ta, 1], [1, 2*Chi_a*Omega_a, Omega_a^2]);
% Actuators data
Wn_act = 90;
chi    = 0.7;
Actuator = tf((Wn_act)^2, [1, 2*chi*(2*Wn_act), (2*Wn_act)^2]);
% Band-pass filter
BandPass = tf([2*chi*Wn_act, 0], [1, 2*chi*Wn_act, Wn_act^2]);
% Notch-filter for the full-wave rectifier
Notch = tf([1, 0, (2*Wn_act)^2], [1, 2*chi*(2*Wn_act), (2*Wn_act)^2]);
% MH-96's parameters
Ka    = 1;
Gamma = 150;
T     = 0.01;
% Desired Dynamics First order model
Tau      = 0.5;
RefModel = tf(-1, [Tau, 1]);
% Simulation time
STOP_TIME = 20;
% Set the initial value of the Kq gain
Kq0 = 0.0;
% Linearize the open-loop model (negative feedback)
[a, b, c, d] = linmod('MH_96');
% Nichols plot
figure(1);
hold all;
nichols(ss(a, b, c, d, 'Name', 'MH-96 Open-Loop'), logspace(-2, 4, 1000))
grid on;
% Compute the stability margins of the closed-loop system
[Gm, Pm] = margin(ss(a, b, c, d, 'Name', 'MH-96 Open-Loop'));

%% Simulate the closed-loop system
sim('MH_96');

% Plot the closed-loop simulation results
isaveGIF = 1;

if isaveGIF == 0
    figure('Color', 0.2 * [1, 1, 1]);
    hs = subplot(4,1,1);
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
    hs = subplot(4,1,2);
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
    hs = subplot(4,1,3);
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
    ylabel('$$K$$', 'Interpreter', 'Latex')
    
    hs = subplot(4,1,4);
    hs.Color            = 'k';
    hs.GridColor        = 0.8 * [1, 1, 1];
    hs.XColor           = 0.8 * [1, 1, 1];
    hs.YColor           = 0.8 * [1, 1, 1];
    hs.MinorGridColor   = 0.8 * [1, 1, 1];
    hs.FontSize         = 16;
    hold all;
    plot(t, theta_m * 180 / pi, 'y', 'LineWidth', 2, 'DisplayName', '$$\theta_m$$')
    plot(t, theta * 180 / pi, 'c', 'LineWidth', 2, 'DisplayName', '$$\theta$$')
    grid on; box on;
    legend('show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
    xlabel('$$t$$', 'Interpreter', 'Latex')
    ylabel('$$\frac{\theta(s)\tau s}{\tau s + 1}$$', 'Interpreter', 'Latex')
    
else
    
    filename = 'mh96_animation.gif';
    
    dt = mean(diff(t));
    
    h = figure('Color', 0.2 * [1, 1, 1], 'Units', 'normalized', 'Position', [0, 0.05, 0.5, 0.7]);
    
    hs(1) = subplot(3,1,1);
    hs(1).Color            = 'k';
    hs(1).GridColor        = 0.8 * [1, 1, 1];
    hs(1).XColor           = 0.8 * [1, 1, 1];
    hs(1).YColor           = 0.8 * [1, 1, 1];
    hs(1).MinorGridColor   = 0.8 * [1, 1, 1];
    hs(1).FontSize         = 16;
    hold all;
    ylabel(hs(1), '$$Pitch\,Rate\, [rad/s]$$', 'Interpreter', 'Latex')
    grid on; box on;
    
    hs(2) = subplot(3,1,2);
    hs(2).Color            = 'k';
    hs(2).GridColor        = 0.8 * [1, 1, 1];
    hs(2).XColor           = 0.8 * [1, 1, 1];
    hs(2).YColor           = 0.8 * [1, 1, 1];
    hs(2).MinorGridColor   = 0.8 * [1, 1, 1];
    hs(2).FontSize         = 16;
    hold all;
    grid on; box on;
    
    hs(3) = subplot(3,1,3);
    hs(3).Color            = 'k';
    hs(3).GridColor        = 0.8 * [1, 1, 1];
    hs(3).XColor           = 0.8 * [1, 1, 1];
    hs(3).YColor           = 0.8 * [1, 1, 1];
    hs(3).MinorGridColor   = 0.8 * [1, 1, 1];
    hs(3).FontSize         = 16;
    hold all;
    grid on; box on;
    xlabel(hs(3), '$$t$$', 'Interpreter', 'Latex')
    ylabel(hs(3), '$$K$$', 'Interpreter', 'Latex')
    
    for i = 1:40:length(t)
        
        hold(hs(1), 'off');
        plot(hs(1), t(1:i), -delta_pilot(1:i), 'm--', 'LineWidth', 1, 'DisplayName', '$$Command$$')
        hs(1).Color            = 'k';
        hs(1).GridColor        = 0.8 * [1, 1, 1];
        hs(1).XColor           = 0.8 * [1, 1, 1];
        hs(1).YColor           = 0.8 * [1, 1, 1];
        hs(1).MinorGridColor   = 0.8 * [1, 1, 1];
        hs(1).FontSize         = 16;
        ylabel(hs(1), '$$Pitch\,Rate\, [rad/s]$$', 'Interpreter', 'Latex')
        grid(hs(1), 'on'); box(hs(1), 'on');
        hold(hs(1), 'on');
        title(hs(1), 'X-15''s Pitch Dynamics at 1.2M @ 60,000ft', 'FontSize', 20, 'Color', 'w');
        plot(hs(1), t(1:i), qm(1:i), 'y', 'LineWidth', 2, 'DisplayName', '$$q_m$$')
        plot(hs(1), t(1:i),  q(1:i), 'c', 'LineWidth', 2, 'DisplayName', '$$q$$')
        xlim(hs(1), [0, t(end)])
        ylim(hs(1), [-0.4, 0.25])
        legend(hs(1),'show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
        
        hold(hs(2), 'off');
        plot(hs(2), t(1:i),     abs(u(1:i)), 'y', 'LineWidth', 2, 'DisplayName', '$$|u|$$')
        hs(2).Color            = 'k';
        hs(2).GridColor        = 0.8 * [1, 1, 1];
        hs(2).XColor           = 0.8 * [1, 1, 1];
        hs(2).YColor           = 0.8 * [1, 1, 1];
        hs(2).MinorGridColor   = 0.8 * [1, 1, 1];
        hs(2).FontSize         = 16;
        grid(hs(2), 'on'); box(hs(2), 'on');
        hold(hs(2), 'on');
        plot(hs(2), t(1:i), mod_u(1:i), 'c', 'LineWidth', 2, 'DisplayName', '$$||u||$$')
        plot(hs(2), [0, t(end)], T * [1, 1], '--g', 'LineWidth', 2, 'DisplayName', '$$T$$')
        xlim(hs(2), [0, t(end)])
        ylim(hs(2), [0, 0.05])
        legend(hs(2), 'show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20)
        
        hold(hs(3), 'off');
        plot(hs(3), t(1:i), Kq(1:i), 'y', 'LineWidth', 2, 'DisplayName', '$$Kq$$')
        hs(3).Color            = 'k';
        hs(3).GridColor        = 0.8 * [1, 1, 1];
        hs(3).XColor           = 0.8 * [1, 1, 1];
        hs(3).YColor           = 0.8 * [1, 1, 1];
        hs(3).MinorGridColor   = 0.8 * [1, 1, 1];
        hs(3).FontSize         = 16;
        grid(hs(3), 'on'); box(hs(3), 'on');
        xlabel(hs(3), '$$t$$', 'Interpreter', 'Latex')
        ylabel(hs(3), '$$Adaptive\, Gain$$', 'Interpreter', 'Latex')
        hold(hs(3), 'on');
        plot(hs(3), [0, t(end)], Gm * [1, 1], '--m', 'LineWidth', 2, 'DisplayName', '$$Kq_{Instability}$$')
        xlim(hs(3), [0, t(end)])
        ylim(hs(3), [0, Gm * 1.2])
        legend(hs(3), 'show', 'Interpreter', 'latex', 'TextColor', 0.8 * [1, 1, 1], 'FontSize', 20, 'Location', 'SouthEast')
        
        drawnow;
        
        % Capture the plot as an image
        frame = getframe(h);
        im = frame2im(frame);
        % Write to the GIF File
        if i == 1
            [imind, cm] = rgb2ind(im,256);
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 5 * dt);
        else
            imind = rgb2ind(im, cm);
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 5 * dt);
        end
        
    end
end