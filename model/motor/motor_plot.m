% MOTOR_PLOT plots a selection of state and input trajectories with shaded quantiles
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function motor_plot(f,til,current_method,t,u_mean,u_lb,u_ub,x_mean,x_lb,x_ub)
% Plots a selection of state and input trajectories with shaded quantiles.
%
% Inputs
%   f               : figure handle
%   til             : tiled layout handle
%   current_method  : name of current control method
%   t               : vector of time steps
%   u_mean          : matrix of mean values of input trajectories
%   u_lb, u_ub      : matrices of lower/higher input quantiles
%   x_mean          : matrix of mean values of state trajectories
%   x_lb, x_ub      : matrices of lower/higher state quantiles

    firstcol = "#EE7733";
    secondcol = "#0077BB";

    figure(f)
    nexttile(til);
    title(current_method)
    hold on;
    handle_xd = plot(t, x_mean(:,1),'color',firstcol,'LineStyle','--','LineWidth',1.5,'DisplayName','$\Delta i_\mathrm{d}$');
    handle_xq = plot(t, x_mean(:,2),'color',firstcol,'LineStyle',':','LineWidth',1.5,'DisplayName','$\Delta i_\mathrm{q}$');
    if size(x_mean,2) > 2
        handle_wel = plot(t, x_mean(:,3),'color','k','LineWidth',0.5,'DisplayName','$\Delta \omega_\mathrm{el}$');
    end
    ylabel('current in [A] / angular velocity in [rad/s]');
    ylim([-25 25]);
    xlim([0, t(end)])
    grid on;
    if ~isempty(x_lb)
        patch([t; flipud(t)], [x_ub(:,1); flipud(x_lb(:,1))],'','FaceColor',firstcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
        patch([t; flipud(t)], [x_ub(:,2); flipud(x_lb(:,2))],'','FaceColor',firstcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
        if size(x_mean,2) > 2
            patch([t; flipud(t)], [x_ub(:,3); flipud(x_lb(:,3))],'','FaceColor','k','FaceAlpha',0.1,'EdgeColor','none','HandleVisibility','off');
        end
    end
    xticklabels([]);
    legend('Location','best')

    nexttile(til);
    hold on;
    grid on;
    handle_ud = plot(t, u_mean(:,1),'color',secondcol,'LineStyle','--','LineWidth',1,'DisplayName','$\Delta u_\mathrm{d}$');
    handle_uq = plot(t, u_mean(:,2),'color',secondcol,'LineStyle',':','LineWidth',1,'DisplayName','$\Delta u_\mathrm{q}$');
    ylabel('voltage in [V]');
    ylim([-2,1]);
    xlim([0.0, t(end)])
    if ~isempty(u_lb)
        patch([t; flipud(t)], [u_ub(:,1); flipud(u_lb(:,1))],'','FaceColor',secondcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
        patch([t; flipud(t)], [u_ub(:,2); flipud(u_lb(:,2))],'','FaceColor',secondcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
    end
    xlabel('time');
    legend('Location','best')

end

