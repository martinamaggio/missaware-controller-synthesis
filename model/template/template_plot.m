% TEMPLATE_PLOT plots a selection of state and input trajectories with
% shaded quantiles (template)
%
% Copyright (c) 2026 Robert Bosch GmbH
% SPDX-License-Identifier: AGPL-3.0

function template_plot(f,til,current_method,t,u_mean,u_lb,u_ub,x_mean,x_lb,x_ub)
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
    plot(t, x_mean(:,1),'color',firstcol,'DisplayName','$x_1$');
    grid on;
    if ~isempty(x_lb)
        patch([t; flipud(t)], [x_ub(:,1); flipud(x_lb(:,1))],'','FaceColor',firstcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
    end
    xticklabels([]);
    legend('Location','best')

    nexttile(til);
    hold on;
    grid on;
    plot(t, u_mean(:,1),'color',secondcol,'DisplayName','$u_1$');
    if ~isempty(u_lb)
        patch([t; flipud(t)], [u_ub(:,1); flipud(u_lb(:,1))],'','FaceColor',secondcol,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
    end
    xlabel('time');
    legend('Location','best')
end

