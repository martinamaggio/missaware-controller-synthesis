function [u,z] = ctrl_none(x,z,u,sequence,~)
% Wrapper used for open-loop simulation
    u = zeros(size(u));
end