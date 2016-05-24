function [value discrete] = C(x) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Yuchun Li
%
% Project: Simulation of a hybrid system
%
% Name: C.m
%
% Description: Flow set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% global parameters
% global G A H1 H2 K1 K2 T1 T2


% states
% xp      = x(1:2);               % state of plant
timer      = x(end-1:end);               % state of timer

if  length(find(timer >= 0)) == length(timer)
    value = 1; % always flow, there is no jump yet.
else
    value = 0;
end
