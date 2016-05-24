%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Yuchun Li
%
% simulation of event based observer with 2 bidirectional connected agents
% note that when timer10 and timer20 have same initial conditions, 
% it reduces to the scenario of synchronuous event times 
%
% Name: run.m
%
% Description: run script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% global data -----------
clear all
global G A H1 H2 K11 K12 K21 K22 T1 T2 W mm

% global parameters
A  = [0 -1;1 0];            % dynamics of oscillator 
H1 = [1 0]; H2 = [0 1];     % measurements at agent 1 and 2
T1 = 0.2; T2 = 2;           % events' lower and upper bounds
K11 = -[0.5 0.2]';          % gain for agent 1
K12 = -[0.2 0.2]';          % gain for agent 1
K21 = [0.2 0.3]';           % gain for agent 2
K22 = [-0.1 -0.5]';         % gain for agent 2
W   = -0.1;                 % gain for consensus

% gain for measurement noise
% mm = 0.8; 
mm = 0;                     

% Graph (1) - 2 agents
G = ones(2,2);

% Initial Condition for plant states
xp0 = [2 2]';

% Initial Condition for agent1;
xo10 = [15 5]';
eta10 = [1 1]';
timer10 = 0;

% Initial Condition for agent2;
xo20 = [-1 0]';
eta20 = [-1 -1]';
timer20 = 0.3;

y0 = [xp0; xo10; xo20; eta10; eta20; timer10; timer20]; 

% simulation horizon
TSPAN = [0 30];
JSPAN = [0 20000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-1,'MaxStep',1e-2);

% simulate
[t y j] = hybridsolver(@f,@g,@C,@D,y0,TSPAN,JSPAN,rule,options,1);

 
%% ploting timer
figure
plotHarcT(t,j,y(:,11),'k')
hold on 
plotHarcT(t,j,y(:,12),'b')
axis([0, 10, 0, 2])
set(gca,'FontSize',14)
%%

%% ploting estimation errors
figure
subplot(211)
plotHarcT(t,j,y(:,3) - y(:,1),'k')
hold on 
plotHarcT(t,j,y(:,5) - y(:,1),'b')
set(gca,'FontSize',14)

subplot(212)
plotHarcT(t,j,y(:,4) - y(:,2),'k')
hold on 
plotHarcT(t,j,y(:,6) - y(:,2),'b')
set(gca,'FontSize',14)
%%
