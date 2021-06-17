clc;
clear;
close all;

% Load the plant
load ohm100_10kHz.mat;
G = ohm100_10kHz;

% Sampling period
Ts = G.Ts;

A = G.A; B = G.B; C = [0 1];
ref = 5; %5V steady state voltage

Q = [90,0;0,1];
R = 1;

% Steady state values computation
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

% Horizon
N = 10;

% Defining the model
model = LTISystem('A',A,'B',B,'Ts',Ts);

%State constraints
model.x.min = [-xs(1); -xs(2)];
model.x.max = [0.2-xs(1); 10-xs(2)];

% Input constraints
model.u.min = -us;
model.u.max = 1-us;

% State penalty
model.x.penalty = QuadFunction(Q);

% Input penalty
model.u.penalty = QuadFunction(R);


% Terminal set and cost
Tset = model.LQRSet;
PN = model.LQRPenalty;
model.x.with('terminalSet');
model.x.terminalSet = Tset;
model.x.with('terminalPenalty');
model.x.terminalPenalty = PN;

mpc = MPCController(model, N);

% Explicit MPC
expmpc = mpc.toExplicit();
expmpc.optimizer.trimFunction('primal', 1);
expmpc.optimizer.toMatlab('exp_sol.m', 'primal', 'first-region');

%% Plots 
figure;
expmpc.feedback.fplot()
xlabel('\Deltax_1 (A)');
ylabel('\Deltax_2 (V)');
zlabel('\Deltau');

% Extract the numbers
H = {}; h = {};
F = {}; g = {};
for i = 1:expmpc.optimizer.Num
        F{i} = expmpc.optimizer.Set(i).Functions('primal').F;
        g{i} = expmpc.optimizer.Set(i).Functions('primal').g;
        H{i} = expmpc.optimizer.Set(i).A;
        h{i} = expmpc.optimizer.Set(i).b;
end
%% Simulation

% Time vector
t = 0:0.1:20;

% Slow LQR backup feedback gain matrix
K = dlqr(A,B,[1000000,0;0,1],1000);

% State vector
x_hist = zeros(2,length(t));

% Input vector
u_hist = zeros(1,length(t)-1);

% Region vector
regions = zeros(1,length(t)-1);

for i=1:length(t)-1
    % PWA mapping
    [u,regions(i)] = exp_sol(x_hist(:,i)-xs);
    
    % If no region found slow LQR
    if isnan(u)
        u_hist(i) = -K*(x_hist(:,i)-xs);
        regions(i)=0;
    else
        u_hist(i) = u;
    end
    % add steady state input
    u_hist(i) = u_hist(i) + us;
    
    % Input saturation
    if u_hist(i) < 0
        u_hist(i) = 0;
    end
    if u_hist(i) >1
        u_hist(i) = 1;
    end
    % State update
    x_hist(:,i+1) = A*x_hist(:,i)+B*u_hist(i);
end

% State plot
figure;
plot(expmpc.optimizer.Set);
hold on;
plot(x_hist(1,:)-xs(1)*ones(1,length(t)), x_hist(2,:)-xs(2)*ones(1,length(t)), 'k--o');

% Time plots
figure;
subplot(3,1,1);
grid on; hold on;
plot(t, x_hist(1,:));
ylabel('x_1(mA)');

subplot(3,1,2);
grid on; hold on;
plot(t, x_hist(2,:));
ylabel('x_2(V)');

subplot(3,1,3);
grid on; hold on;
plot(t, [0 u_hist]);
ylabel('Duty cycle(%)');
xlabel('Time(ms)');