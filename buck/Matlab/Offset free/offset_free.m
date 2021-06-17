clc;
clear;
close all;

% Load the plant
load ohm100.mat
load ohm50.mat
load ohm10.mat
load ohm200.mat
G1 = ohm100_10kHz;
G2 = ohm50;
G3 = ohm10;
G4 = ohm200;

% Sampling period
Ts = G1.Ts;

A = G1.A; B = G1.B; C = [0 1];
ref = 5; %5V steady state voltage

Q = [50,0;0,1];
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
model.x.max = [1-xs(1); 15-xs(2)];

% Input constraints
model.u.min = -us;
model.u.max = 1-us;

% State penalty
model.x.penalty = QuadFunction(Q);

% Input penalty
model.u.penalty = QuadFunction(R);

mpc = MPCController(model, N);

%%
% Explicit MPC
expmpc = mpc.toExplicit();
expmpc.optimizer.trimFunction('primal', 1);
expmpc.optimizer.toMatlab('exp_sol.m', 'primal', 'first-region');
figure;
expmpc.feedback.fplot();


%% Simulation

% Time vector
t = 0:0.1:40;

% Disturbance estimate
d_hist  = zeros(1,length(t));

% Augmented plant 
A_aug = [A,[0;0];0,0,1];
B_aug = [B;0];

% Slow LQR backup feedback gain matrix
K = dlqr(A,B,[100,0;0,1],100000);

% State vector
x_hist = zeros(2,length(t));

% Input vector
u_hist = zeros(1,length(t)-1);

x_real = zeros(2,length(t));

L = -place(A_aug',[0,1,1]',[0.8,0.85,0.9])';

for i=1:length(t)-1
    % Find target according to the disturbance
    val_ss = [eye(2)-A,-B;C,0]\[0;0;ref-d_hist(i)];
    xs = val_ss(1:2);
    us = val_ss(3);
    
    % PWA mapping
    u = exp_sol(x_hist(:,i)-xs);
    
    % If no region found slow LQR
    if isnan(u)
        u_hist(i) = -K*(x_hist(:,i)-xs);
        display('Nan');
    else
        u_hist(i) = u;
    end
    % add steady state input
    u_hist(i) = u_hist(i) + us;
    if u_hist(i)<0
        u_hist(i) = 0;
    end
    if u_hist(i)>1
        u_hist(i) = 1;
    end
    
    % State update
    x_aug = A_aug*[x_hist(:,i);d_hist(i)] + B_aug*u_hist(i)+...
            L*(x_hist(2,i)+d_hist(i)-x_real(2,i));
    x_hist(:,i+1) = x_aug(1:2);
    d_hist(:,i+1) = x_aug(3);
    
    % Load switch
    if t(i)<20
        x_real(:,i+1) = G1.A*x_real(:,i) + G1.B*u_hist(:,i);
    else
        x_real(:,i+1) = G3.A*x_real(:,i) + G3.B*u_hist(:,i);
    end
end

% Time plots
figure;
subplot(4,1,1); 
plot(t, x_hist(1,:));
grid on;
hold on;
plot(t, x_real(1,:));
legend('Estimate','Real');
ylabel('x_1(mA)');

subplot(4,1,2);
plot(t, x_hist(2,:));
grid on; 
hold on;
plot(t, x_real(2,:));
plot(t, 5*ones(1,length(t)),'r--');
ylabel('x_2(V)');

subplot(4,1,3);
plot(t, d_hist);
grid on; 
ylabel('Disturbance');

subplot(4,1,4);
grid on; hold on;
plot(t, [0 u_hist]);
ylabel('Duty cycle(%)');
xlabel('Time(ms)');