clc;
clear;
close all;

% Load the plant
load ohm100_10kHz.mat;
G1 = ohm100_10kHz;

% Sampling period
Ts = G1.Ts;

A = G1.A; B = G1.B; C = [0 1];
ref = 5; % 5V steady state voltage

Q = [90,0;0,1];
R = 10;

K = dlqr(A,B,Q,R);

% Steady state input and current computation
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

%% Simulation

% Time vector
t = 0:0.1:16;

% State vector
x_sim = zeros(2,length(t));

% Input vector
u_sim = zeros(1,length(t)-1);

for i=1:length(t)-1
    u_sim(i) = -K*(x_sim(:,i)-xs)+us;
    x_sim(:,i+1) = G1.A*x_sim(:,i)+min(1,max(G1.B*u_sim(i),0));   
end
x_sim(:,end) = [];
t(end) = [];

figure
subplot(2,1,1)
plot(t,x_sim(1,:),'linewidth',2)
subplot(2,1,2)
plot(t,x_sim(2,:),'linewidth',2)

%% Experimental results

% Get the scope data
x1_exp = readmatrix('x1.csv');
x2_exp = readmatrix('x2.csv');
u_exp = readmatrix('u.csv');

% Remove the sample if time < 0
x1_exp(x1_exp(:,1)<0,:) = [];
x2_exp(x2_exp(:,1)<0,:) = [];
u_exp(u_exp(:,1)<0,:) = [];

% Sampling of the scope data
x1_d = x1_exp(1,2);
x2_d = x2_exp(1,2);
u_d = [];

% Time of the scope data in ms
time = 1000*u_exp(:,1);

%{
To account for the delay between the time at which the state is sampled 
and the time at which the input is given, the input is sampled 50us after
the sampling time.
%}

% Sampling time for the state
T1 = 0.1;
% Sampling time for the duty cycle and regions
T2 = 0.05;
% Sampling
for i=1:length(time)
    if time(i)>T1
        x1_d = [x1_d;x1_exp(i,2)];
        x2_d = [x2_d;x2_exp(i,2)];
        T1 = T1 + 0.1;
    end
    if time(i)>T2
        u_d = [u_d;u_exp(i,2)];
        T2 = T2 + 0.1;
    end
end

% Shrinking and processing the sampled data
u_d=u_d(1:160,:);

% Computing x1
x1_d = x1_d(1:160,:); 
x1_d = x1_d/7.5;  % Current sensor gain division

% Shrinking x2 table 
x2_d=x2_d(1:160,:);

% Time plots
figure;
subplot(3,1,1);
plot(t,1000*x1_d);
hold on;
plot(t,1000*x_sim(1,:));
grid on;
ylabel('x_1(mA)');

subplot(3,1,2);
plot(t,x2_d);
hold on;
plot(t,x_sim(2,:));
grid on;
ylabel('x_2(V)');

subplot(3,1,3);
plot(t,100*u_d);
hold on;
plot(t,100*u_sim);
grid on;
ylabel('Duty cycle(%)');
legend('Real system','Simulation');
xlabel('Time(ms)');