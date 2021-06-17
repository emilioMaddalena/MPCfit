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

% Steady state values computation
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

%% Simulation

% Time vector
t = 0:0.1:20;

% State vector
x_sim = zeros(2,length(t));

for i=1:length(t)-1
    x_sim(:,i+1) = A*x_sim(:,i)+B*us;
    
    % If current < 0 then current = 0
    if x_sim(1,i+1)<0
        x_sim(1,i+1)=0;
    end
end
x_sim(:,end) = [];
t(end) = [];

%% Experimental results
% Load the scope data
x1_exp = readmatrix('x1.csv');
x2_exp = readmatrix('x2.csv');

% Remove sample if time < 0
x1_exp(x1_exp(:,1)<0,:) = [];
x2_exp(x2_exp(:,1)<0,:) = [];

% Sampling of the scope data
x1_d = x1_exp(1,2);
x2_d = x2_exp(1,2);

% Time of the scope data
time = 1000*x1_exp(:,1);

% Sampling time for the state
T = 0.1;
for i=1:length(time)
    if time(i)>T
        x1_d = [x1_d;x1_exp(i,2)];
        x2_d = [x2_d;x2_exp(i,2)];
        T = T + 0.1;
    end
end

% Process x1
x1_d = x1_d(1:200,:); 
x1_d = x1_d/7.5; % Division by the current sensor gain

% Shrink x2 table 
x2_d=x2_d(1:200,:);

% Time plots
figure;
subplot(2,1,1);
plot(t,1000*x1_d);
hold on;
plot(t,1000*x_sim(1,:));
grid on;
ylabel('x_1(mA)');

subplot(2,1,2);
plot(t,x2_d);
hold on;
plot(t,x_sim(2,:));
grid on;
ylabel('x_2(V)');
legend('Real system','Simulation');
xlabel('Time(ms)');