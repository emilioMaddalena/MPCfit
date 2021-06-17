clc
clear
close all

% Load the scope data
x1_lqr = readmatrix('x1_lqr.csv');
x2_lqr = readmatrix('x2_lqr.csv');
x1_nn = readmatrix('x1_nn.csv');
x2_nn = readmatrix('x2_nn.csv');
x1_ol = readmatrix('x1_ol.csv');
x2_ol = readmatrix('x2_ol.csv');

% Remove sample if time < 0
x1_lqr(x1_lqr(:,1)<0,:) = [];
x2_lqr(x2_lqr(:,1)<0,:) = [];
x1_nn(x1_nn(:,1)<0,:) = [];
x2_nn(x2_nn(:,1)<0,:) = [];
x1_ol(x1_ol(:,1)<0,:) = [];
x2_ol(x2_ol(:,1)<0,:) = [];

% Sampling of the scope data
x1_lqr_d = x1_lqr(1,2);
x2_lqr_d = x2_lqr(1,2);
x1_nn_d = x1_nn(1,2);
x2_nn_d = x2_nn(1,2);
x1_ol_d = x1_ol(1,2);
x2_ol_d = x2_ol(1,2);

%%%%%% lqr sampling %%%%%%
% Time of the scope data lqr 2ms/div
time = 1000*x1_lqr(:,1);

% Sampling time for the state
T = 0.1;

% Sampling
for i=1:length(time)
    if time(i)>T
        x1_lqr_d = [x1_lqr_d;x1_lqr(i,2)];
        x2_lqr_d = [x2_lqr_d;x2_lqr(i,2)];
        T = T + 0.1;
    end
end

%%%%%% nn sampling %%%%%%
% Time of the scope data nn 2ms/div
time = 1000*x1_nn(:,1);

% Sampling time for the state
T = 0.1;

% Sampling
for i=1:length(time)
    if time(i)>T
        x1_nn_d = [x1_nn_d;x1_nn(i,2)];
        x2_nn_d = [x2_nn_d;x2_nn(i,2)];
        T = T + 0.1;
    end
end

%%%%%% ol sampling %%%%%%
% Time of the scope data ol 5ms/div
time = 1000*x1_ol(:,1);

% Sampling time for the state
T = 0.1;

% Sampling
for i=1:length(time)
    if time(i)>T
        x1_ol_d = [x1_ol_d;x1_ol(i,2)];
        x2_ol_d = [x2_ol_d;x2_ol(i,2)];
        T = T + 0.1;
    end
end

% Resize
x1_ol_d(181:end) = [];
x2_ol_d(181:end) = [];
x1_lqr_d(181:end) = [];
x2_lqr_d(181:end) = [];
x1_nn_d(181:end) = [];
x2_nn_d(181:end) = [];

 
%%
 
x1_ol_d = x1_ol_d/7.5; 
x1_lqr_d = x1_lqr_d/7.5;
x1_nn_d = x1_nn_d/7.5;

t = 0:0.1:(180-1)*0.1;

% Time plots
figure;
subplot(2,1,1);
p1=plot(t,1000*x1_nn_d);
hold on;
p2=plot(t,1000*x1_lqr_d);
grid on;
p3=plot(t,1000*x1_ol_d);
ylabel('x_1(mA)');
legend([p1;p2;p3],{'eMPC';'LQR';'Open loop'});

subplot(2,1,2);
plot(t,x2_nn_d);
hold on;
plot(t,x2_lqr_d);
grid on;
plot(t,x2_ol_d);
ylabel('x_2(V)');
xlabel('Time (ms)');