% Supplementary material for the paper:
% 'Embedded PWM Predictive Control of DC-DC Power Converters Via Piecewise-Affine Neural Networks'
% 
% Authors: E. T. Maddalena, M. W. F. Specq, V. L. Wisniewski and C. N. Jones
%
% Dependencies: Yalmip, MPT3 toolbox and Gurobi
%
% Run model.m first to calculate the model parameters

clc
clear all
close all

%% Loading the discrete-time system

load model_params.mat;

G = model_params;
A = G.A; 
B = G.B; 
C = G.C;
Ts = G.Ts;

% reference output voltage
ref = 5;

% steady-state values
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

%% MPC controller design

% The MPC controller is defined w.r.t. the distance to the steady-state
% values xs, us => its goal is to regulate the system to the origin

% horizon and costs
N = 10; 
Q = [90, 0; 0, 1];
R = 1; 

% constraints
xmax = [0.2-xs(1); 7-xs(2)];
xmin = [-xs(1); -xs(2)];
xdelta = xmax - xmin;
umax = 1-us;
umin = -us;
udelta = umax-umin;

model = LTISystem('A',A,'B',B,'Ts',Ts);
model.x.min = xmin; model.x.max = xmax;
model.u.min = umin; model.u.max = umax;
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

Tset = model.LQRSet;
PN = model.LQRPenalty;
model.x.with('terminalSet');
model.x.terminalSet = Tset;
model.x.with('terminalPenalty');
model.x.terminalPenalty = PN;

mpc = MPCController(model, N);

% Calculate explicit solution
expMpc = mpc.toExplicit();
expMpc.optimizer.trimFunction('primal', 1);
expMpc.optimizer.toMatlab('empc_exact.m', 'primal', 'first-region');

expMpc.feedback.fplot
view(80,40)
set(gcf,'color','w');

%% Sampling the exact MPC controller

nSamples = 5e3; 
name = 'dataset';

XX = [];
UU = [];

while size(XX,2) < nSamples
    
    fprintf('%i of %i\n', size(XX,2), nSamples);
    X1 = (xmax(1) - xmin(1)).*rand(1,nSamples/10) + xmin(1);
    X2 = (xmax(2) - xmin(2)).*rand(1,nSamples/10) + xmin(2);
    X = [X1; X2];
    
    % checking if they are feasible MPC initial conditions
    for i = 1:(nSamples/10)
        if expMpc.optimizer.contains(X(:,i))
            XX = [XX X(:,i)];
            val = expMpc.optimizer.feval(X(:,i),'primal','tiebreak','obj');
            UU = [UU val(1)];
        end
    end
    
end
fprintf('%i of %i\n', nSamples, nSamples);
clear X U

X = XX(:,1:nSamples); U = UU(:,1:nSamples);

% normalizing the features and labels to [0,1]
X(1,:) = (X(1,:) - xmin(1)) / (xmax(1) - xmin(1));
X(2,:) = (X(2,:) - xmin(2)) / (xmax(2) - xmin(2));
U = (U - umin) / (umax - umin);

% checking the sample distribution
for i = 1:nSamples
    plot3(X(1,i),X(2,i),U(i),'bo');
    hold on
    grid on
end

fprintf('Saving data in file %s.mat\n', name);
save(['./data/' name],'X','U')
fprintf('~~Sampling process concluded~~\n');

%% Run python scripts to process the data and find the PWA-NN parameters


%% Retrieving the PWA-NN parameters and calculating its PWA representation

% pQP format: min .5*z'*Q*z + (F*x + f')'*z
%             s.t. z >= 0
%
% Plus, solve the parameters only in within the
% feasible range, i.e. xmin <= x <= xmax

% Select the file to be loaded 
PYTHONfile = 'NNparams_var_3_run_2_epoch_109';
fprintf('Loading Neural Network parameters... \n');
filePath = ['./data/' PYTHONfile '.mat'];
fit = load(filePath);

% Extracting PWA-NN parameters
fit.F = double(fit.F);
fit.f = double(fit.f(:));
fit.H = double(fit.H);
fit.G = double(fit.G);
fit.g = double(fit.g(:));
epsilon = 1e-3*eye(size(fit.H,2));
fit.Q = fit.H'*fit.H + epsilon;

% Formulating the pQP 
nVar = size(fit.H,2);
nParam = size(fit.F,2); 
z = sdpvar(nVar,1);
x = sdpvar(nParam,1);
obj = 0.5*z'*fit.Q*z + (fit.F*x+fit.f)'*z;
constr = [z >= 0; 0 <= x <= 1];

% Solving it using YALMIP 
sett = sdpsettings('solver','gurobi+');
yalmipSol = solvemp(constr,obj,[],x,z);
mptSol = mpt_mpsol2pu(yalmipSol); 
mptSol.trimFunction('primal',nVar);
mptSol.toMatlab('empc_simple.m', 'primal', 'first-region');

%% Comparing the two controllers

clear x x1 x2 

[X1,X2] = meshgrid(xmin(1):((xmax(1)-xmin(1))/30):xmax(1),xmin(2):((xmax(2)-xmin(2))/30):xmax(2));
x1 = X1(:); 
x2 = X2(:);

MPC = zeros(numel(x1),1); 
PWANN = zeros(numel(x1),1);

for i = 1:numel(x1)
    
    x(:,i) = [x1(i); x2(i)];
    
    % exact MPC controller 
    mpc = empc_exact(x(:,i));
    MPC(i) = mpc(1);
    
    % PWA-NN approximate controller
    % scaling inputs
    x(:,i) = (x(:,i) - xmin) ./ (xmax - xmin);
    % evaluating PWA part
    [pwaTemp, ~] = empc_simple(x(:,i));
    % second linear layer
    pwann = fit.G*pwaTemp+ fit.g;
    % projection (saturation)
    pwann= max(min(pwann,1),0);
    % scaling output back
    PWANN(i) = (pwann * (umax - umin)) + umin;
    
end

set(0,'defaulttextinterpreter','latex')
MPC = reshape(MPC,size(X1)); PWANN = reshape(PWANN,size(X1));
figure; subplot(1,2,1); surf(X1,X2,MPC); hold on; grid on; view(80,40)
axis([xmin(1) xmax(1) xmin(2) xmax(2) umin umax]); 
xlabel('iL'); ylabel('vO'); zlabel('Duty cycle'); 
title('\textbf{MPC controller}')

subplot(1,2,2); surf(X1,X2,PWANN); view(80,40)
axis([xmin(1) xmax(1) xmin(2) xmax(2) umin umax]);
xlabel('iL'); ylabel('vO'); zlabel('Duty cycle'); 
title('\textbf{PWA-NN controller}');
set(gcf,'color','w', 'Position', [200 200 1100 500]);
fprintf('The axes are centered at the steady state values... \n')

%% Closed-loop simulation of the discrete-time system

clear x y u v
Tsim = 40;

% Some initial conditions
x(:,1) = zeros(2,1); 
% x(:,1) = [0.02; 0.5]; 
% x(:,1) = [0.1; 2]; 
% x(:,1) = [0.02; 3.5]; 
% x(:,1) = [0.18; 3.5]; 
% x(:,1) = [0.18; 4.5]; 

y(:,1) = x(:,1);
for t = 1:Tsim-1
    
    % optimal MPC controller
    xx = x(:,t) - xs; 
    [u(:,t), region] = empc_exact(xx);
    if region == 0, error(['No region found for state x = ' mat2str(x(:,t))]); end
    u(:,t) = u(:,t) + us;
    
    % PWA-NN controller
    yy = y(:,t) - xs; 
    % scaling 
    yy = (yy - xmin) ./ (xmax - xmin);
    % First linear layer and PWA combined
    [vTemp, ~] = empc_simple(yy);
    % Second linear layer
    vTemp = fit.G*vTemp + fit.g; 
    % Projection (saturation in this case)
    vTemp = max(min(vTemp, 1), 0);
    % scaling output back
    v(t) = (vTemp *(umax - umin)) + umin;
    v(t) = v(t) + us;
    
    x(:,t+1) = A*x(:,t) + B*u(:,t);
    y(:,t+1) = A*y(:,t) + B*v(:,t);

end

% Plot phase portrait
figure(1); hold on
title('\textbf{Closed-loop behavior}'); grid on;
plot(x(1,:),x(2,:),'k-x'); hold on 
plot(y(1,:),y(2,:),'r-x');
plot(x(1,1),x(2,1),'ok', 'markersize', 10, 'markerfacecolor', 'g');
plot(xs(1),xs(2),'pk', 'markersize', 15, 'markerfacecolor', 'g');
xlabel('iL'); ylabel('vO'); 
legend('exact MPC controller', 'PWA-NN controller')
set(gcf,'color','w');
