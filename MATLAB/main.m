clc
clear all
close all

% Loading discrete-time system
load ohm100_10kHz.mat;

G = ohm100_10kHz;
Ts = G.Ts;
A = G.A; B = G.B; C = [0 1];
ref = 5;
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

% MPC design
N = 10; 
Q = [90,0;0,1];
R = 1; 

xMax = [0.2-xs(1); 7-xs(2)];
xMin = [-xs(1); -xs(2)];

model = LTISystem('A',A,'B',B,'Ts',Ts);
model.x.min = xMin;
model.x.max = xMax;
model.u.min = -us;
model.u.max = 1-us;
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
Tset = model.LQRSet;
PN = model.LQRPenalty;
model.x.with('terminalSet');
model.x.terminalSet = Tset;
model.x.with('terminalPenalty');
model.x.terminalPenalty = PN;

mpc = MPCController(model, N);
expMpc = mpc.toExplicit();
expMpc.optimizer.trimFunction('primal', 1);
expMpc.optimizer.toMatlab('empc_original.m', 'primal', 'first-region');

xmax = [0.15; 2]; 
xmin = [-0.05; -5];

umax = 0.6621;
umin = -0.3379;

xdelta = xmax-xmin;
udelta = umax-umin;

expMpc.feedback.fplot
view(80,40)
set(gcf,'color','w');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sampling the original MPC controller    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nSamples = 5e3; 
name = 'dataset';

XX = [];
UU = [];
while size(XX,2) < nSamples
    fprintf('%i of %i\n', size(XX,2), nSamples);
    X1 = (xMax(1) - xMin(1)).*rand(1,nSamples/10) + xMin(1);
    X2 = (xMax(2) - xMin(2)).*rand(1,nSamples/10) + xMin(2);
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

xmin = [-.05; -5]; xmax = [.15; 2];
umin = -us; umax = 1 - us;

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
save(['/Users/emilio/Desktop/TII-stuff/pQP-NN/data/' name],'X','U')

fprintf('~~~~Sampling process concluded~~~~\n');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating the explicit solution to the first linear + pQP layers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select the file to be loaded 
%PYTHONfile = 'NNparams_var_3_run_5_epoch_51';
%PYTHONfile = 'NNparams_var_3_run_6_epoch_104';
PYTHONfile = 'NNparams_var_3_run_2_epoch_109';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Loading Neural Network parameters... \n');
%filePath = ['/Users/emilio/Desktop/TII-stuff/pQP-NN/data/' PYTHONfile '.mat'];
filePath = [PYTHONfile '.mat'];
fit = load(filePath);

fit.F = double(fit.F);
fit.f = double(fit.f(:));
fit.L = double(fit.L);
fit.O = double(fit.O);
fit.o = double(fit.o(:));

nVar = size(fit.L,2);
nParam = size(fit.F,2); 

fit.Q = fit.L'*fit.L + 1e-3*eye(nVar);

z = sdpvar(nVar,1);
x = sdpvar(nParam,1); % nStates

% pQP format: min .5*z'*Q*z + (F*x + f')'*z
%            s.t. z >= 0
% Plus, solve the parameters only in within the
% feasible range, i.e. xMin <= x <= xMax

obj = 0.5*z'*fit.Q*z + (fit.F*x+fit.f)'*z;
constr = [z >= 0; 0 <= x <= 1];
sett = sdpsettings('solver','gurobi+');

yalmipSol = solvemp(constr,obj,[],x,z);
mptSol = mpt_mpsol2pu(yalmipSol); 
mptSol.trimFunction('primal',nVar);
mptSol.toMatlab('empc_simple.m', 'primal', 'first-region');

%outScaling = [Vin/2; Vin/2; Vin]; % Must be the same as before
%uPolyFit = Polyhedron(Cu*diag(outScaling),du);
%umax = [Vin*dutyMax; Vin*dutyMax; Vin*dutyMax] ./ outScaling; 
%umin = [0; 0; 0];

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comparing the two controllers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear x x1 x2 
fprintf('Comparing two slices of the control laws... \n')

[X1,X2] = meshgrid(xmin(1):((xmax(1)-xmin(1))/30):xmax(1),xmin(2):((xmax(2)-xmin(2))/30):xmax(2));
x1 = X1(:); x2 = X2(:);
GT = zeros(numel(x1),1); 
NN = zeros(numel(x1),1);

for i = 1:numel(x1)
    
    x(:,i) = [x1(i); x2(i)];
    
    % ground-truth
    gt = expMpc.feedback.feval(x(:,i));
    GT(i) = gt(1);
    
    % scaling inputs
    x(:,i) = (x(:,i) - xmin) ./ (xmax - xmin);
    % PWA-NN surrogate
    [nnTemp, ~] = empc_simple(x(:,i));
    nn = fit.O*nnTemp + fit.o;
    nn = max(min(nn,1),0);
    % scaling output back
    NN(i) = (nn * (umax - umin)) + umin;
    
end

GT = reshape(GT,size(X1)); NN = reshape(NN,size(X1));
figure; subplot(1,2,1); surf(X1,X2,GT); hold on; grid on; view(80,40)
axis([xmin(1) xmax(1) xmin(2) xmax(2) umin umax]); 
%xticks([-5 -2.5 0 2.5 5]); yticks([-5 -2.5 0 2.5 5]); zticks([-50 -30 -10 10 30])
%xlabel('Idm1'); ylabel('Idm2'); zlabel('vdm1'); 
title('MPC controller')
subplot(1,2,2); surf(X1,X2,NN); view(80,40)
axis([xmin(1) xmax(1) xmin(2) xmax(2) umin umax]);
%axis([-5 5 -5 5 -50 30]); 
%xticks([-5 -2.5 0 2.5 5]); yticks([-5 -2.5 0 2.5 5]); zticks([-50 -30 -10 10 30])
%xlabel('Idm1'); ylabel('Idm2'); zlabel('vdm1'); 
title('PWA-NN controller');
set(gcf,'color','w');

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Closed-loop simulation of the discrete-time system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear x y u v
Tsim = 50;

% Some initial conditions
x(:,1) = zeros(nx,1); 
%x(:,1) = [1;-2;15;200]; 
%x(:,1) = [2;-2;5;325]; 
%x(:,1) = [2;1;-5;150];
%x(:,1) = [-4;-3;10;200]; 
%x(:,1) = [3;-2;25;0]; % unstable

y(:,1) = x(:,1);
for t = 1:Tsim-1
    
    % optimal MPC controller
    [u(:,t), region] = eMpc(x(:,t));
    if region == 0, error(['No region found for state x = ' mat2str(x(:,t))]); end
    
    % First linear layer + PWA function
    [vTemp, ~] = fitMpc(y(:,t));
    % Second linear layer
    surr = fit.O*vTemp + fit.o; 
    % Simplified projection
    surr = max(min(surr,umax),umin);
    v(:,t) = (T*surr) .* outScaling;
    
    x(:,t+1) = Ad*x(:,t) + Bd*u(:,t);
    y(:,t+1) = Ad*y(:,t) + Bd*v(:,t);

end

% Plotting states x3 and x4
figure(1); hold on
axis([xMin(3) xMax(3) xMin(4) xMax(4)]); xlabel('I_{cm}'); ylabel('V_{l}'); 
title('Phase portrait'); grid on;
plot(x(3,1),x(4,1),'kx'); hold on 
plot(x(3,:),x(4,:),'k--'); 
plot(y(3,:),y(4,:),'r-o'); 