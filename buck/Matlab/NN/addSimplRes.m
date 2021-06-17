clc
clear all
close all

% Loading sys parameters
load ohm100_10kHz.mat;

G = ohm100_10kHz;
Ts = G.Ts;
A = G.A; B = G.B; C = [0 1];
ref = 5;
val_ss = [A-eye(2) B; C 0]\[0;0;1]*ref;
xs = val_ss(1:2);
us = val_ss(3);

% MPC design
N = 100;
Q = [1,0;0,1];
R = 100;
model = LTISystem('A',A,'B',B,'Ts',Ts);
model.x.min = [-xs(1); -xs(2)];
model.x.max = [0.2-xs(1); 7-xs(2)];

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

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sampling the optimal control law
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nSamples = 5e3; 
outScaling = [Vin/2; Vin/2; Vin];

MATLABfile = 'MATLABsamples-appendix';

XX = [];
UU = [];
while size(XX,2) < nSamples
    fprintf('%i of %i\n', size(XX,2), nSamples);
    X = (xMax - xMin).*rand(1,nSamples/10) + xMin;
    % checking if they are feasible MPC initial conditions
    for i = 1:(nSamples/10)
        if expMpc.optimizer.contains(X(:,i))
            XX = [XX X(:,i)];
            UU = [UU expMpc.optimizer.feval(X(:,i),'primal','tiebreak','obj')];
        end
    end
end
X = XX; U = UU;
U = U ./ outScaling; % normalizing the data

fprintf('Saving data in file %s.mat\n', MATLABfile);
save(['./data/' MATLABfile],'X','U')

fprintf('Sampling process concluded \n');

%%
% I am using 2 as the max voltage because the feasible set reduced the
% original 6 a lot, it is just to get better scaling
xmax = [0.15; 2]; 
xmin = [-0.05; -5];

umax = 0.6621;
umin = -0.3379;

xdelta = xmax-xmin;
udelta = umax-umin;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating the explicit solution to the first linear + pQP layers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select the file to be loaded 
PYTHONfile = 'NNparams_var_3_run_2_epoch_109'; % GOLDEN!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('Loading Neural Network parameters... \n');
filePath = ['./' PYTHONfile '.mat'];
fit = load(filePath);

% converting python params to MATLAB doubles
fit.F = double(fit.F);
fit.f = double(fit.f(:));
fit.L = double(fit.L);
fit.O = double(fit.O);
fit.o = double(fit.o(:));

% Building the pQP problem in YALMIP
nVar = size(fit.L,2);
nParam = size(fit.F,2); 
fit.Q = fit.L'*fit.L + 1e-3*eye(nVar);
z = sdpvar(nVar,1);
x = sdpvar(nParam,1); 

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
mptSol.toMatlab('fitMpc.m', 'primal', 'first-region');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulating the two controllers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear x z
fprintf('Comparing the two controllers... \n')

x(:,1) = [0;0];
x(:,1) = [0.04;1];
x(:,1) = [0.02;2];
x(:,1) = [0.04;3];
x(:,1) = [0.02;4];
x(:,1) = [0;5];
x(:,1) = [0.1;6];
    
z(:,1) = x(:,1);

Tsim = 40;

for i = 1:Tsim
    
    % ground-truth
    gt = expMpc.feedback.feval(x(:,i) - xs);
    u(i) = gt(1) + us;
    
    x(:,i+1) = A*x(:,i) + B*u(i);
    
    % NN surrogate (what needs to be implemented)
    % scaling the input
    temp = ((z(:,i)-xs) - xmin) ./ xdelta;
    % regular PWA function evaluation (i.e. approx eMPC)
    [nnTemp, ~] = fitMpc(temp);
    % output affine map
    nn = fit.O*nnTemp + fit.o;
    % scaling control back
    nn = (nn*udelta) + umin;
    v(i) = max(min(nn,umax),umin) + us;
    
    z(:,i+1) = A*z(:,i) + B*v(i);
end

figure(1)
subplot(1,2,1)
plot([0:1:Tsim]*Ts,x(1,:),'--k','linewidth',1.5); grid on; hold on
plot([0:1:Tsim]*Ts,z(1,:),'-k','linewidth',1.5);
axis([0 Tsim*Ts -0.005 0.205])
xticks([0 1 2 3 4]*1e-3)
yticks([0 4 8 12 16 20]*1e-2)

subplot(1,2,2)
plot([0:1:Tsim]*Ts,x(2,:),'--k','linewidth',1.5); grid on; hold on
plot([0:1:Tsim]*Ts,z(2,:),'-k','linewidth',1.5);
axis([0 Tsim*Ts -0.005 7.005])
xticks([0 1 2 3 4]*1e-3)
yticks([0 1 2 3 4 5 6 7])
set(gcf,'Position', [10 10 900 300])

figure(2)
plot(x(1,:),x(2,:),'--k','linewidth',1.5); grid on; hold on
plot(x(1,1),x(2,1),'ko','linewidth',1.5,'markersize',8,'MarkerFaceColor','k');
plot(z(1,:),z(2,:),'-k','linewidth',1.5); 
slacx = 0.0065; slacy = 0.25;
axis([0-slacx 0.2+slacx 0-slacy 7+slacy])
plot(xs(1),xs(2),'kp','markersize',12,'linewidth',2,'MarkerFaceColor','k')
xticks([0 4 8 12 16 20]*1e-2)
yticks([0 1 2 3 4 5 6 7])

%%
Xfeas = Polyhedron([1 0; -1 0; 0 1; 0 -1],[0.2; 0; 7; 0]);
plot(Xfeas,'linestyle','--','facecolor','blue','alpha',0.01);

exportgraphics(gcf, 'val2.pdf', 'ContentType', 'vector');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comparing the two controllers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear x1 x2 
fprintf('Comparing the two controllers... \n')

[X1,X2] = meshgrid(xmin(1):((xmax(1)-xmin(1))/30):xmax(1),xmin(2):((xmax(2)-xmin(2))/30):xmax(2));
x1 = X1(:); x2 = X2(:);
GT = zeros(numel(x1),1); 
NN = zeros(numel(x1),1); 

for i = 1:numel(x1)
    
    % ground-truth
    gt = expMpc.feedback.feval([x1(i);x2(i)]);
    GT(i) = gt(1);
    
    % NN surrogate (what needs to be implemented)
    % scaling the input
    temp = ([x1(i);x2(i)] - xmin) ./ xdelta;
    % regular PWA function evaluation (i.e. approx eMPC)
    [nnTemp, ~] = fitMpc(temp);
    % output affine map
    nn = fit.O*nnTemp + fit.o;
    % scaling control back
    nn = (nn*udelta) + umin;
    NN(i) = max(min(nn,umax),umin);
    
end

% plotting results
figure; 
subplot(1,2,1)
GT = reshape(GT+us,size(X1));
surf(X1+xs(1),X2+xs(2),GT); hold on; grid on; axis([0 0.2 0 7 0 1])
xlabel('\Deltax_1'); ylabel('\Deltax_2'); zlabel('\Deltau'); 
title('Optimal Explicit MPC');
view(75.6,31.2)
subplot(1,2,2)
NN = reshape(NN+us,size(X1));
surf(X1+xs(1),X2+xs(2),NN); hold on; grid on; axis([0 0.2 0 7 0 1])
xlabel('\Deltax_1'); ylabel('\Deltax_2'); zlabel('\Deltau');
title('NN approximation n_z = 3 (6 regions)');
view(75.6,31.2)
set(gcf,'color','w'); set(gcf, 'Position',  [100 100 1200 400]);

% EOF

% Emilio part %

figure
subplot(1,2,1);
expMpc.optimizer.plot(); grid on
title('Original controller');
subplot(1,2,2);
mptSol.plot(); grid on
title('Simplified controller');
%axis([0 T(2) 0 40])
set(gcf,'color','w'); set(gcf, 'Position',  [100 100 1200 400]);
export_fig partitions -pdf -painters
print('part','-dpdf')
%axis([xmin(1)-xs(1) xmax(1)+xs(1) xmin(2)-xs(2) xmax(2)+xs(2)])