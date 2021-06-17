clc;
clear;
close all;

%{
Rc = 0.5;
Rl = 1.6;
Ron = 0.05;
R1 = 2e3;
R2 = 5e3;
Ro = 100;
C = 56e-6;
L = 10e-3;
Vin = 12;
Vj = 0.2;
x2_eq = 5;
%}

sympref('FloatingPointOutput',true);
syms Rc Rl Ron Ro C L Vin Vj x2_eq x1 u
eqns = [-Rl/L*x1-1/L*x2_eq-Ron/L*x1*u+(Vin+Vj)/L*u-Vj/L == 0,...
        ((-Rc*Ro*Rl*C+Ro*L)/C*x1 - (Rc*Ro*C+L)/C*x2_eq + Rc*Ro*(Vin+Vj)*u - Rc*Ro*Ron*x1*u -Rc*Ro*Vj) == 0];
    
S = solve(eqns,[x1,u]);
x1 = S.x1
u = S.u

