clc;clear all;close all;

load('n_spiral.mat');
load('thetA_spiral.mat');
load('thetaEst_spiral.mat');
load('thetaEst_Circle.mat');
load('thetaEst_Leminiscate.mat');
load('thetaEst_Bicorn.mat');
%%
result_plot_x(n, thetaA, thetaEst, thetaEst_Circle, thetaEst_Leminiscate, thetaEst_Bicorn);
%close all;
%%
result_plot_y(n, thetaA, thetaEst, thetaEst_Circle, thetaEst_Leminiscate, thetaEst_Bicorn);
%close all;
%%
result_plot_z(n, thetaA, thetaEst, thetaEst_Circle, thetaEst_Leminiscate, thetaEst_Bicorn);
close all;

%% Std Dev Calculation:
x1 = thetaEst(7,20) - thetaA(7,1);
x2 = thetaEst_Bicorn(7,20) - thetaA(7,1);
x3 = thetaEst_Circle(7,20) - thetaA(7,1);
x4 = thetaEst_Leminiscate(7,20) - thetaA(7,1);
xStdDevError = std([x1 x2 x3 x4]);
xMean = (thetaEst(7,20) + thetaEst_Bicorn(7,20) + thetaEst_Circle(7,20) + thetaEst_Leminiscate(7,20))/4;

y1 = thetaEst(8,20) - thetaA(8,1);
y2 = thetaEst_Bicorn(8,20) - thetaA(8,1);
y3 = thetaEst_Circle(8,20) - thetaA(8,1);
y4 = thetaEst_Leminiscate(8,20) - thetaA(8,1);
yStdDevError = std([y1 y2 y3 y4]);
yMean = (thetaEst(8,20) + thetaEst_Bicorn(8,20) + thetaEst_Circle(8,20) + thetaEst_Leminiscate(8,20))/4;

z1 = thetaEst(9,20) - thetaA(9,1);
z2 = thetaEst_Bicorn(9,20) - thetaA(9,1);
z3 = thetaEst_Circle(9,20) - thetaA(9,1);
z4 = thetaEst_Leminiscate(9,20) - thetaA(9,1);
zStdDevError = std([z1 z2 z3 z4]);
zMean = (thetaEst(9,20) + thetaEst_Bicorn(9,20) + thetaEst_Circle(9,20) + thetaEst_Leminiscate(9,20))/4;

