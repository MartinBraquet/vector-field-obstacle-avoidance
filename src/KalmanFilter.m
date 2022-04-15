%% Extended Kalman Filter
% Author: Martin Braquet

clear all
close all
format long g

addpath('../../Functions database');

ratioFigPoints = 2;
saveTikzFigures = 0;

data = load('ASE381P_ProjectData.dat');
t = data(:,1);
z = data(:,2:end)';
[m, l] = size(z);
n = 7;
d = 4;

useNLKF = 0;

% The first measurement is done at the initial state x0
isFirstMeasInitState = 1;

% Measurement noise
epsbar = zeros(m,1);
R = diag([0.5 0.1*pi/180 0.2*pi/180]).^2;
zNames{1} = '$\rho - \hat{\rho}$ [m]';
zNames{2} = '$\alpha - \hat{\alpha}$ [rad]';
zNames{3} = '$\beta - \hat{\beta}$ [rad]';
zScaledNames{1} = '$(\rho - \hat{\rho})/\sigma_{\rho}$';
zScaledNames{2} = '$(\alpha - \hat{\alpha})/\sigma_{\alpha}$';
zScaledNames{3} = '$(\beta - \hat{\beta})/\sigma_{\beta}$';

% Process noise
Vbar = zeros(d,1);
Qpos = 2e-3;
Qomega = 1e-11;
Q = diag([Qpos Qpos Qpos Qomega]);

w0 = 1e-3;



% meanInnovationsnoInit = mean(innovations')
% meanPostFitResidualsnoInit = mean(postFitResiduals')
% varInnovationsnoInit = var(innovations')
% varPostFitResidualsnoInit = var(postFitResiduals')
% 
% plotResiduals([], innovations, postFitResiduals, t, 'EKF Residuals noInit', zNames, 3*ratioFigPoints);
% plotScaledResiduals([], innovations, postFitResiduals, Pzz, t, 'EKF Scaled Residuals noInit', zScaledNames, 3*ratioFigPoints, saveTikzFigures);
% 
% plotEstimatedState(t, Xhat, Pxx, XMeas, 'EKF noInit', 3*ratioFigPoints, saveTikzFigures);
% 
% plotNIS(innovations, Pzz, t, 'EKF NIS noInit', ratioFigPoints, saveTikzFigures);
% 
% plotCovariance(t, Pxx, ratioFigPoints, saveTikzFigures);


%% Functions

function output = h(X,params)
% Measurement model

dim = length(params.Pa);

Pa = params.Pa;
pos = X(1:dim);
posR = pos - Pa;

rho = sqrt(sum(posR.^2));

output = zeros(dim,1);

output(1) = rho;
output(2) = atan2(posR(1),posR(2));

end

function output = dhdx(X,params)
% Partial of the measurement with respect to the state

dim = length(params.Pa);

Pa = params.Pa;
pos = X(1:dim);
posR = pos - Pa;

dx = posR(1); dy = posR(2);
if dim == 3
    dz = posR(3);
end

rho = sqrt(sum(posR.^2));

output = zeros(dim,length(X));

if dim == 2
    output(1,1:2) = posR' / rho;
    output(2,1:2) = 1 / (1 + (dx/dy)^2) / dy * [1, -dx/dy];
else
    output(1,1:3) = posR' / rho;
    output(2,1:2) = 1 / (1 + (dx/dy)^2) / dy * [1, -dx/dy];
    output(3,1) = - dz * dx / rho^3 / sqrt(1 - (dz/rho)^2);
    output(3,2) = - dz * dy / rho^3 / sqrt(1 - (dz/rho)^2);
    output(3,3) = sqrt(1 - (dz/rho)^2) / rho;
end

end

function dz = zError(z, zbar)
% Measurement error where the angles are wrapped around the circle

dim = length(zbar);

dz = z - zbar;
dz(2:dim) = wrapToPi(dz(2:dim));
    
end
% 
% function Xcart = sph2cartProject(Xsph, ~)
% % Transform measurements in spherical to cartesian coordinates
% 
% r = Xsph(1);
% az = Xsph(2);
% elev = Xsph(3);
% 
% xs = 1000;
% ys = 1000;
% zs = 20;
% 
% zr = r .* sin(elev);
% rcoselev = r .* cos(elev);
% xr = rcoselev .* sin(az);
% yr = rcoselev .* cos(az);
% 
% x = xs + xr;
% y = ys + yr;
% z = zs + zr;
% 
% Xcart = [x;y;z];
% 
% end

function Xbar = f(Xhat, t1, t0)
% Dynamics equation (for UKF)

dt = t1 - t0;

x = Xhat(1);    y = Xhat(2);    z = Xhat(3);
xdot = Xhat(4); ydot = Xhat(5); zdot = Xhat(6);
w = Xhat(end);

Xbar = zeros(length(Xhat),1);

if w > 1e-6
    Xbar(1) = x - ydot * (1-cos(w*dt))/w + xdot * sin(w*dt)/w;
    Xbar(2) = y + xdot * (1-cos(w*dt))/w + ydot * sin(w*dt)/w;
else
    Xbar(1) = x + xdot * dt;
    Xbar(2) = y + ydot * dt;
end

Xbar(3) = z + zdot * dt;
Xbar(4) = xdot * cos(w*dt) - ydot * sin(w*dt);
Xbar(5) = ydot * cos(w*dt) + xdot * sin(w*dt);
Xbar(6) = zdot;
Xbar(7) = w;
 
end

function Phi = STM(t1, t0, X)
% Dynamics: state-transition matrix (for NL KF and EKF)

xdot = X(4);
ydot = X(5);
w = X(end);
dt = t1 - t0;

Phi = zeros(length(X));
Phi([1:3 6:7],[1:3 6:7]) = eye(5);
Phi(4:5,4:5) = eye(2) * cos(w*dt)   + [0 -1; 1 0] * sin(w*dt);
Phi(3,6)     = dt;
Phi(4,end)   = - ydot*dt*cos(w*dt) - xdot*dt*sin(w*dt);
Phi(5,end)   =   xdot*dt*cos(w*dt) - ydot*dt*sin(w*dt);
if w > 1e-6
    Phi(1:2,4:5) = eye(2) * sin(w*dt)/w + [0 -1; 1 0] * (1-cos(w*dt))/w;
    Phi(1,end)   = - ((ydot-xdot*dt*w)*cos(w*dt) + (xdot+ydot*dt*w)*sin(w*dt) - ydot) / w^2;
    Phi(2,end)   =   ((xdot+ydot*dt*w)*cos(w*dt) - (ydot-xdot*dt*w)*sin(w*dt) - xdot) / w^2;
else
    Phi(1:2,4:5) = eye(2) * dt;
end
   
end

function Phi = STMnoOmega(t1, t0)
% Dynamics: state-transition matrix in straight line (omega = 0)

dt = t1 - t0;

Phi = zeros(6);
Phi([1:3 6],[1:3 6]) = eye(4);
Phi(4:5,4:5) = eye(2);
Phi(3,6)     = dt;
Phi(1:2,4:5) = eye(2) * dt;
   
end

function g = gammaNoise(t1, t0, X)
% Dynamics: process noise transition matrix

% X = 1:7;
% t0 = 0;
% t1 = .42;

w = X(end);
dt = t1-t0;

g = zeros(length(X),4);

if w > 1e-6
    g(1:2,1:2) = eye(2) * (1-cos(w*dt))/w^2 + [0 -1; 1 0] * (w*dt-sin(w*dt))/w^2;
    g(4:5,1:2) = eye(2) * sin(w*dt)/w       + [0 -1; 1 0] * (1-cos(w*dt))/w;
    g(3,3)     = dt^2 / 2;
    g(6:7,3:4) = eye(2) * dt;
else
    % Avoid the divide-by-zero
    g(1:3,1:3) = eye(3) * dt^2/2;
    g(4:7,1:4) = eye(4) * dt;
end

end

function g = noGammaNoise(~, ~, X)
% Dynamics: process noise transition matrix without noise

g = zeros(length(X),4);

end

function [] = plotMeasurements(t,Xmeas,ratioFigPoints)
% Plot the measurements in cartesian coordinates

t = t(1:ratioFigPoints:end);
Xmeas = Xmeas(:,1:ratioFigPoints:end);

x = Xmeas(1,:);
y = Xmeas(2,:);
z = Xmeas(3,:);

figure;
plot3(x(:), y(:), z(:));
grid on
% c = 1:numel(t);
% surface([x(:), x(:)], [y(:), y(:)], [z(:), z(:)], [c(:), c(:)], 'EdgeColor','flat', 'FaceColor','none');
% colormap( jet(numel(t)) )
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Drone trajectory (measurements)');

end

function [] = plotEstimatedState(t, X, Pxx, XMeas, figLegend, ratioFigPoints, saveTikzFigures)
% Plot the state estimate and the measurements in cartesian coordinates (1D
% and 3D) and compare to state covariance Pxx

plotVecIndex = 1:ratioFigPoints:t(end);

[n, l] = size(X);

X = X(:,plotVecIndex);
XMeas = XMeas(:,plotVecIndex);

x = X(1,:);
y = X(2,:);
z = X(3,:);

xMeas = XMeas(1,:);
yMeas = XMeas(2,:);
zMeas = XMeas(3,:);

sigmaX = zeros(3,l);
for k = 1:l
    for j = 1:3
        sigmaX(j,k) = sqrt(Pxx{k}(j,j));
    end
end

t = t(plotVecIndex);
sigmaX = sigmaX(:,plotVecIndex);

figure; hold on; grid on
plot3(x,y,z, 'linewidth', 2);
plot3(xMeas,yMeas,zMeas,'.', 'MarkerSize', 2);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend(figLegend, 'Measurements');
title('Drone trajectory (Estimation)');

if saveTikzFigures
    matlab2tikz(sprintf('../report/img/3DmeasEst%s.tex', figLegend))
end

figure();
for j = 1:3
    win(j) = subplot(3,1,j);
end
set(win,'Nextplot','add')

plot(win(1),t,x, 'linewidth', 2);
plot(win(1),t,xMeas,'.', 'MarkerSize', 3);
xlabel(win(1),'time [s]');
ylabel(win(1),'X [m]');

plot(win(2),t,y, 'linewidth', 2);
plot(win(2),t,yMeas,'.', 'MarkerSize', 3);
xlabel(win(2),'time [s]');
ylabel(win(2),'Y [m]');

plot(win(3), t,z, 'linewidth', 2);
plot(win(3),t,zMeas,'.', 'MarkerSize', 3);
xlabel(win(3),'time [s]');
ylabel(win(3),'Z [m]');
legend(win(1),figLegend, 'Measurements');
title(win(1),'Drone trajectory (Estimation)');

if saveTikzFigures
    matlab2tikz(sprintf('../report/img/1DmeasEst%s.tex', figLegend))
end


figure();
for j = 1:3
    win(j) = subplot(3,1,j);
end
set(win,'Nextplot','add')

yLabelNames{1} = '$x - z_x$ [m]';
yLabelNames{2} = '$y - z_y$ [m]';
yLabelNames{3} = '$z - z_z$ [m]';

for j = 1:3
    plot(win(j),t,X(j,:)-XMeas(j,:),'.', 'MarkerSize', 2);
    xlabel(win(j),'time [s]');
    ylabel(win(j), yLabelNames{j}, 'interpreter', 'latex');
    plot(win(j), t, 3 * sigmaX(j,:), 'k--', 'linewidth', 2 )
    plot(win(j), t, -3 * sigmaX(j,:), 'k--', 'linewidth', 2 )
end
legend(win(1), {'Samples', '$\pm 3 \sigma$'}, 'interpreter', 'latex');
title(win(1), 'Drone trajectory state residuals');

if saveTikzFigures
    matlab2tikz(sprintf('../report/img/stateRes%s.tex', figLegend))
end

end

function MSError = cartResidualsProject(X1, X2)
    error = X1 - X2;
    MSError = sqrt(sum(sum(error.^2))/length(X1));
end

function [] = plotCovariance(t, Pxx, ratioFigPoints, saveTikzFigures)
% Plot the split components of the state covariance Pxx

l = size(Pxx,2);
n = size(Pxx{1},1);

figure(); hold on;

sigma2 = zeros(n,l);
for i = 1:n
    for k = 1:l
        sigma2(i,k) = Pxx{k}(i,i);
    end
    subplot(n,1,i);
    semilogy(t(1:ratioFigPoints:end), sigma2(i,1:ratioFigPoints:end), 'linewidth', 2);
    ylabel(sprintf('sigma^2_{%d%d}', i, i));
end
%corr = sigma2(3,:) ./ sqrt(sigma2(1,:) .* sigma2(2,:));

% subplot(2,1,2);
% plot( t, corr, 'linewidth', 2 )
% ylabel({'$\rho_{12}$'}, 'interpreter', 'latex');
% xlabel('time [s]')

end

function [Xhat, Pxx, Pzz, innovations, postFitResiduals] = extendedKalmanFilter(Pxx0, Xbar0, z, R, t, f, STM, gammaNoise, Q, h, dhdx, zError, params)
% Generate state PDF estimate using an Extended Kalman Filter

[m, l] = size(z);
n = size(Xbar0,1);

Xbar = f(Xbar0, t(k), t(k-1));
Phi = STM(t(k), t(k-1), Xbar);
gammaTk = gammaNoise(t(k), t(k-1), Xbar);
Pxxbar = Phi * Pxx0 * Phi' + gammaTk * Q * gammaTk';

dz = zError(z, h(Xbar,params));
H = dhdx(Xbar);
Pzz = H * Pxxbar * H' + R;
K = Pxxbar * H' * inv(Pzz);
dXhat = K * dz;
Xhat = Xbar + dXhat;
Pxx = Pxxbar - K * H * Pxxbar;

% For EKF, pre-fit = Innovations
innovations = dz;
postFitResiduals = dz - H * dXhat;
end