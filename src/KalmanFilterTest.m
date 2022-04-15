close all; clear all;
a = 2; b = 5; theta = pi/4;
params = [a, b, theta];
Xo = [0;2];

Xbar0 = [0; 0; 0; 0; 0; 0]; % Initial state estimate of the obstacle
Pxx0 = diag([1 1 1 0 0 0]);
lInit = 0;
isIEKF = 0;

tVec = 0:0.1:10;
V = [1;0];
for t = tVec
    Pa = [0;1] + t * V;
    
    params.Pa = Pa;
    
    % EKF
    [Xhat, Pxx, Pzz, innovations, postFitResiduals] = extendedKalmanFilter(Pxx0, Xbar0, z, R, t, f, STM, gammaNoise, Q, h, dhdx, zError, params);
    Xbar0 = Xhat;
    Pxx0 = Pxx;
end