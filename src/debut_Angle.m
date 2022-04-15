close all; clear all;
figure; hold on;
plot(cos(0:0.1:2*pi), sin(0:0.1:2*pi));
for tau = 0:0.1:2*pi
    I = [cos(tau); sin(tau)];
    n_hat = I/4;
    ObsXf = [1;0];

    th = atan2([0 0 1]*cross([ObsXf;0],[n_hat;0]),dot(n_hat,ObsXf)) / 2;
    R = [cos(th) sin(th); -sin(th) cos(th)];
    
    n_hat = R* n_hat;
    quiver(I(1),I(2),n_hat(1),n_hat(2));
end
% 
% n_hat = [0;-1];
% ObsXf = [1;0];
% th = atan2([0 0 1]*cross([ObsXf;0],[n_hat;0]),dot(n_hat,ObsXf))