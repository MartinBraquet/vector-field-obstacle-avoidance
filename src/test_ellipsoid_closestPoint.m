close all; clear all;
a=5; b=3; c = 2;
ell = Shape.ellipsoid(a,b,c);

P = [0.1;10;4];

Pell = ell.closestPoint(P);
r = [P Pell];

[X,Y,Z] = ellipsoid(0,0,0,a,b,c);

figure; hold on; grid on;
surf(X,Y,Z);
plot3(P(1),P(2),P(3), '.', 'Markersize', 20);
plot3(Pell(1),Pell(2),Pell(3), '.', 'Markersize', 20);
plot3(r(1,:),r(2,:),r(3,:));
axis equal