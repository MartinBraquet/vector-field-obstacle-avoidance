close all; clear all;
ro = 1; ri = 4; ao = 0.5;
Xo = [0;0];
Xf = [2;2];
X = [-2;-0];


%% testing ellipse functions
if(true)
    
    % cavf formulation
    htest = cavf(X, Xf, Xo, ro, ri, ao);

    % plot results
    figure(1); clf; hold on; axis equal;
    xlabel('x');
    ylabel('y');
    T = 0:.01:2*pi;
    circle = Xo + ro * [cos(T); sin(T)];
    dXo = [0;2];
    plot(circle(1,:),circle(2,:));
    %scatter(P(1), P(2));
    %scatter(I(1), I(2));
    %quiver(I(1),I(2),n(1)/4,n(2)/4);
    %quiver(I(1),I(2),t(1)/4,t(2)/4);
    %quiver(P(1),P(2),h(1)/4,h(2)/4);
    %plot([I(1) P(1)],[I(2) P(2)]);

    % plot cavf over space
    xrange = -5:.2:5;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            P = [x;y];
            h = cavf(P, Xf, Xo, ro, ri, ao) + cavf(P, Xf, Xo+dXo, ro, ri, ao);
            Hx(xrange==x, yrange==y) = h(1);
            Hy(xrange==x, yrange==y) = h(2);
        end
    end
    quiver(X,Y,Hx',Hy');

    % rk4 integrate pathlines to check obstacle avoidance
%     x=xrange(1);
%     for(y=yrange(1:3:end)+0.01)
%         % dynamic equation
%         dyn = @(t,x,~) cavf(x, Xf, Xo, ro, ri, ao);
%         t0 = 0; tf = 10; dt = .05;
%         [X,t,Xdot] = rk4(dyn, [x;y],[],t0,tf,.01);
%         plot(X(1,:),X(2,:),'LineWidth',2);
%     end

    x=xrange(1);
    for(y=yrange(25))
        % dynamic equation
        dyn = @(t,x,~) doubleIntegrator(x, Xf, Xo, ro, ri, ao);
        t0 = 0; tf = 10; dt = .05;
        [X,t,Xdot] = rk4(dyn, [x;y;0;0],[],t0,tf,.01);
        u = vecnorm(Xdot(3:4,:));
        utot = trapz(t,u.^2)
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
    
    for(y=yrange(25))
        % dynamic equation
        dyn = @(t,x,~) doubleIntegratorKThreshold(x, Xf, Xo, ro, ri, ao);
        t0 = 0; tf = 10; dt = .05;
        [X,t,Xdot] = rk4(dyn, [x;y;0;0],[],t0,tf,.01);
        uKThreshold = vecnorm(Xdot(3:4,:));
        utotKThreshold = trapz(t,uKThreshold.^2)
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
    
    for(y=yrange(25))
        % dynamic equation
        dyn = @(t,x,~) doubleIntegratorK(x, Xf, Xo, ro, ri, ao);
        t0 = 0; tf = 10; dt = .05;
        [X,t,Xdot] = rk4(dyn, [x;y;0;0],[],t0,tf,.01);
        uK = vecnorm(Xdot(3:4,:));
        utotK = trapz(t,uK.^2)
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
    legend('','','u', 'uKThreshold', 'uk');
    
    
    figure; hold on;
    plot(t,u,'LineWidth',2);
    plot(t,uKThreshold,'LineWidth',2);
    plot(t,uK,'LineWidth',2);
    ylabel('u');
    xlabel('time');
    legend('u', 'uKThreshold', 'uk');
end


%% helper functions

function Xdot = doubleIntegrator(X, Xf, Xo, ro, ri, ao)
    n = size(X,1);
    P = X(1:n/2);
    V = X(n/2+1:end);
    
    k = 1;
    h = cavf(P, Xf, Xo, ro, ri, ao);
    errorV = h - V;
    norm(errorV) / norm(h);
    
    dx = ri/100; dX = [dx;0];
    dy = dx; dY = [0;dy];
    dhdx = (cavf(P+dX, Xf, Xo, ro, ri, ao) - cavf(P-dX, Xf, Xo, ro, ri, ao)) / (2*dx);
    dhdy = (cavf(P+dY, Xf, Xo, ro, ri, ao) - cavf(P-dY, Xf, Xo, ro, ri, ao)) / (2*dy);
    gradh = [dhdx, dhdy];
    
    u = k * errorV + gradh * V;
    %norm(k * errorV) / (norm(k * errorV) + norm(gradh * V))
    
    Xdot = [V; u];
end

function Xdot = doubleIntegratorKThreshold(X, Xf, Xo, ro, ri, ao)
    n = size(X,1);
    P = X(1:n/2);
    V = X(n/2+1:end);
    
    k = 15;
    h = cavf(P, Xf, Xo, ro, ri, ao);
    errorV = h - V;
    norm(errorV) / norm(h);
    
    dx = ri/100; dX = [dx;0];
    dy = dx; dY = [0;dy];
    dhdx = (cavf(P+dX, Xf, Xo, ro, ri, ao) - cavf(P-dX, Xf, Xo, ro, ri, ao)) / (2*dx);
    dhdy = (cavf(P+dY, Xf, Xo, ro, ri, ao) - cavf(P-dY, Xf, Xo, ro, ri, ao)) / (2*dy);
    gradh = [dhdx, dhdy];
    
    if (norm(errorV) / norm(h) > 0.5)
        u = k * errorV;
    else
        u = gradh * V;
    end
    Xdot = [V; u];
end

function Xdot = doubleIntegratorK(X, Xf, Xo, ro, ri, ao)
    n = size(X,1);
    P = X(1:n/2);
    V = X(n/2+1:end);
    
    k = 15;
    h = cavf(P, Xf, Xo, ro, ri, ao);
    errorV = h - V;
    norm(errorV);
    
    u = k * errorV;

    Xdot = [V; u];
end

function h = cavf(X, Xf, Xo, ro, ri, ao)

    TargetVec = Xf - X;
    
    ObsXf = Xf - Xo;
    thetaObsXf = atan2(ObsXf(2),ObsXf(1));

    r = norm(X-Xo);
    rVec = X-Xo;
    rUnit = rVec/r;
    if(r > ri)
        ho = zeros(2,1);
        th = 0;
    elseif(r < ro)
        h = zeros(2,1);
        return
    else
        ho = norm(TargetVec) * ro/r * (ri-r)/(ri-ro) * rUnit;
        
        thXXo = atan2(rVec(2),rVec(1));
        th = -wrapToPi(thXXo - thetaObsXf) / 2;
        
        d1 = r - ri; d2 = r - ro;
        x = (d1 + d2) / (d1*d2);
        if(d2==0)
            g = 1;
        else
            g = ao*x/sqrt(1+(2*ao*x)^2) + .5;
        end

        th = g * th;
        
    end
    
    hf = TargetVec;
    h = hf + ho;
    
    R = [cos(th) sin(th); -sin(th) cos(th)];
    h = R * h;
end
