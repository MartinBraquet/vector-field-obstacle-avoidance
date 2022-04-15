close all; clear all;
a = 2; b = 5; theta = pi/4;
params = [a, b, theta]; % x' and y' axes lengths and angle of major axis relative to the horizontal
% theta is backwards in cod?
P = [-10;0];
Xo = [100;0];
Xf = [8;0];

%% testing ellipse functions
if(true)
%     % point closest to P on ellipse
%     I = ellipse_closest(P, params)
%     % normal of ellipse at a point on its boundary; normal is equivalent to the normalized vector from I to P
%     n = ellipse_normal(I, params)
%     % tangent that is in the direction of desired travel
%     t = ellipse_tangent(I, phi_des, params)

    % cavf formulation
    % distance out of the obstacle that the cavf influences and sharpness parameter
    di = 4; ao = .5;
    h = cavf(P, Xf, Xo, di, ao, params)

    % plot results
    figure(1); clf; hold on; axis equal;
    T = 0:.01:2*pi;
    Ellipse = Xo + [cos(theta) sin(theta); -sin(theta) cos(theta)] * [ a*cos(T); b*sin(T) ];
    plot(Ellipse(1,:),Ellipse(2,:));
    plot(Xf(1), Xf(2), '*');
    %scatter(P(1), P(2));
    %scatter(I(1), I(2));
    %quiver(I(1),I(2),n(1)/4,n(2)/4);
    %quiver(I(1),I(2),t(1)/4,t(2)/4);
    %quiver(P(1),P(2),h(1)/4,h(2)/4);
    %plot([I(1) P(1)],[I(2) P(2)]);

    % plot cavf over space
    xrange = -10:.5:10;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            % if(x==0 && y== -4) keyboard; end
            P = [x;y];
            V = cavf(P, Xf, Xo, di, ao, params);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');
    

    % rk4 integrate pathlines to check obstacle avoidance
%     x=xrange(1);
%     for(y=yrange(1:3:end))
%         % dynamic equation
%         dyn = @(t,x,~) cavf(x, Xf, Xo, di, ao, ellipse_params);
%         t0 = 0; tf = 10; dt = .05;
%         [X,t,Xdot] = rk4(dyn, [x;y],[],t0,tf,.01);
%         plot(X(1,:),X(2,:),'LineWidth',2);
%     end

    x=xrange(1);
    for(y=yrange(20))
        % dynamic equation
        dyn = @(t,x,~) doubleIntegrator(x, Xf, Xo, di, ao, params);
        t0 = 0; tf = 10; dt = .05;
        [X,t,Xdot] = rk4(dyn, [x;y;0;0],[],t0,tf,.01);
        u = vecnorm(Xdot(3:4,:));
        utot = trapz(t,u.^2)
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
    
    figure; hold on;
    plot(t,u,'LineWidth',2);
    ylabel('u');
    xlabel('time');
end


%% helper functions

function Xdot = doubleIntegrator(X, Xf, Xo, di, ao, params)
    n = size(X,1);
    P = X(1:n/2);
    V = X(n/2+1:end);
    
    k = 1;
    h = cavf(P, Xf, Xo, di, ao, params);
    errorV = h - V;
    norm(errorV) / norm(h);
    
    dx = di/100; dX = [dx;0];
    dy = dx; dY = [0;dy];
    dhdx = (cavf(P+dX, Xf, Xo, di, ao, params) - cavf(P-dX, Xf, Xo, di, ao, params)) / (2*dx);
    dhdy = (cavf(P+dY, Xf, Xo, di, ao, params) - cavf(P-dY, Xf, Xo, di, ao, params)) / (2*dy);
    gradh = [dhdx, dhdy];
    
    u = k * errorV + gradh * V;
    %norm(k * errorV) / (norm(k * errorV) + norm(gradh * V))
    
    Xdot = [V; u];
end


function h = cavf(P, Xf, Xo, di, ao, params)

    TargetVec = Xf - P;
    phi_des = atan2(TargetVec(2),TargetVec(1));
    V_des = TargetVec;
    
    ObsXf = Xf - Xo;
    thetaObsXf = atan2(ObsXf(2),ObsXf(1));

    r = norm(P-Xo);
    rVec = P-Xo;
    rUnit = rVec/r;

    I = ellipse_closest(P, params);
    if(norm(P-I) > di)
        h = TargetVec;
    elseif(isnan(I))
        h = [0;0];
    else
        ro = norm(Xo-I);
        d1 = norm(P-I); d2 = d1-di;
        x = (d1 + d2) / (d1*d2);
        if(d2==0)
            g = 1;
        else
            g = ao*x/sqrt(1+(2*ao*x)^2) + .5;
        end
        % modulation of steer angle from the tangent to the desired direction
        t = ellipse_tangent(I, phi_des, params);
        n = ellipse_normal(I, params);
        % adjust modulation if 'passed obstacle'
        tstar = dot(ellipse_normal(I, params),V_des);
        if(tstar > 0) % passed
            %if(P(1)==.5 && P(2)==-1) keyboard; end
            l = -(1-tstar)*(1-g) +1;
        else
            l = g;
        end
        
        thn = atan2(n(2),n(1));
        th = -wrapToPi(thn - thetaObsXf) / 2 * g;
        
        d = norm(P-I); db = 1;
        ho = norm(TargetVec) * db/(d+db) * (di-d)/di * n;
        if (ho(1) > 50)
            keyboard;
        end
        
        hf = TargetVec;
        h = hf + ho;
        
        R = [cos(th) sin(th); -sin(th) cos(th)];
        h = R * h;
    end
end

% calculate tangent of the ellipse at the point I on the boundary that is closest to the phi_des direction
function t = ellipse_tangent(I, phi_des, params)
    a = params(1); b = params(2); theta = params(3);
    % get normal
    n = ellipse_normal(I, params);
    % normal cross z since in in 2D; would have to be more smart if extended to 3D
    t = cross([n;0],[0;0;1]);
    t = t(1:2);
    % choose +-t depending on direction of phi_des
    if(dir_vec(phi_des)'*t < 0)
        t = -t;
    end
end

function v = dir_vec(phi) % calculate unit vector that makes an angle phi to the horizontal
    v = [cos(phi); sin(phi)];
end

% calculate the unit normal pointing out of the ellipse at the point I on the boundary
function n = ellipse_normal(I, params)
    a = params(1); b = params(2); theta = params(3);
    % rotate I
    R_OI = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    I_O = R_OI'*I;
    if(I_O(2)==0)
        n_O = -[1;0];
    elseif(I_O(1)==0)
        n_O = -[0;1];
    else

        mt = -b^2/a^2 * I_O(1)/I_O(2);
        mn = -mt^-1; % slope of normal
        n_O = normalize([1;mn],'norm');
    end
    % condition n_O to always point out of ellipse
    n_O = n_O * sign(dot(n_O,I_O));
    n = R_OI * n_O;
    % keyboard
    % if(I(1)==0) s1 = 1; else s1 = sign(I(1)); end
    % if(I(2)==0) s2 = 1; else s2 = sign(I(2)); end
    % n = - n * s1*s2;
    % keyboard
end


% approximate the point on the ellipse that is closest to the point p. returns [nan;nan] if the point is inside the ellipse
% ellipse_closest(p,params)
%   p - point to evaluate
%   params - ellipse major and minor axes and angle; equivalent to the matrix representation but easier to work with in a geometric sense
%   ellipse center is at the origin
function i = ellipse_closest(p_I, params)
    a = params(1); b = params(2); theta = params(3);
    % apply rotation to p from I to O frame
    R_OI = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    p_O = R_OI'*p_I;

    A = [a^-2 0; 0 b^-2];
    % check if within ellipse
    if( p_O' * A * p_O <1 )
        i =  [nan;nan];
    else
        p = abs(p_O);
        % algorithm taken from 
        %   https://wet-robots.ghost.io/simple-method-for-distance-to-ellipse/
        % initial guess of angle of incidence
        t = atan(p(2)/p(1));
        for(iter = 1:4) % 4 iterations
            i = [a*cos(t); b*sin(t)];
            % evolute
            e = [(a^2-b^2) * cos(t)^3 / a; (b^2-a^2) * sin(t)^3 / b];
            % vector from evolute point to guess
            r = i-e;
            % vector from evolute to position
            q = p-e;
            % approx angle * |r| between r and q; indication of error of guess; uses small angle approximation
            dc = (cross([r;0],[q;0]))/norm(q);
            dc = dc(3); % account for sign of dc
            dt = dc / sqrt(a^2+b^2-norm(i)^2);

            % update guess of t, limit to the upper quadrant for stability
            t = t + dt;
            t = min(pi/2, max(t,0));
        end
        % calculate point on ellipse corresponding to t
        i_O = [sign(p_O(1)) 0;
        0   sign(p_O(2))] * [a*cos(t);b*sin(t)];
        % rotate back to I
        i = R_OI * i_O;
    end
end
