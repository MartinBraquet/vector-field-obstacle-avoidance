% use the general idea of the paper to generate a cavf for any convex? obstacle
clear;
% approach:
%   for an obstacle, find point i on the boundary that is nearest to the position p
%   find the tangent of the obstacle at i
%   use the lambda function to determine the direction of the vector field
%       limits are the tangent on the obstacle boundary, and the desired heading at the influence boundary

% questions:
%   is this computationally effective?
%   is the cavf valid (continuous, smooth, sensible)?
%   does this create a controllable cavf?
%   is it possible/necessary to create an open loop steering control from the cavf?
%   the implication of the 


% first example will be with an ellipsoidal obstacle (ironic)

a = .5; b = 3; theta = pi/4;
phi_des = -pi/4;
phi_des = 0;
ellipse_params = [a, b, theta]; % x' and y' axes lengths and angle of major axis relative to the horizontal
% theta is backwards in cod?
P = [-3.5;-0];

%% testing ellipse functions
if(true)
    % point closest to P on ellipse
    I = ellipse_closest(P, ellipse_params)
    % normal of ellipse at a point on its boundary; normal is equivalent to the normalized vector from I to P
    n = ellipse_normal(I, ellipse_params)
    % tangent that is in the direction of desired travel
    t = ellipse_tangent(I, phi_des, ellipse_params)

    % cavf formulation
    % distance out of the obstacle that the cavf influences and sharpness parameter
    d_i = 2; a_i = .5;
    h = cavf(P, phi_des, d_i, a_i, ellipse_params)

    % plot results
    figure(1); clf; hold on; axis equal;
    T = 0:.01:2*pi;
    Ellipse = [cos(theta) sin(theta); -sin(theta) cos(theta)] * [ a*cos(T); b*sin(T) ];
    plot(Ellipse(1,:),Ellipse(2,:));
    %scatter(P(1), P(2));
    %scatter(I(1), I(2));
    %quiver(I(1),I(2),n(1)/4,n(2)/4);
    %quiver(I(1),I(2),t(1)/4,t(2)/4);
    %quiver(P(1),P(2),h(1)/4,h(2)/4);
    %plot([I(1) P(1)],[I(2) P(2)]);

    % plot cavf over space
    xrange = -5:.25:-2;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            % if(x==0 && y== -4) keyboard; end
            P = [x;y];
            V = cavf(P, phi_des, d_i, a_i, ellipse_params);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');

    % rk4 integrate pathlines to check obstacle avoidance
    x=xrange(1);
    for(y=yrange(1:3:end))
        % dynamic equation
        dyn = @(t,x,~) cavf(x, phi_des, d_i, a_i, ellipse_params);
        t0 = 0; tf = 10; dt = .05;
        [X,t,Xdot] = rk4(dyn, [x;y],[],t0,tf,.01);
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
end


%% helper functions

function h = cavf(P, phi_des, dist_i, a_i, params)
    % desired velocity
    V_des = dir_vec(phi_des);
    I = ellipse_closest(P, params);
    if(norm(P-I) > dist_i)
        h = V_des;
    elseif(isnan(I)) % calculate cavf
        h = I;
    else
        d1 = norm(P-I); d2 = d1-dist_i;
        x = (d1 + d2) / (-d1*d2);
        if(d2==0)
            g = 0;
        else
            g = a_i*x/sqrt(1+(2*a_i*x)^2) + .5;
        end
        % modulation of steer angle from the tangent to the desired direction
        t = ellipse_tangent(I, phi_des, params);
        phi_t = atan2(t(2),t(1));
        % adjust modulation if 'passed obstacle'
        tstar = dot(ellipse_normal(I, params),V_des);
        if(tstar > 0) % passed
            %if(P(1)==.5 && P(2)==-1) keyboard; end
            l = -(1-tstar)*(1-g) +1;
        else
            l = g;
        end
        phi_h = (phi_des-phi_t)*l + phi_t;
        h = dir_vec(phi_h);
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
