
clear; close all;
params = struct('x',1,'y',3);
rect = struct('params',params,'isInside',@inside_rect,'closestPoint',@closest_rect);
params_e = struct('a',1,'b',3);
    c = @c_ellipse; E = @E_ellipse; dt = @dt_ellipse;
    nearest_ellipse = @(P,params) find_nearest_point(P,c,E,dt,params);
ellipse = struct('params',params_e,'isInside',@inside_ellipse,'closestPoint',nearest_ellipse);
phi_des = 0; d_i = 3; a_i = 1; V_d = 1;
%cavf_plot(ellipse,phi_des,d_i,a_i);
%streamlines_plot(ellipse,phi_des,d_i,a_i);
% moving obstacle
cavf_el = @(P) cavf(P,phi_des,d_i,a_i,ellipse);
h_o = cavf_el([-2;-2]);
V_o = [0,1]';

h_check = cavf_moving([-2;-2],phi_des,V_d,d_i,a_i,V_o,ellipse)
cavf_e = @(P) cavf(P,phi_des,d_i,a_i,ellipse);

cavf_moving_plot(ellipse,phi_des,V_d,d_i,a_i,V_o)
hold on;
plot_ellipse(params_e)

return


phi_des = 0; d_i = 3; a_i = 1;
cavf_plot(rect,phi_des,d_i,a_i)
plot_rect(params)
streamlines_plot(rect,phi_des,d_i,a_i)
dyn = @(t,x,~) cavf(x, phi_des, d_i, a_i, rect);
t0 = 0; tf = 10; t_step = .05;
[X,t,Xdot] = rk4(dyn, [-5;.5],[],t0,tf,t_step);
heading = atan2(Xdot(2,:),Xdot(1,:));
dheading = diff(heading)/t_step;

% dump of superellipse test
function [] = test_superellipse()
    %% testing
    figure(1); clf; hold on;
    %params = struct('a',2, 'b',4, 'n',1.5);
    params = struct('a',2, 'b',4, 'n',1.2);
    p_t = [2,2]';
    c = @c_sellipse; E = @E_sellipse; dt = @dt_sellipse;
    nearest_sellipse = @(P,params) find_nearest_point(P,c,E,dt,params);
    s_ellipse = struct();
    s_ellipse.params = params;
    s_ellipse.closestPoint = nearest_sellipse;
    s_ellipse.isInside = @inside_sellipse;

    phi_des = 0; d_i = 3; a_i = 1;

    xrange = -5:.25:5;
    yrange = xrange;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            % if(x==0 && y== -4) keyboard; end
            P = [x;y];
            V = cavf(P, phi_des, d_i, a_i, s_ellipse);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');
    % rk4 integrate pathlines to check obstacle avoidance
    x=xrange(1);
    for(y=yrange(1:3:end))
        % dynamic equation
        dyn = @(t,x,~) cavf(x, phi_des, d_i, a_i, s_ellipse);
        t0 = 0; tf = 10; t_step = .05;
        [X,t,Xdot] = rk4(dyn, [x;y],[],t0,tf,t_step);
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
    %h = cavf([-2;2],0,3,1,s_ellipse)

    if true
        p_n = find_nearest_point(p_t,c,E,dt,params);
        %p_n = find_nearest_point(p_t,@c_ellipse,@E_ellipse,@dt_ellipse,params);
        %T=0:.01:2*pi;
        T=0:pi*.001:pi/2;

        curve = c(T,params);
        e = E(T,params);
        plot(curve(1,:),curve(2,:))
        plot(-curve(1,:),curve(2,:))
        plot(-curve(1,:),-curve(2,:))
        plot(curve(1,:),-curve(2,:))
        %plot(e(1,:),e(2,:))
        %keyboard
        %scatter(p_t(1),p_t(2))
        %scatter(p_n(1),p_n(2))
        axis equal
    end
end


function cavf_moving_plot(shape,phi_des,V_des,d_i,a_i,V_o)
    phi_o_des = atan2(V_des*sin(phi_des)-V_o(2),V_des*cos(phi_des)-V_o(1));
    xrange = -5:.25:5;
    yrange = xrange;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            % if(x==0 && y== -4) keyboard; end
            P = [x;y];
            V = cavf_moving(P, phi_o_des,V_des,d_i, a_i,V_o, shape);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');
end

function cavf_plot(shape,phi_des,d_i,a_i)
    %phi_des = 0; d_i = 3; a_i = 1;
    xrange = -5:.25:5;
    yrange = xrange;
    yrange = xrange;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            % if(x==0 && y== -4) keyboard; end
            P = [x;y];
            V = cavf(P, phi_des, d_i, a_i, shape);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');
end

function streamlines_plot(shape,phi_des,d_i,a_i)
    % rk4 integrate pathlines to check obstacle avoidance
    hold on;
    x = -5;
    yrange = -3:.5:3;
    for(y=yrange)
        % dynamic equation
        dyn = @(t,X,~) cavf(X, phi_des, d_i, a_i, shape);
        t0 = 0; tf = 10; t_step = .05;
        [X,t,Xdot] = rk4(dyn, [x;y],[],t0,tf,t_step);
        plot(X(1,:),X(2,:),'LineWidth',2);
    end
end

% moving obstacle cavf algo
% generalied cavf algorithm for a moving obstacle
% solves the quadratic formula for the situation where the obstacle is moving
% inputs:
%   point P = [x,y]' : to calculate the cavf heading at
%   phi_des : desired heading
%   V_des : desired speed
%   dist_i : influence distance from obstacle
%   a_i : sharpness parameter 
%   V_o : velocity of obstacle
%   shape : structure representing the shape of the obstacle. contains:
function h = cavf_moving(P,phi_des,V_des,dist_i,a_i,V_o,shape)
    h_o = cavf(P,phi_des,dist_i,a_i,shape);
    
    a = norm(h_o)^2;
    b = 2*( dot(V_o,h_o) );
    c = norm(V_o)^2-V_des^2;
    
    det = b^2-4*a*c;
    if det <0
        % no valid solutions
        h = [nan;nan];
    elseif det ==0
        % one valid solution
        s = -b/(2*a);
        h = s*h_o+V_o;
    else
        % two valid solutions, have to pick the better
        s1 = (-b + sqrt(det)) / (2*a);
        s2 = (-b - sqrt(det)) / (2*a);
        h1 = s1*h_o+V_o;
        h2 = s2*h_o+V_o;
        if( dot( dir_vec(phi_des),h1 ) > dot( dir_vec(phi_des),h2) )
            h = h1;
        else
            h = h2;
        end
    end
end
%
%% cavf algo
% generalized cavf algorithm
% inputs:
%   point P = [x,y]' : to calculate the cavf heading at
%   phi_des : desired heading
%   dist_i : influence distance from obstacle
%   a_i : sharpness parameter 
%   shape : structure representing the shape of the obstacle. contains:
%           @isInside(P,params) : function handle that returns true if P is inside the shape
%           @closestPoint(P,params) : function handle that returns the point on the shape that is closest to the point P
%           params : structure of shape parameters passed to the other functions
function h = cavf(P, phi_des, dist_i, a_i, shape)
    V_des = dir_vec(phi_des);
    isInside = shape.isInside;
    closestPoint = shape.closestPoint;
    params = shape.params;

    if(isInside(P,params)) % if inside shape, return nan
        h = [nan;nan];
        return
    else
        I = closestPoint(P,params);
        if( norm(P-I) > dist_i ) % outside influence
            h = V_des;
            return
        else
            d1 = norm(P-I); d2 = d1-dist_i;
            x = (d1+d2) / (-d1*d2);
            if(d2==0)
                %g = 0;
                g = 1;
            else
                g = a_i*x/sqrt(1+(2*a_i*x)^2) + .5;
            end
            % modulate steer angle from tangent to desired direction
            % take normal as the vector from I to P, pointed out of shape
            n = normalize( P-I , 'norm' );
            % calculate surface tangent that is in the direction of V_des
            t = cross([n;0],[0;0;1]);
            t = t(1:2);
            
            if( abs(t'*normalize(V_des,'norm')) < 0.01)
                if [0,0,1]*cross([P;0],[V_des;0]) > 0
                    % below obstacle
                    t = -t;
                end
            else
                if(dot(t,normalize(V_des,'norm')) <0)
                    t = -t;
                end
            end
            %if(n == [-1;0]); keyboard; end
            phi_t = atan2(t(2),t(1));

            tstar = dot(n,V_des);
            if(tstar > 0) % passed obstacle
                l = -(1-tstar)*(1-g) +1;
            else
                l = g;
            end
            phi_h = (phi_des-phi_t)*l + phi_t;
            h = dir_vec(phi_h);
            %if P(1)==-4; keyboard;end
        end
    end
end

%% helper functions
function v = dir_vec(phi) % calculate unit vector that makes an angle phi to the horizontal
    v = [cos(phi); sin(phi)];
end

%% generic nearest point iterative estimator
% uses the parametric equation for a curve and its evolute to approximate the closest point on the curve to a test point
% iterative process that could be computationally efficient and quickly converging

% estimate the nearest point.
% inputs:
%       p_t: test point outside curve
%       c: function handle of the parametric curve as a function of t : [x,y]' = c(t,params)
%       E: function handle of the parametric evolute of the curve as a function of t: [X,Y]' = E(t,params)
%       dt_func : function handle of the approximate delta t to travel a distance delta c from point p on the curve based on the derivatives of c : delta_t = dt_func(dc,p,params)
%       params: struct of parameters to give to c and E
function p_nearest = find_nearest_point(p_t,c_func,E_func,dt_func,params)
    p = abs(p_t);
    t = atan(p(2)/p(1));
    for(iter = 1:4)
        % point on curve based on guessed t
        i = c_func(t,params);
        % evolute
        e = E_func(t,params);
        % vector from evolute point to guess
        r = i-e;
        % vector from evolute to position
        q = p-e;
        % approx angle *r between r and q; estimation of second intersection point
        dc = (cross([r;0],[q;0]))/norm(q);
        dc = dc(3);
        dt = dt_func(t,dc,i,params);
        % update guess of t
        t = t + dt;
        if(t<0)
            t = 0;
        end
        %t = min
    end
    % calculate point on curve corresponding to t
    p_nearest = c_func(t,params);
    p_nearest = [sign(p_t(1)) 0;
    0   sign(p_t(2))] * p_nearest;
    if(~isreal(p_nearest))
        p_nearest = real(p_nearest);
    end
end

%% ellipse
% parametric curve of an ellipse
function p = c_ellipse(t,params)
    a = params.a; b = params.b;
    p = [a*cos(t); b*sin(t)];
end
% evolute of an ellipse
function e = E_ellipse(t,params)
    a = params.a; b = params.b;
    e = [(a^2-b^2) * cos(t)^3 / a; (b^2-a^2) * sin(t)^3 / b];
end
% dt func of an ellipse
function dt = dt_ellipse(t,dc,p,params)
    a = params.a; b = params.b;
    dt = dc / sqrt(a^2+b^2-norm(p)^2);
end
% inside ellipse
function inside = inside_ellipse(P,params)
    a = params.a; b = params.b;
    inside = P(1)^2/a^2 + P(2)^2/b^2 <= 1;
end
% plot ellipse
function h = plot_ellipse(params)
    t = 0:.01:2*pi;
    P = c_ellipse(t,params)
    plot(P(1,:),P(2,:))
end

%% superellipse
% is inside curve
function inside = inside_sellipse(P,params)
    a = params.a; b = params.b; n = params.n;
    inside = abs(P(1,:)./a) .^n + abs(P(2,:)./b) .^n <1;
end
% parametric curve
function p = c_sellipse(t,params)
    a = params.a; b = params.b; n = params.n;
    x = a.* (cos(t)).^(2./n);
    y = b.* (sin(t)).^(2./n);
    p = [x;y];
end
% evolute
function e = E_sellipse(t,params)
    t_lim = .075;
    if(t < t_lim); t = t_lim; end
    if(t > pi/2-t_lim); t = pi/2-t_lim; end
    c = c_sellipse(t,params);
    x = c(1,:); y = c(2,:);
    dc = dc_sellipse(t,params);
    dx = dc(1,:); dy = dc(2,:);
    ddc = ddc_sellipse(t,params);
    ddx = ddc(1,:); ddy = ddc(2,:);

    X = x - ( dy.*(dx.^2+dy.^2) ) ./ (dx.*ddy - ddx.*dy);
    Y = y + ( dx.*(dx.^2+dy.^2) ) ./ (dx.*ddy - ddx.*dy);
    e = [X;Y];
end
% dt func
function dt = dt_sellipse(t,deltac,p,params)
    dc = dc_sellipse(t,params);
    dcdt = norm(dc);
    if(abs(dcdt) > 1e6); dt = 0; return; end
    if(abs(dcdt) <.001); dt = 0; return; end;
    dt = dcdt.^-1 .* deltac;
end
% derivative
function dc = dc_sellipse(t,params)
    a = params.a; b = params.b; n = params.n;
    xd = -a.*2./n .* sin(t) .* (cos(t)).^(2./n-1);
    yd =  b.*2./n .* cos(t) .* (sin(t)).^(2./n-1);
    dc = [xd;yd];
end
% second derivative
function ddc = ddc_sellipse(t,params)
    a = params.a; b = params.b; n = params.n;
    xdd = 2.*a./n .* ( (2./n-1).*sin(t).^2 .* (cos(t)).^(2./n-2) - (cos(t)).^(2./n) );
    ydd = 2.*b./n .* ( (2./n-1).*cos(t).^2 .* (sin(t)).^(2./n-2) - (sin(t)).^(2./n) );
    ddc = [xdd; ydd];
end

%% rectangle
% params are: x half-width, y half-width
% is inside rectangle
function inside = inside_rect(P,params)
    x = params.x; y = params.y;
    inside = all( abs(P) <= [x;y] );
end
% closest point: simple since it is a rectangle
function P_closest = closest_rect(P,params)
    x = params.x; y = params.y;
    P_abs = abs(P);
    if all( P_abs > [x;y] )
        P_closest = [x;y];
    elseif P_abs(1) > x
        P_closest = [x;P_abs(2)];
    else 
        P_closest = [P_abs(1);y];
    end
    P_closest = P_closest .* sign(P);
end
% plot rectangle
function h = plot_rect(params)
    x = params.x; y = params.y;
    h = patch([-x,x,x,-x],[-y,-y,y,y],[.2,.2,.5])
end
