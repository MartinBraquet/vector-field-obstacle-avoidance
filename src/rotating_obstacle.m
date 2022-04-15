clear
% parameters of ellipse are time varying


%speed = .5;
speed = 1;
tf = 20/speed;
T = 0:.25:tf;
P = [-2;1];
heading = zeros(2,length(T));
i = 1;
t = 0;

a = @(t) 1+.5*cos(t);
da = @(t) .5*-sin(t);
b = @(t) 2-.5*sin(t);
db = @(t) -.5*cos(t);
a = @(t) 1;
b = @(t) 2;
da = @(t) 0;
db = @(t) 0;
theta = @(t) .75*t;
omega = @(t) .75;
params = struct('a',a,'b',b,'da',da,'db',db,'theta',theta,'omega',omega)

x_min = -5; x_max = 5;
y_min = -5; y_max = 5;

ell = Shape.ellipseMoving(a,b,da,db,theta,omega);
%obs = ObstacleElliptical(ell,[0;0],[-.5;0],0,5,1);
obs = ObstacleElliptical(ell,[0;0],[0;0],0,7,2);
agent = Agent([-10;0],0,speed);
env = Environment(0,agent,obs,[],0.01);
env.iterate(tf);
env.animate_hist();
return

%t = .5;
%P = [-5;-1];
%cavf(P,t,speed,params);
%%return
%
%ode = @(t,X,~) cavf(X,t,speed,params);
%x0 = [-5;0];
%h = .1;
%[path,timestamps] = rk4(ode,x0,[],0,tf,h);
%plot(path(1,:),path(2,:));
%return

for t=T
    %t = 0;
    clf
    hold on;
    axis equal;
    [~,idx] = min(abs(timestamps-t));
    plot(path(1,1:idx),path(2,1:idx),'LineWidth',2);
    scatter(path(1,idx),path(2,idx),50,'filled');

    shape = Shape.ellipse(a(t),b(t));
    obs = ObstacleElliptical(shape,[0;0],[0;0],theta(t),5,1);
    obs.plot(t);
    xrange = x_min:.5:x_max;
    yrange = y_min:.5:y_max;
    [X,Y] = meshgrid(xrange,yrange);
    Hx = zeros(size(X));
    Hy = Hx;
    for(x=xrange)
        for(y=yrange)
            P = [x;y];
            V = cavf(P,t,speed,params);
            Hx(xrange==x, yrange==y) = V(1);
            Hy(xrange==x, yrange==y) = V(2);
        end
    end
    quiver(X,Y,Hx',Hy');
    drawnow
    pause(.01)
    %return
end


function h = cavf(P,t,speed,params)
    a = params.a;
    da = params.da;
    b = params.b;
    db = params.db;
    theta = params.theta;
    omega = params.omega;

    x = @(t,theta) a(t)*cos(theta);
    y = @(t,theta) b(t)*sin(theta);
    dx = @(t,theta) da(t)*cos(theta);
    dy = @(t,theta) db(t)*sin(theta);
    dist_i = 5; a_i = 1;
    phi_des = 0;

    shape = Shape.ellipse(a(t),b(t));

    % alter the cavf algorithm to meet the surface velocity avoidance condition
    V_des = Obstacle.dir_vec(phi_des);
    isInside = shape.isInside;
    closestPoint = shape.closestPoint;

    R = rotation(theta(t));
    Pe = R * P;

    if(isInside(Pe)) % if inside shape, return nan
        h = [nan;nan];
        return
    else
        Ie = closestPoint(Pe);
        I = R' * Ie;
        if( norm(P-I) > dist_i ) % outside influence
            h = V_des;
            return
        else
            d1 = norm(P-I); d2 = d1-dist_i;
            d_sigm = (d1+d2) / (-d1*d2);
            if(d2==0)
                %g = 0;
                g = 1;
            else
                g = a_i*d_sigm/sqrt(1+(2*a_i*d_sigm)^2) + .5;
            end

            th = atan2(I(2)/b(t),I(1)/a(t));
            V_exp = [dx(t,th);dy(t,th)];
            V_rot = cross( [0,0,omega(t)]', [I;0] );
            V_rot = V_rot(1:2);

            V_surf = V_exp + V_rot;
            %keyboard

            n_hat = normalize( P-I , 'norm' );
            t_hat = cross([n_hat;0],[0;0;1]);
            t_hat = t_hat(1:2);
            if( abs(t_hat'*normalize(V_des,'norm')) < 0.01)
                if [0,0,1]*cross([P;0],[V_des;0]) > 0
                    % below obstacle
                    t_hat = -t_hat;
                end
            else
                if(dot(t_hat,normalize(V_des,'norm')) <0)
                    t_hat = -t_hat;
                end
            end

            % required angle at the surface is no longer in t_hat direction
            v_n = V_surf' * n_hat;
            if(speed < v_n)
                v_t = 0; % potentially no escape
            elseif(speed < abs(v_n))
                % negative v_n
                v_t = 0; v_n = -speed;
            else
                v_t = sqrt( speed^2 - v_n^2);
            end
            % velocity required at tangency
            V_t = v_n*n_hat + v_t*t_hat;
            if(~isreal(V_t(1)) || ~isreal(V_t(2)) )
                keyboard
            end
            phi_t = atan2(V_t(2),V_t(1));

            tstar = dot(n_hat,V_des);
            if(tstar > 0) % passed obstacle
                l = -(1-tstar)*(1-g) +1;
            else
                l = g;
            end
            if(phi_des - phi_t > pi)
                phi_t = phi_t + 2*pi;
            elseif(phi_t -phi_des > pi)
                phi_des = phi_des + 2*pi;
            end
            phi_h = (phi_des-phi_t)*l + phi_t;
            h = speed*Obstacle.dir_vec(phi_h);


            return
        end
    end
end


function R = rotation(th)
    R = [cos(th) ,sin(th);
    -sin(th),cos(th);];
end
