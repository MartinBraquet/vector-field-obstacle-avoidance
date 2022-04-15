clear all; close all;
obstacle_case = 'movingellipse'
anim_name = 'xxx.gif'
animate = true
multiplot = false

uMax = 20e16;

switch obstacle_case

    case 'ellipse'
        a=0.1; b=.2;
        ell = Shape.ellipse(a,b);
        Po = [0.3;0.5]; 
        Vo = [0;0];
        theta = 0;
        %theta = pi/2;
        %theta = pi/4;
        d_i = 0.3;
        a_i = 0.01;
        obs_ell = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i);
        X0agent = [0;0.25];
        V0agent = [0;0];
        Xfagent = [1;0.75];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs_ell,[],0.01);
        tf = 5;

    case 'movingellipse'
        a=0.1; b=.2;
        ell = Shape.ellipse(a,b);
        Po = [.6;0.2]; 
        Vo = [0;0.5];
        theta = 0;
        d_i = 0.3;
        a_i = 0.01;
        obs_ell = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i);
        X0agent = [0.2;0.2];
        V0agent = [0;0];
        Xfagent = [.8;0.6];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs_ell,[],0.01);
        tf = 5;
        
    case 'ellipsoid'
        a=1; b=2; c=3;
        ell = Shape.ellipsoid(a,b,c);
        Po = [0;-1;0];
        Vo = [0;0;0];
        theta = 0;
        beta = pi/4;
        %theta = pi/4;
        d_i = 5;
        a_i = 0.3;
        obs_ell = ObstacleEllipsoidal(ell,Po,Vo,[theta,beta],d_i,a_i);
        X0agent = [-10;10;0];
        V0agent = [0;0;0];
        Xfagent = [10;-10;0];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        frame = [-10;10;-10;10;-10;10];
        env = Environment(0,agent,obs_ell,frame, 0.01);
        tf = 5;

    case 'multiellipse'
        a=0.05; b=0.1;
        ell = Shape.ellipse(a,b);
        Po = [0.4;0.5]; 
        Vo = [0;0];
        theta = pi/2;
        d_i = 0.3;
        a_i = 0.01;
        obs_1 = ObstacleElliptical(ell,Po,Vo,0,d_i,a_i);
        obs_2 = ObstacleElliptical(ell,[0.7;0.7],Vo,0,d_i,a_i);
        obs_3 = ObstacleElliptical(ell,[0.6;0.2],Vo,0,d_i,a_i);
        obs = [obs_1 obs_2 obs_3];
        
        X0agent = [0.4;0.05];
        V0agent = [0;0];
        Xfagent = [0.6;0.8];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs,[],0.01);
        tf = 10;
        
    case 'multiellipsoid'
        a=3; b=2; c=3;
        ell = Shape.ellipsoid(a,b,c);
        Po = [-2;-2;-1];
        Vzero = [0;0;0];
        Vo = [0;0;1];
        Po2 = [2;2;2];
        Vo2 = [0;0;-1];
        Po3 = [6;2;0];
        Vo3 = [0;-1;0];
        theta = pi/6;
        beta = pi/4;
        %theta = pi/4;
        d_i = 3;
        a_i = 0.3;
%         obs_ell = ObstacleEllipsoidal(ell,Po,Vo,[theta,beta],d_i,a_i);
%         obs_ell2 = ObstacleEllipsoidal(ell,Po2,Vo2,[-theta,-beta],d_i,a_i);
%         obs_ell3 = ObstacleEllipsoidal(ell,Po3,Vo3,[theta,-beta],d_i,a_i);
        obs_ell = ObstacleEllipsoidal(ell,Po,Vzero,[theta,beta],d_i,a_i);
        obs_ell2 = ObstacleEllipsoidal(ell,Po2,Vzero,[-theta,-beta],d_i,a_i);
        obs_ell3 = ObstacleEllipsoidal(ell,Po3,Vzero,[theta,-beta],d_i,a_i);
        obs = [obs_ell obs_ell2 obs_ell3];
        X0agent = [-10;0;0];
        V0agent = [0;0;0];
        Xfagent = [10;0;0];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        frame = [-10;10;-10;10;-10;10];
        env = Environment(0,agent,obs,frame,.01);
        tf = 5;

    case 'timevaryingshapeellipse'
        delta = .6;
        adot = .0075; a0 = .1;
        bdot = .0175; b0 = .05;
        a = @(t) a0+adot*t;
        da = @(t) adot*t;
        b = @(t) b0+bdot*t;
        db = @(t) bdot*t;

        a2 = @(t) a0+adot/2*t;
        da2 = @(t) adot/2*t;
        b2 = @(t) b0+bdot/2*t;
        db2 = @(t) bdot/2*t;

        theta = @(t) 0;
        omega = @(t) 0;
        ell1 = Shape.ellipseMoving(a,b,da,db,theta,omega);
        ell2 = Shape.ellipseMoving(a2,b2,da2,db2,theta,omega);

        Po = [.2;0.5]; Po2_off = [.35;.35];
        Vo = [0;-0.25];
        theta = 0;
        d_i = 1;
        a_i = 0.03;
        obs1 = ObstacleElliptical(ell1,Po,Vo,theta,d_i,a_i);
        obs2 = ObstacleElliptical(ell2,Po+Po2_off,Vo,theta,d_i,a_i);
        obs = [obs1,obs2];
        
        X0agent = [0;0.5];
        V0agent = [0.1;0];
        Xfagent = [0.8;0.5];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs,[],0.01);
        tf = 10;
        
    case 'timevaryingshapeellipsoid'
        delta = 6;
        adot = .15; a0 = 2;
        bdot = .35; b0 = 1;
        cdot = .35; c0 = 1;
        a = @(t) a0+adot*t;
        da = @(t) adot*t;
        b = @(t) b0+bdot*t;
        db = @(t) bdot*t;
        c = @(t) c0+cdot*t;
        dc = @(t) cdot*t;

        a2 = @(t) a0+adot*2*t;
        da2 = @(t) adot*2*t;
        b2 = @(t) b0+bdot*t;
        db2 = @(t) bdot*t;
        c2 = @(t) c0+cdot/2*t;
        dc2 = @(t) cdot/2*t;

        theta = @(t) 0;
        beta = @(t) 0;
        omega = @(t) 0;
        ell1 = Shape.ellipsoidMoving(a,b,c,da,db,dc,theta,beta,omega);
        ell2 = Shape.ellipsoidMoving(a2,b2,c2,da2,db2,dc2,theta,beta,omega);
        
        Po = [-2;-2;-1];
        Vo = [0;0;1];
        Po2 = [2;2;2];
        Vo2 = [0;0;-1];
        theta = 0;
        beta = pi/4;
        d_i = 3;
        a_i = 0.3;
        obs1 = ObstacleEllipsoidal(ell1,Po,Vo,[theta,beta],d_i,a_i);
        obs2 = ObstacleEllipsoidal(ell2,Po2,Vo2,[theta,-beta],d_i,a_i);
        obs = [obs1 obs2];
        
        X0agent = [-10;0;0];
        V0agent = [0;0;0];
        Xfagent = [10;0;0];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        frame = [-10;10;-10;10;-10;10];
        env = Environment(0,agent,obs,frame,[],0.01);
        tf = 5;

    case 'multiellipseWall'
        a=2; b=4;
        ell = Shape.ellipse(a,b);
        xmax = 10;
        ell2 = Shape.ellipse(10*xmax,xmax/100);
        Po = [0;0.5]; 
        Vo = [0;0];
        theta = pi/2;
        d_i = 5;
        a_i = 0.3;
        obs_1 = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i)
        obs_2 = ObstacleElliptical(ell2,[-10;0],[0;0],pi/2,d_i,a_i)
        obs_3 = ObstacleElliptical(ell2,[10;0],[0;0],pi/2,d_i,a_i)
        obs_4 = ObstacleElliptical(ell2,[0;10],[0;0],0,d_i,a_i)
        obs_5 = ObstacleElliptical(ell2,[0;-10],[0;0],0,d_i,a_i)
        obs = [obs_1 obs_2 obs_3 obs_4 obs_5];
        
        X0agent = [-8;0];
        V0agent = [0;0];
        Xfagent = [8;0];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs,[-10;10;-10;10],.01);
        tf = 5;

    case 'multiellipseSquare'
        a=1; b=2;
        ell = Shape.ellipse(a,b);
        ws = 2;
        ell2 = Shape.ellipse(ws,ws/100);
        Po = [-3;0]; 
        Vo = [0;0];
        theta = pi/2;
        d_i = 5;
        a_i = 0.3;
        obs_1 = ObstacleElliptical(ell,[2;2],Vo,theta,d_i,a_i)
        obs_2 = ObstacleElliptical(ell2,Po+[0;ws],[0;0],0,d_i,a_i)
        obs_3 = ObstacleElliptical(ell2,Po+[0;-ws],[0;0],0,d_i,a_i)
        obs_4 = ObstacleElliptical(ell2,Po+[-ws;0],[0;0],pi/2,d_i,a_i)
        obs_5 = ObstacleElliptical(ell2,Po+[ws;0],[0;0],pi/2,d_i,a_i)
        obs = [obs_1 obs_2 obs_3 obs_4 obs_5];
        
        X0agent = [-8;0];
        V0agent = [0;0];
        Xfagent = [8;0];
        agent = Agent(X0agent,V0agent,Xfagent,uMax);
        env = Environment(0,agent,obs,[-10;10;-10;10],0.01);
        tf = 5;

    % Ellipse with uncertain motion, growing probability ellipsoid.
    % This probability is reduced by using embedded sensors so that a Kalman 
    % filter merges the data from the dynamics/prediction and the
    % measurements. THe uncertainty over an obstacle should be reduced the
    % closer is the agent to this specific obstacle
    % TOFINISH    
%     case 'uncertellipseKalman'
%         delta = 6;
%         adot = .15; a0 = 2;
%         bdot = .35; b0 = 2;
%         a = @(t) a0+adot*t;
%         da = @(t) adot*t;
%         b = @(t) b0+bdot*t;
%         db = @(t) bdot*t;
% 
%         theta = @(t) 0;
%         omega = @(t) 0;
%         ell1 = Shape.ellipseMoving(a,b,da,db,theta,omega);
% 
%         Po = [0.5;0];
%         Vo = [-0.75;-0.5];
%         theta = 0;
%         d_i = 10;
%         a_i = 0.3;
%         obs1 = ObstacleElliptical(ell1,Po,Vo,theta,d_i,a_i);
%         obs = [obs1];
%         
%         X0agent = [-10;0];
%         V0agent = [1;0];
%         Xfagent = [8;0];
%         agent = Agent(X0agent,V0agent,Xfagent,uMax);
%         env = Environment(0,agent,obs,[-10;10;-10;10],0.01);
%         tf = 10;
%         
%     case 'uncertobject'
%         delta = 6;
%         
%         Po = [-6;0];
%         Vo = [1;0];
%         
%         adot = 0.1*Vo(1); a0 = 1;
%         bdot = 0.1*Vo(2); b0 = 1;
%         
%         a = @(t) a0+adot*t;
%         da = @(t) adot;
%         b = @(t) b0+bdot*t;
%         db = @(t) bdot;
%         
%         theta = @(t) 0;
%         omega = @(t) 0;
%         ell1 = Shape.ellipseMoving(a,b,da,db,theta,omega,a0,b0);
% 
%         d_i = 10;
%         a_i = 1;
%         
%         theta = 0;
%         obs1 = ObstacleElliptical(ell1,Po,Vo,theta,d_i,a_i);
%         obs = [obs1];
%         
%         X0agent = [0;-8];
%         V0agent = [0;0];
%         Xfagent = [-6;4];
%         agent = Agent(X0agent,V0agent,Xfagent,uMax);
%         env = Environment(0,agent,obs,[-10;10;-10;10],0.01);
%         tf = 6;


%     case 'sell'
%         a=2;b=2;n=4;
%         sell = Shape.superellipse(a,b,n);
%         %Po = [5;0];
%         Po = [-1;0];
%         Vo = [0;0];
%         %Vo = [-1;0];
%         theta = -pi/4;
%         d_i = 6; a_i = 2;
%         obs_1 = ObstacleSuperElliptical(sell,Po,Vo,theta,d_i,a_i);
%         %agent = Agent([-10;0],0,1);
%         %env = Environment(0,agent,obs);
%         %tf = 18;
% 
%         a=1; b=5;
%         ell = Shape.ellipse(a,b);
%         Po = [2;0]; 
%         Vo = [0;0];
%         %theta = 0;
%         theta = pi/2;
%         %theta = pi/4;
%         d_i = 10;
%         a_i = 3;
%         %obs_1 = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i)
%         obs_2 = ObstacleElliptical(ell,Po+[0;-a*6],Vo,theta+pi/6,d_i,a_i)
%         obs_3 = ObstacleElliptical(ell,Po+[0;a*3*2],Vo,theta-pi/6,d_i,a_i)
%         obs = [obs_1,obs_2,obs_3];
%         agent = Agent([-10;0],0,1,uMax);
%         env = Environment(0,agent,obs,[],0.01);
%         tf = 25;

%     case 'expanding'
%         delta = 6; % for plotting
%         w_exp = .35;
%         mag_exp = 2;
%         a = @(t) 4+mag_exp*sin(w_exp*t);
%         da = @(t) mag_exp*w_exp*cos(w_exp*t);
%         b = @(t) 2.5+mag_exp*sin(w_exp*t);
%         db = @(t) mag_exp*w_exp*cos(w_exp*t);
%         w = 0;
%         theta = @(t) w*t;
%         omega = @(t) w;
%         ell1 = Shape.ellipseMoving(a,b,da,db,theta,omega);
%         Po = [6;b(0)+.5];
%         Vo = [-.75;0];
%         theta = 0;
%         d_i = 10;
%         a_i = 1;
%         obs1 = ObstacleElliptical(ell1,Po,Vo,theta,d_i,a_i);
%         a = @(t) 4+mag_exp-sin(w_exp*t);
%         da = @(t) -mag_exp*w_exp*cos(w_exp*t);
%         b = @(t) 2.5-mag_exp*sin(w_exp*t);
%         db = @(t) -mag_exp*w_exp*cos(w_exp*t);
%         ell2 = Shape.ellipseMoving(a,b,da,db,@(t)0,omega);
%         obs2 = ObstacleElliptical(ell2,[Po(1);-Po(2)],Vo,theta,d_i,a_i);
%         %obs2 = ObstacleElliptical(ell2,-Po,-Vo,theta,d_i,a_i);
% 
%         obs = [obs1,obs2];
%         agent = Agent([-10;0],0,1,uMax);
%         V_obs = w_exp*mag_exp+norm(Vo)
%         env = Environment(0,agent,obs,[],0.01);
%         tf = 18;

%     case 'rotating'
%         delta = 8; % for plotting
%         a = @(t) 5;
%         da = @(t) 0;
%         b = @(t) 1;
%         db = @(t) 0;
%         w = 1/max(b(0),a(0));
%         theta = @(t) w*t;
%         omega = @(t) w;
%         ell = Shape.ellipseMoving(a,b,da,db,theta,omega);
%         Po = [0;0];
%         Vo = [0;0];
%         theta = 0;
%         d_i = 10;
%         a_i = 1;
%         obs = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i);
%         agent = Agent([-10;0],0,1,uMax);
%         env = Environment(0,agent,obs,[],0.01);
%         tf = 25;
% 
%     case 'gap'
%         a=1; b=10;
%         ell = Shape.ellipse(a,b);
%         Po = [2;b+1]; 
%         Vo = [0;0];
%         theta = 0;
%         %theta = pi/2;
%         %theta = pi/4;
%         d_i = 10;
%         a_i = 3;
%         obs_1 = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i)
%         obs_2 = ObstacleElliptical(ell,[Po(1),-Po(2)]',Vo,theta,d_i,a_i)
%         obs = [obs_1,obs_2];
%         agent = Agent([-10;0],0,1,uMax);
%         env = Environment(0,agent,obs,[],0.01);
%         tf = 25;

end

env.iterate(tf);

if animate
    env.animate_hist(true,anim_name);
end

%figure(2); clf;

X = agent.P_hist;
t = env.t_hist;

if multiplot
    num = floor(t(end)/delta); 

    for n=1:num
        figure(n+1); clf;
        t_start = delta*(n-1)
        t_end = delta*n
        c_start = max(find(t<=t_start));
        c_end = max(find(t<t_end));
        env.plot(t_start);
        plot(X(1,c_start:c_end),X(2,c_start:c_end),'LineWidth',3,'Color',[0,.7,.2]);
        %for o=obs
        %    obs_next1 = o.plot(t_start+delta/2);
        %    obs_next1.LineStyle='--';
        %    %obs_next2 = o.plot(t_end);
        %    %obs_next2.LineStyle='--';
        %end
        title(['t = [' num2str(t_start) ' , ' num2str(t_end) ']']);
    end
end

if strcmp(obstacle_case,'uncertellipse') && multiplot
    for n=1:num
        figure(n+1);
        plot([Po(1),Po(1)+tf*Vo(1)],[Po(2)+b(0),Po(2)+tf*Vo(2)+b(tf)],'--','LineWidth',2,'Color',[.6,.3,.1])
        plot([Po(1),Po(1)+tf*Vo(1)],[Po(2)-b(0),Po(2)+tf*Vo(2)-b(tf)],'--','LineWidth',2,'Color',[.6,.3,.1])

        plot([Po(1)+Po2_off(1),Po(1)+Po2_off(1)+tf*Vo(1)],[Po(2)+Po2_off(2)+b2(0),Po(2)+Po2_off(2)+tf*Vo(2)+b2(tf)],'--','LineWidth',2,'Color',[.6,.3,.1])
        plot([Po(1)+Po2_off(1),Po(1)+Po2_off(1)+tf*Vo(1)],[Po(2)+Po2_off(2)-b2(0),Po(2)+Po2_off(2)+tf*Vo(2)-b2(tf)],'--','LineWidth',2,'Color',[.6,.3,.1])
    end

end
