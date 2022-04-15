clear;
obstacle_case = 'parabola'

switch obstacle_case

    case 'parabola'
        a=2; b=1.5;
        t0 = 6;
        P_t = @(t) [(t/2-t0); -.4*(t/2-t0).^2] + [3;7];
        V_t = @(t) [1   ; -.4*2*(t/2-t0)]/2;
        A_t = @(t) [0   ; -.8]/2;

        ell = Shape.ellipse(a,b);
        obs_parametric = ObstacleEllipticalParametric(ell,P_t,V_t,A_t,0,10,1);
        agent = Agent([-10;0],0,1);
        env = Environment(0,agent,obs_parametric,[],0.01);
        tf = 25;

    case 'accelerating'
        a=4; b=1.5;
        accel = .1;
        P0 = [3;-2];

        A_t = @(t) [-accel; 0];
        V_t = @(t) [-accel*t; .5];
        P_t = @(t) [-accel./2*t^2; [0,1]*V_t(t)*t] + P0;

        ell = Shape.ellipse(a,b);
        obs_parametric = ObstacleEllipticalParametric(ell,P_t,V_t,A_t,0,10,1);
        agent = Agent([-10;0],0,1);
        env = Environment(0,agent,obs_parametric);
        tf = 17;


    case 'circle'
        w=1; R=2;
        a=1; b=1.5;
        T=0:.01:2*pi/w;

        P_t = @(t) [R*cos(w*t);         R*sin(w*t)];
        V_t = @(t) [-R*w*sin(w*t);      R*w*cos(w*t)];
        A_t = @(t) [-R*w^2*cos(w*t);    -R*w^2*sin(w*t)];

        path = P_t(T);
        ell = Shape.ellipse(a,b);
        obs_parametric = ObstacleEllipticalParametric(ell,P_t,V_t,A_t,0,10,1);
        agent = Agent([-10;2],0,1);
        env = Environment(0,agent,obs_parametric);
        tf = 25;
end

env.iterate(tf);
env.animate_hist();
