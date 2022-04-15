% tester script
clear

example = 'multi';
%example='rectangle';

valid_cases = {'circle','multi','eccentric','rectangle','moving','parabola','parabola inf'};

case_valid = any(strcmp(valid_cases,example));
if(case_valid)
    disp(['showing example ''' example '''']);
end

switch example
    case 'circle'
        circle = Shape.ellipse(2,2);
        Po = [0;0];
        Vo = [0;0];
        theta = 0;
        d_i = 7;
        a_i = 1;
        obstacles = ObstacleElliptical(circle,Po,Vo,theta,d_i,a_i);

        P0 = [-10;0]; phi_des = 0; V_des = 1.;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 20;
    case 'multi'
        rect = Shape.rectangle(1,2);
        ell = Shape.ellipse(3,2);
        Po = [0;0];
        Po = [ Po+[0;1],Po+[4;-6.5],Po+[-4;4] ];
        Vo = [0;0];
        Vo = [ Vo,Vo+[.2;.2],Vo+[.0;-.7] ];
        theta = [pi/6,pi/4,-pi/3];
        d_i = [7,6,4];
        a_i = [3,.5,1];

        obstacles = Obstacle.empty;
        obstacles(1) = ObstacleRectangular(rect,Po(:,1),Vo(:,1),theta(1),d_i(1),a_i(1));
        obstacles(2) = ObstacleElliptical(ell,Po(:,2),Vo(:,2),theta(2),d_i(2),a_i(2));
        obstacles(3) = ObstacleRectangular(rect,Po(:,3),Vo(:,3),theta(3),d_i(3),a_i(3));

        P0 = [-10;0]; phi_des = 0; V_des = 1.;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);

        tf = 25;
        
    case 'eccentric'
        ell = Shape.ellipse(5,1.5);
        Po = [2;-2];
        Vo = [-.5;0];
        theta = -pi/6;
        d_i = 3;
        a_i = 1;
        obstacles = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i);

        P0 = [-10;0]; phi_des = 0; V_des = 1.;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 25;
        
    case 'rectangle'
        rect = Shape.rectangle(5,1.5);
        Po = [2;-2];
        Vo = [0;0];
        theta = pi/6;
        d_i = 3;
        a_i = 1;
        obstacles = ObstacleRectangular(rect,Po,Vo,theta,d_i,a_i);

        P0 = [-10;-2.5]; phi_des = 0; V_des = 1.;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 25;
    
    case 'moving'
        ell = Shape.ellipse(1,5);
        Po = [7;0];
        Vo = [-1.5;0];
        theta = pi/6;
        d_i = 5;
        a_i = 1;
        obstacles = ObstacleElliptical(ell,Po,Vo,theta,d_i,a_i);

        P0 = [-10;0]; phi_des = 0; V_des = 1;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 15;
    case 'parabola'
        parab = Shape.parabola(.5,5);
        Po = [4;-1];
        Vo = [-.5;.3];
        theta =pi- pi/4;
        d_i = 7;
        a_i = 2;
        obstacles = ObstacleParabolic(parab,Po,Vo,theta,d_i,a_i);

        P0 = [-10;0]; phi_des = 0; V_des = 1;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 20;
    case 'parabola inf'
        parab = Shape.parabola(.25,Inf);
        Po = [0;0];
        Vo = [-0;0];
        theta = -pi/2;
        theta = -pi;
        d_i = 7;
        a_i = 1;
        obstacles = ObstacleParabolic(parab,Po,Vo,theta,d_i,a_i);

        P0 = [-10;0]; phi_des = 0; V_des = 1;
        agent = Agent(P0,phi_des,V_des);

        env = Environment(0,agent,obstacles);
        tf = 20;
    otherwise
        warning(['case ''' example ''' is invalid.'])
        case_valid=false

end

if case_valid
    f1 = figure(1); clf;
    set(f1,'Units','Normalized','Position',[.01 .05 .3 .8])
    env.iterate(tf);
    env.animate_hist();
    subplot(3,1,[1,2]);
    env.plot_streamlines();
end
