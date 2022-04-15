clear
% example use case of the new triangle obstacle shape

% triangles are defined by a base B, height H, and tilt T. Vertex 1 is at (0,0), Vertex 2 at (B,0), and Vertex 3 at (B*T,H)

B = 5; H = 5; T = 1.5;

% specific obstacles are defined by the instance of their shape, which are determined by two functions: isInside and closestPoint. The 'Shape.m' file contains all current shape definitions. 
% Each shape instance is a struct with:
%   'type' - string uniquely identifying the shape, used to ensure consistency between shapes and obstacles
%   'nonrigid' - boolean true if the shape has parameters as functions of time
%   'isInside(P)' - function handle that determines if a point P in the shape reference frame is within the shape 
%   'closestPoint(P)' - function handle that finds the point on the shape closest to the point P; returns NaN if P is inside the shape
%
% note that a particular shape is a Matlab struct, so parameter updating might not work as expected. In a better implementation, shapes would be better defined as objects

% triangle functions are defined in Shape.m-71:82 & 358:428
triangle = Shape.triangle(B,H,T);
% 

triangle.isInside([5;1]); % returns true
triangle.isInside([6;1]); % returns false
triangle.closestPoint([6;1]);

% shapes are then passed to the corresponding obstacle instance, in this case ObstacleTriangle.m
% obstacle instances inherit most functions from Obstacle.m, where the main CAVF functions are
% Particular Obstacle____.m files are necessary mostly for plotting obstacles, as the transformation mapping the inertial frame to the shape frame is handled by the obstacle position 'P(t)' and angle 'theta(t)'. Specific Obstacle files implement the pointdata(t) abstract function, which returns the points of the obstacle at time 't' for plotting.

% initial object position
Po = [-5;0];
% object velocity vector 
Vo = [0;0];
% angle of shape frame to iniertial frame
theta = 0;
% range of influence and sharpness parameter of the CAVF
d_i = 5; a_i = 3;

% construct an obstacle object- Obstacle.m-17:29, ObstacleTriangle.m
obstri = ObstacleTriangle(triangle,Po,Vo,theta,d_i,a_i);

% To test the CAVF environment, the simulation operates on an 'agent', defined in agent.m, which contains a controller to follow the combined cavf(Agent.m-32:44). The agent has position 'P', desired heading 'phi_des', and constant speed 'V_des' (Agent.m-20:29).  
P_a = [-10;1];
phi_des = 0;
V_des = 1;

ag = Agent(P_a,phi_des,V_des);

% the agent and the set of obstacles are contained within the Envrionment.m class, which handles the interface between the individual obstacle's CAVFs and the agent controller.

obstacles = [obstri]; % can consist of multiple obstacles
t0 = 0;

env = Environment(t0,ag,obstacles,[],.01); % (Environment.m-16:25)

% the environment handles simulation and plotting
% simulation ending time
tf = 20;
% run the simulation until tf, uses RK4 simulation I believe
env.iterate(tf); % (Environment.m-28:48,249:289;Obstacle.m-110:270)

% show the animated simulation. The environment is shown in the top two thirds of the figure, and the agent heading is shown at the bottom
env.animate_hist(); % (Environment.m-50:167)
% plot streamlines of the cavf at regular y intervals
env.plot_streamlines(); % (Environment.m-169:203)
