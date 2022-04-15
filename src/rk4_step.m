% implementation of the rk4 method for a single time step
% returns estimation of state from given state based on input ode function current time, and time step
% update function is run with predefined params upd_func(X_1, t_1)
% inputs: 
%   ode     - function handle to ode describing system dynamics
%             must be of the form ode(t,X,params)
%   X_0     - state at initial time t
%   params  - additional input parameters to ode
%   t       - time of state X_0
%   h       - time step between t and t_1
%   upd_func- optional function handle to some additional updating function run on updated state and time
%             must be of the form upd_func(X,t)
%
% outputs:
%   X_1     - state at time step t_1 = t+h
%   t_1     - time after t by time step h

function [X_1,t_1] = rk4_step(ode,X_0,params,t,h,upd_func)
    k1 = h * ode(t     ,X_0,params);
    k2 = h * ode(t+h/2 ,X_0+k1/2,params);
    k3 = h * ode(t+h/2 ,X_0+k2/2,params);
    k4 = h * ode(t+h   ,X_0+k3,params);
    X_1 = X_0 + (k1 + 2*k2 + 2*k3 + k4)/6;
    t_1 = t + h;
    if(exist('upd_func','var') && isa(upd_func,'function_handle'))
        dim = length(X_0)/2;
        u = (k1 + 2*k2 + 2*k3 + k4)/(6*h); u = u(dim+1:end);
        upd_func(X_1,t_1,u);
    end
end
