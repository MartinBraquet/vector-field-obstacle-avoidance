% implementation of the rk4 method for a range of time steps
% state space is required to be in the form of a column vector of states
% update function is run at each step of rk4 iteration
%
% see rk4_step for further definitions for ode, upd_func
% inputs:
%   ode     - ode to integrate
%   X_0     - initial state at time t_i
%   params  - ode parameters
%   t_i     - initial time value
%   t_f     - final time value
%   h       - time step between rk4_step iterations
%   upd_func- updating function
%
% outputs:
%   X       - time history of state for each time value in t
%   t       - time vector = t_i:h:t_f
%   X_dot   - time history of state derivative

function [X,t,X_dot] = rk4(ode,X_0,params,t_i,t_f,h,upd_func)
    t = t_i:h:t_f;
    n = numel(t);
    X = zeros(size(X_0,1),n);
    X(:,1) = X_0;
    X_dot(:,1) = ode(t_i, X_0, params);
    for(i = 2:n)
        if(exist('upd_func','var'))
            X(:,i) = rk4_step(ode,X_0,params,t(i-1),h,upd_func);
        else
            X(:,i) = rk4_step(ode,X_0,params,t(i-1),h);
        end
        % store X_dot time-history
        % have to calculate X_dot from ode 
        X_dot(:,i) = ode(t(i-1), X_0, params);

        X_0 = X(:,i);
    end
end
