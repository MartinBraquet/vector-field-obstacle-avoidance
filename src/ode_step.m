function [X_1,t_1] = ode_step(ode,X_0,params,t,h,upd_func)
    xdot = ode(t     ,X_0,params);
    X_1 = X_0 + xdot * h;
    t_1 = t + h;
    if(exist('upd_func','var') && isa(upd_func,'function_handle'))
        dim = length(X_0)/2;
        u = xdot(dim+1:end);
        upd_func(X_1,t_1,u);
    end
end