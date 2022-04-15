function [t, U, reachedTarget, X] = ComputeControlToCompletionObstacleAvoidance(X0agent, V0agent, Xfagent, Vfagent, tf, dt, SimuParamsCell)
    
    obs = SimuParamsCell.obstacles;
    uMax = SimuParamsCell.uMax;
    xmax = SimuParamsCell.map_width;
    agent = Agent(X0agent,V0agent,Xfagent,uMax);
    env = Environment(0,agent,obs,[],dt);
    
    env.iterate(tf);
    
    animate = 0;
    if animate
        anim_name = 'xxx.gif';
        env.animate_hist(true,anim_name);
    end
    
    X = [agent.P_hist; agent.V_hist];
    U = [agent.u_hist, zeros(2,1)];
    XfNum = agent.P_hist(:,end);
    t = env.t_hist;
    
    if norm(Xfagent - XfNum) / xmax > 5e-2
        %print("Agent did not reach final location");
        reachedTarget = 0;
    else
        reachedTarget = 1;
    end
end