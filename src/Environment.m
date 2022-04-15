% class of the environment the agent is traversing
% holds the current time and any obstacles present

classdef Environment < handle
    properties
        obstacles;
        agent;
        t;
        t_hist;
        frame = [0,1,0,1]; % static plotting frame coordinates -x,x,-y,y
        dt = .05;
        e_m = .9;
    end

    methods
        function obj = Environment(t0,agent,obstacles,frame,dt)
            if ~isempty(frame)
                obj.frame = frame;
            end
            obj.t = t0;
            obj.dt = dt;
            obj.t_hist = [t0];
            obj.agent = agent;
            if( exist('obstacles','var') && isa(obstacles,'Obstacle') )
                obj.obstacles = obstacles;
            else
                obj.obstacles = Obstacle.empty;
            end
        end
        
        % performs the rk4 step iteration and updates the agent
        function X_1 = iterate(obj,tf)
            if( ~exist('tf','var') )
                steps=1;
            elseif tf > obj.t
                steps = round( (tf-obj.t)/obj.dt );
            else 
                warning(['end time ' num2str(tf) ...
                ' is less than the current Environment time ' num2str(obj.t)] )
                return
            end
            
            dim = length(obj.agent.P);
            ode = @(t,X,~) obj.agent.controller(obj.cavf_combined(X(1:dim),t),X,obj.hdot(X,t));
            for step=1:steps
                X0 = [obj.agent.P ; obj.agent.V];
                t = obj.t; dt = obj.dt;
                %upd = @(X,t) obj.agent.update(X,t); obj.t_hist(end+1) = t; obj.t = t;
                upd = @(X,t,u) obj.update_func(X,t,u);
                [X_1,t_1] = ode_step(ode,X0,[],t,dt,upd);
                % record velocity (cavf) at position
            end
        end

        function [] = animate_hist(obj, gif,filename)
            if ~exist('gif','var')
                gif = false;
            end
            
            t = obj.t_hist(1);
            idx = 1;
            fig = figure(1);
            clf;
            subplot(2,3,1)
            hold on; grid on;
            title('Position error')
            xlabel('time (sec)');
            ylabel('||X - Xf||');
            error = vecnorm(obj.agent.P_hist - obj.agent.Xf);
            h_error = plot(obj.t_hist,error,'LineWidth',2);

            subplot(2,3,4)
            hold on; grid on;
            title('Control input')
            xlabel('time (sec)');
            ylabel('||u||');
            Normu = vecnorm(obj.agent.u_hist);
            h_u = plot(obj.t_hist(1:length(Normu)),Normu,'LineWidth',2);
            
            dim = length(obj.agent.Xf);
            
            switch dim
                case 2

                    subplot(2,3,[2 3 5 6])
                    hold on; axis equal;
                    title('Environment')
                    xlabel('x');
                    ylabel('y');
                    xlim(obj.frame(1:2));
                    ylim(obj.frame(3:4));
                    
                    Xf = obj.agent.Xf;
                    scatter(Xf(1),Xf(2),100,'*','LineWidth',2,'Cdata', [0.5 0.5 0.5]);

                    plot(obj.agent.P_hist(1,:),obj.agent.P_hist(2,:),'LineWidth',2,'Color',[.7,.3,.3]);
                    if(any(any(isnan(obj.agent.P_hist))))
                        crash_idx = min(find(isnan(obj.agent.P_hist(1,:))))-1;
                        crash = obj.agent.P_hist(:,crash_idx);
                        scatter(crash(1),crash(2),200,'x','LineWidth',3,'Cdata',[.8,.3,.3]);
                    end

                    frame = obj.frame;
                    num_arrows = 15;
                    x_space = (frame(2)-frame(1))/num_arrows;
                    y_space = (frame(4)-frame(3))/num_arrows;
                    xrange = frame(1):x_space:frame(2);
                    yrange = frame(3):y_space:frame(4);
                    [X,Y] = meshgrid(xrange,yrange);
                    Hx = zeros(size(X));
                    Hy = zeros(size(X));
                    for(ix=1:size(X,1))
                        for(iy=1:size(X,2))
                            x = X(ix,iy);
                            y = Y(ix,iy);
                            P = [x;y];
                            V = obj.cavf_combined(P,t);
                            Hx(ix,iy) = V(1);
                            Hy(ix,iy) = V(2);
                        end
                    end
                    h_cavf = quiver(X,Y,Hx,Hy,'Color',[.3,.3,.6],'LineWidth',.75);
                    h_cavf.UDataSource='Hx';
                    h_cavf.VDataSource='Hy';

                    if ( length(obj.t_hist) <=1)
                        obj.plot();
                        return
                    end
                    for o=obj.obstacles
                        o.plot(t);
                    end
                    h_agent = obj.agent.plot_hist(idx);
                    plots = [obj.obstacles.plot_handle];
                    step = ceil(.3/obj.dt);
                    for i=1:length(plots)
                        plots(i).XDataSource = ['[1 0] * ' 'obj.obstacles(' num2str(i) ').pointdata(t)' ];
                        plots(i).YDataSource = ['[0 1] * ' 'obj.obstacles(' num2str(i) ').pointdata(t)' ];
                        %plots(i).YDataSource = ['plots(' num2str(i) ').YData + ' num2str(d(2)) ];
                    end

                    
                    plotsLim = [obj.obstacles.plot_handleLim];
                    for i=1:length(plotsLim)
                        plotsLim(i).XDataSource = ['[1 0] * ' 'obj.obstacles(' num2str(i) ').pointdataLim(t)' ];
                        plotsLim(i).YDataSource = ['[0 1] * ' 'obj.obstacles(' num2str(i) ').pointdataLim(t)' ];
                    end
                    %plot([1 0] * obj.obstacles(1).pointdata(t), [0 1] * obj.obstacles(1).pointdata(t), 'LineWidth',2, 'Color', [.5 .5 .5]);

                    if gif
                        frame = getframe(fig);
                        im = frame2im(frame);
                        [imind,cm] = rgb2ind(im,256);
                        imwrite(imind,cm,filename,'gif','Loopcount',inf);
                    end

                    for t=obj.t_hist(step:step:end)
                        % cavf update
                        for(ix=1:size(X,1))
                            for(iy=1:size(X,2))
                                x = X(ix,iy);
                                y = Y(ix,iy);
                                P = [x;y];
                                V = obj.cavf_combined(P,t);
                                Hx(ix,iy) = V(1);
                                Hy(ix,iy) = V(2);
                            end
                        end
                        %h_cavf.UData = Hx; h_cavf.VData = Hy;
                        idx = idx+step;
                        % agent
                        h_agent.XData = obj.agent.P_hist(1,idx);
                        h_agent.YData = obj.agent.P_hist(2,idx);
                        %plot([1 0] * obj.obstacles(1).pointdata(t), [0 1] * obj.obstacles(1).pointdata(t), '--', 'Color', 'k');
                        
                        for i=1:length(plotsLim)
                            obj.obstacles(i).dLim = computeDLim(obj.obstacles(i),obj.agent.P_hist(:,idx),obj.agent.V_hist(:,idx),obj.agent.uMax);
                        end
                        
                        % draw
                        refreshdata(gcf,'caller');
                        drawnow;

                        if gif
                            frame = getframe(fig);
                            im = frame2im(frame);
                            [imind,cm] = rgb2ind(im,256);
                            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',obj.t_hist(step));
                        end
                    end
                    
                case 3
                    
                    fig = figure(2); hold on; axis equal; view(30,30)
                    title('Environment');
                    xlabel('x');
                    ylabel('y');
                    zlabel('z');
                    xlim(obj.frame(1:2));
                    ylim(obj.frame(3:4));      
                    zlim(obj.frame(5:6));
                    
                    
                    Xf = obj.agent.Xf;
                    scatter3(Xf(1),Xf(2),Xf(3),100,'*','LineWidth',3,'Cdata', [0.5 0.5 0.5]);
                    
                    plot3(obj.agent.P_hist(1,:),obj.agent.P_hist(2,:),obj.agent.P_hist(3,:),'LineWidth',2,'Color',[.7,.3,.3]);
                    if(any(any(isnan(obj.agent.P_hist))))
                        crash_idx = min(find(isnan(obj.agent.P_hist(1,:))))-1;
                        crash = obj.agent.P_hist(:,crash_idx);
                        scatter3(crash(1),crash(2),crash(3),200,'x','LineWidth',3,'Cdata',[.8,.3,.3]);
                    end

                    frame = obj.frame;
                    num_arrows = 10;
                    x_space = (frame(2)-frame(1))/num_arrows;
                    y_space = (frame(4)-frame(3))/num_arrows;
                    z_space = (frame(6)-frame(5))/num_arrows;
                    xrange = frame(1):x_space:frame(2);
                    yrange = frame(3):y_space:frame(4);
                    zrange = frame(5):z_space:frame(6);
                    [X,Y,Z] = meshgrid(xrange,yrange,zrange);
                    Hx = zeros(size(X));
                    Hy = zeros(size(X));
                    Hz = zeros(size(X));
                    for(ix=1:size(X,1))
                        for(iy=1:size(X,2))
                            for(iz=1:size(X,3))
                                x = X(ix,iy,iz);
                                y = Y(ix,iy,iz);
                                z = Z(ix,iy,iz);
                                P = [x;y;z];
                                V = obj.cavf_combined(P,t);
                                Hx(ix,iy,iz) = V(1);
                                Hy(ix,iy,iz) = V(2);
                                Hz(ix,iy,iz) = V(3);
                            end
                        end
                    end
                    h_cavf = quiver3(X,Y,Z,Hx,Hy,Hz,'Color',[.3,.3,.6],'LineWidth',.75);
                    h_cavf.UDataSource='Hx';
                    h_cavf.VDataSource='Hy';
                    h_cavf.WDataSource='Hz';

                    if ( length(obj.t_hist) <=1)
                        obj.plot();
                        return
                    end
                    for o=obj.obstacles
                        o.plot(t);
                    end
                    h_agent = obj.agent.plot_hist(idx);
                    plots = [obj.obstacles.plot_handle];
                    step = ceil(.3/obj.dt);


                    if gif
                        frame = getframe(fig);
                        im = frame2im(frame);
                        [imind,cm] = rgb2ind(im,256);
                        imwrite(imind,cm,filename,'gif','Loopcount',inf);
                    end

                    for t=obj.t_hist(step:step:end)
                        % cavf update
                        for(ix=1:size(X,1))
                            for(iy=1:size(X,2))
                                for(iz=1:size(X,3))
                                    x = X(ix,iy,iz);
                                    y = Y(ix,iy,iz);
                                    z = Z(ix,iy,iz);
                                    P = [x;y;z];
                                    V = obj.cavf_combined(P,t);
                                    Hx(ix,iy,iz) = V(1);
                                    Hy(ix,iy,iz) = V(2);
                                    Hz(ix,iy,iz) = V(3);
                                end
                            end
                        end
                        %h_cavf.UData = Hx; h_cavf.VData = Hy;
                        idx = idx+step;
                        % agent
                        h_agent.XData = obj.agent.P_hist(1,idx);
                        h_agent.YData = obj.agent.P_hist(2,idx);
                        h_agent.ZData = obj.agent.P_hist(3,idx);
                        
                        for i=1:length(plots)
                            [plots(i).XData, plots(i).YData, plots(i).ZData] = obj.obstacles(i).pointdata(t);
                        end
                        
                        % draw
                        refreshdata(gcf,'caller');
                        drawnow;

                        if gif
                            frame = getframe(fig);
                            im = frame2im(frame);
                            [imind,cm] = rgb2ind(im,256);
                            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',obj.t_hist(step));
                        end
                    end
            end
            
        end

        function [] = plot_streamlines(obj)
            for o = obj.obstacles
                o.plot(0);
            end

            xlim(obj.frame(1:2));
            ylim(obj.frame(3:4));

            dt = 5*obj.dt;
            ode = @(t,X,~) obj.cavf_combined(X,t);
            %x0 = obj.frame(1);
            x0 = obj.agent.P_hist(1,1);
            ystarts = obj.frame(3):2:obj.frame(4);
            for y0=ystarts
                P0=[x0;y0];
                X = P0;
                Pend = P0;
                t0 = obj.t_hist(1);
                %X(:,1)=P0;
                idx = 1;
                while ~any(isnan(Pend)) && Pend(1) < obj.frame(2)
                    Pend = rk4_step(ode,P0,[],t0,dt);
                    X(:,idx+1) = Pend;
                    t0 = t0+dt;
                    P0 = Pend;
                    idx = idx+1;
                end
                if(any(isnan(Pend)))
                    scatter(X(1,idx-1),X(2,idx-1),...
                        200,'x','LineWidth',3,'Cdata',[.7,.2,.2],'MarkerFaceAlpha',.5)
                end
                hold on;
                plot(X(1,:),X(2,:),'LineWidth',2,'Color',[.2,.5,.2,.5],'LineStyle','-');
            end
        end

        function [X,t] = update_func(obj,X,t,u)
            obj.agent.update(X,t,u);
            obj.t_hist(end+1) = t;
            obj.t = t;
        end

        function [] = reset(obj)
            obj.t = obj.t_hist(1);
            obj.t_hist = obj.t;
            obj.agent.reset();
        end

        function [] = plot(obj,t)
            if(~exist('t','var'))
                t = obj.t;
            end
            hold on;
            title('Environment CAVF')
            xlabel('x'); ylabel('y');
            frame = obj.frame;
            xrange = frame(1):.5:frame(2);
            yrange = frame(3):.5:frame(4);
            [X,Y] = meshgrid(xrange,yrange);
            Hx = zeros(size(X));
            Hy = Hx;
            for(x=xrange)
                for(y=yrange)
                    % if(x==0 && y== -4) keyboard; end
                    P = [x;y];
                
                    V = obj.cavf_combined(P,t);
                    Hx(xrange==x, yrange==y) = V(1);
                    Hy(xrange==x, yrange==y) = V(2);
                end
            end
            quiver(X,Y,Hx',Hy');
            for o=obj.obstacles
                %o.plot_cavf(obj.t,obj.agent.phi_des,obj.agent.V_des, frame(1),frame(2),frame(3),frame(4));
                o.plot(t);
            end
            axis equal;
            xlim(obj.frame(1:2));
            ylim(obj.frame(3:4));
        end

        function vel = cavf_combined(obj,P,t)
            if( ~exist('P','var') )
                P = obj.agent.P;
            end
            if( ~exist('t','var') )
                t = obj.t;
            end
            obs = obj.obstacles;
            agent = obj.agent;
            
            d_P_to_Xf = agent.Xf - P;
            %d_P_to_Xf = zeros(3,1);
            
            dim = length(P);
            H = zeros(dim,length(obs));
            D = zeros(1,length(obs));
            %infl = boolean(zeros(1,length(obs)));
            infl = logical(zeros(1,length(obs)));
            for i=1:length(obs)
                D(i) = obs(i).dist_to(t,P);
                infl(i) = D(i) < obs(i).d_i;
                H(:,i) = obs(i).cavf(t,P,agent.V,d_P_to_Xf,agent.Xf,agent.uMax);
                %H(:,i) = obs(i).cavf_local_DoubleIntegrator(t, P, agent.Xf);
            end
            if(any(isnan(H(1,:))))
                vel = nan * ones(dim,1);
                return
            end
            if(~any(infl))
                vel = TargetCAVF(agent.Xf, P);
                return
            end
            if(sum(infl)==1)
                vel = H(:,infl);
                return
            end
            dists = D(infl);
            
            cavfs = H(:,infl);
%             W = 1 - dists/sum(dists);
%             W = W / sum(W);
            
            distsprod = prod(dists);
            W = distsprod ./ dists;
            W = W / sum(W);
            
%            [w_max,k] = max(W);
%             if(w_max > obj.e_m)
%                 vel = cavfs(:,k);
%                 return
%             end
            vel = cavfs*W';
            %vel = normalize(sum(W.*cavfs,2),'norm');
            return
        end

        function hd = hdot(obj,X,t)
            dim = length(X)/2;
            P = X(1:dim);
            V = X(dim+1:end);
            dx = obj.obstacles(1).d_i/100;
            
            switch dim
                case 2
                    dX = [dx;0]; dY = [0;dx];
                    dhdx = (obj.cavf_combined(P+dX,t) - obj.cavf_combined(P-dX,t)) / (2*dx);
                    dhdy = (obj.cavf_combined(P+dY,t) - obj.cavf_combined(P-dY,t)) / (2*dx);
                    gradh = [dhdx, dhdy];
                case 3
                    dX = [dx;0;0]; dY = [0;dx;0]; dZ = [0;0;dx];
                    dhdx = (obj.cavf_combined(P+dX,t) - obj.cavf_combined(P-dX,t)) / (2*dx);
                    dhdy = (obj.cavf_combined(P+dY,t) - obj.cavf_combined(P-dY,t)) / (2*dx);
                    dhdz = (obj.cavf_combined(P+dZ,t) - obj.cavf_combined(P-dZ,t)) / (2*dx);
                    gradh = [dhdx, dhdy, dhdz];
            end
            
            hd = gradh * V;
        end
        
        
    end
end
