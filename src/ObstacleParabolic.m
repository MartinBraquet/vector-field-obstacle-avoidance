% object representing an elliptical obstacle

classdef ObstacleParabolic < Obstacle
    properties
        type='parabola';
    end
    methods
        
        function h = plot(obj,t)
            P = obj.pointdata(t);
            p = obj.position(t);
            hold on;
            h = plot(P(1,:),P(2,:),'LineWidth',2);
            quiver(p(1),p(2),obj.Vo(1),obj.Vo(2),2 ,'LineWidth',2,'MaxHeadSize',.7);
            obj.plot_handle = h;
        end
        function P = pointdata(obj,t)
            h = obj.shape.params.h;
            a = obj.shape.params.a;
            if isinf(h)
                x_lim = sqrt(20/a);
                X=-x_lim:x_lim/20:x_lim;
                P_rel = [X;a.*X.^2];
            else
                x_lim = sqrt(h/a);
                X=-x_lim:x_lim/20:x_lim;
                P_rel = [X;a.*X.^2];
                P_rel(:,end+1) = P_rel(:,1);
            end
            p = obj.position(t);
            P= obj.rotation()' * P_rel + p;
        end

    end
end
