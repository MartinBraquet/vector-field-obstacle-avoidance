% object representing an elliptical obstacle

classdef ObstacleElliptical < Obstacle
    properties
        type='ellipse';
    end
    methods
        
        function P = pointdata(obj,t)
            T = 0:.1:2*pi+.1;
            P_rel = obj.shape.parametric(T,t);
            p = obj.position(t);
            P= obj.rotation(t)' * P_rel + p;
        end
        
        function P = pointdataLim(obj,t)
            T = 0:.1:2*pi+.1;
            if (isa(obj.shape.params.a,'function_handle'))
                a = obj.shape.params.a(t) + obj.dLim;
                b = obj.shape.params.b(t) + obj.dLim;
            else
                a = obj.shape.params.a + obj.dLim;
                b = obj.shape.params.b + obj.dLim;
            end
            P_rel = [a*cos(T); b*sin(T)];
            p = obj.position(t);
            P= obj.rotation(t)' * P_rel + p;
        end
        
        function P = pointdataDeterminist(obj,t)
            T = 0:.1:2*pi+.1;
            P_rel = obj.shape.parametricDeterminist(T,t);
            p = obj.position(t);
            P= obj.rotation(t)' * P_rel + p;
        end
    end
end
