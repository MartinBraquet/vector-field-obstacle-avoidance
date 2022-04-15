% object representing an elliptical obstacle

classdef ObstacleEllipticalParametric < ObstacleParametric
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
    end
end
