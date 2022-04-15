% object representing an elliptical obstacle

classdef ObstacleRectangular < Obstacle
    properties
        type='rectangle';
    end

    methods
        function P = pointdata(obj,t)
            if(obj.shape.nonrigid)
                params = obj.shape.params_static(t);
            else
                params = obj.shape.params;
            end
            x = params.x; y = params.y;
            pos = obj.position(t);
            coords = [-x,x,x,-x;
                      -y,-y,y,y] ;
                      coords(:,end+1) = coords(:,1);
            P = obj.rotation(t)' * coords + pos;
        end
    end
end
