% object representing a triangular obstacle

classdef ObstacleTriangle < Obstacle
    properties
        type='triangle';
    end
    methods
        
        function P = pointdata(obj,t)
            params = obj.shape.params;
            B = params.B; H = params.H; T = params.T;
            pos = obj.position(t);
            coords = [0, B, B*T;
                      0, 0, H];
                      coords(:,end+1) = coords(:,1);
            P = obj.rotation(t)' * coords + pos;
        end

    end
end
