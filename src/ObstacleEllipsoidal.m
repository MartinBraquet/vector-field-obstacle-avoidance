% object representing an elliptical obstacle

classdef ObstacleEllipsoidal < Obstacle
    properties
        type='ellipsoid';
    end
    methods
        
        function [X,Y,Z] = pointdata(obj,t)
            a = obj.shape.params.a;
            b = obj.shape.params.b;
            c = obj.shape.params.c;
            if (isa(a,'function_handle'))
                a = a(t);
                b = b(t);
                c = c(t);
            end
            [X,Y,Z] = ellipsoid(0,0,0,a,b,c);
            s = size(X);
            P_rel = [reshape(X,1,[]); reshape(Y,1,[]); reshape(Z,1,[])];
            p = obj.position(t);
            P= obj.rotation(t)' * P_rel + p;
            X = reshape(P(1,:),s);
            Y = reshape(P(2,:),s);
            Z = reshape(P(3,:),s);
        end
        
        function P = pointdataDeterminist(obj,t)
            T = 0:.1:2*pi+.1;
            P_rel = obj.shape.parametricDeterminist(T,t);
            p = obj.position(t);
            P= obj.rotation(t)' * P_rel + p;
        end
    end
end
