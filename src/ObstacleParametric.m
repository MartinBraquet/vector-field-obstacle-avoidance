% abstract class representing an obstacle
classdef (Abstract) ObstacleParametric < Obstacle
    properties
        % position,velocity,acceleration as functions of t
        P_t;
        Vo_t;
        A_t;
    end

    % shared constructor
    methods 
        function obj = ObstacleParametric(shape,P_t,V_t,A_t,theta,d_i,a_i)
            obj@Obstacle(shape,[NaN;NaN],[NaN;NaN],theta,d_i,a_i);
            if(~strcmp(shape.type,obj.type))
                name = matlab.mixin.CustomDisplay.getClassNameForHeader(obj)
                error([name ' requires a shape of type ''' obj.type ''' but was given shape of type ''' shape.type ''''])
            end
            obj.P_t = P_t;
            obj.Vo_t = V_t;
            obj.A_t = A_t;
            %obj.theta = theta;
            %obj.d_i = d_i;
            %obj.a_i = a_i;
            %obj.shape = shape;
        end
    end

    % private helper functions
    %methods (Access=private,Static)
    methods (Static)
        function v = dir_vec(phi) % calculate unit vector that makes an angle phi to the horizontal
            v = [cos(phi); sin(phi)];
        end
    end

    % cavf functions
    methods
        %% helper function 
        % gets position at time t
        function P = position(obj,t)
            P = obj.P_t(t);
            %P = [obj.Po] + [obj.Vo].*t;
        end

        function R = rotation(obj,t)
            if(obj.shape.nonrigid)
                %params = obj.shape.params(t);
                th = obj.theta + obj.shape.params.theta(t);
            else
                th = obj.theta;
            end
            R = [cos(th) ,sin(th);
            -sin(th),cos(th);];
        end

        function h = plot(obj,t)
            P = obj.pointdata(t);
            p = obj.position(t);
            hold on;
            h(1) = plot(P(1,:),P(2,:),'LineWidth',2);
            quiver(p(1),p(2),obj.Vo(1),obj.Vo(2),2 ,'LineWidth',2,'MaxHeadSize',.7);
            obj.plot_handle = h;
        end

        % plot the cavf and obstacle, over a range of values
        function h = plot_cavf(obj,t,phi_des,V_des,x_min,x_max,y_min,y_max)
            hold on;
            axis equal;
            obj.plot(t);
            xrange = x_min:.5:x_max;
            yrange = y_min:.5:y_max;
            [X,Y] = meshgrid(xrange,yrange);
            Hx = zeros(size(X));
            Hy = Hx;
            for(x=xrange)
                for(y=yrange)
                    % if(x==0 && y== -4) keyboard; end
                    P = [x;y];
                    V = obj.cavf(t,P, phi_des,V_des );
                    Hx(xrange==x, yrange==y) = V(1);
                    Hy(xrange==x, yrange==y) = V(2);
                end
            end
            quiver(X,Y,Hx',Hy');
        end

        % check if a point is within the influence
        function d = dist_to(obj,t,P_inertia)
            Po = obj.position(t);
            %th = obj.theta;
            R = obj.rotation(t);
            P = R * (P_inertia-Po);
            I = obj.shape.closestPoint(P,t);
            d = norm(P-I);
        end

        function infl = influences(obj,t,P_inertia)
            infl = ( obj.dist_to(t,P_inertia) < obj.d_i ); % outside influence
        end
        % calculate the cavf based on the positions at time t, and orientation of the obstacle 
        function h = cavf(obj,t, P_inertia,phi_des_inertia,V_des)
            Po = obj.position(t);
            if(obj.shape.nonrigid)
                th = obj.theta + obj.shape.params.theta(t);
            else
                th = obj.theta;
            end
            R = obj.rotation(t);
            P = R * (P_inertia-Po);
            phi_des = phi_des_inertia-th;
            V_vect_rel = V_des*[cos(phi_des_inertia);sin(phi_des_inertia)] - obj.Vo_t(t);
            %phi_des = atan2(V_vect_rel(2),V_vect_rel(1)) - th;
            %keyboard
            %if(norm(obj.Vo))
                %h = obj.cavf_local_moving(t,P,phi_des,V_des);
            %else
                h = obj.cavf_local(t,P,phi_des,V_des);
            %end
            h = R' * h;
        end

        function h = cavf_local_moving(obj,t,P,phi_des,V_des)
            shape = obj.shape;
            dist_i = obj.d_i; a_i = obj.a_i;
            V_o = obj.rotation(t)*obj.Vo_t(t);

            h_o = obj.cavf_local(t,P,phi_des,V_des);

            a = norm(h_o)^2;
            b = 2*( dot(V_o,h_o) );
            c = norm(V_o)^2-V_des^2;

            det = b^2-4*a*c;
            if det <0
                % no valid solutions
                h = [nan;nan];
            elseif det ==0
                % one valid solution
                s = -b/(2*a);
                h = s*h_o+V_o;
            else
                % two valid solutions, have to pick the better
                s1 = (-b + sqrt(det)) / (2*a);
                s2 = (-b - sqrt(det)) / (2*a);
                h1 = s1*h_o+V_o;
                h2 = s2*h_o+V_o;
                if( dot( Obstacle.dir_vec(phi_des),h1 ) > dot( Obstacle.dir_vec(phi_des),h2) )
                    h = h1;
                else
                    h = h2;
                end
            end
        end

        % calculate the cavf_local of the obstacle, assuming no movement and positioned at the origin
        %% cavf_local algo
        % generalized cavf_local algorithm
        % inputs:
        %   point P = [x,y]' : to calculate the cavf_local heading at
        %   phi_des : desired heading
        function h = cavf_local(obj,t,P,phi_des,speed)
            shape = obj.shape;
            dist_i = obj.d_i; a_i = obj.a_i;

            V_des = speed*Obstacle.dir_vec(phi_des);
            isInside = shape.isInside;
            closestPoint = shape.closestPoint;


            if(isInside(P,t)) % if inside shape, return nan
                h = [nan;nan];
                return
            else
                I = closestPoint(P,t);
                if( norm(P-I) > dist_i ) % outside influence
                    h = V_des;
                    return
                else
                    d1 = norm(P-I); d2 = d1-dist_i;
                    x = (d1+d2) / (-d1*d2);
                    if(d2==0)
                        %g = 0;
                        g = 1;
                    else
                        g = a_i*x/sqrt(1+(2*a_i*x)^2) + .5;
                    end
                    % modulate steer angle from tangent to desired direction
                    % take normal as the vector from I to P, pointed out of shape
                    n_hat = normalize( P-I , 'norm' );
                    % calculate surface tangent that is in the direction of V_des
                    t_hat = cross([n_hat;0],[0;0;1]);
                    t_hat = t_hat(1:2);

                    if( abs(t_hat'*normalize(V_des,'norm')) < 0.01)
                        if [0,0,1]*cross([P;0],[V_des;0]) > 0
                            % below obstacle
                            t_hat = -t_hat;
                        end
                    else
                        if(dot(t_hat,normalize(V_des,'norm')) <0)
                            t_hat = -t_hat;
                        end
                    end

                    % required angle at the surface is no longer in t_hat direction
                    V_o = obj.rotation(t)*obj.Vo_t(t);
                    if(shape.nonrigid)
                        V_exp = shape.V_exp(I,t);
                        V_rot = cross([0;0;shape.params.omega(t)],[I;0]);
                        %keyboard
                        V_rot = V_rot(1:2);
                        V_surf = V_exp + V_rot + V_o;
                    else
                        V_surf = [0;0] + V_o;
                    end

                    v_n = V_surf' * n_hat;
                    if(speed < v_n)
                        v_t = 0; % potentially no escape
                    elseif(speed < abs(v_n))
                        % negative v_n
                        v_t = 0; v_n = -speed;
                    else
                        v_t = sqrt( speed^2 - v_n^2);
                    end

                    % velocity required at tangency
                    V_t = v_n*n_hat + v_t*t_hat;
                    if(~isreal(V_t(1)) || ~isreal(V_t(2)) )
                        keyboard
                    end

                    phi_t = atan2(V_t(2),V_t(1));

                    tstar = dot(n_hat,V_des);
                    if(tstar > 0) % passed obstacle
                        l = -(1-tstar)*(1-g) +1;
                    else
                        l = g;
                    end
                    % restrict angles to [0,2*pi) for angle interpolation to work properly
                    %keyboard
                    while(abs(phi_des-phi_t) > pi)
                        if(phi_des - phi_t > pi)
                            phi_t = phi_t + 2*pi;
                            %keyboard
                        elseif(phi_t -phi_des > pi)
                            phi_des = phi_des + 2*pi;
                            %keyboard
                        end
                    end
                    %phi_des = mod(phi_des,2*pi);
                    %phi_t = mod(phi_t,2*pi);

                    phi_h = (phi_des-phi_t)*l + phi_t;
                    h = speed*Obstacle.dir_vec(phi_h);
                    %keyboard
                    %if P(1)==-4; keyboard;end
                end
            end
        end

    end
end
