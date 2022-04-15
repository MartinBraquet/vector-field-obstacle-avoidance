% abstract class representing an obstacle
classdef (Abstract) Obstacle < handle & matlab.mixin.Heterogeneous & matlab.mixin.CustomDisplay
    properties
        Po; % initial position
        Vo; % obstacle velocity
        d_i; a_i; % cavf_local properties
        theta; % angle of the shape relative to the x axis; +ccw
        beta; % angle of the shape relative to the y axis; +ccw
        shape; % inheriting class defines particular shape
        plot_handle; 
        plot_handleLim;
        dim;
        dLim;
    end
    properties (Abstract)
        type; % shape type of the obstacle
    end

    % shared constructor
    methods
        function obj = Obstacle(shape,Po,Vo,theta,d_i,a_i)
            if(~strcmp(shape.type,obj.type))
                name = matlab.mixin.CustomDisplay.getClassNameForHeader(obj)
                error([name ' requires a shape of type ''' obj.type ''' but was given shape of type ''' shape.type ''''])
            end
            obj.Po = Po;
            obj.Vo = Vo;
            if length(theta) > 1
                obj.theta = theta(1);
                obj.beta = theta(2);
            else
                obj.theta = theta;
            end
            obj.d_i = d_i;
            obj.a_i = a_i;
            obj.shape = shape;
            obj.dim = length(Po);
            obj.dLim = 0;
        end
    end

    % abstract functions that need to be implemented
    methods (Abstract)
        % draw position at time t
        pointdata(obj,t)
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
            P = [obj.Po] + [obj.Vo].*t;
        end

        function R = rotation(obj,t)
            
            dim = length(obj.Po);
            switch dim
                case 2
                    if(obj.shape.nonrigid)
                        %params = obj.shape.params(t);
                        th = obj.theta + obj.shape.params.theta(t);
                    else
                        th = obj.theta;
                    end
                    R = [cos(th) , sin(th);
                         -sin(th), cos(th);];
                case 3
                    if(obj.shape.nonrigid)
                        %params = obj.shape.params(t);
                        theta = obj.theta + obj.shape.params.theta(t);
                        beta = obj.beta + obj.shape.params.beta(t);
                    else
                        theta = obj.theta;
                        beta = obj.beta;
                    end
                    Rtheta = [ cos(theta) sin(theta) 0;
                              -sin(theta) cos(theta) 0;
                              0               0      1];
                    Rbeta = [1 0             0;
                             0 cos(beta) sin(beta);
                             0 -sin(beta) cos(beta)];
                    R = Rbeta * Rtheta;
            end
        end

        function h = plot(obj,t)
            dim = obj.dim;
            %p = obj.position(t);
            hold on;
            switch dim
                case 2
                    P = obj.pointdata(t);
                    h(1) = plot(P(1,:),P(2,:),'LineWidth',2, 'Color', 'k');
                    PLim = obj.pointdataLim(t);
                    hlim(1) = plot(PLim(1,:),PLim(2,:),'k--','LineWidth',1);
                    obj.plot_handleLim = hlim;
                case 3
                    [X,Y,Z] = obj.pointdata(t);
                    h(1) = surf(X,Y,Z);
                otherwise
                    disp('Error: plot_hist(obj,idx)');
            end
            
%             if isa(obj,'ObstacleElliptical')
%                 Pdet = obj.pointdataDeterminist(t);
%                 h(2) = plot(Pdet(1,:),Pdet(2,:),'LineWidth',2);
%             end
            
            %quiver(p(1),p(2),obj.Vo(1),obj.Vo(2),2 ,'LineWidth',2,'MaxHeadSize',.7);
            obj.plot_handle = h;
        end

        % plot the cavf and obstacle, over a range of values
        function h = plot_cavf(obj,t,Xf,V_des,x_min,x_max,y_min,y_max)
            hold on;
            axis equal;
            obj.plot(t);
            nPoints = 20;
            xrange = linspace(x_min,x_max,nPoints);
            yrange = linspace(y_min,y_max,nPoints);
            [X,Y] = meshgrid(xrange,yrange);
            Hx = zeros(size(X));
            Hy = Hx;
            %keyboard
            for(x=xrange)
                for(y=yrange)
                    % if(x==0 && y== -4) keyboard; end
                    P = [x;y];
                    d_P_to_Xf = agent.Xf - P;
                    phi_des = atan2(d_P_to_Xf(2),d_P_to_Xf(1));
                    V = obj.cavf(t,P, phi_des,V_des );
                    Hx(xrange==x, yrange==y) = V(1);
                    Hy(xrange==x, yrange==y) = V(2);
                end
            end
            quiver(X,Y,Hx,Hy);
        end
        
        % plot the cavf and obstacle, over a range of values
        function h = plot_cavf3D(obj,t,Xf,V_des,x_min,x_max,y_min,y_max,z_min,z_max)
            hold on;
            axis equal;
            obj.plot(t);
            nPoints = 20;
            xrange = linspace(x_min,x_max,nPoints);
            yrange = linspace(y_min,y_max,nPoints);
            zrange = linspace(z_min,z_max,nPoints);
            [X,Y] = meshgrid(xrange,yrange,zrange);
            Hx = zeros(size(X));
            Hy = zeros(size(X));
            Hz = zeros(size(X));
            for x = xrange
                for y = yrange
                    for z = zrange
                        P = [x;y;z];
                        d_P_to_Xf = agent.Xf - P;
                        %phi_des = atan2(d_P_to_Xf(2),d_P_to_Xf(1));
                        % TODO
                        V = obj.cavf(t,P, phi_des,V_des );
                        Hx(xrange==x, yrange==y, zrange==z) = V(1);
                        Hy(xrange==x, yrange==y, zrange==z) = V(2);
                        Hz(xrange==x, yrange==y, zrange==z) = V(3);
                    end
                end
            end
            quiver(X,Y,Hx,Hy);
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
        function h = cavf(obj,t,P_inertia,V_inertia,r_inertia,Xf,uMax)
            Po = obj.position(t);
            if(obj.shape.nonrigid)
                th = obj.theta + obj.shape.params.theta(t);
            else
                th = obj.theta;
            end
            R = obj.rotation(t);
            P = R * (P_inertia-Po);
            V = R * V_inertia;
            
            r = R * r_inertia;

            h = obj.cavf_local(t,P,V,r,Xf,uMax);
        end

%         function h = cavf_local_moving(obj,t,P,phi_des,V_des)
%             shape = obj.shape;
%             dist_i = obj.d_i; a_i = obj.a_i;
%             V_o = obj.rotation(t)*obj.Vo;
% 
%             h_o = obj.cavf_local(t,P,phi_des,V_des);
% 
%             a = norm(h_o)^2;
%             b = 2*( dot(V_o,h_o) );
%             c = norm(V_o)^2-V_des^2;
% 
%             det = b^2-4*a*c;
%             if det <0
%                 % no valid solutions
%                 h = [nan;nan];
%             elseif det ==0
%                 % one valid solution
%                 s = -b/(2*a);
%                 h = s*h_o+V_o;
%             else
%                 % two valid solutions, have to pick the better
%                 s1 = (-b + sqrt(det)) / (2*a);
%                 s2 = (-b - sqrt(det)) / (2*a);
%                 h1 = s1*h_o+V_o;
%                 h2 = s2*h_o+V_o;
%                 if( dot( Obstacle.dir_vec(phi_des),h1 ) > dot( Obstacle.dir_vec(phi_des),h2) )
%                     h = h1;
%                 else
%                     h = h2;
%                 end
%             end
%         end

        % calculate the cavf_local of the obstacle, assuming no movement and positioned at the origin
        %% cavf_local algo
        % generalized cavf_local algorithm
        % inputs:
        %   point P = [x,y]' : to calculate the cavf_local heading at
        %   phi_des : desired heading
        function h = cavf_local(obj,t,P,V,r,Xf,uMax)  
            dim = length(P);
            shape = obj.shape;
            di = obj.d_i; a_i = obj.a_i;

            V_des = r;
            isInside = shape.isInside;
            closestPoint = shape.closestPoint;
            
            if(isInside(P,t)) % if inside shape, return nan
                h = nan * ones(dim,1);
                return
            else
                I = closestPoint(P,t);
                
                dLim = computeDLim(obj, P, V, uMax);
                P = P + (I-P)/norm(I-P) * dLim;
                
                if(any(isnan(P)))
                    h = nan * ones(dim,1);
                    return
                elseif (norm(P-I) > di) % outside influence
                    h = V_des;
                    %h = zeros(dim,1);
                    return
                else
                    d1 = norm(P-I); d2 = d1-di;
                    x = (d1+d2) / (d1*d2);
                    if(d2==0)
                        g = 0;
                    else
                        g = a_i*x/sqrt(1+(2*a_i*x)^2) + .5;
                    end
                    % modulate steer angle from tangent to desired direction
                    % take normal as the vector from I to P, pointed out of shape
                    n_hat = normalize( P-I , 'norm' );
                    % calculate surface tangent that is in the direction of V_des
%                     t_hat = cross([n_hat],[0;0;1]);
%                     t_hat = t_hat(1:2);
% 
%                     if( abs(t_hat'*normalize(V_des,'norm')) < 0.01)
%                         if [0,0,1]*cross([P;0],[V_des;0]) > 0
%                             % below obstacle
%                             t_hat = -t_hat;
%                         end
%                     else
%                         if(dot(t_hat,normalize(V_des,'norm')) <0)
%                             t_hat = -t_hat;
%                         end
%                     end

                    % required angle at the surface is no longer in t_hat direction
                    V_o = obj.rotation(t)*obj.Vo;
                    if(shape.nonrigid)
                        V_exp = shape.V_exp(I,t);
                        switch dim
                            case 2
                                V_rot = cross([0;0;shape.params.omega(t)],[I;0]);
                                V_rot = V_rot(1:2);
                            case 3
                                V_rot = cross([0;0;shape.params.omega(t)],I);
                        end
                        V_surf = V_exp + V_rot + V_o;
                    else
                        V_surf = V_o;
                    end

                     v_n = V_surf' * n_hat;
                     % Considering the obstacle’s velocity in the normal
                     % direction n only if it is greater than zero. 
                     if (v_n < 0)
                         V_surf = V_surf - v_n * n_hat;
                     end
%                     if(speed < v_n)
%                         v_t = 0; % potentially no escape
%                     elseif(speed < abs(v_n))
%                         % negative v_n
%                         v_t = 0; v_n = -speed;
%                     else
%                         v_t = sqrt( speed^2 - v_n^2);
%                     end
% 
%                     % velocity required at tangency
%                     V_t = v_n*n_hat + v_t*t_hat;
%                     if(~isreal(V_t(1)) || ~isreal(V_t(2)) )
%                         keyboard
%                     end
% 
%                     phi_t = atan2(V_t(2),V_t(1));
% 
%                     tstar = dot(n_hat,V_des);
%                     if(tstar > 0) % passed obstacle
%                         l = -(1-tstar)*(1-g) +1;
%                     else
%                         l = g;
%                     end
%                     % restrict angles to [0,2*pi) for angle interpolation to work properly
%                     %keyboard
%                     while(abs(phi_des-phi_t) > pi)
%                         if(phi_des - phi_t > pi)
%                             phi_t = phi_t + 2*pi;
%                             %keyboard
%                         elseif(phi_t -phi_des > pi)
%                             phi_des = phi_des + 2*pi;
%                             %keyboard
%                         end
%                     end
                    
%                     tVec = V_t / d1;
%                     h = (r-tVec)*l + tVec;
%                     return

                    switch dim
                        case 2
                            RthetaObs = [ cos(obj.theta) sin(obj.theta);
                                         -sin(obj.theta) cos(obj.theta)];
                        case 3
                            Rtheta = [ cos(obj.theta) sin(obj.theta) 0;
                                      -sin(obj.theta) cos(obj.theta) 0;
                                      0               0              1];
                            Rbeta = [1 0             0;
                                     0 cos(obj.beta) sin(obj.beta);
                                     0 -sin(obj.beta) cos(obj.beta)];
                            RthetaObs = Rbeta * Rtheta;
                    end

                    
                    Xo = obj.position(t);
                    P = Xo + RthetaObs' * P;
                    I = Xo + RthetaObs' * I;
                    n_hat = normalize( P-I , 'norm' );
                    V_surf = RthetaObs' * V_surf;
                    
                    TargetVec = TargetCAVF(Xf, P);
                    ObsXf = Xf - Xo;
                    
                    %thetaObsXf = atan2(ObsXf(2),ObsXf(1));  
                    %thn = atan2(n_hat(2),n_hat(1));
                    %th2 = -wrapToPi(thn - thetaObsXf);

                    d = norm(P-I); d2 = d-di;
                    x = (d+d2) / (d*d2);
                    if(d2==0)
                        g = 0;
                    else
                        g = a_i*x/sqrt(1+(2*a_i*x)^2) + .5;
                    end
                    bi = 0.01;
                    beta = exp(-bi * x^2);
                    
                    ho = g * norm(TargetVec) * n_hat;
                    hf = TargetVec;

                    h = ho + hf;
                    
                    %if (I(1) < 0.1 && d < 0.1) keyboard end
                    
                    switch dim
                        case 2
                            th = -atan2([0 0 1]*cross([ObsXf;0],[n_hat;0]),dot(n_hat,ObsXf));
                            %th = th / 2 * g;
                            th = th / 2 * beta;
                            R = [cos(th) sin(th); -sin(th) cos(th)];
                        case 3
                            biss = norm(n_hat)*ObsXf + norm(ObsXf)*n_hat;
                            % find a rotation matrix R that rotates unit vector a onto unit vector b.
                            % https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
                            % Skew symmetric matrix
                            b = biss/norm(biss);
                            a = ObsXf/norm(ObsXf);
                            v = cross(a,b);
                            c = dot(a,b);
                            V = [0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ];
                            if (c < -1 + 1e-9)
                                R = [cos(c) sin(c) 0; -sin(c) cos(c) 0; 0 0 1];
                            else
                                R = eye(3) + g * (V + V^2 / (1 + c));
                            end
                    end
                    h = R * h + g * V_surf;
                end
            end
        end
%         
%         function h = cavf_local_DoubleIntegrator(obj, t, P, Xf)
%             Xo = obj.position(t);
%             
%             shape = obj.shape;
%             
%             ao = obj.a_i;
%             di = obj.d_i;
%             
%             TargetVec = Xf - P;
%             phi_des = atan2(TargetVec(2),TargetVec(1));
% 
%             ObsXf = Xf - Xo;
%             thetaObsXf = atan2(ObsXf(2),ObsXf(1));
% 
%             r = norm(P-Xo);
%             rVec = P-Xo;
%             rUnit = rVec/r;
%             
%             if(shape.isInside(P,t)) % if inside shape, return nan
%                 h = [nan;nan];
%                 return
%             end
%             I = shape.closestPoint(P,t);
%             if(norm(P-I) > di)
%                 h = TargetVec;
%             elseif(isnan(I))
%                 h = [0;0];
%             else
%                 d1 = norm(P-I); d2 = d1-di;
%                 x = (d1 + d2) / (d1*d2);
%                 if(d2==0)
%                     g = 1;
%                 else
%                     g = ao*x/sqrt(1+(2*ao*x)^2) + .5;
%                 end
%                 n = obj.find_normal(I);
% 
%                 thn = atan2(n(2),n(1));
%                 th = -wrapToPi(thn - thetaObsXf) / 2 * g;
% 
%                 d = norm(P-I); db = 1;
%                 ho = norm(TargetVec) * db/(d+db) * (di-d)/di * n;
% 
%                 hf = TargetVec;
%                 h = hf + ho;
% 
%                 R = [cos(th) sin(th); -sin(th) cos(th)];
%                 h = R * h;
%             end
%         end
        
       function n = find_normal(obj, I)
            a = obj.shape.params.a; b = obj.shape.params.b;
            theta = obj.theta;
            % rotate I
            R_OI = [cos(theta) sin(theta); -sin(theta) cos(theta)];
            I_O = R_OI'*I;
            if(I_O(2)==0)
                n_O = -[1;0];
            elseif(I_O(1)==0)
                n_O = -[0;1];
            else
                mt = -b^2/a^2 * I_O(1)/I_O(2);
                mn = -mt^-1; % slope of normal
                n_O = normalize([1;mn],'norm');
            end
            % condition n_O to always point out of ellipse
            n_O = n_O * sign(dot(n_O,I_O));
            n = R_OI * n_O;
            % keyboard
            % if(I(1)==0) s1 = 1; else s1 = sign(I(1)); end
            % if(I(2)==0) s2 = 1; else s2 = sign(I(2)); end
            % n = - n * s1*s2;
            % keyboard
        end

    end

    %methods (Sealed), for display
    methods (Sealed,Access=protected)
        function header = getHeader(obj)
            header = getHeader@matlab.mixin.CustomDisplay(obj);
        end
        function footer = getFooter(obj)
            footer = getFooter@matlab.mixin.CustomDisplay(obj);
        end
        function PropertyGroups = getPropertyGroups(obj)
            PropertyGroups = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
        end
        function displayNonScalarObject(obj)
            displayNonScalarObject@matlab.mixin.CustomDisplay(obj);
        end
    end
end
