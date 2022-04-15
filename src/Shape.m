classdef Shape < handle
    methods (Static)
        % ellipse struct constructor
        %   inputs:
        %       a - length of semi-axis along x
        %       b - length of semi-axis along y
        function ell = ellipse(a,b)
            params = struct('a',a,'b',b);
            nearest_ellipse = @(P,~) Shape.find_nearest_point(P,@Shape.c_ellipse,@Shape.E_ellipse,@Shape.dt_ellipse,params);
            inside_ell = @(P,~) Shape.inside_ellipse(P,params);
            parametric_ell = @(t,~) Shape.c_ellipse(t,params);
            ell = struct('type','ellipse','nonrigid',false,'params',params,'isInside',inside_ell,'closestPoint',nearest_ellipse,'parametric',parametric_ell);
        end
        
        function ell = ellipsoid(a,b,c)
            params = struct('a',a,'b',b,'c',c);
            nearest_ellipse = @(P,~) Shape.find_nearest_point_ellipsoid(P,params);
            inside_ell = @(P,~) Shape.inside_ellipsoid(P,params);
            parametric_ell = @(u,v,~) Shape.c_ellipsoid(u,v,params);
            ell = struct('type','ellipsoid','nonrigid',false,'params',params,'isInside',inside_ell,'closestPoint',nearest_ellipse,'parametric',parametric_ell);
        end

        function ell = ellipseMoving(a,b,da,db,theta,omega,a0,b0)
            if(exist('a0','var'))
                params = struct('a',a,'b',b,'da',da,'db',db,'theta',theta,'omega',omega,'a0',a0,'b0',b0);
            else
                params = struct('a',a,'b',b,'da',da,'db',db,'theta',theta,'omega',omega);
            end
            unparam = @(I,t) atan2(I(2)/params.b(t),I(1)/params.a(t));
            V_exp = @(I,t) [da(t)*cos(unparam(I,t));db(t)*sin(unparam(I,t))];

            params_static = @(t) Shape.moving_wrapper(t,params);
            nearest_ellipse = @(P,t) Shape.find_nearest_point(P,@Shape.c_ellipse,@Shape.E_ellipse,@Shape.dt_ellipse,params_static(t));
            inside_ell = @(P,t) Shape.inside_ellipse(P,params_static(t));
            parametric_ell = @(th,t) Shape.c_ellipse(th,params_static(t));
            parametricDet_ell = @(th,t) Shape.cDet_ellipse(th,params_static(t));
            ell = struct('type','ellipse','nonrigid',true,'params',params,'params_static',params_static,'isInside',inside_ell,'closestPoint',nearest_ellipse,'parametric',parametric_ell,'parametricDeterminist',parametricDet_ell,'V_exp',V_exp);
        end
        
        function ell = ellipsoidMoving(a,b,c,da,db,dc,theta,beta,omega,a0,b0,c0)
            if(exist('a0','var'))
                params = struct('a',a,'b',b,'c',c,'da',da,'db',db,'dc',dc,'theta',theta,'beta',beta,'omega',omega,'a0',a0,'b0',b0,'c0',c0);
            else
                params = struct('a',a,'b',b,'c',c,'da',da,'db',db,'dc',dc,'theta',theta,'beta',beta,'omega',omega);
            end
            phi = @(I,t) atan2(I(2)/params.b(t),I(1)/params.a(t));
            theta = @(I,t) acos(I(3)/params.c(t));
            V_exp = @(I,t) [da(t)*sin(theta(I,t))*cos(phi(I,t));db(t)*sin(theta(I,t))*sin(phi(I,t));dc(t)*cos(theta(I,t))];

            params_static = @(t) Shape.moving_wrapper(t,params);
            nearest_ellipse = @(P,t) Shape.find_nearest_point_ellipsoid(P,params_static(t));
            inside_ell = @(P,t) Shape.inside_ellipsoid(P,params_static(t));
            parametric_ell = @(u,v,t) Shape.c_ellipsoid(u,v,params_static(t));
            parametricDet_ell = @(th,t) Shape.cDet_ellipse(th,params_static(t));
            ell = struct('type','ellipsoid','nonrigid',true,'params',params,'params_static',params_static,'isInside',inside_ell,'closestPoint',nearest_ellipse,'parametric',parametric_ell,'parametricDeterminist',parametricDet_ell,'V_exp',V_exp);
        end

        % superellipse struct constructor
        %   inputs:
        %       a - length of semi-axis along x
        %       b - length of semi-axis along y
        %       n - order of superellipse
        function sell = superellipse(a,b,n)
            params = struct('a',a,'b',b,'n',n);
            nearest_sellipse = @(P,~) Shape.find_nearest_point(P,@Shape.c_sellipse,@Shape.E_sellipse,@Shape.dt_sellipse,params);
            inside_sell = @(P,~) Shape.inside_sellipse(P,params);
            parametric_sell = @(t,~) Shape.c_sellipse(t,params);
            sell = struct('type','superellipse','nonrigid',false,'params',params,'isInside',inside_sell,'closestPoint',nearest_sellipse,'parametric',parametric_sell);
        end

        % rectangle
        %   inputs:
        %       x - half length of rectangle along x
        %       y - half length of rectangle along y
        function rect = rectangle(x,y)
            params = struct('x',x,'y',y);
            nearest_r = @(P,~) Shape.closest_rect(P,params);
            inside_r = @(P,~) Shape.inside_rect(P,params);
            rect = struct('type','rectangle','nonrigid',false,'params',params,'isInside',inside_r,'closestPoint',nearest_r);
        end

        function rect = rectangleMoving(x,y,dx,dy,theta,omega)
            params = struct('x',x,'y',y,'dx',dx,'dy',dy,'theta',theta,'omega',omega);
            params_static = @(t) Shape.moving_wrapper(t,params);
            nearest_r = @(P,t) Shape.closest_rect(P,params_static(t));
            inside_r = @(P,t) Shape.inside_rect(P,params_static(t));
            V_exp = @(I,t) Shape.velexp_rect(I,t,params);
            rect = struct('type','rectangle','nonrigid',true,'params',params,'params_static',params_static,'isInside',inside_r,'closestPoint',nearest_r,'V_exp',V_exp);
        end
        % parabola
        %   inputs:
        %       a - coefficient
        %       h - height
        function parab = parabola(a,h)
            params = struct('a',a,'h',h);
            nearest = @(P,~) Shape.nearest_parab(P,params);
            inside = @(P,~) Shape.inside_parab(P,params);
            parab = struct('type','parabola','nonrigid',false,'params',params,'isInside',inside,'closestPoint',nearest);
        end


        % triangle
        %   inputs:
        %       B - base, B>0
        %       H - height, H>0
        %       T - percent of B tilt
        function tri= triangle(B,H,T)
            params = struct('B',B,'H',H,'T',T);
            nearest = @(P,~) Shape.nearest_triangle(P,params);
            inside = @(P,~) Shape.inside_triangle(P,params);
            tri = struct('type','triangle','nonrigid',false,'params',params,'isInside',inside,'closestPoint',nearest);
        end
    end

    methods (Access=private,Static)
        % moving shape parameter wrapper
        function params_out = moving_wrapper(t,params)
            params_out = struct();
            fs = fields(params);
            for i=1:length(fs)
                in = params.(fs{i});
                if isa(in,'function_handle')
                    params_out.(fs{i}) = in(t);
                else
                    params_out.(fs{i}) = in;
                end
            end
        end
        % expanding velocity of rectangle
        function V_exp = velexp_rect(I,t,params)
            if( abs(I(1))>=params.x(t) && abs(I(2))>=params.y(t) )
                V_exp = [sign(I(1))*params.dx(t);sign(I(2))*params.dy(t)];
                return
            elseif( abs(I(1))>=params.x(t) )
                V_exp = [sign(I(1))*params.dx(t);0];
                return
            else
                V_exp = [0;sign(I(2))*params.dy(t)];
            end
        end
    end

    % evolute closest point estimator
    methods (Access=private,Static)
        %% generic nearest point iterative estimator
        % uses the parametric equation for a curve and its evolute to approximate the closest point on the curve to a test point
        % iterative process that could be computationally efficient and quickly converging

        % estimate the nearest point.
        % inputs:
        %       p_t: test point outside curve
        %       c: function handle of the parametric curve as a function of t : [x,y]' = c(t,params)
        %       E: function handle of the parametric evolute of the curve as a function of t: [X,Y]' = E(t,params)
        %       dt_func : function handle of the approximate delta t to travel a distance delta c from point p on the curve based on the derivatives of c : delta_t = dt_func(dc,p,params)
        %       params: struct of parameters to give to c and E
        function p_nearest = find_nearest_point(p_t,c_func,E_func,dt_func,params)
            p = abs(p_t);
            t = atan(p(2)/p(1));
            for(iter = 1:4)
                % point on curve based on guessed t
                i = c_func(t,params);
                % evolute
                e = E_func(t,params);
                % vector from evolute point to guess
                r = i-e;
                % vector from evolute to position
                q = p-e;
                % approx angle *r between r and q; estimation of second intersection point
                dc = (cross([r;0],[q;0]))/norm(q);
                dc = dc(3);
                dt = dt_func(t,dc,i,params);
                % update guess of t
                t = t + dt;
                if(t<0)
                    t = 0;
                end
                %t = min
            end
            % calculate point on curve corresponding to t
            p_nearest = c_func(t,params);
            p_nearest = [sign(p_t(1)) 0;
            0   sign(p_t(2))] * p_nearest;
            if(~isreal(p_nearest))
                p_nearest = real(p_nearest);
            end
        end
        
        function Xo = find_nearest_point_ellipsoid( X, axis )
        %
        % The shortest distance from a point to Triaxial Ellipsoid or Biaxial Ellipsoid or Sphere
        %
        %   (x/a)^2+(y/b)^2+(z/c)^2=1   Triaxial Ellipsoid Equation centered at the
        %   origin
        %    
        %
        % Parameters:
        % * X, [x y z]     - Cartesian coordinates data, n x 3 matrix or three n x 1 vectors
        % * axis,[a; b; c] - ellipsoid radii  [a; b; c],its axes % along [x y z] axes
        %  
        %                  For Triaxial ellipsoid ,it must be a > b > c
        %
        %                  For Biaxial ellipsoid ,it must be a = b > c
        %
        %                  For Sphere ,it must be a = b = c
        %
        % Output:
        % * Xo,[xo yo zo]   -  Cartesian coordinates of Point onto ellipsoid 

            format long
            eps=0.0001; % three sholde.ar
            a=axis.a;b=axis.b;c=axis.c;

            x=X(1);y=X(2);z=X(3);

            E=sign(a)/a^2;F=sign(b)/b^2;G=sign(c)/c^2;
            xo=x;yo=y;zo=z;% deneme
            %xo,yo,zo
            for i=1:20
                j11=F*yo-(yo-y)*E;
                j12=(xo-x)*F-E*xo;

                j21=G*zo-(zo-z)*E;
                j23=(xo-x)*G-E*xo;

                A=[ j11   j12   0 
                    j21   0   j23
                    2*E*xo    2*F*yo  2*G*zo  ];

                sa=(xo-x)*F*yo-(yo-y)*E*xo;
                sb=(xo-x)*G*zo-(zo-z)*E*xo;
                se=E*xo^2+F*yo^2+G*zo^2-1;
                Ab=[ sa  sb  se]';
                bil=-A\Ab;
                xo=xo+bil(1);
                yo=yo+bil(2);
                zo=zo+bil(3);
                if max(abs(bil))<eps
                    break
                end
            end
            dis=sqrt((x-xo)^2+(y-yo)^2+(z-zo)^2);
            Xo=[xo; yo; zo];
        end
        
    end

    % functions for ellipse
    % params are: a x-semiaxis, b y-semiaxis
    methods (Access=private,Static) 
        % parametric curve of an ellipse
        function p = c_ellipse(t,params)
            a = params.a; b = params.b;
            p = [a*cos(t); b*sin(t)];
        end
        function p = cDet_ellipse(t,params)
            a = params.a0; b = params.b0;
            p = [a*cos(t); b*sin(t)];
        end
        % evolute of an ellipse
        function e = E_ellipse(t,params)
            a = params.a; b = params.b;
            e = [(a^2-b^2) * cos(t)^3 / a; (b^2-a^2) * sin(t)^3 / b];
        end
        % dt func of an ellipse
        function dt = dt_ellipse(t,dc,p,params)
            a = params.a; b = params.b;
            dt = dc / sqrt(a^2+b^2-norm(p)^2);
        end
        % inside ellipse
        function inside = inside_ellipse(P,params)
            a = params.a; b = params.b;
            inside = P(1)^2/a^2 + P(2)^2/b^2 <= 1;
        end
        
        % plot ellipse
        function h = plot_ellipse(params)
            t = 0:.01:2*pi;
            P = c_ellipse(t,params)
            plot(P(1,:),P(2,:))
        end

    end
    
    % functions for ellipsoid
    % params are: a x-semiaxis, b y-semiaxis
    methods (Access=private,Static) 
        % parametric curve of an ellipse
        function p = c_ellipsoid(u,v,params)
            a = params.a; b = params.b; c = params.c;
            p = [a*cos(u).*sin(v); b*sin(u).*sin(v); c*cos(v)];
        end
        function p = cDet_ellipsoid(u,v,params)
            a = params.a0; b = params.b0; c = params.c0;
            p = [a*cos(u)*sin(v); b*sin(u)*sin(v); c*cos(v)];
        end
        
%         % dt func of an ellipse
%         function dt = dt_ellipsoid(t,dc,p,params)
%             a = params.a; b = params.b;
%             dt = dc / sqrt(a^2+b^2-norm(p)^2);
%         end
        % inside ellipse        
        function inside = inside_ellipsoid(P,params)
            a = params.a; b = params.b; c = params.c;
            inside = P(1)^2/a^2 + P(2)^2/b^2 + P(3)^2/c^2 <= 1;
        end
        
        % plot ellipse
        function h = plot_ellipsoid(params)
            u = 0:.01:2*pi;
            v = 0:.01:pi;
            P = c_ellipsoid(u,v,params);
            surf(P(1,:),P(2,:),P(3,:));
        end

    end

    % functions for superellipse
    % params are: a x-semiaxis,b y-semiaxis,n power
    methods (Access=private,Static)
        % is inside curve
        function inside = inside_sellipse(P,params)
            a = params.a; b = params.b; n = params.n;
            inside = abs(P(1,:)./a) .^n + abs(P(2,:)./b) .^n <1;
        end
        % lagrange multiplier method of nearest point estimation, potentially more stable?
        function P_near = nearest_sellipse(P,params)
            n = params.n; a = params.a; b = params.b;
            g = @(x,y) (x/a)^n + (y/b)^n -1;
            dgdx = @(x,y) (n/a)*(x/a)^(n-1);
            dgdy = @(x,y) (n/b)*(y/b)^(n-1);
            d2gdx2 = @(x,y) n*(n-1)/a^2 * (x/a)^(n-2);
            d2gdy2 = @(x,y) n*(n-1)/b^2 * (y/b)^(n-2);
            d2gdxy = @(x,y) 0;

            f = @(x,y) (x-P(1))^2 + (y-P(2))^2;
            dfdx = @(x,y) 2*(x-P(1));
            dfdy = @(x,y) 2*(y-P(2));
            d2fdx2 = @(x,y) 2;
            d2fdy2 = @(x,y) 2;
            d2fdxy = @(x,y) 0;

            % vector valued function to root find
            h = @(x,y,l) [  dfdx(x,y) - l*dgdx(x,y);
            dfdy(x,y) - l*dgdy(x,y);
            g(x,y) ];
            h_vec = @(H) h(H(1),H(2),H(3));
            % jacobian
            j = @(x,y,l) [  d2fdx2(x,y) - l*d2gdx2(x,y), d2fdxy(x,y) - l*d2gdxy(x,y), -1;
            d2fdxy(x,y) - l*d2gdxy(x,y), d2fdy2(x,y) - l*d2gdy2(x,y), -1;
            dgdx(x,y)                   ,dgdy(x,y)                  , 0];
            J_vec = @(J) j(J(1),J(2),J(3));

            H0=[1;1;100];

            hist = H0;
            %figure(2); clf; hold on; 
            % newton raphson method
            tic;
            iters = 0:70
            for i=iters(2:end)
                H1 = H0 - inv(J_vec(H0))*h_vec(H0);
                H0 = H1
                hist(:,end+1) = H0;
            end
        end

        % parametric curve
        function p = c_sellipse(t,params)
            a = params.a; b = params.b; n = params.n;
            x = a * abs(cos(t)).^(2./n) .*sign(cos(t));
            y = b * abs(sin(t)).^(2./n) .*sign(sin(t));
            p = [x;y];
        end
        % evolute
        function e = E_sellipse(t,params)
            t_lim = .075;
            if(t < t_lim); t = t_lim; end
            if(t > pi/2-t_lim); t = pi/2-t_lim; end
            c = Shape.c_sellipse(t,params);
            x = c(1,:); y = c(2,:);
            dc = Shape.dc_sellipse(t,params);
            dx = dc(1,:); dy = dc(2,:);
            ddc = Shape.ddc_sellipse(t,params);
            ddx = ddc(1,:); ddy = ddc(2,:);

            X = x - ( dy.*(dx.^2+dy.^2) ) ./ (dx.*ddy - ddx.*dy);
            Y = y + ( dx.*(dx.^2+dy.^2) ) ./ (dx.*ddy - ddx.*dy);
            e = [X;Y];
        end
        % dt func
        function dt = dt_sellipse(t,deltac,p,params)
            dc = Shape.dc_sellipse(t,params);
            dcdt = norm(dc);
            if(abs(dcdt) > 1e6); dt = 0; return; end
            if(abs(dcdt) <.001); dt = 0; return; end;
            dt = dcdt.^-1 .* deltac;
        end
        % derivative
        function dc = dc_sellipse(t,params)
            a = params.a; b = params.b; n = params.n;
            xd = -a.*2./n .* sin(t) .* (cos(t)).^(2./n-1);
            yd =  b.*2./n .* cos(t) .* (sin(t)).^(2./n-1);
            dc = [xd;yd];
        end
        % second derivative
        function ddc = ddc_sellipse(t,params)
            a = params.a; b = params.b; n = params.n;
            xdd = 2.*a./n .* ( (2./n-1).*sin(t).^2 .* (cos(t)).^(2./n-2) - (cos(t)).^(2./n) );
            ydd = 2.*b./n .* ( (2./n-1).*cos(t).^2 .* (sin(t)).^(2./n-2) - (sin(t)).^(2./n) );
            ddc = [xdd; ydd];
        end
    end

    % functions for rectangle
    % params are: x half-width, y half-width
    methods (Access=private,Static)
        % is inside rectangle
        function inside = inside_rect(P,params)
            x = params.x; y = params.y;
            inside = all( abs(P) <= [x;y] );
        end
        % closest point: simple since it is a rectangle
        function P_closest = closest_rect(P,params)
            x = params.x; y = params.y;
            P_abs = abs(P);
            if all( P_abs > [x;y] )
                P_closest = [x;y];
            elseif P_abs(1) > x
                P_closest = [x;P_abs(2)];
            else 
                P_closest = [P_abs(1);y];
            end
            P_closest = P_closest .* sign(P);
        end
        % plot rectangle
        function h = plot_rect(params)
            x = params.x; y = params.y;
            h = patch([-x,x,x,-x],[-y,-y,y,y],[.2,.2,.5])
        end
    end

    % functions for parabola
    % params are: a coefficient, h height from origin
    methods (Access=private,Static)
        % parabolas are of the form y=ax^2, y <= h
        % off center and rotated parabolas are defined by obstacle position and rotation
        function inside = inside_parab(P,params)
            a = params.a; h = params.h;
            if(any(P(2,:) > h))
                inside = P(2,:) <= h; return
            end
            inside = P(2) >= a.*P(1,:).^2 ;
        end

        function P_near = nearest_parab(P,params)
            a = params.a; h = params.h;
            if(P(2) >= h)
                if(abs(P(1)) - sqrt(h/a) > 0)
                    P_near = [sign(abs(P(1))-sqrt(h/a)) * sqrt(h/a); h];
                    return
                else
                    P_near = [P(1); h];
                    return
                end
            end
            % solve cubic nearest point problem
            polynom = [4*a^2, 0, 2-4*a*P(2), -2*P(1)];
            r_pot = roots(polynom);
            % guaranteed to have a real root
            x = NaN;
            y = @(x) a*x^2;
            for r=r_pot'
                if isreal(r)
                    if isnan(x)
                        x=r;
                    elseif norm( [r;y(r)]-P ) < norm( [x;y(x)]-P )
                        x=r;
                    end
                end
            end
            P_near = [x;y(x)];
        end
    end

    % functions for triangle
    methods (Access=private,Static)
        % triangle is defined as three points, and three parameters:
        %   point 1 is positioned at (0,0)
        %   point 2 is positioned 'B' to the right of 1 at (B,0), B>0
        %   point 3 is positioned at (T*B,H), where 'H' is the height of the triangle and 'T' represents the percent tilt of the triangle top. T=0 and T=1 are right angle triangles. H>0

        function inside = inside_triangle(Pm,params)
            % Triangle consists of 3 lines, L1=P1->P2; L2=P2->P3; L3=P3->P1.
            % A point P is within the triangle as determined by the sign of the determinant

            B = params.B; H = params.H; T = params.T;
            P1=[0;0]; P2=[B;0]; P3=[B*T;H];
            numpoints = size(Pm); numpoints = numpoints(2);
            inside = zeros(1,numpoints);
            i=1;
            for P=Pm
                d1 = det( [P';P2']-P1' );
                d2 = det( [P';P3']-P2' );
                d3 = det( [P';P1']-P3' );
                inside(i) = d1<=1e-3 && d2<=1e-3 && d3<=1e-3;
                i = i+1;
            end
        end
        function P_near = nearest_triangle(P,params)
            B = params.B; H = params.H; T = params.T;
            P1=[0;0]; P2=[B;0]; P3=[B*T;H];
            d1 = det( [P';P2']-P1' );
            d2 = det( [P';P3']-P2' );
            d3 = det( [P';P1']-P3' );
            % assume point is not inside
            if Shape.inside_triangle(P,params)
                P_near = [NaN;NaN];
                return
            end
            
            % check closest 
            if d1<0 && d2<0
                % closest to a point on line 3-1
                u = (P1-P3)/norm(P1-P3); 
                v = P-P3;
                dist = min(norm(P1-P3),max(0,dot(v,u)));
                P_near = dist*u+P3;
            elseif d1<0 && d3<0
                % closest to a point on line 2-3
                u = (P3-P2)/norm(P3-P2); 
                v = P-P2;
                dist = min(norm(P3-P2),max(0,dot(v,u)));
                P_near = dist*u+P2;
            elseif d2<0 && d3<0 
                % closest to a point on line 1-2
                u = (P2-P1)/norm(P2-P1); 
                v = P-P1;
                dist = min(norm(P2-P1),max(0,dot(v,u)));
                P_near = dist*u+P1;

            elseif d1<0
                % closest to point 3
                P_near = P3;
            elseif d2<0
                % closest to point 1
                P_near = P1;
            elseif d3<0
                % closest to point 2
                P_near = P2;
            else
                % error
                warning('nearest point algorithm for triangle failed to find a valid solution');
            end
        end
    end

end
