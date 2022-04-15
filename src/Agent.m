% class of an agent moving through obstacles
% contains the algorithm for following multiple obstacles
% along with what controller is used to follow the cavf

classdef Agent < handle
    properties
        P;      % position at time t
        %phi;    % heading at time t
        V;      % velocity
        Xf;
        uMax;
        %Vf;
        P_hist; % time histories; updated to time t
        V_hist;
        u_hist;
        %phi_hist;
        % controller gains
        K = 3;
    end

    methods
        function obj = Agent(P,V,Xf,uMax)
            if nargin < 4
                uMax = 1e16;
            end
            dim = length(V);
            obj.P = P;
            %obj.phi = phi;
            obj.V = V;
            obj.Xf = Xf;
            obj.uMax = uMax;
            obj.P_hist = P;
            obj.V_hist = V;
            obj.u_hist;
            %obj.phi_hist = phi;
        end

        % agent controller, outputs velocities (Xdot) given a cavf (h) and current state (velocities)
        function Xdot = controller(obj,h,X,hdot)
            dim = length(X) / 2;
            P = X(1:dim); 
            V = X(dim+1:end);
            % simple proportional control
            K = obj.K;
            u = K*(h-V) + hdot;
            u = limitU(obj,u);
            P_dot = V;
            V_dot = u;
            if dim == 3
                g = 9.81;
                u = u + [0;0;g];
                V_dot = u + [0;0;-g];
            end
            Xdot = [P_dot; V_dot];
            
            %norm(K*(h-V)) / (norm(K*(h-V)) + norm(hdot))
            %keyboard
            return
            %Xdot = h;
        end

        function [] = update(obj,X,t_new,u)
            u = limitU(obj,u);
            dim = length(X) / 2;
            P_new = X(1:dim); V_new = X(dim+1:end); %phi_new = atan2(X(4),X(3));
            obj.P = P_new;
            obj.V = V_new;
            %obj.phi = phi_new;
            obj.P_hist(:,end+1) = P_new;
            %obj.phi_hist(:,end+1) = phi_new;
            obj.V_hist(:,end+1) = V_new;
            obj.u_hist(:,end+1) = u;
        end

        function [] = reset(obj)
            obj.P = obj.P_hist(:,1);
            obj.P_hist = obj.P;
            obj.V_hist = obj.V;
        end
        
        function uNew =  limitU(obj,u)
            if norm(u) > obj.uMax
                uNew = u / norm(u) * obj.uMax;
            else
                uNew = u;
            end
        end

        function h = plot_hist(obj,idx)
            if(idx > length(obj.P_hist))
                warning('histories are not updated?')
                return
            end
            dim = length(obj.P_hist(:,1));
            switch dim
                case 2
                    h = scatter(obj.P_hist(1,idx),obj.P_hist(2,idx),50,'filled');
                case 3
                    h = scatter3(obj.P_hist(1,idx),obj.P_hist(2,idx),obj.P_hist(3,idx),50,'filled');
                otherwise
                    disp('Error: plot_hist(obj,idx)');
            end
        end
    end
end
