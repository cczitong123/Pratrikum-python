classdef Qbmove2Dof < Arm2Dof
    %MCCPVD2D 
    % x: q1 q2 qd1 qd2 
    % u: 
    properties (Access = private)
      name = 'qbmove2'
    end
    properties
        %name = 'qbmove'
        actuator1
        actuator2
        
        dimQ = 2
        dimU = 4
        
        L = [0.3; 0.3]; 
        I = [0.0045; 0.0045]; 
        M = [0.15; 0.15]; %[1.59; 1.44];
        Lg = [0.15; 0.15];
        g = 0;
        
        viscous_friction = 0.01;
        coulomb_friction = 0;
        
    end
    
    methods
        function obj = Qbmove2Dof()
            obj = obj@Arm2Dof();
            obj.actuator1 = Qbmove();
            obj.actuator2 = Qbmove();
        end
        function torque = tau(obj, x, q1, q2)
            torque = obj.actuator1.torque(x(1,:), q1(1,:), q2(1,:));
            %{
            tau2 = obj.actuator2.torque(x(2,:), q1(2,:), q2(2,:));
            torque = [tau1; tau2];
            %}
        end

        function J = get_jacobian_fd( f, x )

            delta=1e-6;
            y = f(x);
            J = zeros(length(y),length(x));
            for i=1:length(x)
	            dx = zeros(size(x)); dx(i) = delta;
                try 
                    yp = f(x+dx);
                catch
                    yp = NaN;
                end
                try
                    ym = f(x-dx);
                catch
                    ym = NaN;
                end
                if isnan(yp)
                    J(:,i) = -ym/delta;
                elseif isnan(ym)
                    J(:,i) = yp/delta;
                else
                    J(:,i) = ((yp - ym)/(2*delta));
                end
                
            end
            end
        
        function qddot = qddot(obj, q, qdot, u)
            % u: [q1 q2]motor的角度
            q1_1 = u(1,:);
            q2_1 = u(2,:);
            q1_2 = u(3,:);
            q2_2 = u(4,:);
            x_1 = (q1_1+ q2_1)/2;
            x_2 = (q1_2+ q2_2)/2;
            tau1 = obj.tau(x_1, q1_1, q2_1);
            tau2 = obj.tau(x_2, q1_2, q2_2);
            % origin:tau = obj.tau(q, qdot, u);
            tau = [tau1;tau2];

            qddot = Arm2Dof.compute_qddot(q, qdot, tau, obj);

        end
        
        % x: q1 q2 qd1 qd2
        % u: u
        function xdot = dynamics(model, x, u)
            qddot = model.qddot( x(1:2,:), x(3:4,:), u);
            xdot = [x(3:4); qddot];
        end
        
        function x = endpoint(obj, q)
            x = Arm2Dof.endpoint(q, obj.L); 
        end
        
%         function J = jacobian(obj, q)
%            J = Arm2Dof.jacobian(q, obj.L);
%         end

        function [xdot, xdot_x, xdot_u] = dynamics_with_jacobian_fd(model, x, u)
            % uu: (m1 m2 u3)_1 , (m1 m2 u3)_2
            %uu = [x(5:6,:); u(3,:); x(9:10,:); u(6,:)];
            % joint accel
            %qddot = model.qddot( x, u);
            % first 4 of xdot
            %xdot1 = [x(3:4); qddot];
            % last 8 of xdot
            %[xdot2, xdot2_x, xdot2_u] = model.motor_dynamics_2nd(x(5:8,:), u(1:2,:));
            
            %[xdot3, xdot3_x, xdot3_u] = model.motor_dynamics_2nd(x(9:12,:), u(4:5,:));
            
            %xdot = [xdot1; xdot2; xdot3];
            xdot = model.dynamics(x,u);
            if nargout > 1
                fx = @(x)model.dynamics(x, u);
                %dfdx = get_jacobian_fd(fx, x);
                dfdx = jacobian(fx, x);
                if iscolumn(dfdx), dfdx = dfdx'; end
                % Compute derivative of acceleration w.r.t. u
                %daccdu = zeros(1,model.dimU);
                fu = @(u)model.dynamics(x, u);
                %dfdu = get_jacobian_fd(fu, u);%格式错误还是函数错误
                dfdx = jacobian(fu, u);
                if iscolumn(dfdu), dfdu = dfdu'; end
                xdot_x = dfdx;
                      
                xdot_u = dfdu;
            end
        end
    end

       
end
