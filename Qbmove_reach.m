function [l, l_x, l_xx, l_u, l_uu, l_ux] = Qbmove_reach(x,u,t)
            %Rtf  = diag([w_tf, 0, 0, 0, 0, 0]);
            Hf = [1,0,0,0;
                0,1,0,0;
                0,0,1,0;
                0,0,0,1];
            Hr = [1,1,0,0;
                1,1,0,0;
                0,0,1,1;
                0,0,1,1];
            if (isnan(u))
                % final cost
                %fh = @(x)obj.costf_reach(x);
                
                l = (x')*Hf*(x)/2 ; %+ Hf(5)*u(1)^2/2 + Hf(6)*u(2)^2/2 + Hf(7)*u(4)^2/2 + Hf(8)*u(5)^2/2;
                if nargout > 1
                    %l_x = get_jacobian_fd(fh, x);
                    %l_xx = get_hessian_fd(fh, x);
                    l_x = (x')*Hf ;
                    l_xx = Hf;
                end
            else
                % running cost
                %para = [];
                %para.w_t = w_t;
                %para.w_e = w_e;
                %fl = @(x,u,t) obj.costr_reach(x,u,t);
                %l = fl(x,u,t);
                l = u'*Hr*u;
                if nargout > 1
                    % finite difference
                    %flJ=@(x,u,t)J_cost_fd ( fl, x, u, t );
                    %[l_x ,l_u      ] = flJ ( x, u, t );
                    %flH =@(x,u,t)H_cost_fd  ( flJ, x, u, t );
                    %[l_xx,l_uu,l_ux] = flH  ( x, u, t );
                    
                    l_x = [0 0 0 0 0 0 0 0];
                    l_u = u'*Hr;
                    l_xx = zeros(4,4);
                    l_ux = zeros(4,4);
                    l_uu = Hr;
                    
                end
            end
        end