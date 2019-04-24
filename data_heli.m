% System, constraints and target set
% 
% system - x+ = Ax + Bu + Ew
% constraints -  Z = {x, u | Cx + Du \leq e }
% disturbance - W = { w | Ew \leq g }
% cost J = sum_0^L-1  x_k'Qx_k + u_k'Ru_k + x_L' P x_L
% terminal constraints -  X_f = { x | Gx \leq h }
% terminal cost Vf = 0.5 * x' * P * x, A'PA - P- A'PB(R+B'PB)^-1 B'PA + Q = 0
%

% initial condition


% system 
h = 0.1;
system.A = [1 h   0            0         0          0;
            0 1 -h*K_2         0         0          0;
            0 0   1            h         0          0;
            0 0 -h*K_1*K_pp 1-h*K_1*K_pd 0          0;
            0 0   0            0         1          h;
            0 0   0            0       -h*K_3*K_ep 1-h*K_3*K_pd ];
system.B = [0 0;
            0 0;
            0 0;
            h*K_1*K_pp 0;
            0 0;
            0 h*K_3*K_ep ];

% For now we're gonna do this        
% Disturbance on each state, lambda, p, e
system.E = [0 0; 
            1 1; 
            0 0; 
            0 1; 
            0 0; 
            0 1];
system.n = size(system.A,2);
system.m = size(system.B,2);

system.alpha = 0.15;
% constraints
% abs(p_k) \leq 30 deg
% this is so sin(p) = p becomes accurate
constraints.C = [0 0  1 0 0 0;
                 0 0 -1 0 0 0;
                 0 0  0 0 0 0;
                 0 0  0 0 0 0 ];
constraints.D = [0 0;
                 0 0;
                 1 0;
                -1 0 ];
constraints.e = ( 30*pi / 180 ) * ones(size(constraints.C,1),1);

% disturbance
disturbance.E = [1 0; -1 0; 0 1; 0 -1];
disturbance.g = 0.2*ones(size(disturbance.E,1),1);

% cost
cost.Q = 3*eye(system.n);
cost.R = 2*eye(system.m);



% max number of iterations
system.Nsim = 100;