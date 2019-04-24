


% terminal cost
[cost.P, ~, ~] = dare(system.A, system.B, cost.Q, cost.R);

% state feedback
[K, cost.P] = dlqr(system.A, system.B, cost.Q, cost.R);
system.K = -K;

% problem reformulation
system.A_K = system.A + system.B * system.K;
constraints.C_K = constraints.C + constraints.D * system.K;

calc_constraints = tightened_constraints(constraints, system, disturbance);
system.N = 15;
[Sx, Su] = genMPCprob(system.A,system.B,system.N);
system.Sx = Sx;
system.Su = Su;
%constraints.e_org = constraints.e;
%constraints.e = t_constraints.e;

%terminal constraint set 
constraints.G = calc_constraints.G;
constraints.h = calc_constraints.h;

%sets
%X_c = c_tube(system,constraints);
%for i=1:system.N+1
%    X(i) = X_c(i) + system.S_K;
%end    

%assign to problem
problem.system = system;
problem.constraints = constraints;
problem.cost =  cost;
problem.disturbance = disturbance;

% generate mpc matrices
[problem.mpc_cost, problem.mpc_constraints] = generate_mpc_matrices(problem);

