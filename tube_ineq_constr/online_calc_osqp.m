function optimal = online_calc_osqp(problem,x)

n = problem.system.n;
m = problem.system.m;
N = problem.system.N;


% get optimal decision variable and optimal value
%OSQP: min 0.5 x^T P x + q^T x
%      s.t l \leq Ax \leq u


qp_problem = osqp;

ub = problem.mpc_constraints.bin + problem.mpc_constraints.cin * x;
lb = -Inf * ones(size(ub,1),1);
 
A = [problem.mpc_constraints.Ain];
settings = qp_problem.default_settings();
settings.eps_abs = 1e-04;
settings.eps_rel = 1e-04;
settings.verbose = 0;

qp_problem.setup(problem.mpc_cost.H, ...
                 x' * problem.mpc_cost.f, ...             
                 A, lb, ub, ...
                 settings);
             
disp('Solving with OSQP');             
output = qp_problem.solve();
optimal = output.x;
display(output.x)        

% calculate output to obtain optimal x
% x = Su u + Sx x

z = zeros(n,N);
v = zeros(m,N);

z = problem.system.Su * output + problem.system.Sx * x;
v = output;

optimal.z = z;
optimal.v = v;
