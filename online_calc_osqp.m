function optimal = online_calc(problem,x)

n = problem.system.n;
m = problem.system.m;
N = problem.system.N;


% get optimal decision variable and optimal value
%OSQP: min 0.5 x^T P x + q^T x
%      s.t l \leq Ax \leq u
%
%          -inf \leq Ain \leq bin
%          -inf \leq Aeq \leq beq
%          beq  \leq Aeq \leq inf
qp_problem = osqp;

inf_array_in = Inf * ones(size(problem.mpc_constraints.bin,1),1);
inf_array_eq = Inf * ones(size(problem.mpc_constraints.beq,1),1);

lb = [-inf_array_in; ...
     -inf_array_eq; ...
     problem.mpc_constraints.beq];
 
ub = [problem.mpc_constraints.bin + problem.mpc_constraints.cin * x; ...
     problem.mpc_constraints.beq; ...
     inf_array_eq];
A = [problem.mpc_constraints.Ain; problem.mpc_constraints.Aeq; problem.mpc_constraints.Aeq];
settings = qp_problem.default_settings();
settings.eps_abs = 1e-04;
settings.eps_rel = 1e-04;
settings.verbose = 0;

qp_problem.setup(problem.mpc_cost.H, ...
                 problem.mpc_cost.f, ...             
                 A, lb, ub, ...
                 settings);
output = qp_problem.solve();
        

% devectorize output to obtain optimal x
% and optimal u

z = zeros(n,N);
v = zeros(m,N);
z(:,1) = output.x(1:n);

for i=1:N
    z(:,i+1) = output.x(i*n + 1: (i+1)*n);
    v(:,i) = output.x((N+1)*n + (i-1)*m + 1:(N+1)*n + i*m);
end

optimal.z = z;
optimal.v = v;
