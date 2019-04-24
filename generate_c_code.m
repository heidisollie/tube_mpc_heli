

n = problem.system.n;
m = problem.system.m;
N = problem.system.N;
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
output = qp_problem.solve();

qp_problem.codegen('c_code', 'parameters', 'matrices');