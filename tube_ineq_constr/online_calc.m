function optimal = online_calc(problem,x)

n = problem.system.n;
m = problem.system.m;
N = problem.system.N;

options = optimset('Display', 'on');
% get optimal decision variable and optimal value
disp('Solving with quadprog');             
[output, ~] = quadprog(problem.mpc_cost.H, ... 
                                    x' * problem.mpc_cost.f, ...
                                    problem.mpc_constraints.Ain, ...
                                    problem.mpc_constraints.bin + problem.mpc_constraints.cin * x, ...
                                    [], ... % equality constraints now inequality
                                    [], ... % equality constraints now inequality
                                    [], [], [], ...
                                    options);

% calculate output to obtain optimal x
% x = Su u + Sx x

disp(output);
z = zeros(n,N);
v = zeros(m,N);

z = problem.system.Su * output + problem.system.Sx * x;
v = output;

optimal.z = z;
optimal.v = v;
