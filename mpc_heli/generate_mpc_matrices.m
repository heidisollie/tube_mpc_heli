function [mpc_cost, mpc_constraints] = generate_mpc_matrices(problem)

N = problem.system.N;
n = problem.system.n;
m = problem.system.m;
% Inequality constraints


% Dette er nok feil
% Du må legge den G under, se original koden 
c1 = blkdiag(kron(eye(N), problem.constraints.C));
c2 = [zeros(size(problem.constraints.G,1) , n * (N-1)) problem.constraints.G ];
c = [c1; c2];
d = [kron(problem.constraints.D, eye(N)); zeros(size(problem.constraints.G,1) , m * N) ];
a = [c d];

b = kron(ones(N,1),problem.constraints.e);
cin = zeros(N*size(problem.constraints.C,1) + size(problem.constraints.G,1), n);

mpc_constraints.Ain = [c d];
mpc_constraints.bin = [b; problem.constraints.h];
mpc_constraints.cin = [cin];

% Equality constraints

a = [kron(eye(N), -problem.system.A) zeros(n*N, n)] + [zeros(n*N, n) kron(eye(N), eye(n))];
b = kron(eye(N), -problem.system.B);

mpc_constraints.Aeq = gen_aeq(problem.system.A, problem.system.B, N, n, m);
mpc_constraints.beq = [problem.system.A; zeros((N-1)*n, n) ];

% Cost
H = blkdiag(kron(eye(problem.system.N-1), problem.cost.Q), problem.cost.P, ...
            kron(eye(problem.system.N), problem.cost.R));
mpc_cost.H = 2*H;
mpc_cost.f = zeros(size(H,1), 1);