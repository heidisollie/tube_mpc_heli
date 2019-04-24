function [mpc_cost, mpc_constraints] = generate_mpc_matrices(problem)

N = problem.system.N;
n = problem.system.n;
m = problem.system.m;
Sx = problem.system.Sx;
Su = problem.system.Su;
%x = Su * u + Sx * x0


% Inequality constraints

% Cx + Du \leq e
C_bar = kron(eye(N), problem.constraints.C);
D_bar = kron(eye(N), problem.constraints.D);
a1 = C_bar * Su + D_bar;
b1 = kron(ones(N,1),problem.constraints.e);
c1 =  -C_bar * Sx;

% GzN \leq h

%a2 = problem.constraints.G * Su((N-1)*n+1:end, 1:end);
%b2 = problem.constraints.h;
%c2 = -problem.constraints.G * Sx((N-1)*n+1:end,1:n);

% S(x - z0) \leq r
%a3 = [-problem.constraints.S * problem.system.B zeros(size(problem.constraints.S,1), m * (N-1)) ];
%b3 = problem.constraints.r;
%c3 = problem.constraints.S*(problem.system.A * eye(n));



%mpc_constraints.Ain = [a1; a2; a3];
%mpc_constraints.bin = [b1; b2; b3];
%mpc_constraints.cin = [c1; c2; c3];

mpc_constraints.Ain = [a1];
mpc_constraints.bin = [b1];
mpc_constraints.cin = [c1];

% Cost

Q_big = blkdiag(kron(eye(N-1), problem.cost.Q), 0*problem.cost.P);
R_big = kron(eye(problem.system.N), problem.cost.R);
H = Su' * Q_big * Su + R_big;
f = 2 * Sx' * Q_big * Su;

mpc_cost.H = 2 * H;
mpc_cost.f = 2 * f;


