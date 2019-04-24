function [Sx Su] = genMPCprob(A,B,N)

% Generate reduced space matrices for MPC QP problem
% Inputs: 
%   A, B        System matrices in discrete time system: x+ = A x + B u
%   N           Control horizon length (degrees of freedom)
% 
% Outputs:
%   Sx          State predictions: 
%   Su              [x_1 x_2 ... x_N] = Sx*x_0 + Su*[u_0 u_1 ... u_N-1]

% 12/02/2018 Lars Imsland

% Define predictions:
% [x_1 x_2 ... x_N] = Sx*x_0 + Su*[u_0 u_1 ... u_N-1]

nx = size(A,1);
nu = size(B,2);

Sx = [eye(nx);A];
Su = zeros(N*nx,N*nu);
Su(1:nx,1:nu) = B;
for i = 2:N,
    Sx = [Sx;Sx((i-1)*nx+1:i*nx,:)*A];
    for j=1:i,
        Su((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu) = ...
            Sx((i-j)*nx+1:(i-j+1)*nx,1:nx)*B;
    end
end
Sx = Sx(nx+1:end,:); % remove first I


