function A = gen_aeq(A1,B1,N,mx,mu)
% Function to build a matrix representing equality constraints for a 
% linear discrete time dynamical system. The resulting matrix has the form                            
%      -                    -                                   
%      |I        |-B1       |                                   
%  A = |-A1 I    |  -B1     |                                   
%      |   . .   |     .    |                                   
%      |   -A1 I |      -B1 |                                   
%      -                    -                                   
%                                                               
% A1 - Discrete time system matrix                          
% B1 - Discrete time input matrix                       
% N  - Time horizon, assumes M = N
% mx - Number of states                               
% mu - Number of inputs                                     
%                                                               
% 08.03.2001 Geir Stian Landsverk
% January 2018, Andreas L. Flåten (translation to English)
                                                              

A 	= zeros(N*mx,N*mx+N*mu);
b1	= diag_repeat(-B1,N);
a1	= eye(N*mx);
for i=1:(N-1)
  a1([(i*mx+1):((i+1)*mx)],[((i-1)*mx+1):(i*mx)])=-A1;
end
A	= [a1 b1];