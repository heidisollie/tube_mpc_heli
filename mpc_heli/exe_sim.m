% close all;

alpha_val = 0.15;
init;
data;
offline_calc;
%%

problem.system.w_sequence = generate_disturbance(problem);

system.x0 = [pi/6; 0; pi/6; 0; pi/8; 0];
x(:,1) = system.x0;


%%
for i = 1:system.Nsim
    % Return optimal u for horizon
    optimal(i) = online_calc(problem, x(:,i));

    v(:,i) = optimal(i).v(:,1);
    z(:,i) = optimal(i).z(:,1);
    u(:,i) = v(:,i) + problem.system.K * ( x(:,i) - z(:,i));
    if ~(i == system.Nsim)
        x(:,i+1) = problem.system.A * x(:,i) + problem.system.B * u(:,i) ...
             + problem.system.E * problem.system.w_sequence(:,i);
    end
end


 %[X_tube,U_tube]=construct_tubes(z,v,system.S_K,system.K, system.Nsim);
  
% [alpha_array, N_array] = generate_N_alpha(problem);
% constraint_set = Polyhedron(constraints.C + constraints.D, constraints.e_org);
% t_constraint_set = Polyhedron(constraints.C + constraints.D, constraints.e);
% K_constraint_set = Polyhedron(constraints.C + constraints.D*system.K, constraints.e_org);
% K_t_constraint_set = Polyhedron(constraints.C + constraints.D*system.K, constraints.e);

%% PLOTTING


%Plot N and alpha (A_K^N W \in alpha W)
%generate_N_alpha;

%Plot predicticed tubes X, U, and stats x,z
figure; hold on;
color1 = [0.3 0 0.3];
color2 = [0 0.3 0.3];
cs=1.75;
%for i=1:system.N+1
%    plot(X_c(system.N+1-i+1),'Color',(1-i/(cs*(system.N+1)))*color2, 'alpha', 0.0125, 'LineStyle',':');    
%    plot(X(system.N+2-i),'Color',(1-i/(cs*(system.N+1)))*color1, 'alpha', 0.0125);
%end
%plot(X_tube,'Color', [0.3 0.2 1], 'alpha', 0.125);
z_legend = plot(z(1,:),z(2,:), 'r.--', 'LineWidth', 1.2);
x_legend = plot(x(1,:),x(2,:),'Color', [0.9 0.9 0.2], 'LineStyle', '--', 'LineWidth', 1.2);
e_legend = plot(x(1,:) - z(1,:),x(2,:) - z(2,:), 'c--', 'LineWidth', 1.2);
legend([z_legend,x_legend, e_legend],{'nominal state $\bar{x}$', 'system state $x$', 'error state $x - \bar{x}$'}, 'Interpreter', 'Latex')
%legend([z_legend,x_legend],{'nominal state $\bar{x}$', 'system state $x$'}, 'Interpreter', 'Latex')
%legend([S_K_leg e_legend],{'${S_K}_N(\alpha)$', 'error state $(x - \bar{x})$'}, 'Interpreter', 'Latex')
hold off;
print('simulation', '-depsc', '-r300') 


%% Plotting
t = 0:system.h:system.h*(system.Nsim-1);

figure(1)
subplot(811)
plot(t,v(1,:), 'm', t, u(1,:), 'b'),grid
ylabel('u_1')
subplot(812)
plot(t,v(2,:), 'm', t, u(2,:), 'b'),grid
ylabel('u_2')
subplot(813)
plot(t,x(1,:), 'm', t, z(1,:), 'b'),grid
ylabel('lambda')
subplot(814)
plot(t,x(2,:), 'm', t, z(2,:), 'b'),grid
ylabel('r')
subplot(815)
plot(t,x(3,:), 'm', t, z(3,:), 'b'),grid
ylabel('p')
subplot(816)
plot(t,x(4,:), 'm', t, z(4,:), 'b'),grid
xlabel('tid (s)'),ylabel('pdot')
subplot(817)
plot(t,x(5,:), 'm', t, z(5,:), 'b'),grid
xlabel('tid (s)'),ylabel('e')
subplot(818)
plot(t,x(6,:), 'm', t, z(6,:), 'b'),grid
xlabel('tid (s)'),ylabel('edot')
