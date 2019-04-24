% close all;

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
set(0,'defaultTextInterpreter','latex'); %trying to set the default
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


%% PLOTTING


t = 0:system.h:system.h*(system.Nsim-1);

figure(1)
subplot(811)
plot(t,u(1,:), 'm', t, v(1,:), 'b'),grid
ylabel('u_1')
legend('input with feedback $u_1$', 'nominal input $v_1$');
subplot(812)
plot(t,u(2,:), 'm', t, v(2,:), 'b'),grid
ylabel('u_2')
legend('input with feedback $u_1$','nominal input $v_1$');
subplot(813)
plot(t,x(1,:), 'm', t, z(1,:), 'b'),grid
ylabel('travel $\lambda$')
legend('system state $x_1$', 'nominal state $z_1$');
subplot(814)
plot(t,x(2,:), 'm', t, z(2,:), 'b'),grid
ylabel('travel rate $r$')
legend('system state $x_2$', 'nominal state $z_2$');
subplot(815)
plot(t,x(3,:), 'm', t, z(3,:), 'b'),grid
ylabel('pitch $p$')
legend('system state $x_3$', 'nominal state $z_3$');
subplot(816)
plot(t,x(4,:), 'm', t, z(4,:), 'b'),grid
ylabel('pitch rate $\dot{p}$')
legend('system state $x_4$', 'nominal state $z_4$');
subplot(817)
plot(t,x(5,:), 'm', t, z(5,:), 'b'),grid
ylabel('elevation $e$')
legend('system state $x_5$', 'nominal state $z_5$');
subplot(818)
plot(t,x(6,:), 'm', t, z(6,:), 'b'),grid
legend('system state $x_6$', 'nominal state $z_6$');
xlabel('tid (s)'),ylabel('elevation rate $\dot e$')

