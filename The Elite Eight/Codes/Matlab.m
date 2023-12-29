clear; clc;

t_span = [0 10];
q_initial = [0.1; 0.1];
qdot_initial = [0; 0];
x_initial = [0; 0];

q_desired = [0; 0];

[t, states] = ode45(@compute_dynamics, t_span, [q_initial; qdot_initial; x_initial]);
q1 = states(:,1);
q2 = states(:,2);
q1_dot = states(:,3);
q2_dot = states(:,4);
x1 = states(:,5);
x2 = states(:,6);

e1 = q_desired(1) - q1;
e2 = q_desired(2) - q2;

figure; subplot(2,1,1);
plot(t,q1);
title("q_1");
xlabel("t");
ylabel("q_1(t)");
subplot(2,1,2);
plot(t,q2);
title("q_2");
xlabel("t");
ylabel("q_2(t)");

figure; subplot(2,1,1);
plot(t,e1);
title("Error in q_1");
xlabel("t");
ylabel("e_1(t)");
subplot(2,1,2);
plot(t,e2);
title("Error in q_2");
xlabel("t");
ylabel("e_2(t)");

% q1_dd = - (5*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) - (5*(2*cos(q2) + 1)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);
% q2_dd = (5*(2*cos(q2) + 1)*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) + (5*(4*cos(q2) + 13)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);

function dxdt = compute_dynamics(t, states)
    q1 = states(1);
    q2 = states(2);
    q1_dot = states(3);
    q2_dot = states(4);
    x1 = states(5);
    x2 = states(6);
    
    Kp1 = 200; Kd1 = 150; Ki1 = 100;
    Kp2 = 200; Kd2 = 150; Ki2 = 100;
    
    q_desired = [0; 0];

    e1 = q_desired(1) - q1;
    e2 = q_desired(2) - q2;

    f1 = Kp1*e1 + Ki1*x1 - Kd1*q1_dot;
    f2 = Kp2*e2 + Ki2*x2 - Kd2*q2_dot;
    F = [f1; f2];

    m1 = 10; m2 = 5;
    l1 = 0.2; l2 = 0.1;

    M11 = (m1+m2)*(l1^2) + m2*l2*(l2+2*l1*cos(q2));
    M12 = m2*l2*(l2+l1*cos(q2));
    M22 = m2*(l2^2);
    M = [M11, M12; M12, M22];

    tau = M*F;
    tau1 = tau(1);
    tau2 = tau(2);

    dxdt = zeros(size(states));
    dxdt(1) = q1_dot;
    dxdt(2) = q2_dot;
    dxdt(3) = - (5*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) - (5*(2*cos(q2) + 1)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);
    dxdt(4) = (5*(2*cos(q2) + 1)*(tau1 - (981*cos(q1 + q2))/200 - (2943*cos(q1))/100 + (q1_dot*q2_dot*sin(q2))/10 + (q2_dot*sin(q2)*(q1_dot + q2_dot))/10))/(cos(q2)^2 - 3) + (5*(4*cos(q2) + 13)*((sin(q2)*q2_dot^2)/10 - tau2 + (981*cos(q1 + q2))/200))/(cos(q2)^2 - 3);
    dxdt(5) = q_desired(1) - q1;
    dxdt(6) = q_desired(2) - q2;
end