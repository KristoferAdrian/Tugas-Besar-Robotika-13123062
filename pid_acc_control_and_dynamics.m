function [new_state, tau_r, tau_l, tau_r_desired, tau_l_desired] = pid_acc_control_and_dynamics(...
    desired_dir, robot_state, robot_params, pid_params, dt)

x = robot_state.x;
y = robot_state.y;
theta = robot_state.theta;
v = robot_state.v;
omega = robot_state.omega;
dv = robot_state.dv;
domega = robot_state.domega;

v_des = 0;
omega_des = 0;
if norm(desired_dir) > 0
    angle_des = atan2(desired_dir(2), desired_dir(1));
    angle_error = wrapToPi(angle_des - theta);

    v_des = robot_params.max_v * (1 - abs(angle_error) / pi);
    v_des = max(v_des, 0.05);  

    omega_des = robot_params.max_omega_robot * angle_error / pi;
end

e_v = v_des - v;
e_dv = -dv;

e_omega = omega_des - omega;
e_domega = -domega;

robot_state.e_lin_int = robot_state.e_lin_int + e_v * dt;
robot_state.e_ang_int = robot_state.e_ang_int + e_omega * dt;

acc_cmd = pid_params.Kp_lin * e_v + ...
          pid_params.Ki_lin * robot_state.e_lin_int + ...
          pid_params.Kd_lin * e_dv;

alpha_cmd = pid_params.Kp_ang * e_omega + ...
            pid_params.Ki_ang * robot_state.e_ang_int + ...
            pid_params.Kd_ang * e_domega;

m = robot_params.mass;
I = robot_params.inertia;
F = m * acc_cmd;
tau_body = I * alpha_cmd;

r = robot_params.wheel_radius;
L = robot_params.wheel_base;

tau_r = (F * r + tau_body * r / L);
tau_l = (F * r - tau_body * r / L);
tau_r_desired = tau_r;
tau_l_desired = tau_l;

max_tau = robot_params.max_torque;
max_actual = max(abs([tau_r, tau_l]));
if max_actual > max_tau
    scale = max_tau / max_actual;
    tau_r = tau_r * scale;
    tau_l = tau_l * scale;
end

F_applied = (tau_r + tau_l) / r;
tau_applied = (tau_r - tau_l) * L / (2 * r);

dv = F_applied / m;
domega = tau_applied / I;

v = v + dv * dt;
omega = omega + domega * dt;

x = x + v * cos(theta) * dt;
y = y + v * sin(theta) * dt;
theta = wrapToPi(theta + omega * dt);

new_state = robot_state;
new_state.x = x;
new_state.y = y;
new_state.theta = theta;
new_state.v = v;
new_state.omega = omega;
new_state.dv = dv;
new_state.domega = domega;

new_state.e_lin_int = robot_state.e_lin_int;
new_state.e_ang_int = robot_state.e_ang_int;
end
