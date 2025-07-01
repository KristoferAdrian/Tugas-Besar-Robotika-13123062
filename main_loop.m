clc; clear; close all;

[robot_state, robot_params, pid_params, env_params, sim_params, path, phi, mask, seen_obstacles, custom_map] = config();

mode = 'apf';
bug_state = 'approach';
follow_side = 'left';
position_history = [];

figure; hold on;
h_phi = imagesc(phi);
set(h_phi, 'AlphaData', 0.6);
axis equal tight xy;
set(gca, 'YDir', 'normal');
colormap(custom_map);
contour(phi, [950 950], 'r');
plot(env_params.goal(1), env_params.goal(2), 'g*', 'MarkerSize', 10);
robot_marker = rectangle('Position', ...
    [robot_state.x - robot_params.radius, ...
     robot_state.y - robot_params.radius, ...
     2*robot_params.radius, 2*robot_params.radius], ...
    'Curvature', [1, 1], ...
    'FaceColor', 'g', ...
    'EdgeColor', 'k');
heading = quiver(robot_state.x, robot_state.y, ...
    cos(robot_state.theta) * robot_params.radius, ...
    sin(robot_state.theta) * robot_params.radius, ...
    'Color', 'r', 'LineWidth', 2);
path_plot = plot(path(:,1), path(:,2), 'b--');
title('Robot Navigation with APF+Bug and Dynamics');

time = 0;
while norm([robot_state.x, robot_state.y] - env_params.goal) > env_params.goal_threshold && time < sim_params.max_time

    [phi, mask, seen_obstacles, newly_detected] = updateHPF(robot_state, env_params.obstacles, seen_obstacles, phi, mask, env_params.grid_size, env_params.goal, env_params.obstacle_radius, env_params.proximity_radius);

    if newly_detected
        set(h_phi, 'CData', phi);
        drawnow;
    end

    [desired_direction, mode, bug_state, follow_side, position_history] = ...
        apf_bug_navigation([robot_state.x, robot_state.y], phi, mask, env_params.goal, ...
        struct('step_size',1, 'displacement_window',10, 'stagnation_threshold',1.5, 'flat_gradient',0.5), ...
        mode, bug_state, follow_side, position_history);

    [robot_state, tau_r, tau_l, tau_r_desired, tau_l_desired] = ...
        pid_acc_control_and_dynamics(desired_direction, robot_state, robot_params, pid_params, env_params.dt);

    e_goal = norm([robot_state.x, robot_state.y] - env_params.goal);
    
    angle_des = atan2(desired_direction(2), desired_direction(1));
    angle_error = wrapToPi(angle_des - robot_state.theta);
    v_des = robot_params.max_v * cos(angle_error);
    if v_des < 0
        v_des = 0;
    end
    omega_des = robot_params.max_omega_robot * angle_error / pi;
    
    pid_params.tau_r_desired_log(end+1) = tau_r_desired;
    pid_params.tau_l_desired_log(end+1) = tau_l_desired;
    pid_params.tau_r_actual_log(end+1) = tau_r;
    pid_params.tau_l_actual_log(end+1) = tau_l;

    pid_params.time_log(end+1) = time;    

    pid_params.v_actual_log(end+1) = robot_state.v;
    pid_params.v_desired_log(end+1) = v_des;
    
    pid_params.omega_actual_log(end+1) = robot_state.omega;
    pid_params.omega_desired_log(end+1) = omega_des;

    pid_params.x_log(end+1) = robot_state.x;
    pid_params.y_log(end+1) = robot_state.y;
    pid_params.theta_log(end+1) = robot_state.theta;
    
    pid_params.goal_error_log(end+1) = e_goal;

    path = [path; [robot_state.x, robot_state.y]];

    set(robot_marker, 'Position', ...
        [robot_state.x - robot_params.radius, ...
         robot_state.y - robot_params.radius, ...
         2*robot_params.radius, 2*robot_params.radius]);
    set(heading, 'XData', robot_state.x, ...
             'YData', robot_state.y, ...
             'UData', cos(robot_state.theta) * robot_params.radius, ...
             'VData', sin(robot_state.theta) * robot_params.radius);
    set(path_plot, 'XData', path(:,1), 'YData', path(:,2));
    plot(robot_state.x, robot_state.y, 'b.', 'MarkerSize', 2);
    drawnow;
    title(sprintf('Robot Navigation — Sim Time: %.2fs', time));

    idx = round([robot_state.x, robot_state.y]);
    idx(1) = max(min(idx(1), env_params.grid_size),1);
    idx(2) = max(min(idx(2), env_params.grid_size),1);
    if mask(idx(2), idx(1)) == 0 && phi(idx(2), idx(1)) ~= -10000
        warning('Collision detected - stopping');
        break;
    end

    time = time + env_params.dt;
end

plot(robot_state.x, robot_state.y, 'mo', 'MarkerSize', 10, 'LineWidth', 2);
legend('Potential', 'Goal', 'Obstacles', 'Robot', 'Path', 'Final Position', 'Location', 'northeast');

figure('Name', 'Control Performance and Stability', 'NumberTitle', 'off');

subplot(3,3,1);
plot(pid_params.time_log, pid_params.v_desired_log, 'g--', ...
     pid_params.time_log, pid_params.v_actual_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('v_{desired} vs v_{actual}'); legend('Desired','Actual'); grid on;

subplot(3,3,2);
plot(pid_params.time_log, pid_params.omega_desired_log, 'g--', ...
     pid_params.time_log, pid_params.omega_actual_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('\omega_{desired} vs \omega_{actual}'); legend('Desired','Actual'); grid on;

subplot(3,3,3);
plot(pid_params.x_log, pid_params.y_log, 'b-', 'LineWidth', 1); hold on;
plot(env_params.goal(1), env_params.goal(2), 'g*', 'MarkerSize', 10);
xlabel('x'); ylabel('y'); axis equal;
title('Robot Path'); legend('Path', 'Goal'); grid on;

subplot(3,3,4);
plot(pid_params.time_log, pid_params.theta_log, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Heading Over Time'); grid on;

subplot(3,3,5);
plot(pid_params.time_log, pid_params.goal_error_log, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('‖e‖ to goal');
title('Distance to Goal Over Time'); grid on;

subplot(3,3,6);
plot(pid_params.time_log, pid_params.tau_r_desired_log, 'g--', ...
     pid_params.time_log, pid_params.tau_r_actual_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('\tau_{r} Desired vs Actual'); legend('Desired','Actual'); grid on;

subplot(3,3,7);
plot( pid_params.time_log, pid_params.tau_l_desired_log, 'g--', ...
     pid_params.time_log, pid_params.tau_l_actual_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('\tau_{l} Desired vs Actual'); legend('Desired','Actual'); grid on;