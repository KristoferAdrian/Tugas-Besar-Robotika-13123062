function [robot_state, robot_params, pid_params, env_params, sim_params, path, phi, mask, seen_obstacles, custom_map] = config()

    working_area = 100;
    buffer = 50;
    grid_size = working_area + 2 * buffer;

    goal = [125, 125] + buffer;

    [Xobs, Yobs] = meshgrid(40:5:100, 50:5:100);
   obstacles = [Xobs(:), Yobs(:)];

    robot_pos = [-30, -30] + buffer;
    robot_theta = 0;

    robot_params.radius = 5;
    robot_params.width = 10;
    robot_params.mass = 10;
    robot_params.inertia = 0.3;
    robot_params.wheel_radius = 0.1;
    robot_params.wheel_base = robot_params.width;
    robot_params.max_omega = 20;
    robot_params.max_v = 1;
    robot_params.max_omega_robot = pi;
    robot_params.max_torque = 5;
    robot_params.motor_inertia = 0.01; 

    pid_params.Kp_lin = 10;
    pid_params.Ki_lin = 0;
    pid_params.Kd_lin = 2;
    pid_params.Kp_ang = 5;
    pid_params.Ki_ang = 0;
    pid_params.Kd_ang = 1.5;
    pid_params.tau_l_desired_log = [];
    pid_params.tau_r_desired_log = [];
    pid_params.tau_l_actual_log = [];
    pid_params.tau_r_actual_log = [];
    pid_params.time_log = [];
    pid_params.v_actual_log = [];
    pid_params.v_desired_log = [];
    
    pid_params.omega_actual_log = [];
    pid_params.omega_desired_log = [];

    pid_params.x_log = [];
    pid_params.y_log = [];
    pid_params.theta_log = [];
    
    pid_params.goal_error_log = [];

    env_params.working_area = working_area;
    env_params.buffer = buffer;
    env_params.grid_size = grid_size;
    env_params.goal = goal;
    env_params.obstacles = obstacles;
    env_params.proximity_radius = 30;
    env_params.obstacle_radius = 3;
    env_params.goal_threshold = 2;
    env_params.dt = 0.5;  

    sim_params.max_time = 10000;

    robot_state.x = robot_pos(1);
    robot_state.y = robot_pos(2);
    robot_state.theta = robot_theta;
    robot_state.v = 0;
    robot_state.omega = 0;
    robot_state.e_lin_prev = 0;
    robot_state.e_lin_int = 0;
    robot_state.e_ang_prev = 0;
    robot_state.e_ang_int = 0;

    robot_state.dv = 0;
    robot_state.domega = 0;

    path = [robot_pos];

    phi = 0.5 * ones(grid_size);
    mask = ones(grid_size);
    seen_obstacles = zeros(0, 2);  

    n_colors = 256;
    turbo_map = turbo(n_colors);
    gamma = 50.0;
    remap = linspace(0, 1, n_colors).^gamma;
    custom_map = interp1(linspace(0,1,n_colors), turbo_map, remap, 'linear');
end

