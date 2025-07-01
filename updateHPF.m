function [phi, mask, seen_obstacles, newly_detected] = updateHPF(robot_state, obstacles, seen_obstacles, phi, mask, grid_size, goal, obstacle_radius, proximity_radius)

    newly_detected = false;

    rx = round(robot_state.x);
    ry = round(robot_state.y);

    rx = min(max(rx,1),grid_size);
    ry = min(max(ry,1),grid_size);

    dists = sqrt((obstacles(:,1) - robot_state.x).^2 + (obstacles(:,2) - robot_state.y).^2);
    prox_obstacles = obstacles(dists <= proximity_radius, :);

    for i = 1:size(prox_obstacles,1)
        obs = prox_obstacles(i, :);
        if ~ismember(obs, seen_obstacles, 'rows')
            seen_obstacles = [seen_obstacles; obs];
            newly_detected = true;
        end
    end

    prev_mask = mask;

    phi(:) = 0.5;
    mask(:) = 1;

    goal_idx = round(goal);
    phi(goal_idx(2), goal_idx(1)) = -10000;
    mask(goal_idx(2), goal_idx(1)) = 0;

    for i = 1:size(seen_obstacles, 1)
        obs = seen_obstacles(i, :);
        [Xgrid, Ygrid] = meshgrid(1:grid_size, 1:grid_size);
        dist_grid = sqrt((Xgrid - obs(1)).^2 + (Ygrid - obs(2)).^2);

        affected_cells = dist_grid <= obstacle_radius;
        phi(affected_cells) = 1000;
        mask(affected_cells) = 0;
    end

    wall_potential = 1000;
    phi(1, :)       = wall_potential;
    phi(end, :)     = wall_potential;
    phi(:, 1)       = wall_potential;
    phi(:, end)     = wall_potential;
    mask(1, :)      = 0;
    mask(end, :)    = 0;
    mask(:, 1)      = 0;
    mask(:, end)    = 0;

    if any(prev_mask(:) ~= mask(:))
        newly_detected = true;
    end

    max_iters = 5000;
    convergence_threshold = 1e-6;
    
    kernel = [0 1 0; 1 0 1; 0 1 0] / 4;
    
    for iter = 1:max_iters
        phi_old = phi;
    
        laplace = conv2(phi_old, kernel, 'same');
    
        phi(mask == 1) = laplace(mask == 1);
    
        if max(abs(phi(:) - phi_old(:))) < convergence_threshold
            break;
        end
    end
end
