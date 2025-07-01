function [grad_direction, mode, bug_state, follow_side, position_history] = ...
    apf_bug_navigation(robot_pos, phi, mask, goal, path_params, ...
    mode, bug_state, follow_side, position_history)

    step_size = path_params.step_size;
    displacement_window = path_params.displacement_window;
    stagnation_threshold = path_params.stagnation_threshold;
    flat_gradient = path_params.flat_gradient;
    
    grid_size = size(phi,1);
    
    x = round(robot_pos(1));
    y = round(robot_pos(2));
    
    if x <= 2 || x >= grid_size-1 || y <= 2 || y >= grid_size-1
        warning('Robot out of bounds');
        grad_direction = [0 0];
        return;
    end

    dphidx = (phi(y, x+1) - phi(y, x-1)) / 2;
    dphidy = (phi(y+1, x) - phi(y-1, x)) / 2;
    grad_phi = [dphidx, dphidy];
    grad_mag = norm(grad_phi);

    position_history(end+1, :) = robot_pos;
    if size(position_history, 1) > displacement_window
        position_history(1,:) = [];
    end
    net_displacement = norm(position_history(end,:) - position_history(1,:));
    
    if strcmp(mode, 'apf') && net_displacement < stagnation_threshold && grad_mag < flat_gradient
        disp('Stuck in local minimum — switching to BUG mode');
        mode = 'bug';
        bug_state = 'approach';

        to_goal = goal - robot_pos;
        to_goal = to_goal / (norm(to_goal) + eps);
        
        scan_radius = 4;
        xc = round(robot_pos(1));
        yc = round(robot_pos(2));
        local_phi = phi(max(yc-scan_radius,1):min(yc+scan_radius,grid_size), ...
                        max(xc-scan_radius,1):min(xc+scan_radius,grid_size));
        [obs_y, obs_x] = find(local_phi > 950);
        
        if isempty(obs_x)
            follow_side = 'right';  
        else
            rel_x = obs_x - (xc - max(xc-scan_radius,1) + 1);
            rel_y = obs_y - (yc - max(yc-scan_radius,1) + 1);
            rel_coords = [rel_x, rel_y];
            [~, idx] = min(vecnorm(rel_coords, 2, 2));
            nearest = rel_coords(idx, :);
        
            obs_vec = nearest / (norm(nearest) + eps);
        
            z = obs_vec(1)*to_goal(2) - obs_vec(2)*to_goal(1);
            if z > 0
                follow_side = 'left';
            else
                follow_side = 'right';
            end
        end

        disp(['Wall-following side: ', follow_side]);
    end

    to_goal = goal - robot_pos;
    to_goal = to_goal / (norm(to_goal) + eps);
    alignment = dot(-grad_phi / (grad_mag + eps), to_goal);
    if strcmp(mode, 'bug') && grad_mag > 0.01 && alignment > 0.3 && net_displacement > 1.5
        disp('HPF path found');
        mode = 'apf';
        bug_state = 'approach';
        position_history = [];
    end
    
    if strcmp(mode, 'apf')
        if grad_mag < 1e-5
            grad_direction = rand(1,2) - 0.5;
            grad_direction = grad_direction / norm(grad_direction);
        else
            grad_direction = -grad_phi / grad_mag;
        end
    elseif strcmp(mode, 'bug')
        switch bug_state
            case 'approach'
                to_goal = goal - robot_pos;
                dir = to_goal / (norm(to_goal) + eps);
                step_try = step_size;
                min_step = 0.1;
                moved = false;
                while step_try >= min_step
                    test_pos = robot_pos + step_try * dir;
                    idx = round(test_pos);
                    if idx(2)<1 || idx(2)>grid_size || idx(1)<1 || idx(1)>grid_size
                        step_try = step_try / 2;
                        continue;
                    end
                    if mask(idx(2), idx(1)) == 1 && phi(idx(2), idx(1)) < 950
                        grad_direction = dir;
                        moved = true;
                        break;
                    else
                        step_try = step_try / 2;
                    end
                end
                if ~moved
                    disp('Switching to wall-following');
                    bug_state = 'follow';
                    grad_direction = [0, 0];
                end
            case 'follow'
                scan_radius = 4;
                x_center = round(robot_pos(1));
                y_center = round(robot_pos(2));
                local_phi = phi(max(y_center - scan_radius, 1):min(y_center + scan_radius, grid_size), ...
                                max(x_center - scan_radius, 1):min(x_center + scan_radius, grid_size));
                [obs_y, obs_x] = find(local_phi > 950);
                found_safe_direction = false;
                if ~isempty(obs_x)
                    rel_x = obs_x - (x_center - max(x_center - scan_radius, 1) + 1);
                    rel_y = obs_y - (y_center - max(y_center - scan_radius, 1) + 1);
                    rel_coords = [rel_x, rel_y];
                    dists = vecnorm(rel_coords, 2, 2);
                    [~, idx] = min(dists);
                    nearest = rel_coords(idx, :);
                    normal = -nearest / norm(nearest);
                    tangent = [-normal(2), normal(1)]; 
                    test_pos = robot_pos + step_size * tangent;
                    test_idx = round(test_pos);
                    test_idx = max(min(test_idx, grid_size - 1), 2);
                    if mask(test_idx(2), test_idx(1)) == 1 && phi(test_idx(2), test_idx(1)) < 950
                        grad_direction = tangent;
                        found_safe_direction = true;
                    end
                end
                if ~found_safe_direction
                    grad_direction = -grad_phi / (grad_mag + eps);
                end
        end
    end

    if norm(grad_direction) > 0
        grad_direction = grad_direction / norm(grad_direction);
    else
        grad_direction = [0 0];
    end
end
