function attaquant()
    TIME_STEP = 64;
    wb_robot_init();

    receiver = wb_robot_get_device('receiver');
    left_motor = wb_robot_get_device('left wheel motor');
    right_motor = wb_robot_get_device('right wheel motor');

    wb_receiver_enable(receiver, TIME_STEP);
    wb_motor_set_position(left_motor, inf);
    wb_motor_set_position(right_motor, inf);

    robot_name = wb_robot_get_name();
    robot_names = ["B1", "B2", "B3", "Y1", "Y2", "Y3"];
    self_index = find(strcmpi(robot_names, robot_name));
    is_blue = strcmpi(robot_name(1), 'B');

    % Demi-terrain adverse STRICT (corrigé selon ta remarque)
    if is_blue
        zone_x_min = -0.75 + 0.01; % zone adverse pour bleu : x<0
        zone_x_max = 0.0 - 0.01;
        goal_x = -0.75;
    else
        zone_x_min = 0.0 + 0.01;   % zone adverse pour jaune : x>0
        zone_x_max = 0.75 - 0.01;
        goal_x = 0.75;
    end
    zone_y_min = -0.65 + 0.01;
    zone_y_max = 0.65 - 0.01;

    field_x_min = -0.75; field_x_max = 0.75;
    field_y_min = -0.65; field_y_max = 0.65;
    edge_buffer = 0.07;
    goal_width = 0.14;
    goal_area_x_min = goal_x - 0.04;
    goal_area_x_max = goal_x + 0.04;
    goal_area_ymin = -goal_width/2;
    goal_area_ymax = goal_width/2;

    last_ball_x = 0; last_ball_y = 0;

    while wb_robot_step(TIME_STEP) ~= -1
        if wb_receiver_get_queue_length(receiver) > 0
            data = wb_receiver_get_data(receiver, 'double');
            wb_receiver_next_packet(receiver);

            if length(data) == 27
                my_x = data(4*self_index - 3);
                my_y = data(4*self_index - 2);
                my_theta = data(4*self_index);

                ball_x = data(25);
                ball_y = data(26);

                % Prédiction mouvement de la balle
                ball_vx = (ball_x - last_ball_x) / (TIME_STEP/1000);
                ball_vy = (ball_y - last_ball_y) / (TIME_STEP/1000);
                pred_ball_x = ball_x + 0.15 * ball_vx;
                pred_ball_y = ball_y + 0.15 * ball_vy;

                % Propre position cible clampée à la zone adverse
                target_x = min(max(pred_ball_x, zone_x_min), zone_x_max);
                target_y = min(max(pred_ball_y, zone_y_min), zone_y_max);

                % Si la balle n'est pas dans la zone adverse, attendre sur la frontière adverse aligné sur y
                if pred_ball_x < zone_x_min
                    target_x = zone_x_min;
                elseif pred_ball_x > zone_x_max
                    target_x = zone_x_max;
                end
                if pred_ball_y < zone_y_min
                    target_y = zone_y_min;
                elseif pred_ball_y > zone_y_max
                    target_y = zone_y_max;
                end

                % Ne jamais rentrer dans le but adverse
                if is_blue && my_x < goal_area_x_max && my_x > goal_area_x_min && my_y > goal_area_ymin && my_y < goal_area_ymax
                    target_x = goal_area_x_max + 0.04;
                elseif ~is_blue && my_x > goal_area_x_min && my_x < goal_area_x_max && my_y > goal_area_ymin && my_y < goal_area_ymax
                    target_x = goal_area_x_min - 0.04;
                end

                % Évitement bords (dans la zone adverse uniquement)
                edge_avoidance = [0, 0];
                if my_x < zone_x_min + edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) + 1.5;
                elseif my_x > zone_x_max - edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) - 1.5;
                end
                if my_y < zone_y_min + edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) + 1.5;
                elseif my_y > zone_y_max - edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) - 1.5;
                end

                % Évitement joueurs
                avoidance_vec = [0, 0];
                for i = 1:6
                    if i == self_index, continue; end
                    other_x = data(4*i - 3);
                    other_y = data(4*i - 2);
                    dist = norm([other_x - my_x, other_y - my_y]);
                    if dist < 0.13
                        avoid = [my_x - other_x, my_y - other_y] / (dist^2 + 0.01);
                        avoidance_vec = avoidance_vec + 0.7 * avoid;
                    end
                end

                vec_to_target = [target_x - my_x, target_y - my_y];
                distance = norm(vec_to_target);

                % Si la balle est proche dans la zone adverse : viser l'entrée des buts (centre ou extrémités)
                in_zone = (zone_x_min <= ball_x) && (ball_x <= zone_x_max) && (zone_y_min <= ball_y) && (ball_y <= zone_y_max);
                base_speed = 8.0;

                if in_zone && distance < 0.07
                    % Cherche la trajectoire la plus proche pour le but (centre ou extrémités)
                    y_goal_top = goal_width/2;
                    y_goal_bot = -goal_width/2;
                    d_centre = abs(my_y - 0);
                    d_top = abs(my_y - y_goal_top);
                    d_bot = abs(my_y - y_goal_bot);

                    if d_centre <= d_top && d_centre <= d_bot
                        shoot_target = [goal_x, 0];
                    elseif d_top < d_bot
                        shoot_target = [goal_x, y_goal_top-0.02];
                    else
                        shoot_target = [goal_x, y_goal_bot+0.02];
                    end

                    % Ne vise jamais dans le but adverse
                    shoot_x = shoot_target(1);
                    shoot_y = shoot_target(2);
                    if is_blue && shoot_x < goal_area_x_max
                        shoot_x = goal_area_x_max;
                    elseif ~is_blue && shoot_x > goal_area_x_min
                        shoot_x = goal_area_x_min;
                    end

                    vec_shoot = [shoot_x - my_x, shoot_y - my_y];
                    combined_vec = vec_shoot + avoidance_vec + edge_avoidance;
                    base_speed = 10.0;
                else
                    combined_vec = vec_to_target + avoidance_vec + edge_avoidance;
                    if distance < 0.1
                        base_speed = 6.0;
                    end
                end

                angle_to_target = atan2(combined_vec(2), combined_vec(1));
                angle_diff = atan2(sin(angle_to_target - my_theta), cos(angle_to_target - my_theta));
                Kp = 3.2;
                left_speed = base_speed - Kp * angle_diff;
                right_speed = base_speed + Kp * angle_diff;

                % Saturation
                max_speed = 10.0;
                left_speed = max(min(left_speed, max_speed), -max_speed);
                right_speed = max(min(right_speed, max_speed), -max_speed);

                wb_motor_set_velocity(left_motor, left_speed);
                wb_motor_set_velocity(right_motor, right_speed);

                last_ball_x = ball_x;
                last_ball_y = ball_y;
            end
        end
    end

    wb_robot_cleanup();
end