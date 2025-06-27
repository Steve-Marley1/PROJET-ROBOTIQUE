function defenseur()
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
    is_blue = strcmpi(robot_name, 'B2');

    % Zone de défense (moitié du terrain)
    if is_blue
        zone_x_min = 0.0;
        zone_x_max = 0.75;
        push_x = -0.75; % repousser vers le camp adverse (jaune)
    else
        zone_x_min = -0.75;
        zone_x_max = 0.0;
        push_x = 0.75; % repousser vers le camp adverse (bleu)
    end
    zone_y_min = -0.65;
    zone_y_max = 0.65;

    field_x_min = -0.75; field_x_max = 0.75;
    field_y_min = -0.65; field_y_max = 0.65;
    edge_buffer = 0.07;

    % Paramètres pour éviter d'entrer dans le but
    goal_area_x = is_blue * (field_x_max - edge_buffer - 0.02) + (~is_blue) * (field_x_min + edge_buffer + 0.02);
    goal_area_width = 0.14;
    goal_area_ymin = -goal_area_width/2;
    goal_area_ymax = goal_area_width/2;

    last_ball_x = 0; last_ball_y = 0;
    pred_factor = 0.14;

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

                % Prédiction du mouvement de la balle
                ball_vx = (ball_x - last_ball_x) / (TIME_STEP/1000);
                ball_vy = (ball_y - last_ball_y) / (TIME_STEP/1000);
                pred_ball_x = ball_x + pred_factor * ball_vx;
                pred_ball_y = ball_y + pred_factor * ball_vy;

                % Suivre la balle UNIQUEMENT dans la moitié de terrain du défenseur
                if pred_ball_x >= zone_x_min && pred_ball_x <= zone_x_max
                    target_x = pred_ball_x;
                    target_y = min(max(pred_ball_y, zone_y_min), zone_y_max);
                else
                    % Si la balle n'est pas dans la zone, rester sur la frontière, aligné sur y
                    if pred_ball_x < zone_x_min
                        target_x = zone_x_min;
                    else
                        target_x = zone_x_max;
                    end
                    target_y = min(max(pred_ball_y, zone_y_min), zone_y_max);
                end

                % === Evitement bords ===
                edge_avoidance = [0, 0];
                if my_x < field_x_min + edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) + 1.5;
                elseif my_x > field_x_max - edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) - 1.5;
                end
                if my_y < field_y_min + edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) + 1.5;
                elseif my_y > field_y_max - edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) - 1.5;
                end

                % === Evitement robots ===
                avoidance_vec = [0, 0];
                for i = 1:6
                    if i == self_index, continue; end
                    other_x = data(4*i - 3);
                    other_y = data(4*i - 2);
                    dist = norm([other_x - my_x, other_y - my_y]);
                    if dist < 0.07
                        avoid = [my_x - other_x, my_y - other_y] / (dist^2 + 0.01);
                        avoidance_vec = avoidance_vec + 0.7* avoid;
                    end
                end

                % === Evitement zone de but (ne jamais entrer dans le goal) ===
                goal_avoidance = [0, 0];
                if is_blue
                    % Goal bleu à droite
                    if my_x > field_x_max - edge_buffer - 0.02 && my_y > goal_area_ymin && my_y < goal_area_ymax
                        goal_avoidance(1) = goal_avoidance(1) - 2.5;
                    end
                else
                    % Goal jaune à gauche
                    if my_x < field_x_min + edge_buffer + 0.02 && my_y > goal_area_ymin && my_y < goal_area_ymax
                        goal_avoidance(1) = goal_avoidance(1) + 2.5;
                    end
                end

                vec_to_target = [target_x - my_x, target_y - my_y];
                distance = norm(vec_to_target);

                % Si la balle est proche et dans la zone, dégagement fort vers le camp adverse
                in_zone = (zone_x_min <= ball_x) && (ball_x <= zone_x_max);
                if in_zone && distance < 0.07
                    vec_push = [push_x - my_x, 0 - my_y];
                    combined_vec = vec_push + avoidance_vec + edge_avoidance + goal_avoidance;
                    angle_push = atan2(combined_vec(2), combined_vec(1));
                    angle_diff = atan2(sin(angle_push - my_theta), cos(angle_push - my_theta));
                    if abs(angle_diff) > pi/16
                        left_speed = -6.0 * sign(angle_diff);
                        right_speed = 6.0 * sign(angle_diff);
                    else
                        left_speed = 10.0;
                        right_speed = 10.0;
                    end
                else
                    % Suivi normal dans la zone
                    combined_vec = vec_to_target + avoidance_vec + edge_avoidance + goal_avoidance;
                    angle_to_target = atan2(combined_vec(2), combined_vec(1));
                    angle_diff = atan2(sin(angle_to_target - my_theta), cos(angle_to_target - my_theta));
                    if abs(angle_diff) > pi/14
                        left_speed = -7.0 * sign(angle_diff);
                        right_speed = 7.0 * sign(angle_diff);
                    else
                        v = 7.0 + 2.0 * min(distance/0.3, 1.0);
                        left_speed = v;
                        right_speed = v;
                    end
                end

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