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
    robot_names = ["b1", "b2", "b3", "y1", "y2", "y3"];
    self_index = find(strcmpi(robot_names, robot_name));
    is_blue = startsWith(robot_name, 'b');

    % === ZONE DE DÉFENSE (adapte les bornes si besoin) ===
    if ~is_blue
        zone_x_min = -0.7; zone_x_max = 0;
        gardien_index = find(strcmpi(robot_names, 'b3'));
        push_direction = 1;
    else
        zone_x_min = 0; zone_x_max = 0.7;
        gardien_index = find(strcmpi(robot_names, 'y3'));
        push_direction = -1;
    end
    zone_y_min = -0.45; zone_y_max = 0.45;

    field_x_min = -0.7; field_x_max = 0.7;
    field_y_min = -0.65; field_y_max = 0.65;
    edge_buffer = 0.05;

    last_ball_x = 0; last_ball_y = 0;
    pred_factor = 0.13;

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

                % Prédiction simple du mouvement de la balle
                ball_vx = (ball_x - last_ball_x) / (TIME_STEP/1000);
                ball_vy = (ball_y - last_ball_y) / (TIME_STEP/1000);
                pred_ball_x = ball_x + pred_factor * ball_vx;
                pred_ball_y = ball_y + pred_factor * ball_vy;

                % Clamp la cible à la zone
                target_x = min(max(pred_ball_x, zone_x_min), zone_x_max);
                target_y = min(max(pred_ball_y, zone_y_min), zone_y_max);

                % Évitement robots
                avoidance_vec = [0, 0];
                for i = 1:6
                    if i == self_index, continue; end
                    other_x = data(4*i - 3);
                    other_y = data(4*i - 2);
                    dist = norm([other_x - my_x, other_y - my_y]);
                    if dist < 0.16
                        avoid = [my_x - other_x, my_y - other_y] / (dist^2 + 0.01);
                        if i == gardien_index
                            avoid = 0.5 * avoid;
                        end
                        avoidance_vec = avoidance_vec + avoid;
                    end
                end

                % Évitement bords
                edge_avoidance = [0, 0];
                if my_x < field_x_min + edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) + 1;
                elseif my_x > field_x_max - edge_buffer
                    edge_avoidance(1) = edge_avoidance(1) - 1;
                end
                if my_y < field_y_min + edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) + 1;
                elseif my_y > field_y_max - edge_buffer
                    edge_avoidance(2) = edge_avoidance(2) - 1;
                end

                % Comportement balle dans zone
                in_zone = (zone_x_min <= ball_x) && (ball_x <= zone_x_max) && ...
                          (zone_y_min <= ball_y) && (ball_y <= zone_y_max);

                vec_to_target = [target_x - my_x, target_y - my_y];
                distance = norm(vec_to_target);

                angle_to_target = atan2(vec_to_target(2), vec_to_target(1));
                angle_diff = atan2(sin(angle_to_target - my_theta), cos(angle_to_target - my_theta));
                angle_thresh = pi/12; % ~15°

                if in_zone && distance < 0.14
                    % Dégagement fort vers l'extérieur de la zone
                    push_x = zone_x_max + push_direction * 0.3;
                    push_y = 0;
                    vec_push = [push_x - my_x, push_y - my_y];
                    angle_push = atan2(vec_push(2), vec_push(1));
                    angle_push_diff = atan2(sin(angle_push - my_theta), cos(angle_push - my_theta));
                    if abs(angle_push_diff) > angle_thresh
                        % Tourne sur place pour viser la sortie
                        left_speed = -6.0 * sign(angle_push_diff);
                        right_speed = 6.0 * sign(angle_push_diff);
                    else
                        % Va tout droit, vitesse max
                        combined_vec = vec_push + 0.3 * avoidance_vec + 0.3 * edge_avoidance;
                        base_speed = 10.0;
                        left_speed = base_speed;
                        right_speed = base_speed;
                    end
                elseif in_zone
                    % Suivi précis : tourne d'abord, avance ensuite
                    if abs(angle_diff) > angle_thresh
                        left_speed = -5.5 * sign(angle_diff);
                        right_speed = 5.5 * sign(angle_diff);
                    else
                        % Ralentit à l'approche
                        v = 6.0 + 2.5 * min(distance/0.3, 1.0);
                        combined_vec = vec_to_target + 0.3 * avoidance_vec + 0.3 * edge_avoidance;
                        left_speed = v;
                        right_speed = v;
                    end
                else
                    % Hors zone : reste sur la frontière (y aligné balle), pas de rotation inutile
                    if abs(angle_diff) > angle_thresh
                        left_speed = -5.0 * sign(angle_diff);
                        right_speed = 5.0 * sign(angle_diff);
                    else
                        combined_vec = vec_to_target + 0.3 * avoidance_vec + 0.3 * edge_avoidance;
                        base_speed = 5.0;
                        left_speed = base_speed;
                        right_speed = base_speed;
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