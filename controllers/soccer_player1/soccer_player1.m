function soccer_player1()
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
 
    is_blue = startsWith(robot_name, 'B');
    goal_x = ~is_blue * (-0.75) + is_blue * (0.75);
 
    field_x_min = -0.75;
    field_x_max = 0.75;
    field_y_min = -0.65;
    field_y_max = 0.65;
    edge_buffer = 0.07;
 
    shoot_mode = false;
    shoot_start_pos = 0;
    shoot_start_time = 0;
 
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
                vec_to_ball = [ball_x - my_x, ball_y - my_y];
                distance_to_ball = norm(vec_to_ball);
 
                % === TIR ===
                distance_to_goal = abs(goal_x - my_x);
                in_shoot_zone = distance_to_goal < 0.5 && distance_to_ball < 0.1;
 
                % === Evitement robots ===
                avoidance_vec = [0, 0];
                for i = 1:6
                    if i == self_index, continue; end
                    other_x = data(4*i - 3);
                    other_y = data(4*i - 2);
                    dist = norm([other_x - my_x, other_y - my_y]);
                    if dist < 0.15
                        avoid = [my_x - other_x, my_y - other_y] / (dist^2 + 0.01);
                        avoidance_vec = avoidance_vec + avoid;
                    end
                end
 
                % === Evitement bords ===
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
 
                if shoot_mode
                    shoot_distance = abs(my_x - shoot_start_pos);
                    if shoot_distance >= 0.16
                        wb_motor_set_velocity(left_motor, 0);
                        wb_motor_set_velocity(right_motor, 0);
                        if shoot_start_time == 0
                            shoot_start_time = wb_robot_get_time();
                        elseif wb_robot_get_time() - shoot_start_time > 1.5
                            shoot_mode = false;
                            shoot_start_time = 0;
                        end
                        continue;
                    else
                        shoot_speed = 10.0;
                        wb_motor_set_velocity(left_motor, shoot_speed);
                        wb_motor_set_velocity(right_motor, shoot_speed);
                        continue;
                    end
                end
 
                if in_shoot_zone
                    shoot_mode = true;
                    shoot_start_pos = my_x;
                    continue;
                end
 
                % === Déplacement normal ===
                combined_vec = vec_to_ball + 0.8 * avoidance_vec + 1.2 * edge_avoidance;
                angle_to_target = atan2(combined_vec(2), combined_vec(1));
                angle_diff = atan2(sin(angle_to_target - my_theta), cos(angle_to_target - my_theta));
 
                % Réduction de vitesse proche de la balle
                if distance_to_ball < 0.1
                    base_speed = 3.0;
                else
                    base_speed = 6.0;
                end
 
                Kp = 3.0;
                left_speed = base_speed - Kp * angle_diff;
                right_speed = base_speed + Kp * angle_diff;
 
                max_speed = 10.0;
                left_speed = max(min(left_speed, max_speed), -max_speed);
                right_speed = max(min(right_speed, max_speed), -max_speed);
 
                wb_motor_set_velocity(left_motor, left_speed);
                wb_motor_set_velocity(right_motor, right_speed);
            end
        end
    end
 
    wb_robot_cleanup();
end