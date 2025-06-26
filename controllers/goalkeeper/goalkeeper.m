function Guardien()

    TIME_STEP = 64;

    wb_robot_init();
 
    % Initialisation des moteurs et du receiver

    left_motor = wb_robot_get_device('left wheel motor');

    right_motor = wb_robot_get_device('right wheel motor');

    receiver = wb_robot_get_device('receiver');
 
    wb_motor_set_position(left_motor, inf);

    wb_motor_set_position(right_motor, inf);

    wb_receiver_enable(receiver, TIME_STEP);
 
    % Identification du robot

    robot_name = wb_robot_get_name();

    is_blue = startsWith(robot_name, 'b'); % true si équipe bleue, sinon jaune
 
    % Position fixe du but selon l'équipe

    goal_x = is_blue * 0.75 + (~is_blue) * (-0.75);
 
    % Limites de déplacement vertical du gardien dans le but

    min_y = -0.15;

    max_y = 0.15;
 
    % Liste des robots pour retrouver l'index du gardien

    robot_names = ["B1", "B2", "B3", "Y1", "Y2", "Y3"];

    self_index = find(strcmpi(robot_names, robot_name));
 
    while wb_robot_step(TIME_STEP) ~= -1

        left_speed = 0;

        right_speed = 0;
 
        if wb_receiver_get_queue_length(receiver) > 0

            data = wb_receiver_get_data(receiver, 'double');

            wb_receiver_next_packet(receiver);
 
            if length(data) == 27

                % Récupération de la position Y du gardien et de la balle

                gardien_y = data(4*self_index - 2);

                ball_x = data(25);

                ball_y = data(26);
 
                % Suivi de la balle dans la zone du but (contrôle proportionnel)

                target_y = max(min(ball_y, max_y), min_y); % Clamp dans la zone du but

                error_y = target_y - gardien_y;
 
                Kp = 500; % Gain proportionnel

                correction = Kp * error_y;
 
                left_speed = correction;

                right_speed = correction;

            end

        end
 
        % Saturation des vitesses moteur

        max_speed = 10;

        left_speed = max(min(left_speed, max_speed), -max_speed);

        right_speed = max(min(right_speed, max_speed), -max_speed);
 
        % Application des vitesses aux moteurs

        wb_motor_set_velocity(left_motor, left_speed);

        wb_motor_set_velocity(right_motor, right_speed);

    end
 
    wb_robot_cleanup();

end

 