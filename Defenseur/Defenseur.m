function defender_controller()
% Contrôleur pour un robot défenseur - MATLAB + Webots
% Suivi de balle, évitement des bords et des robots

% Constantes
ROBOTS = 6;
TIME_STEP = 64;
MAX_SPEED = 10.0;
BORDER_MARGIN = 0.05;         % Distance de sécurité avec les bords
AVOID_RADIUS = 0.12;          % Rayon d'évitement entre robots
FIELD_X = 0.95;               % Demi-largeur terrain
FIELD_Y = 0.65;               % Demi-hauteur terrain

% Initialisation
wb_robot_init();

name = wb_robot_get_name();
team = name(1);       % 'y' ou 'b'
player = name(2);     % '1', '2', '3'

receiver = wb_robot_get_device('receiver');
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');

wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0.0);
wb_motor_set_velocity(right_motor, 0.0);

wb_receiver_enable(receiver, TIME_STEP);

while wb_robot_step(TIME_STEP) ~= -1
    left_speed = 0.0;
    right_speed = 0.0;

    while wb_receiver_get_queue_length(receiver) > 0
        packet = wb_receiver_get_data(receiver);
        packet = typecast(packet, 'double');

        % Position du robot
        rx = robot_get_x(packet, team, player, ROBOTS);
        ry = robot_get_y(packet, team, player, ROBOTS);
        orientation = robot_get_orientation(packet, team, player, ROBOTS);

        % Position de la balle
        bx = ball_get_x(packet, ROBOTS);
        by = ball_get_y(packet, ROBOTS);

        % Évitement bords
        if rx < -FIELD_X + BORDER_MARGIN || rx > FIELD_X - BORDER_MARGIN || ...
           ry < -FIELD_Y + BORDER_MARGIN || ry > FIELD_Y - BORDER_MARGIN
            % Retour vers le centre
            [left_speed, right_speed] = orient_towards_target(rx, ry, orientation, 0, 0, MAX_SPEED * 0.8);
            break;
        end

        % Évitement des autres robots
        avoid_done = false;
        for i = 1:ROBOTS
            % Identifier le robot i
            t = 'y'; if i > ROBOTS/2, t = 'b'; end
            p = num2str(mod(i - 1, 3) + 1);
            if t == team && p == player
                continue % ignorer soi-même
            end
            ox = robot_get_x(packet, t, p, ROBOTS);
            oy = robot_get_y(packet, t, p, ROBOTS);

            if distance(rx, ry, ox, oy) < AVOID_RADIUS
                % Fuir l'autre robot
                angle_away = atan2(ry - oy, rx - ox);
                tx = rx + cos(angle_away);
                ty = ry + sin(angle_away);
                [left_speed, right_speed] = orient_towards_target(rx, ry, orientation, tx, ty, MAX_SPEED);
                avoid_done = true;
                break;
            end
        end

        if avoid_done
            break;
        end

        % Suivi balle
        [left_speed, right_speed] = orient_towards_target(rx, ry, orientation, bx, by, MAX_SPEED * 0.8);

        wb_receiver_next_packet(receiver);
    end

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
end

wb_robot_cleanup();

end
