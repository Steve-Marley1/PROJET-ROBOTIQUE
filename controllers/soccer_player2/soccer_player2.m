function soccer_player2()
   TIME_STEP = 64; % durée du pas de simulation en millisecondes
   wb_robot_init();
% Initialisation des capteurs de distance
   wb_distance_sensor_enable('distance sensor', TIME_STEP);


% Initialisation des moteurs
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);

% Boucle principale
while wb_robot_step(TIME_STEP) ~= -1
    % Lecture des capteurs avant
    front_left = wb_distance_sensor_get_value('distance sensor');
   % front_right = wb_distance_sensor_get_value('ps2');

    % Seuil de détection
    seuil = 80;
    
    % Logique d’évitement
    if front_left > seuil %&& front_right > seuil
        % Obstacle droit devant → reculer et tourner
        left_speed = 2.0;
        right_speed = 2.0;
    %elseif front_left > seuil
        % Obstacle à gauche → tourner à droite
        %left_speed = 2.0;
        %right_speed = -2.0;
    %elseif front_right > seuil
        % Obstacle à droite → tourner à gauche
        %left_speed = -2.0;
       % right_speed = 2.0;
    else
        % Pas d’obstacle → aller tout droit
        left_speed = -3.0;
        right_speed = -3.0;
    end
    
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
end
 wb_robot_cleanup();
end
