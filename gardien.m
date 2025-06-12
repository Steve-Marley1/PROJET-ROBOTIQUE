function gardien
    % Initialisation de la simulation Webots
    TIME_STEP = 64;              % Durée entre chaque pas de simulation (en ms)
    wb_robot_init();             % Initialisation du robot dans Webots

    % === Récupération des moteurs et du récepteur ===
    left_motor = wb_robot_get_device('left wheel motor');   % Moteur roue gauche
    right_motor = wb_robot_get_device('right wheel motor'); % Moteur roue droite
    receiver = wb_robot_get_device('receiver');             % Récepteur pour recevoir les données du superviseur

    % Les moteurs sont configurés en mode vitesse (position infinie)
    wb_motor_set_position(left_motor, inf);
    wb_motor_set_position(right_motor, inf);
    wb_motor_set_velocity(left_motor, 0);   % Moteurs initialement à l'arrêt
    wb_motor_set_velocity(right_motor, 0);

    % Activation du récepteur pour recevoir les coordonnées toutes les 64 ms
    wb_receiver_enable(receiver, TIME_STEP);

    % === Boucle principale ===
    while wb_robot_step(TIME_STEP) ~= -1  % Exécution tant que la simulation tourne

        % Si un nouveau message est reçu du superviseur
        if wb_receiver_get_queue_length(receiver) > 0
            % === Traitement du message reçu ===

            % Lecture brute des données depuis le superviseur
            size = wb_receiver_get_data_size(receiver);                    % Taille du message
            data_ptr = wb_receiver_get_data(receiver);                     % Pointeur vers les données
            setdatatype(data_ptr, 'uint8Ptr', 1, size);                    % Définition du type des données
            raw_data = data_ptr.Value;                                     % Récupération des octets
            packet = typecast(raw_data, 'double');                         % Conversion en tableau de doubles

            % === Extraction des informations robot B2 (gardien) ===
            my_index = 2;                                                  % Indice de B2 dans le paquet
            my_x = packet(3*(my_index-1) + 1);                             % Position X du robot
            my_y = packet(3*(my_index-1) + 2);                             % Position Y du robot
            my_theta = packet(3*(my_index-1) + 3);                         % Orientation du robot (non utilisée ici)

            % === Extraction des coordonnées de la balle ===
            ball_x = packet(19);                                           % Position X de la balle
            ball_y = packet(20);                                           % Position Y de la balle

            % === Paramètres du gardien ===
            x_guard = 0.75;          % Position X fixe dans les cages (côté bleu)
            y_min = -0.2;            % Bord inférieur du but
            y_max = 0.2;             % Bord supérieur du but
            center_y = 0;            % Position centrale de repos dans le but
            max_speed = 10;          % Vitesse maximale autorisée par Webots

            % === Initialisation des vitesses ===
            v = 0;                   % Vitesse linéaire du robot
            w = 0;                   % Vitesse angulaire (rotation) du robot

            % === Décision du comportement ===
            if ball_x >= 0
                % Si la balle est dans le camp du gardien :
                % → le robot suit la balle en Y, dans la limite des cages
                target_y = min(max(ball_y, y_min), y_max);
            else
                % Sinon (balle dans le camp adverse) :
                % → le gardien reste au centre
                target_y = center_y;
            end

            % === Calcul de la correction à effectuer en Y ===
            dy = target_y - my_y;    % Erreur entre position actuelle et cible

            % === Contrôle proportionnel pour générer la vitesse ===
            Kp = 2;                  % Gain proportionnel doux
            v = Kp * dy;             % Avance/recul en fonction de l'écart en Y
            w = 0;                   % Pas de rotation : le robot reste droit

            % === Conversion vers vitesses des roues ===
            L = 0.2;                 % Distance entre les deux roues
            R = 0.05;                % Rayon des roues
            left_speed = (v - (L/2) * w) / R;
            right_speed = (v + (L/2) * w) / R;

            % === Limitation de la vitesse pour éviter les erreurs Webots ===
            left_speed = max(min(left_speed, max_speed), -max_speed);
            right_speed = max(min(right_speed, max_speed), -max_speed);

            % === Affichage console pour debug ===
            disp(['Gardien : x=', num2str(my_x), ', y=', num2str(my_y), ...
                  ', balle x=', num2str(ball_x), ', balle y=', num2str(ball_y)]);

            % === Application des vitesses aux moteurs ===
            wb_motor_set_velocity(left_motor, left_speed);
            wb_motor_set_velocity(right_motor, right_speed);

            % On passe au message suivant (s'il y en a d'autres)
            wb_receiver_next_packet(receiver);
        end
    end

    % Nettoyage en fin de simulation
    wb_robot_cleanup();
end
