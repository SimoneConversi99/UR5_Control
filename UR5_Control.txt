% --- Controllo di traiettoria per un manipolatore cinematico --- %

clear();
clc();
close all;

%Numero matricola per la parametrizzazione
%N = 161227;
c1 = 1;
c2 = 2;
c3 = 6;
c4 = 7;
c5 = 4;

%Robot
startConfiguration = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
goalConfiguration = [c1 / (2 * pi); pi / 3; c3 / (2 * pi); 0.0; 0.0; 0.0];
robot = loadrobot("universalUR5");
robot.DataFormat = "column";

%Obstacle 1
s1 = [0.7; 0.1; -0.4];
r1 = 0.2;
obstacle1 = collisionSphere(r1);
obstacle1Pose = eye(4);
obstacle1Pose(1:3,4) = s1;
obstacle1.Pose = obstacle1Pose;

%Obstacle 2
s2 = [0.2; 0.4; 0.1];
r2 = 0.1;
obstacle2 = collisionSphere(r2);
obstacle2Pose = eye(4);
obstacle2Pose(1:3,4) = s2;
obstacle2.Pose = obstacle2Pose;

%Environment
environment = {obstacle1, obstacle2};

%Plot
show(robot,startConfiguration);
hold on
show(robot,goalConfiguration);
for i=1:length(environment)
    show(environment{i});
end

disp("Press a button to continue");
pause(); %Press a button to continue

%% Pianificazione del movimento

% Configurazione dell'algoritmo
rrt = manipulatorRRT(robot, environment);
rrt.MaxConnectionDistance = 0.4;
rrt.EnableConnectHeuristic = true; 
rng('shuffle');

% Esecuzione dell'algoritmo
[Percorso, info] = plan(rrt, startConfiguration.', goalConfiguration.');

% Generazione dei grafici di posizione dei giunti
for i = 1:6
    subplot(6, 1, i);
    plot(Percorso(:, i), 'o');
    xlabel('Indice rrt [ad]');
    ylabel('Posizione [rad]');
    title(strcat("Giunto ", num2str(i)));
end

disp('Grafici della posizione dei giunti creati. Premere un tasto per continuare.')
pause();
close all;
clc();

% Display del robot nell'ambiente in config. iniziale e finale
show(robot, startConfiguration);
hold on
show(robot, goalConfiguration);
for i=1:length(environment)
    show(environment{i});
end

% Posizione dell' end effector in base alla configurazione
for i = 1:size(Percorso, 1)
    robotPose = robot.getTransform(Percorso(i, :).', 'ee_link');
    scatter3(robotPose(1, 4), robotPose(2, 4), robotPose(3, 4), 'r');
end

disp('Percorso generato. Premere un tasto per continuare')
pause();
close all;
clc();

%% Generazione della traiettoria

% Creazione del percorso p(s). Interpolazione cubica Spline
PosSpline = cscvn(Percorso.');  %Percorso di posizione p = p(s)
VelSpline = fnder(PosSpline);   %Percorso di velocità dp = dp(s)/ds
AccSpline = fnder(VelSpline);   %Percorso di accelerazione ddp = d^2p(s)/ds^2

LunghezzaSpline = PosSpline.breaks(end);    %Lunghezza del percorso di posizione

% Generazione della legge oraria s(t). Discretizzazione p(s) nel tempo
TempoTot = 3;   %Tempo che deve impiegare il robot per raggiungere la congif. finale
PuntiTraiettoria = 1000;
Tempo = linspace(0, TempoTot, PuntiTraiettoria);
[st, dst, ddst, pp] = quinticpolytraj([0, LunghezzaSpline], [0, TempoTot], Tempo);

% Grafici del profilo di moto
% Posizione
subplot(1, 3, 1)
plot(Tempo, st);
xlabel("Tempo [s]");
ylabel("Spazio [m]");
title('Profilo di moto s(t)');

%Velocità
subplot(1, 3, 2)
plot(Tempo, dst, 'r');
xlabel("Tempo [s]");
ylabel("Velocità [m/s]");
title('Profilo di velocità ds(t)');

%Accelerazione
subplot(1, 3, 3)
plot(Tempo, ddst, 'k');
xlabel("Tempo [s]");
ylabel("Accelerazione [m/s^2]");
title('Profilo di accelerazione dds(t)');

disp("Profili di moto generati. Premere un tasto per continuare");
pause();
close all;
clc;

% Generazione della traiettoria x(t)
% Posizione x(t) = p(s(t))
PosTraiettoria = (ppval(PosSpline, st)).';

% Velocità dx(t) = dp * dst
VelTraiettoria = (ppval(VelSpline, st) .* dst).';

% Accelerazione ddx(t) = ddp * dst^2 + dp * ddst 
AccTraiettoria = (ppval(AccSpline, st) .* dst.^2 + ppval(VelSpline, st) .* ddst).';

% Grafici dei giunti in funzione del tempo
% Posizione giunti
for i = 1:6
    subplot(2, 3, i);
    plot(Tempo, PosTraiettoria(:, i));
    xlabel('Tempo [s]');
    ylabel('Posizione [rad]');
    title(strcat("Posizione giunto ", num2str(i)));
end
disp("Grafico di posizione generato. Premere un tasto per continuare");
pause();
clc;

% Velocità giunti
for i = 1:6
    subplot(2, 3, i);
    plot(Tempo, VelTraiettoria(:, i), 'r');
    xlabel('Tempo [s]');
    ylabel('Velocità [rad/s]');
    title(strcat("Velocità giunto ", num2str(i)));
end
disp("Grafico di velocità generato. Premere un tasto per continuare");
pause();
clc;

%Accelerazione giunti  
for i = 1:6
    subplot(2, 3, i);
    plot(Tempo, AccTraiettoria(:, i), 'k');
    xlabel('Tempo [s]');
    ylabel('Accelerazione [rad/s^2]');
    title(strcat("Accelerazione giunto ", num2str(i)));
end
disp("Grafico di accelerazione generato. Premere un tasto per continuare");
pause();
close all;
clc;

show(robot, startConfiguration);
hold on
show(robot, goalConfiguration);
for i=1:length(environment)
    show(environment{i});
end

% Waypoints
for i = 1:size(Percorso, 1)
    robotPose = robot.getTransform(Percorso(i, :).', 'ee_link');
    scatter3(robotPose(1, 4), robotPose(2, 4), robotPose(3, 4), 'r');
end

% Posizioni dell'end effector lungo la traiettoria
for i = 1:10:length(Tempo)
    robotPose = robot.getTransform(PosTraiettoria(i, :).', 'ee_link');
    scatter3(robotPose(1, 4), robotPose(2, 4), robotPose(3, 4), 10, 'b');
end

disp("Traiettoria generata. Premere un tasto per continuare");
pause();
close all;
clc;

%% Controllo del manipolatore cinematico

% Configurazione della simulazione
StepSimulazione = 0.003;        %Aggiornamento dell'input al robot ogni StepSimulazione secondi 
K = diag([10 10 10 10 10 10]);  %Matrice di controllo della convergenza 6x6 diagonale

t = 0;                  % tempo incrementale di simulazione 1x1
q = startConfiguration; % configurazione reale 6x1
dq = zeros(6, 1);       % velocità di giunto reale 6x1

Storia_q = q.';         % lista delle configurazioni reali 
Storia_t = t;           % lista degli istanti di tempo 

Storia_q_des = q.';     % lista delle configurazioni desiderate

% Simulazione
while (t < TempoTot)

    % Configurazione desiderata ad ogni step di simulazione
    q_des = interp1(Tempo, PosTraiettoria, t).';
    dq_des = interp1(Tempo, VelTraiettoria, t).';

    % Output del controllore
    u = dq_des + K * (q_des - q); %6x6 * 6x1 = 6x1

    % Aggiornamento
    dq = u;     %Manipolatore cinematico --> q_punto = u
    q = q + dq * StepSimulazione;
    t = t + StepSimulazione;

    disp(t);
    
    % Salvataggio dei dati calcolati
    Storia_q = [Storia_q; q.'];
    Storia_t = [Storia_t; t];
    Storia_q_des = [Storia_q_des; q_des.'];
end

% Grafici delle traiettorie dei giunti
for i = 1 : 6
    subplot(2, 3, i)
    
    %Grafico traiettoria reale
    plot(Storia_t, Storia_q(:, i), 'LineWidth', 2);
    hold on

    %Grafico traierìttoria desiderata
    plot(Storia_t(1:25:end), Storia_q_des(1:25:end, i), 'o');
    grid on
    xlabel("Time [s]")
    ylabel("Position [rad]")
    title(strcat("Giunto", num2str(i)));
    legend('Reale', 'Desiderata', 'Location', 'eastoutside');
    xlim([0,3]);
end

% Conclusione
disp("La configurazione desiderata era:");
for i = 1:6
    disp("Giunto " + num2str(i) + ": " + num2str(goalConfiguration(i)) + " [rad]");
end
disp(" ");
disp("Il robot ha raggiunto la configurazione:");
for i = 1:6
    disp("Giunto " + num2str(i) + ": " + num2str(Storia_q(1001, i)) + " [rad]");
end

disp("Dati conclusivi. Premere un tasto per continuare");
pause();
close all;
clc;

%% Video
figure
Storia_t_30Fps = 0: 1/30: Storia_t(end);
% Interpolazione lineare dei valori Storia_q calcolati per ogni valore di
% Storia_t ma presi solo i punti di Storia_t_3Fps
Storia_q_30Fps = interp1(Storia_t, Storia_q, Storia_t_30Fps);

show(robot, startConfiguration);
hold on
for i=1:length(environment)
    show(environment{i});
end

for i = 1:size(Percorso, 1)
    robotPose = robot.getTransform(Percorso(i, :).', 'ee_link');
    scatter3(robotPose(1, 4), robotPose(2, 4), robotPose(3, 4), 'r');
end

for i = 1:length(Storia_q_30Fps)
    robotPose = robot.getTransform(Storia_q_30Fps(i, :).', 'ee_link');
    scatter3(robotPose(1, 4), robotPose(2, 4), robotPose(3, 4), 10, 'b');
    show(robot, Storia_q_30Fps(i, :).', 'PreservePlot', false);
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);

    F(i) = getframe(gcf);
end

video = VideoWriter("Progetto_1_2_video.mp4", 'MPEG-4');
open(video);
for i = 2 : length(F) - 1
    writeVideo(video, F(i));
end
close(video);

disp("Video generato. Premere un tasto per continuare.")
close all;
clc;





