clear; clc; close all;

%% =========================================================================
% NODO DE CONTROL MATLAB + ROS 2 + BASE MECANUM POR SERIAL
% =========================================================================
% Lee:
%   /lider/mediciones    -> u, v, z, dx, dy
%   /gimbal/estado       -> q1, q2, q1dot, q2dot
%
% Publica:
%   /control/comando_gimbal
%
% Envía por serial al Arduino:
%   Iw1Bw2Cw3Dw4F
%
% Control:
%   mu = [ul; um; wz; q1dot; q2dot]
%% =========================================================================

%% =========================================================================
% 0) CONFIGURACION GENERAL
% =========================================================================
usar_robot_real = true;            % true: envía al Arduino
leer_velocidades_arduino = false;  % opcional

%% =========================================================================
% 1) PARAMETROS DE CAMARA (OJO DERECHO)
% =========================================================================
cam.W = 1920;
cam.H = 1080;

par.u0 = 978.760226;      % [px]
par.v0 = 561.374034;      % [px]
par.fx = 1058.639499;     % [px]
par.fy = 1062.803319;     % [px]
par.baseline = 0.110608;  % [m]

%% =========================================================================
% 2) GEOMETRIA DEL SISTEMA
% =========================================================================
par.l1 = 0.055;       % [m]
par.l2 = 0.10;        % [m]
par.dx = 0.40;        % [m]
par.zu = 0.4;         % [m]

%% =========================================================================
% 3) ROBOT MECANUM
% =========================================================================
par.Rw = 0.04;        % [m]
par.Am = 0.113;       % [m]
par.Bm = 0.123;       % [m]
par.Lm = par.Am + par.Bm;

par.Tw_mecanum = (1/par.Rw) * [ ...
    1   -1   -par.Lm;
    1    1   -par.Lm;
    1   -1    par.Lm;
    1    1    par.Lm ];

par.Tv_mecanum = (par.Rw/4) * [ ...
     1           1           1          1;
    -1           1          -1          1;
    -1/par.Lm   -1/par.Lm    1/par.Lm   1/par.Lm ];

par.wheel_max = 20.0;   % [rad/s]

%% =========================================================================
% 4) MUESTREO Y GANANCIAS
% =========================================================================
par.Ts = 0.1;

par.alphaF = 0.1319;
par.betaF  = 0.01;

par.Kp = diag([6 6]);
par.Ks = diag([15 2.75]);

%% =========================================================================
% 5) REFERENCIAS
% =========================================================================
par.xi_d   = [par.u0; par.v0];
par.Dd     = 0.5;
par.beta_d = 0.0;

%% =========================================================================
% 6) LIMITES
% =========================================================================
par.ul_max  = 2.0;
par.um_max  = 2.0;
par.wz_max  = 2.0;
par.q1d_max = 3.0;
par.q2d_max = 3.0;

par.q1_min = deg2rad(-140);
par.q1_max = deg2rad(140);
par.q2_min = deg2rad(-30);
par.q2_max = deg2rad(60);

%% =========================================================================
% 7) SERIAL ARDUINO
% =========================================================================
if usar_robot_real
    puerto = "/dev/ttyACM0";    % AJUSTAR SI ES NECESARIO
    baudrate = 19200;

    fprintf('\n=============================================\n');
    fprintf(' ABRIENDO SERIAL ARDUINO\n');
    fprintf('=============================================\n');
    fprintf('Puerto: %s\n', puerto);
    fprintf('Baudrate: %d\n', baudrate);

    try
        serialRobot = serialport(puerto, baudrate);
        configureTerminator(serialRobot, "LF");
        flush(serialRobot);
        pause(2);
        enviarVelocidadesArduino(serialRobot, 0, 0, 0, 0);
        fprintf('Serial Arduino listo.\n');
    catch ME
        error('No se pudo abrir serial Arduino: %s', ME.message);
    end
end

%% =========================================================================
% 8) ROS 2 EN MATLAB
% =========================================================================
node = ros2node("/nodo_control_matlab");

sub_lider = ros2subscriber(node, ...
    "/lider/mediciones", ...
    "mensajes_personalizados/DeteccionLider");

sub_gimbal = ros2subscriber(node, ...
    "/gimbal/estado", ...
    "sensor_msgs/JointState");

pub_gimbal = ros2publisher(node, ...
    "/control/comando_gimbal", ...
    "mensajes_personalizados/ComandoGimbal");

msg_gimbal_cmd = ros2message(pub_gimbal);

fprintf('\n=============================================\n');
fprintf(' MATLAB CONECTADO A ROS 2\n');
fprintf('=============================================\n');

%% =========================================================================
% 9) ESTADOS INICIALES
% =========================================================================
% Estado base estimado por integración
xu = 0.0;
yu = 0.0;
psi = deg2rad(45);

% Estado gimbal (se actualizará desde /gimbal/estado)
q1 = 0.0;
q2 = 0.0;

% Filtro alpha-beta del líder
po_hat = [0; 0; 0];
vo_hat = [0; 0; 0];
primera_muestra = true;

% Último comando
mu = zeros(5,1);

%% =========================================================================
% 10) HISTORIA
% =========================================================================
Tmax = 120;
Nmax = round(Tmax/par.Ts);

t_hist   = nan(1,Nmax);
u_hist   = nan(1,Nmax);
v_hist   = nan(1,Nmax);
z_hist   = nan(1,Nmax);
rho_hist = nan(1,Nmax);

dx_hist  = nan(1,Nmax);
dy_hist  = nan(1,Nmax);

xu_hist  = nan(1,Nmax);
yu_hist  = nan(1,Nmax);
psi_hist = nan(1,Nmax);

q1_hist  = nan(1,Nmax);
q2_hist  = nan(1,Nmax);

ul_hist  = nan(1,Nmax);
um_hist  = nan(1,Nmax);
wz_hist  = nan(1,Nmax);
q1d_hist = nan(1,Nmax);
q2d_hist = nan(1,Nmax);

w1_hist  = nan(1,Nmax);
w2_hist  = nan(1,Nmax);
w3_hist  = nan(1,Nmax);
w4_hist  = nan(1,Nmax);

po_hist    = nan(3,Nmax);
pohat_hist = nan(3,Nmax);
vohat_hist = nan(3,Nmax);

det_hist = false(1,Nmax);

k = 1;
tic;

%% =========================================================================
% 11) BUCLE PRINCIPAL
% =========================================================================
try
    while k <= Nmax
        msg_lider  = sub_lider.LatestMessage;
        msg_gimbal = sub_gimbal.LatestMessage;

        if ~isempty(msg_lider) && ~isempty(msg_gimbal)

            % -------------------------------------------------------------
            % 11.1) LECTURA DE MEDICIONES
            % -------------------------------------------------------------
            detectado = logical(msg_lider.detectado);
            u = double(msg_lider.u);
            v = double(msg_lider.v);
            z = double(msg_lider.z);
            dx_img = double(msg_lider.dx);
            dy_img = double(msg_lider.dy);

            if numel(msg_gimbal.position) >= 2
                q1 = double(msg_gimbal.position(1));
                q2 = double(msg_gimbal.position(2));
            end

            % -------------------------------------------------------------
            % 11.2) CONTROL COMPLETO
            % -------------------------------------------------------------
            if detectado && z > 0

                % Punto del líder en marco de cámara
                cx = (u - par.u0) * z / par.fx;
                cy = (v - par.v0) * z / par.fy;
                cz = z;
                cpo = [cx; cy; cz];

                % Estado actual
                q = [xu; yu; psi; q1; q2];

                % Cinemática
                [pb_w, pc_w, Rwc] = forwardKinematics(q, par);
                Rcw = Rwc.';

                % Posición del líder en mundo
                po = pc_w + Rwc * cpo;

                % Filtro alpha-beta
                if primera_muestra
                    po_hat = po;
                    vo_hat = [0; 0; 0];
                    primera_muestra = false;
                else
                    po_pred = po_hat + par.Ts * vo_hat;
                    vo_pred = vo_hat;

                    r = po - po_pred;

                    po_hat = po_pred + par.alphaF * r;
                    vo_hat = vo_pred + (par.betaF / par.Ts) * r;
                end

                % Error visual
                xi = [u; v];
                e_img = xi - par.xi_d;
                rho = norm(e_img);

                % Jacobiano visual
                [Jv, Jw] = cameraGeometricJacobiansNumeric(q, par);
                JG = [Jv; Jw];

                JI = interactionMatrix(cpo, par);
                J  = JI * blkdiag(Rcw, Rcw) * JG;

                % Velocidad visual estimada del objetivo
                cpo_dot_obj = Rcw * vo_hat;
                xi_dot_o_est = JI(:,1:3) * cpo_dot_obj;

                % Tarea primaria
                J_pinv = pinv(J);
                mu_p = -J_pinv * (xi_dot_o_est + par.Kp * e_img);

                % Tarea secundaria
                Jeta = secondaryJacobianNumeric(q, po_hat, vo_hat, par);

                dx_ = po_hat(1) - pb_w(1);
                dy_ = po_hat(2) - pb_w(2);
                dxy = sqrt(dx_^2 + dy_^2);

                theta_o = atan2(vo_hat(2), vo_hat(1) + 1e-9);
                beta = wrapToPi(theta_o - psi);

                e_sec = [dxy - par.Dd;
                         wrapToPi(beta - par.beta_d)];

                eta = -pinv(Jeta) * (par.Ks * tanh(e_sec));

                Nproj = eye(5) - J_pinv * J;
                mu_s = Nproj * eta;

                % Control total
                mu = mu_p + mu_s;

                % Saturaciones
                mu(1) = saturate(mu(1), par.ul_max);
                mu(2) = saturate(mu(2), par.um_max);
                mu(3) = saturate(mu(3), par.wz_max);
                mu(4) = saturate(mu(4), par.q1d_max);
                mu(5) = saturate(mu(5), par.q2d_max);

            else
                po = [NaN; NaN; NaN];
                rho = 0.0;
                mu = zeros(5,1);
            end

            % -------------------------------------------------------------
            % 11.3) BASE MECANUM
            % -------------------------------------------------------------
            w_ruedas = par.Tw_mecanum * [mu(1); mu(2); mu(3)];

            w_ruedas(1) = saturate(w_ruedas(1), par.wheel_max);
            w_ruedas(2) = saturate(w_ruedas(2), par.wheel_max);
            w_ruedas(3) = saturate(w_ruedas(3), par.wheel_max);
            w_ruedas(4) = saturate(w_ruedas(4), par.wheel_max);

            if usar_robot_real
                enviarVelocidadesArduino(serialRobot, ...
                    w_ruedas(1), w_ruedas(2), w_ruedas(3), w_ruedas(4));

                if leer_velocidades_arduino
                    [ok_read, ~] = leerVelocidadesArduino(serialRobot); %#ok<NASGU>
                end
            end

            % -------------------------------------------------------------
            % 11.4) GIMBAL POR ROS 2
            % -------------------------------------------------------------
            msg_gimbal_cmd.velocidad_yaw   = mu(4);
            msg_gimbal_cmd.velocidad_pitch = mu(5);
            send(pub_gimbal, msg_gimbal_cmd);

            % -------------------------------------------------------------
            % 11.5) INTEGRACION ESTIMADA DE LA BASE
            % -------------------------------------------------------------
            Rwu_2d = [cos(psi) -sin(psi);
                      sin(psi)  cos(psi)];

            vu_world = Rwu_2d * [mu(1); mu(2)];

            xu = xu + vu_world(1) * par.Ts;
            yu = yu + vu_world(2) * par.Ts;
            psi = wrapToPi(psi + mu(3) * par.Ts);

            % -------------------------------------------------------------
            % 11.6) GUARDAR HISTORIA
            % -------------------------------------------------------------
            t_hist(k)   = toc;
            u_hist(k)   = u;
            v_hist(k)   = v;
            z_hist(k)   = z;
            rho_hist(k) = rho;

            dx_hist(k) = dx_img;
            dy_hist(k) = dy_img;

            xu_hist(k)  = xu;
            yu_hist(k)  = yu;
            psi_hist(k) = psi;

            q1_hist(k) = q1;
            q2_hist(k) = q2;

            ul_hist(k)  = mu(1);
            um_hist(k)  = mu(2);
            wz_hist(k)  = mu(3);
            q1d_hist(k) = mu(4);
            q2d_hist(k) = mu(5);

            w1_hist(k) = w_ruedas(1);
            w2_hist(k) = w_ruedas(2);
            w3_hist(k) = w_ruedas(3);
            w4_hist(k) = w_ruedas(4);

            if detectado && z > 0
                po_hist(:,k)    = po;
                pohat_hist(:,k) = po_hat;
                vohat_hist(:,k) = vo_hat;
            end

            det_hist(k) = detectado;

            fprintf(['det=%d | u=%8.2f v=%8.2f z=%6.3f | ' ...
                     'ul=%7.3f um=%7.3f wz=%7.3f | ' ...
                     'q1d=%7.3f q2d=%7.3f\n'], ...
                     detectado, u, v, z, mu(1), mu(2), mu(3), mu(4), mu(5));

            k = k + 1;
        end

        pause(par.Ts);
    end

catch ME
    fprintf('\nERROR EN MATLAB:\n%s\n', ME.message);
end

%% =========================================================================
% 12) SEGURIDAD Y CIERRE
% =========================================================================
if usar_robot_real
    enviarVelocidadesArduino(serialRobot, 0, 0, 0, 0);
    flush(serialRobot);
    clear serialRobot;
end

%% =========================================================================
% 13) GRAFICAS
% =========================================================================
idx = ~isnan(t_hist);

figure('Name','Plano imagen','Color','w');
plot(u_hist(idx), v_hist(idx), 'b', 'LineWidth', 1.5); hold on;
plot(par.u0, par.v0, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
xlim([0 cam.W]);
ylim([0 cam.H]);
set(gca,'YDir','reverse');
grid on;
xlabel('u [px]');
ylabel('v [px]');
title('Trayectoria del líder en imagen');
legend('\xi(t)','\xi_d','Location','best');

figure('Name','Errores visuales','Color','w');
subplot(3,1,1);
plot(t_hist(idx), dx_hist(idx), 'LineWidth',1.5); hold on;
yline(0,'r--');
grid on;
ylabel('dx [px]');
title('Desviación horizontal');

subplot(3,1,2);
plot(t_hist(idx), dy_hist(idx), 'LineWidth',1.5); hold on;
yline(0,'r--');
grid on;
ylabel('dy [px]');
title('Desviación vertical');

subplot(3,1,3);
plot(t_hist(idx), rho_hist(idx), 'LineWidth',1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('\rho [px]');
title('Norma del error visual');

figure('Name','Control base','Color','w');
subplot(3,1,1);
plot(t_hist(idx), ul_hist(idx), 'LineWidth',1.5); grid on;
ylabel('u_l [m/s]');
title('Velocidad longitudinal');

subplot(3,1,2);
plot(t_hist(idx), um_hist(idx), 'LineWidth',1.5); grid on;
ylabel('u_m [m/s]');
title('Velocidad lateral');

subplot(3,1,3);
plot(t_hist(idx), wz_hist(idx), 'LineWidth',1.5); grid on;
xlabel('Tiempo [s]');
ylabel('\omega_z [rad/s]');
title('Velocidad angular');

figure('Name','Control gimbal','Color','w');
subplot(2,1,1);
plot(t_hist(idx), q1d_hist(idx), 'LineWidth',1.5); grid on;
ylabel('q1dot [rad/s]');
title('Comando yaw');

subplot(2,1,2);
plot(t_hist(idx), q2d_hist(idx), 'LineWidth',1.5); grid on;
xlabel('Tiempo [s]');
ylabel('q2dot [rad/s]');
title('Comando pitch');

figure('Name','Velocidades ruedas','Color','w');
plot(t_hist(idx), w1_hist(idx), 'LineWidth',1.2); hold on;
plot(t_hist(idx), w2_hist(idx), 'LineWidth',1.2);
plot(t_hist(idx), w3_hist(idx), 'LineWidth',1.2);
plot(t_hist(idx), w4_hist(idx), 'LineWidth',1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('[rad/s]');
title('Velocidades enviadas a ruedas');
legend('w1','w2','w3','w4','Location','best');

figure('Name','Estimación del líder','Color','w');
subplot(3,1,1);
plot(t_hist(idx), pohat_hist(1,idx), 'LineWidth',1.4); grid on;
ylabel('x [m]');
title('Posición estimada del líder');

subplot(3,1,2);
plot(t_hist(idx), pohat_hist(2,idx), 'LineWidth',1.4); grid on;
ylabel('y [m]');

subplot(3,1,3);
plot(t_hist(idx), pohat_hist(3,idx), 'LineWidth',1.4); grid on;
xlabel('Tiempo [s]');
ylabel('z [m]');

figure('Name','Velocidad estimada del líder','Color','w');
subplot(3,1,1);
plot(t_hist(idx), vohat_hist(1,idx), 'LineWidth',1.5); grid on;
ylabel('[m/s]');
title('v_{ox} estimada');

subplot(3,1,2);
plot(t_hist(idx), vohat_hist(2,idx), 'LineWidth',1.5); grid on;
ylabel('[m/s]');
title('v_{oy} estimada');

subplot(3,1,3);
plot(t_hist(idx), vohat_hist(3,idx), 'LineWidth',1.5); grid on;
xlabel('Tiempo [s]');
ylabel('[m/s]');
title('v_{oz} estimada');

idx_eval = idx & (t_hist > 5);

rmse_img = sqrt(mean(rho_hist(idx_eval).^2));
rmse_z   = sqrt(mean((z_hist(idx_eval) - par.Dd).^2));

fprintf('\n================ RESULTADOS ================\n');
fprintf('RMSE error visual rho = %.3f px\n', rmse_img);
fprintf('RMSE distancia z      = %.3f m\n', rmse_z);
fprintf('============================================\n');

%% =========================================================================
% FUNCIONES AUXILIARES
% =========================================================================
function [pb_w, pc_w, Rwc] = forwardKinematics(q, par)
    xu  = q(1);
    yu  = q(2);
    psi = q(3);
    q1  = q(4);
    q2  = q(5);

    pb_w = [xu + par.dx*cos(psi);
            yu + par.dx*sin(psi);
            par.zu];

    pc_w = [xu + par.dx*cos(psi) + par.l2*cos(psi+q1)*cos(q2);
            yu + par.dx*sin(psi) + par.l2*sin(psi+q1)*cos(q2);
            par.zu + par.l1 + par.l2*sin(q2)];

    Rwc = Rz(psi) * Rz(q1) * Ry(q2);
end

function JI = interactionMatrix(cpo, par)
    x = cpo(1);
    y = cpo(2);
    z = cpo(3);

    fx = par.fx;
    fy = par.fy;

    xn = x/z;
    yn = y/z;

    JI = [ -fx/z, 0, fx*x/z^2, fx*xn*yn, -fx*(1 + xn^2),  fx*yn; ...
            0, -fy/z, fy*y/z^2, fy*(1 + yn^2), -fy*xn*yn, -fy*xn ];
end

function [Jv, Jw] = cameraGeometricJacobiansNumeric(q, par)
    h = 1e-6;
    [~, pc0, Rwc0] = forwardKinematics(q, par);

    Jv = zeros(3,5);
    Jw = zeros(3,5);

    for i = 1:5
        mu = zeros(5,1);
        mu(i) = 1.0;

        psi = q(3);
        R2d = [cos(psi) -sin(psi);
               sin(psi)  cos(psi)];

        vu = R2d * mu(1:2);
        qdot = [vu(1); vu(2); mu(3); mu(4); mu(5)];
        q2 = q + h*qdot;

        [~, pc1, Rwc1] = forwardKinematics(q2, par);

        Jv(:,i) = (pc1 - pc0)/h;

        Rdot = (Rwc1 - Rwc0)/h;
        S = Rdot * Rwc0.';
        Jw(:,i) = [S(3,2); S(1,3); S(2,1)];
    end
end

function Jeta = secondaryJacobianNumeric(q, po, vo_hat, par)
    h = 1e-6;
    chi0 = secondaryVariables(q, po, vo_hat, par);
    Jeta = zeros(2,5);

    for i = 1:5
        mu = zeros(5,1);
        mu(i) = 1.0;

        psi = q(3);
        R2d = [cos(psi) -sin(psi);
               sin(psi)  cos(psi)];
        vu = R2d * mu(1:2);

        qdot = [vu(1); vu(2); mu(3); mu(4); mu(5)];
        q2 = q + h*qdot;

        chi1 = secondaryVariables(q2, po, vo_hat, par);
        dchi = [chi1(1)-chi0(1);
                wrapToPi(chi1(2)-chi0(2))];

        Jeta(:,i) = dchi / h;
    end
end

function chi = secondaryVariables(q, po, vo_hat, par)
    [pb_w, ~, ~] = forwardKinematics(q, par);

    dx_ = po(1) - pb_w(1);
    dy_ = po(2) - pb_w(2);
    dxy = sqrt(dx_^2 + dy_^2);

    theta_o = atan2(vo_hat(2), vo_hat(1) + 1e-9);
    beta = wrapToPi(theta_o - q(3));

    chi = [dxy; beta];
end

function y = saturate(x, xmax)
    y = min(max(x, -xmax), xmax);
end

function R = Rz(a)
    R = [cos(a) -sin(a) 0;
         sin(a)  cos(a) 0;
         0       0      1];
end

function R = Ry(a)
    R = [ cos(a) 0 sin(a);
          0      1 0;
         -sin(a) 0 cos(a)];
end

function ang = wrapToPi(ang)
    ang = atan2(sin(ang), cos(ang));
end

function enviarVelocidadesArduino(serialRobot, w1, w2, w3, w4)
    trama = sprintf('I%.3fB%.3fC%.3fD%.3fF', w1, w2, w3, w4);
    writeline(serialRobot, trama);
end

function [ok, w_actual] = leerVelocidadesArduino(serialRobot)
    ok = false;
    w_actual = zeros(4,1);

    try
        flush(serialRobot);
        writeline(serialRobot, 'G');
        pause(0.03);

        respuesta = readline(serialRobot);
        respuesta = strtrim(respuesta);

        partes = split(respuesta, ':');
        if numel(partes) ~= 4
            return;
        end

        w1 = str2double(partes(1));
        w2 = str2double(partes(2));
        w3 = str2double(partes(3));
        w4 = str2double(partes(4));

        if any(isnan([w1 w2 w3 w4]))
            return;
        end

        w_actual = [w1; w2; w3; w4];
        ok = true;
    catch
        ok = false;
    end
end