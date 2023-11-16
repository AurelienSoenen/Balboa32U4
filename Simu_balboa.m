% Import for wheeled inverted pendulum

addpath('real_data')  % Add path to the folder containing the data file

% Sampling time
timestep = 10 * 1e-3;                 % [s]
n_steps = size(m, 1);
max_time = (n_steps - 1) * timestep;  % [s]
start = 0;
t_list = linspace(start, max_time, n_steps);

% Read data from CSV file
FILE = 'real_data/Test_Balboa_20231031_15h18.csv';
m = dlmread(FILE, ',', 1, 0);

v_list = (m(:, 1) / 1000.0) .* (m(:, 2) / 400.0);  % [V]

theta_list = deg2rad(m(:, 5) / 1000.0);      % [rad]
phi_list = m(:, 3) / 4.0 / 1000.0 / R;       % [m]
theta_dot_list = deg2rad(m(:, 6) / 1000.0);  % [rad / s]
phi_dot_list = m(:, 4) / 4.0 / 1000.0 / R;   % [m / s]

% Initial state = [ϕ, Θ, ϕ_dot, Θ_dot]
state0 = [phi_list(idx); theta_list(idx); phi_dot_list(idx); theta_dot_list(idx)];

% Simulate the Balboa system
sol = simulate_balboa(state0, t_list, @controller);

% Plot the results
figure('Position', [100, 100, 1200, 800])

subplot(2, 2, 1)
plot(t_list, sol(:, 1), 'DisplayName', 'phi (ODE)')
hold on
plot(t_list, phi_list, 'DisplayName', 'phi (Real data)')
xlabel('Time (s)')
ylabel('phi (rad)')
legend()

subplot(2, 2, 2)
plot(t_list, sol(:, 2), 'DisplayName', 'Theta (ODE)')
hold on
plot(t_list, theta_list, 'DisplayName', 'Theta (Real data)')
xlabel('Time (s)')
ylabel('Theta (m)')
legend()

subplot(2, 2, 3)
plot(t_list, sol(:, 3), 'DisplayName', 'phi-dot (ODE)')
hold on
plot(t_list, phi_dot_list, 'DisplayName', 'phi-dot (Real data)')
xlabel('Time (s)')
ylabel('phi-dot(rad/s)')
legend()

subplot(2, 2, 4)
plot(t_list, sol(:, 4), 'DisplayName', 'Theta-dot (ODE)')
hold on
plot(t_list, theta_dot_list, 'DisplayName', 'Theta-dot (Real data)')
xlabel('Time (s)')
ylabel('Theta-dot (m/s)')
legend()

saveas(gcf, 'plot/InvertedPendulum3.png')


% Define the simulate_balboa function
function sol = simulate_balboa(x0, t_list, controller, method)
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);  % Adjust tolerances as needed

    if nargin < 4
        [~, sol] = ode45(@(t, x) f(t, x, controller), t_list, x0, options);
    else
        [~, sol] = ode45(@(t, x) f(t, x, controller), t_list, x0, options, method);
    end
end

% Define the system dynamics function f(x, controller, t)
function x_dot = f(t, x, controller)
    % Parameters from MATLAB code
    mr = 0.316;                % body part mass [kg]
    mw = 2 * 0.021;            % wheel(*2) mass [kg]
    L = 23.0 * 1e-3;           % position of COM [m]
    R = 40 * 1e-3;             % radius of wheel [m]
    
    I = 444.43 * 1e-6;         % inertia of body part [kg*m^2]
    Iw = 2 * 26.89 * 1e-6;     % inertia of wheel [kg*m^2]
    
    Br = 0.00;                 % rolling damping ratio [N*m/(rad/s)]
    Bm = 0.00;                 % bearing damping ratio [N*m/(rad/s)]
    
    g = 9.81;                  % gravity [m/s^2]
    
    % Weighting matrices
    F = [Br + Bm, -Bm; -Bm, Bm];

    H = [1 0];
    
    % Constants for DC motor
    K = 25 / 41 * 0.506;
    K_tau = 48.39 / 1000;
    Res = 4.0;
    K_Res = K_tau / Res;

    z1 = x(1);
    z2 = x(2);
    z3 = x(3);
    z4 = x(4);

    v = controller(x, t);
    T = K_Res * (v + K * z3);

    f_p_vector = mr * L * sin(z2) * [R * z4^2; g];

    M11 = Iw + (mw + mr) * R^2;
    M12 = mr * R * L * cos(z2);
    M21 = mr * R * L * cos(z2);
    M22 = I + mr * L^2;
    M_matrix = [M11, M12; M21, M22];

    tmp = M_matrix \ (f_p_vector - F * [z2; z4] - H * T);
    x_dot = [z3; z4; tmp(1); tmp(2)];
end

function v = controller(x, t)
    % Read data from a CSV file
    FILE = 'real_data/Test_Balboa_20231031_15h18.csv';
    m = dlmread(FILE, ',', 1, 0);
    
    timestep = 10 * 1e-3;                 % [s]
    n_steps = size(m, 1);
    max_time = (n_steps - 1) * timestep;  % [s]
    start = 0;
    
    v_list = (m(:, 1) / 1000.0) .* (m(:, 2) / 400.0);  % [V]
    
    % Define a sampled controller using interpolation
    t_list = linspace(start, max_time, n_steps);
    v_interp = @(t) interp1(t_list, v_list, t, 'linear', 'extrap');
    v = v_interp(t);
end

