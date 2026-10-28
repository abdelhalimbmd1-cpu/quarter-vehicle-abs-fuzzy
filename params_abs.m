%% params_abs.m - Parameters for Quarter-Vehicle ABS Fuzzy Logic Controller Model
% Reference: Geleta, Gopal & Lemi (2024), Research on Engineering Structures & Materials 10(2), 495-511
% ALL tunable values live here only.

%% Road Condition Selector
% Options: 'dry' (default), 'wet', 'ice'
if ~exist('road_condition', 'var') || isempty(road_condition)
    road_condition = 'dry';
end

%% Simulation Solver Settings
solver_type         = 'ode4';       % Runge-Kutta 4th order fixed-step solver
solver_step         = 1e-3;         % Fixed step size: 1 ms (1e-3 s)
sim_time            = 10.0;         % Simulation stop time: 10 s

%% Initial Conditions
v0_kmh              = 60.0;         % Initial vehicle speed in km/h
v0                  = 16.67;        % Initial vehicle speed (m/s) [60 km/h = 16.67 m/s]
v_stop              = 0.02;         % Stop simulation threshold when vehicle comes to rest (m/s)

%% Quarter-Vehicle Model Parameters
m                   = 360.0;        % Quarter vehicle mass (kg) [used in Fz0=m*g/4 and 1/m]
g                   = 9.81;         % Acceleration due to gravity (m/s^2)
R                   = 0.32;         % Wheel rolling radius (m)
L                   = 2.73;         % Wheelbase (m) [from Fig. 6: 2730 mm]
hcg                 = 0.50;         % Center of gravity height (m) [from Fig. 6: 500 mm]

%% Aerodynamic & Rolling Resistance Parameters
rho                 = 1.2;          % Air density (kg/m^3)
Cd                  = 0.35;         % Aerodynamic drag coefficient
Cr                  = 0.015;        % Rolling resistance coefficient
Af                  = 1.6 + 0.00056 * (m - 765); % Frontal area (m^2) [Eq. 4: 1.3732 m^2]

%% Wheel Dynamics & Inertia
mw                  = 20.0;         % Mass of the wheel (kg)
Iw                  = 0.5 * mw * R^2; % Wheel moment of inertia (kg*m^2) [Eq. 12: 1.024 kg*m^2]
omega0              = v0 / R;       % Initial wheel angular velocity (rad/s) [~ 52.09 rad/s]

%% Brake Hydraulic System Parameters
P_max               = 3.0e6;        % Maximum brake line pressure (Pa) [3 MPa, Section 3.2]
P_min               = 0.0;          % Minimum brake line pressure (Pa)
Kb                  = 13.0;         % Lumped gain converting pressure to brake torque (N*m/Pa)
                                    % Calibrated so Tb can exceed Fx*R (~265 N*m) and stop at ~7.3 s matching Fig. 8

%% Load Transfer and Force Gains
K_Fl                = -m * hcg / (2 * L); % Longitudinal load transfer gain: Fl = K_Fl * v_dot [Eq. 9]
K_Fa                = 0.5 * rho * Cd * Af;% Aerodynamic drag gain: Fa = K_Fa * v^2 [Eq. 4]
Fr_val              = Cr * m * g;         % Rolling resistance force (N) [Eq. 5: ~ 52.97 N]
Fz0                 = m * g / 4;          % Static normal load on quarter vehicle (N) [Eq. 8: ~ 882.9 N]

%% Target Slip
desired_slip        = 0.20;         % Desired optimum slip ratio lambda_0 [Table 2, Fig. 7]

%% Digitized Mu-Slip Lookup Tables from Fig. 3
slip_table = [0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00];

% Dry asphalt (peak mu ~ 0.90 near lambda = 0.22, plateau mu ~ 0.865)
mu_dry  = [0.00, 0.40, 0.72, 0.84, 0.88, 0.905, 0.90, 0.892, 0.88, 0.85, 0.82, 0.79, 0.76, 0.73, 0.71];

% Wet asphalt (peak mu ~ 0.46 near lambda = 0.25)
mu_wet  = [0.00, 0.16, 0.33, 0.41, 0.44, 0.46, 0.46, 0.45, 0.44, 0.43, 0.41, 0.39, 0.37, 0.35, 0.34];

% Icy road (peak mu ~ 0.22 near lambda = 0.25)
mu_ice  = [0.00, 0.12, 0.19, 0.21, 0.222, 0.226, 0.222, 0.218, 0.214, 0.205, 0.197, 0.190, 0.183, 0.176, 0.170];

switch lower(road_condition)
    case {'dry', 'dry asphalt'}
        mu_table = mu_dry;
    case {'wet', 'wet asphalt'}
        mu_table = mu_wet;
    case {'ice', 'icy road'}
        mu_table = mu_ice;
    otherwise
        warning('Unknown road condition "%s". Defaulting to dry asphalt.', road_condition);
        mu_table = mu_dry;
end

%% Suppress transient fuzzy logic warnings during stopping lockup
warning('off', 'fuzzy:general:errOutOfRangeInputValue');
warning('off', 'fuzzy:general:errNoRuleFired');
