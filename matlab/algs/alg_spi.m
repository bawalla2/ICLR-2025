function out_data = alg_spi(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SIMULTANEOUS POLICY ITERATION (SPI) ALGORITHM
%
% [ ***** ANONYMIZED ***** ]  
%
% 2021-11-06
%
% This program implements the SPI algorithm presented in,
%
%   K.G. Vamvoudakis and F.L. Lewis. "Online actor-critic algorithm to
%   solve the continuous-time infinite horizon optimal control problem."
%   Automatica, 46:878-888, 2010.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_spi(alg_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   preset                  (String) example preset (see main.m for
%                           options).
%   sys                     (Struct) contains system tag/info. See notes in
%                           'config.m' for specific fields.
%   alg                     (String) redundant for this function. Contains
%                           the tag of this algorithm.
%   Q                       (n x n matrix, or string) If matrix, is the
%                           positive definite state penalty matrix. If
%                           string, is the tag of the desired non-quadratic
%                           positive definite state penalty function.
%   R                       (m x m matrix, or string) If matrix, is the
%                           positive definite control penalty matrix. If
%                           string, is the tag of the desired non-quadratic
%                           positive definite control penalty function.
%   basis                   (Struct) contains activation function basis
%                           parameters. Has the following fields:
%       .tag                (String) tag of the desired activation function
%                           basis to use (see eval_phi.m for options).
%       .N                  (Integer) the integer "N," or the basis
%                           dimension.
%   tf                      (Double) Length of learning window [0, t_f].
%                           I.e., time to insert probing noise for and
%                           dynamically update actor/critic weights (sec).
%   tsim                    (Double) Length of simulation to run after
%                           learning window (sec). I.e., post-learning
%                           simulation happens [t_f, t_f + tsim].
%   alpha1                  (Double) critic NN tuning gain alpha_1 > 0 (cf.
%                           eqn. (41)).
%   alpha2                  (Double) actor NN tuning gain alpha_2 > 0 (cf.
%                           eqn. (42)).
%   F1                      (N-dimensional vector) actor NN tuning
%                           parameter F_1 (cf. eqn. (42)).
%   F2                      (N x N matrix) positive definite actor NN
%                           tuning matrix F_2 (cf. eqn. (42)).
%   noise                   (Struct) contains info for probing noise. Has
%                           the following fields:
%       .tag                (String) tag of specific probing noise signal
%                           to be injected (see eval_noise.m for options).
%   c_0                     (N-dimensional vector) critic NN weight ICs.
%   w_0                     (N-dimensional vector) actor NN weight ICs.
%   x0                      (n-dimensional vector) state vector ICs.        
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       .tvec               ('simlength'-dimensional vector) vector of
%                           'simlength' time indices corresponding to the
%                           simulation time instants over the course of the
%                           algorithm execution.
%       .xmat               ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       .umat               ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       .c_mat              ('simlength' x N matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose N-columns are the critic NN weights
%                           at the respective time instant.
%       .w_mat              ('simlength' x N matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose N-columns are the actor NN weights at
%                           the respective time instant.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% 
% GLOBAL VARIABLES
% 
% *************************************************************************

% Clear global variables
if isfield(group_settings, 'clearglobal')
    if group_settings.clearglobal
        clear global
    end
else
    clear global
end

global sys;

global u_sett;


% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

noise = alg_settings.noise;

tf = alg_settings.tf;
        
Q = alg_settings.Q;
R = alg_settings.R;

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
nlq = sys.nlq;                          % System order for LQ servo
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Nominal model
model_nom_ind = alg_settings.model_nom_ind;
model_nom = get_elt_multidim(model_cell, model_nom_ind);

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = get_elt_multidim(model_cell, model_sim_ind);

% Basis
basis = alg_settings.basis_critic;         % Basis struct
N = basis.N;                        % Basis dimension

alpha1 = alg_settings.alpha1;
alpha2 = alg_settings.alpha2;
F1 = alg_settings.F1;
F2 = alg_settings.F2;



% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
% 

% Initial conditions
x0 = alg_settings.x0;
c = alg_settings.c_0;
w = alg_settings.w_0;

x0_sim = [x0; c; w];

% ***********************
%       
% CONTROL SETTINGS
%   

% Learning flag
u_sett.islearning = 1;

u_sett.Q = Q;
u_sett.R = R;
u_sett.basis = basis;

u_sett.alpha1 = alpha1;
u_sett.alpha2 = alpha2;
u_sett.F1 = F1;
u_sett.F2 = F2;

u_sett.noise = noise;

u_sett.model_nom = model_nom;
u_sett.model_sim = model_sim;



% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tvec = [];
xmat = [];
umat = [];

% Critic, actor weights
c_mat = [];
w_mat = [];




% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% LEARNING PHASE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% ***********************
%       
% RUN SIMULATION
%


% Time span for simulation
tspan = [0, tf];

% Run simulation
[t, x] = ode45(@odefunct, tspan, x0_sim);

% ***********************
%       
% STORE DATA
%

% Store time data
tvec = [    tvec
            t           ];

% Store system state data        
xmat = [    xmat
            x(:,1:n)    ]; 

% Store critic weights c        
c_mat = [   c_mat
            x(:,n+1:n+N)    ];  

% Store actor weights w         
w_mat = [   w_mat
            x(:,n+N+1:n+2*N)    ];
        
        
% DEBUGGING: Final critic NN params
c = c_mat(end,:)'

% DEBUGGING: Final actor NN params
w = w_mat(end,:)'
        



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PREPARE OUTPUT DATA
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% STATE VECTOR, ACTOR AND CRITIC WEIGHTS
% 
% *************************************************************************


out_data.tvec = tvec;
out_data.xmat = xmat;

out_data.c_mat = c_mat;
out_data.w_mat = w_mat;


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), 1);

% Calculate control
for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);
    
    % Extract state
    x = xmat(k,:)';

    % Extract actor weights
    Watmp = w_mat(k,:)';
      
    % Evaluate control 
    u = uxt_alg(x, t, Watmp);
                
    % Store control
    umat(k,:) = u';

end

% Store control signal
out_data.umat = umat;



% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMICS FOR LEARNING PORTION OF ALGORITHM
%
% State consists of system state x (n-dimensional), plus critic weights c
% (N-dimensional), plus actor weights w (N-dimensional)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;

global u_sett;

Q = u_sett.Q;
R = u_sett.R;
basis = u_sett.basis;

alpha1 = u_sett.alpha1;
alpha2 = u_sett.alpha2;
F1 = u_sett.F1;
F2 = u_sett.F2;

noise = u_sett.noise;

model_nom = u_sett.model_nom;
model_sim = u_sett.model_sim;

% Get basis dimensions
N = basis.N;

% Get system dimensions
n = sys.n;          % Order of system
% m = sys.m;

% Dynamic state variables
xp = x(1:n);

% Critic NN state variables
c = x(n+1:end-N);

% Actor NN state variables
w = x(n+N+1:end);

% Evaluate drift dynamics -- nominal model
f_x_nom = model_nom.fx(xp);
% Evaluate input gain matrix -- nominal model
g_x_nom = model_nom.gx(xp);

% Evaluate drift dynamics -- simulation model
f_x = model_sim.fx(xp);
% Evaluate input gain matrix -- simulation model
g_x = model_sim.gx(xp);

% Evaluate basis functions and gradient
[~, dphix] = eval_phi(xp, basis);

% Calculate control signal u2(x)
% cf. Vamvoudakis, Lewis (2010), Equation (39)
u2 = - 1/2 * inv(R) * g_x_nom' * dphix' * w;

% Calculate term sigma2 = (\nabla phi(x)) * (f + g u2)
sigma2 = dphix * (f_x + g_x_nom * u2);

% Calculate the term \bar{sigma2} = sigma2 / (sigma2^T * sigma2 + 1)
% cf. Vamvoudakis, Lewis (2010), under Equation (41)
bar_sigma2 = sigma2 / (sigma2' * sigma2 + 1);

% Calculate the term m = sigma2 / (sigma2^T * sigma2 + 1)^2
% cf. Vamvoudakis, Lewis (2010), under Equation (42)
mx = sigma2 / (sigma2' * sigma2 + 1)^2;

% Calculate instantaneous cost r(x,u2)
Qx = xp' * Q * xp;
rxu2 = Qx + u2' * R * u2;

% Critic NN state update law
% cf. Vamvoudakis, Lewis (2010), Equation (41)
dotWc = -alpha1 * mx * (sigma2' * c + rxu2);

% Calculation of term \overline{D}_1(x) in R^{N x N}
% cf. Vamvoudakis, Lewis (2010), below Equation (42)
D1x = dphix * g_x_nom * inv(R) * g_x_nom' * dphix';

% Actor NN state update law
% cf. Vamvoudakis, Lewis (2010), Equation (42)

% % As it appears in (42):
% dotWa = -alpha2 * ((F2 * w - F1 * bar_sigma2' * c) ...
%         - 0.25 * D1x * w * mx' * c);

% NOTE: As implemented in Vamvoudakis's code, it doesn't match (42)!!
dotWa = -alpha2 * ((F2 * w - F2 * c) - 0.25 * D1x * w * mx' * c);

% Probing noise insertion
u = uxt_alg(xp, t, w);

% Calculate state derivative
xdot = [    f_x + g_x * u
            dotWc
            dotWa       ];



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(t)
%
% The control applied depends on the current stage of the algorithm.
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function u = uxt_alg(x, t, W)

% Global variables
global sys;
global u_sett;

R = u_sett.R;
basis = u_sett.basis;

islearning = u_sett.islearning;

noise = u_sett.noise;

model_nom = u_sett.model_nom;

% Get system dimensions
% n = sys.n;          % Order of system
m = sys.m;

% Evaluate input gain matrix -- nominal model
g_x_nom = model_nom.gx(x);

% Evaluate basis functions and gradient
[~, dphix] = eval_phi(x, basis);

% Calculate control signal u2(x)
% cf. Vamvoudakis, Lewis (2010), Equation (39)
u2 = - 1/2 * inv(R) * g_x_nom' * dphix' * W;


% Probing noise insertion
if islearning
   
    u = u2 + eval_noise(t, m, noise);
    
else
    
    u = u2;
    
end

