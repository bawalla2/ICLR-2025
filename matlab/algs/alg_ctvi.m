function out_data = alg_ctvi(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% VALUE ITERATION (VI) ALGORITHM
%
% [ ***** ANONYMIZED ***** ] 
%
% 2022-01-06
%
% This program implements the VI algorithm presented in,
%
%   T. Bian and Z.-P. Jiang. "Reinforcement Learning and Adaptive Optimal
%   Control for Continuous-Time Nonlinear Systems: A Value Iteration
%   Approach." IEEE Transactions on Neural Networks and Learning Systems,
%   Accepted for Publication, 2021.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_radp_unmatched(alg_settings)
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
%   basis                   (Struct) contains activation function bases
%                           parameters. Has the following fields:
%       Phi                 (Struct) Contains parameters for the critic NN
%                           basis. Has the following fields:
%           tag             (String) Tag of desired activation functions.
%           N               (Integer) Number of critic activation functions
%                           N_1.
%       Psi                 (Struct) Contains parameters for the
%                           Hamiltonian NN basis which are associated with
%                           the actor. Has the following fields:
%           tag             (String) Tag of desired activation functions.
%           N               (Integer) Number of activation functions of x
%                           and u N_2. 
%       Theta               (Struct) Contains parameters for the remaining
%                           activations of the Hamiltonian NN basis (i.e.,
%                           those which do not pertain to the actor). Has
%                           the following fields:
%           tag             (String) Tag of desired activation functions.
%           N               (Integer) Number of activation functions of x
%                           only N_3.
%   noise                   (Struct) contains info for probing noise. Has
%                           the following fields:
%       tag                 (String) tag of specific probing noise signal
%                           to be injected (see eval_noise.m for options).
%   u_0                     (String, optional) Tag corresponding to an
%                           initial policy u_0(x). NOTE: Initial policy not
%                           required for VI algorithm. If one is not used,
%                           then simply declare this vield as '0'.
%   sf                      (Double) Amount of time to run weight
%                           differential equation learning for (sec).
%   tf                      (Double) Length of learning window [0, t_f]
%                           (sec).
%   tsim                    (Double) Length of simulation to run after
%                           learning window (sec). I.e., post-learning
%                           simulation happens [t_f, t_f + tsim].
%   maxstep                 (Double) Max step size for ode45 (sec).
%   int_H_ode45_1_man_0     (Bool) 1 = perform Hamiltonian function
%                           integral for the critic weight update (13) via
%                           ode45 (=1, slower, more numerically reliable)
%                           or manually with pre-existing state trajectory
%                           data (=0, faster).
%   x0                      (n-dimensional vector) ICs x(t_0).
%   c_0                     (N1-dimensional vector) ICs for critic NN
%                           weights.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%   tvec                    ('simlength'-dimensional vector) vector of
%                           'simlength' time indices corresponding to the
%                           simulation time instants over the course of the
%                           algorithm execution.
%   xmat                    ('simlength' x (p+n+1) matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%   umat                    ('simlength' x m matrix) Matrix (vector,
%                           really) whose row indexes the time instants
%                           specified in .tvec, and whose m columns are the
%                           control signal u(t) at the respective time
%                           instant.
%   lentvec_learn           (Integer) Number of simulation data in the
%                           learning phase of the algorithm; i.e., over the
%                           interval [0, t_f].
%   svec                    ('max_iter'-dim vector) Vector whose ith entry
%                           is the ith time instant of the VI learning
%                           dynamic weight update. So svec(1) = 0,
%                           svec(max_iter) = s_f.
%   c_mat                   ('max_iter' x N1 matrix) Matrix whose ith row
%                           indexes the ith VI time index svec(i), and
%                           whose N1-columns are the critic NN weights c_s
%                           at the respective index svec(i).
%   w_mat                   ('max_iter' x N2 matrix) Matrix whose ith row
%                           indexes the ith VI time index svec(i), and
%                           whose N2-columns are the Hamiltonian NN weights
%                           w_s (i.e., the weights associated with the
%                           actor) at the respective index svec(i).
%   v_mat                   ('max_iter' x N3 matrix) Matrix whose ith row
%                           indexes the ith VI time index svec(i), and
%                           whose N3-columns are the Hamiltonian NN weights
%                           v_s (i.e, the weights NOT associated with the
%                           actor) at the respective index svec(i).
%   cond_A_vec              (2-dim. vector) The first index is the
%                           condition number cond(K_{\phi}(t_f)). The
%                           second is the condition number
%                           cond(K_{\sigma}(t_f))
%   num_iter                (Integer) Number of iterations performed by the
%                           algorithm before termination.
%     
% *************************************************************************
%
% REMARK ON NOTATION
%
% *************************************************************************
%
% In T. Bian, Z.-P. Jiang, the there is the notation
%
% PI INDEX VARIABLE: k
%
% CRITIC NN:
%
%   ACTIVATION FUNCTIONS:
%   {\phi_i(x))}_{i=1}^{\infty}
%
%   TRUNCATION:
%   {\phi_i(x))}_{i=1}^{N_1}
%
%   WEIGHTS:
%   w_k \in R^{N_1}
%
% HAMILTONIAN NN:
%
%   ACTIVATION FUNCTIONS:
%   {\psi_i(x,u)}_{i=1}^{\infty} = 
%
%       {\psi_i^{(0)}(x)}_{i=1}^{\infty}
%       U
%       {\psi_i^{(1)}(x) * u}_{i=1}^{\infty}
%       U
%       {u^T R u}
%
%   TRUNCATION:
%   {\psi_i^{(0)}(x)}_{i=1}^{N_{2,0}}
%   {\psi_i^{(1)}(x)}_{i=1}^{N_{2,1}}
%   N_2 = N_{2,0} + N_{2,1}
%
%   \Phi(x) = (\phi_1(x), ... , \phi_{N1}(x))
%   \Psi(x,u) = [       \psi_1^{(0)}(x)
%                       ...
%                       \psi_{N_{2,0}}^{(0)}(x)
%                       \psi_1^{(1)}(x) u
%                       ...
%                       \psi_{N_{2,1}}^{(1)}(x) u
%                       u^T R u             ]
%
%   WEIGHTS:
%   c_k^{(0)} \in R^{N_{2,0}}
%   c_k^{(1)} \in R^{N_{2,1}}
%
% Unfortunately, none of this notation is consistent with the standard
% notation we have adopted. We thus make the name changes:
%
% PI INDEX VARIABLE: i
%
% CRITIC NN:
%
%   ACTIVATION FUNCTIONS:
%   {\phi_j(x))}_{j=1}^{\infty}
%
%   TRUNCATION:
%   {\phi_j(x))}_{j=1}^{N_1}
%
%   WEIGHTS:
%   w_s \in R^{N_1}
%
% HAMILTONIAN NN:
%
%   ACTIVATION FUNCTIONS:
%   {\sigma_j(x,u)}_{j=1}^{\infty} =

%       {\theta_j(x)}_{j=1}^{\infty}
%       U
%       {\psi_j(x) * u}_{j=1}^{\infty}
%       U
%       {u^T R u}
%
%   TRUNCATION:
%   {\theta_j(x)}_{j=1}^{N_3}
%   {\psi_j(x)}_{j=1}^{N_2}
%   N_H = N_2 + N_3
%
%   \Phi(x) = (\phi_1(x), ... , \phi_{N1}(x))
%   \Sigma(x,u) = [     \psi_1(x) u
%                       ...
%                       \psi_{N2}(x) u
%                       \theta_1(x)
%                       ...
%                       \theta_{N3}(x))
%                       u^T R u             ]
%
%   WEIGHTS:
%   w_s \in R^{N_2}
%   v_s \in R^{N_3}
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


% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Nominal model
model_nom_ind = alg_settings.model_nom_ind;
model_nom = get_elt_multidim(model_cell, model_nom_ind);

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = get_elt_multidim(model_cell, model_sim_ind);

% State, control penalties
Q = alg_settings.Q;
R = alg_settings.R;

% Probing noise
noise = alg_settings.noise;

% Weight differential equation learning time s_f
sf = alg_settings.sf;

% Length of learning window [0, t_f] (sec).
tf = alg_settings.tf;

% Max stepsize for ode45
maxstep = alg_settings.maxstep;

% Whether to integrate Hamiltonian function (13) via ode45 (=1) or manually
% (=0)
int_H_ode45_1_man_0 = alg_settings.int_H_ode45_1_man_0;

% ***********************
%       
% BASIS SETTINGS
%
% See description for details.
%

% Basis struct
basis = alg_settings.basis;

% Dimension of critic NN
N1 = basis.Phi.N;

% Dimensions of Hamiltonian NN
N2 = basis.Psi.N;            % Functions associated with actor network
N3 = basis.Theta.N;          % Remaining functions not associated w/ actor
N_H = N2 + N3;               % Total dimension


% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% INITIAL CONDITIONS
%
% NOTE: State partitioning for simulation
%
% In order to perform the RADP algorithm with unmatched uncertainty, the
% following types of dynamic variables will be necessary for simulation:
%
%   System states x
%   Integrals associated with basis functions
%

% Initial conditions
x0 = alg_settings.x0;

% Initial condition for simulation
% See odefunct() for a description of the state partition
x0_sim = [  x0
            zeros(N1^2, 1)
            zeros(N_H^2, 1)
            zeros(N_H * N1, 1)
            zeros(N_H, 1)
            zeros(N_H, 1)       ];



% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initialize critic NN weight vector
c_s = alg_settings.c_0;     % Holds critic NN weights of current iteration

% Initialize Hamiltonian NN weight vector
wv_s = zeros(N_H,1);     % Holds Hamiltonian NN weights of current iteration   

% Initial policy u_0(x)
u_0 = alg_settings.u_0;

% Initialize vector to hold condition number of K_{\phi}(t_f),
% K_{\sigma}(t_f)
cond_A_vec = zeros(2,1);


% ***********************
%       
% CONTROL SETTINGS
%   

% Learning flag
u_sett.islearning = 1;

u_sett.Q = Q;
u_sett.R = R;
u_sett.basis = basis;

u_sett.noise = noise;

u_sett.model_nom = model_nom;
u_sett.model_sim = model_sim;

% Set IC
u_sett.x0 = x0;

% Set initial policy
u_sett.u_0 = u_0;

% Whether to integrate Hamiltonian function (13) via ode45 (=1) or manually
% (=0)
u_sett.int_H_ode45_1_man_0 = int_H_ode45_1_man_0;

u_sett.tf = tf;

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************


% ***********************
%       
% STATE VARIABLES
%

% Time vector corresponding to the total simulation (including the PI phase
% and the final approximate optimal policy phase)
tvec = [];

% Matrix consisting of the system state vector in each row, corresponding
% to the times in tvec
xmat = [];


% ***********************
%       
% WEIGHTS
%

% Stores critic NN weights c_s at each iteration of the VI algorithm
% (iteration indexed by row)
c_mat = c_s';

% Stores Hamiltonian NN weights [w_s^T v_s]^T at each iteration of the VI
% algorithm (iteration indexed by row)
wv_mat = wv_s';

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
%
% LEARNING PHASE: COLLECT ONLINE DATA, EVALUATE BASIS FUNCTION INTEGRALS
%
% Here, the system will be simulated with the initial control/noise input
% over the interval [0, t_f]. In addition, the following integrals will be
% performed:
%
%   IPhiPhi         R^{N_1^2}
%   ISigmaSigma     R^{N_H^2}
%   ISigmadPhidx    R^{N_H * N_1}
%   ISigmaQ         R^{N_H}
%   ISigmaR         R^{N_H}
%
% See the odefunct() for a description of these matrices.
% 
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% COLLECT SIMULATION DATA
%  
% *************************************************************************


% *************************************************************************
%       
% RUN SIMULATION
%

tspan = [0 tf];

% Set max step size for ode45
ode_opts = odeset('MaxStep',maxstep);

[t, x] = ode45(@odefunct, tspan, x0_sim, ode_opts);

% % DEBUGGING: Plot state trajectory
% figure(100)
% hold on
% plot(t,x(:,1:n));
% title('State Trajectory')
% grid on
% xlabel('Time (sec)')
% ylabel('x(t)')

% DEBUGGING: Print IC x(0)
x0

% *************************************************************************
%       
% EXTRACT DYNAMIC VARIABLE DATA
%         

% % Extract dynamic state variable data at beginning of simulation
% x0 = (x(1,:))';

% Extract dynamic state variable data at end of simulation
x1 = (x(size(x,1),:))';

% Unpack state data at end of simulation. See odefunct() for a description
% of the state partition.

% State dynamic variables
xs1 = x1(1:n);       

% Extract dynamic variables associated with integrals of basis functions
% (vectors)
ind0 = n;
len = N1^2;
IPhiPhi = x1(ind0+1:ind0+len);          % R^{N_1^2}
ind0 = ind0 + len;
len = N_H^2;
ISigmaSigma = x1(ind0+1:ind0+len);      % R^{N_H^2}
ind0 = ind0 + len;
len = N_H * N1;
ISigmadPhidx = x1(ind0+1:ind0+len);     % R^{N_H * N_1}
ind0 = ind0 + len;
len = N_H;
ISigmaQ = x1(ind0+1:ind0+len);          % R^{N_H}
ind0 = ind0 + len;
len = N_H;
ISigmaR = x1(ind0+1:ind0+len);          % R^{N_H}

% IPhiPhi = x1(n+1:n+N1^2);                                % R^{N_1^2}
% ISigmaSigma = x1(n+N1^2+1:n+N1^2+N_H^2);                 % R^{N_H^2}
% ISigmadPhidx = x1(n+N1^2+N_H^2+1:n+N1^2+N_H^2+N1*N_H);   % R^{N_H * N_1}
% ISigmaQ = x1(n+N1^2+N_H^2+N1*N_H+1:n+N1^2+N_H^2+N1*N_H+N_H);   % R^{N_H}

% Reshape dynamic variables associated with integrals of basis functions
% into matrices of appropriate dimension
%
% K_\phi(t_f) \in R^{N_1 x N_1} (cf. Sec. IV. A.)
K_phi = reshape(IPhiPhi, [N1, N1]);

% K_\sigma(t_f) \in R^{N_H x N_H} (cf. Sec. IV. A.)
K_sigma = reshape(ISigmaSigma, [N_H, N_H]);

% \int_{0}^{t_f} \Sigma(x,u) [\nabla\Phi(x) \dot{x}]^T dt \in R^{N_H x N_1}
% Used for updating Hamiltonian NN weight c_s (cf. Sec. IV. D.)
ISigmadPhidx = reshape(ISigmadPhidx, [N_H, N1]);

% ***********************
%       
% DATA STORAGE -- SYSTEM STATE DATA
% 

% Store time data
tvec = t;
len_tvec_learn = size(tvec,1);   % Length of time vector for learning phase

% Store system state data
xmat = x(:,1:n);

% Store condition number data
cond_A_vec(1) = cond(K_phi);
cond_A_vec(2) = cond(K_sigma);

% DEBUGGING: Check problem conditioning
disp(['Condition Number of K_{\phi}(t_f) for Least Squares:         '...
            num2str(cond(K_phi), 4)])
disp(['Condition Number of K_{\sigma}(t_f) for Least Squares:       '...
            num2str(cond(K_sigma), 4)])

% Store data for weight tuning
u_sett.tvec = tvec;
u_sett.xmat = xmat;
u_sett.K_phi = K_phi;
u_sett.K_sigma = K_sigma;

u_sett.ISigmadPhidx = ISigmadPhidx;
u_sett.ISigmaQ = ISigmaQ;
u_sett.ISigmaR = ISigmaR;

%%
% *************************************************************************
% *************************************************************************
%
% VI PHASE: CALCULATE CRITIC NN WEIGHTS c_s, HAMILTONIAN NN WEIGHTS w_s,
% v_s               
% 
% *************************************************************************
% *************************************************************************


% *************************************************************************
%       
% RUN SIMULATION
%

tspan = [0 sf];

% % Set max step size for ode45
% ode_opts = odeset('MaxStep',maxstep);

[svec, c_mat] = ode45(@odefunct_vi, tspan, c_s);

% *************************************************************************
%       
% CALCULATE HAMILTONIAN WEIGHTS w_s, v_s BASED ON CRITIC WEIGHTS c_s
%

% Length of VI simulation
lensvec = size(svec, 1);

% Data storage
w_mat = zeros(lensvec, N2);
v_mat = zeros(lensvec, N3);

% Calculate the weights
for i = 1:lensvec
    
    % Critic weight c(s)
    cs = c_mat(i,:)';
    
    % Hamiltonian weights update  
    wvs = update_H_weights(cs);    
    
    % Extract both components of the weight vector
    %
    %   wv_s = [    w_s
    %               v_s   ]
    
    w_mat(i,:) = wvs(1:N2);               % R^{N_2}
    v_mat(i,:) = wvs(N2+1:N2+N3);           % R^{N_3}

end



% DEBUGGING: Final critic NN params
c_s = c_mat(end,:)'

% DEBUGGING: Final Hamiltonian NN params w_s associated with the actor
w_s = w_mat(end,:)'

% DEBUGGING: Final Hamiltonian NN params v_s NOT associated with the actor
v_s = v_mat(end,:)'



            
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

% Simulation data
out_data.tvec = tvec;
out_data.xmat = xmat;

out_data.lentvec_learn = len_tvec_learn;

% Weights data
out_data.svec = svec;
out_data.c_mat = c_mat;
out_data.w_mat = w_mat;
out_data.v_mat = v_mat;

% Condition number data
out_data.cond_A_vec = cond_A_vec;


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), m);

% Calculate control
for k = 1:size(tvec,1)
    
    % Extract time
    t = tvec(k);
    
    % Extract state
    xs = xmat(k,:)';
    
    % Evaluate control 
    u = uxt_alg(xs, t);
                
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
% NOTE: State partitioning for simulation
%
% In order to perform the VI algorithm, the following two types of dynamic
% variables will be necessary for simulation:
%
%   System states
%   Integrals associated with activation functions
% 
% *************************************************************************
%
% DYNAMIC VARIABLES ASSOCIATED WITH INTEGRALS OF ACTIVATION FUNCTIONS
%
% Here, the system will be simulated with the initial control/noise input
% over the interval [0, t_f]. In addition to the system state variables,
% the following integrals will be performed:
%
%   IPhiPhi         R^{N_1^2}
%   ISigmaSigma     R^{N_H^2}
%   ISigmadPhidx    R^{N_H * N_1}
%   ISigmaQ         R^{N_H}
%   ISigmaR         R^{N_H}
%
% A description of each of these matrices can be found below:
%
%                   IPhiPhi     R^{N_1^2}
%
% This integral is given by,
%
%   IPhiPhi = \int_{0}^{t_f} kron(\Phi(x), \Phi(x)) dt
%
% where kron(a,b) denotes the Kronecker tensor product of two vectors a, b.
% IPhiPhi is used to calculate the matrix (cf. Sec. IV. A.),
%
%   K_\phi(t_f) \in R^{N_1 x N_1},
%   K_\phi(t_f) = \int_{0}^{t_f} \Phi(x) \Phi^T(x) dt 
%
%                   ISigmaSigma     R^{N_H^2}
%
% This integral is given by,
%
%   ISigmaSigma = \int_{0}^{t_f} kron(\Sigma(x,u), \Sigma(x,u)) dt
%
% where kron(a,b) denotes the Kronecker tensor product of two vectors a, b.
% ISigmaSigma is used to calculate the matrix (cf. Sec. IV. A.),
%
%   K_\sigma(t_f) \in R^{N_H x N_H},
%   K_\sigma(t_f) = \int_{0}^{t_f} \Sigma(x,u) \Sigma^T(x,u) dt 
%   
%                   ISigmadPhidx  R^{N_H * N_1}
%
% This integral is given by,
%
%   IPsidPhidx = vec( \Sigma(x,u) [\nabla\Phi(x) \dot{x}]^T dt )
%
% where vec(A) denotes the vectorization of the matrix A. This has to be
% performed because ode45 can only integrate a vector of states, not a
% matrix of states. ISigmadPhidx is used to calculate the matrix (cf. Sec.
% IV. A.),
%
%   \int_{0}^{t_f} \Sigma(x,u) [\nabla\Phi(x) \dot{x}]^T dt 
%                   \in R^{N_H x N_1}
%
% which, it can be checked, given c \in R^{N_1}, satisfies the identity:
%
%   [ \int_{0}^{t_f} \Sigma(x,u) [\nabla\Phi(x) \dot{x}]^T dt ] * c
%   =
%   \int_{0}^{t_f} \Sigma(x,u) {d \hat{V}_{N1}(x, w)}.
%
% Where \nabla\Phi(x) \in R^{N_1 x n} is the Jacobian matrix of the critic
% activation functions \Phi(x). Given that the control penalty U^T R u is a
% member of the Hamiltonian NN basis, the right-hand-side integral is
% exactly the integral involved in the Hamiltonian NN weight update
% (cf. Sec. IV. A.).
%
%                   ISigmaQ     R^{N_H}
%
% This integral is given by,
%
%   ISigmaQ = \int_{0}^{t_f} \Sigma(x,u) * Q(x) dt
%
%                   ISigmaR     R^{N_H}
%
% This integral is given by,
%
%   ISigmaR = \int_{0}^{t_f} \Sigma(x,u) * u^T R u dt
%
% *************************************************************************
%
% FINAL STATE PARTITION:
%
% x_sim =   [   x                   R^n
%               IPhiPhi             R^{N_1^2}
%               ISigmaSigma         R^{N_H^2}
%               ISigmadPhidx        R^{N_H * N_1}
%               ISigmaQ             R^{N_H} 
%               ISigmaR             R^{N_H}         ]
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

% Get system dimensions
n = sys.n;          % Order of system
% m = sys.m;          % Number of inputs

% model_nom = u_sett.model_nom;
model_sim = u_sett.model_sim;
    
 
% ***********************
%       
% SYSTEM DYNAMICS
%

% Extract system variables
xp = x(1:n);

% Evaluate drift dynamics -- simulation model
f_x = model_sim.fx(xp);
% Evaluate input gain matrix -- simulation model
g_x = model_sim.gx(xp);

% Evaluate control signal u(t)
u = uxt_alg(xp, t);

% Evaluate state derivative \dot{x}
dxs = f_x + g_x * u;

% ***********************
%       
% BASIS FUNCTIONS
%

% Dimension of critic NN
N1 = basis.Phi.N;

% Dimensions of Hamiltonian NN
N2 = basis.Psi.N;            % Functions associated with actor network
N3 = basis.Theta.N;          % Remaining functions not associated w/ actor
N_H = N2 + N3;               % Total dimension

% Evaluate basis functions
[Phix, dPhix] = eval_phi(xp, basis.Phi);

% \Sigma(x,u)
% BEFORE multiplying first N_2 functions on the right by u 
Psix = eval_phi(xp, basis.Psi);         % First N_2 components
Thetax = eval_phi(xp, basis.Theta);     % Last N_3 components

% Multiply factor of u in first N_2 elements
Psixu = Psix * u;

% Combine activation functions into one vector
Sigmaxu = [     Psixu 
                Thetax  ];

% Evaluate state penalty Q(x)
Qx = xp' * Q * xp;            
            
% Evaluate control penalty
Rxu = u' * R * u;

% % Evaluate running cost r(x,u)
% rxu = Qx + Rxu;  


% ***********************
%       
% DYNAMIC VARIABLES ASSOCIATED WITH BASIS FUNCTION INTEGRALS
%

% For K_phi = \int_{0}^{tf} Phi(x) Phi^T(x) dt
% cf. Sec. IV. A.
dIPhiPhi = kron(Phix, Phix);

% For K_psi = \int_{0}^{tf} \Sigma(x,u) \Sigma^T(x,u) dt
% cf. Sec. IV. A.
dISigmaSigma = kron(Sigmaxu, Sigmaxu);

% For \int_{0}^{tf} \Sigma(x,u) [\nabla \Phi(x) \dot{x}]^T dt
% cf. Sec. IV. D.
% Original shape (N_H x N1)
dISigmadPhidx = Sigmaxu * (dPhix * dxs)';
% Reshaped into an N_H*N1-dimensional vector
dISigmadPhidx = reshape(dISigmadPhidx, N_H*N1, 1);

% For ISigmaQ = \int_{0}^{t_f} \Sigma(x,u) * Q(x) dt
dISigmaQ = Sigmaxu * Qx;

% For ISigmaR = \int_{0}^{t_f} \Sigma(x,u) * u^T R u dt
dISigmaR = Sigmaxu * Rxu;


% ***********************
%       
% PACKAGE STATE DERIVATIVE OUTPUT
%
% x_sim =   [   x                   R^n
%               IPhiPhi             R^{N_1^2}
%               ISigmaSigma         R^{N_H^2}
%               ISigmadPhidx        R^{N_H * N_1}
%               ISigmaQ             R^{N_H}         
%               ISigmaR             R^{N_H}         ]
%
xdot = [    dxs
            dIPhiPhi
            dISigmaSigma
            dISigmadPhidx
            dISigmaQ
            dISigmaR        ];
        
        
%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE WEIGHT DYNAMICS FOR VI PHASE: CRITIC NN WEIGHTS c_s,
% HAMILTONIAN NN WEIGHTS w_s, v_s
%
% We shall use ode45 for weight simulation, NOT the forward-Euler
% approximation of the updates (13) and (14) found at the beginning of Sec.
% IV. D.
%
% The Hamiltonian NN weight update is as follows:
%
%   wv_s = K_\sigma^{-1}(t_f) \int_{0}^{t_f} \Sigma(x,u) * 
%                           (d \hat{V}_{N1}(x, c_s) + r) dt
%
% where wv_s = [w_s^T v_s^T]^T is shorthand
%
% The critic NN weight update is as follows:
%
% d/ds c_s = K_\phi^{-1}(t_f) \int_{0}^{t_f} \Phi(x) *
%         \hat{H}_{N_H}(x, \mu_{N_H}(x, wv_s), wv_s) dt
%
% The integral in the above will have to be done manually with the
% pre-existing generated state dynamic data, given that the Hamiltonian
% \hat{H}_{N_H}(x, \mu_{N_H}(x, wv_s), wv_s) is nonlinear in the weights
% wv_s = [v_s^T w_s^T]^T. This is the reason why the associated integral
% could not be performed in the learning phase, as was done with the
% Hamiltonian weights update integral. See odefunct() for a more detailed
% explanation.
%
% *************************************************************************
%
% FINAL STATE PARTITION:
%
% x_sim =   [   c_s                 R^{N_1}  ]
%                               
% 
% *************************************************************************
% *************************************************************************        

function xdot = odefunct_vi(t, x)


% Global variables
global sys;
global u_sett;

tvec = u_sett.tvec;
xmat = u_sett.xmat;

K_phi = u_sett.K_phi;

basis = u_sett.basis;
Q = u_sett.Q;
R = u_sett.R;

tf = u_sett.tf;
x0 = u_sett.x0;

% *********************************************************************
%       
% EXTRACT STATE, BASIS PARAMETERS
%

% Extract the critic parameters
c_s = x;


% ***********************
%       
% BASIS FUNCTIONS
%

% Dimension of critic NN
N1 = basis.Phi.N;

% Dimensions of Hamiltonian NN
N2 = basis.Psi.N;            % Functions associated with actor network
N3 = basis.Theta.N;         % Remaining functions not associated w/ actor
N_H = N2 + N3;               % Total dimension


% *********************************************************************
%       
% HAMILTONIAN NN WEIGHT w_s, v_s UPDATE
%
% cf. Sec. IV. D.
%

% Hamiltonian weights update  
wv_s = update_H_weights(c_s);

% Extract both components of the weight vector. Recall the weight
% vector is partitioned according to the number of basis functions N_2
% chosen in the actor basis {\psi_i(x) * u}_{j=1}^{N_2} and the number
% of basis functions N_3 chosen in the other basis not pertaining to
% the actor {\theta_i(x)}_{j=1}^{N_3}. In sum
%
%   wv_s = [    w_s
%               v_s   ]
%

w_s = wv_s(1:N2);               % R^{N_2}
v_s = wv_s(N2+1:N2+N3);           % R^{N_3}



% *********************************************************************
%       
% CRITIC NN WEIGHT c_s UPDATE
%
% cf. Sec. IV. D.
%


if u_sett.int_H_ode45_1_man_0
    
    % Perform integral 
    % \int_{0}^{t_f} \Phi(x) * \hat{H}_{N_H}(x, \mu_{N2}(x, wv_s), wv_s) dt
    % via ode45

    tspan = [0 tf];
    x0_sim = [  x0
                zeros(N1, 1)    ];      
    [~, x_IPhiH] = ode45(@odefunct_H, tspan, x0_sim);

    % Extract integral
    IPhiH = x_IPhiH(end, end-N1+1:end)';
    

else
    
    % Perform integral 
    % \int_{0}^{t_f} \Phi(x) * \hat{H}_{N_H}(x, \mu_{N2}(x, wv_s), wv_s) dt
    % Manually with pre-existing state trajectory data

    IPhiH = zeros(N1,1);  % Integration variable
    
    % ***********************
    %       
    % INTEGRATION LOOP
    %   
    for k = 1:size(tvec,1)-1

        % Calculate time differential
        dt = tvec(k+1) - tvec(k);

        % Get state vector at current time value
        xs = xmat(k,:)';

        % ***********************
        %       
        % BASIS FUNCTIONS
        %        

        % \Phi(x)
        Phix = eval_phi(xs, basis.Phi);


        % \Sigma(x,u)
        % BEFORE multiplying first N_2 functions on the right by u 
        Psix = eval_phi(xs, basis.Psi);         % First N_2 components
        Thetax = eval_phi(xs, basis.Theta);     % Last N_3 components

        % ***********************
        %       
        % EVALUATE POLICY \mu(x, wv_k)
        %    
        mux = - (1 / 2) * inv(R) * (w_s' * Psix)';


        % Multiply factor of \mu(x, wv_k) in first N_2 elements
        Psixmu = Psix * mux;

        % Combine activation functions into one vector
        Sigmaxmu = [    Psixmu 
                        Thetax  ];                  

        % ***********************
        %       
        % RUNNING COST r(x,\mu_{N_H}(x, wv_k))
        %

        % Evaluate state penalty Q(x)
        Qx = xs' * Q * xs;

        % Evaluate control penalty
        Rxmu = mux' * R * mux;

        % Evaluate running cost r(x,\mu_{N_H}(x, wv_k))
        rxmu = Qx + Rxmu;     

        % ***********************
        %       
        % EVALUATE HAMILTONIAN APPROXIMATION 
        %
        % \hat{H}(x, \mu(x, wv_k), wv_k)
        %
        %       = wv_k^T * \Sigma(x,\mu(x, wv_k)) + R(\mu(x, wv_k))
        %

        H_x_mu = wv_s' * Sigmaxmu + rxmu;


        % ***********************
        %       
        % INCREMENT INTEGRAND
        %    

        IPhiH = IPhiH + Phix * H_x_mu * dt;

    end

end



% ***********************
%       
% WEIGHT DERIVATIVE
%      

dc = K_phi \ IPhiH;

 
% ***********************
%       
% PACKAGE STATE DERIVATIVE OUTPUT
%
% x_sim =   [   c_s                 R^{N_2}  ]
%                
   
xdot = [    dc  ];



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INTEGRATE HAMILTONIAN FOR VI PORTION OF ALGORITHM
%
% This is for performing the integral,
%
% \int_{0}^{t_f} \Phi(x) * \hat{H}_{N_H}(x, \mu_{N2}(x, wv_s), wv_s) dt
%
% which is associated with the critic differential weight update
%
%   d / ds {c(s)}.
%
% The state variables x as well as the Hamiltonain integral above will need
% to be simulated as dynamic variables.
%
% NOTE: The state trajectry generated {x(t)}_{t=0}^{t_f} is the same as
% that generated by the initial learning phase. This separate integral was
% found necessary for numerics purposes.
%
% *************************************************************************
%
% FINAL STATE PARTITION:
%
% x_sim =   [   x           R^n     
%               IPhiH       R^{N_1}     ]
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct_H(t, x)


% Global variables
global sys;
global R;
global Q;
global basis;

% global c_s;
global w_s;
% global v_s;
global wv_s;


% Get system dimensions
n = sys.n;          % Order of system
% m = sys.m;          % Number of inputs

% ***********************
%       
% SYSTEM DYNAMICS
%

% Extract system variables
xs = x(1:n);

% Evaluate system drift dynamics
fx = eval_f(xs, sys);

% Evaluate input gain matrix
gx = eval_g(xs, sys); 

% Evaluate control u(t)
u = uxt_alg(xs, t);

% Evaluate state derivative \dot{x}
dxs = fx + gx * u;


% ***********************
%       
% HAMILTONIAN INTEGRAL
%        

% \Phi(x)
Phix = eval_phi(xs, basis.Phi);


% \Sigma(x,u)
% BEFORE multiplying first N_2 functions on the right by u 
Psix = eval_phi(xs, basis.Psi);         % First N_2 components
Thetax = eval_phi(xs, basis.Theta);     % Last N_3 components

% ***********************
%       
% EVALUATE POLICY \mu(x, wv_k)
%    
mux = - (1 / 2) * inv(R) * (w_s' * Psix)';


% Multiply factor of \mu(x, wv_k) in first N_2 elements
Psixmu = Psix * mux;

% Combine activation functions into one vector
Sigmaxmu = [    Psixmu 
                Thetax  ];                  

% ***********************
%       
% RUNNING COST r(x,\mu_{N_H}(x, wv_k))
%

% Evaluate state penalty Q(x)
Qx = eval_Q(xs, Q);

% Evaluate control penalty
Rxmu = eval_R(xs, mux, R);

% Evaluate running cost r(x,\mu_{N_H}(x, wv_k))
rxmu = Qx + Rxmu;     

% ***********************
%       
% EVALUATE HAMILTONIAN APPROXIMATION 
%
% \hat{H}(x, \mu(x, wv_k), wv_k)
%
%       = wv_k^T * \Sigma(x,\mu(x, wv_k)) + R(\mu(x, wv_k))
%

H_x_mu = wv_s' * Sigmaxmu + rxmu;


% ***********************
%       
% HAMILTONIAN DIFFERENTIAL
%    

dIPhiH = Phix * H_x_mu;


% ***********************
%       
% PACKAGE STATE DERIVATIVE OUTPUT
%
% x_sim =   [   x           R^n     
%               IPhiH       R^{N_1}     ]
%
xdot = [    dxs
            dIPhiH  ];





%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMICS FOR FINAL PORTION OF ALGORITHM
%
% Note now that the dynamic variables associated with the activation
% function integrals no longer need to be simulated, as learning has
% concluded. The state is then simply the dynamic system state.
%
% *************************************************************************
%
% FINAL STATE PARTITION:
%
% x_sim =   [   x           R^n     ]
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct_final(t, x)

% Global variables
global sys;

% Get system dimensions
n = sys.n;          % Order of system
% m = sys.m;          % Number of inputs

% ***********************
%       
% SYSTEM DYNAMICS
%

% Extract system variables
xs = x(1:n);

% Evaluate system drift dynamics
fx = eval_f(xs, sys);

% Evaluate input gain matrix
gx = eval_g(xs, sys); 

% Evaluate policy \mu_{N_2}(x, c_s)
u = uxt_alg(xs, t);

% Evaluate state derivative \dot{x}
dxs = fx + gx * u;


% ***********************
%       
% PACKAGE STATE DERIVATIVE OUTPUT
%
% x_sim =   [   x           R^n     ]
%
xdot = [    dxs    ];


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


function u = uxt_alg(x, t)

% Global variables
global sys;
global u_sett;

noise = u_sett.noise;

u_0 = u_sett.u_0;



% Get system dimensions
% n = sys.n;          % Order of system
m = sys.m;


% *********************************************************************
%
% LEARNING PHASE -- APPLY PROBING NOISE
%
% *********************************************************************


% Evaluate noise e(t)
et = eval_noise(t, m, noise);

% Evaluate control signal u(t)
u = eval_u(x, t, u_0) + et;    


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% HAMILTONIAN WEIGHT UPDATE
%
% See (14), T. Bian and Z.-P. Jiang.
%
% The Hamiltonian NN weight update is as follows:
%
%   wv_s = K_\sigma^{-1}(t_f) \int_{0}^{t_f} \Sigma(x,u) * 
%                           (d \hat{V}_{N1}(x, c_s) + r) dt
%
% where wv_s = [w_s^T v_s^T]^T is shorthand.
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


function wv_s = update_H_weights(cs)

% Global variables
global u_sett;

K_sigma = u_sett.K_sigma;
ISigmadPhidx = u_sett.ISigmadPhidx;


wv_s = K_sigma \ (ISigmadPhidx * cs);



