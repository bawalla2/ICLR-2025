function out_data = alg_radp(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ROBUST ADAPTIVE DYNAMIC PROGRAMMING (RADP) ALGORITHM -- MATCHED
% UNCERTAINTY
%
% [ ***** ANONYMIZED ***** ]
%
% 2022-01-17
%
% This program implements the RADP algorithm (with matched uncertainty)
% presented in,
%
%   Y. Jiang and Z.-P. Jiang. "Robust adaptive dynamic programming and
%   feedback stabilization of nonlinear systems." IEEE Transactions on
%   Neural Networks and Learning Systems, 25:882-893, 2014.
%
% *** NOTE:
%
% This program was developed to run single-input systems (m = 1) ONLY. 
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_radp_matched(alg_settings)
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
%       .Phi               (Struct) contains critic NN activation function
%                           parameters (cf. eqn. (13)). Has fields:
%           .tag            (String) tag of desired activation functions.
%           .N              (Integer) number of critic NN activation
%                           functions N1.
%       .Psi               (Struct) contains actor NN activation function
%                           parameters (cf. eqn. (14)). Has fields:
%           .tag            (String) tag of desired activation functions.
%           .N              (Integer) number of actor NN activation
%                           functions N2.
%   noise                   (Struct) contains info for probing noise. Has
%                           the following fields:
%       .tag                (String) tag of specific probing noise signal
%                           to be injected (see eval_noise.m for options).
%   l                       (Integer) number of samples to collect for each
%                           least-squares minimization (cf. eqn. (15)).
%   tf                      (Double) Length of learning window [0, t_f]
%                           (sec).
%   u_0                     (String) Tag corresponding to the initial
%                           stabilizing policy u_0(x) (cf. Assumption 3.2).
%                           Evaluation of this policy is handled by
%                           eval_u.m, so the policy needs to be encoded
%                           there.
%   x0                      (n-dimensional vector) ICs for known x-dynamics
%                           x(t_0)..
%   w_0                     (N2-dimensional vector) ICs for actor NN
%                           weights.
%   istar                   (Integer) Number of iterations to run
%                           algorithm for before manually terminating.
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
%       .xmat               ('simlength' x (p+n+1) matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%           xmat(k,:) =   [     w(tvec(k))   R^p
%                               x(tvec(k))   R^n   ]^T
%       .umat               ('simlength' x 1 matrix) Matrix (vector,
%                           really) whose row indexes the time instants
%                           specified in .tvec, and whose 1 column is the
%                           control signal u(t) at the respective time
%                           instant.
%       .istar              (Integer) final termination index of the PI
%                           algorithm.
%       .c_mat            ('istar'+1 x N1 matrix) Matrix whose row
%                           indexes the PI index, and whose N1-columns are
%                           the critic NN weights c_i at the respective PI
%                           index i.
%       .w_mat            ('istar'+1 x N2 matrix) Matrix whose row
%                           indexes the PI index, and whose N2-columns are
%                           the actor NN weights w_i at the respective PI
%                           index i.
%       cond_A_vec          ('istar'-dim. vector) The i-th index of
%                           this vector contains the condition number of
%                           the matrix involved in performing the
%                           least-squares minimization associated with the
%                           i-th iteration weight update.
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

% System dimensions
n = master_settings.sys.n;         % Order of known x-dynamics

Q = alg_settings.Q;
R = alg_settings.R;

noise = alg_settings.noise;

u_0 = alg_settings.u_0;

l = alg_settings.l;

% Simulation times
tf = alg_settings.tf;

% Number of iterations
istar = alg_settings.istar;


% ***********************
%       
% BASIS SETTINGS
%
% See description for details.
%

% Basis struct
basis = alg_settings.basis;

% Critic basis for approximating cost function V_i(x) (cf. eqn. (13))
N1 = alg_settings.basis.Phi.N;    
% Actor basis for approximating policy u_{i + 1}(x) (cf. eqn. (14))
N2 = alg_settings.basis.Psi.N;



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
% following three types of dynamic variables will be necessary for
% simulation:
%
%   System states (including w, x, cf. eqns. (8)-(9))
%   Integrals associated with the least squares minimization (15)
%

% Initial conditions
xs0 = alg_settings.x0;


% Initial condition for simulation. See odefunct() for a description of the
% state partition.
x0_sim = [  xs0
            zeros(N2^2, 1)
            zeros(N2, 1);
            0               ];   


% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initialize critic NN weight vector
c_i = zeros(N1, 1);     % Holds critic NN weights of current iteration

% Initialize actor NN weight vector
w_i = alg_settings.w_0; % Holds actor NN weights of current iteration   

% % % w_i = [-5; 1];

% w_i = [0; -2; 0; -1];
% w_i = [-1; -2; -1/2; -1] * 5;

% c = [1; 0; 4];
% w_i = [-c(2); -2*c(3); -c(2)/2; -c(3)];

% Initialize vector to store previous iteration critic NN weights
c_im1 = zeros(N1, 1);   % Holds critic NN weights of previous iteration
c_im1(1) = Inf;         % So PI algorithm starts properly

% % Initialize vector to store previous iteration actor NN weights
% w_im1 = zeros(N2, 1);   % Holds critic NN weights of previous iteration
% w_im1(1) = Inf;         % So PI algorithm starts properly

% Sample collection window T
T = tf / l;

% Initialize PI index
i = 1;

% Initialize vector to hold condition number at each iteration
cond_A_vec = [];


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

% Set initial weights
u_sett.c_i = c_i;
u_sett.w_i = w_i;

% Set initial policy
u_sett.u_0 = u_0;

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% ***********************
%       
% LEAST SQUARES MINIMIZATION VARIABLES
%
% The dynamic variables
%
%   IPsiPsi(k)   R^{N_2^2}
%   IPsiupd(k)    R^{N_2}
%   IQ(k)          R^1      
%
%       0 <= k <= l-1
%
% (see odefunct() for a description of these variables) need to be stored
% in corresponding l-row matrices (or l-dimensional vectors) for subsequent
% least squares minimization at the end of the current PI iteration. These
% storage variables are initialized as zeros here.
%

IPsiPsi_mat = zeros(l,N2^2);
IPsiupd_mat = zeros(l,N2);
IQ_vec = zeros(l,1);


% In addition, the non-dynamic variables
%
%   d_Phi(k) = \Phi_{1}(x(t_{k+1})) - \Phi_{1}(x(t_k))       R^{N_1}
%
%       0 <= k <= l-1
%
% (see the description of odefunct() for definition of the function
% \Phi_{1}) need to be stored.
%

d_Phi_mat = zeros(l, N1);


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

% Stores actor weights c_i at each iteration of the PI algorithm (iteration
% indexed by row)
c_mat = c_i';

% Stores actor weights w_i at each iteration of the PI algorithm (iteration
% indexed by row)
w_mat = w_i';

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
% PHASE 1, PHASE 2 LEARNING
%
% See steps 1-3 of Algorithm 1, Jiang and Jiang
% 
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% COLLECT SIMULATION DATA
%
% This loop collects the l data points needed to perform the least-squares
% minimization (15).
%  
% *************************************************************************

for k = 0:l-1

    % ***********************
    %       
    % RUN SIMULATION
    %
 
    tspan = k * T + [0, T];

    [t, x] = ode45(@odefunct, tspan, x0_sim);

    % ***********************
    %       
    % EXTRACT STATE DATA
    %         

    % Extract dynamic state variable data at beginning of simulation
    x0 = (x(1,:))';

    % Extract dynamic state variable data at end of simulation
    x1 = (x(size(x, 1),:))';

    % State dynamic variables
    xs0 = x0(1:n);           

    % Unpack state data at end of simulation. See odefunct() for a
    % description of the state partition.
    
    % State dynamic variables
    xs1 = x1(1:n);     

    % Dynamic variables associated with least squares minimizations
    x1_lsq = x1(n+1:end);

    IPsiPsi = x1_lsq(1:N2^2);
    IPsiupd = x1_lsq(N2^2+1:N2^2+N2);
    IQ = x1_lsq(N2^2+N2+1);

    % ***********************
    %       
    % DATA STORAGE -- SYSTEM STATE DATA, DYNAMIC LEAST-SQUARES DATA
    % 

    % Store time data
    tvec = [    tvec
                t       ];

    % Store system state data 
    xmat = [    xmat
                x(:,1:n)    ]; 


    % Store dynamic variables associated with least squares data 
    IPsiPsi_mat(k+1,:) = IPsiPsi';
    IPsiupd_mat(k+1,:) = IPsiupd';
    IQ_vec(k+1) = IQ;


    % ***********************
    %       
    % DATA STORAGE -- REMAINING LEAST SQUARES DATA
    %         

    % Evaluate critic basis functions \Phi_{1}(x) = (\phi_1(x), ...,
    % \phi_{N1}(x)) at x = x(t_k) and x = x(t_{k+1})
    Phix_tk = eval_phi(xs0, basis.Phi);
    Phix_tkp1 = eval_phi(xs1, basis.Phi); 

    % Calculate and store \Phi_{1}(x(t_{k+1})) - \Phi_{1}(x(t_k))
    d_Phi_mat(k+1,:) = (Phix_tkp1 - Phix_tk)';

    % ***********************
    %       
    % PREPARE FOR NEXT SIMULATION
    % 

    % IC for next simulation. System state variables for next
    % simulation are carried over from their final values in this
    % simulation. Integration variables associated with least squares
    % minimizations are reset to zero.  
    %
    % See odefunct() for a description of the state partition.
    %
    x0_sim = [  xs1
                zeros(N2^2, 1)
                zeros(N2, 1);
                0               ];    
         

end


% *************************************************************************
%
% PHASE 1 LEARNING: PI LOOP
%
% *************************************************************************

while i < istar
    
    % DEBUGGING: Show iteration count, norm difference
    disp('*****')
    disp(['i = ' num2str(i) ',     ||c_i - c_{i-1}|| =     ' ...
        num2str(norm(c_i - c_im1))])
    
    % *********************************************************************
    %       
    % LEAST SQUARES MINIMIZATION (15)
    %
    % Defined as in the description of odefunct(), the least squares
    % minimization (15) has the form:
    %
    %   [A1 A2] [ c_i    = - v
    %             w_i ]
    %
    % Expressed in terms of the variables we have declared above, this is:
    %
    % A1 = d_Phi_mat;
    % A2 = 2r * ( IPsiupd_mat - IPsiPsi_mat * kron(w_i, eye(N2)) )
    % v = IQ_vec + IPsiPsi_mat * kron(w_i, w_i)
    %
    
    % Least squares variables
    A1 = d_Phi_mat;
    A2 = 2 * R * ( IPsiupd_mat - IPsiPsi_mat * kron(w_i, eye(N2)) );
    v = IQ_vec + IPsiPsi_mat * R * kron(w_i, w_i);
    
    % Perform least squares
    A = [ A1, A2 ];
    lsq = A\(-v);
    
    % Store condition number of matrix involved in least-squares
    % minimization
    cond_A_vec = [cond_A_vec ; cond(A)];
    
    % Store current w_i as w_{i-1}, extract new least squares solutions
    c_im1 = c_i;
    c_i = lsq(1:N1);
    w_i = lsq(N1+1:end);
    
    % Store c_i, w_i for later plotting
    c_mat =   [   c_mat
                    c_i'        ];
    w_mat =   [   w_mat
                    w_i'        ];
                
    % DEBUGGING: Check problem conditioning
    disp(['Condition Number of "A" for Least Squares:           '...
                num2str(cond(A), 4)])
    disp(['Condition Number of "A^T A" for Least Squares:       '...
                num2str(cond(A'*A), 4)])   
                
    % *********************************************************************
    %       
    % PREPARE FOR NEXT PI ITERATION
    %    
    
    % Increment PI index
    i = i + 1;
    
    
end

% % DEBUGGING: ICs
% x_0 = alg_settings.x0

% DEBUGGING: Final critic weights c_i
c_i

% DEBUGGING: Final actor weights w_i
w_i



            
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

% Trajectory data
out_data.tvec = tvec;
out_data.xmat = xmat;

% Final iteration
out_data.istar = i;

% Weight data
out_data.c_mat = c_mat;
out_data.w_mat = w_mat;

% Condition number data
out_data.cond_A_vec = cond_A_vec;


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), 1);

% Calculate control
for k = 1:size(tvec,1)
    
    % Extract state
    x = xmat(k,:)';

    % Extract system variables
    xs = x; 
    
    % Evaluate control 
    u = uxt_alg(xs, tvec(k));
                
    % Store control
    umat(k) = u;

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
% CALCULATE DYNAMICS FOR LEARNING PORTION OF RADP ALGORITHM
%
% I.e., steps 1-3 of Algorithm 1 (phase-one and phase-two learning)
%
% NOTE: State partitioning for simulation
%
% In order to perform the RADP algorithm with unmatched uncertainty, the
% following two types of dynamic variables will be necessary for
% simulation:
%
%   System states (including w, x, cf. eqns. (8)-(9))
%   Integrals associated with the least squares minimization (15)
% 
% *************************************************************************
%
% DYNAMIC VARIABLES ASSOCIATED WITH LEAST SQUARES MINIMIZATION (15):
%
% The least squares minimization (15) can be written in the form:
%
%   [A1 A2] [ c_i    = - v
%             w_i ]
%
% Where,
%
% A1 \in R^{l x N1},
%
%       [ \phi_1(x(t_1)) - \phi_1(x(t_0))  ... \phi_{N1}(x(t_1)) - \phi_{N1}(x(t_0))  
% A1 =  [                                  ...
%       [ \phi_1(x(t_l)) - \phi_1(x(t_{l-1}))  ... \phi_{N1}(x(t_l)) - \phi_{N1}(x(t_{l-1}))
% 
% A2 \in R^{l x N2},
%
% \hat{v}_i(t) = u(t) + \Delta(w(t), x(t)) - \hat{u}_i(x(t));
%
%           [ \int_{t_0}^{t_1} \phi_1(x) \hat{v}_i dt  ... \int_{t_0}^{t_1} \phi_{N2}(x) \hat{v}_i dt  
% A2 = 2r * [                                  ...
%           [ \int_{t_{l-1}}^{t_l} \phi_1(x) \hat{v}_i dt  ... \int_{t_{l-1}}^{t_l} \phi_{N2}(x) \hat{v}_i dt
% 
% v \in R^{l}  
%
%       [ \int_{t_0}^{t_1} Q(x) + r \hat{u}_i^2(x) dt  
% v =   [                                  ...
%       [ \int_{t_{l-1}}^{t_l} Q(x) + r \hat{u}_i^2(x) dt
%
%
% Define,
%
% \Phi_{j} : R^n -> R^{Nj}, \Phi_{j}(x) = (\phi_1(x), ..., \phi_{Nj}(x)),
% j = 1, 2, 4
%
% IPsiPsi_mat \in R^{l x N2}
%
%                   [ ( \int_{t_0}^{t_1}  kron(\Phi_{2}(x), \Phi_{2}(x)) )^T    ]
% IPsiPsi_mat =   [                   ...                                       ]
%                   [ ( \int_{t_{l-1}}^{t_l} kron(\Phi_{2}(x), \Phi_{2}(x)) )^T ]
%
% Where kron(A,B) is the Kronecker product of two matrices/vectors A, B
% (see Mathworks for description).
%
% Then, it can be verified algebraically that the following identities
% hold:
%           [ (\int_{t_0}^{t_1} \Phi_{2}(x)(z + \Delta) dt)^T      ]
% A2 = 2r * [                  ...                                  ]
%           [ (\int_{t_{l-1}}^{t_l} \Phi_{2}(x)(z + \Delta) dt)^T  ]
%
%    - 2r * IPsiPsi_mat * kron(w_i, eye(N2)) 
%
%   Where w_i is the current actor NN weight vector, i.e., \hat{u}_i(x) =
%   \sum_{j = 1}^{N2} w_i(j) * \phi_j(x),
% 
%
%       [ \int_{t_0}^{t_1} Q(x) dt      ]
% v =   [                  ...          ]  - r * IPsiPsi_mat * kron(w_i, w_i)
%       [ (\int_{t_{l-1}}^{t_l} Q(x) dt ]
%
%
% The above two identities motivate the need for the dynamic variables:
%
%   IPsiPsi       (N2^2-dim.) Associated with the integral of     
%                   kron(\Phi_{2}(x), \Phi_{2}(x))
%   IPsiupd        (N2-dim.) Associated with the integral of     
%                   \Phi_{2}(x)(u + \Delta)
%   IQ              (1-dim.) Associated with the integral of Q(x)
%
%
% *************************************************************************
%
% FINAL STATE PARTITION:
%
% x_sim =   [   x           R^n
%               IPsiPsi   R^{N_2^2}
%               IPsiupd    R^{N_2}
%               IQ          R^1       ]
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;
global u_sett;

% Get system order
n = sys.n;  

% model_nom = u_sett.model_nom;
model_sim = u_sett.model_sim;

basis = u_sett.basis;
Q = u_sett.Q;

% Extract system variables
xp = x(1:n);   

% ***********************
%       
% EVALUATE SYSTEM DYNAMICS
%

% Evaluate drift dynamics -- simulation model
f_x = model_sim.fx(xp);
% Evaluate input gain matrix -- simulation model
g_x = model_sim.gx(xp);

% Evaluate basis functions
% Phix = eval_phi(xs, basis.Phi.tag);
Psix = eval_phi(xp, basis.Psi);

% Evaluate Q(x)
Qx = xp' * Q * xp;

% Evaluate control signal u(x(t)) = \hat{u}_0(x(t)) + e(t)
u = uxt_alg(xp, t);

% Calculate state derivative \dot{x} (cf. eqn. (9)) 
dxs = f_x + g_x * u;

% ***********************
%       
% EVALUATE DYNAMIC VARIABLES ASSOCIATED WITH LEAST SQUARES MINIMIZATION
% (15) (SEE ABOVE)
%

dIPsiPsi = kron(Psix, Psix);
dIPsiupd = Psix * u;
dIQ = Qx;


% ***********************
%       
% PACKAGE STATE DERIVATIVE OUTPUT
%
% x_sim =   [   x           R^n
%               IPsiPsi   R^{N_2^2}
%               IPsiupd    R^{N_2}
%               IQ          R^1         ]
%
xdot = [    dxs
            dIPsiPsi   
            dIPsiupd   
            dIQ             ];



        
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

% *********************************************************************
%
% LEARNING PHASE -- INITIAL STABILIZING POLICY u_0(x) (ASSUMPTION 3.2)
%
% *********************************************************************

% Evaluate noise
et = eval_noise(t, 1, noise);

% Evaluate control
u = eval_u(x, t, u_0) + et;

