function out_data = alg_irl(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INTEGRAL REINFORCEMENT LEARNING (IRL) ALGORITHM
%
% [ ***** ANONYMIZED ***** ]
%
% 2021-11-06
%
% This program implements the IRL algorithm presented in,
%
%   D. Vrabie and F.L. Lewis. "Neural network approach to continuous-time
%   direct adaptive optimal control for partially unknown nonlinear
%   systems." Neural Networks, 22:237-246, 2009.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_irl(alg_settings)
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
%   basis_critic            (Struct) contains activation function basis
%                           parameters. Has the following fields:
%       .tag                (String) tag of the desired activation function
%                           basis to use (see eval_phi.m for options).
%       .N                  (Integer) the integer "N," or the basis
%                           dimension.
%   noise                   (Struct) contains info for probing noise. NOTE:
%                           Not required for this algorithm. Has the
%                           following fields:
%       .tag                (String) tag of specific probing noise signal
%                           to be injected (see eval_noise.m for options).
%                           If no noise is desired, simply enter this as
%                           '0'.
%   T                       (Double) integral reinforcement interval length
%                           (sec).
%   istar                   (Integer) number of policy iterations to
%                           execute before terminating the algorithm.
%   num_sims_per_iter       (Integer) Number of simulations to execute per
%                           iteration of the PI algorithm. NOTE: a
%                           simulation consists of 'l'
%                           data samples, each taken over 'T' seconds. So
%                           each iteration has 'num_sims_per_iter' *
%                           'l' total T-second samples
%                           collected.
%   l     (Integer) number of samples to collect per
%                           simulation.     
%   x0mat                   (Matrix) This matrix can be empty or have up to
%                           'istar' * 'num_sims_per_iter' rows, with
%                           n columns. Each time a new simulation begins,
%                           the initial conditions need to be set. These
%                           ICs can be set manually for as many of the
%                           total 'istar' * 'num_sims_per_iter'
%                           simulations run in the algorithm. If x0mat runs
%                           out of ICs, then the ICs will be generated
%                           either randomly at the beginning of each new
%                           simulation, or will be carried over from the
%                           previous simulation's final values (see
%                           variable x0_behavior for details).
%   x0_behavior             (String) Determines the manner in which ICs are
%                           generated at the beginning of each new
%                           simulation, after the algorithm has run out of
%                           ICs specified in x0mat. Has the options:
%       'rand'              Randomly generate new ICs.
%       'cont'              Continue ICs of new simulation as final values
%                           of previous simulation.
%   c_0                      (N-dimensional vector) ICs for critic NN, where
%                           N is the critic NN basis dimension.
%   tsim                    (Double) time window for post-learning
%                           simulation (sec). I.e., if learning happens
%                           over [0, t_f], post-learning happens over [t_f,
%                           t_f + tsim].
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       .tvec               ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%       .xmat               ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       .umat               ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       .tvec_pi            ('istar' - dimensional vector)
%                           Vector of time instants corresponding to the
%                           sample instants of each new iteration of the PI
%                           algorithm.
%       .c_mat               (N x 'istar' matrix) critic NN weights
%                           at each of the time instants of .tvec_pi.
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

Ts = alg_settings.Ts;

% Holds settings for each loop
loop_cell = alg_settings.loop_cell;

istar = loop_cell{1}.istar;
num_sims_per_iter = 1;
l = loop_cell{1}.l;
        
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

Q = loop_cell{1}.Q;
R = loop_cell{1}.R;

basis_critic = alg_settings.basis_critic;         % Basis struct
N = basis_critic.N;           % Basis dimension

% Probing noise
noise = alg_settings.noise; 

% Post-learning simulation length
tsim = alg_settings.tsim;



% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Matrix of ICs
x0 = alg_settings.x0; 

% IC generation behavior. See above for details.
x0_behavior = 'cont';

% Initial critic NN weights
c_i = alg_settings.c_0;


% Initialize simulation counter
simcount = 1;

% Initialize vector to hold condition number at each iteration
cond_A_vec = zeros(istar, 1);

% ***********************
%       
% CONTROL SETTINGS
%   

% Learning flag
u_sett.islearning = 1;

u_sett.Q = Q;
u_sett.R = R;
u_sett.basis_critic = basis_critic;

u_sett.noise = noise;

u_sett.model_nom = model_nom;
u_sett.model_sim = model_sim;

% Set initial weights
u_sett.c_i = c_i;

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************


% Critic NN weights
c_mat = zeros(istar + 1,N);
c_mat(1,:) = c_i';

% Time vector, state trajectory, control signal
tvec = [];
xmat = [];
umat = [];



% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


for i = 1:istar
% for i = 1:istar - 1          % DEBUGGING

    
    % ***********************
    %       
    % INITIALIZE SAMPLE COUNTER
    %
    % Each iteration of the PI algorithm, 'num_sims_per_iter' *
    % 'l' samples of the state and integral reinforcement
    % are collected for the least squares minimization at the end of the
    % iteration, each data point corresponding to a T-second simulation.
    % This counter is initialized to 1 here and is incremented after each
    % T-second simulation until it will eventually reach the value
    % 'num_sims_per_iter' * 'l'
    %
    samplecount = 1;
    
    % Storage for time data and simulation data at the current PI index i
    tvec_i = [];
    xmat_i = [];
    
    % *********************************************************************
    %
    % OUTER PI LOOP
    %
    % This outer loop corresponds to one iteration of the PI algorithm.
    % 'num_sims_per_iter' simulations are run, and then the critic
    % parameters are updated via least squares.
    %        
    for m = 1:num_sims_per_iter
 
        % ***********************
        %       
        % INITIAL CONDITION FOR SIMULATION
        %
        % Dynamic variables include state x (first n entries) and integral
        % reinforcement ((n+1)-th entry)
        %
        
        % Determine if state IC is to be set manually or by the algorithm
        if simcount <= 1 %size(x0mat, 1)
            % Set IC manually
            xs0 = x0;
        else
            % No more manual ICs
            if (samplecount*i == 1) || (strcmp(x0_behavior, 'rand'))
                % Set IC randomly
                xs0 = 2 * (rand(n, 1) - 1/2);    
            else
                % Carry IC over from previous simulation
                xs0 = x1;
            end
        end
        
        % Total IC vector (consists of state + integral reinforcement)
        x0_sim = [  xs0
                    0   ];
        
        
        % *************************************************************
        %
        % INNERMOST LOOP
        %
        % This inner loop is the one in which the individual
        % simulations are run. One simulation will consist of
        % 'l' T-second simulations, each run
        % separately so that data may be collected at the end of the
        % T-second interval.
        %  
        for s = 1:l
            
            % ***********************
            %       
            % RUN SIMULATION
            %
            
            % Time span for current simulation
            tspan = ...
                Ts * (i - 1) * num_sims_per_iter * l +...
                Ts * [samplecount - 1, samplecount];

            % Run simulation
            [t, x] = ode45(@odefunct, tspan, x0_sim);
          
            % ***********************
            %       
            % DATA MANAGEMENT
            %         
            
            % Extract state data at end of simulation
            x1 = (x(length(x),(1:end-1)))';
            
            % Extract integral reinforcement at end of simulation
            V(samplecount,1) = x(length(x), end);
            
            % Evaluate the basis function difference
            %
            % phi(x(t + T)) - phi(x(t))
            %
            [phix_tpT, ~] = eval_phi(x1, basis_critic);     % phi(x(t + T))
            [phix_t, ~] = eval_phi(x0_sim(1:n), basis_critic);       % phi(x(t))
            phidiff(samplecount, :) = phix_tpT - phix_t;             
            
            % ***********************
            %       
            % DATA STORAGE
            %              

            tvec_i = [  tvec_i
                        t       ];
                    
            xmat_i = [  xmat_i
                        x(:,1:end-1)    ];
            
            % ***********************
            %       
            % PREPARE NEXT SAMPLE
            % 
            
            % IC for next T-second simulation
            % State is carried over from previous T-second simulation,
            % integral reinforcement is reset
            x0_sim = [ x1 ; 0];            
            
            % Increment sample counter
            samplecount = samplecount + 1;
            
        end
        
        % ***********************
        %       
        % PREPARE NEXT SIMULATION
        %
        
        % Increment simulation counter
        simcount = simcount + 1;
               
    end

    % ***********************
    %       
    % CALCULATE CONTROL SIGNAL APPLIED FOR THIS ITERATION
    %      
    for k = 1:size(tvec_i, 1)
       
        % Get time
        tk = tvec_i(k);
        
        % Get system state
        xs = xmat_i(k,:)';
        
        % Evaluate control signal u(t)
        u = uxt_alg(xs, tk);
        
        % Store control signal
        umat = [    umat
                    u'      ];
        
    end
    
    % ***********************
    %       
    % LEAST SQUARES MINIMIZATION TO DETERMINE CRITIC NN WEIGHTS FOR NEXT
    % ITERATION
    %
    
    c_im1 = c_i;
    
    % Least-squares minimization
    A = - phidiff;
    c_i = A \ V;

    % Update weights
    u_sett.c_i = c_i;

    % Store new weight
    c_mat(i + 1,:) = c_i';
    
    % Store condition number of matrix involved in least-squares
    % minimization
    cond_A_vec(i) = cond(A);
    
    % DEBUGGING: Show iteration count, norm difference
    disp('*****')
    disp(['i = ' num2str(i) ',     ||c_i - c_{i-1}|| =     ' ...
        num2str(norm(c_i - c_im1))])    
    
    % DEBUGGING: Check problem conditioning
    disp(['Condition Number of "A" for Least Squares:           '...
                num2str(cond(A), 4)])
%     disp(['Condition Number of "A^T A" for Least Squares:       '...
%                 num2str(cond(A'*A), 4)])

    % DEBUGGING: Print critic NN params
    c_i


    % ***********************
    %       
    % DATA STORAGE
    %              

    tvec = [    tvec
                tvec_i      ];

    xmat = [    xmat
                xmat_i      ];    
    
end




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

% Time, state, control data
out_data.tvec = tvec;
out_data.xmat = xmat;
out_data.umat = umat;

% Time indices corresponding samples of new PI iterations.
tvec_pi = (0:1:istar) * Ts * num_sims_per_iter * l;

% Weight data
out_data.tvec_pi = tvec_pi;
out_data.c_mat = c_mat;

% Condition number data
out_data.cond_A_vec = cond_A_vec;


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
% CALCULATE DYNAMICS -- LEARNING PHASE
%
% State consists of system state x (n-dimensional), plus the integral
% reinforcement (appended at the (n+1)-th entry).
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

% model_nom = u_sett.model_nom;
model_sim = u_sett.model_sim;

% Dynamic state variables
xp = x(1:end-1);

% Evaluate drift dynamics -- simulation model
f_x = model_sim.fx(xp);
% Evaluate input gain matrix -- simulation model
g_x = model_sim.gx(xp);

% Calculate control signal
u = uxt_alg(xp, t);

% Calculate state derivative
xdot = [    f_x + g_x * u
            xp' * Q * xp + u' * R * u    ];
        

                
        
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

function u = uxt_alg(xp, t)

% Global variables
global sys;

global u_sett;

% Q = u_sett.Q;
R = u_sett.R;

model_nom = u_sett.model_nom;
% model_sim = u_sett.model_sim;

% Get system dimensions
% n = sys.n;          % Order of system
m = sys.m;

basis_critic = u_sett.basis_critic;
noise = u_sett.noise;

% Get weights
c_i = u_sett.c_i;


% *********************************************************************
%
% LEARNING PHASE -- APPLY PROBING NOISE
%
% *********************************************************************

% Evaluate noise
et = eval_noise(t, m, noise);

% Evaluate input gain matrix -- nominal model
g_x_nom = model_nom.gx(xp);

% Evaluate basis functions and gradient
[~, dphix] = eval_phi(xp, basis_critic);

% Calculate control signal
u = - 1/2 * inv(R) * g_x_nom' * dphix' * c_i + et;

