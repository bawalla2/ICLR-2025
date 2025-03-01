function [Jxmat, out_data] = alg_eval_cost(alg_settings, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE INFINITE HORIZON COST OF CONTROL ALGORITHM
%
% [ ***** ANONYMIZED ***** ]
%
% 2022-11-14
%
% This program, given a system, controller, initial condition x_0,
% termination time T > 0, and cost structure Q, R, integrates the cost
%
%   J(x_0) = \int_{0}^{T} x^T Q x + u^T R u dt
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
clear global

global sys;

global normeps;

% Control settings
global u_sett_eval;



% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% System names
sysnames = master_settings.sysnames;

% Current system being executed
systag = master_settings.systag;

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
nlq = sys.nlq;                          % System order for LQ servo
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% Use pre-filter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end

% Nominal, perturbed models
model = model_cell{indnom};                  % System model

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Integration time interval T
T = alg_settings.T;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% Threshold \epsilon > 0 such that simulation terminates when ||x|| <
% \epsilon
donormeps = alg_settings.donormeps;
normeps0 = alg_settings.normeps;

% Algorithm tag
algtag = alg_settings.algtag;

% Extract algorithm names
algnames = master_settings.algnames;

% Nominal model
model_nom_ind = alg_settings.model_nom_ind;
model_nom = model_cell{model_nom_ind};
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = model_cell{model_sim_ind};
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;

% Extract system plot settings
sys_plot_settings = master_settings.sys_plot_settings;

% Extract threshold matrix
threshmat_x = sys_plot_settings.threshmat_x;
numthresh_x = size(threshmat_x, 1);

% ***********************
%       
% STATE VECTOR PARTITION INDICES
%   

% Indices of how state vector is partitioned
indsxr = model.inds_xr;

% Vehicle states
cnt = 1;
len = n;
indsx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Integral augmentation states
if hasintaug
    len = m;
    indsz = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Pre-filter states
if pf1nopf0
    len = m;
    indspf = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Cost integral J(x)
len = 1;
indsJ = (cnt:cnt+len-1)';
cnt = cnt + len;

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett_eval.n_sim = n_sim;

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
if hasintaug
    inds.indsz = indsz;
end
if pf1nopf0
    inds.indspf = indspf;
end
inds.indsJ = indsJ;
u_sett_eval.inds = inds;



% % DEBUGGING: Print done loading
% disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% Time span for simulation
tspan = [0, T];

% Options -- terminate algorithm when norm of state goes below threshold
if donormeps
    odeopt = odeset('Events', @normevent);
end

% ***********************
%       
% CONTROL SETTINGS
%   

% Tag
u_sett_eval.tag = alg_settings.utag;

% System dimensions
u_sett_eval.nlq = nlq;
u_sett_eval.m = m;

% Model linear (=1) or nonlinear (=0)
u_sett_eval.lin1nonlin0 = lin1nonlin0;

% Use pre-filter (=1) or not (=0)
u_sett_eval.pf1nopf0 = pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett_eval.pfavec = pfavec;   
end

% Has integral augmentation (=1) or not (=0)
u_sett_eval.hasintaug = hasintaug;

% Coordinate transformations
sx = model.lin.io.sxd;
sy = model.lin.io.syd;
u_sett_eval.su = model.lin.io.sud;
u_sett_eval.sx = sx;
u_sett_eval.sy = sy;
% Transform [z, y, x_r] -> [x_1, x_2, x_3]
Sxirl_x3 = model.lin.io.Sxirl_x3;

% Store nominal model in global settings
u_sett_eval.model_nom = model_nom;
u_sett_eval.xe_nom = xe_nom;
u_sett_eval.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett_eval.model_sim = model_sim;
u_sett_eval.xe_sim = xe_sim;
u_sett_eval.ue_sim = ue_sim;

% Q, R
u_sett_eval.Q = alg_settings.Q;
u_sett_eval.R = alg_settings.R;
u_sett_eval.Sxirl_x3 = Sxirl_x3;

% Get if is FVI (=1) or not (=0)
isfvi = strcmp(algtag, algnames.cfvi) || strcmp(algtag, algnames.rfvi);

% ***********************
%       
% ALGORITHM-SPECIFIC CONTROL SETTINGS
%  

switch algtag

    case algnames.mi_dirl

        % Policy linear (=1) or nonlinear (=0)
        linpol1nonlinpol0 = alg_settings.linpol1nonlinpol0;
        u_sett_eval.linpol1nonlinpol0 = linpol1nonlinpol0;

        % Controller
        if linpol1nonlinpol0
            u_sett_eval.K = alg_settings.K;
        else
            u_sett_eval.P = alg_settings.P;
            u_sett_eval.gx = model_nom.gx;
        end

        % Equilibrium state, control for control calculations
        u_sett_eval.xe_ctrl = xe_sim;
        u_sett_eval.ue_ctrl = ue_sim;


    case {algnames.cfvi; algnames.rfvi}

        % Get if to evaluate via table lookup (=1) or Python call (=0)
        tbl1_py0 = alg_settings.tbl1_py0;
           
        if tbl1_py0
            % Policy lookup table, grid vectors
            u_sett_eval.xgridvec_cell = alg_settings.xgridvec_cell;
            u_sett_eval.u_tbl = alg_settings.u_tbl;
    
            % Equilibrium state, control for control calculations
            u_sett_eval.xe_ctrl = xe_nom;
            u_sett_eval.ue_ctrl = ue_nom;
        else
            % Relative path to Python code
            relpath_py_code = master_settings.relpath_py_code;
            systag_py = master_settings.systag_py;
            abspath = master_settings.abspath;
            relpath_data_eval = master_settings.relpath_data_eval;
            % RNG seed to evaluate
            seed = alg_settings.seed;
            % Iteration to evaluate
            iter = alg_settings.iter;
            % Simulation frequency and return frequency
            fs = alg_settings.fs;
            fs_return = alg_settings.fs_return;
            % Perturbation parameters
            nuvec = model_sim.nuvec;
            dtheta = nuvec - 1;
            switch systag
                case sysnames.pendulum
                    % \theta = [m, L]
                    dtheta = [0; dtheta * model.L];
            end
        end

    % THROW ERROR OTHERWISE
    otherwise

        error(['EVALUATION OF COST J(X) -- ALG TAG     ' algtag ...
            '     NOT RECOGNIZED']);
end

% ***********************
%       
% ICS
%

% Initial condition matrix
x0mat = alg_settings.x0mat; 

% Size of IC matrix
nx0r = size(x0mat, 1);
nx0c = size(x0mat, 2);



% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Stores integrated cost J(x_0) at each IC x_0
Jxmat = zeros(nx0r, nx0c);

% Stores success percentages
successmat_x = zeros(nx0r, nx0c, numthresh_x);

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% RUN SIMULATION
% 
% *************************************************************************
% *************************************************************************

% Check whether to run ode45 (=1) or not (=0)
if isfvi 
    runode = tbl1_py0;
else
    runode = 1;
end

if runode
for i = 1:nx0r 

    % Initialize empty vector to store mean final state norm over this
    % value of x_1(0) swept
    normxtfvec = zeros(nx0c,1);

    % Start timer
    t0 = tic;

    for j = 1:nx0c

        % Initialize empty IC vector
        x0_sim = zeros(n_sim,1);

        % Get current IC
        x0 = squeeze(x0mat(i,j,:));

        % Determine threshold \epsilon > 0 such that simulation terminates
        % when ||x|| < \epsilon
        if norm(x0) > 2 * normeps0
            normeps = normeps0;
        else
            normeps = norm(x0) / 5;
        end
    
        % Set current IC
        x0_sim(indsx) = x0;
        if pf1nopf0
            x0_sim(indspf) = sy * (x0(indsxr) - xe_nom(indsxr));
        end        
        
        % Run simulation
        if donormeps
            [~, x] = ode45(@odefunct_eval_cost, tspan, x0_sim, odeopt);
        else
            [~, x] = ode45(@odefunct_eval_cost, tspan, x0_sim);         
        end
 
        % Final state x(t_f)
        xend = x(end,:)';

        % Final plant state -- post-transformation, shifted to trim
        txp = sx * (xend(indsx) - xe_nom);

        % Check thresholds
        for ti = 1:numthresh_x
            currthreshind = threshmat_x(ti, 1);
            currthresh = threshmat_x(ti, 2);
            successmat_x(i, j, ti) = txp(currthreshind) <= currthresh && ...
                txp(currthreshind) >= -currthresh;
        end

        % Store cost
        Jxmat(i,j) = xend(indsJ);

        % Store norm of final state
        if hasintaug
            xtf = [ xend(indsz)
                    xend(indsx) - xe_sim ];
        else
            xtf = xend(indsx) - xe_sim;
        end
        normxtfvec(j) = norm(xtf);

    end

    % Stop timer
    t1 = toc(t0);

    % DEBUGGING: Display current IC in x_1
    disp(['***** EVALUATING COST J(x) FOR ALG --      ' algtag ...
        '   --      IC x_1 COUNT:   ' num2str(i) '      OUT OF      ' ...
        num2str(nx0r)])   

    % DEBUGGING: Display mean cost
    disp(['     *** MEAN COST J(x)' ...
        '   =   ' num2str(mean(Jxmat(i,:)))])

    % DEBUGGING: Display mean final state norm
    disp(['     *** MEAN FINAL STATE NORM ||(z, x_p)(t_f)|| WRT TRIM' ...
        '   =   ' num2str(mean(normxtfvec))])

    % DEBUGGING: Display mean final state norm
    disp(['     *** BATCH ELAPSED TIME' ...
        '   =   ' num2str(t1) ' s'])    
end
else

% Else, this is an FVI algorithm and it is desired to evaluate the cost in
% Python.

% Begin construction of the command to run the script
cmdstr = ['python "' relpath_py_code 'value_iteration/eval_policy.py"'];

% Options: Evaluation mode is 2D grid
cmdstr = [cmdstr ' -eval_mode="grid_2D"'];

% Options: Path to file data
model_file = [relpath_py_code 'data/' systag_py '/' algtag ...
    '_seed_' num2str(seed, '%03.f') ...
    '_step_' num2str(iter, '%03.f') '.torch'];
cmdstr = [cmdstr ' -file_data="' model_file '"'];

% % Model perturbations
cmdstr = [cmdstr ' -theta_mode="const"'];
dtheta_str = [];
for i = 1:length(dtheta)
    dtheta_str = [dtheta_str num2str(dtheta(i))];
    if i < length(dtheta)
        dtheta_str = [dtheta_str ', '];
    end
end
cmdstr = [cmdstr ' -dtheta="' dtheta_str '"'];

% Options: Where to save the output data to
nuvec_filename = num2filename(nuvec);
filename_out = [algtag '_' systag_py ...
    '_nu_' nuvec_filename '_return_data.mat'];
file_out = [abspath relpath_data_eval filename_out];
cmdstr = [cmdstr ' -file_out="' file_out '"'];

% Options: Simulation frequency and return frequency
cmdstr = [cmdstr ' -T="' num2str(T) '"'];
cmdstr = [cmdstr ' -fs="' num2str(fs) '"'];
cmdstr = [cmdstr ' -fs_return="' num2str(fs_return) '"'];

% Options: Delete (x, u) data (to avoid memory issues)
cmdstr = [cmdstr ' -delete_xu_data'];

% Gridpoints
cmdstr = [cmdstr ' -grid_2D_npts="' num2str(nx0r) ', ' num2str(nx0c) '"'];

% Options: Pre-filter
if pf1nopf0
    pfavec_str = [];
    for i = 1:length(pfavec)
        pfavec_str = [pfavec_str num2str(pfavec(i))];
        if i < length(pfavec)
            pfavec_str = [pfavec_str ', '];
        end
    end
    cmdstr = [cmdstr ' -pfavec="' pfavec_str '"'];
    cmdstr = [cmdstr ' -pfICtype="x0"'];
end

% EXECUTE PYTHON SCRIPT
[status, evalout] = system(cmdstr);

if ~isempty(evalout)
    disp(evalout)
end

% Extract data
outdata = load(file_out);
Jxmat = - outdata.R;

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

% % Cost data
% out_data.Jxmat = Jxmat;

out_data = [];



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
% EVENT HANDLING -- TERMINATE ode45 WHEN STATE NORM GOES BELOW THRESHOLD
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [value, isterminal, direction] = normevent(t, x)

% Global variables
global u_sett;
global normeps;

% Get indices of state partition
inds = u_sett.inds;

% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Extract the integral augmentation states z (post-transformation)
z = x(inds.indsz);

% Concatenate (x_p, z)
txz = [tx; z];

% CONDITION CHECK
value = norm(txz) < normeps;      % Check if norm < \epsilon
isterminal = 1;                 % Stop the integration
direction  = 0;

