function out_data = alg_cfvi_tracking(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% cFVI NONLINEAR TRACKING
%
% [ ***** ANONYMIZED ***** ]
%
% 2023-03-15
%
% This program, given a trained cFVI controller, implements the controller
% in a tracking control scheme.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings              (Struct) contains algortithm settings
%                           configured in config_preset.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       tvec                ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%       xmat                ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       umat                ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       inds                (Struct) Contains the state vector partition
%                           for the numerical simulation conducted by
%                           ode45. E.g., to find where the plant states are
%                           in the ode45 state vector, query inds.indsx.
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


% Reference signal r(t) settings
global r_sett;

% Do nominal min-phase model (=1) or nonmin-phase model (=0)
global u_sett;

        

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
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;

% Nominal, perturbed models
model = model_cell{indnom};                  % System model

% Evaluate FVI via table lookup (=1) or Python call (=0)  
tbl1_py0 = group_settings.tbl1_py0;

% Algorithm tag
algtag = alg_settings.algtag;

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;

% Simulation length
tsim = alg_settings.tsim;

% ***********************
%       
% TRIM CONDITIONS
%    

% Nominal model
model_nom_tag = alg_settings.model_nom_tag;
model_nom_ind = alg_settings.model_nom_ind;
model_nom = model_cell{model_nom_ind};
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Simulation model
model_sim_tag = alg_settings.model_sim_tag;
model_sim_ind = alg_settings.model_sim_ind;
model_sim = model_cell{model_sim_ind};
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;


% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;


% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end

% Check if max stepsize specified
hasmaxstep = isfield(alg_settings, 'maxstep');
if hasmaxstep
    maxstep = alg_settings.maxstep;
end

% ***********************
%       
% CONTROLLER
%

% Policy lookup table, grid vectors
if tbl1_py0
    u_sett.xgridvec_cell = alg_settings.xgridvec_cell;
    u_sett.u_tbl = alg_settings.u_tbl;
else
    % Relative path to python code
    relpath_py_code = master_settings.relpath_py_code;
    abspath = master_settings.abspath;
    relpath_data_eval = master_settings.relpath_data_eval;    
    % System tag for python
    systag_py = master_settings.systag_py;
    % Sample frequency 
    fs = alg_settings.fs;
    % Return sample frequency
    fs_return = alg_settings.fs_return;
    % RNG seed to evaluate
    seed = alg_settings.seed;
    % Iteration to evaluate
    iter = alg_settings.iter;    
end


% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initial condition
x0 = alg_settings.x0;  

x0_sim = x0;

% Append integrator ICs
if hasintaug
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

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

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
if hasintaug
    inds.indsz = indsz;
end
if pf1nopf0
    inds.indspf = indspf;
end
u_sett.inds = inds;

% ***********************
%       
% CONTROL SETTINGS
%   

% Store control tag
u_sett.tag = 'cfvi_lookup';

% System dimensions
u_sett.n = n;
u_sett.m = m;

% Has integral augmentation (=1) or not (=0)
u_sett.hasintaug = hasintaug;

% Reference command settings
u_sett.r_sett = r_sett;

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

% Store trim state, control for control calculations
u_sett.xe_ctrl = xe_nom;
u_sett.ue_ctrl = ue_nom;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Indices of state variables to be tracked
inds_xr = model.inds_xr;
u_sett.inds_xr = inds_xr;

% Coordinate transformations
sxd = model.lin.io.sxd;
syd = model.lin.io.syd;
u_sett.su = model.lin.io.sud;
u_sett.sx = sxd;
u_sett.sy = syd;


% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end

% *************************************************************************
% 
% PYTHON COMMAND CALL (IF APPLICABLE)
% 
% *************************************************************************

if ~tbl1_py0

    % Begin construction of the command to run the script
    cmdstr = ['python "' relpath_py_code 'value_iteration/eval_policy.py"'];
    
    % Options: Evaluation mode is single simulation
    cmdstr = [cmdstr ' -eval_mode="single"'];

    % Convert the coords to FVI coords
    tx0 = x0 - xe_nom;
    tx0p = sxd * tx0;
    if hasintaug
        tx0pz = [tx0p; zeros(m, 1)];
    else
        tx0pz = tx0p;
    end

    % Options: IC x_0
    x0_str = [];
    for i = 1:length(tx0pz)
        x0_str = [x0_str num2str(tx0pz(i))];
        if i < length(tx0pz)
            x0_str = [x0_str ', '];
        end
    end
    cmdstr = [cmdstr ' -x0="' x0_str '"'];

    % Options: Path to file data
    model_file = [relpath_py_code 'data/' systag_py '/' algtag ...
        '_seed_' num2str(seed, '%03.f') ...
        '_step_' num2str(iter, '%03.f') '.torch'];
    cmdstr = [cmdstr ' -file_data="' model_file '"'];

    % Perturbation parameters
    nuvec = model_sim.nuvec;
    dtheta = nuvec - 1;
    switch systag
        case sysnames.pendulum
            % \theta = [m, L]
            dtheta = [0; dtheta * model.L];
    end

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
        '_nu_' nuvec_filename '_sim_data.mat'];
    file_out = [abspath relpath_data_eval filename_out];
    cmdstr = [cmdstr ' -file_out="' file_out '"'];
    
    % Options: Simulation frequency and return frequency
    cmdstr = [cmdstr ' -T="' num2str(tsim) '"'];
    cmdstr = [cmdstr ' -fs="' num2str(fs) '"'];
    cmdstr = [cmdstr ' -fs_return="' num2str(fs_return) '"'];

    % Options: Reference command
    rt_Avec = syd * r_sett.Arvec;
    rt_Avec_str = [];
    for i = 1:length(rt_Avec)
        rt_Avec_str = [rt_Avec_str num2str(rt_Avec(i))];
        if i < length(rt_Avec)
            rt_Avec_str = [rt_Avec_str ', '];
        end
    end
    cmdstr = [cmdstr ' -rt_Avec="' rt_Avec_str '"'];

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
    end

end

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

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

% *************************************************************************
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% RUN SIMULATION
%

% Time span for simulation
tspan = [0, tsim];

% Run simulation
if tbl1_py0
    if ~hasmaxstep
        [t, x] = ode45(@odefunct, tspan, x0_sim);
    else
        opts = odeset('MaxStep', maxstep);
        [t, x] = ode45(@odefunct, tspan, x0_sim, opts);
    end
else

    % EXECUTE PYTHON SCRIPT
    [status, evalout] = system(cmdstr);

    % Print error messages if any
    if ~isempty(evalout)
        disp(evalout)
    end
    
    % Extract data
    outdata = load(file_out);
    x = outdata.x;
    umat = outdata.u;
    t = outdata.tvec;


end


% ***********************
%       
% STORE DATA
%

% Store time data
tvec = [    tvec
            t       ];


% Store system state data
xmat = [    xmat
            x       ];


% DEBUGGING: Print done simulating
disp('***** SIMULATION COMPLETE *****')



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
% CONTROL SIGNAL
% 
% *************************************************************************

if tbl1_py0
% Initialize empty matrix
umat = zeros(size(tvec,1), m);

% Calculate control
for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    u = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

end
end

% Store control signal
out_data.umat = umat;

% *************************************************************************
%
% TIME, STATE
% 
% *************************************************************************

% Time vector
out_data.tvec = tvec;

% Rescale integrator states to reflect original units
if hasintaug
    xmat(:,indsz) = (inv(syd) * xmat(:,indsz)')';
end

% If a prefilter was used, rescale and shifted filtered reference command
% responses to reflect trim (pre-transformation)
if pf1nopf0
   xmat(:,indspf) = (inv(u_sett.sy) * xmat(:,indspf)')' ...
       +  [xe_sim(inds_xr(1)); xe_sim(inds_xr(2))]';
   out_data.xmat = xmat;
end

% Store modified state vector
out_data.xmat = xmat;

% Indices of how state vector is partitioned
out_data.inds = inds;

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
% CALCULATE DYNAMICS 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get indices of state partition
inds = u_sett.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;

% Initialize empty state derivative vector
xdot = zeros(u_sett.n_sim, 1);

% % Get number of states in simulation plant
% nd = u_sett.nd;

% Get coordinate transformations
sy = u_sett.sy;

% Extract plant states
xp = x(inds.indsx);


% Get equilibrium point x_e (pre-transformation) -- nominal system
xe = u_sett.xe_nom;

% Get equilibrium control u_e (pre-transformation) -- nominal system
ue = u_sett.ue_nom;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_sim.lin.Ap * tx;
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett.lin1nonlin0
    % System linear
    g_x = model_sim.lin.Bp;
else
    % System nonlinear
    g_x = model_sim.gx(xp);
end

% Calculate control signal
u = uxt_alg(x, t);

% If model is linear, apply \tilde{u} = u - u_{e} (pre-transformation)
if u_sett.lin1nonlin0
    tu = u - ue;
end

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (post-transformation)
yr = rt(:,1);
yrp = sy * yr;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(inds.indspf);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if hasintaug
    if u_sett.pf1nopf0
        zdot = -(trfp - typ);
    else
        zdot = -(trp - typ);
    end
    % Append integral augmentation state derivatives
    xdot(inds.indsz) = zdot;
end

% State derivatives
if u_sett.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;


% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot(inds.indspf) = pfdot;
end       



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function u = uxt_alg(x, t)

% % Global variables
global u_sett;


% Evaluate control
u = eval_u(x, t, u_sett);



