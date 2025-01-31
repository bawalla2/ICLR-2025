function out_data = alg_eval_trained_pol(alg_settings, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE TRAINED POLICIES
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
% It also evaluates the success percentage with respect to pre-specified
% settling time tasks.
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


% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
nlq = sys.nlq;                          % System order for LQ servo
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;
indsmodelplot = master_settings.indsmodelplot;

% Number of \nu values to plot for
nummodelsplot = size(indsmodelplot,1);

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% Nominal, perturbed models
model = model_cell{indnom};                  % System model

% Trim data
xe = model_cell{indnom}.trimconds.xe;

% Integration time interval T
T = alg_settings.T;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% Extract algorithm names
algnames = master_settings.algnames;

% System tag
systag = master_settings.systag;
systag_py = master_settings.systag_py;

% Extract system names
sysnames = master_settings.sysnames;

% Algorithm list
alg_list = group_settings.alg_list;

% Number of algorithms tested
numalgs = size(alg_list,1);

% Use pre-filter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;
% pf1nopf0 = 1;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end

% Nominal model
model_nom_ind = alg_settings.model_nom_ind;
model_nom = model_cell{model_nom_ind};
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Final policies
if group_settings.hasmidirl
    Kf_mat = alg_settings.Kf_mat;
    Pf_mat = alg_settings.Pf_mat;
end

% Number of seeds, simulations evaluated
n_seed = alg_settings.n_seed;
nsim = alg_settings.nsim;

% Start, end seeds to evaluate
seed_start = alg_settings.seed_start;
seed_end = alg_settings.seed_end;

% Extract system plot settings
sys_plot_settings = master_settings.sys_plot_settings;

% Policy linear (=1) or nonlinear (=0)
linpol1nonlinpol0 = alg_settings.linpol1nonlinpol0;

% IC distributions
x_init = sys_plot_settings.x_init;

% x thresholds to check
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


% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% Time span for simulation
tspan = [0, T];


% ***********************
%       
% CONTROL SETTINGS
%   

% System dimensions
u_sett_eval.nlq = nlq;
u_sett_eval.m = m;

% Model linear (=1) or nonlinear (=0)
u_sett_eval.lin1nonlin0 = lin1nonlin0;

% Use linear policy (=1) or nonlinear policy (=0)
u_sett_eval.linpol1nonlinpol0 = linpol1nonlinpol0;

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

% Q, R
u_sett_eval.Q = alg_settings.Q;
u_sett_eval.R = alg_settings.R;
u_sett_eval.Sxirl_x3 = Sxirl_x3;





% ***********************
%       
% GET STATE-SPACE MATRICES FOR EACH LOOP
%   


% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Stores integrated return at each seed
ERmat = zeros(numalgs, nummodelsplot, n_seed, nsim);

% Stores success instances
successmat_x = zeros(numalgs, nummodelsplot, n_seed, nsim, numthresh_x);


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

% ***********************
%
% LOOP OVER MODELS TESTED
%  

for algcnt = 1:numalgs

    % Current algorithm
    algi = alg_list{algcnt};

switch algi

% *************************************************************************
%
% RUN SIMULATION
% 
% *************************************************************************

case algnames.mi_dirl 

% Tag
u_sett_eval.tag = 'lq_servo';

for mcnt = 1:nummodelsplot

    % Current model index
    imcnt = indsmodelplot(mcnt);

    % Simulation model
    model_sim = model_cell{imcnt};
    xe_sim = model_sim.trimconds.xe;
    ue_sim = model_sim.trimconds.ue;
    
    % Store simulation model in global settings
    u_sett_eval.model_sim = model_sim;
    u_sett_eval.xe_sim = xe_sim;
    u_sett_eval.ue_sim = ue_sim;

    % Equilibrium state, control for control calculations
    u_sett_eval.xe_ctrl = xe_sim;
    u_sett_eval.ue_ctrl = ue_sim;    

    % DEBUGGING: Display current algorithm, model
    disp('% *********************************************************')
    disp('%')
    disp(['% EVALUATING RETURN AND SUCCESS PCT FOR ALGORITHM -- ' algi])
    disp(['% AND MODEL    ' num2str(mcnt) '   OUT OF  ' ...
        num2str(nummodelsplot)])
    disp('%')
    disp('% *********************************************************')
    disp(' ')   

    for seedcnt = 1:n_seed
     

    % ***********************
    %
    % EVALUATE
    %  

    % Set RNG seed -- evaluation
    %rng(seedcnt-1+n_seed);    
    rng(seedcnt-1 + 100);

    % Get final controller
    if linpol1nonlinpol0
        u_sett_eval.K = Kf_mat(:,:,mcnt,seedcnt);
    else
        u_sett_eval.P = Pf_mat(:,:,mcnt,seedcnt);
        u_sett_eval.gx = model_nom.gx;        
    end  

    % Initialize empty vector to store mean final state norm over this
    % iteration's policy
    normxtfvec = zeros(nsim,1);

    % Initialize empty vector to store mean final state norm over this
    % iteration's policy
    Jxtfvec = zeros(nsim,1);

    % Initialize IC matrix: entry (i,:) contains the i-th total IC vector
    x0mat_eval = zeros(nsim,n);
    for ix0 = 1:nsim 
        % Initialize 
        x0mat_eval(ix0,:) = inv(sx) * 2 * x_init .* (rand(n,1) - 0.5) + xe;
    end    

    % Start timer
    t0 = tic;

    for j = 1:nsim 

        % Initialize empty IC vector
        x0_sim = zeros(n_sim,1);

        % Get current IC
        x0 = x0mat_eval(j,:)';    
    
        % Set current IC
        x0_sim(indsx) = x0;
        if pf1nopf0
            x0_sim(indspf) = sy * (x0(indsxr) - xe_nom(indsxr));
        end   

        % Run simulation
        [t, x] = ode45(@odefunct_eval_cost, tspan, x0_sim);
            
        % Final state x(t_f)
        xend = x(end,:)';

        % Store cost
        Jxtfvec(j) = xend(indsJ);

        % Store norm of final state
        if hasintaug
            xtf = [ xend(indsz)
                    xend(indsx) - xe_sim ];
        else
            xtf = xend(indsx) - xe_sim;
        end
        normxtfvec(j) = norm(xtf);

        % Check thresholds
        for ti = 1:numthresh_x
            currthreshind = threshmat_x(ti, 1);
            currthresh = threshmat_x(ti, 2);
            currt = threshmat_x(ti, 3);
            % Get min index >= threshold time
            gtt = find(t >= currt);
            if isempty(gtt)
                currtind = size(t,1);
            else
                currtind = gtt(1);
            end
            % Get state at threshold time
            xtind = x(currtind,:)';
            % Plant state -- post-transformation, shifted to trim
            txp = sx * (xtind(indsx) - xe_nom);
            successmat_x(algcnt, mcnt, seedcnt, j, ti) = ...
                txp(currthreshind) <= currthresh && ...
                txp(currthreshind) >= -currthresh;
        end

    end

    % Stop timer
    t1 = toc(t0);

    % Calculate mean, std cost J(x)
    Jxtfmean = mean(Jxtfvec);
    Jxtfstd = std(Jxtfvec);

    % Store mean, std cost J(x)
    ERmat(algcnt, mcnt, seedcnt, :) = - Jxtfvec;

    % DEBUGGING: Display current IC in x_1
    disp(['***** RETURN AND SUCCESS PCT FOR ALG --      ' algi ...
        '   --      SEED:   ' num2str(seedcnt) '      OUT OF      ' ...
        num2str(n_seed) ])   

    % DEBUGGING: Display mean cost
    disp(['     *** MEAN RETURN +/- 2 \sigma' ...
        '   =   ' num2str(-Jxtfmean) ' +/- ' num2str(2*Jxtfstd)])

    % DEBUGGING: Display mean final state norm
    disp(['     *** MEAN FINAL STATE NORM ||(z, x_p)(t_f)|| WRT TRIM' ...
        '   =   ' num2str(mean(normxtfvec))])

    % DEBUGGING: Display mean final state norm
    disp(['     *** BATCH ELAPSED TIME' ...
        '   =   ' num2str(t1) ' s'])    

    end

end

% *************************************************************************
%
% RUN SIMULATION -- FVI
% 
% *************************************************************************

case {algnames.cfvi; algnames.rfvi}

% Get if to evaluate via table lookup (=1) or Python call (=0)
tbl1_py0 = alg_settings.tbl1_py0;
   
if tbl1_py0
    % Simulation model
    model_sim = model_cell{indnom};
    xe_sim = model_sim.trimconds.xe;
    ue_sim = model_sim.trimconds.ue;
    
    % Store simulation model in global settings
    u_sett_eval.model_sim = model_sim;
    u_sett_eval.xe_sim = xe_sim;
    u_sett_eval.ue_sim = ue_sim;

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
    % Iteration to evaluate
    iter = alg_settings.iter;
    % Simulation frequency and return frequency
    fs = alg_settings.fs;
    fs_return = alg_settings.fs_return;
end

for mcnt = 1:nummodelsplot

    % DEBUGGING: Display current algorithm, model
    disp('% *********************************************************')
    disp('%')
    disp(['% EVALUATING RETURN AND SUCCESS PCT FOR ALGORITHM -- ' algi])
    disp(['% AND MODEL    ' num2str(mcnt) '   OUT OF  ' ...
        num2str(nummodelsplot)])
    disp('%')
    disp('% *********************************************************')
    disp(' ')  

    % Current model index
    imcnt = indsmodelplot(mcnt);

    % Simulation model
    model_sim = model_cell{imcnt};

    if tbl1_py0
        
    else

    % Perturbation parameters
    nuvec = model_sim.nuvec;
    dtheta = nuvec - 1;
    switch systag
        case sysnames.pendulum
            % \theta = [m, L]
            dtheta = [0; dtheta * model.L];
    end

    % Begin construction of the command to run the script
    cmdstr = ['python "' relpath_py_code ...
        'value_iteration/eval_trained_policy.py"'];

    % Options: Algoroithm name
    cmdstr = [cmdstr ' -alg_name="' algi '"'];    

    % Options: Evaluation mode is rand
    cmdstr = [cmdstr ' -system_name="' systag_py '"'];    

    % Options: Starting evaluation seed
    cmdstr = [cmdstr ' -seed_start="' num2str(seed_start) '"'];   

    % Options: Ending evaluation seed
    cmdstr = [cmdstr ' -seed_end="' num2str(seed_end) '"'];

    % Options: Iteration to evaluate
    cmdstr = [cmdstr ' -iter_eval="' num2str(iter) '"'];

    % Options: Where to save the output data to
    nuvec_filename = num2filename(nuvec);
    filename_out = [algi '_' systag_py ...
        '_nu_' nuvec_filename '_return_data.mat'];
    file_out = [abspath relpath_data_eval filename_out];
    cmdstr = [cmdstr ' -file_out="' file_out '"'];

    % Options: Simulation frequency and return frequency
    cmdstr = [cmdstr ' -T="' num2str(T) '"'];
    cmdstr = [cmdstr ' -fs="' num2str(fs) '"'];
    cmdstr = [cmdstr ' -fs_return="' num2str(fs_return) '"']; 
    
    % Options: Number of simulations to run
    cmdstr = [cmdstr ' -n_sim="' num2str(nsim) '"'];
    
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
    
    % Options: Do x thresholding
    cmdstr = [cmdstr ' -do_thresh_x'];

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
    
    t0 = tic;

    % EXECUTE PYTHON SCRIPT
    [status, evalout] = system(cmdstr);
    
    if ~isempty(evalout)
        disp(evalout)
    end

    % Stop timer
    t1 = toc(t0);

    % Extract data
    outdata = load(file_out);
    R = outdata.R;   
    succxmat = outdata.successmat_x;

    % Store
    ERmat(algcnt, mcnt, :, :) = R;
    successmat_x(algcnt, mcnt, :, :, :) = succxmat;

    avgR = mean(R(:));
    stdR = std(R(:));

    % DEBUGGING: Display current IC in x_1
    disp(['***** RETURN AND SUCCESS PCT FOR ALG --  ' algi ])   

    % DEBUGGING: Display mean cost
    disp(['     *** MEAN RETURN +/- 2 \sigma' ...
        '   =   ' num2str(avgR) ' +/- ' num2str(2*stdR)])

    % DEBUGGING: Display mean final state norm
    disp(['     *** BATCH ELAPSED TIME' ...
        '   =   ' num2str(t1) ' s'])      

    end

end

end

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

% Out data
out_data.ERmat = ERmat;
out_data.successmat_x = successmat_x;

