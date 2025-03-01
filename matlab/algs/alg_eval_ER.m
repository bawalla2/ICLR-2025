function [meanJxmat, out_data] = alg_eval_ER(alg_settings, group_settings, master_settings)
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

% Algorithm settings
alg_hyperparams = alg_settings.alg_hyperparams;

% Uniform distribution for weight initialization
Uc0vec = alg_hyperparams.Uc0vec;

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

% Threshold \epsilon > 0 such that simulation terminates when ||x|| <
% \epsilon
donormeps = alg_settings.donormeps;
normeps0 = alg_settings.normeps;

% DIRL loop_cell data
loop_cell = alg_settings.loop_cell;
% Number of loops executed
numloops = size(loop_cell,1);

% Max number of iterations
istarmax = alg_settings.istarmax;

% Algorithm tag
algtag = alg_settings.algtag;

% Extract algorithm names
algnames = master_settings.algnames;

% System tag
systag = master_settings.systag;

% Extract system names
sysnames = master_settings.sysnames;

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

% Number of seeds, simulations evaluated
n_seed = alg_settings.n_seed;
nsim = alg_settings.nsim;

% LQ data
lq_data_cell = master_settings.lq_data_cell; 

% Indexing from DIRL
inds_dirl = alg_settings.inds_dirl;

% Extract system plot settings
sys_plot_settings = master_settings.sys_plot_settings;

% Policy linear (=1) or nonlinear (=0)
linpol1nonlinpol0 = alg_settings.linpol1nonlinpol0;

% IC distributions
x_init_train = sys_plot_settings.x_init_train;
x_init = sys_plot_settings.x_init;

% When generating initial policies, perturb the nominal P matrix (=1) or
% initialize from a mean-0 distribution (=0)
perturb_nominal_P = alg_settings.perturb_nominal_P;

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
% Transform [z, y, x_r] -> [x_1, x_2]
Sxirl = model.lin.io.Sxirl;
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

% Extract loop settings
setts.dox3 = alg_hyperparams.dox3;
setts.numloops = numloops;
setts.loop_cell = loop_cell;
setts.inds = inds_dirl;

% Nominal system
[A_cell_nom, B_cell_nom] = make_AB_cells(model_nom.lin, setts);

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Stores integrated cost J(x_0) at each seed
meanJxmat = zeros(n_seed, istarmax+1);

% Stores runtime at each seed
runtimevec = zeros(n_seed);

% Storage for final policy parameters at each seed
if hasintaug
    Kf_mat = zeros(m, nlq+m, nummodelsplot, n_seed);
    Pf_mat = zeros(nlq+m, nlq+m, nummodelsplot, n_seed);
else
    Kf_mat = zeros(m, nlq, nummodelsplot, n_seed);
    Pf_mat = zeros(nlq, nlq, nummodelsplot, n_seed);
end

% Storage for output data
outdata_cell = cell(nummodelsplot, n_seed);

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


for seedcnt = 1:n_seed

    % Set RNG seed -- training
    rng(seedcnt-1);

    % Set ICs for training on this seed
    alg_hyperparams.x0 = inv(sx) * 2 * x_init_train .* (rand(n,1) - 0.5) + xe;

    K0_cell = cell(numloops,1);
    P0_cell = cell(numloops,1);


    for k = 1:numloops
        Rk = loop_cell{k}.R;
        Akk = A_cell_nom{k,k};
        Bkk = B_cell_nom{k,k};
        nxzk = size(Akk,1);
        Uc0k = Uc0vec(k);
        % Generate stabilizing random weights
        doloop = 1;
        while doloop
            % Generate random weights
            c0k = 2 * Uc0k * (rand(nxzk*(nxzk+1)/2,1) - 0.5);
            P0k = invvecsym(c0k);
            if perturb_nominal_P
                Pk_nom = eval(['lq_data_nom.lq_data_' ...
                    num2str(k) num2str(k) '.P']);
                P0k = Pk_nom + P0k;
            end
            K0k = inv(Rk) * Bkk' * P0k;
            % Check for stability
            eigs = eig(Akk - Bkk * K0k);
            reigs = real(eigs);
            if sum(reigs > 0) == 0
                doloop = 0;
            end
        end
        K0_cell{k} = K0k;
        P0_cell{k} = P0k;
    end
    
    alg_hyperparams.K0_cell = K0_cell;

    for mcnt = 1:nummodelsplot
    
    % Current model index
    imcnt = indsmodelplot(mcnt);
    alg_hyperparams.model_sim_ind = imcnt;

    % Run algorithm
    outdata = eval(['alg_' alg_hyperparams.alg ...
                '(alg_hyperparams, group_settings, master_settings)']);

    % Store output data
    outdata_cell{mcnt, seedcnt} = outdata;
    
    % Get controller cell array
    K_cell = outdata.K_cell;
    P_cell = outdata.P_cell;

    % Get run time 
    runtimevec(seedcnt) = outdata.runtime;

    % ***********************
    %
    % EVALUATE
    %  

    % Set RNG seed -- evaluation
    %rng(seedcnt-1+n_seed);    
    rng(seedcnt-1 + 100);

    % Evaluate mean cost for each iteration i
    for i = 1:istarmax+1

    % Get indices of current-iteration controller in each loop
    i1 = min([i, loop_cell{1}.istar]);
    if numloops == 2
        i2 = min([i, loop_cell{2}.istar]);
    end

    % Form controller -- IRL state partition
    switch numloops
        case 1
            Kirl = K_cell{1}(:,:,i1);
        case 2
            % Final controllers
            K11 = K_cell{1}(:,:,i1);
            K22 = K_cell{2}(:,:,i2);
            Kirl = blkdiag(K11, K22);
    end

    % Form composite controller: [z, y, x_r]
    K = Kirl * Sxirl;

    % Init composite P -- DIRL coords
    if ~linpol1nonlinpol0
        Pirl = [];
        for j = 1:numloops
            
            % Get final Riccati equation solution
            Pij = eval(['P_cell{j}(:,:,i' num2str(j)  ')']);   
    
            % Make block-diagonal composite P
            Pirl = blkdiag(Pirl,Pij);
                           
        end
        P = Sxirl' * Pirl * Sxirl;
    end

    % Set controller as this iteration's controller
    if linpol1nonlinpol0
        u_sett_eval.K = K;
    else
        u_sett_eval.P = P;
        u_sett_eval.gx = model_nom.gx;        
    end

    % If this is the final iteration, store K, P
    if i == istarmax+1
        Kf_mat(:,:,mcnt,seedcnt) = K;
        Pf_mat(:,:,mcnt,seedcnt) = P;
    end

    % Evaluate if is nominal model only
    if imcnt == indnom

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
    
            % Simulation model
            model_sim = model_cell{indnom};
            xe_sim = model_sim.trimconds.xe;
            ue_sim = model_sim.trimconds.ue;
            
            % Store simulation model in global settings
            u_sett_eval.model_sim = model_sim;
            u_sett_eval.xe_sim = xe_sim;
            u_sett_eval.ue_sim = ue_sim;
    
            % Equilibrium state, control for control calculations
            u_sett_eval.xe_ctrl = xe_sim;
            u_sett_eval.ue_ctrl = ue_sim;
    
            % Initialize empty IC vector
            x0_sim = zeros(n_sim,1);
    
            % Get current IC
            x0 = x0mat_eval(j,:)';
    
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

        end

    % Stop timer
    t1 = toc(t0);

    % Calculate mean cost J(x)
    Jxtfmean = mean(Jxtfvec);

    % Store mean cost J(x)
    meanJxmat(seedcnt,i) = Jxtfmean;

    % DEBUGGING: Display current IC in x_1
    disp(['***** EVALUATING COST J(x) FOR ALG --      ' algtag ...
        '   --      SEED:   ' num2str(seedcnt) '      OUT OF      ' ...
        num2str(n_seed) ...
        '   ITERATION:   ' num2str(i) '      OUT OF   ' num2str(istarmax)])   

    % DEBUGGING: Display mean cost
    disp(['     *** MEAN COST J(x)' ...
        '   =   ' num2str(Jxtfmean)])

    % DEBUGGING: Display mean final state norm
    disp(['     *** MEAN FINAL STATE NORM ||(z, x_p)(t_f)|| WRT TRIM' ...
        '   =   ' num2str(mean(normxtfvec))])

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
out_data.runtimevec = runtimevec;
out_data.Kf_mat = Kf_mat;
out_data.Pf_mat = Pf_mat;
out_data.outdata_cell = outdata_cell;

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
global u_sett_eval;
global normeps;

% Get indices of state partition
inds = u_sett_eval.inds;

% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett_eval.xe_sim;

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

