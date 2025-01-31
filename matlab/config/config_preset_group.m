function [preset_list, group_settings] = config_preset_group( ...
    preset_group, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET GROUP
%
% [ ***** ANONYMIZED ***** ] 
%
% 2022-02-16
%
% This program, given a desired preset group, initializes each of the
% presets to run in the group.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset_group      (String) Tag corresponding to the preset group to run.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% preset_list       ('numpresets' x 1 Cell) The i-th entry of this cell
%                   array is a string which is the tag of the desired
%                   preset to execute as the i-th preset in the group. 
% group_settings    (Struct) This is where any group-shared settings (e.g.,
%                   system, initial conditions, etc.) are stored for
%                   subsequent initialization of each of the presets. All
%                   of these settings are user-configurable, and each are
%                   well-commented below.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

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

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT FORMATTING OPTIONS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Check if pre-existing preset group settings have been initialized
if isfield(preset_group, 'group_settings')
    group_settings = preset_group.group_settings;
    preset_group = preset_group.tag;
end

% Tag of design being executed
designtag = master_settings.designtag;

% Store the preset group tag in the 'group_settings' struct
group_settings.preset_group = preset_group;

% Master plot settings
psett_master = group_settings.psett_master;

% Plot formatting -- line width (wide)
psett_linewidthwide = psett_master.psett_linewidthwide;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- dotted line
psett_dot = psett_master.psett_dot;

% Plot formatting -- dot-dash line
psett_dotdash = psett_master.psett_dotdash;

% Plot formatting -- colors
color_cmds = psett_master.color_cmds;
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;

% Extract system names
sysnames = master_settings.sysnames;

% Extract algorithm names
algnames = master_settings.algnames;

% Array of sweep type names
sweeptypenames = master_settings.sweeptypenames;

% Preset group name
groupname = group_settings.groupname;


switch preset_group
     
    % *********************************************************************
    %
    % MAIN PRESET GROUP
    % 
    %
    
    case 'main'

        % *****************************************************************
        %
        % PRESET GROUP SHARED SETTINGS      
        %        

        % Tag of system being executed
        systag = group_settings.systag;
        
        % Extract algorithm list
        alg_list = group_settings.alg_list;

        % Number of algorithms executed
        numalgs = size(alg_list,1);

        % Is a sweep preset group (=1) or not (=0)
        issweep = group_settings.issweep;
        issweep_IC = group_settings.issweep_IC;
        issweep_nu = group_settings.issweep_nu;

        % Get current sweep type if applicable
        if issweep
            sweeptype = group_settings.sweeptype;
        end

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0vec = zeros(numalgs,1);

        % Do prefilter (=1) or not (=0)
        pf1nopf0vec = group_settings.pf1nopf0 * ones(numalgs,1);


        % Is training (=1) or not (=0)
        istraining = group_settings.istraining;

        % Is DIRL (=1) or old IRL (=0)
        notirlold = group_settings.notirlold;

        % Do reference command r(t) injection (=1) or not (=0)
        if istraining
            dort = group_settings.dort;
        end

        % Legend, formatting cells, algorithm settings
        preset_list = cell(numalgs,1);
        lgd_p = alg_list;
        indiv_sett_cell = cell(numalgs,1);
        color_sett_cell = cell(numalgs,1);
        for i = 1:numalgs
            switch alg_list{i}
                case algnames.mi_dirl
                    color_sett_cell{i} = color_cmds.CSSdarkorange;
                    indiv_sett_cell{i} = ...
                        [psett_linewidthwide; color_sett_cell{i}];
                    lin1nonlin0vec(i) = 0;                        
                    preset_list{i} = 'dirl_nonlin';      
                case algnames.cfvi
                    color_sett_cell{i} = color_cmds.CSSlightseagreen;
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                      
                    preset_list{i} = 'cfvi_tracking';  
                case algnames.rfvi
                    color_sett_cell{i} = color_cmds.CSSlimegreen;
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                   
                    preset_list{i} = 'cfvi_tracking';                        
                case algnames.irl
                    color_sett_cell{i} = psett_matlabmaroon;
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                 
                    preset_list{i} = 'irl';                  
                case algnames.spi                   
                    color_sett_cell{i} = psett_matlabgreen;            
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                        
                    preset_list{i} = 'spi';  
                case algnames.radp                   
                    color_sett_cell{i} = psett_matlaborange;            
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                        
                    preset_list{i} = 'radp';  
                case algnames.ctvi                   
                    color_sett_cell{i} = psett_matlabyellow;            
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                        
                    preset_list{i} = 'ctvi'; 
            end

        end


        % ***********************
        %
        % GET SYSTEM DATA
        %   

        % System
        sys = master_settings.sys;
        model_cell = sys.model_cell;
        indnom = sys.indnom;
        model = get_elt_multidim(model_cell,indnom);

        % Dimensions of system cell array
        numnu1 = master_settings.numnu1;
        numnu2 = master_settings.numnu2;
        nummodels = master_settings.nummodels;

        % Trim
        x_e = model.trimconds.xe;

        % State dimension n
        n = size(x_e, 1);

        % Input dimension m
        m = model.m;

        % Sweep variable indices
        inds_x_sweep = group_settings.inds_x_sweep;

        switch systag
            case sysnames.hsv
                % Trim speed (ft/s)
                x1_e = x_e(model.indV);  
                % Trim FPA (rad)
                x2_e = x_e(model.indg); 
            case sysnames.pendulum
                % Trim pendulum angle (rad)
                x1_e = x_e(model.indth);  
                % Trim pendulum angular velocity (rad/s)
                x2_e = x_e(model.indthd);            
            otherwise
                x1_e = x_e(inds_x_sweep(1));  
                x2_e = x_e(inds_x_sweep(2));                    
        end

        % Degree/radian conversions
        D2R =   pi/180;
        R2D =   180/pi;


        % ***********************
        %
        % ICS
        %   

        switch systag
            case sysnames.hsv
                Vscl = model.lin.yscl(1);
                invVscl = 1 / Vscl;
        end

        % Vectors of initial values (x_{1}(0), x_{2}(0)) to test in the IC
        % sweep -- before trim applied
        switch systag
            case sysnames.hsv
                % x_{1}(0) -- in kft/s
                tx10vec_sweep = 0.1*(-1:1:1)';     % FINAL     
                % x_{2}(0) -- in deg 
                tx20vec_sweep = (-1:1:1)';      % FINAL                  
            case sysnames.pendulum
                % x_{1}(0) -- in deg
                tx10vec_sweep = 180 * (-1:1:1)';       % FINAL
                % x_{2}(0) -- in deg/s
                tx20vec_sweep = 45 * (-1:1:1)';       % FINAL                         
            case sysnames.vamvoudakis2010
                % x_{1}(0)
                tx10vec_sweep = 1*(-1:1:1)';    % TEST
                % x_{2}(0)
                tx20vec_sweep = 1*(-1:1:1)';       % TEST  
            otherwise
                tx10vec_sweep = 0;
                tx20vec_sweep = 0;
        end

        % If current preset group is a sweep, then take sweep ICs. Else,
        % use single IC
        if issweep_IC
           
            tx10vec = tx10vec_sweep;
            tx20vec = tx20vec_sweep;

        else

            switch systag
                case sysnames.hsv
                    tx10vec = 0;
                    tx20vec = 0;                               
                case sysnames.pendulum                 
                    tx10vec = 180;
                    tx20vec = 0;      
                case sysnames.vamvoudakis2010                  
                    tx10vec = 1;
                    tx20vec = 1; 
                    if contains(groupname, 'ES_nom_all_training')
                        tx10vec = 0;
                        tx20vec = 0;
                    end
            end

        end

        
        % Number of ICs tested in each variable
        numx10 = size(tx10vec,1);
        numx20 = size(tx20vec,1);
        numICs = numx10 * numx20;

        % Apply deg -> rad, deg/s -> rad/s conversion for ICs x_{0}
        switch systag
            case sysnames.hsv
                tx10vec = tx10vec * 1000;
                tx10vec_sweep = tx10vec_sweep * 1000;                 
                tx20vec = tx20vec * D2R;
                tx20vec_sweep = tx20vec_sweep * D2R;                        
            case sysnames.pendulum
                tx10vec = tx10vec * D2R;
                tx10vec_sweep = tx10vec_sweep * D2R;
                tx20vec = tx20vec * D2R;
                tx20vec_sweep = tx20vec_sweep * D2R;                
        end

        % Apply shift by trim
        x10vec = tx10vec + x1_e;
        x10vec_sweep = tx10vec_sweep + x1_e;
        x20vec = tx20vec + x2_e;
        x20vec_sweep = tx20vec_sweep + x2_e;

        % Initialize IC matrix: entry (i,j,:) contains the total IC vector
        % with x10vec(i), x20vec(j) in their appropriate place
        x0mat = zeros(numx10,numx20,n);
        for i = 1:numx10
            for j = 1:numx20
                x0mat(i,j,:) = x_e;
                x0mat(i,j,inds_x_sweep(1)) = x10vec(i);
                x0mat(i,j,inds_x_sweep(2)) = x20vec(j);
            end
        end

        % Find indices of trim states in sweep
        indsxe = zeros(2,1);
        for i = 1:2
            ind_xri = inds_x_sweep(i);
            xei = x_e(ind_xri);
            switch i
                case 1
                    xi0vec = x10vec_sweep;
                case 2
                    xi0vec = x20vec_sweep;
            end
            indsxe(i) = find(xi0vec == xei);
        end


        % ***********************
        %
        % VALUE FUNCTION, POLICY EVALUATION (POST-TRAINING) SETTINGS
        %   

        % Initialize evaluation data (=1) or load previous (=0)
        init1load0_eval_data = 0;     

        % Do plots for evaluations (=1) or not (=0)
        doplots_eval = 1;

        % Do policy plots for evaluations (=1) or not (=0) 
        doplots_eval_pol = 0;

        % Use uniform color limits on countour plots (=1) or not (=0)
        doClim = 1;

        % Evaluate at nominal model only (=1) or perturbed models too (=0)    
        nomonly = 0;

        % Plot settings: Do titles, labels, etc.
        psett_eval.do_ttl = 1;
        psett_eval.do_xlbl = 1;
        psett_eval.do_ylbl = 1;
        psett_eval.do_cbar = 1;

        % ***********************
        %
        % FVI -- EVALUATION PROCEDURE
        %
        % Evaluate FVI via table lookup (=1) or Python call (=0)
        %
        tbl1_py0 = 0; 
        
        % Evaluation grid
        switch systag
            case sysnames.hsv
                % x_{1}(0) -- in kft/s
                tx10vecp = 0.1 * linspace(-1,1,150)';     % FINAL
                % x_{2}(0) -- in deg 
                tx20vecp = linspace(-1,1,150)';         % FINAL
            case sysnames.pendulum
                % x_{1}(0) -- in deg
                tx10vecp = 180 * linspace(-1,1,150)';   % FINAL
                % x_{2}(0) -- in deg/s
                tx20vecp = 45 * linspace(-1,1,150)';   % FINAL                              
            case sysnames.vamvoudakis2010
                % x_{1}(0) 
                tx10vecp = 1 * linspace(-1,1,150)';    % FINAL
                % x_{2}(0)
                tx20vecp = 1 * linspace(-1,1,150)';    % FINAL
            otherwise
                tx10vecp = [];
                tx20vecp = [];
        end

        % Number of ICs tested in each variable
        numx10p = size(tx10vecp,1);
        numx20p = size(tx20vecp,1);
        numICsp = numx10p * numx20p;

        % Apply deg -> rad, deg/s -> rad/s conversion for ICs x_{0}
        switch systag
            case sysnames.hsv
                tx10vecp = tx10vecp * 1000;
                tx20vecp = tx20vecp * D2R; 
            case sysnames.pendulum
                tx10vecp = tx10vecp * D2R;
                tx20vecp = tx20vecp * D2R;            
        end

        % Apply shift by trim
        x10vecp = tx10vecp + x1_e;
        x20vecp = tx20vecp + x2_e;

        % ***********************
        %
        % COST J(x) EVALUATION SETTINGS
        %   

        % Do cost J(x) evaluations (=1) or not (=0)
        Jx_sett.doJx = 1;
        
        % Integration horizon length (sec)
        switch systag
            case sysnames.hsv   
                Jx_sett.T = 50;  
            case sysnames.pendulum
                Jx_sett.T = 5; 
            case sysnames.vamvoudakis2010
                Jx_sett.T = 5; 
            otherwise
                Jx_sett.T = 10; 
        end
  
        % ***********************
        %
        % FVI -- SEED, ITERATION TO EVALUATE 
        %
        % NOTE: These settings are applicable only if evaluating FVI from a
        % Python call; i.e., if tbl1_py0 = 1
        %

        % If evaluating via Python: Simulation frequency and return
        % frequency
        Jx_sett.seed_eval = 0;
        if ~tbl1_py0            
            Jx_sett.fs = 50;
            Jx_sett.fs_return = 10;  
            switch systag
                case sysnames.hsv  
                    Jx_sett.iter_eval = 50;
                    Jx_sett.fs = 25;
                    Jx_sett.fs_return = 25;
                case sysnames.pendulum
                    Jx_sett.fs_return = 50;                     
                    Jx_sett.iter_eval = 25;                 
                case sysnames.vamvoudakis2010   
                    Jx_sett.iter_eval = 25; 
                    Jx_sett.fs = 100;
                    Jx_sett.fs_return = 50;  
            end
        end


        % Threshold \epsilon > 0 such that simulation terminates when ||x||
        % < \epsilon
        Jx_sett.donormeps = 0;
        Jx_sett.normeps = 5e-2;       

        % ***********************
        %
        % EXPECTED RETURN EVALUATION SETTINGS
        %   

        ER_sett.doER = 1;

        % Evaluate DIRL from sweep (=1) or from ER evaluation (=0)
        sweep1_ER0 = 1;

        % Integration horizon length (sec), and other settings
        ER_sett.T = Jx_sett.T;

        % Number of seeds
        ER_sett.n_seed = 20;

        % Number of simulations for evaluating ER
        ER_sett.nsim = 100;      

        % Threshold \epsilon > 0 such that simulation terminates when ||x||
        % < \epsilon (NOTE: Not functional. Keep low)
        ER_sett.donormeps = 0;
        ER_sett.normeps = Jx_sett.normeps;  

        % ***********************
        %
        % REFERENCE COMMAND SETTINGS
        %
        refcmd = group_settings.refcmd;
        refcmdtype = group_settings.refcmdtype;

        switch refcmd

            case 'training'

                % Simulation, plot time
                switch systag
                    case sysnames.vamvoudakis2010
                        tsim = 10;
                    otherwise
                        tsim = 100; 
                end

                tsim_plot = tsim; 

            case 'step_V'

                switch systag
                    case sysnames.hsv

                        % Simulation, plot time         
                        tsim = 50;          
                        tsim_plot = tsim; 

                        % 100 ft/s step-velocity command
                        x1r = 100;
                        x2r = 0;

                        % Output index step is being applied to
                        indstep = 1;                    

                end

            case 'step_g'

                switch systag

                    case sysnames.hsv

                        % Simulation, plot time   
                        tsim = 50;
                        tsim_plot = tsim;   

                        % 1 deg FPA command
                        x1r = 0;
                        x2r = 1*D2R;  

                        % Output index step is being applied to
                        indstep = 2;

                end        

            case 'step_th'

                switch systag
                    case sysnames.pendulum

                        % Simulation, plot time
                        tsim = 10;  
                        tsim_plot = 2.5;
            
                        % 0 deg \theta command
                        x1r = 0;  

                        % Output index step is being applied to
                        indstep = 1;                                                                        

                end
             

            case 'step_x1'

                % Simulation, plot time
                tsim = 5;          
                tsim_plot = 5;  

                % 0 x_1 command
                x1r = 0; 

                % Output index step is being applied to
                indstep = 1;              

        end

        % ***********************
        %
        % REFERENCE COMMANDS
        %

        switch refcmdtype

            case 'step'
                
                switch m
                    case 1
                        % Set params
                        x1_m = x1_e + x1r;
        
                        biasvec = x1_m;
                        nderivvec = 0;
                        cos1_sin0 = 0;
                        Amat = 0;
                        Tmat = 1;  
                    otherwise
                        % Set params
                        x1_m = x1_e + x1r;
                        x2_m = x2_e + x2r;
        
                        biasvec = [x1_m; x2_m];
                        nderivvec = [0; 0];
                        cos1_sin0 = zeros(2,1);
                        Amat = zeros(2,1);
                        Tmat = ones(2,1);  
 
                        % Output index step is being applied to
                        group_settings.indstep = indstep;                           
                
                end
      

        end

        % ***********************
        %
        % APPLY DECAYING EXPONENTIAL TO REF CMD ACCORDING TO IC
        %
        % NOTE: Not functional (keep low)
        %   

        doexpx0 = 0;       

        % ***********************
        %
        % CRITIC BASIS   
        %  
        
        % Default basis for critic NN; i.e., \hat{V}(x) =
        % \sum_{i=1}^{N} w_i * \phi_i(x).
        
        if contains(groupname, 'big_basis')
            basis_critic.tag = 'order_2_degree_4';
        else
            basis_critic.tag = 'order_2_degree_2';
        end

        % ICs
        switch basis_critic.tag
            case 'order_2_degree_2'
                basis_critic.c0 = [1; 0; 4];
            case 'order_2_degree_4'
                basis_critic.c0 = [1; 0; 4; 0; 0; 0; 0; 0];
        end

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % Penalties
        Q = lq_data.Qirl;
        R = lq_data.R; 

        basis_critic.Q = Q;
        basis_critic.R = R;

        % ***********************
        %
        % ACTOR BASIS -- g(x) UNKNOWN
        %  
        % Basis for actor NN without g(x); i.e., \hat{\mu}(x) =
        % \sum_{i=1}^{N} w_i * \phi_i(x).
        %
        
        basis_actor_no_g.tag = 'vamvoudakis_2010_actor_no_g';          

        % ***********************
        %
        % HAMILTONIAN NETWORK BASIS (VI ONLY)
        %        
        
        % Hamiltonian NN basis \Theta(x)
        Theta.tag = 'vamvoudakis_2010_hamiltonian_min';
        
        % Store parameters
        basis_hamiltonian.Theta = Theta;


        % ***********************
        %
        % PLOT SETTINGS
        %

        % Print performance metrics (=1) or don't print (=0)
        print_metrics = 0;

        % Threshold percentages to calculate rise time, settling time (%)
        trpctvec = [90];
        tspctvec = [10; 1];

        numtr = size(trpctvec,1);
        numts = size(tspctvec,1);

        % Threshold values to determine settling time
        % Column 1: thresholds for y_1
        % Column 2: thresholds for y_2
        switch systag
            case {sysnames.hsv}
                threshmat = [ 1     0.01
                              10    0.1   ];
            case sysnames.pendulum
                threshmat = [   0.1
                                0.01    ]; 
            otherwise
                switch m
                    case 1
                        threshmat = [   0.1
                                        0.01    ];  
                    case 2
                        threshmat = [   0.1     1
                                        0.01    0.1   ];
                end
        end


        % Get variable-name friendly threshold values for storage
        numthresh = size(threshmat,1);
        threshmat_txt = cell(numthresh,2);
        for i = 1:sys.m
            for j = 1:numthresh
                threshmat_txt{j,i} = num2filename(threshmat(j,i));
            end
        end      

        % ***********************
        %
        % STORE SETTINGS
        %

        % Relative path to write data to (on top of relpath)
        group_settings.relpath_data = 'data\';        

        % Number of presets
        group_settings.numpresets = numalgs;

        % Print performance metrics (=1) or don't print (=0)
        group_settings.print_metrics = print_metrics;

        % Thresholds to determine settling time
        group_settings.trpctvec = trpctvec;
        group_settings.tspctvec = tspctvec;
        group_settings.threshmat = threshmat;      
        group_settings.threshmat_txt = threshmat_txt;

        % Legend entries
        group_settings.lgd_p = lgd_p;

        % Individual plot settings
        group_settings.color_sett_cell = color_sett_cell;        
        group_settings.indiv_sett_cell = indiv_sett_cell;

        group_settings.tsim = tsim;
        group_settings.tsim_plot = tsim_plot;

        % ICs
        ICs.numx10 = numx10;
        ICs.numx20 = numx20;
        ICs.numICs = numICs;
        ICs.x10vec = x10vec;
        ICs.x20vec = x20vec;
        ICs.tx10vec = tx10vec_sweep;
        ICs.tx20vec = tx20vec_sweep;
        ICs.x0mat = x0mat;
        ICs.indsxe = indsxe;
        group_settings.ICs = ICs;

        % Initialize evaluation data (=1) or load previous (=0)
        group_settings.init1load0_eval_data = init1load0_eval_data;

        % Evaluate at nominal model only (=1) or perturbed models too (=0)
        group_settings.nomonly = nomonly;

        % Do plots for evaluations (=1) or not (=0)
        group_settings.doplots_eval = doplots_eval;

        % Plot settings: Do titles, labels, etc.
        group_settings.psett_eval = psett_eval;

        % Do policy plots for evaluations (=1) or not (=0) 
        group_settings.doplots_eval_pol = doplots_eval_pol;

        % Evaluate FVI via table lookup (=1) or Python call (=0)  
        group_settings.tbl1_py0 = tbl1_py0;

        % Evaluate DIRL from sweep (=1) or from ER evaluation (=0)
        group_settings.sweep1_ER0 = sweep1_ER0;

        % Evaluation points
        xplot.numx10p = numx10p;
        xplot.numx20p = numx20p;
        xplot.numICsp = numICsp;
        xplot.tx10vecp = tx10vecp;
        xplot.tx20vecp = tx20vecp; 
        xplot.x10vecp = x10vecp;
        xplot.x20vecp = x20vecp;
        group_settings.xplot = xplot;

        % Use uniform color limits on countour plots (=1) or not (=0)
        group_settings.doClim = doClim;

        % Cost J(x) evaluation settings
        group_settings.Jx_sett = Jx_sett;

        % Expected return evaluation settings
        group_settings.ER_sett = ER_sett;

        % Model linear (=1) or nonlinear (=0)
        group_settings.lin1nonlin0vec = lin1nonlin0vec;

        % Do prefilter (=1) or not (=0)
        group_settings.pf1nopf0vec = pf1nopf0vec;

        % Do decaying exponential r(t) according to x_0 (=1) or not (=0)
        group_settings.doexpx0 = doexpx0;
        if doexpx0
           group_settings.expx0avec = expx0avec;
        end

        % Commands
        switch refcmdtype
            case 'step'
                r_sett.tag = 'sum_sin';
                switch m
                    case 1
                        r_sett.Arvec = x1r;
                    case 2
                        r_sett.Arvec = [x1r; x2r];
                end              
        end
        switch refcmdtype
            case 'training'
                r_sett = [];
            case 'step'   
                r_sett.biasvec = biasvec;
                r_sett.nderivvec = nderivvec;
                r_sett.dorefvec = ones(2,1);
                r_sett.cos1_sin0 = cos1_sin0;
                r_sett.Amat = Amat;
                r_sett.Tmat = Tmat;                
        end
        group_settings.r_sett = r_sett;

      
        % ***********************
        %
        % SWEEP SETTINGS
        %          

        sweepsetts.issweep = issweep;
        if issweep
            % Vector, each entry of which determines the number of
            % sweep parameters in the respective variable (e.g., number
            % of ICs)
            switch sweeptype
                case sweeptypenames.IC
                sweepsizevec = [numx10; numx20; nummodels];              
                case sweeptypenames.nu           
                sweepsizevec = [numnu1; numnu2]; 
                case sweeptypenames.rand_nu           
                sweepsizevec = [numnu1]; 
                otherwise
                sweepsizevec = 1;    
            end
            sweepsetts.sweepsizevec = sweepsizevec;
            % Dimension of the sweep
            sweepsetts.sweepdim = size(sweepsizevec,1);
        end

        % Store sweep settings
        group_settings.sweepsetts = sweepsetts;        

        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('*** ERROR: PRESET GROUP TAG NOT RECOGNIZED ***');  
       
end    



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE BASES (IF APPLICABLE)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% List of networks by variable name
NN_list = {    
                'basis_critic'
                'basis_actor_no_g'
                'basis_hamiltonian' 
                                            };

% Number of network types
num_NN = size(NN_list, 1);
                                    
% Basis initialization settings
b_sett.sys = sys;
b_sett.alg = 'debug';
if exist('Q', 'var') && exist('R', 'var')
    b_sett.Q = Q;
    b_sett.R = R;
end
                                    
% ***********************
%
% INITIALIZE BASES
%

for i = 1:num_NN
   
    % Extract current network type
    NN_i = NN_list{i};
    
    % Check if the current network type exists as a variable
    if exist(NN_i, 'var')
        
        % Extract the basis
        switch NN_i
            case 'basis_critic'
                basis_i = basis_critic;
            case 'basis_actor_no_g'
                basis_i = basis_actor_no_g;
            case 'basis_actor_g_known'
                basis_i = basis_critic;
            case 'basis_hamiltonian'
                basis_i = basis_hamiltonian.Theta;
        end
        
        % Initialize the basis parameters
        basis_i = config_basis(basis_i, b_sett);
        
        % Store the initialized basis
        switch NN_i
            case 'basis_critic'
                basis_critic = basis_i;
                group_settings.basis_critic = basis_critic;
            case 'basis_actor_no_g'
                basis_actor_no_g = basis_i;
                group_settings.basis_actor_no_g = basis_actor_no_g;
            case 'basis_actor_g_known'
                basis_critic = basis_i;
                group_settings.basis_actor_g_known = basis_critic;
            case 'basis_hamiltonian'
                basis_hamiltonian.Theta = basis_i;
                group_settings.basis_hamiltonian.Theta = basis_hamiltonian.Theta;
        end
        
    end
    
end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE GROUP SETTINGS WHICH ARE ALWAYS DECLARED
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Preset group
group_settings.preset_group = preset_group;

