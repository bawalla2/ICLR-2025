function alg_settings = config_preset(preset, group_settings, ...
    master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SELECT ALGORITHM, SYSTEM, DESIGN PARAMETERS BASED ON PRESET
%
% [ ***** ANONYMIZED ***** ] 
%
% 2021-11-06
%
% This program, given a desired preset, handles all algorithm
% initialization/configuration. This is the main program for configuring
% specific algorithm hyperparameters (e.g., sample rates, excitations,
% etc.)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset            (String) Algorithm/system preset for desired example.
% group_settings    (Struct) Contains system/design parameters to
%                   be shared across all presets in the desired group.
%                   E.g., if for this preset group all designs share the
%                   same Q, R matrices, those fields may be included in
%                   this struct. This is initialized in
%                   config_preset_group.m.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% alg_settings  (Struct) Algorithm settings/parameters for subsequent
%               execution according to desired preset (see respective
%               algorithm .m-file for details). 
%               
%               Regardless of the preset, alg_settings will have the
%               following fields:
%
%   plot_settings       (Struct) Contains plot settings for this preset.
%                       Has the following fields:
%       legend_entry    (String) A label for this specific preset to
%                       distinguish it from the other designs executed in
%                       the preset group. E.g., if a group executes
%                       different algorithms, legend_entry for one preset
%                       might be 'IRL'.
%       plotfolder      (String) Name of the folder to save plots to for
%                       this preset. This could be the preset tag, or any
%                       other convenient identifier.
%
% NOTE: These are initialized automatically by settings in
% config_preset_group, but they nevertheless may be manually overriden.
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

% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************

% System
sys = master_settings.sys;
m = sys.m;

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


% Tag of system being executed
systag = master_settings.systag;
% List of system names
sysnames = master_settings.sysnames;

% Extract algorithm names
algnames = master_settings.algnames;

% Tag of design being executed
designtag = master_settings.designtag;

% Is a sweep (=1) or not (=0)
issweep = group_settings.issweep;
issweep_IC = group_settings.issweep_IC;
issweep_nu = group_settings.issweep_nu;
issweep_rand_nu = group_settings.issweep_rand_nu;

% ICs
ICs = group_settings.ICs;
tx10vec = ICs.tx10vec;
tx20vec = ICs.tx20vec;

% ***********************
%
% GET SWEEP ITERATION VECTOR, PARAMETERS
%    

% Sweep iteration vector
sweepindvec = group_settings.sweepindvec;

% IC indices 
if issweep_IC
    indICs = sweepindvec(1:2);
    tx10 = tx10vec(indICs(1));
    tx20 = tx20vec(indICs(2));
    disp(['IC = ' num2str(tx10) '   ' num2str(tx20)])
else
    indICs = [1;1];
end

% Model index
if issweep_IC
    indmodel = sweepindvec(3);
elseif issweep_nu
    indmodel = sweepindvec(1:2);
elseif issweep_rand_nu
    indmodel = sweepindvec(1);
else
    indmodel = master_settings.indnueval;
end        

% ***********************
%
% GET ICs FOR THIS SWEEP ITERATION
%    

% Initial condition
x0 = ICs.x0mat(indICs(1),indICs(2),:);  
x0 = x0(:);

% ***********************
%
% GET NOMINAL, PERTURBED MODELS FOR THIS ITERATION
%    

% Nominal, perturbed models
model_cell = sys.model_cell;
indnom = sys.indnom;
model = get_elt_multidim(model_cell, indnom);
model_nu = get_elt_multidim(model_cell, indmodel);

% Indices of output variables
inds_xr = model.inds_xr;

% Nominal model
model_nom_tag = group_settings.model_nom_tag;
switch model_nom_tag
    case 'default'
        model_nom_ind = indnom;
    case 'perturbed'
        model_nom_ind = indmodel;
end 

% Simulation model
model_sim_tag = group_settings.model_sim_tag;
switch model_sim_tag
    case 'default'
        model_sim_ind = indnom;
    case 'perturbed'
        model_sim_ind = indmodel;
end      




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Get preset count
presetcount = group_settings.presetcount;
switch preset

    %%    
    % *********************************************************************
    % *********************************************************************
    %
    % MI-DIRL
    %      
    % *********************************************************************
    % *********************************************************************

    
    case [{'dirl_nonlin'};{'irl_old'};{'irl'}]

        % ***********************
        %
        % ALGORITHM
        %        

        alg = preset;
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end

        % Is training (=1) or not (=0)
        istraining = group_settings.istraining;

        % Policy linear (=1) or nonlinear (=0)
        linpol1nonlinpol0 = 0;

        % Is DIRL (=1) or old IRL (=0)
        notirlold = ~(strcmp(preset, 'irl_old') || strcmp(preset, 'irl'));
       
        % Time to simululate for 
        tsim = group_settings.tsim;

        % IRL settings
        irl_setts = group_settings.irl_setts_cell{presetcount};

        % Number of loops executed for this preset
        numloops = irl_setts.numloops;

        % Which loops to train
        doloopvec = irl_setts.doloopvec;

        % Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
        sim_w = 0;
         

        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data_0 = master_settings.lq_data_0;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % ***********************
        %
        % LOOP SETTINGS
        %   

        % Sample period offset
        nTs_begin = 0; 

        % Holds loop settings
        loop_cell = cell(numloops,1);
        
        % Initialize loop settings
        switch numloops

            % ***********************
            %
            % SINGLE-LOOP
            %   

            case 1

            % System-specific settings
            switch systag                         
                case sysnames.pendulum
                    % Sample period
                    Ts = 1;
                    % x, y indices
                    tmp.indsx = (1:2)';
                    tmp.indsy = (1)';
                    % Learning settings
                    tmp.istar = 5;
                    tmp.nTs = 1;    
                    tmp.l = 15; 
                 case sysnames.vamvoudakis2010
                    % Sample period
                    Ts = 0.1;
                    % x, y indices
                    tmp.indsx = (1:2)';
                    tmp.indsy = 1;
                    % Learning settings
                    tmp.istar = 5;
                    tmp.nTs = 1;   
                    tmp.l = 15;
                    switch preset
                        case 'irl'
                            % Basis params
                alg_settings.basis_critic = group_settings.basis_critic; 
                alg_settings.c_0 = group_settings.basis_critic.c0;
                            % ICs
                            x0 = [10; 10];
                    end
            end


            tmp.Q = lq_data.Qirl;
            tmp.R = lq_data.R;    
            % Initial stabilizing design on nominal model
            tmp.K0 = lq_data_0.Kdirl;   


            % Set current loop settings
            loop_cell{1} = tmp;

            % ***********************
            %
            % DOUBLE-LOOP
            %   

            case 2

                % Sample period
                % System-specific settings
                switch systag
                    case sysnames.hsv
                        Ts = 2;         
                end

            for k = 1:numloops
                switch k
    
                    % Loop j = 1
                    case 1
                        % System-specific settings
                        switch systag
                            case sysnames.hsv
                                % x, y indices
                                tmp.indsx = 1;
                                tmp.indsy = 1;   
                                % Learning settings
                                tmp.istar = 10;  
                                tmp.nTs = 6 / Ts;
                                tmp.l = 15;                               
                        end  

                        tmp.Q = lq_data.lq_data_11.Q;
                        tmp.R = lq_data.lq_data_11.R;

%                         % Nominal design on nominal model
                        tmp.K0 = lq_data_0.lq_data_11.K; 

                        
    
                    % Loop j = 2   
                    case 2

                        % System-specific settings
                        switch systag
                            case sysnames.hsv
                                % x, y indices
                                tmp.indsx = (2:4)';
                                tmp.indsy = 2; 
                                % Learning settings
                                tmp.istar = 10;  
                                tmp.nTs = 1;
                                tmp.l = 15;   
                                tmp.S = diag([1 1 D2R D2R]);            
                        end  

                        tmp.Q = lq_data.lq_data_22.Q;
                        tmp.R = lq_data.lq_data_22.R;

                        % Initial stabilizing design on nominal model
                        tmp.K0 = lq_data_0.lq_data_22.K;                         
  
                end
    
                % Set current loop settings
                loop_cell{k} = tmp;
            end

        end

        % ***********************
        %
        % EXPLORATION NOISE
        %
        % Note: In post-transformation units (i.e., after applying S_u)
        %


        switch systag
            case sysnames.hsv
                noise.cos1_sin0 = [    0   
                                       0      ];
                noise.Amat = [       0 
                                     0    ];
                noise.Tmat = [       1 
                                     1   ];        
            case sysnames.pendulum
                noise.cos1_sin0 = [ 0   0      ];
                noise.Amat = [      0.5 0.1 ];
                noise.Tmat = [      10  5   ];                  
            case sysnames.vamvoudakis2010
                noise.cos1_sin0 = [ 0 ];
                noise.Amat = [      0 ];
                noise.Tmat = [      1 ];  

        end   

        switch numloops
            case 1
                noise.donoisevec = ones(m,1);
            case 2
                noise.donoisevec = doloopvec;
        end
        noise.tag = 'multivar_sum_sinusoid';        

        % If is old IRL, don't insert noise
        if ~notirlold
            noise.tag = '0';
        end


        % ***********************
        %
        % REFERENCE COMMANDS -- FOR TRAINING PHASE
        %


        % Determine whether or not to insert r(t)
        hasdort = isfield(group_settings, 'dort');
        if hasdort
            dort = group_settings.dort;
        else
            dort = 1;
        end

        % Proceed to insert r(t) if desired 
        if dort
            refcmdl = 'sum_sin';
        else
            refcmdl = 'zero';
        end


        % Trim
        x_e = model.trimconds.xe;

        switch m
            case 1
                x1_e = x_e(inds_xr(1));
                x1_m = x1_e; 
                biasvec = x1_m;
                nderivvec = 0;                
            case 2
                x1_e = x_e(inds_xr(1));
                x2_e = x_e(inds_xr(2));
                x1_m = x1_e;
                x2_m = x2_e;   
                biasvec = [x1_m; x2_m];
                nderivvec = [0; 0];
        end

        switch refcmdl

            % NO r(t)
            case 'zero'

                cos1_sin0 = zeros(m,1);
                Amat = zeros(m,1);
                Tmat = ones(m,1);

            % INSERT r(t)
            case 'sum_sin'
                
                switch systag

                    case sysnames.hsv

                        % FINAL
                        cos1_sin0 = [   0   0   1  
                                        0   0   1   ];
                        Amat = [        50  5   5   
                                        5 0.3   2.5   ];
                        Tmat = [        100 25  10  
                                        15  6   100     ]; 

                        Amat(2,:) = 0.1 * Amat(2,:);
                        % Convert \gamma amplitudes to rad
                        Amat(2,:) = D2R * Amat(2,:);



                    case sysnames.pendulum

                        % FINAL 
                        cos1_sin0 = [   0   0     ];
                        Amat = [        10  5    ];
                        Tmat = [        10 5  ];
           
        
                        % Perform deg -> rad conversions
                        Amat(1,:) = D2R * Amat(1,:);


                     case sysnames.vamvoudakis2010

                        % FINAL
                        cos1_sin0 = [   0          ];
                        Amat = [        1      ];
                        Tmat = [        2*pi / 5           ];   


                end


        end

        % Do x_3 loop
        switch systag
            case sysnames.hsv
                lenx3 = 1;
                dox3 = 1;
            otherwise
                lenx3 = 0; 
                dox3 = 0;             
        end

        % Do decaying exponential r(t) according to x_0 (=1) or not (=0)
        doexpx0 = group_settings.doexpx0;
        if doexpx0
            sy = model.lin.io.syd;
            x_e = model.trimconds.xe;
            r_sett_train.ty0 = ...
                (x0(model.inds_xr) - x_e(model.inds_xr));  
            r_sett_train.expx0avec = group_settings.expx0avec;
        end


        % ***********************
        %
        % TRANINING SETTINGS
        %


        % DEBUGGING: Use the nominal linearization A for w = f(x) - A x at
        % nominal trim (=1) or simulation trim (=0)
        % NOTE: Not functional. Keep high.
        wnom1sim0 = 1;


        % Reference command settings
        switch refcmdl
            case 'zero'
                r_sett_train.tag = 'sum_sin';   
            case 'sum_sin'
                r_sett_train.tag = 'sum_sin'; 
        end
        switch refcmdl
            case {'sum_sin';'zero'}
                r_sett_train.biasvec = biasvec;
                r_sett_train.nderivvec = nderivvec;
                r_sett_train.dorefvec = noise.donoisevec;
                r_sett_train.cos1_sin0 = cos1_sin0;
                r_sett_train.Amat = Amat;
                r_sett_train.Tmat = Tmat;
        end
        



        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;     



    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case 'lq_servo_inout'
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'lq_servo_inout';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
        % Get preset count
        presetcount = group_settings.presetcount;

        % Reference command
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);     

        % Policy linear (=1) or nonlinear (=0)
        linpol1nonlinpol0 = 1;

        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);
        lq_data_0 = master_settings.lq_data_0;

        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Time to simululate for 
        tsim = group_settings.tsim;
 

        % Nominal, simulation model
        switch curralg
            case algnames.lq_opt_nu
                model_nom_tag = group_settings.model_sim_tag;
                model_sim_tag = group_settings.model_sim_tag;
            otherwise
                model_nom_tag = group_settings.model_nom_tag;
                model_sim_tag = group_settings.model_sim_tag;                
        end  


        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           


    % *********************************************************************
    %
    % cFVI TRACKING
    %
    
    case 'cfvi_tracking'
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'cfvi_tracking';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       


        % Get preset count
        presetcount = group_settings.presetcount;

        % Reference command
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Time to simululate for 
        tsim = group_settings.tsim;

        % Nominal, simulation model
        model_nom_tag = group_settings.model_nom_tag;
        model_sim_tag = group_settings.model_sim_tag;     


        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        % Relative path to cFVI data
        relpath_py = master_settings.relpath_py;
        filename_py =  master_settings.filename_py;

        % Append 'cFVI' or 'rFVI' tag to file path, name
        relpath_py = [relpath_py curralg '/'];
        filename_py = [curralg filename_py];

        % Evaluate FVI via table lookup (=1) or Python call (=0)  
        tbl1_py0 = group_settings.tbl1_py0;        
        
        % ***********************
        %
        % n-D GRID DATA
        %  

        if tbl1_py0

            % Load cFVI data -- n-D
            cfvi_data = load([relpath_py filename_py]);
            
            cfvi_x_tbl_min = cfvi_data.x_tbl_min;
            cfvi_x_tbl_max = cfvi_data.x_tbl_max;
            cfvi_x_tbl_nxpts = cfvi_data.x_tbl_nxpts;
            cfvi_u_tbl =  cfvi_data.u_tbl;
            
            % ***********************
            %
            % cFVI GRID VECTORS -- n-D
            %  
            
            % Get number of states in cFVI plant
            n_cfvi = length(cfvi_x_tbl_min);
            
            % Cell array containing grid vectors
            cfvi_xgridvec_cell = cell(n_cfvi, 1);
            
            % Initialize grid vectors
            for i = 1:n_cfvi
                cfvi_xgridvec_cell{i} = ...
                    linspace(cfvi_x_tbl_min(i), cfvi_x_tbl_max(i),...
                    cfvi_x_tbl_nxpts(i))';
            end

        else

            % Cost J(x) settings
            Jx_sett = group_settings.Jx_sett;

            % Evaluation settings
            seed = Jx_sett.seed_eval;
            iter = Jx_sett.iter_eval;
            fs = Jx_sett.fs;
            fs_return = Jx_sett.fs_return;
        
        end


        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           

    % *********************************************************************
    %
    % VAMVOUDAKIS, LEWIS (2010) -- SPI
    %
    
    case 'spi'
        
        
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'spi';   
     
        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % Penalties
        Q = lq_data.Qirl;
        R = lq_data.R; 

        
        switch systag
            case sysnames.vamvoudakis2010
                % Probing noise signal
                noise.tag = 'sum_sinusoids';        
                % A cos(t)
                noise.cos1_sin0 = 1;
                noise.scalevec = 5;
                noise.freqvec = 1;
                % Length of learning window [0, t_f].
                % I.e., time to insert probing noise for (sec).
                tf = 500;
%                 tf = 6.73;
                % NN tuning gains
                alpha1 = 10;
                alpha2 = 10;        
                % NN tuning parameters
                F1 = zeros(group_settings.basis_critic.N,1);   % Not used  
                F2 = 5*eye(group_settings.basis_critic.N);
        end
                  
        
        % ICs for critic NN
        c_0 = group_settings.basis_critic.c0;
        
        % ICs for actor NN
        w_0 = group_settings.basis_critic.c0;
                   
        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           


    case 'radp'
        
        
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'radp';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
                
        % Probing noise signal
        noise.tag = 'sum_sinusoids';        
        % A cos(t)
        noise.cos1_sin0 = 1;
        noise.scalevec = 5;
        noise.freqvec = 1;
        
        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % Penalties
        Q = lq_data.Qirl;
        R = lq_data.R;     

        % Critic NN activation function parameters (cf. eqn. (13))
        Phi = group_settings.basis_critic; 
        
        % Actor NN activation function parameters (cf. eqn. (14))
        Psi = group_settings.basis_actor_no_g;
        
        % Pack basis parameters
        basis.Phi = Phi;
        basis.Psi = Psi;           
                        
        % Number of samples to collect for least squares minimizations
        l = 15;
               
        % Number of iterations i^* to terminate algorithm after (if
        % termination mode is 'manual')
        istar = 10;
        
        % Length of learning window [0, t_f] (sec)
        tf = 15;
                    
        % Tag corresponding to the initial stabilizing policy u_0(x) (cf.
        % Assumption 4.2).
        u_0.tag = 'vamvoudakis_2010';
                       
        
        % IC for actor NN w_0 \in R^{N_2}
        c0 = group_settings.basis_critic.c0(1:3);
        w_0 = [-c0(2); -2*c0(3); -c0(2)/2; -c0(3)];
 
        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry (not used)
        legend_entry = 'RADP';        

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;                   

    % *********************************************************************
    %
    % BIAN, JIANG (2021) -- CT-VI
    %
    
    case 'ctvi'
         
        % ***********************
        %
        % ALGORITHM AND SYSTEM
        %        
        
        alg = 'ctvi';
        
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %     
        
        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = get_elt_multidim(lq_data_cell, indnom);

        % Penalties
        Q = lq_data.Qirl;
        R = lq_data.R;    
        
        % Critic NN basis 
        Phi = group_settings.basis_critic;           
 
        % Hamiltonian NN basis \Psi(x)
        Psi = group_settings.basis_actor_no_g;        
        
        % Hamiltonian NN basis \Theta(x)
        Theta = group_settings.basis_hamiltonian.Theta;
                
        % Store basis parameters
        basis.Phi = Phi;
        basis.Psi = Psi;
        basis.Theta = Theta;
        
        % Probing noise signal
        noise.tag = 'sum_sinusoids';        
        % A cos(t)
        noise.cos1_sin0 = 1;
        noise.scalevec = 5;
        noise.freqvec = 1;
        
        % Initial policy u_0(x)
        u_0.tag = 'vamvoudakis_2010';
 
        % Length of time to tune weights for [0, s_f] (sec)
        sf = 50;

        % Length of learning window [0, t_f] (sec)
        tf = 50;  
                
        % Max stepsize for ode45
        maxstep = 1e-1;
        
        % Whether to integrate Hamiltonian function (13) via ode45 (=1) or
        % manually (=0)
        int_H_ode45_1_man_0 = 0;
       
        
        % Initial critic weights c_0 \in R^{N_1}
        c_0 = group_settings.basis_critic.c0;

        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry (not used)
        legend_entry = 'CT-VI';    
        
        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;

    % *********************************************************************    
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: PRESET TAG NOT RECOGNIZED ***');  
       
end





%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE ALGORITHM SETTINGS/DESIGN PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% GENERAL SETTINGS
%
% *************************************************************************

% Preset tag
alg_settings.preset = preset;

% Plot settings -- general
plot_settings.legend_entry = legend_entry;
plot_settings.plotfolder = plotfolder;

% Write plot settings
alg_settings.plot_settings = plot_settings;


% *************************************************************************
%
% ALGORITHM-SPECIFIC SETTINGS
%
% *************************************************************************

switch alg

    % *********************************************************************
    %
    % DIRL
    %
    
    case [{'dirl_lin'};{'dirl_nonlin'};{'irl_old'};{'irl'}]

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.indmodel = indmodel;

        % Whether or not to perform learning in each loop
        alg_settings.doloopvec = irl_setts.doloopvec;       

        % Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
        alg_settings.sim_w = sim_w;
  
        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = ...
            group_settings.lin1nonlin0vec(presetcount);

        % Policy linear (=1) or nonlinear (=0)
        alg_settings.linpol1nonlinpol0 = linpol1nonlinpol0;

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = master_settings.pfavec; 
        end  

        % Exploration noise
        alg_settings.noise = noise;    

        % Reference command settings -- training phase
        alg_settings.r_sett_train = r_sett_train;

        % Reference command settings
        alg_settings.r_sett = group_settings.r_sett;

        % Loop settings
        alg_settings.loop_cell = loop_cell;

        % Sample period
        alg_settings.Ts = Ts;

        % Sampling begin offset
        alg_settings.nTs_begin = nTs_begin;

        % Do x_3 loop
        alg_settings.dox3 = dox3;
        alg_settings.lenx3 = lenx3;

        % Is training preset (=1) or not (=0)
        istraining = group_settings.istraining;
        alg_settings.istraining = istraining;
        
        % Do reference command r(t) injection (=1) or not (=0)
        alg_settings.dort = group_settings.dort;

        % Overwrite current controller (=1) or not (=0)
        alg_settings.updatecontroller = istraining && ~issweep_IC;
  
        % Use the nominal linearization A for w = f(x) - A x at nominal
        % trim (=1) or simulation trim (=0)
        alg_settings.wnom1sim0 = wnom1sim0;

        % ICs, simulation time
        alg_settings.x0 = x0;  
        alg_settings.tsim = group_settings.tsim; 


    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case 'lq_servo_inout'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.model = model;
        alg_settings.model_nu = model_nu;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % Policy linear (=1) or nonlinear (=0)
        alg_settings.linpol1nonlinpol0 = linpol1nonlinpol0;


        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = master_settings.pfavec;
        end   

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Controller parameters
        if linpol1nonlinpol0
            alg_settings.K = K;
        else
            alg_settings.P = P;
        end

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 
  


    % *********************************************************************
    %
    % cFVI
    %
    
    case 'cfvi'

    % *********************************************************************
    %
    % cFVI TRACKING
    %
    
    case 'cfvi_tracking'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.model = model;
        alg_settings.model_nu = model_nu;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % Algorithm tag
        alg_settings.algtag = curralg;

        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = master_settings.pfavec; 
        end

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        
        if tbl1_py0
            % Policy lookup table, grid vectors
            alg_settings.xgridvec_cell = cfvi_xgridvec_cell;
            alg_settings.u_tbl = cfvi_u_tbl;
        else
            alg_settings.seed = seed;
            alg_settings.iter = iter;  
            alg_settings.fs = fs;
            alg_settings.fs_return = fs_return;            
        end

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim;

    % *********************************************************************
    %
    % VAMVOUDAKIS, LEWIS (2010) -- SPI
    %
    
    case 'spi'
  
        alg_settings.alg = alg;
              
        alg_settings.Q = Q;
        alg_settings.R = R;
        alg_settings.noise = noise;
        alg_settings.tf = tf;
        
        alg_settings.alpha1 = alpha1;
        alg_settings.alpha2 = alpha2;
        alg_settings.F1 = F1;
        alg_settings.F2 = F2;
        
        alg_settings.c_0 = c_0;
        alg_settings.w_0 = w_0;
        alg_settings.x0 = x0;

        % Basis
        alg_settings.basis_critic = group_settings.basis_critic;

        % Nominal model
        alg_settings.model_nom_ind = model_nom_ind;        
        % Simulation model
        alg_settings.model_sim_ind = model_sim_ind;

    % *********************************************************************
    %
    % JIANG, JIANG (2014) -- RADP 
    %
    
    case 'radp'
  
        alg_settings.alg = alg;
        
        alg_settings.Q = Q;
        alg_settings.R = R;
        alg_settings.basis = basis;
        alg_settings.noise = noise;
        
        alg_settings.l = l;
        alg_settings.istar = istar;
        
        % Nominal model
        alg_settings.model_nom_ind = model_nom_ind;        
        % Simulation model
        alg_settings.model_sim_ind = model_sim_ind;   

        alg_settings.tf = tf;
        
        alg_settings.u_0 = u_0;
        
        alg_settings.x0 = x0;  
        
        alg_settings.w_0 = w_0;  

    % *********************************************************************
    %
    % BAIN, JIANG (2022) -- CT-VI
    %
    
    case 'ctvi'        
             
        alg_settings.alg = alg;

        alg_settings.Q = Q;
        alg_settings.R = R;
        alg_settings.basis = basis;
        alg_settings.noise = noise;
        alg_settings.u_0 = u_0;
        
        alg_settings.sf = sf;
        alg_settings.tf = tf;
        alg_settings.maxstep = maxstep;
        alg_settings.int_H_ode45_1_man_0 = int_H_ode45_1_man_0;

        alg_settings.x0 = x0;
        alg_settings.c_0 = c_0;

        % Nominal model
        alg_settings.model_nom_ind = model_nom_ind;        
        % Simulation model
        alg_settings.model_sim_ind = model_sim_ind; 

        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('**** ERROR: ALGORITHM TAG NOT RECOGNIZED ***');  
       
end

