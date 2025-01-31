function [master_settings, group_settings_master, preset_list_cell] = ...
    config_settings(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PROGRAM SETTINGS
%
% [ ***** ANONYMIZED ***** ]
%
% 2023-03-29
%
% This program performs the master config of this entire MATLAB code suite.
% The initialization process includes:
%
%   * Configure relative paths to programs, data
%   * Configure algorithm settings
%   * Configure master plot formatting
%   * Configure frequency response plot settings
%   * System initialization
%       * Modeling error parameter values \nu
%       * Integral augmentation settings
%       * Configure misc system settings -- config_sys.m
%   * Configure relative paths to data for selected system
%   * Configure controllers for selected system
%   * Configure individual preset group settings 
%       -- config_preset_group_cell.m  
%       -- config_preset_group.m 
%       
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
%
% UNPACK USER-SET SETTINGS
% 
% *************************************************************************
% *************************************************************************

% System names
sysnames = master_settings.sysnames;

% System tag
systag = master_settings.systag;

% Preset group
preset_group = master_settings.preset_group;

% Preset group list
preset_group_list = master_settings.preset_group_list;

% Number of preset groups executed
numgroups = master_settings.numgroups;


% ***********************
%
% SETTINGS -- SYSTEM/CONTROLLER INIT
%
% Initialize (=1) or load from previous (=0)
%

% Model
init1_load0_model = 0;

% LQ servo controllers
init1_load0_lq = init1_load0_model;

% Pull DIRL data from sweep data location (=1) or temp data location (=0)
dirldatasweep1temp0 = 1;

% Model/controller init
master_settings.init1_load0_model = init1_load0_model;
master_settings.init1_load0_lq = init1_load0_lq;
master_settings.dirldatasweep1temp0 = dirldatasweep1temp0;


%%
% *************************************************************************
% *************************************************************************
%
% INCLUDE FILE PATHS
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% INCLUDE UTILITY FOLDER
%
addpath('util');
addpath('util/plot');
addpath('util/DIRL');
addpath('util/indexing/');

% ***********************
%
% INCLUDE AIRCRAFT UTILITY FOLDERS
%
relpath_data_main = '01 data/';

% ***********************
%
% INCLUDE SYSTEM UTILITY FOLDERS
%
addpath([relpath_data_main 'hsv/']);
addpath([relpath_data_main 'pendulum/']);
addpath([relpath_data_main 'vamvoudakis2010/']);

% ***********************
%
% INCLUDE CONFIG FUNCTIONS FOLDER
%
addpath('config');

% ***********************
%
% INCLUDE EVALUATION FUNCTIONS FOLDER
%
addpath('eval_functs');

% ***********************
%
% INCLUDE ALGORITHM FOLDER
%
addpath('algs');

% ***********************
%
% INCLUDE PLOT FUNCTIONS FOLDER
%
addpath('plot');


% ***********************
%
% RELATIVE PATHS TO PROGRAM DATA
%
relpath_data_root = '01 data/';

% ***********************
%
% ABSOLUTE PATH TO MATLAB FOLDER
%
master_settings.abspath = [pwd '/'];



%%
% *************************************************************************
% *************************************************************************
%
% SYSTEM INIT
% 
% *************************************************************************
% *************************************************************************

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;

% ***********************
%
% RELATIVE PATH TO CURRENT MODEL
%

% Root path
relpath_model = [relpath_data_root systag '/'];

% Path to model data
relpath_model_data = [relpath_model 'model/'];

% Filename of model data
filename_model = 'model.mat';

% Group name
groupnamei = preset_group_list{1};  

% ***********************
%
% SETTINGS -- MODELING ERROR PARAMETERS \nu USED FOR EVALUATIONS
%

switch systag

    % ***********************
    %
    % HSV
    %

    case sysnames.hsv

        % Tag of which model cell array to use
        modelcelltag = '0_10_25';

        % Tag of which design to use
        designtag = 'NeurIPS_2024';

        % ***********************
        %       
        % DETERMINE WHICH MODELING ERROR PARAMETER TO USE
        %   
   
        % Modeling error type
        nutag = 'CL';
    
        % Perturbation params
        nuvec = [1; 0.9; 0.75];
        nu1vec = nuvec;
        nu2vec = [nan];
        
        % Which perturbations to plot data for
        nuvecplot = [1; 0.9; 0.75];


        % Which single perturbation value to use for modeling error
        % evaluation studies
        if contains(groupnamei,'nom')
            nueval = 1;
        else
            if contains(groupnamei,'10pct')
                nueval = 0.9;
            else
                nueval = 0.75;
            end
        end

        % Name of model initialization program
        model_init_program = 'hsv_wang_stengel_2000_init';

        % System name (for displaying on plots)
        systag_disp = 'HSV';


    % ***********************
    %
    % PENDULUM
    %

    case sysnames.pendulum

        % Tag of which design to use
        designtag = 'NeurIPS_2024';

        % Tag of which model cell array to use
        modelcelltag = 'm0_10_25';

        % Tag of which modeling error type to use
        nutag = 'L';

        % Perturbation parms
        nuvec = [1; 0.9; 0.75]; 

        % Which perturbations to plot data for
        nuvecplot = nuvec;

        % Which single perturbation value to use for modeling error
        % evaluation studies
        if contains(groupnamei,'nom')
            nueval = 1;
        else
            if contains(groupnamei,'10pct')
                nueval = 0.9;
            else
                nueval = 0.75;
            end
        end

        % Name of model initialization program
        model_init_program = 'pendulum_init';

        % System name (for displaying on plots)
        systag_disp = 'Pendulum';

    
    % ***********************
    %
    % VAMVOUDAKIS, LEWIS SPI (2010)
    %

    case sysnames.vamvoudakis2010

        % Tag of which design to use
        designtag = 'NeurIPS_2024';

        % Tag of which modeling error type to use
        nutag = '22';

        % Tag of which model cell array to use
        modelcelltag = 'main';   

        % Perturbation parms
        nuvec = [1; 1.1; 1.25];

        % Which perturbations to plot data for
        nuvecplot = nuvec;

        % Which single perturbation value to use for modeling error
        % evaluation studies
        if contains(groupnamei,'nom')
            nueval = 1;
        else
            if contains(groupnamei,'10pct')
                nueval = 1.1;
            else
                nueval = 1.25;
            end
        end

        % Name of model initialization program
        model_init_program = 'vamvoudakis2010_init';

        % System name (for displaying on plots)
        systag_disp = 'SOS';  

end

% ***********************
%
% MODEL INDICES -- NOMINAL MODEL
%

% Index of nominal model
indnom = find(nuvec == 1);

% Indices of non-nominal models
indsnotnom = setdiff((1:size(nuvec,1)),indnom);

% ***********************
%
% MODEL INDICES -- MODELS TO PLOT DATA FOR
%        

indsmodelplot = [];

for mcnt = 1:size(nuvecplot,1)          
    ind = find(nuvec == nuvecplot(mcnt));
    if ~isempty(ind)
        indsmodelplot = [indsmodelplot; ind];
    end
end

nummodelsplot = size(indsmodelplot,1);

% Overwrite this settings if is a \nu sweep
switch systag
    case sysnames.hsv
        % Nothing
    otherwise
        nu1vec = nuvec;
        nu2vec = 1;
end

% Add extension to relative path
relpath_model_data = [relpath_model_data nutag '/' ...
          modelcelltag '/' designtag '/'];



% ***********************
%
% MODEL INDICES -- MODEL TO PERFORM MODELING ERROR EVALUATIONS FOR
%  

if length(nueval) == 1
    indnueval = find(nuvec == nueval);
    nueval_str = num2filename(nueval);
else
    indnu1eval = find(nu1vec == nueval(1));
    indnu2eval = find(nu2vec == nueval(2));
    indnueval = [indnu1eval; indnu2eval];
    nueval_str = ...
        num2filename([num2str(nueval(1)) '_' num2str(nueval(2))]);
end


% ***********************
%       
% CONTROLLER HAS INTEGRAL AUGMENTATION (=1) OR NOT (=0)
% 

switch systag
    case {sysnames.pendulum; sysnames.vamvoudakis2010}
        hasintaug = 0; 
    otherwise
        hasintaug = 1; 
end

% Set
master_settings.hasintaug = hasintaug;

% ***********************
%
% SET TICK VALUES FOR \nu ABLATION 3D PLOTS
%  

% Tick spacing for \nu ablations
nu_tick_space = 0.05;

nu_ticks_cell = cell(2,1);
nuvec_cell = cell(2,1);
nuvec_cell{1} = nu1vec;
nuvec_cell{2} = nu2vec;

for i = 1:2
    nuivec = nuvec_cell{i};
    if nuivec(1) < nuivec(end)
        nuiticks = (nuivec(1):nu_tick_space:nuivec(end))';        
    else
        nuiticks = (nuivec(end):nu_tick_space:nuivec(1))';        
    end
    % Clip greatest element for display
    nuiticks = nuiticks(nuiticks < max(nuivec)); 
    % Store
    nu_ticks_cell{i} = nuiticks;
end



    

% *************************************************************************
%
% INITIALIZE/LOAD MODEL
%
% *************************************************************************

% ***********************
%
% MISC SETTINGS
%

% Initialize model if desired
if init1_load0_model

    % Individual settings
    switch systag
    
        case sysnames.hsv

            % Set tag
            setts.nutag = nutag;          
          
            % Set perturbation parameters
            nu1vec = nuvec;
            nu2vec = 1;

            % Set perturbation vectors
            setts.nu1vec = nu1vec;
            setts.nu2vec = nu2vec;      
          
        otherwise

            % Perturbation parameters
            setts.nuvec = nuvec;    

            nu1vec = nuvec;
            nu2vec = 1;
        
    end

    % Settings
    setts.indnom = indnom;
    setts.relpath_data = relpath_model_data;
    setts.filename = filename_model;  
    setts.designtag = designtag;
    % Init
    eval([model_init_program '(setts)'])
    clear setts;

end

% Load model parameters
models = load([relpath_model_data filename_model]);
models = models.model_struct;
model_cell = models.model_cell; 

% Get dimensions of system cell array
numnu1 = size(model_cell,1);
numnu2 = size(model_cell,2);
nummodels = numnu1*numnu2;

% Configure system settings
sys.tag = systag;
sys.tag_disp = systag_disp;
sys.relpath_data = relpath_model_data;
sys.nutag = nutag;

% Store model cell arrays, perturbation params
sys.model_cell = model_cell;   
sys.nummodels = size(model_cell,1);
sys.indnom = indnom;
sys.indsnotnom = indsnotnom;
sys.nuvec = nuvec;
sys.nu1vec = nu1vec;
sys.nu2vec = nu2vec;
sys.nuvecplot = nuvecplot;

% Initialize system
[sys, sys_plot_settings] = config_sys(sys);

% Get nominal model
model = get_elt_multidim(model_cell,indnom);

% IC distributions:
% x_init_train: Bounds of initial condition distribution for training
%   (transformed units, NOT shifted to trim)
% x_init: Bounds of initial condition distribution for evaluation of
%   expected return (pre-transformed units, NOT shifted to trim)
switch systag
    case sysnames.hsv
        Vscl = model.lin.yscl(1);
        x_init_train = [150*Vscl; 1.5; 5; 5; 0.01];
        x_init = [100*Vscl; 1; 0.01; 0.01; 0.01];
        threshmat_x = [ 1   10*Vscl 50                         
                        2   0.1     50  
                        1   1*Vscl  25
                        2   0.01    25  ];
    case sysnames.pendulum
        x_init_train = [pi/2; pi];
        x_init = [pi; pi/4];  
        threshmat_x = [1 5*D2R 5];
    case sysnames.vamvoudakis2010
        x_init_train = [2.5; 2.5];
        x_init = [1; 1];  
        threshmat_x = [1 0.1 5];
    otherwise
        x_init_train = [];  
        x_init = []; 
        threshmat_x = [];
end
sys_plot_settings.x_init_train = x_init_train;
sys_plot_settings.x_init = x_init;
sys_plot_settings.threshmat_x = threshmat_x;

% Store system parameters
master_settings.sys = sys;
master_settings.systag_disp = systag_disp;

% Store settings in system plot settings
sys_plot_settings.nu_ticks_cell = nu_ticks_cell;

% Store plot settings
master_settings.sys_plot_settings = sys_plot_settings;

% Save dimensions of system cell array
master_settings.numnu1 = numnu1;
master_settings.numnu2 = numnu2;
master_settings.nummodels = nummodels;


%%
% *************************************************************************
% *************************************************************************
%
% RELATIVE PATHS TO DATA FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% RELATIVE PATHS TO DIRL DATA
%

% Root path
relpath_dirl = [relpath_model 'DIRL/'];

% Relative path to nominal model
relpath_dirl_nom = [relpath_dirl 'nu_1/'];

% Path to this value of \nu for evaluation
relpath_dirl_error = [relpath_dirl 'nu_' nueval_str '/'];

% File name of DIRL data
if dirldatasweep1temp0
    filename_data_dirl = 'out_data_cell_master';
else
    filename_data_dirl = 'dirl_data';
end

% ***********************
%
% RELATIVE PATHS TO DIRL DATA -- x_0 SWEEP TRAINING DATA
%

relpath_dirl_sweep = [relpath_model 'dirl_sweep/'];
relpath_dirl_sweep = [relpath_dirl_sweep ...
            nutag '/' modelcelltag '/' designtag '/'];

% ***********************
%
% RELATIVE PATHS TO EVALUATION DATA
%

relpath_data_eval = [relpath_model 'eval_data/'];
relpath_data_eval = [relpath_data_eval ...
    nutag '/' modelcelltag '/' designtag '/'];


% ***********************
%
% RELATIVE PATHS TO LQ SERVO DATA
%

relpath_lq = [relpath_model 'lq_servo/' nutag '/' modelcelltag ...
   '/' designtag '/'];

% Store relative paths to LQ servo data in master settings
master_settings.relpath_lq = relpath_lq;

% ***********************
%
% PYTHON PATH SETUP
%

% Relative path to base folder of current Python code
relpath_py_code = '../python/';

% Set up PATH variable. This is for system calls to the python code
pyExec = 'C:\Users\ZZZ\anaconda3\';    % Absolute path to your python.exe
if contains(pyExec, 'ZZZ')
    error(['ERROR: Please configure your Python path. ' ...
        'See variable "pyExec" in config_settings.m'])
end
pyRoot = fileparts(pyExec);
p = getenv('PATH');
p = strsplit(p, ';');
addToPath = {
   pyRoot
   fullfile(pyRoot, 'Library', 'mingw-w64', 'bin')
   fullfile(pyRoot, 'Library', 'usr', 'bin')
   fullfile(pyRoot, 'Library', 'bin')
   fullfile(pyRoot, 'Scripts')
   fullfile(pyRoot, 'bin')
};
p = [addToPath(:); p(:)];
p = unique(p, 'stable');
p = strjoin(p, ';');
setenv('PATH', p);

% ***********************
%
% RELATIVE PATHS TO PYTHON DATA
%

% Relative path
relpath_py = [relpath_model 'python/'];
relpath_py = [relpath_py designtag '/'];

% System tags -- Python
sysnames_py.hsv = 'hsvintaug_quad';
sysnames_py.pendulum = 'Pendulum_QuadCost';
sysnames_py.vamvoudakis2010 = 'vamvoudakis2010_quad';

% File names, system tag
switch systag
    case sysnames.hsv
        systag_py = sysnames_py.hsv;
        filename_py_base = ['_' sysnames_py.hsv];
    case sysnames.pendulum
        systag_py = sysnames_py.pendulum;
        filename_py_base = ['_' sysnames_py.pendulum];        
    case sysnames.vamvoudakis2010
        systag_py = sysnames_py.vamvoudakis2010;
        filename_py_base = ['_' sysnames_py.vamvoudakis2010];  
    otherwise
        systag_py = '';
        filename_py_base = '';  
end

filename_py = [filename_py_base '_tbl_data.mat'];
filename_py_2d = [filename_py_base '_tbl_data_2D.mat'];



%%
% *************************************************************************
% *************************************************************************
%
% INITIALIZE/LOAD CONTROLLERS FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************

% Tag of design being executed
master_settings.designtag = designtag;

% Initalize/load controllers
master_settings = eval(['config_controllers_' systag '(master_settings)']);

%%
% *************************************************************************
% *************************************************************************
%
% ALGORITHM INIT
% 
% *************************************************************************
% *************************************************************************

% Algorithm names
algnames.mi_dirl = 'IPA';
algnames.cfvi = 'cFVI';
algnames.rfvi = 'rFVI';
algnames.irl = 'IRL';
algnames.spi = 'SPI';
algnames.radp = 'RADP';
algnames.ctvi = 'CT-VI';

% Set names
master_settings.algnames = algnames;

%%
% *************************************************************************
% *************************************************************************
%
% PLOT FORMATTING
% 
% *************************************************************************
% *************************************************************************


% Master plot settings
psett_master = init_psett(master_settings);

% SET MASTER PLOT SETTINGS
master_settings.psett_master = psett_master;

%%
% *************************************************************************
% *************************************************************************
%
% SWEEP INIT
% 
% *************************************************************************
% *************************************************************************

% Array of sweep type names
sweeptypenames.IC = 'IC';
sweeptypenames.nu = 'nu';
sweeptypenames.rand_nu = 'rand_nu';
sweeptypenames.plot = 'plot';

%%
% *************************************************************************
% *************************************************************************
%
% SAVE SETTINGS
%
% *************************************************************************
% *************************************************************************

% System indices
master_settings.indsmodelplot = indsmodelplot;
master_settings.nummodelsplot = nummodelsplot;
master_settings.indnueval = indnueval;

% Store relative paths to DIRL data
master_settings.relpath_dirl = relpath_dirl;
master_settings.relpath_dirl_nom = relpath_dirl_nom;
master_settings.relpath_dirl_error = relpath_dirl_error;
master_settings.filename_data_dirl = filename_data_dirl;

% Store relative paths to DIRL x_0 sweep data in master settings
master_settings.relpath_dirl_sweep = relpath_dirl_sweep;

% Store relative path to current Python code
master_settings.relpath_py_code = relpath_py_code;

% Store system tag and names for Python
master_settings.sysnames_py = sysnames_py;
master_settings.systag_py = systag_py;

% Store relative path and filename to Python data in master settings
master_settings.relpath_py = relpath_py;
master_settings.filename_py_base = filename_py_base;
master_settings.filename_py = filename_py;
master_settings.filename_py_2d = filename_py_2d;

% Store relative path and filename to evaluation data in master settings
master_settings.relpath_data_eval = relpath_data_eval;

% Array of sweep type names
master_settings.sweeptypenames = sweeptypenames;

% Is a re-plot
master_settings.isreplot = 0;



%%
% *************************************************************************
% *************************************************************************
%
% METHOD/SYSTEM/DESIGN PRESET LIST AND SETTINGS
%
% This is a list of tags correspond to the presets to execute for the
% selected preset group. Each preset consists of settings which include the
% specific
%
%   Algorithm/methodology (e.g., dEIRL, FBL etc.)
%   System (e.g., HSV system)
%   Design (with numerical design parameters)
%
% Below is the selection of the preset list, along with initialization of
% any preset group settings which are shared among all the designs in the
% group (e.g., if all designs share the same system, or the same Q, R
% matrices, etc.)
%
% *************************************************************************
% *************************************************************************


% Preset group cell
group_settings_master = ...
    config_preset_group_cell(master_settings);

% Each entry contains the presets executed for the respective group
preset_list_cell = cell(numgroups, 1);

% Initialize 
for i = 1:numgroups

    preset_groupi.group_settings = group_settings_master{i};
    preset_groupi.tag = preset_group;

    [preset_list_cell{i}, group_settings_master{i}] = ...
                config_preset_group(preset_groupi, master_settings);

end

% Get total number of presets executed -- including sweeps
numpresets_tot = 0;
for i = 1:numgroups

    % Group settings
    group_settingsi = group_settings_master{i};

    % Number of presets in this group
    numpresetsi = group_settingsi.numpresets;

    % Extract sweep settings
    sweepsettsi = group_settingsi.sweepsetts;

    % Is a sweep preset group (=1) or not (=0)
    issweep = sweepsettsi.issweep;

    % Total number of sweep iterations
    if issweep
        numsweepits = prod(sweepsettsi.sweepsizevec);
    else
        numsweepits = 1;
    end

    % Increment total
    numpresets_tot = numpresets_tot + numsweepits * numpresetsi;

end

% Store
master_settings.numpresets_tot = numpresets_tot;
