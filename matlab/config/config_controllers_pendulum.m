function master_settings = config_controllers_pendulum(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG LQ SERVO CONTROLLERS
%
% [ ***** ANONYMIZED ***** ]  
%
% 2023-03-29
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
model_cell = sys.model_cell;
nummodels = sys.nummodels;
indnom = sys.indnom;

% Current system being executed
systag = master_settings.systag;

% Controller initialization controls
init1_load0_lq = master_settings.init1_load0_lq;

% Relative path to controller data
relpath_lq = master_settings.relpath_lq;

% String with system specifier
relpath_sys = [systag '/'];

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE CONTROLLERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% LQ SERVO
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS
%
% NOTE: State partitioned as (see below)
%
%      = [  z   (INTEGRAL AUGMENTATION NOT USED -- empty)
%           y
%           x_r ]
%

% WHAT LUTTER USED
Q1 = diag([1 0.1]);
R1 = 0.5;



% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS -- INITIAL STABILIZING CONTROLLER
%


Q10 = diag([0.5 0.25]);
R10 = 0.01;



% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER
%   

% Relative path to controller params
relpath_ctrl_tmp = [relpath_lq];

% File name to save to
filename_tmp = 'lq_data_cell.mat';

% Cell array to contain LQ data
lq_data_cell = cell(nummodels,1);

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.hasintaug = hasintaug;


    % Loop over model cell array
    for i = 1:nummodels

        % Get current model
        alg_settings.model_d = model_cell{i};
 
        % Initialize controller 
        lq_datai = config_lqr(alg_settings);

        % Store lq_data in array
        lq_data_cell{i} = lq_datai;

    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp)

    % Save data 
    varname = 'lq_data_cell';
    save([relpath_ctrl_tmp filename_tmp], varname);

end

% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER -- INITIAL STABILIZING CONTROLLER
%   

% File name to save to
filename_tmp0 = 'lq_data_0.mat';

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q10;
    alg_settings.R1 = R10;
    alg_settings.hasintaug = hasintaug;

    % Get current model -- nominal model
    alg_settings.model_d = model_cell{indnom};

    % Initialize controller 
    lq_data_0 = config_lqr(alg_settings);

    % Save data 
    varname = 'lq_data_0';
    save([relpath_ctrl_tmp filename_tmp0], varname);

end

% Load controllers
data = load([relpath_ctrl_tmp filename_tmp]);
lq_data_cell = data.lq_data_cell;
      
% Load initial stabilizing controllers
data = load([relpath_ctrl_tmp filename_tmp0]);
lq_data_0 = data.lq_data_0;




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE DATA
% 
% *************************************************************************
% *************************************************************************       
% *************************************************************************

% ***********************
%
% LQ SERVO
%   

master_settings.lq_data_cell = lq_data_cell;
master_settings.pfavec = lq_data_cell{indnom}.pfavec;

% ***********************
%
% LQ SERVO -- INITIAL STABILIZING CONTROLLER
%   

master_settings.lq_data_0 = lq_data_0;


