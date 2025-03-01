function master_settings = config_controllers_hsv(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG HSV CONTROLLERS
%
% [ ***** ANONYMIZED ***** ] 
%
% 2023-04-13
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
numnu1 = master_settings.numnu1;
numnu2 = master_settings.numnu2;
nummodels = master_settings.nummodels;
indnom = sys.indnom;

% Current system being executed
systag = master_settings.systag;

% Controller initialization controls
init1_load0_lq = master_settings.init1_load0_lq;

% Relative path to controller data
relpath_lq = master_settings.relpath_lq;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% Get design tag
designtag = master_settings.designtag;

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
% LQ SERVO INNER-OUTER DESIGN PARAMETERS -- INITIAL STABILIZING CONTROLLER
%
% NOTE: State partitioned as (see below)
%
%      = [  z
%           y
%           x_r ]
%

% V
Q1 = diag([2 2]);
R1 = 2.5;
% \gamma
Q2 = diag([2.5 5 0.05 0]);
R2 = 1;

% K_0 -- SAME AS DESIGN
Q10 = Q1;
R10 = R1;
Q20 = Q2;
R20 = R2;


% ***********************
%
% LQ SERVO INNER/OUTER CONTROLLER
%   

% Relative path to controller params
relpath_ctrl_tmp = [relpath_lq];

% File name to save to
filename_tmp = 'lq_data_cell.mat';

% Cell array to contain LQ data
lq_data_cell = cell(numnu1,numnu2);

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.Q2 = Q2;
    alg_settings.R2 = R2;
    alg_settings.hasintaug = hasintaug;

    % Loop over model cell array
    for i = 1:numnu1
        for j = 1:numnu2

            % Get current model
            alg_settings.model_d = model_cell{i,j};
     
            % Initialize controller 
            lq_datai = config_lq_servo_tito(alg_settings);
    
            % Store lq_data in array
            lq_data_cell{i,j} = lq_datai;

        end
    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp);

    % Save data 
    varname = 'lq_data_cell';
    save([relpath_ctrl_tmp filename_tmp], varname);

end


% ***********************
%
% LQ SERVO INNER/OUTER CONTROLLER -- APPROXIMATE DECENTRALIZED DESIGN
%   

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.Q2 = Q2;
    alg_settings.R2 = R2;
    alg_settings.hasintaug = hasintaug;

    % Loop over model cell array
    for i = 1:numnu1
        for j = 1:numnu2

            % Get current model
            model_i = model_cell{i,j};
    
            % Modify model parameters for approximate decentralized design
            io = model_i.lin.io;
            io.Ad11 = io.AdTV;
            io.Bd11 = io.BdTV;
            io.Cd11 = io.CdTV;
            io.Dd11 = io.DdTV;
            io.Pd11 = io.PdTV;
            io.Ad22 = io.AdEg;
            io.Bd22 = io.BdEg;
            io.Cd22 = io.CdEg;
            io.Dd22 = io.DdEg;
            io.Pd22 = io.PdEg;
            model_i.lin.io = io;
    
            % Set model
            alg_settings.model_d = model_i;
     
            % Initialize controller 
            lq_datai_d = config_lq_servo_tito(alg_settings);
    
            % Get current lq_data and add on new fields
            lq_datai = lq_data_cell{i,j};
            lq_datai.lq_data_V = lq_datai_d.lq_data_11;
            lq_datai.lq_data_g = lq_datai_d.lq_data_22;
            lq_datai.lq_data_d = lq_datai_d;
    
            % Store lq_data in array
            lq_data_cell{i,j} = lq_datai;

        end
    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp);

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
    alg_settings.Q2 = Q20;
    alg_settings.R2 = R20;
    alg_settings.hasintaug = hasintaug;

    % Get current model -- nominal model
    alg_settings.model_d = get_elt_multidim(model_cell,indnom);

    % Initialize controller 
    lq_data_0 = config_lq_servo_tito(alg_settings);

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
master_settings.pfavec = [0.75; 0.5];
lq_data_nom = get_elt_multidim(lq_data_cell,indnom);


% ***********************
%
% LQ SERVO -- INITIAL STABILIZING CONTROLLER
%   

master_settings.lq_data_0 = lq_data_0;

