function lq_data = config_lqr(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% LQR SINGLE-LOOP DESIGN
%
% [ ***** ANONYMIZED ***** ] 
%
% 2023-03-29
%
% This program performs a single-loop LQR design.
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
% *************************************************************************
%
% LOAD NONLINEAR MODEL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% System tag
sys = alg_settings.sys;


% *************************************************************************
% 
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
% model = sys.model;                  % System model

% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end

% Get Q, R
Q1 = alg_settings.Q1;
R1 = alg_settings.R1;
% Q2 = alg_settings.Q2;
% R2 = alg_settings.R2;

% Whether or not to make prefilter
makepf = isfield(alg_settings, 'makepf');
if makepf
    makepf = alg_settings.makepf;
else
    makepf = 1;
end

% Specify index of \dot{y} in K_r for making prefilter
hasindydot = isfield(alg_settings, 'indydot');
if hasindydot
    indydot = alg_settings.indydot;
else
    indydot = 1;
end

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Design model
model_d = alg_settings.model_d;


% ***********************
%
% LINEARIZATION TERMS
%

% Linearization params
lin = model_d.lin;
io = lin.io;

% Scaled linear dynamics
Ad = io.Ad;
Bd = io.Bd;
Cd = io.Cd;
Dd = io.Dd;

% DIRL state transformation 
% x_{lq servo} -> x_{dirl}
% [z y x_{r}] -> [x_1 x_2]
Sxirl = io.Sxirl;



%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- OVERALL SYSTEM
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = eye(size(Bd,2));
alg_settings.sx = eye(size(Ad,1));
alg_settings.sy = eye(size(Cd,1));

% Plant state-space
alg_settings.Ap = Ad;
alg_settings.Bp = Bd;
alg_settings.Cp = Cd;
alg_settings.Dp = Dd;

% Q, R
alg_settings.Q = Q1;
alg_settings.R = R1;


% Perform design
lq_data_c = lqr_init(alg_settings);

% Extract controller
Kc = lq_data_c.K;


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX
%
% *************************************************************************
% *************************************************************************

% Total controller
K = Kc;

% Form composite controller
Ky = lq_data_c.Ky;
Kr = lq_data_c.Kr;


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX -- IRL STATE PARTITION
%
% *************************************************************************
% *************************************************************************

% EQUIVALENT CONTROLLER -- COMPOSITE SYSTEM
Kcirl = Kc * inv(Sxirl);

% EQUIVALENT CONTROLLER -- DECENTRALIZED DESIGN
Kdirl = Kcirl;

% EQUIVALENT "Q", "R" MATRICES
% Note: Wrt the DIRL state partition. Need transformation to the LQ servo
% state partition
Qirl = Q1;

% % Q -- in LQ servo coords
% Q = Sxirl' * Qirl * Sxirl;


%%
% *************************************************************************
% *************************************************************************
%
% PREFILTER POLE LOCATIONS
%
% A prefilter is inserted before the reference command in each
% channel, of form
%
%   W_i(s) =    a_i
%             --------,         i = 1,...,m
%               s + a_i
%
% *************************************************************************
% *************************************************************************

% Calculate zero locations automatically
if makepf 

    g1 = Kr(indydot);
    z1 = Ky / g1;
    pfavec = [z1];
    
    
    % Make pre-filter
    Aw = -z1;
    Bw = z1;
    Cw = 1;
    Dw = 0;
    W = ss(Aw,Bw,Cw,Dw);

end



%%
% *************************************************************************
% *************************************************************************
%
% PACK OUTPUT DATA
%
% *************************************************************************
% *************************************************************************

% Individual loop data
% lq_data.lq_data_11 = lq_data_11;
% lq_data.lq_data_22 = lq_data_22;
lq_data.lq_data_tot = lq_data_c;

% Composite Q, R
lq_data.Q = Q1;
lq_data.R = R1;

% Composite controller
lq_data.K = K;
% lq_data.Kz = lq_data_c.Kz;
lq_data.Ky = lq_data_c.Ky;
lq_data.Kr = lq_data_c.Kr;

% Composite data -- IRL state partition
lq_data.Qirl = Qirl;
lq_data.Pirl = lq_data_c.P;
lq_data.Kcirl = Kcirl;
lq_data.Kdirl = Kdirl;

% Prefilter data
if makepf
    lq_data.pfavec = pfavec;
    lq_data.W = W;
end


%%
% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************

if savedata

    % Make directory to save data to
    mkdir(relpath_data)

    % Save data 
    varname = 'lq_data';
    save([relpath_data filename], varname);

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************
