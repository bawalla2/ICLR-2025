function vamvoudakis2010_init(settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL (VAMVOUDAKIS 2010)
%
% [ ***** ANONYMIZED ***** ] 
%
% 2023-11-25
%
% This program initializes model data for the system studied in Section 5.2
% of 
%
%   Vamvoudakis, Kyriakos G., and Frank L. Lewis. "Online actor–critic
%   algorithm to solve the continuous-time infinite horizon optimal control
%   problem." Automatica 46.5 (2010): 878-888.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;

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
% *************************************************************************
%
% SETTINGS
%
% *************************************************************************
% *************************************************************************

% Number of inputs
m = 1;

% Perturbation params
nuvec = settings.nuvec;

% Number of perturbed models initialized
nummodels = size(nuvec,1);

% Index of nominal model
indnom = settings.indnom;

% Relative path to save data to
relpath_data = settings.relpath_data;

% *************************************************************************
% *************************************************************************
%
% INITIALIZE SYSTEM PARAMETERS
%
% *************************************************************************
% *************************************************************************



% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE DYNAMIC FUNCTIONS
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% x = [x_1 x_2] in R^{2}
xs = sym('x', [1 2], 'real')';


% u in R^{m}
us = sym('u', [1 m], 'real')'; 



% Output variables to track: y = x_1    
inds_xr = [1];


% ***********************
%       
% MISC
%   

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;


% ***********************
%       
% MODEL CELL ARRAY
%
% Holds model for each of the perturbations tested
%   

model_cell = cell(nummodels, 1);



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL -- DEFAULT PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************


% ***********************
%       
% SYMBOLIC VARIABLES
%      

model.xs = xs;
% model.xvs = xvs;
model.us = us; 
% model.xvus = xvus;
% model.indsz = indsz;
% model.xvs0 = xvs0;

% Degree/radian conversions
model.D2R = D2R;
model.R2D = R2D;



% ***********************
%
% MODEL PARAMETERS
%

model.m = m;


% Output variables to track: y     
model.inds_xr = inds_xr;



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL -- PERTURBED PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Indices not corresponding to the nominal model
inds_notnom = 1:nummodels;

for i = 1:nummodels

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************

% Initialize as the default, modify perturbed functions
model_nui = model;

% Store pertubation values
model_nui.nuvec = nuvec(i);


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model_nui = init_model(model_nui);
trimconds_nui = model_nui.trimconds;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT PERTURBED MODEL TRIM
%

lin_nui = linearize_model(model_nui, trimconds_nui);
model_nui.lin = lin_nui;


% ***********************
%
% SET THIS PERTURBED MODEL IN THE ARRAY
%

model_cell{i} = model_nui;

end



% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************


% Make directory to save data to
mkdir(relpath_data)

% Initialize model struct
% model_struct.model = model;
model_struct.model_cell = model_cell;

% Save data 
varname = 'model_struct';
filename = settings.filename;
save([relpath_data filename], varname);



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

% Display complete
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* MODEL INITIALIZATION COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function model = init_model(model)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
% xvs = model.xvs;
us = model.us; 
% xvus = model.xvus;
% xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% ***********************
%
% VEHICLE PARAMETERS
%

m = model.m;

nuvec = model.nuvec;

% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% "A" MATRIX TERMS ENTERING DIRECTLY IN THE NONLINEAR EQUATIONS
%
%    



Af = [  -1  1
        -0.5   0     ];




% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
% 
% *************************************************************************


% ***********************
%       
% f(x)
%  

% Trim state
xe = zeros(2,1);

fxs(xs) =   Af * (xs - xe) + [0; nuvec(1) * -0.5 * xs(2) * (1 - (cos(2*xs(1)) + 2)^2)];

% ***********************
%       
% g(x)
%  

% g_1(x)
g1s(xs) =  [    0
            cos(2*xs(1)) + 2    ];
%g1s = symfun(g1x,xs);   % Do for constant sym functions only
g1s(xs) = formula(g1s);


% Combine
gxs(xs) = formula([g1s]);




%%
% *************************************************************************
% *************************************************************************
%
% CREATE INLINE FUNCTIONS
% 
% *************************************************************************
% *************************************************************************


% f(x) -
fx = matlabFunction(fxs, 'vars', {xs});
sys.model.fx = fx;

% g(x) 
gx = matlabFunction(gxs, 'vars', {xs});
sys.model.gx = gx;



%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************


% Calculate equilibrium controls u_e 
u1e = 0;


% Store equilibrium calculations
trimconds.x1e = 0;               
trimconds.x2e = 0;                      
trimconds.xe = xe;
trimconds.ue = [u1e];



%%
% *************************************************************************
% *************************************************************************
%
% SYMBOLIC LINEARIZATION
% 
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% PARTIALS PERTAINING TO VEHICLE DYNAMICAL EQUATIONS
% 
% *************************************************************************

% ***********************
%       
% df / dx
%   

dfdxs(xs) = symfun(Af,xs) + ...
    [   0   0
        nuvec(1) * -0.5*xs(2)*(-2*(cos(2*xs(1))+2)*2*(-sin(2*xs(1)))) nuvec(1) * -0.5*(1-(cos(2*xs(1))+2)^2)];

% ***********************
%       
% "A", "B", "C", "D" MATRIX
%
% NOTE: Is sum of terms from the dynamical equations + load/armature
% equations
%   


% d g_j(x) / d x 
dg1dxs = formula(jacobian(g1s));







%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************


% ***********************
%
% SYSTEM DYNAMICAL EQUATIONS
%


% f(x), g(x)
model.fxs = fxs;
model.gxs = gxs;


% ***********************
%
% INLINE FUNCTIONS
%

% f(x), g(x)
model.fx = fx;
model.gx = gx;

% ***********************
%
% VEHICLE TRIM
%

model.trimconds = trimconds;


% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

model.dfdxs = dfdxs;
% model.g1s = g1s;
model.dg1dxs = dg1dxs;

%%
% *************************************************************************
% *************************************************************************
%
% LINEARIZE SYSTEM
% 
% *************************************************************************
% *************************************************************************

function lin = linearize_model(model, trimconds)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% VEHICLE PARAMETERS
%    

% Number of inputs
m = model.m;

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
% xvs = model.xvs;
% us = model.us; 
% xvus = model.xvus;
% indsz = model.indsz;
% xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% ***********************
%
% INLINE FUNCTIONS
%

% f(x), g(x)
% fx = model.fx;
gx = model.gx;

% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

dfdxs = model.dfdxs;
dg1dxs = model.dg1dxs;

% ***********************
%
% VEHICLE TRIM
%

xe = trimconds.xe;
ue = trimconds.ue;


% d {g(x) u} / d x 
dgudx = double(subs(dg1dxs, xs, xe)) * ue(1);

% Outputs
Cp = [1 0];

% "D" matrix
Dp = zeros(m);

% Evaluate numerically at trim
Ap = double(subs(dfdxs, xs, xe)) + dgudx;
Bp = gx(xe);

% Number of states 
n = size(Ap,1);

% Plant 
Py = ss(Ap,Bp,Cp,Dp);

% Plant y = x
Px = ss(Ap,Bp,eye(n),zeros(n,m));


%%
% *************************************************************************
% *************************************************************************
%
% FORM DESIGN PLANT FOR PI-PD INNER-OUTER DESIGN
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% FORM DESIGN PLANT
%
% Given an original plant of the form P = (A, B, C, D), we would like to
% form a design plant (appropriately scaled) such that the state vector
% assumes the form
%
% x' = [    y
%           x_r     ]
%
% Where y \in R^m are the outputs to be tracked, and x_r \in R^{n-m} forms
% the rest of the state.
%
% Introduce the coordinate transformations:
%
%  u' = S_u * u
%  x' = S_x * x
%  y' = S_y * y
%
%
% New Control Variables
%     u' = u        (unchanged)
%
%
% New State Variables
%       x' =  x     (unchanged)
%
%            = [    y
%                   x_r             ]
%    
% New Output Variables
%       y' = y      (unchanged)
%
% *************************************************************************


% Coordinate transformations
sud = eye(m);
sxd = eye(n);
syd = eye(m);

% Scaled linear dynamics
Ad = sxd*Ap*inv(sxd);
Bd = sxd*Bp*inv(sud);
Cd = syd*Cp*inv(sxd);
Dd = syd*Dp*inv(sud);

% Scaled design plant
Pd = ss(Ad,Bd,Cd,Dd);


% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS
%
% [                   [   x_1
%   y           ->          x_2     ]
%   x_r ]      
%
% *************************************************************************

% DIRL state transformation: [y, x_r] -> [x_1 x_2]
% x_{lq servo} -> x_{dirl}
Sxirl = eye(2);


invSxirl = inv(Sxirl);

% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS (WITH x_3)
%
% [ z                   [   x_1
%   y           ->          x_2     
%   x_r                     x_3     ];
%   x_3     ]
%
%       NOTE: x_3 absent    
%
% *************************************************************************

Sxirl_x3 = Sxirl;

invSxirl_x3 = inv(Sxirl_x3);

% *************************************************************************
%
% FORM PLANT FOR DIRL
%
% State: x = [  x_1
%               x_2     ]
%
% *************************************************************************


% DIRL state-space matrices
Airl = Sxirl_x3 * Ad * inv(Sxirl_x3);
Birl = Sxirl_x3 * Bd;



% *************************************************************************
%
% FORM SIMPLIFIED DECOUPLED DESIGN PLANT FOR INNER-OUTER LOOP DESIGN
%
% *************************************************************************


% (1,1) Linearization
Ad11 = Ad;
Bd11 = Bd;
Cd11 = Cd;
Dd11 = Dd;

% Plant (1,1)
Pd11 = ss(Ad11,Bd11,Cd11,Dd11);



%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************

% ***********************
%
% LINEARIZATION TERMS
%

% A, B, C, D matrix
lin.Ap = Ap;
lin.Bp = Bp;
lin.Cp = Cp;
lin.Dp = Dp;

% ***********************
%
% LINEARIZATION TERMS -- DESIGN PLANT FOR PD-PI INNER-OUTER
%

io.sud = sud;
io.sxd = sxd;
io.syd = syd;

io.Ad = Ad;
io.Bd = Bd;
io.Cd = Cd;
io.Dd = Dd;
io.Pd = Pd;
io.nlq = size(Ad,1);        % System order for LQ servo

% IRL linear dynamics, coord transformations
io.Airl = Airl;
io.Birl = Birl;
io.Sxirl = Sxirl;
io.invSxirl = invSxirl;
io.Sxirl_x3 = Sxirl_x3;
io.invSxirl_x3 = invSxirl_x3;


% Plant (1,1)
io.Ad11 = Ad11;
io.Bd11 = Bd11;
io.Cd11 = Cd11;
io.Dd11 = Dd11;
io.Pd11 = Pd11;

% Store all inner/outer terms
lin.io = io;

% Store linearization params
model.lin = lin;




