function pendulum_init(settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PENDULUM MODEL
%
% [ ***** ANONYMIZED ***** ] 
%
% 2023-03-29
%
% This program initializes model data for a pendulum model. Data is from:
%
%   M. Lutter, et al. "Value Iteration in Continuous Actions, States and
%   Time", ICML. 2023.
%
%   https://github.com/milutter/value_iteration
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
% INITIALIZE VEHICLE PARAMETERS
%
% *************************************************************************
% *************************************************************************


% Pendulum length (m)
L = 1;

% Pendulum mass (kg)
mp = 1;

% Gravitational field constant (m/s^2)
g = 9.81;



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

% x = in R^{n}
xs = sym('x', [1 2], 'real')';

% u in R^{m}
us = sym('u', [1 m], 'real')'; 


% Index variables (i.e., where each state is in x)
indth = 1;
indthd = 2;

% Output variables to track: y = [\theta]^T     
inds_xr = [indth];


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
% VEHICLE PARAMETERS
%

model.m = m;


model.mp = mp;
model.L = L;
model.g = g;


% Index variables (i.e., where each state is in x)
model.indth = indth;
model.indthd = indthd;

% Output variables to track: y     
model.inds_xr = inds_xr;


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model = init_model(model);
trimconds_nom = model.trimconds;

% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT NOMINAL MODEL TRIM
%

lin_nom = linearize_model(model, trimconds_nom);
model.lin = lin_nom;

% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT PERTURBED MODEL TRIM
%

% Initialize empty. Will be filled below
lin_atnu = cell(nummodels,1);


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
inds_notnom = inds_notnom(inds_notnom ~= indnom);

for i = inds_notnom

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************

% Initialize as the default, modify perturbed functions
model_nui = model;

% Perturbations for this model
nui.L = nuvec(i);

% Store pertubation values
model_nui.nu = nui;
model_nui.nuvec = nuvec(i);

% Modify perturbed model params from nominal
model_nui.L = nui.L * L;


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
% LINEARIZATION -- NOMINAL MODEL AT PERTURBED MODEL TRIM
%

lin_nom_atnui = linearize_model(model, trimconds_nui);
lin_atnu{i} = lin_nom_atnui;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT NOMINAL MODEL TRIM
%

lin_nui_atnom = linearize_model(model_nui, trimconds_nom);
model_nui.lin_atnom = lin_nui_atnom;


% ***********************
%
% SET THIS PERTURBED MODEL IN THE ARRAY
%

model_cell{i} = model_nui;

end


% ***********************
%
% SET THE NOMINAL MODEL
%

% Perturbations for this model
nui.d = nuvec(indnom);

% Store pertubation values
model.nu = nui;
model.nuvec = nuvec(indnom);

model.lin_atnu = lin_atnu;
model_cell{indnom} = model;


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
disp(['* PENDULUM MODEL INITIALIZATION COMPLETE'])
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

mp = model.mp;
L = model.L;
g = model.g;


% Index variables (i.e., where each state is in x)
indth = model.indth;
indthd = model.indthd;


% ***********************
%       
% CALCULATE PHYSICAL CONSTANTS
%    

% Pendulum moment of inertia (kg-m^2)
I = 1 / 3 * mp * L^2;


% ***********************
%       
% VARIABLE SHORTHANDS
%    

ths = xs(indth);
thds = xs(indthd);

% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% FIRST DERIVATIVES (WITHOUT CONTROL)
%    

% \dot{\theta} -- f(x)
% Cf. Wang, Eqn. (20)
dth0(xs) = thds;

% \dot{\omega} -- f(x)
dthd0(xs) = 1 / I * m * g * L / 2 * sin(ths);




% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
% 
% *************************************************************************


% ***********************
%       
% f(x)
%  

fxs(xs) =   [   dth0
                dthd0   ];

% ***********************
%       
% g(x)
%  

% g_1(x)
g1x =  [    0
            1 / I   ];
g1s = symfun(g1x,xs);   % Do for constant sym functions only
g1s(xs) = formula(g1s);




% Combine
% gxs(xs) = formula([g1s g2s]);
gxs(xs) = formula(g1s);




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

% Initialize trim parameters
the = 0;            % Equilibrium pendulum angle (rad -- NOTE 0 = up)
thde = 0;           % Equilibrium rotational velocity (rad/s)

% Calculate equilibrium controls u_e = [\bar{e}_{a,e}, \Delta e_{a,e}]
taue = -m*g*L/2*sin(the);


disp('***********************')
disp('*')
disp('* TRIM CALCULATION')
disp('*')
disp('***********************')
disp(' ')

% Trim state x_e
disp(['Trim State x_e = [\theta_e, \dot{\theta}_e]^T = '])
xe = [  the
        thde  ];

% Trim control u_e 
disp(['Trim Control u_e = [\tau_{e}]^T = '])
ue = [  taue    ];


% Evaluate system dynamics
f_xe = fx(xe);
g_xe = gx(xe);
xdot = f_xe + g_xe * ue;

disp(['State Derivatives Evaluated at Trim: ' ...
    '\dot{x} = f(x_e) + g(x_e) * u_e ='])

xdot

disp(['||\dot{x} = f(x_e) + g(x_e) * u_e || ='])

norm(xdot)

% Store equilibrium calculations
trimconds.the = the;               
trimconds.thde = thde;                      
trimconds.xe = xe;
trimconds.ue = ue;



%%
% *************************************************************************
% *************************************************************************
%
% SYMBOLIC LINEARIZATION
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% df_1 / dx
%    

df11 = 0;
df12 = 1;


% ***********************
%       
% df_2 / dx
%   

df21(xs) = 1 / I * m * g * L / 2 * cos(ths);
df22 = 0;

% ***********************
%       
% df / dx
%   

% (V, \omega)
dfdxs = [   df11    df12
            df21    df22    ];
dfdxs = formula(dfdxs);




% ***********************
%       
% "A", "B", "C", "D" MATRIX
%
% NOTE: Is sum of terms from the dynamical equations + load/armature
% equations
%   

% % d f / d x
% dfdxs(xs) = dfdxd4 + Apla;
% dfdxs = formula(dfdxs);


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
% PHYSICAL CONSTANTS
%    

% Pendulum moment of inertia (kg-m^2)
model.I = I;

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
model.g1s = g1s;
% model.g2s = g2s;
model.dg1dxs = dg1dxs;
% model.dg2dxs = dg2dxs;

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
% g1s = model.g1s;
% g2s = model.g2s;
dg1dxs = model.dg1dxs;
% dg2dxs = model.dg2dxs;

% ***********************
%
% VEHICLE TRIM
%

xe = trimconds.xe;
ue = trimconds.ue;


% d {g(x) u} / d x 
dgudx = double(subs(dg1dxs, xs, xe)) * ue(1);

% Outputs: [\theta]
Cp = [ 1    0 ];

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
%       x' =   [    \theta (rad)
%                   \dot{\theta} (rad/s)    ]
%
%            = [    y
%                   x_r            ]
%    
% New Output Variables
%       y' =   [    \theta (rad)     ] 
%          = y
%
% *************************************************************************


% Coordinate transformations
sud = eye(m);
sxd = eye(2);
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
% [ z                   [   x_1
%   y           ->               ]
%   x_r ]
%
%
%       x_1 = x 
%
% *************************************************************************

% % DIRL state transformation: [z, y, x_r] -> [x_1]
% % x_{lq servo} -> x_{dirl}
% Sxirl = eye(3);

% DIRL state transformation: [y, x_r] -> [x_1]
% x_{lq servo} -> x_{dirl}
Sxirl = eye(2);


invSxirl = inv(Sxirl);

% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS (WITH x_3)
%
% [ z                   [   x_1
%   y           ->               
%   x_r                          ]; 
%
%     NOTE: None of this is really applicable for the pendulum. It's a
%     single-loop system, and there are no remaining states not fed back.
%
% *************************************************************************

Sxirl_x3 = Sxirl;

invSxirl_x3 = inv(Sxirl_x3);

% *************************************************************************
%
% FORM PLANT FOR DIRL
%
% State: x = [  x_1 ]
%
%       x_1 = x  
%
% *************************************************************************

% 
% % Augmented plant
% % x = [z^T x_p^T]^T
% Aaug = [    zeros(m)   Cd
%             zeros(n,m) Ad  ];
% Baug = [    zeros(m,m)
%             Bd         ];
% 
% % DIRL state-space matrices
% Airl = Sxirl_x3 * Aaug * inv(Sxirl_x3);
% Birl = Sxirl_x3 * Baug;


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

% % Plant (2,2)
% Pd22 = ss(Ad22,Bd22,Cd22,Dd22);



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

% Plant ss objects
lin.Py = Py;          % Plant y = [V, \omega]^T
lin.Px = Px;            % Plant y = x


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




