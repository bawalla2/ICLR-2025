function hsv_wang_stengel_2000_init(settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV PITCH-AXIS MODEL
%
% Brent A. Wallace 
%
% 2022-09-28
%
% This program initializes aerodynamic and other model data for a
% pitch-axis HSV model. Data is from:
%
%   Q. Wang and R. F. Stengel. "Robust nonlinear control of a hypersonic
%   aircraft." AIAA J. Guid., Contr., & Dyn., 23(4):577–585, July 2000
%
% And:
%
%   C. I. Marrison and R. F. Stengel. "Design of robust control systems for
%   a hypersonic aircraft." AIAA J. Guid., Contr., & Dyn., 21(1):58–63,
%   Jan. 1998.
%
% Which get their aerodynamic data from
%
% J. D. Shaughnessy, S. Z. Pinckney, J. D. McMinn, C. I. Cruz, and M.-L.
% Kelley. "Hypersonic vehicle simulation model: Winged-cone configuration."
% NASA TM-102610, Nov. 1990.
%
% NOTE: Make sure the MATLAB path is in the folder holding main.m before
% running this program.
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
m = 2;

% Tag of which parameter to perturb
nutag = settings.nutag;

% Perturbation params
nu1vec = settings.nu1vec;
nu2vec = settings.nu2vec;

% Number of perturbed models initialized
numnu1 = size(nu1vec,1);
numnu2 = size(nu2vec,1);
nummodels = numnu1*numnu2;

% Index of nominal model
indnom = settings.indnom;

% Relative path to save data to
relpath_data = settings.relpath_data;

% DEBUGGING: Evaluate aero functions at trim (=1) or not (=0)
evaltrim = 0;


% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE PARAMETERS
%
% *************************************************************************
% *************************************************************************

% Gravitational field constant (ft/s^2)
g = 32.1741;

% Gravitational constant (ft^3/s^2)
% Cf. Marrison, Nomenclature
mu = 1.39e16;

% Radius of earth (ft)
% Cf. Marrison, Nomenclature
RE = 20903500;

% Vehicle mass (slug)
% Cf. Marrison, Nomenclature
mref = 9375;

% Vehicle weight (lb)
wref = mref * g;

% Pitch-axis moment of inertia (slug-ft^2)
% Cf. Marrison, Nomenclature
Iyy = 7e6;

% Reference area (ft^2)
% Cf. Marrison, Nomenclature
S = 3603;

% MAC (ft)
% Cf. Marrison, Nomenclature
cbar = 80;

% Actuator model parameters

% zeta = 0.9;
% omega_n = 10;
zeta = 0.7;
omega_n = 20;

k1 = - 2 * zeta * omega_n;
k2 = - omega_n^2;
k3 = omega_n^2;        




% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE AERODYNAMIC COEFFICIENT FUNCTIONS
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% x = [V, \gamma, h, \alpha, q] in R^{5}
xs = sym('x', [1 5], 'real')';

% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xvs = sym('xv', [1 7], 'real')';

% u in R^{m}
us = sym('u', [1 2], 'real')'; 

% \nu = [\nu_{C_L}, \nu_{C_D}, \nu_{C_{M_\alpha}}]
n_nu = 3;
nus = sym('nu', [1 n_nu], 'real')'; 


% [x, \nu] in R^{5+n_\nu}
xnus = [    xs
            nus  ];

% [x_v, u, \nu] in R^{7+m+n_\nu}
xvunus = [  xvs
            us
            nus  ];

% Index variables (i.e., where each state is in x)
indV = 1;
indg = 2;
indh = 3;
inda = 4;
indq = 5;
inddT = 6;
indddT = 7;
% indVI = 8;
% indhI = 9;

% Index variables (i.e., where each control is in [xv u \nu])
inddTc = 8;
inddE = 9;

% Index variables (i.e., where each \nu is in [xv u \nu])
indnustart = inddE;
nunomvec = ones(n_nu,1);
indnuCL = indnustart + 1;
indnuCD = indnustart + 2;
indnuCMa = indnustart + 3;

% Output variables to track: y = [V, \omega]^T     
inds_xr = [indV; indg];

% Ordered indices corresponding to
% z = [V, \gamma, \alpha, \delta_{T}, h] in R^{5}
% Cf. Wang, Eqn. (39)
indsz = [   indV
            indg
            inda
            inddT
            indh    ];

% Total vehicle state (i.e., with actuator dynamics), with zeros plugged
% into the actuator dynamics terms
xvs0(xs) = [    xs
                zeros(2,1) ];

xvs00nu(xnus) = [  xvs0 
                  zeros(m,1)
                  nus   ];

% ***********************
%       
% MISC
%   

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% sin(\alpha), cos(\alpha)
sa(xvunus) = sin(xvunus(inda));
ca(xvunus) = cos(xvunus(inda));

% sin(\gamma), cos(\gamma)
sg(xvunus) = sin(xvunus(indg));
cg(xvunus) = cos(xvunus(indg));

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg(xvunus) = sin(xvunus(inda) + xvunus(indg));
capg(xvunus) = cos(xvunus(inda) + xvunus(indg));


% *************************************************************************
%
% AERO FUNCTIONS
% 
% *************************************************************************

% ***********************
%       
% TRIM CONDITIONS
%   

% Trim AOA (rad)
alpha_er = 0.0315;

% DEBUGGING: TEST TRIM
% Trim conditions
% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xve = [15060 ; 0 ; 110e3 ; alpha_er ; 0 ; 0.183 ; 0];
ue_ws = [0.183 ; -0.0066];

xvuenunom = [   xve
                ue_ws
                nunomvec ];
    
% ***********************
%       
% TOTAL DISTANCE FROM EARTH CENTER TO VEHICLE
%  

% Cf. Wang, Eqn. (30)
r(xvunus) = RE + xvunus(indh);
% DEBUGGING: Evaluate at trim
if evaltrim
    re = double(subs(r,xvunus,xvuenunom))
end

% ***********************
%       
% AIR DENSITY \rho
%  

% Cf. Wang, Eqn. (A7)
rho(xvunus) = 0.00238 * exp(-xvunus(indh)/24000);
% DEBUGGING: Evaluate at trim
if evaltrim
    rhoe = double(subs(rho,xvunus,xvuenunom))
end

% ***********************
%       
% SPEED OF SOUND a
%  

% Cf. Wang, Eqn. (A5)
a(xvunus) = 8.99e-9*xvunus(indh)^2 - 9.16e-4*xvunus(indh) + 996;
% DEBUGGING: Evaluate at trim
if evaltrim
    ae = double(subs(a,xvunus,xvuenunom))
end

% Inline function
axnus(xnus) = subs(a,xvunus,xvs00nu);
axnuil = matlabFunction(axnus, 'vars', {xnus});

% ***********************
%       
% MACH NUMBER M
%  

% Cf. Wang, Eqn. (A6)
M(xvunus) = xvunus(indV) / a;
% DEBUGGING: Evaluate at trim
if evaltrim
    Me = double(subs(M,xvunus,xvuenunom))
end


% ***********************
%       
% LIFT COEFFICIENT C_L
%  

% Cf. Wang, Eqn. (A8)
CL(xvunus) = xvunus(indnuCL) * xvunus(inda) * (0.493 + 1.91 / M);
% DEBUGGING: Evaluate at trim
if evaltrim
    CLe = double(subs(CL,xvunus,xvuenunom))
end


% ***********************
%       
% LIFT COEFFICIENT C_L -- WITH NONMINIMUM PHASE BEHAVIOR ADDED
%
%   C_L = C_L(\alpha) + C_L(\delta_E)
%
% Where C_L(\alpha) is the main term defined above, and C_L(\delta_E) is
% the elevator-induced lift coefficient (not modeled by Wang and Stengel,
% added in here). New term fit over Shaughnessy data as
%
%   C_L(\delta_E) = (c1 \alpha^2 + c2 \alpha + c3) * \delta_E
%
% See fit_CL_de.m.
%  

% % Constants -- fit over \delta_E = {-10deg,10deg}
% CLdE_c1 = -0.449769705479619;
% CLdE_c2 = -0.0138761375623472;
% CLdE_c3 = -0.0161431704966951;

% Constants -- fit over \delta_E = all points
CLdE_c1 = -0.235580737050513;
CLdE_c2 = -0.00451812931308866;
CLdE_c3 = -0.0291335007132150;

% C_L(\delta_E)
CLdEu(xvunus) = (CLdE_c1*xvunus(inda)^2 + CLdE_c2*xvunus(inda) + CLdE_c3);
CLdE(xvunus) = CLdEu * xvunus(inddE);

% DEBUGGING: Evaluate at trim
if evaltrim
    CLdEue = double(subs(CLdEu,xvunus,xvuenunom))
end

% ***********************
%       
% DRAG COEFFICIENT C_D
%  

% Cf. Wang, Eqn. (A9)
CD(xvunus) = xvunus(indnuCD) * ...
            0.0082 * (171*xvunus(inda)^2 + 1.15*xvunus(inda) + 1) * ...
            (0.0012*M^2 - 0.054*M + 1);

% DEBUGGING: Evaluate at trim
if evaltrim
    CDe = double(subs(CD,xvunus,xvuenunom))
end



% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- MAIN TERM C_M(\alpha)
%  

% Cf. Wang, Eqn. (A11)
CMalpha(xvunus) = xvunus(indnuCMa) *...
    1e-4 * (0.06 - exp(-M/3)) ...
    * (-6565 * xvunus(inda)^2 + 6875*xvunus(inda) + 1);

% % Marrison, Eqn. (24) version (expressed in rad)
% % Note the "-1" difference
% CMalpha(xvunus) = 1e-4 * (0.06 - exp(-M/3)) ...
%     * (-6565 * xvunus(inda)^2 + 6875*xvunus(inda) - 1);

% % Marrison, Eqn. (24) version (as originally expressed)
% CMalpha(xvunus) = 1e-4 * (0.06 - exp(-M/3)) ...
%     * (-2 * xvunus(inda)^2 + 120*xvunus(inda) - 1);
% DEBUGGING: Evaluate at trim
if evaltrim
    CMalphae = double(subs(CMalpha,xvunus,xvuenunom))
end


% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- ROTATIONAL DAMPING C_M(q)
%  

% Cf. Wang, Eqn. (A12)
CMq(xvunus) = cbar / (2*xvunus(indV)) * xvunus(indq) * (-0.025*M + 1.37) * ...
    (-6.83*xvunus(inda)^2 + 0.303*xvunus(inda) - 0.23);
% DEBUGGING: Evaluate at trim
if evaltrim
    CMqe = double(subs(CMq,xvunus,xvuenunom))
end


% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- ELEVATOR-INDUCED PITCHING COEFFICIENT
% C_M(\delta_{E})
%  

% Cf. Wang, Eqn. (A13)
c_CMdE = 0.0292;
% CMdEu(xvunus) = c_CMdE;
CMdEu = c_CMdE;
CMdE0(xvunus) = c_CMdE * ( - xvunus(inda));
CMdE(xvunus) = CMdE0 + CMdEu * xvunus(inddE);
% DEBUGGING: Evaluate at trim
if evaltrim
    CMdE0e = double(subs(CMdE0,xvunus,xvuenunom))
end

% ***********************
%       
% THRUST COEFFICIENT -- TERM k(x)
%  

% Do C_T AOA effects (=1) or not (=0)
doCTAOA = 0;

% Cf. Wang, Eqn. (A10)
c_k = 0.0105;
% c_k = 38 / S;
if doCTAOA 
    kxv(xvunus) = c_k * (1 - 164 * (xvunus(inda) - alpha_er)^2) * (1 + 17/M);
else
    kxv(xvunus) = c_k * (1 + 17/M);
end
% DEBUGGING: Evaluate at trim
if evaltrim
    ke = double(subs(kxv,xvunus,xvuenunom))
end

% ***********************
%       
% THRUST COEFFICIENT C_T
%  

% Cf. Wang, Eqn. (A10)
% NOTE: Used the case \delta_{T} < 1 only
CTu(xvunus) = kxv * (1 + 0.15);
CT(xvunus) = CTu * xvunus(inddT);
% DEBUGGING: Evaluate at trim
if evaltrim
    CTe = double(subs(CT,xvunus,xvuenunom))
end


% ***********************
%       
% DYNAMIC PRESSURE q_{\infty}
%  

qinf(xvunus) = 0.5 * rho * xvunus(indV)^2;
% DEBUGGING: Evaluate at trim
if evaltrim
    qinfe = double(subs(qinf,xvunus,xvuenunom))
end


% ***********************
%       
% THRUST T
%  

% Cf. Wang, Eqn. (27)
Tu(xvunus) = qinf * S * CTu;
T(xvunus) = qinf * S * CT;
% DEBUGGING: Evaluate at trim
if evaltrim
    Te = double(subs(T,xvunus,xvuenunom))
end


% ***********************
%       
% LIFT L
%  

% Cf. Wang, Eqn. (25)
L(xvunus) = qinf * S * CL;
% DEBUGGING: Evaluate at trim
if evaltrim
    Le = double(subs(L,xvunus,xvuenunom))
end




% ***********************
%       
% LIFT L -- WITH ELEVATOR TERMS ADDED
%  

LdE0(xvunus) = L;
LdEu(xvunus) = qinf * S * CLdEu;
LdE(xvunus) = formula(LdE0) + formula(LdEu) * xvunus(inddE);

% DEBUGGING: Evaluate at trim
if evaltrim
    LdE0e = double(subs(LdE0,xvunus,xvuenunom))
    % DEBUGGING: Evaluate at trim
    LdEue = double(subs(LdEu,xvunus,xvuenunom))
    % DEBUGGING: Evaluate at trim
    LdEe = double(subs(LdE0,xvunus,xvuenunom)) + double(subs(LdEu,xvunus,xvuenunom))*ue_ws(2)
    % LdEe = double(subs(L,xvunus,[xvuenunom;ue]))
end


% ***********************
%       
% DRAG D
%  

% Cf. Wang, Eqn. (26)
D(xvunus) = qinf * S * CD;
% DEBUGGING: Evaluate at trim
if evaltrim
    De = double(subs(D,xvunus,xvuenunom))
end


% ***********************
%       
% PITCHING MOMENT M_{yy}
%  

% Cf. Wang, Eqn. (29)
Myy0(xvunus) = qinf * S * cbar * (CMalpha + CMdE0 + CMq);
Myyu(xvunus) = qinf * S * cbar * CMdEu;
Myy(xvunus) = formula(Myy0) + formula(Myyu) * xvunus(inddE);
% DEBUGGING: Evaluate at trim
if evaltrim
    Myy0e = double(subs(Myy0,xvunus,xvuenunom))
    % DEBUGGING: Evaluate at trim
    Myyue = double(subs(Myyu,xvunus,xvuenunom))
    % DEBUGGING: Evaluate at trim
    Myye = double(subs(Myy0,xvunus,xvuenunom)) + double(subs(Myyu,xvunus,xvuenunom))*ue_ws(2)
    % Myye = double(subs(Myy,xvunus,[xvuenunom;ue]))
end



% ***********************
%       
% MODEL CELL ARRAY
%
% Holds model for each of the perturbations tested
%   

model_cell = cell(numnu1, numnu2);


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL -- DEFAULT PARAMETERS
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
model.xvs = xvs;
model.us = us; 
model.nus = nus; 
model.xnus = xnus;
model.xvunus = xvunus;
model.indsz = indsz;
model.xvs0 = xvs0;
model.xvs00nu = xvs00nu;

% Degree/radian conversions
model.D2R = D2R;
model.R2D = R2D;

% sin(\alpha), cos(\alpha)
model.sa = sa;
model.ca = ca;

% sin(\gamma), cos(\gamma)
model.sg = sg;
model.cg = cg;

% sin(\alpha+\gamma), cos(\alpha+\gamma)
model.sapg = sapg;
model.capg = capg;



% ***********************
%
% VEHICLE PARAMETERS
%

model.m = m;

model.g = g;
model.mu = mu;
model.RE = RE;
model.wref = wref;
model.mref = mref;
model.Iyy = Iyy;


model.S = S;
model.cbar = cbar;
% model.b = b;

model.k1 = k1;
model.k2 = k2;
model.k3 = k3;

% Index variables (i.e., where each state is in x)
model.indV = indV;
model.indg = indg;
model.indh = indh;
model.inda = inda;
model.indq = indq;
model.inddT = inddT;
model.indddT = indddT;
% model.indVI = indVI;
% model.indhI = indhI;

% Output variables to track: y     
model.inds_xr = inds_xr;

% Index variables (i.e., where each control is in [xv u \nu])
model.inddTc = inddTc;
model.inddE = inddE;

% Index variables (i.e., where each \nu is in [xv u \nu])
model.indnuCL = indnuCL;
model.indnuCD = indnuCD;
model.indnuCMa = indnuCMa;


% ***********************
%
% VEHICLE AERODYNAMIC FUNCTIONS
%

% Initial trim estimate
model.xve = xve;
model.ue_ws = ue_ws;

model.r = r;
model.rho = rho;
model.a = a;
model.axnuil = axnuil;
model.M = M;

model.CL = CL;
model.CD = CD;

model.c_CMdE = c_CMdE;
model.CMdEu = CMdEu;
model.CMdE0 = CMdE0;
model.CMdE = CMdE;

model.kxv = kxv;
model.CT = CT;

model.qinf = qinf;

model.T = T;
model.Tu = Tu;

model.L = L;
model.D = D;

model.Myy0 = Myy0;
model.Myyu = Myyu;
model.Myy = Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
model.LdE0 = LdE0;
model.LdEu = LdEu;
model.LdE = LdE;


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model = init_model(model);


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL -- PERTURBED PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Uniform distribution bound
unifbd = 0.1;

% Model counter
mcnt = 1;

% If random \nu values are used, set the RNG seed
if strcmp(nutag, 'rand')
    rng(0);
end

for i = 1:numnu1

% Get current value of \nu_1
nu1 = nu1vec(i);


for j = 1:numnu2

% Display initialization
disp(['***** INITIALIZING MODEL ' num2str(mcnt) ...
    '  OUT OF  ' num2str(nummodels)])

% Get current value of \nu_2
nu2 = nu2vec(j);

% Check if this is the nominal model or not
isnom = (nu1 == 1) && (nu2 == 1);

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************

% Initialize as the default, modify perturbed functions
if isnom
    model_nuij = model;
    indnom = [i; j];
else
    clear model_nuij;
end

% Perturbations for this model
nuij.CL = nu1;
nuij.CD = 1;
nuij.CMa = 1;

% Store pertubation values
model_nuij.nu = nuij;
nuvecij = [nuij.CL; nuij.CD; nuij.CMa];
model_nuij.nuvec = nuvecij;



% ***********************
%
% GET TRIM
%

model_nuij = trim_model(model, model_nuij);
trimconds_nui = model_nuij.trimconds;

% ***********************
%
% LINEARIZATION
%

lin_nui = linearize_model(model, trimconds_nui, nuvecij, settings);
model_nuij.lin = lin_nui;


% ***********************
%
% SET THIS MODEL IN THE ARRAY
%

model_cell{i,j} = model_nuij;

% Increment model count
mcnt = mcnt + 1;

end

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
disp(['* HSV MODEL INITIALIZATION COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL FROM AERO DATA
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
xvunus = model.xvunus;
xnus = model.xnus;
us = model.us; 
xvunus = model.xvunus;
indsz = model.indsz;
xvs0 = model.xvs0;
xvs00nu = model.xvs00nu;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% sin(\alpha), cos(\alpha)
sa = model.sa;
ca = model.ca;

% sin(\gamma), cos(\gamma)
sg = model.sg;
cg = model.cg;

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg = model.sapg;
capg = model.capg;



% ***********************
%
% VEHICLE PARAMETERS
%

% Initial trim estimate
xve = model.xve;
ue_ws = model.ue_ws;

m = model.m;

g = model.g;
mu = model.mu;
RE = model.RE;
wref = model.wref;
mref = model.mref;
Iyy = model.Iyy;


S = model.S;
cbar = model.cbar;
% b = model.b;

k1 = model.k1;
k2 = model.k2;
k3 = model.k3;

% Index variables (i.e., where each state is in x)
indV = model.indV;
indg = model.indg;
indh = model.indh;
inda = model.inda;
indq = model.indq;
inddT = model.inddT;
indddT = model.indddT;
% indVI = model.indVI;
% indhI = model.indhI;

% Index variables (i.e., where each control is in [xv u])
inddTc = model.inddTc;
inddE = model.inddE;


% ***********************
%
% VEHICLE AERODYNAMIC FUNCTIONS
%

r = model.r;
rho = model.rho;
a = model.a;
M = model.M;

CL = model.CL;
CD = model.CD;

c_CMdE = model.c_CMdE;
CMdEu = model.CMdEu;
CMdE0 = model.CMdE0;
CMdE = model.CMdE;

kxv = model.kxv;
CT = model.CT;

qinf = model.qinf;

T = model.T;
Tu = model.Tu;

L = model.L;
D = model.D;

Myy0 = model.Myy0;
Myyu = model.Myyu;
Myy = model.Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
LdE0 = model.LdE0;
LdEu = model.LdEu;
LdE = model.LdE;

% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% FIRST DERIVATIVES
%    

% \dot{V}
% Cf. Wang, Eqn. (20)
dVu(xvunus) = Tu*ca /mref;
dV(xvunus) = (T*ca - D)/mref - mu*sg/r^2;

% \dot{\gamma}
% Cf. Wang, Eqn. (21)
dgu(xvunus) = Tu*sa / (mref*xvunus(indV));
dg(xvunus) = (L + T*sa) / (mref*xvunus(indV)) ...
    - ((mu-xvunus(indV)^2*r)*cg) / (xvunus(indV)*r^2);

% \dot{\gamma} -- ELEVATOR TERM \delta_{E}
dgudE(xvunus) = LdEu / (mref*xvunus(indV));

% \dot{h}
% Cf. Wang, Eqn. (22)
dh(xvunus) = xvunus(indV)*sg;

% \dot{\alpha}
% Cf. Wang, Eqn. (23)
dau(xvunus) = - dgu;
da(xvunus) = xvunus(indq) - dg;

% \dot{q}
% Cf. Wang, Eqn. (24)
dqu(xvunus) = Myyu / Iyy;
dq0(xvunus) = Myy0 / Iyy;
% dq(xvunus) = Myy / Iyy;

% \ddot{\delta_{T}}
% Cf. Wang, Eqn. (35)
dddT0(xvunus) = k1 * xvunus(indddT) + k2 * xvunus(inddT);
% dddTc(xvunus) = k3;
dddTc = k3;
% dddT(xvunus) = dddT0 + dddTc * xvunus(inddTc);

% \ddot{\alpha}
% Cf. Wang, Eqns. (23), (43), (45)
% dda0(xvunus) = Myy0 / Iyy - ddg;
ddadE(xvunus) = Myyu / Iyy;
% dda(xvunus) = dq - ddg;


% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- NO ACTUATOR DYNAMICS
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
%
% Throttle setting \delta_{T} is now a control, no longer a state. The
% state vector is
%
% x = [V, \gamma, h, \alpha, q] in R^{5}
% 
% *************************************************************************
% *************************************************************************


% \dot{V} -- f(x)
% Cf. Wang, Eqn. (20)
dV0x(xnus) = subs(dV,xvunus,xvs00nu);

% \dot{V} -- g(x)
% Cf. Wang, Eqn. (20)
dVux(xnus) = subs(dVu,xvunus,xvs00nu);

% \dot{\gamma} -- f(x)
% Cf. Wang, Eqn. (21)
dg0x(xnus) = subs(dg,xvunus,xvs00nu);

% \dot{\gamma} -- g(x)
% Cf. Wang, Eqn. (21)
dgux(xnus) = subs(dgu,xvunus,xvs00nu);

% \dot{\gamma} -- g(x) -- ELEVATOR TERM \delta_{E}
% Cf. Wang, Eqn. (21)
dgudEx(xnus) = subs(dgudE,xvunus,xvs00nu);

% \dot{h}
% Cf. Wang, Eqn. (22)
dhx(xnus) = subs(dh,xvunus,xvs00nu);

% \dot{\alpha} -- f(x)
% Cf. Wang, Eqn. (23)
da0x(xnus) = subs(da,xvunus,xvs00nu);

% \dot{\alpha} -- g(x)
% Cf. Wang, Eqn. (23)
daux(xnus) = subs(dau,xvunus,xvs00nu);

% \dot{q} -- f(x)
% Cf. Wang, Eqn. (24)
dq0x(xnus) = subs(dq0,xvunus,xvs00nu);

% \dot{q} -- g(x)
% Cf. Wang, Eqn. (24)
dqux(xnus) = subs(dqu,xvunus,xvs00nu);




% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
% 
% *************************************************************************


% f(x) -- with actuator dynamics
fxvs(xvunus) = [   dV
                dg
                dh
                da
                dq0
                xvunus(indddT)
                dddT0       ];

% g(x) -- with actuator dynamics
gxvs(xvunus) = [   0       0
                0       0
                0       0
                0       0
                0       ddadE
                0       0
                dddTc   0       ];

% g(x) -- with actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxvdEs(xvunus) = [ 0       0
                0       dgudE
                0       0
                0       -dgudE
                0       ddadE
                0       0
                dddTc   0       ];

% f(x) -- without actuator dynamics
fxs(xnus) =   [   dV0x
                dg0x
                dhx
                da0x
                dq0x    ];


% g(x) -- without actuator dynamics
gxs(xnus) =   [   dVux    0
                dgux    0
                0       0
                daux    0
                0       dqux    ];

% g(x) -- without actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxdEs(xnus) = [   dVux    0
                dgux    dgudEx
                0       0
                daux    -dgudEx
                0       dqux    ];


%%
% *************************************************************************
% *************************************************************************
%
% PARTIAL DERIVATIVES OF AERO FUNCTIONS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% THRUST T
%      

% Jacobian
dTdz = jacobian(T);
fdTdz = formula(dTdz);

% % Hessian
% d2Tdz2 = jacobian(dTdz);
% fd2Tdz2 = formula(d2Tdz2);

% Extract terms from Jacobian
T_V = fdTdz(indV);
T_a = fdTdz(inda);
T_dT = fdTdz(inddT);
T_h = fdTdz(indh);


% ***********************
%       
% LIFT L
%      

% Jacobian
dLdz = jacobian(L);
fdLdz = formula(dLdz);

% % Hessian
% d2Ldz2 = jacobian(dLdz);
% fd2Ldz2 = formula(d2Ldz2);

% Extract terms from Jacobian
L_V = fdLdz(indV);
L_a = fdLdz(inda);
L_h = fdLdz(indh);

% ***********************
%       
% DRAG D
%      

% Jacobian
dDdz = jacobian(D);
fdDdz = formula(dDdz);

% % Hessian
% d2Ddz2 = jacobian(dDdz);
% fd2Ddz2 = formula(d2Ddz2);

% Extract terms from Jacobian
D_V = fdDdz(indV);
D_a = fdDdz(inda);
D_h = fdDdz(indh);



% ***********************
%       
% PITCHING MOMENT M_{yy}|_{u=0}
%     

% Jacobian
dMyy0dxv = jacobian(Myy0);
fdMyy0dxv = formula(dMyy0dxv);

Myy0_V = fdMyy0dxv(indV);
Myy0_h = fdMyy0dxv(indh);
Myy0_a = fdMyy0dxv(inda);
Myy0_q = fdMyy0dxv(indq);


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
% LINEARIZATION -- FROM HAND DERIVATION -- WITHOUT ACTUATOR DYNAMICS
% 
% *************************************************************************

% ***********************
%       
% df_1 / dx
%    

a11(xvunus) = (- D_V) / mref;
a12(xvunus) = - mu * cg / r^2;
a13(xvunus) = (- D_h) / mref + 2 * mu * sg / r^3;
a14(xvunus) = (- D_a) / mref;
% a15 = 0;
a16 = T_dT * ca / mref;
% a17 = 0;

% ***********************
%       
% df_2 / dx
%   

a21(xvunus) = (L_V ) / (mref*xvunus(indV)) ...
    - (L ) / (mref*xvunus(indV)^2) + 2 * cg / r ...
    + (mu - xvunus(indV)^2 * r) * cg / (xvunus(indV)*r)^2;
a22(xvunus) = (mu - xvunus(indV)^2 * r) * sg / (xvunus(indV) * r^2);
a23(xvunus) = (L_h ) / (mref*xvunus(indV)) + xvunus(indV) * cg / r^2 ...
    + 2 * (mu - xvunus(indV)^2 * r) * cg / (xvunus(indV) * r^3);
a24(xvunus) = (L_a) / (mref*xvunus(indV));
% a25 = 0;
a26 = T_dT * sa / (mref*xvunus(indV));
% a27 = 0;


% ***********************
%       
% df_3 / dx
%   
a31 = sg;
a32 = xvunus(indV) * cg;
% a33 = 0;
% a34 = 0;
% a35 = 0;
% a36 = 0;
% a37 = 0;

% ***********************
%       
% df_4 / dx
%   
% See below

% ***********************
%       
% df_5 / dx
%   

a51 = Myy0_V / Iyy;
% a52 = 0;
a53 = Myy0_h / Iyy;
a54 = Myy0_a / Iyy;
a55 = Myy0_q / Iyy;
% a56 = 0
% a57 = 0;


% ***********************
%       
% "B" MATRIX -- ADDING IN ELEVATOR-INDUCED LIFT EFFECTS 
%   
b22 = dgudE;


% ***********************
%       
% "A", "B", "C", "D" MATRIX FOR STANDARD 3-DOF MODEL
%   

% d f / d x
dfdx_sym = [  a11     a12     a13     a14     0
            a21     a22     a23     a24     0
            a31     a32     0       0       0
            -a21    -a22    -a23    -a24    1
            a51     0       a53     a54     a55     ];
dfdx_sym(xnus) = subs(dfdx_sym,xvunus,xvs00nu);
dfdx_sym = formula(dfdx_sym);
dfdxil = matlabFunction(dfdx_sym, 'vars', {xnus});

% "B" MATRIX -- WITHOUT ELEVATOR-INDUCED LIFT EFFECTS
% g_1(x): d_T
g1s =  [    a16    
            a26    
            0     
            -a26  
            0        ];
g1s(xnus) = subs(g1s,xvunus,xvs00nu);
g1s(xnus) = formula(g1s);
% g_2(x): d_E
g2s =  [    0   
            0    
            0     
            0  
            ddadE        ];
g2s(xnus) = subs(g2s,xvunus,xvs00nu);
g2s(xnus) = formula(g2s);

Bp_sym(xnus) = formula([g1s g2s]);


% "B" MATRIX -- WITH ELEVATOR-INDUCED LIFT EFFECTS
% g_2(x): d_E (with elevator-lift effects)
g2sdE = [   0   
            b22    
            0     
            -b22  
            ddadE        ];
g2sdE(xnus) = subs(g2sdE,xvunus,xvs00nu);
g2sdE(xnus) = formula(g2sdE);


BpdE_sym(xnus) = formula([g1s g2sdE]);
BpdEil = matlabFunction(BpdE_sym, 'vars', {xnus});

% d g_j(x) / d x 
dg1dxs = formula(jacobian(g1s, xs));
dg2dxs = formula(jacobian(g2s, xs));
dg2dxsdE = formula(jacobian(g2sdE, xs));

dg1dxil = matlabFunction(dg1dxs, 'vars', {xnus});
dg2dxdEil = matlabFunction(dg2dxsdE, 'vars', {xnus});



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

% With actuator dynamics
model.fxvs = fxvs;
model.gxvs = gxvs;
model.gxvdEs = gxvdEs;

% Without actuator dynamics
model.fxs = fxs;
model.gxs = gxs;

% Without actuator dynamics -- WITH ELEVATOR-LIFT COEFFICIENT EFFECTS
model.gxdEs = gxdEs;

% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

model.dfdx_sym = dfdx_sym;
model.g1s = g1s;
model.g2s = g2s;
model.Bp_sym = Bp_sym;
model.g2sdE = g2sdE;
model.BpdE_sym = BpdE_sym;
model.dg1dxs = dg1dxs;
model.dg2dxs = dg2dxs;
model.dg2dxsdE = dg2dxsdE;

% Inline functions
model.dfdxil = dfdxil;
model.BpdEil = BpdEil;
model.dg1dxil = dg1dxil;
model.dg2dxdEil = dg2dxdEil;

% ELEVATOR-INDUCED LIFT EFFECTS
model.dgudE = dgudE;




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL FROM AERO DATA
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function model_nu = trim_model(model, model_nu)


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

m = model.m;

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
xvs = model.xvs;
% xvunus = model.xvunus;
xnus = model.xnus;
% us = model.us; 
xvunus = model.xvunus;
% indsz = model.indsz;
% xvs0 = model.xvs0;
% xvs00nu = model.xvs00nu;

% ***********************
%
% SYSTEM DYNAMICAL EQUATIONS
%

% Model with actuator dynamics
fxvs = model.fxvs;
gxvdEs = model.gxvdEs;

% Without actuator dynamics
fxs = model.fxs;
gxdEs = model.gxdEs;

% ***********************
%
% MISC
%

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% Trim flight condition
xve = model.xve;
ue_ws = model.ue_ws;

% Inline function for speed of sound a
axnuil = model.axnuil;

% AOA index
inda = model.inda;

% \nu vector
nuvec = model_nu.nuvec;

%%
% *************************************************************************
% *************************************************************************
%
% CREATE INLINE FUNCTIONS
% 
% *************************************************************************
% *************************************************************************

% Symbolic variable with specific value of \nu
xnunums = [ xs
            nuvec ];

% Symbolic variable xv with specific value of \nu
xvnunums = [ xvs
             zeros(m,1)
             nuvec ];

% f(x) -- with actuator dynamics
tmp(xvs) = subs(fxvs,xvunus,xvnunums);
fil = matlabFunction(tmp, 'vars', {xvs});

% g(x) -- with actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
tmp(xvs) = subs(gxvdEs,xvunus,xvnunums);
gil = matlabFunction(tmp, 'vars', {xvs});

% f(x) -- without actuator dynamics
fxnus(xs) = subs(fxs,xnus,xnunums);
fx = matlabFunction(fxnus, 'vars', {xs});
sys.model.fx = fx;


% g(x) -- without actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxnus(xs) = subs(gxdEs,xnus,xnunums);
gx = matlabFunction(gxnus, 'vars', {xs});
sys.model.gx = gx;



%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM -- WITHOUT LIFT ELEVATOR EFFECTS -- REMOVED
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************

% Initialize trim parameters
M0 = 15;                            % M = 15 at trim
a0 = axnuil([xve; nuvec]);       % Solve for speed of sound at h_0
V0 = M0*a0;                         % V = M * a
gamma0 = 0;                         % Level flight trim condition
h0 = 110e3;                         % h_0 = 110 kft
% Store trim conditions
trimconds.V0 = V0;               
trimconds.gamma0 = gamma0;           
trimconds.h0 = h0;              
sys.model.trimconds = trimconds;

% Initial guess at trim values. From Wang, Stengel (2000)
% optparam = [  dT      Throttle setting
%               dE      Elevator deflection (rad)
%               alpha   AOA (rad)                   ]
% initparam = [   ue_ws(1)
%                 ue_ws(2)
%                 xve(inda) ];
%
dTinit = 0.2 * (nuvec(2) - 1) + 0.175621441396264;
dEinit_deg = 0.4 * (nuvec(1) - 1) + 2 * (nuvec(3) - 1) - 0.394672235421157;
dEinit = D2R * dEinit_deg;
ainit = D2R * 1.770370827541618;
initparam = [   dTinit
                dEinit
                ainit ];

% fminsearch options
options = optimset('TolX',1e-16, 'TolFun',1e-16, 'MaxFunEvals',1000);



%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM -- WITH LIFT ELEVATOR EFFECTS
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************

dotrim = 1;
currinitparam = initparam;

while dotrim

% Peform trim solve
[optparam, J, exitflag, output] = ...
    fminsearch('trimcost_3dof', currinitparam, options);

% Get trim values
dT0_dE = optparam(1);
dE0_dE = optparam(2);
alpha0_dE = optparam(3);

dE0_dE_deg = R2D*dE0_dE;

if (abs(dE0_dE_deg) > 1) %|| (dE0_dE_deg > -0.2) 
    for i = 1:3
        iprmi = initparam(i);
        currinitparam(i) = iprmi + 0.2 * abs(iprmi) * (rand - 0.5);
    end
else
    dotrim = 0;
end

end

disp('***********************')
disp('*')
disp('* TRIM CALCULATION -- WITH LIFT ELEVATOR EFFECTS')
disp('*')
disp('***********************')
disp(' ')

% Final trim cost, exit flag
disp(['Trim Cost = ' num2str(J)])
disp(['Exit Flag = ' num2str(exitflag)])

% Trim settings -- throttle d_T, elevator d_E, AOA \alpha
disp(['Trim Throttle            d_T =      ' num2str(dT0_dE)])
disp(['Trim Elevator Deflection d_E =      ' num2str(dE0_dE) ' rad = ' ...
    num2str(R2D*dE0_dE) ' deg'])
disp(['Trim AOA                 alpha =    ' num2str(alpha0_dE) ...
    ' rad = ' num2str(R2D*alpha0_dE) ' deg'])

% % Trim state x_0
% disp(['Trim State x_0 = [V_0, \gamma_0, h_0, \alpha_0, q_0]^T = '])
xe_dE = [  V0
        gamma0
        h0
        alpha0_dE
        0               ];

% Trim control x_0
disp(['Trim Control u_0 = [d_{T,0}, d_{E,0}]^T = '])
ue_dE = [  dT0_dE
        dE0_dE     ]
				
% Evaluate system dynamics
f_xe_dE = fx(xe_dE);
g_xe_dE = gx(xe_dE);
xdot_dE = f_xe_dE + g_xe_dE * ue_dE;

% disp(['State Derivatives Evaluated at Trim: ' ...
%     '\dot{x} = f(x_0) + g(x_0) * u_0 ='])
% 
% xdot_dE

disp(['||\dot{x} = f(x_0) + g(x_0) * u_0 || = ' num2str(norm(xdot_dE))])


% % Store equilibrium calculations
% trimconds.xe_dE = xe_dE;
% trimconds.ue_dE = ue_dE;

% Store equilibrium calculations
trimconds.xe = xe_dE;
trimconds.ue = ue_dE;






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
% INLINE FUNCTIONS
%

% With actuator dynamics
model_nu.fil = fil;
model_nu.gil = gil;

% Without actuator dynamics
model_nu.fx = fx;
model_nu.gx = gx;

% ***********************
%
% VEHICLE TRIM
%

model_nu.trimconds = trimconds;



%%
% *************************************************************************
% *************************************************************************
%
% LINEARIZE SYSTEM
% 
% *************************************************************************
% *************************************************************************

function lin = linearize_model(model, trimconds, nuvec, settings)

% Get current design tag
designtag = settings.designtag;


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% Number of inputs
m = 2;

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

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% h index
indh = model.indh;

% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

% Inline functions
dfdxil = model.dfdxil;
BpdEil = model.BpdEil;
dg1dxil = model.dg1dxil;
dg2dxdEil = model.dg2dxdEil;


% ***********************
%
% VEHICLE TRIM
%

xe = trimconds.xe;
ue = trimconds.ue;


% [x_e, \nu]
xenu = [    xe
            nuvec ];

% d {g(x) u} / d x 
dgudx = dg1dxil(xenu) * ue(1) ...
    + dg2dxdEil(xenu) * ue(2);

% Outputs: [V h]
Cvh = [    1   0   0   0   0
            0   0   1   0   0   ];

% Outputs y = [V, \gamma]^T
Cvg = [eye(2) zeros(2,3)];

Dp = zeros(m);

% % Evaluate numerically at trim
% Ap = double(subs(dfdx_sym, xs, xe)) + dgudx;
% Bp = double(subs(Bp_sym, xs, xe));
% 
% % Evaluate numerically at trim -- WITH ELEVATOR-LIFT EFFECTS
% ApdE = double(subs(dfdx_sym, xs, xe_dE)) + dgudxdE;
% BpdE = double(subs(BpdE_sym, xs, xe_dE));

% Evaluate numerically at trim -- WITH ELEVATOR-LIFT EFFECTS
Ap = dfdxil(xenu) + dgudx;
Bp = BpdEil(xenu);



% Plant y = [V, h]^T -- WITH ELEVATOR-LIFT EFFECTS
Pvh = ss(Ap,Bp,Cvh,Dp);

% Plant y = [V, \gamma]^T -- WITH ELEVATOR-LIFT EFFECTS
Pvg = ss(Ap,Bp,Cvg,Dp);

% Plant y = x
Px = ss(Ap,Bp,eye(5),zeros(5,2));


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
%     u' = [    d_T
%               d_E  (deg)]
%
%
% New State Variables
%       x' =   [    V (ft/s)
%                   \gamma (deg)
%                   -----------
%                   \theta (deg)
%                   q (deg/s)       ]
%
%            = [    y
%                   x_r             ]
%    
% New Output Variables
%       y' =   [    V (ft/s)
%                   \gamma (deg)    ] 
%          = y
%
% I.e., we have permuted the FPA \gamma and altitude h, we have converted
% all angular states/controls from rad to deg, and we have kept the outputs
% unchanged (since the outputs are both transaltional in nature, which we
% have not scaled).
%

% Remove height mode from original matrices
indsnoh = [(1:indh-1)'; (indh+1:5)'];
% Apnoh = ApdE(indsnoh,indsnoh);
% Bpnoh = BpdE(indsnoh,:);
Apnoh = Ap(indsnoh,indsnoh);
Bpnoh = Bp(indsnoh,:);
Cvgnoh = Cvg(:,indsnoh);
Dpnoh = Dp;
Pnoh = ss(Apnoh,Bpnoh,Cvgnoh,Dpnoh);

% Velocity scaling factor for transformation
% Vscl = 1;
switch designtag
    case 'ICLR_2025'
        Vscl = 1/100;
    otherwise
        Vscl = 1/1000;
end

% Use degrees in LQ servo design coords (=1) or rad (=0)
deg1rad0 = 1;
% deg1rad0 = 0;

if deg1rad0

    % Coordinate transformations
    sud = diag([1 R2D]);
    sxdnoh = [ Vscl 0   0   0   
            0   R2D 0   0   
            0   R2D R2D 0 
            0   0   0   R2D        ];
    syd = diag([Vscl R2D]);
    
    % Coordinate transformation -- with h
    sxd = [ Vscl 0   0   0  0
            0   R2D 0   0   0  
            0   R2D 0   R2D 0 
            0   0   0   0   R2D
            0   0   1   0   0       ];

else

    sud = diag([1 1]);
    sxdnoh = [ Vscl 0   0   0   
            0   1 0   0   
            0   1 1 0 
            0   0   0   1        ];
    syd = diag([Vscl 1]);
    sxd = [ Vscl 0   0   0  0
            0   1 0   0   0  
            0   1 0   1 0 
            0   0   0   0   1
            0   0   1   0   0       ];

end

% Scaled linear dynamics -- without h
Ad = sxdnoh*Apnoh*inv(sxdnoh);
Bd = sxdnoh*Bpnoh*inv(sud);
Cd = syd*Cvgnoh*inv(sxdnoh);
Dd = syd*Dpnoh*inv(sud);

% Scaled design plant -- without h
Pd = ss(Ad,Bd,Cd,Dd);

% Scaled linear dynamics -- with h
Adh = sxd*Ap*inv(sxd);
Bdh = sxd*Bp*inv(sud);
Cdh = syd*Cvg*inv(sxd);
Ddh = syd*Dp*inv(sud);

% Scaled design plant -- with h
Pdh = ss(Adh,Bdh,Cdh,Ddh);


% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS
%
% [ z                   [   x_1
%   y           ->          x_2     ]
%   x_r ]
%
%
%       x_1 = [ z_V (kft/s -s)
%               V   (kft/s)     ]
%
%       x_2 = [ z_\gamma    (deg-s)
%               \gamma      (deg)
%               \theta      (deg)
%               q           (deg/s) ]
%               
%       x_3 =   h   (ft)         
%
% *************************************************************************

% DIRL state transformation: [z, y, x_r] -> [x_1 x_2]
Sxirl = [   1 0 0       0 0 0 
            0 0 1       0 0 0 
            0 1 0       0 0 0 
            zeros(3)    eye(3)  ];

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
%       x_1 = [ z_V (kft/s -s)
%               V   (kft/s)     ]
%
%       x_2 = [ z_\gamma    (deg-s)
%               \gamma      (deg)
%               \theta      (deg)
%               q           (deg/s) ]
%               
%       x_3 =   h   (ft)         
%
% *************************************************************************

Sxirl_x3 = blkdiag(Sxirl, eye(1));

invSxirl_x3 = inv(Sxirl_x3);

% *************************************************************************
%
% FORM PLANT FOR DIRL
%
% State: x = [  x_1
%               x_2
%               x_3     ]
%
% *************************************************************************


% Augmented plant
% x = [z^T x_p^T]^T
Aaug = [    zeros(m)   Cdh
            zeros(5,m) Adh  ];
Baug = [    zeros(m,m)
            Bdh         ];


% DIRL state-space matrices
Airl = Sxirl_x3 * Aaug * inv(Sxirl_x3);
Birl = Sxirl_x3 * Baug;

% *************************************************************************
%
% FORM SIMPLIFIED DECOUPLED DESIGN PLANT FOR INNER-OUTER LOOP DESIGN
%
% *************************************************************************

% Whole system
Adio = Ad;
Adio(1,:) = 0;
Adio(:,1) = 0;
Adio(4,4) = 0;

Bdio = Bd;
Bdio(2,1) = 0;

Cdio = Cd;
Ddio = Dd;

% \delta_{T} -> V subsystem
AdTV = Adio(1,1);
BdTV = Bdio(1,1);
CdTV = 1;
DdTV = 0;

% \delta_{T} -> V subsystem -- exact A_{11}
Ad11 = Ad(1,1);
Bd11 = BdTV;
Cd11 = CdTV;
Dd11 = DdTV;

% \delta_{E} -> \gamma subsystem
AdEg = Adio(2:4,2:4);
BdEg = Bdio(2:4,2);
CdEg = [1 0 0];
DdEg = 0;

% \delta_{E} -> \gamma subsystem -- exact A_{22}
Ad22 = Ad(2:4,2:4);
Bd22 = BdEg;
Cd22 = CdEg;
Dd22 = DdEg;


% Multivariable design plant
Pdio = ss(Adio,Bdio,Cdio,Ddio);

% Plant \delta_{T} -> V
PdTV = ss(AdTV,BdTV,CdTV,DdTV);
% zpk(PdTV)

% Plant \delta_{T} -> V -- exact A_{11}
Pd11 = ss(Ad11,Bd11,Cd11,Dd11);

% Plant \delta_{E} -> \gamma
PdEg = ss(AdEg,BdEg,CdEg,DdEg);
% zpk(PdEg)

% Plant \delta_{E} -> \gamma -- exact A_{22}
Pd22 = ss(Ad22,Bd22,Cd22,Dd22);

% Plant \delta_{E} -> \theta
PdEth = ss(Adio,Bdio(:,2),[0 0 1 0],0);
% zpk(PdEth)

% Simplified linear dynamics -- re-scaled back to original
Apnohio = inv(sxdnoh)*Adio*sxdnoh; 
Bpnohio = inv(sxdnoh)*Bpnoh*sud;
Cpnohio = inv(syd)*Cvgnoh*sxdnoh;
Dpnohio = inv(syd)*Dpnoh*sud;



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



% Output scaling original -> LQ servo coords
if deg1rad0
    lin.yscl = [Vscl; R2D];
else
    lin.yscl = [Vscl; 1];
end

% A, B, C, D matrix -- WITH ELEVATOR-LIFT EFFECTS
lin.Apnoh = Apnoh;
lin.Bpnoh = Bpnoh;
lin.Cvh = Cvh;
lin.Cvg = Cvg;
lin.Dpnoh = Dpnoh;

% Plant ss objects -- WITH ELEVATOR-LIFT EFFECTS
lin.Pvh = Pvh;          % Plant y = [V, h]^T
lin.Pvg = Pvg;          % Plant y = [V, \gamma]^T
lin.Px = Px;    



% ***********************
%
% LINEARIZATION TERMS -- DESIGN PLANT FOR PD-PI INNER-OUTER
%

io.sud = sud;
io.sxd = sxd;
io.sxdnoh = sxdnoh;
io.syd = syd;

io.Apnoh = Apnoh;
io.Bpnoh = Bpnoh;
io.Cvgnoh = Cvgnoh;
io.Dpnoh = Dpnoh;
io.Pnoh = Pnoh;

io.Ad = Ad;
io.Bd = Bd;
io.Cd = Cd;
io.Dd = Dd;
io.Pd = Pd;
io.nlq = size(Ad,1);        % System order for LQ servo


% Scaled linear dynamics -- with h
io.Adh = Adh;
io.Bdh = Bdh;
io.Cdh = Cdh;
io.Ddh = Ddh;

% IRL linear dynamics, coord transformations
io.Airl = Airl;
io.Birl = Birl;
io.Sxirl = Sxirl;
io.invSxirl = invSxirl;
io.Sxirl_x3 = Sxirl_x3;
io.invSxirl_x3 = invSxirl_x3;

% Scaled design plant -- with h
io.Pdh = Pdh;

io.Adio = Adio;
io.Bdio = Bdio;
io.Cdio = Cdio;
io.Ddio = Ddio;
io.Pdio = Pdio;

% \delta_{T} -> V subsystem
io.AdTV = AdTV;
io.BdTV = BdTV;
io.CdTV = CdTV;
io.DdTV = DdTV;
io.PdTV = PdTV;

% \delta_{T} -> V subsystem -- with exact A_{11}
io.Ad11 = Ad11;
io.Bd11 = Bd11;
io.Cd11 = Cd11;
io.Dd11 = Dd11;
io.Pd11 = Pd11;

% \delta_{E} -> \gamma subsystem
io.AdEg = AdEg;
io.BdEg = BdEg;
io.CdEg = CdEg;
io.DdEg = DdEg;
io.PdEg = PdEg;

% \delta_{E} -> \gamma subsystem -- with exact A_{22}
io.Ad22 = Ad22;
io.Bd22 = Bd22;
io.Cd22 = Cd22;
io.Dd22 = Dd22;
io.Pd22 = Pd22;


io.PdEth = PdEth;

% Simplified linear dynamics -- re-scaled back to original
io.Apnohio = Apnohio; 
io.Bpnohio = Bpnohio;
io.Cpnohio = Cpnohio;
io.Dpnohio = Dpnohio;


% Store all inner/outer terms
lin.io = io;


% Store linearization params
model.lin = lin;




