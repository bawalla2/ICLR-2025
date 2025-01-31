function xdot = odefunct_eval_cost(t, x)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMICS -- FOR COST EVALUATION
%
% [ ***** ANONYMIZED ***** ]
%
% 2023-12-10
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Global variables
global sys;
n = sys.n;
nlq = sys.nlq;
m = sys.m;

% Control settings
% global r_sett;
global u_sett_eval;

% % Extract nominal model
% model_nom = u_sett_eval.model_nom;

% Extract simulation model
model_sim = u_sett_eval.model_sim;

% Get coordinate transformations
sx = u_sett_eval.sx;
% sxh = u_sett_eval.sxh;
su = u_sett_eval.su;
sy = u_sett_eval.sy;
Sxirl_x3 = u_sett_eval.Sxirl_x3;   % [z, y, x_r, x_3] -> [x_1, x_2, x_3]
% Sxirl = u_sett_eval.Sxirl;   % [z, y, x_r, x_3] -> [x_1, x_2]

% Get cost structure Q, R
Q = u_sett_eval.Q;
R = u_sett_eval.R;

% Get indices of state partition
inds = u_sett_eval.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett_eval.hasintaug;

% Initialize empty state derivative vector
xdot = zeros(u_sett_eval.n_sim, 1);

% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett_eval.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett_eval.ue_sim;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Get state of linear system (post-transformation) 
% \tilde{x}^{\prime} = S_x * \tilde{x}
txp = sx * tx;

% Evaluate drift dynamics
if u_sett_eval.lin1nonlin0
    % System linear
    f_x = model_sim.lin.Ap * tx;
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett_eval.lin1nonlin0
    % System linear
    g_x = model_sim.lin.Bp;
else
    % System nonlinear
    g_x = model_sim.gx(xp);
end

% Calculate control signal
u = eval_u(x, t, u_sett_eval);

% Calculate \tilde{u} = u - u_{e} (pre-transformation) 
tu = u - ue;

% Calculate \tilde{u}^{\prime} = S_u * \tilde{u} (post-transformation) 
tup = su * tu;

% State derivatives
if u_sett_eval.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
% NOTE: \tilde{r}^{\prime}(t) = \tilde{y}_r^{\prime}(t) -
% \tilde{y}_e^{\prime} = 0 here (ref cmd is just the equilibrium output)
trp = zeros(m, 1);

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett_eval.pf1nopf0
    trfp = x(inds.indspf);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if hasintaug
    if u_sett_eval.pf1nopf0
        zdot = -(trfp - typ);
    else
        zdot = -(trp - typ);
    end
    xdot(inds.indsz) = zdot;
end

% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett_eval.pf1nopf0
    pfdot = -diag(u_sett_eval.pfavec) * trfp + diag(u_sett_eval.pfavec) * trp;
    xdot(inds.indspf) = pfdot;
end 

% Extract the integral augmentation states z (post-transformation)
if hasintaug
    z = x(inds.indsz);
    
    % Form aggregate state x = [z^T x_p^T]^T = [z^T y^T x_r^T]^T 
    % (post-trans)
    zxp = [ z 
            txp ];
else
    zxp = txp;
end

% Perform transformation [z, y, x_r, x_3] -> [x_1 x_2 x_3]
xirl_x3 = Sxirl_x3 * zxp;
if hasintaug
    xirl = xirl_x3(1:nlq+m);
else
    xirl = xirl_x3(1:nlq);
end

% Calculate running cost r(x,i) = x^T Q x + u^T R u
rxu = xirl' * Q * xirl + tup' * R * tup;

% Set running cost as J(x) derivative
xdot(inds.indsJ) = rxu;