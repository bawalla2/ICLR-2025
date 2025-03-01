function ux = eval_u(x, t, u_sett)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE FEEDBACK CONTROL LAW u(x, t)
%
% [ ***** ANONYMIZED ***** ]  
%
% 2021-11-06
%
% This program, given a state vector x in R^n and preset tag evaluates the
% a specified policy u(x, t). 
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% x         (n-dim. Vector) Current value of the state.
% t         (double, OPTIONAL) Current time. Required only if policy is
%           time-varying.
%
% u_sett    (Struct) contains settings pertaining to the policy. Must have \
%           the following fields:
%   tag     (String) contains the string of the specific preset to evaluate
%           u(x, t) for.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% ux    Initial stabilizing policy u(x), evaluated at x.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

switch u_sett.tag

    % ***********************
    %
    % ZERO CONTROL SIGNAL
    %
    
    case '0'
        
        ux = 0;

    % ***********************
    %
    % LQ SERVO
    %
    % Required Fields to 'u_sett'
    %   n               (Integer) Number of states (without integral
    %                   augmentation)
    %   m               (Integer) Number of controls
    %   hasintaug       (Bool) Controller has integral augmentation (=1) or
    %                   not (=0)
    %   K               CASE: INTEGRAL AUGMENTATION -- 
    %                       (m x m + n Matrix) Contains the aggregate LQ
    %                       servo controller. State must be partitioned
    %                           x = [z^T, y^T, x_r^T]^T
    %                   CASE: NO INTEGRAL AUGMENTATION -- 
    %                       (m x n Matrix) Contains the aggregate LQ
    %                       servo controller. State must be partitioned
    %                           x = [y^T, x_r^T]^T
    %   xe_ctrl         (n-dim Vector) Contains equilibrium state x_e
    %                   (pre-transformation)
    %   ue_ctrl         (m-dim Vector) Contains equilibrium control u_e
    %                   (pre-transformation)
    %   sx              (n x n Matrix) State transformation to LQ servo
    %                   units.
    %   su              (m x m Matrix) Control transformation to LQ servo
    %                   units. 
    %   sy              (m x m Matrix) Output transformation to LQ servo
    %                   units.    
    %   inds            (Struct) Contains indices of the plant states x_p,
    %                   integral augmentation states z (if used)
    %                   where the rest of the state x_r may be empty.
    %   r_sett          (Struct, OPTIONAL) Reference command settings.
    %   t               (Double, OPTIONAL) Current time. Only needed if
    %                   reference command used.
    %   pf1nopf0        (Bool, OPTIONAL) Use reference command prefilter
    %                   (=1) or not (=0). Required only if prefilters are
    %                   used.
    %   
    %
    
    case 'lq_servo'
       
        % Get system dimensions
        nlq = u_sett.nlq;
        m = u_sett.m;
        
        % Reference command settings
        hasrsett = isfield(u_sett, 'r_sett');
        if hasrsett
            r_sett = u_sett.r_sett;            
        end        
        
        % Get coordinate transformations
        sx = u_sett.sx;
        su = u_sett.su;
        sy = u_sett.sy;

        % Get total system dimension
        n = size(sx, 1);
        
        % Indices of state vector partition
        inds = u_sett.inds;

        % Check for prefilter
        haspf = isfield(u_sett, 'pf1nopf0');

        % Has integral augmentation (=1) or not (=0)
        hasintaug = u_sett.hasintaug;    

        % Implement linear policy (=1) or nonlinear policy (=0)
        linpol1nonlinpol0 = u_sett.linpol1nonlinpol0; 
        
        % Extract policy parameters
        if linpol1nonlinpol0

            % Get contoller
            K = u_sett.K;
            
            % Extract parts of feedback gain matrix K corresponding to each
            % of the states in the partition
            if hasintaug
                Kz = K(:,1:m);
                Ky = K(:,m+1:2*m);
                Kr = K(:,2*m+1:m+nlq);
            else
                Ky = K(:,1:m);
                Kr = K(:,m+1:nlq);   
            end

        else

            % Get network parameters
            P = u_sett.P;

            % Get control penalty
            R = u_sett.R;
            
            % Get input gain matrix g(x) inline function
            gx = u_sett.gx;  

        end

        % Number of states in x_r
        nxr = nlq - m;

        % ***********************
        %       
        % LQ REFERENCE COMMAND FOLLOWING
        %   
        
        
        % Get equilibrium point x_e (pre-transformation) -- nominal system
        xe = u_sett.xe_ctrl;
        
        % Get equilibrium control u_e (pre-transformation) -- nominal
        % system
        ue = u_sett.ue_ctrl;

        % Get equilibrium value of the output y_{e} (pre-transformation)
        ye = xe(inds.indsxr);

        % Evaluate reference trajectory r(t) (pre-transformation)
        if hasrsett
            rt = eval_xr(t, r_sett);
        else
            rt = ye;
        end
        
        % Evaluate reference trajectory r(t) (post-transformation)
        yr = rt(:,1);
        yrp = sy * yr;

        % Get equilibrium value of the output y_{e} (post-transformation)
        yep = sy * ye;

        % Get (small-signal) reference command r (post-transformation)
        trp = yrp - yep;
        
        % If prefilters used, get filtered reference command
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                trfp = x(inds.indspf);
            end
        end
       
        % Extract plant states x_p (pre-transformation)
        xp = x(inds.indsx);
        
        % Extract the integral augmentation states z (post-transformation)
        if hasintaug
            z = x(inds.indsz);
        end
        
        % Get state of linear system (pre-transformation) 
        % \tilde{x} = x - x_e
        tx = xp - xe;
        
        % Apply coordinate transformation
        % x' = S_x x
        %    = [    y
        %           x_r
        %           x_3     ]
        txp = sx * tx;
        typ = txp(1:m);
        txr = txp(m+1:m+nxr);
        
        % Calculate (small-signal) tracking error e = y - r
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                e = -(trfp - typ);
            else
                e = -(trp - typ);
            end
        else
            e = -(trp - typ);
        end

        % Calculate control (post-transformation) 
        if linpol1nonlinpol0

            if hasintaug
                tup = Kz * (-z) + Ky * (-e) - Kr * txr;
            else
                tup = Ky * (-e) - Kr * txr;
            end

        else
            
            % Evaluate input gain matrix
            g_x = gx(xp);

            % Calculate input gain matrix g(x) in LQ servo coords (i.e.,
            % post-transformation in x and u, and not including x_3)
            gxp = sx * g_x * inv(su);
            gxp = gxp(1:nlq, :);

            if hasintaug
                xlq = [z; e; txr];
                gzxp = [zeros(m) ; gxp];
            else
                xlq = [e; txr];
                gzxp = gxp;
            end
    
            % Calculate nonlinear control policy (post-transformation)
            tup = - inv(R) * gzxp' * P * xlq;

        end
        
        
        % Calculate control (pre-tranformation)
        tu = su \ tup;
        
        % Calculate final control u = u_e + \tilde{u}
        ux = ue + tu;




    % ***********************
    %
    % cFVI -- POLICY TABLE LOOKUP
    %
    % Required Fields to 'u_sett'
    %   n               (Integer) Number of states (without integral
    %                   augmentation)
    %   m               (Integer) Number of controls
    %   xgridvec_cell   ((n+m)-dim Cell) entry i (i = 1,...,n+m) contains
    %                   the grid vector in state i
    %                       NOTE: In this table, the state is indexed as:
    %                           x = [x_p^T z^T]^T
    %   u_tbl           ((n+m+1)-dim Array) Contains policy data. The first
    %                   (n+m) dimensions index the state variables (each of
    %                   length equal to the grid length in that state
    %                   dimension), and the last dimension (of length m)
    %                   contains the policy evaluated at the state
    %                   gridpoints.
    %   xe_ctrl         (n-dim Vector) Contains equilibrium state x_e
    %                   (pre-transformation)
    %   ue_ctrl         (m-dim Vector) Contains equilibrium control u_e
    %                   (pre-transformation)
    %   inds            (Struct) Contains indices of the plant states x_p,
    %                   integral augmentation states z (if used)
    %                   where the rest of the state x_r may be empty.
    %   r_sett          (Struct, OPTIONAL) Reference command settings.
    %   t               (Double, OPTIONAL) Current time. Only needed if
    %                   reference command used.
    %   pf1nopf0        (Bool, OPTIONAL) Use reference command prefilter
    %                   (=1) or not (=0). Required only if prefilters are
    %                   used.
    %   
    %
    
    case 'cfvi_lookup'
       
        % Get system dimensions
        nlq = u_sett.n;
        m = u_sett.m;
        
        % Reference command settings
        hasrsett = isfield(u_sett, 'r_sett');
        if hasrsett
            r_sett = u_sett.r_sett;
        end        
        
        % Get coordinate transformations
        sx = u_sett.sx;
        su = u_sett.su;
        sy = u_sett.sy;
        
        % Indices of state vector partition
        inds = u_sett.inds;

        % Check for prefilter
        haspf = isfield(u_sett, 'pf1nopf0');

        % Has integral augmentation (=1) or not (=0)
        hasintaug = u_sett.hasintaug;

        % Get gridpoint vectors
        xgridvec_cell = u_sett.xgridvec_cell;        

        % Get lookup table
        u_tbl = u_sett.u_tbl;

        % ***********************
        %       
        % REFERENCE COMMAND FOLLOWING
        %           
        
        % Get equilibrium point x_e (pre-transformation) -- nominal system
        xe = u_sett.xe_ctrl;
        
        % Get equilibrium control u_e (pre-transformation) -- nominal
        % system
        ue = u_sett.ue_ctrl;

        % Get equilibrium value of the output y_{e} (pre-transformation)
        ye = xe(inds.indsxr);

        % Evaluate reference trajectory r(t) (pre-transformation)
        if hasrsett
            rt = eval_xr(t, r_sett);
        else
            rt = ye;
        end
        
        % Evaluate reference trajectory r(t) (post-transformation)
        yr = rt(:,1);
        yrp = sy * yr;

        % Get equilibrium value of the output y_{e} (post-transformation)
        yep = sy * ye;

        % Get (small-signal) reference command r (post-transformation)
        trp = yrp - yep;
        
        % If prefilters used, get filtered reference command
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                trfp = x(inds.indspf);
            end
        end
       
        % Extract plant states x_p (pre-transformation)
        xp = x(inds.indsx);
        
        % Extract the integral augmentation states z (post-transformation)
        if hasintaug
            z = x(inds.indsz);
        end
        
        % Get state of linear system (pre-transformation) 
        % \tilde{x} = x - x_e
        tx = xp - xe;
        
        % Apply coordinate transformation
        % x' = S_x x
        %    = [    y
        %           x_r
        %           x_3     ]
        txp = sx * tx;
        typ = txp(1:m);
        txr = txp(m+1:nlq);
        
        % Calculate (small-signal) tracking error e = y - r
        % (post-transformation)
        if haspf
            if u_sett.pf1nopf0
                e = -(trfp - typ);
            else
                e = -(trp - typ);
            end
        else
            e = -(trp - typ);
        end

        % Composite state (in cFVI coords): 
        % x = [x_p^T z^T]^T = [y^T x_r^T z^T]^T 
        if hasintaug
            xcfvi = [   e
                        txr
                        z   ];
        else
            xcfvi = [   e
                        txr  ];            
        end


        % Call interpolation function to get policy table lookup (yields
        % control post-transformation, shifted by trim)
        tup = interpn_cell(xgridvec_cell, u_tbl, xcfvi);

        % Check if interpolation failed due to x outside grid range
        if isnan(tup(1))
            error(['cFVI LOOKUP TABLE ERROR -- STATE x =    [' ...
                    num2str(xcfvi') ']      OUT OF RANGE'])
        end
        
        % Calculate control (pre-tranformation)
        tu = su \ tup;
        
        % Calculate final control u = u_e + \tilde{u}
        ux = ue + tu;

    % ***********************
    %
    % VAMVOUDAKIS (2010)
    %
    
    case 'vamvoudakis_2010'
        
        c0 = [1; 0; 4];

        ux = -1/2 * 1/1 * [0 cos(2*x(1)) + 2] * [2*x(1) x(2) 0
                                                 0 x(1) 2*x(2)] * c0;



    
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