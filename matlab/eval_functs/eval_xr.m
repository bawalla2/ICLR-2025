function xrt = eval_xr(t, xr_sett)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE REFERENCE COMMAND TRAJECTORY x_r(t)
%
% [ ***** ANONYMIZED ***** ]  
%
% 2022-08-23
%
% This program, given a tag of a desired noise reference command and time t
% evaluates the probing command x_r(t).
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% xrt = eval_xr(t, xr_sett)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% t         (Double) Time (sec)
% xr_sett        (Struct) Contains properties of the pecific signal to be
%           evaluated. Has the following fields:
%   tag     (String) Tag of the signal to be evaluated
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% xrt        (Vector) Evaluation of probing noise x_r(t)
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

switch xr_sett.tag

    % ***********************
    %
    % ZERO NOISE SIGNAL
    %
    
    case '0'
        
        xrt = 0;
 

    
        
    % ***********************
    %
    % SUM OF SINUSOIDS
    %   
    
    case 'sum_sin'
        

        % Get settings
        biasvec = xr_sett.biasvec;
        cos1_sin0 = xr_sett.cos1_sin0;
        Amat = xr_sett.Amat;
        Tmat = xr_sett.Tmat;       
        dorefvec = xr_sett.dorefvec;
        nderivvec = xr_sett.nderivvec;
        m = size(Amat,1);
        nsin = size(Amat,2);
        nsig = max(nderivvec + 1);

        % Initialize
        xrt = zeros(m,nsig);
        cnt = 1;
        
        for i = 1:m

            % Get count at beginning of this output
            cnt_0 = cnt;

            % Add sinusoidal terms in each derivative (if this output is
            % active)
            for j = 1:nsin

                % Reset counter to beginning
                cnt = cnt_0;

                % Pull settings for this output
                wij = 2*pi/Tmat(i,j);
                c1s0ij = cos1_sin0(i,j);
                c1s0ijk = c1s0ij;
                csgn = 1;

                % Counter to keep track of when to flip sign
                if c1s0ij
                    scnt = 2;
                else
                    scnt = 1;
                end 

                for k = 0:nderivvec(i)
                          
                    wijk = wij^k;


                    if dorefvec(i)
                        if c1s0ijk
                            tmp = cos(wij * t);
                        else
                            tmp = sin(wij * t);
                        end

                        tmp = Amat(i,j) * csgn * wijk * tmp; 
        
                        xrt(i,k+1) = xrt(i,k+1) +  tmp;
                    end

                    if mod(scnt,2) == 0
                        scnt = 0;
                        csgn = -csgn;
                    end

                    c1s0ijk = ~ c1s0ijk;

                    scnt = scnt + 1;
                    cnt = cnt + 1;

                end

               
            end

        end

        % Look for decaying exponential x_0
        hasexpx0 = isfield(xr_sett, 'expx0avec');
        if hasexpx0
            expx0avec = xr_sett.expx0avec;
            ty0 = xr_sett.ty0;
            xrt(:,1) = xrt(:,1) + ty0 .* exp(- expx0avec * t);
        end

        % Add bias term
        xrt(:,1) = xrt(:,1) + biasvec;


    % *********************************************************************
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: REFERENCE TRAJECTORY TAG NOT RECOGNIZED ***');

end