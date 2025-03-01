function [phix, dphix] = eval_phi(x, basis)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE BASIS FUNCTIONS AND BASIS FUNCTION GRADIENTS
%
% [ ***** ANONYMIZED ***** ]
%
% 2021-11-06
%
% This program, given a state vector x in R^n and N-dimensional basis
% selection tag evaluates the basis functions and their gradients
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% [phix, dphix] = eval_phi(x, basis)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% x         Current value of the state (n-dimensional vector).
% basis     (Struct) Contains basis parameters. Includes the fields:
%   tag     (String) Tag of the corresponding basis.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% phix      Basis functions evaluated at x (N-dimensional vector)
% dphix     Basis fuction gradient evaluated at x (N x n matrix)
%
% *************************************************************************
%
% NOTE ON 'monomial' BASIS
%
% *************************************************************************
%
% If the basis is of tag 'monomial', the following additional fields must
% be present in the 'basis' struct:
%
%   type        (String) Monomial basis type. Has the following options:
%       'all'       Evaluate all monomials of total degree <= K.
%       'even'      Evaluate even monomials of total degree <= K.
%       'odd'       Evaluate odd monomials of total degree <= K.
%       'total_deg_K' Monomials of total degree = K only.
%       'custom'    Evaluate custom list of monomials.
%
%   IC_type     (String) Describes the network weight initialization
%               behavior. Has the following options:
%       'none'      ICs are to be declared manually later. Do not
%                   initialize weights here.
%       'custom'    ICs are to be set programatically here. See below note.
%       'lqr'       This network is to be implemented in an actor network
%                   with g(x) known:
%           \hat{\mu}(x) = - 1/2 R^{-1} g^T(x) \nabla \Phi^T(x) w
%                   and the weights w_0 \in R^{N_1} are to be initialized
%                   according to the procedure outlined in
%                   config_basis_ICs_lqr.m. See this function description
%                   for more details.
%
% If 'type' is 'all', 'even', or 'odd', the following additional field
% needs to be declared in the 'basis' struct:
%   
%   K           (Integer) Max total degree of monomials in basis.
%
% If 'type' is 'custom', the following additional field needs to be
% declared in the 'basis' struct:
%
%   dmat        (N x n Matrix) A matrix of integer elements, where N
%               denotes the total number of activation functions and n the
%               system order. The i-th row, j-th column of of dmat will
%               contain the integer corresponding to the power of x_j in
%               the activation function \phi_j(x). E.g., for n = 3, j = 2,
%               dmat(j,:) might look like [ 0 2 1 ]. Then \phi_j(x) = x_1^0
%               * x_2^2 * x_3^1.
%
% NOTE: If 'type' is 'all', 'even', or 'odd', the field 'dmat' is
% initialized automatically in config.m.
%
% NOTE: If 'IC_type' is 'custom', then the function config_basis_ICs.m will
% be executed to initialize the network weights. The following additional
% fields need to be declared 'basis' struct (see config_basis_ICs.m for a
% description of these fields):
%   
%   dmat_IC     ('numIC' x n Matrix)
%   c0_ic       ('numIC'-dimensional vector)
%   
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

switch basis.tag

    
    % *********************************************************************
    %
    % MONOMIAL BASIS
    %
    % Note: See config.m for details of how the basis was initialized.
    % Depending on the value in 'basis.type', this will evaluate the
    % following monomials:
    %
    %   'all'       All monomials of total degree <= K.
    %   'even'      Even monomials of total degree <= K.
    %   'odd'       Odd monomials of total degree <= K.
    %   
    
    case 'monomial'
        
        % ***********************
        %
        % EXTRACT BASIS PARAMETERS
        %
        
        % Holds monomial powers of activation functions
        dmat = basis.dmat;
        
        % Number of activation functions
        N = size(dmat, 1);
        
        % System order
        n = size(dmat, 2);   
        
        % ***********************
        %
        % Phi(x) CALCULATION
        %
        % [Phi(x)](i) = \phi_i(x)
        %
        %             = \prod_{k=1}^{n} x_k^{dmat(i,k)}
        %
        
        phix = ones(N, 1);
        
        for i = 1:N
           
            for k = 1:n
               
                phix(i) = phix(i) * x(k)^dmat(i,k);
                
            end
            
        end
        
        % ***********************
        %
        % \nabla Phi(x) CALCULATION
        %
        % [\nabla Phi(x)](i,j) = \partial \phi_i(x) / \partial x_j
        %
        %             = [ \prod_{k \neq j} x_k^{dmat(i,k)} ] * 
        %                  dmat(i,j) * x_j^{dmat(i,k) - 1}
        %
        
        dphix = ones(N, n);
        
        
        for i = 1:N
            for j = 1:n
                % Calculate [\nabla Phi(x)](i,j)
                % Handle x_j = 0 case
                if abs(x(j)) == 0
                    dphix(i,j) = 0;
                % Else, x_j ~= 0
                else   
                    dphix(i,j) = (dmat(i,j)) * phix(i) / x(j);
                end
            end
        end
        
    
    % *********************************************************************
    %
    % 1ST ORDER SYSTEM, DEGREE 4
    %
    
    case 'order_1_degree_4'
        
        phix = [    x(1)
                    x(1)^2
                    x(1)^3
                    x(1)^4        ];
        
        dphix = [   1      
                    2*x(1)     
                    3*x(1)^2
                    4*x(1)^3        ];    
    
    % *********************************************************************
    %
    % 2ND ORDER SYSTEM, DEGREE 2
    %
    
    case 'order_2_degree_2'
        
        phix = [    x(1)^2
                    x(1) * x(2)
                    x(2)^2        ];
        
        dphix = [   2 * x(1)    0      
                    x(2)        x(1)     
                    0           2 * x(2)    ];       
                
    % *********************************************************************
    %
    % 2ND ORDER SYSTEM, DEGREE 4
    %
    
    case 'order_2_degree_4'
        
        phix = [    x(1)^2
                    x(1) * x(2)
                    x(2)^2
                    x(1)^4
                    x(1)^3 * x(2)
                    x(1)^2 * x(2)^2
                    x(1) * x(2)^3
                    x(2)^4              ];
        
        dphix = [   2 * x(1)            0
                    x(2)                x(1)
                    0                   2 * x(2)
                    4 * x(1)^3          0
                    3 * x(1)^2 * x(2)   x(1)^3
                    2 * x(1) * x(2)^2   2 * x(1)^2 * x(2)
                    x(2)^3              3 * x(1) * x(2)^2
                    0                   4 * x(2)^3          ];               
       

    % *********************************************************************
    %
    % VAMVOUDAKIS, LEWIS (2010) -- BASIS FOR ACTOR THAT DOESN'T USE g(x)
    %       
    
    case 'vamvoudakis_2010_actor_no_g'              

        phix =  [   x(1)
                    x(2)
                    cos(2*x(1)) * x(1)
                    cos(2*x(1)) * x(2) ];    

        dphix = [];     % Unused 

    % *********************************************************************
    %
    % VAMVOUDAKIS, LEWIS (2010) -- BASIS FOR HAMILTONIAN NN FOR VI
    % ALGORITHM \Theta(x) -- TERMS FOR Q(x) INCLUDED
    %       
    
    case 'vamvoudakis_2010_hamiltonian_min'

        phix = [    
                    x(2)^2 * cos(2*x(1))^2
                    x(2)^2 * cos(2*x(1))
                    x(1)^2
                    x(2)^2
                                            ];

        dphix = [];     % Unused        

    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('*** ERROR: BASIS TAG NOT RECOGNIZED ***');

end