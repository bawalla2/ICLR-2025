function Vq = interpn_cell(X_cell, V, xqmat)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% n-DIMENSIONAL INTERPOLATION OF VECTOR-VALUED FUNCTION WITH CELL ARRAY
% INPUT ARGUMENTS
%
% [ ***** ANONYMIZED ***** ]
%
% 2023-03-08
%
% This program allows for calling the function 'interpn' in a programmatic
% fashion without enumeration of the gridpoint vectors and query points as
% explicit arguments to the function. Ultimately, this program executes the
% command:
%
%   Vq = interpn(X1,X2,...,Xn,V,Xq1,Xq2,...,Xqn)
% 
% See: https://www.mathworks.com/help/matlab/ref/interpn.html
%
% Where X1,...,Xn are the gridpoint vectors, V is the lookup table, and
% Xq1,...,Xqn \in R are the coordinates of the query points in each
% dimension.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% X_cell            (n x 1 Cell) The i-th entry contains the gridpoint
%                   vector in the i-th dimension of the lookup table Xi \in
%                   R^{k_i}, where k_i \in N is the length of the i-th
%                   gridpoint vector Xi (i.e., number of data points in the
%                   grid in that dimension).
% V                 TWO POSSIBLE FORMATS:
%                       1). (k_1 x .... x k_n Array) Contains the gridpoint
%                       data to be interpolated.
%                       2). (k_1 x .... x k_n x m Array) Represents a
%                       vector-valued function into R^m over the n-dim.
%                       grid. In this case, the output Vq will be
%                       m-dimensional (see below). 
% xqmat             (n x n_q Matrix) The i-th column of this matrix
%                   represents the i-th query point. This program allows
%                   for querying multiple points (n_q \in N) at once. In
%                   the case of only one query n_q = 1, xqmat is a vector
%                   containing the query point (i.e., consisting of the
%                   coordinates Xq1,...,Xqn in the above notation).
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% Vq                (m x n_q Matrix) Contains the interpolated data of 
%                   V : R^n -> R^m at the query points in xqmat.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Dimension of interpolation vectors
n = size(xqmat,1);

% Number of points to evaluate at
n_q = size(xqmat,2);

% Size, dimension of grid data array
szV = size(V);
dimV = ndims(V);

% Dimension of function output
if dimV == n
    m = 1;
else
    m = szV(end);
end

% Initialize empty output array
Vq = zeros(m, n_q);

% Fill query array
for i = 1:m
    
    % Get lookup table for this output
    if m == 1
        Vi = V;
    else
        % Clever indexing trick for array of unknown dimension
        % https://www.mathworks.com/matlabcentral/answers/32362
        %   -how-to-extract-elements-along-specified-dimension-of-array
        inds = cell(dimV, 1);
        for k = 1:dimV-1
            inds{k} = 1:szV(k);
        end
        inds{dimV} = i;
        Vi = V(inds{:});
    end

    % Loop over query points
    for j = 1:n_q

        % Get current query point
        x = xqmat(:, j);

        % Convert query point to cell array
        x_cell = cell(n, 1);
        for k = 1:n
            x_cell{k} = x(k);
        end

        % Perform n-dim interpolation at query point
        Vq(i, j) = interpn(X_cell{:}, Vi, x_cell{:});

    end

end

