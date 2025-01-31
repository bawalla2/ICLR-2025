function str = num2filename(x)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAKE FILE NAME STRING FOR DECIMAL NUMBER
%
% [ ***** ANONYMIZED ***** ] 
%
% 2023-03-13
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

str = [];

n = length(x);

for i = 1:n

    % Convert to string
    stri = num2str(x(i));
    
    % Replace decimal points '.' with letter 'p'
    stri = strrep(stri, '.', 'p');
    
    % Replace minus sign '-' with letter 'm'
    stri = strrep(stri, '-', 'm');

    % Append '_' to end if this is not the last number in the vector
    if i < n
        stri = [stri '_'];
    end

    % Append this entry to the full string
    str = [str stri];

end