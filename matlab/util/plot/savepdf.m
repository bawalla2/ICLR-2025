function savepdf(figcount, relpath, filename)

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% savepdf.m
%
% [ ***** ANONYMIZED ***** ]
%
% 1/23/20
%
% This function takes a matlab figure and saves it as a pdf
%
% Code from:
%       http://tipstrickshowtos.blogspot.com/2010/08/how-to-get-rid-of-white-margin-in.html
%
% CALL SYNTAX
%
% savepdf(figcount, relpath, filename)
%
% INPUTS
%
% figcount      -- Figure number of figure to be saved
% relpath       -- File path from MATLAB directory to save location
% filename      -- Desired file name with file type added; e.g. "fig.pdf"
%
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Create directory, if it doesn't exist already
if ~isdir(relpath)
    mkdir(relpath);
end

% ***********************
%
% OPEN CURRENT FIGURE
%

figure(figcount)


% ***********************
%
% FORMAT FIGURE FOR SAVING
%

set(gca,'units','centimeters')
pos = get(gca,'Position');
ti = get(gca,'TightInset');

w = pos(3)+ti(1)+ti(3);
h = pos(4)+ti(2)+ti(4);

set(gcf, 'PaperUnits','centimeters');
set(gcf, 'PaperSize', [w h]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition',[0 0 w h]);


% ***********************
%
% SAVE FIGURE
%


print(gcf, [relpath filename], '-dpdf', '-r300');


end

