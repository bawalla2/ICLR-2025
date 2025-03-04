function figcount = plot_dirl(alg_settings, out_data,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOTS FOR IPA ALGORITHM
%
% [ ***** ANONYMIZED ***** ]
%
% 2022-01-13
%
% This program handles plots specific to the data from the developed
% algorithm.
%
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings      (Struct) Algorithm settings/parameters corresponding to
%                   the current preset. See respective algorithm .m-file
%                   for a description of the relevant fields.
%                   NOTE: Regardless of the algorithm, alg_settings must
%                   contain the following fields:
%   group_settings   (Struct) contains plot settings for this particular
%                   preset. Has the following fields:
%       relpath     (String) Name of the folder to save plots to for this
%                   preset. This could be the preset tag, or any
%                   other convenient identifier.
% out_data          (Struct) Output data generated by the algorithm. See
%                   respective algorithm .m-file for a description of the
%                   relevant fields.
% group_settings     (Struct) contains plot settings for the program. Has
%                   the following fields which are used here:
%   savefigs        (Boolean) 1 = save figures to PDF. 0 = don't save.
%   figcount        (Integer) Cumulative figure count before this function
%                   has been called.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% figcount          (Integer) Cumulative figure count after all plots in
%                   this function have been created.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Unpack plot settings
savefigs = group_settings.savefigs;
if savefigs
    relpath = group_settings.relpath;
    relpath_alg = [relpath 'DIRL\'];
    mkdir(relpath_alg);
    relpath_data = [relpath_alg 'data\'];
    mkdir(relpath_data);
end

% Extract system and system plot settings
% sys = master_settings.sys;
sys_plot_settings = master_settings.sys_plot_settings;

% Initialize figure counter
figcount = group_settings.figcount;

% Master plot formatting settings
psett_master = master_settings.psett_master;

% Properties of output variables
y_propts_cell = sys_plot_settings.y_propts_cell;

% Extract plot time window
tsim_plot = group_settings.tsim_plot;

% Save data filename
filename_data = group_settings.filename_data_dirl;

% Is DIRL (=1) or old IRL (=0)
notirlold = group_settings.notirlold;

% DIRL data
if notirlold
    dirl_data = out_data.dirl_data;
end

% Extract indices
inds = out_data.inds;

% Check for integral augmentation
hasintaug = master_settings.hasintaug;

% Loop data
loop_cell = out_data.loop_cell;

% *************************************************************************
% *************************************************************************
%
% UNPACK ALGORITHM OUTPUT DATA
%
% See respective algorithm .m-file for details of 'out_data' struct fields.
%
% *************************************************************************
% *************************************************************************


cmat_cell = out_data.cmat_cell;


% Conditioning data
cond_A_vec_cell = out_data.cond_A_vec_cell;

% Loop data
loopdata = out_data.loopdata;
numloops = loopdata.numloops;
doloopvec = loopdata.doloopvec;
istarvec = loopdata.istarvec;

% Maximum i^{*} among the active loops
maxistar =  loopdata.maxistar;

% Number of loops executed
numdoloops = sum(doloopvec);

% Learning trajectory data
tlvec = out_data.tlvec; 
ulmat = out_data.ulmat;
xlmat = out_data.xlmat;

% % LQ data
% lq_data = out_data.lq_data;

% Figure count at beginning
figcount_0 = figcount;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Legend for loop entries
lgd_loop = cell(numdoloops,1);

% Active loop counter
doloopcnt = 0;

for j = 1:numloops

if doloopvec(j)

% Increment counter
doloopcnt = doloopcnt + 1;


% Determine legend entry
switch numloops
    case 1
        currlgdentry = '';
    case 2
        currlgdentry = ['$j =' num2str(j) '$ $('...
            y_propts_cell{j}.texname ')$'];
        currtexname = ['$' y_propts_cell{j}.texname '$'];
end

% Add this loop to the master legend
lgd_loop{doloopcnt} = currlgdentry;

% Extract current loop weight data
c_mat = cmat_cell{j};

% Extract current loop conditioning data
cond_A_vec = cond_A_vec_cell{j};

% Number of activation functions
Nk = size(c_mat,2);


% *************************************************************************
% *************************************************************************
%
% PLOT: CRITIC NN PARAMETERS
%
% *************************************************************************
% *************************************************************************

% String v(P_{i,k})
switch numloops
    case 1
        strvPiknd = ['v(P_{i})'];
    case 2
        strvPiknd = ['v(P_{i,' num2str(j) '})'];
end
strvPik = ['$' strvPiknd '$'];

% Prepare legend entries
lgd_c = cell(Nk,1);
for i = 1:Nk
    lgd_c{i} = ['$' strvPiknd '_{' num2str(i) '}$'];
end

% PLOT
figure(figcount)
h_fig = plot(0:istarvec(j)-1, c_mat); 
set(h_fig, 'Marker', 'o');
switch numloops
    case 1
        title(['Weights ' strvPik]); 
    case 2
        title(['Weights ' strvPik ' -- ' currtexname]); 
end
xlabel('Iteration $i$'); 
ylabel(strvPik);
% If there aren't too many weights, include legend
if Nk <= 10
    if Nk <= 6
        lgd = legend(lgd_c);
    else
        lgd = legend(lgd_c, 'NumColumns', 2);
    end
end
% set(lgd, 'Numcolumns', 2);          % Make legend 2 columns
% set(lgd, 'Location', 'Best');       % Put legend in empty spot

% x ticks
xticks(0:istarvec(j)-1)  
xlim([0 istarvec(j)-1]);

% Format plot
p_sett.figcount = figcount;
plot_format(p_sett); 

% SAVE PLOT
if savefigs
    filename = ['vPi' num2str(j)];
    savepdf(figcount, relpath_alg, filename); 
end

% Increment figure counter
figcount = figcount + 1;

end

end

%%
% *************************************************************************
% *************************************************************************
%
% SAVE DIRL LEARNING DATA
%
% *************************************************************************
% *************************************************************************

% Save data
if savefigs && notirlold

    % Save data 
    varname = 'dirl_data';
    save([relpath_data filename_data], varname);

end
    



