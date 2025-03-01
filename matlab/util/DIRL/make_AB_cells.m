function [A_cell, B_cell] = make_AB_cells(lin, setts)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAKE DIRL STATE-SPACE MATRICES FOR EACH LOOP
%
% [ ***** ANONYMIZED ***** ]
%
% 2023-03-29
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% "A", "B" IRL matrices -- nominal model
Airl = lin.io.Airl;
Birl = lin.io.Birl;

% Extract loop settings
dox3 = setts.dox3;
numloops = setts.numloops;
loop_cell = setts.loop_cell;
indsxirl = setts.inds.indsxirl;

if dox3
    A_cell = cell(numloops+1, numloops+1);
    B_cell = cell(numloops+1, numloops);
else
    A_cell = cell(numloops, numloops);
    B_cell = cell(numloops, numloops);
end

for k = 1:numloops
    indsxirlk = indsxirl{k};
    indsyk = loop_cell{k}.indsy;
    for j = 1:numloops
        indsxirlj = indsxirl{j};
        indsyj = loop_cell{j}.indsy;
        A_cell{k,j} = Airl(indsxirlk,indsxirlj);
        B_cell{k,j} = Birl(indsxirlk,indsyj);
    end
end

if dox3
    kx3 = numloops+1;
    indsxirlx3 = indsxirl{kx3};
    for j = 1:numloops
        indsxirlj = indsxirl{j};
        indsyj = loop_cell{j}.indsy;
        A_cell{kx3,j} = Airl(indsxirlx3,indsxirlj);
        A_cell{j,kx3} = Airl(indsxirlj,indsxirlx3);
        B_cell{kx3,j} = Birl(indsxirlx3,indsyj);
    end
    A_cell{kx3,kx3} = Airl(indsxirlx3,indsxirlx3);
end
