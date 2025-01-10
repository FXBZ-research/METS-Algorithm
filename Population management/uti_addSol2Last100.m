function Last100 = uti_addSol2Last100(Last100,individual,par_hgs)

if numel(Last100) == 0
    Last100 = individual;
else
    if numel(Last100.ID) < par_hgs.nbLast
        Last100(numel(Last100.ID)+1,:) = individual;
    else
        Last100(1:end-1,:) = Last100(2:end,:);
        Last100(end,:) = individual;
    end
end