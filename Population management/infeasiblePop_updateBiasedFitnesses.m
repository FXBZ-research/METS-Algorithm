function infeasiblePop = infeasiblePop_updateBiasedFitnesses(infeasiblePop, vrp, par_hgs)

% For identical solutions, the newer one with the larger index is placed first.
infeasiblePop.ID = -infeasiblePop.ID;
infeasiblePop = sortrows(infeasiblePop, {'cost_Total', 'ID'});
infeasiblePop.ID = -infeasiblePop.ID;

infeasiblePop.fitRank = (1:numel(infeasiblePop.ID))';
nbPop = numel(infeasiblePop.ID);

% Calculate the pair distance matrix.
brokenPairDistance = zeros(nbPop);
for pp = 1:nbPop-1
    for qq = pp+1:nbPop
        brokenPairDistance(pp, qq) = calculateBrokenPairDistance(infeasiblePop, pp, qq, vrp);
        brokenPairDistance(qq, pp) = brokenPairDistance(pp, qq);
    end
end

% Calculate avgBrokenDist and Fitness.
if any(brokenPairDistance(:))
    for c = 1:nbPop
        broken_a = brokenPairDistance(c, :);
        broken_a(c) = [];
        infeasiblePop.brokenDist{c} = broken_a;
        maxSize = min(par_hgs.nClosest, nbPop-1);
        infeasiblePop.avgBrokenDist(c) = -mean(mink(infeasiblePop.brokenDist{c}, maxSize));
    end

    infeasiblePop = sortrows(infeasiblePop, 'avgBrokenDist', 'ascend');
    infeasiblePop.divRank = linspace(0, 1, nbPop)';

    infeasiblePop.fitRank = (infeasiblePop.fitRank-1)./(numel(infeasiblePop.fitRank)-1);
    for ff = 1:nbPop
        infeasiblePop.Fitness(ff) = infeasiblePop.fitRank(ff) + (1-par_hgs.eliteNum/nbPop)*infeasiblePop.divRank(ff);
    end

    
    infeasiblePop.ID = -infeasiblePop.ID;
    infeasiblePop = sortrows(infeasiblePop, {'Fitness', 'ID'});
    infeasiblePop.ID = -infeasiblePop.ID;
end
end



function distance = calculateBrokenPairDistance(infeasiblePop, pp, qq, vrp)
successors = infeasiblePop.successor{pp};
predecessors = infeasiblePop.predecessors{pp};
successors2 = infeasiblePop.successor{qq};
predecessors2 = infeasiblePop.predecessors{qq};
a=min(numel(successors),numel(successors2));
            successors = successors(1:a);
            successors2 = successors2(1:a);
            predecessors = predecessors(1:a);
            predecessors2 = predecessors2(1:a);
            successors(successors > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors(predecessors > vrp.nb_customer) = vrp.nb_customer+1;
            successors2(successors2 > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors2(predecessors2 > vrp.nb_customer) = vrp.nb_customer+1;
            cc = zeros(size(successors));
            dd = zeros(size(successors));
for jj = 1:a
    if successors(jj) ~= successors2(jj) && successors(jj) ~= predecessors2(jj)
        cc(jj) = 1;
    end
    if predecessors(jj) == 0 && predecessors2(jj) ~= 0 && successors2(jj) ~= 0
        dd(jj) = 1;
    end
end
distance = sum([sum(cc),sum(dd)])/vrp.last_customer;
end