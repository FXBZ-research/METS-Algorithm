function [feasiblePop,infeasiblePop,vrp] = PopManagement(sol_table,par_hgs,tspid,feasiblePop,infeasiblePop,vrp)
% Population management: Assign individuals to their respective populations. (add2Pop) 
% Remove duplicate solutions, Remove inferior solutions (when the population size exceeds the limit).
[feasiblePop,infeasiblePop,vrp] = add2Pop(sol_table,par_hgs,tspid,feasiblePop,infeasiblePop,vrp);
if sol_table.IsFeasible
    subpop = feasiblePop;
    brokenPairDistance = vrp.ALL_brokenDIS_feasible;
    b=1;
else
    subpop = infeasiblePop;
    brokenPairDistance = vrp.ALL_brokenDIS;
    b=0;
end

if numel(subpop.ID) > par_hgs.popSizeMu + par_hgs.popSizeLambda
    IsUpdateFitnees = 1;
    worstIndividualPosition=[];
    ind_del_line = 1;
    while numel(subpop.ID)> par_hgs.popSizeMu
        isWorstIndividualClone = false;               % isWorstIndividualClone: Indicates if the solution is a clone and has the worst fitness
        worstIndividualBiasedFitness = -999999999;    % The larger the worstIndividualBiasedFitness, the worse the solution.
        for i = 2 : numel(subpop.ID)
            % The distance Dist between solution i and its nearest other individual is 0, indicating it is a clone solution.
            % Using mink(,2) excludes the solution itself.
            isClone = sum(mink(brokenPairDistance(i,:),2)) < 0.00000001;  

            % Case 1: The subpopulation has no clone solutions: isClone == 0 and isWorstIndividualClone == 0.
            %          Only compare fitness (isClone == isWorstIndividualClone && subpop(i).Fitness > worstIndividualBiasedFitness).

            % Case 2: The subpopulation has clone solutions (first occurrence): isClone == 1 and isWorstIndividualClone == 0.
            %          Fitness is not considered since a clone solution is always the worst (isClone && ~isWorstIndividualClone).

            % Case 3: The subpopulation has clone solutions (N > 2 occurrences): isClone == 1 and isWorstIndividualClone == 1.
            %          Fitness is considered, and the clone with worse fitness is deemed inferior
            %          (isClone == isWorstIndividualClone && subpop(i).Fitness > worstIndividualBiasedFitness).

            if (isClone && ~isWorstIndividualClone ) || (isClone == isWorstIndividualClone && subpop.Fitness(i) > worstIndividualBiasedFitness)
                worstIndividualBiasedFitness = subpop.Fitness(i);
                isWorstIndividualClone = isClone;
                worstIndividualPosition = i;
            end
        end
        del_worstline(ind_del_line) = subpop.ID(worstIndividualPosition);
        subpop(worstIndividualPosition,:)=[];
        brokenPairDistance(worstIndividualPosition,:)=[];

        ind_del_line = ind_del_line + 1;
    end
    lien_ind = zeros(1,numel(del_worstline));
    for dein = 1:numel(del_worstline)

        if b == 1
            lien_ind(dein) = find(feasiblePop.ID==del_worstline(dein));
        else
            lien_ind(dein) = find(infeasiblePop.ID==del_worstline(dein));
        end
    end
    brokenPairDistance(:,lien_ind)=[];
    brokenPairDistance(:,end+1:end+numel(del_worstline)) = 0;

else
    IsUpdateFitnees = 0;
end
if b
    vrp.ALL_brokenDIS_feasible = brokenPairDistance;
else
    vrp.ALL_brokenDIS = brokenPairDistance;
end
if IsUpdateFitnees == 1 % After deletion, fitness should be recalculated.
    subpop.fitRank(1:end) = 1:numel(subpop.ID);
    subpop.fitRank(1:end) = (subpop.fitRank-1)./(numel(subpop.fitRank)-1);              %get fitRank
    maxSize = min(par_hgs.nClosest,numel(subpop.ID)-1);                                 % Determine how many top dis values to use for computation.
    brokenPairDistance = brokenPairDistance(1:numel(subpop.ID),1:numel(subpop.ID));     % Extract valid values from all.
    ALL_brokenDIS_del =zeros(numel(subpop.ID),numel(subpop.ID)-1);                      % Remove the self-distance.
    for deldel = 1:numel(subpop.ID)
        ALL_brokenDIS_del(deldel,:) = [brokenPairDistance(deldel,1:deldel-1),brokenPairDistance(deldel,deldel+1:end)];
        subpop.avgBrokenDist(deldel) = -mean(mink(ALL_brokenDIS_del(deldel,:)',maxSize));% Since mink returns the maxsize smallest values for each column, the matrix needs to be transposed.
    end
    [~, sortedIndex] = sort(subpop.avgBrokenDist);      
    subpop.divRank(sortedIndex) = 1:numel(sortedIndex);  % Assign diversity rankings to the subpopulation.
    subpop.divRank(1:end) = (subpop.divRank-1)./(numel(subpop.divRank)-1);
    subpop.Fitness = subpop.fitRank + (1 - par_hgs.eliteNum/numel(subpop.ID)) * subpop.divRank;
end

if b
    feasiblePop = subpop;
else
    infeasiblePop = subpop;
end

end
