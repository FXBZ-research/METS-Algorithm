function [feasiblePop,infeasiblePop,vrp] = add2Pop(sol_table,par_hgs,tspid,feasiblePop,infeasiblePop,vrp)
% Place the generated solutions into different populations, calculate their fitness, and manage the populations.
if sol_table.IsFeasible == 1     % Place into the feasible population.
    if numel(feasiblePop) == 0
        feasiblePop(1,:)=sol_table;
        feasiblePop.Fitness(1) = 1;
        feasiblePop.fitRank(1) = 1;
        % Store the diversity-related double values here as well. 
        % The +1 accounts for the additional capacity needed when the size exceeds the limit.
        feasiblePop.brokenPairDistance{1}=zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1); 
        feasiblePop.avgBrokenDist = 0;
        feasiblePop.divRank = 0;
        vrp.ALL_brokenDIS_feasible = zeros(1,par_hgs.popSizeLambda+par_hgs.popSizeMu+1);

    else    
        ALL_brokenDIS_feasible = vrp.ALL_brokenDIS_feasible(1:numel(feasiblePop(:,1)),:); % Extract the subset of ALL based on the subpopulation size to avoid excess after population reduction.
        feasiblePop(numel(feasiblePop(:,1))+1,1:numel(sol_table)) = sol_table; % Place the new solution at the last position of the subpopulation. If all other solutions are better, the process ends without further iterations.
        feasiblePop.fitRank(end) = 99; % Assign its cost rank directly.
        new_solnum = numel(feasiblePop(:,1));
        ALL_brokenDIS_feasible(new_solnum,:) = zeros;% Add this row to the matrix table ALL_brokenDIS_feasible as well

        % If the new solution is better than all existing solutions,
        % no iteration is needed, and it can be directly placed at the end.
        % Otherwise, iteration is required.
        if sol_table.cost_Total < feasiblePop.cost_Total(end-1)

            % Compare the cost of the new solution with the costs of the subpopulation.
            % Determine its position accordingly.
            % All solutions in the subpopulation must be sorted in ascending order of distance.
            for ii = 1:numel(feasiblePop.ID) 

                % When the cost of the new solution is less than that of solution ii,
                % it indicates that the new solution should occupy this position.
                % Shift all solutions from ii:end one position backward.
                if sol_table.cost_Total <= feasiblePop.cost_Total(ii)
                    sol_table.fitRank = ii;% Assign the new solution a cost, fitness rank, and brokenPairDistance to ensure column consistency.
                    sol_table.Fitness(1) = 1;
                    sol_table.brokenPairDistance{1} = zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1);
                    sol_table.avgBrokenDist = 0;
                    sol_table.divRank = 0;
                    feasiblePop(end,:) = [];
                    feasiblePop(ii+1:end+1,:) = feasiblePop(ii:end,:);
                    feasiblePop(ii,:) = sol_table;
                    feasiblePop.fitRank(ii+1:end) = 1; % Update the ranks of solutions after the current position.
                    new_solnum = ii; % Record the position of the new solution for future use.
                    % Similarly, update ALL_brokenDIS: first remove the last row, then shift the remaining rows.
                    ALL_brokenDIS_feasible(end,:) = [];
                    ALL_brokenDIS_feasible(new_solnum+1:end+1,:) = ALL_brokenDIS_feasible(new_solnum:end,:);
                    % The purpose of introducing ALL_brokenDIS is to optimize the following process.
                    % The brokenPairDistance of old solutions is updated directly in the matrix,
                    % avoiding individual updates which are very slow.
                    ALL_brokenDIS_feasible(:,new_solnum+1:end+1) =  ALL_brokenDIS_feasible(:,new_solnum:end);
                    ALL_brokenDIS_feasible(:,new_solnum) = 0;
                    ALL_brokenDIS_feasible(:,end) = [];
                    % At this point, the update of brokenPairDistance is complete.
                    % Only the brokenPairDistance of the new solution needs to be calculated.
                    break
                end
            end
        else
            % Skip initialization since it is already the maximum, but ensure it is initialized.
            % Add this row to the matrix table ALL_brokenDIS_feasible, as it has already been added above.
            feasiblePop.brokenPairDistance{end} = zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1);
        end
        feasiblePop.fitRank(1:end) = 1:numel(feasiblePop.ID);
        feasiblePop.fitRank(1:end) = (feasiblePop.fitRank-1)./(numel(feasiblePop.fitRank)-1); % get fitRank
        successors = sol_table.successor{1};
        predecessors = sol_table.predecessors{1};
        for ii=1:numel(feasiblePop.ID) % The distance between itself and itself does not need to be calculated.
            if ii == new_solnum %skip
                continue
            end
            %% Calculate brokenPairDistance
            % Using linked lists (su, pre) for acceleration may cause asymmetry in the distances between sol1 and sol2.
            % To maintain consistency with the original version, place the smaller distance first.
            if ii < new_solnum
                successors_b = successors;
                predecessors_b = predecessors;
                successors_a = feasiblePop.successor{ii};
                predecessors_a = feasiblePop.predecessors{ii};
            elseif ii > new_solnum
                successors_a = successors;
                predecessors_a = predecessors;
                successors_b = feasiblePop.successor{ii};
                predecessors_b = feasiblePop.predecessors{ii};
            end
            % Differences in the routes of the two solutions may lead to differences in AFS and, consequently, the number of points.
            % Therefore, the extra points need to be removed.
            a=min(numel(successors_a),numel(successors_b));
            successors_a = successors_a(1:a);
            successors_b = successors_b(1:a);
            predecessors_a = predecessors_a(1:a);
            predecessors_b = predecessors_b(1:a);
            % Ensure consistent encoding for all AFS.
            successors_a(successors_a > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors_a(predecessors_a > vrp.nb_customer) = vrp.nb_customer+1;
            successors_b(successors_b > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors_b(predecessors_b > vrp.nb_customer) = vrp.nb_customer+1;
            cc = zeros(size(successors_a));
            dd = zeros(size(successors_a));
            for jj=1:a
                % 1) Extra points: If point jj's successor in sol1 ~= jj's successor in sol2
                %    and jj's successor in sol1 ~= jj's predecessor in sol2,
                %    then point jj must have a discrepancy.
                if successors_a(jj)~=successors_b(jj) && successors_a(jj) ~= predecessors_b(jj)
                    cc(jj) = 1;
                end
                % 2) Missing points: If point jj's predecessor in sol1 is the Depot,
                %    and jj's predecessor in sol2 is not the Depot,
                %    and jj's successor in sol2 is also not the Depot,
                %    then point jj must have a discrepancy.
                if predecessors_a(jj)==0 && predecessors_b(jj)~=0 && successors_b(jj)~=0
                    dd(jj) = 1;
                end
            end
            distance = sum([sum(cc),sum(dd)])/vrp.last_customer;
            ALL_brokenDIS_feasible(new_solnum,ii) = distance;
            ALL_brokenDIS_feasible(ii,new_solnum) = distance;
        end
        vrp.ALL_brokenDIS_feasible = ALL_brokenDIS_feasible; % Save the updated table into vrp.
        maxSize = min(par_hgs.nClosest,numel(feasiblePop.ID)-1);         % Determine the number of top dis values to use for computation.
        ALL_brokenDIS_a_feasible = ALL_brokenDIS_feasible(1:numel(feasiblePop.ID),1:numel(feasiblePop.ID)); % Extract valid values from all.
        % Remove the self-distance.
        ALL_brokenDIS_b_feasible =zeros(numel(feasiblePop.ID),numel(feasiblePop.ID)-1);
        for deldel = 1:numel(feasiblePop.ID)
            ALL_brokenDIS_b_feasible(deldel,:) = [ALL_brokenDIS_a_feasible(deldel,1:deldel-1),ALL_brokenDIS_a_feasible(deldel,deldel+1:end)];
        end
        avgBrokenDist = -mean(mink(ALL_brokenDIS_b_feasible',maxSize));% Since mink returns the maxsize smallest values for each column,the matrix needs to be transposed.

        % When there are only two solutions, ALL_brokenDIS_b' is recognized as an array
        % and returns a constant, so it needs to be manually duplicated.
        if numel(avgBrokenDist) == 1
            avgBrokenDist(1:2) = avgBrokenDist;
        end
        for c=1:numel(feasiblePop.ID)
            feasiblePop.avgBrokenDist(c) = avgBrokenDist(c);
            feasiblePop.brokenPairDistance{c} = ALL_brokenDIS_a_feasible (c,:);
        end
        [~, sortedIndex] = sort(feasiblePop.avgBrokenDist);       % Obtain diversity ranking indices.
        feasiblePop.divRank(sortedIndex) = 1:numel(sortedIndex);  % Assign diversity rankings to the subpopulation.
        feasiblePop.divRank(1:end) = (feasiblePop.divRank-1)./(numel(feasiblePop.divRank)-1);
        feasiblePop.Fitness = feasiblePop.fitRank + (1 - par_hgs.eliteNum/numel(feasiblePop.ID)) * feasiblePop.divRank;
    end


elseif sol_table.IsFeasible == 0        % Place into the infeasible population.
    if numel(infeasiblePop) == 0
        infeasiblePop(1,:)=sol_table;
        infeasiblePop.Fitness(1) = 1;
        infeasiblePop.fitRank(1) = 1;
        infeasiblePop.brokenPairDistance{1}=zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1); 
        infeasiblePop.avgBrokenDist = 0;
        infeasiblePop.divRank = 0;
        vrp.ALL_brokenDIS = zeros(1,par_hgs.popSizeLambda+par_hgs.popSizeMu+1);
    else
        ALL_brokenDIS = vrp.ALL_brokenDIS(1:numel(infeasiblePop(:,1)),:);
        infeasiblePop(numel(infeasiblePop(:,1))+1,1:numel(sol_table))=sol_table;
        infeasiblePop.fitRank(end) = 99;
        new_solnum = numel(infeasiblePop(:,1));
        ALL_brokenDIS(new_solnum,:) = zeros;
        if sol_table.cost_Total < infeasiblePop.cost_Total(end-1) 
            for ii = 1:numel(infeasiblePop.ID)
                if sol_table.cost_Total <= infeasiblePop.cost_Total(ii) 
                    sol_table.fitRank = ii;
                    sol_table.Fitness(1) = 1;
                    sol_table.brokenPairDistance{1} = zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1);
                    sol_table.avgBrokenDist = 0;
                    sol_table.divRank = 0;
                    infeasiblePop(end,:) = [];
                    infeasiblePop(ii+1:end+1,:) = infeasiblePop(ii:end,:);
                    infeasiblePop(ii,:) = sol_table;
                    infeasiblePop.fitRank(ii+1:end) = 1;
                    new_solnum = ii; 
                    ALL_brokenDIS(end,:) = [];
                    ALL_brokenDIS(new_solnum+1:end+1,:) = ALL_brokenDIS(new_solnum:end,:);
                    ALL_brokenDIS(:,new_solnum+1:end+1) =  ALL_brokenDIS(:,new_solnum:end);
                    ALL_brokenDIS(:,new_solnum) = 0;
                    ALL_brokenDIS(:,end) = [];
                    break
                end
            end
        else
            infeasiblePop.brokenPairDistance{end} = zeros(par_hgs.popSizeLambda+par_hgs.popSizeMu+1,1);
        end
        infeasiblePop.fitRank(1:end) = 1:numel(infeasiblePop.ID);
        infeasiblePop.fitRank(1:end) = (infeasiblePop.fitRank-1)./(numel(infeasiblePop.fitRank)-1);
        successors = sol_table.successor{1};
        predecessors = sol_table.predecessors{1};

        for ii=1:numel(infeasiblePop.ID)
            if ii == new_solnum
                continue
            end
            if ii < new_solnum 
                successors_b = successors;
                predecessors_b = predecessors;
                successors_a = infeasiblePop.successor{ii};
                predecessors_a = infeasiblePop.predecessors{ii};
            elseif ii > new_solnum
                successors_a = successors;
                predecessors_a = predecessors;
                successors_b = infeasiblePop.successor{ii};
                predecessors_b = infeasiblePop.predecessors{ii};
            end
            a=min(numel(successors_a),numel(successors_b));
            successors_a = successors_a(1:a);
            successors_b = successors_b(1:a);
            predecessors_a = predecessors_a(1:a);
            predecessors_b = predecessors_b(1:a);
            successors_a(successors_a > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors_a(predecessors_a > vrp.nb_customer) = vrp.nb_customer+1;
            successors_b(successors_b > vrp.nb_customer) = vrp.nb_customer+1;
            predecessors_b(predecessors_b > vrp.nb_customer) = vrp.nb_customer+1;
            cc = zeros(size(successors_a));
            dd = zeros(size(successors_a));
            for jj=1:a
                if successors_a(jj)~=successors_b(jj) && successors_a(jj) ~= predecessors_b(jj)
                    cc(jj) = 1;
                end
                if predecessors_a(jj)==0 && predecessors_b(jj)~=0 && successors_b(jj)~=0
                    dd(jj) = 1;
                end
            end
            distance = sum([sum(cc),sum(dd)])/vrp.last_customer;
            ALL_brokenDIS(new_solnum,ii) = distance;
            ALL_brokenDIS(ii,new_solnum) = distance;
        end
        vrp.ALL_brokenDIS = ALL_brokenDIS;
        maxSize = min(par_hgs.nClosest,numel(infeasiblePop.ID)-1);
        ALL_brokenDIS_a = ALL_brokenDIS(1:numel(infeasiblePop.ID),1:numel(infeasiblePop.ID));
        ALL_brokenDIS_b =zeros(numel(infeasiblePop.ID),numel(infeasiblePop.ID)-1);
        for deldel = 1:numel(infeasiblePop.ID)
            ALL_brokenDIS_b(deldel,:) = [ALL_brokenDIS_a(deldel,1:deldel-1),ALL_brokenDIS_a(deldel,deldel+1:end)];
        end
        avgBrokenDist = -mean(mink(ALL_brokenDIS_b',maxSize));
        if numel(avgBrokenDist) == 1 
            avgBrokenDist(1:2) = avgBrokenDist;
        end
        for c=1:numel(infeasiblePop.ID)
            infeasiblePop.avgBrokenDist(c) = avgBrokenDist(c);
            infeasiblePop.brokenPairDistance{c} = ALL_brokenDIS_a (c,:);
        end
        [~, sortedIndex] = sort(infeasiblePop.avgBrokenDist); 
        infeasiblePop.divRank(sortedIndex) = 1:numel(sortedIndex); 
        infeasiblePop.divRank(1:end) = (infeasiblePop.divRank-1)./(numel(infeasiblePop.divRank)-1);
        infeasiblePop.Fitness = infeasiblePop.fitRank + (1 - par_hgs.eliteNum/numel(infeasiblePop.ID)) * infeasiblePop.divRank;
    end
end

end


