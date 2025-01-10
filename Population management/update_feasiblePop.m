function [feasiblePop] = update_feasiblePop(feasiblePop,nbClients,eliteNum,nClosest)
% Update the information of the feasible population.
feasiblePop = sortrows(feasiblePop,'cost_Total','ascend');
feasiblePop.fitRank=(1:numel(feasiblePop.ID))';
feasiblePop.fitRank(1:end) = (feasiblePop.fitRank-1)./(numel(feasiblePop.fitRank)-1);
nbPop = numel(feasiblePop.ID);
feasiblePop.brokenDist(:) = cell(1);
brokenPairDistance=zeros(nbPop-1,nbPop-1);
for pp=1:nbPop-1
    for qq=pp+1:nbPop
        % Calculate brokenPairDistance
        successors = feasiblePop.successor{pp};
        predecessors = feasiblePop.predecessors{pp};
        successors2 = feasiblePop.successor{qq};
        predecessors2 = feasiblePop.predecessors{qq};
        differences = 0;
        for jj=1:min(sum(feasiblePop.node_location{pp}==100),sum(feasiblePop.node_location{qq}==100))+nbClients
            % 1) Extra points: If point jj's successor in sol1 ~= jj's successor in sol2
            %    and jj's successor in sol1 ~= jj's predecessor in sol2,
            %    then point jj must have a discrepancy.
            if successors(jj)~=successors2(jj) && successors(jj) ~= predecessors2(jj)
                differences=differences+1;
            end
            % 2) Missing points: If point jj's predecessor in sol1 is the Depot,
            %    and jj's predecessor in sol2 is not the Depot,
            %    and jj's successor in sol2 is also not the Depot,
            %    then point jj must have a discrepancy.
            if predecessors(jj)==0 && predecessors2(jj)~=0 && successors2(jj)~=0
                differences=differences+1;
            end
        end
        distance = differences/nbClients;
        % Symmetry: brokenPairDistance between two individuals is consistent.
        brokenPairDistance(pp,qq) = distance;
        brokenPairDistance(qq,pp) = distance;
    end
end
if numel(brokenPairDistance) == 0
else
    for c=1:numel(feasiblePop.ID)
        broken_a = brokenPairDistance(c,:);
        broken_a(c) = [];
        feasiblePop.brokenDist{c}=broken_a;
        maxSize = min(nClosest,numel(feasiblePop.ID)-1);
        feasiblePop.avgBrokenDist(c) = -mean(mink(feasiblePop.brokenDist{c},maxSize));
    end
    feasiblePop = sortrows(feasiblePop,'avgBrokenDist','ascend');
    for cc=1:numel(feasiblePop.ID)
        feasiblePop.divRank(cc) = cc;
    end
    feasiblePop.divRank(1:end) = (feasiblePop.divRank-1)./(numel(feasiblePop.divRank)-1);
    for ff=1:numel(feasiblePop.ID)
        feasiblePop.Fitness(ff) = feasiblePop.fitRank(ff) + (1-eliteNum/numel(feasiblePop.ID))*feasiblePop.divRank(ff);
    end
    feasiblePop = sortrows(feasiblePop,'Fitness','ascend');
end
end