function [infeasiblePop] = update_infeasiblePop(infeasiblePop,nbClients,eliteNum,nClosest)
% Update the information of the infeasible population.
infeasiblePop = sortrows(infeasiblePop,'cost_Total','ascend');
infeasiblePop.fitRank=(1:numel(infeasiblePop.ID))';
infeasiblePop.fitRank(1:end) = (infeasiblePop.fitRank-1)./(numel(infeasiblePop.fitRank)-1);
nbPop = numel(infeasiblePop.ID);
infeasiblePop.brokenDist(:) = cell(1);
brokenPairDistance=zeros(nbPop-1,nbPop-1);
for pp=1:nbPop-1
    for qq=pp+1:nbPop
        % Calculate brokenPairDistance
        successors = infeasiblePop.successor{pp};
        predecessors = infeasiblePop.predecessors{pp};
        successors2 = infeasiblePop.successor{qq};
        predecessors2 = infeasiblePop.predecessors{qq};
        differences = 0;
        for jj=1:min(sum(infeasiblePop.node_location{pp}==100),sum(infeasiblePop.node_location{qq}==100))+nbClients
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
    for c=1:numel(infeasiblePop.ID)
        broken_a = brokenPairDistance(c,:);
        broken_a(c) = [];
        infeasiblePop.brokenDist{c}=broken_a;
        maxSize = min(nClosest,numel(infeasiblePop.ID)-1);
        infeasiblePop.avgBrokenDist(c) = -mean(mink(infeasiblePop.brokenDist{c},maxSize));
    end
    infeasiblePop = sortrows(infeasiblePop,'avgBrokenDist','ascend');
    for cc=1:numel(infeasiblePop.ID)
        infeasiblePop.divRank(cc) = cc;
    end
    infeasiblePop.divRank(1:end) = (infeasiblePop.divRank-1)./(numel(infeasiblePop.divRank)-1);
    for ff=1:numel(infeasiblePop.ID)
        infeasiblePop.Fitness(ff) = infeasiblePop.fitRank(ff) + (1-eliteNum/numel(infeasiblePop.ID))*infeasiblePop.divRank(ff);
    end
    infeasiblePop = sortrows(infeasiblePop,'Fitness','ascend');
end
end