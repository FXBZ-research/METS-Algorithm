function [predecessors,successor,routeID] = phrase_chromR(vrp,tspid,nbClients,chromR_move)
% Extract predecessors, successors, and route IDs from the route chromR: phrase_chromR

chromR = chromR_move;
a = 0;

% Calculate the total number of elements in all routes
for i=1:numel(chromR)
    a = a+numel(chromR{i});
end

% Initialize outputs
[predecessors,successor,routeID] = deal(zeros(a,1));

% Loop through each route
for j=1:length(chromR)
    if ~isempty(chromR{j})
        % Update successors and predecessors for consecutive indices
        for k=2:numel(chromR{j})
            predecessors(chromR{j}(k)) = chromR{j}(k-1);
            successor(chromR{j}(k-1)) = chromR{j}(k);
        end
        routeID(chromR{j}) = j;
    end
end


end