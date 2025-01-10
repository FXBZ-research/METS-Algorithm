
function chromR = get_chromR(predecessors, successor,routeID,nb) 
% Obtain the actual routes from predecessors and successors 
chromR = cell(numel(unique(routeID)),1);
sort_candidate = 1:max(max(predecessors),max(successor));% Initialize the candidate points to be arranged
nn = numel(sort_candidate);   % To reduce computation in each iteration, obtain the number of candidate points
depot = sort_candidate((predecessors(sort_candidate) == 0));       
x = [];
for i=1:numel(depot)                             
    x(1,i) = depot(i);                                                    %#ok<AGROW>
    for ii=2:nn
        x(ii,i) = successor(x(ii-1,i));           % Use successors to determine the updated actual routes
        if x(ii,i)==0
            break
        end
    end
    xROUTE = x(:,i);
    xROUTE(xROUTE==0) = [];
    nn = nn-numel(xROUTE);                  
    chromR{routeID(depot(i))} = xROUTE;
end
chromR(cellfun(@isempty,chromR)) = [];
end
