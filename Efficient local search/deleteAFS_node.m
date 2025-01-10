function [routeID,node_location,predecessors,successor] = deleteAFS_node(routeID,delete_idx,node_location,predecessors,successor)
% deleteAFS function: This is triggered only when, after u or ux leaves delete_idx, 
% the route delete_idx contains only the AFS. 
% It handles the deletion of this route and the corresponding changes brought about by it.
% Sort the input according to the order of use.
node_deleteAfs = delete_idx + sum(node_location~=100);
if numel(node_deleteAfs) ~= 1
    disp deleteAFS_node
end
routeID(routeID > delete_idx) = routeID(routeID > delete_idx) - 1;
predecessors(predecessors > node_deleteAfs) = predecessors(predecessors > node_deleteAfs) - 1;
successor(successor > node_deleteAfs) = successor(successor > node_deleteAfs) - 1;
node_location(node_deleteAfs) = [];
predecessors(node_deleteAfs) = [];
successor(node_deleteAfs) = [];
routeID(node_deleteAfs) = [];
end