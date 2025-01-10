function [isSuccess,nbMoves,searchCompleted,yu,yv,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
    m4(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay)
% Neighborhood operator 4: swap u and v
% Retrieve basic information: speed and service time
nbClients = vrp.nb_customer;
everTime = vrp.T_Customer;
speed = vrp.V_speed;
V_Dmax = vrp.V_Dmax;
T_max = vrp.T_max_V;
T_Afs = vrp.T_Afs;
C_Afs = vrp.C_Afs;
% Retrieve the penalty
wt = Penalty_all(1,1);
wc = Penalty_all(1,2);
wd = Penalty_all(1,3);
wm = Penalty_all(1,4);
pd_v = Route_related(:,[1,2]);
pt_v = Route_related(:,3);
pm = Penalty_all(2,4);
pt = Penalty_all(2,1);
pc = Penalty_all(2,2);
pd = Penalty_all(2,3);
% Retrieve the distance
dAll = vrp.distance_table;
distance_pre_su = Route_related(:,[6,7]);
distance_pre = distance_pre_su(:,1);
distance_su = distance_pre_su(:,2);
% Retrieve the time
time_v = Route_related(:,5);
time_afs = Route_related(:,4);
time_su = time_v - time_afs;
% Node_related = [predecessors,successor,routeID,node_location];
predecessors = Node_related(:,1);
successor = Node_related(:,2);
routeID = Node_related(:,3);
node_location = Node_related(:,4)';
% Retrieve information of nodeU: V, X, and Y
nodeU_loc = node_location(nodeU);
routeU = routeID(nodeU);
preU = predecessors(nodeU);
nodeX = successor(nodeU);

if nodeX,nodeX_loc = node_location(nodeX);end
nodeV_loc = node_location(nodeV);
routeV = routeID(nodeV);
nodeY = successor(nodeV);
preV = predecessors(nodeV);
yu = routeU;
yv = routeV;
if nodeY,nodeY_loc = node_location(nodeY);end

if (nodeU == preV)||(nodeU == nodeY) || (nodeU>nodeV)
    isSuccess = false;
    return
end
%% Cost calculation
costOne = - dAll(preU+1,nodeU+1) - dAll(nodeU+1,nodeX+1) + dAll(preU+1,nodeV+1) + dAll(nodeV+1,nodeX+1);
costTwo = - dAll(preV+1,nodeV+1) - dAll(nodeV+1,nodeY+1) + dAll(preV+1,nodeU+1) + dAll(nodeU+1,nodeY+1);
if nodeU_loc == -1
    distance_pre(routeU) = distance_pre(routeU) + costOne;
    time_afs(routeU) = time_afs(routeU) + costOne/speed;
elseif nodeU_loc == 1
    distance_su(routeU) = distance_su(routeU) + costOne;
    time_su(routeU) = time_su(routeU) + costOne/speed;
end
if  nodeV_loc == -1
    distance_pre(routeV) = distance_pre(routeV) + costTwo;
    time_afs(routeV) = time_afs(routeV) + costTwo/speed;
elseif nodeV_loc == 1
    distance_su(routeV) = distance_su(routeV) + costTwo;
    time_su(routeV) = time_su(routeV) + costTwo/speed;
end
pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
pd_v(routeV,1) = max(distance_pre(routeV) - V_Dmax,0);
pd_v(routeV,2) = max(distance_su(routeV) - V_Dmax,0);
pd_now = wd*sum(sum(pd_v));
pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - vrp.T_max_V,0) ;
pt_v(routeV) = max(time_afs(routeV) + time_su(routeV) - vrp.T_max_V,0) ;
pt_now = wt*sum(pt_v);
time_v = time_afs + time_su;
[pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
pc_now = pc_now*wc;
pm_now = pm;
% Calculate delta
delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd;
if  delta > -0.000001
    isSuccess = false;
    return
end
% Predecessor and successor routeID
if preU
    successor(preU) = nodeV;
end
predecessors(nodeU) = preV;
successor(nodeU) = nodeY;
if nodeX
    predecessors(nodeX) = nodeV;
end
if preV
    successor(preV) = nodeU;
end
predecessors(nodeV) = preU;
successor(nodeV) = nodeX;
if nodeY
    predecessors(nodeY) = nodeU;
end
routeID(nodeU) =routeV;
routeID(nodeV) =routeU;

%% Update related matrix information
% Update route-related matrices
distance_pre_su = [distance_pre,distance_su];
Route_related = [pd_v,pt_v,time_afs,time_v,distance_pre_su];
% Update node-related matrices
node_location(nodeU) = nodeV_loc;
node_location(nodeV) = nodeU_loc;
node_location = node_location';
Node_related = [predecessors,successor,routeID,node_location];
% Update penalty-related matrices
Penalty_all(2,4) = pm_now;
Penalty_all(2,1) = pt_now;
Penalty_all(2,2) = pc_now;
Penalty_all(2,3) = pd_now;
% Update nbMoves and set searchCompleted to false
nbMoves = nbMoves + 1;
searchCompleted = false;
% Update whenLastModified
whenLastModified(routeU) = nbMoves;
whenLastModified(routeV) = nbMoves;
% Update isSuccess to indicate success
isSuccess = true;
end


