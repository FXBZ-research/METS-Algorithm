function[isSuccess,nbMoves,searchCompleted,yu,yv,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
    m7(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay)
% Neighborhood operator 7: 2opt (u,x)(v,y)<->(u,v)(x,y)
routeV = Node_related(nodeV,3);
routeU = Node_related(nodeU,3);
yu = routeU;
yv = routeV;
if (routeV ~= routeU)
    isSuccess = false;
    return
end
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
% Node_related = [predecessors,successor,routeID,node_location]
predecessors = Node_related(:,1);
successor = Node_related(:,2);
routeID = Node_related(:,3);
node_location = Node_related(:,4)';
% Retrieve information of nodeU: V, X, and Y
nodeU_loc = node_location(nodeU);
routeU = routeID(nodeU);
preU = predecessors(nodeU);
nodeX = successor(nodeU);
if nodeX
    suX = successor(nodeX);
    nodeX_loc = node_location(nodeX);
end

nodeV_loc = node_location(nodeV);
routeV = routeID(nodeV);
nodeY = successor(nodeV);
preV = predecessors(nodeV);
if nodeY
    suY = successor(nodeY);
    nodeY_loc = node_location(nodeY);
end
yu = routeU;
yv = routeV;
a=nodeU;
for i=1:sum(routeID == routeU)
    if a==0
        break
    end
    a=predecessors(a);
    if a==nodeV
        isSuccess = false;
        return
    end
end
if nodeX == nodeV
    isSuccess = false;
    return
end
xxvv = [];
xxvv(1)=nodeU;
for i=2:sum(routeID == routeU)
    if (xxvv(i-1)==nodeV)||(xxvv(i-1)==0)
        break
    end
    xxvv(i)=successor(xxvv(i-1));
end
xxvv(xxvv==0)=[];
xxvv(1)=[];
if max(node_location(xxvv)) ~= 100
    % Cost calculation
    costOne = - dAll(nodeU+1,nodeX+1) - dAll(nodeV+1,nodeY+1);
    costTwo = + dAll(nodeU+1,nodeV+1) + dAll(nodeX+1,nodeY+1);
    if nodeX_loc == -1
        distance_pre(routeU) = distance_pre(routeU) + costOne + costTwo;
        time_afs(routeU) = time_afs(routeU) + (costOne + costTwo)/speed;
    elseif nodeU_loc == 1
        distance_su(routeU) = distance_su(routeU) + costOne + costTwo;
        time_su(routeU) = time_su(routeU) + (costOne + costTwo)/speed;
    end
    pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
    pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
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
    successor(nodeU) =  nodeV;
    if nodeY,predecessors(nodeY)  = nodeX;end
    predecessors(xxvv(numel(xxvv))) = nodeU;
    for i=1:numel(xxvv)-1
        predecessors(xxvv(i)) = xxvv(i+1);
    end
    successor(xxvv(1)) = nodeY;
    for i=2:numel(xxvv)
        successor(xxvv(i)) = xxvv(i-1);
    end
elseif max(node_location(xxvv)) == 100
    % Cost calculation
    costOne = - dAll(nodeU+1,nodeX+1) - dAll(nodeV+1,nodeY+1);
    costTwo = + dAll(nodeU+1,nodeV+1) + dAll(nodeX+1,nodeY+1);
    routeU_NOW= [];
    routeU_NOW(1)=nodeV;
    for i=2:sum(routeID == routeU)
        if (routeU_NOW(i-1)==0)
            break
        end
        routeU_NOW(i)=successor(routeU_NOW(i-1));
    end
    routeU_NOW(routeU_NOW==0)=[];
    routeU_NOW(1) =[];
    routeU_now =[];
    routeU_now(1)=nodeV;
    for i=2:sum(routeID == routeU)
        if (routeU_now(i-1)==0)
            break
        end
        routeU_now(i)=predecessors(routeU_now(i-1));
    end
    routeU_now(routeU_now==0)=[];
    routeU_now = fliplr(routeU_now);
    routeU_now = [routeU_now routeU_NOW];
    routeU_now(find(routeU_now==nodeX):find(routeU_now==nodeV)) =  fliplr(routeU_now(find(routeU_now==nodeX):find(routeU_now==nodeV)));
    [d1,d2,t1,t2] = get_pd_pt(vrp,routeU_now,everTime,speed);
    distance_pre(routeU) = d1;
    distance_su(routeU) = d2;
    pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
    pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
    pd_now = wd*sum(sum(pd_v));
    time_afs(routeU) = t1;
    time_su(routeU) = t2;
    pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - vrp.T_max_V,0) ;
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
    successor(nodeU) =  nodeV;
    if nodeY,predecessors(nodeY)  = nodeX;end
    predecessors(xxvv(numel(xxvv))) = nodeU;
    for i=1:numel(xxvv)-1
        predecessors(xxvv(i)) = xxvv(i+1);
    end
    successor(xxvv(1)) = nodeY;
    for i=2:numel(xxvv)
        successor(xxvv(i)) = xxvv(i-1);
    end
    node_location(xxvv(find(xxvv == max(routeU_now)):end)) = -1;
    node_location(xxvv(1:find(xxvv == max(routeU_now)))) = 1;
    node_location(max(routeU_now)) = 100;

end

%% Update related matrix information
% Update route-related matrices
distance_pre_su = [distance_pre,distance_su];
Route_related = [pd_v,pt_v,time_afs,time_v,distance_pre_su];
% Update node-related matrices
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
