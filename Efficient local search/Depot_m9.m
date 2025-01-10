function [isSuccess,nbMoves,searchCompleted,isdelete,yu,yv,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
    Depot_m9(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay)
% Neighborhood operator 9 (routeV(1))
isdelete = 0;
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
if (routeV == routeU)
    isSuccess = false;
    return
end
d_xx = 0;
xx(1)=nodeU;
for i=2:sum(routeID == routeU)
    if xx(i-1)==0
        break
    end
    xx(i)=successor(xx(i-1));
    if xx(i) ~= 0 && i ~= 2
        d_xx=d_xx+dAll(xx(i)+1,xx(i-1)+1);
    end
end
xx(xx==0)=[];
xx(1)=[];
if numel(xx) == 1 && node_location(xx) == 100
    isSuccess = false;
    return
end
yy(1)=nodeV;
d_yy = 0;
for i=2:sum(routeID == routeV)
    if yy(i-1)==0
        break
    end
    yy(i)=successor(yy(i-1));
    if yy(i) ~= 0
        d_yy=d_yy+dAll(yy(i)+1,yy(i-1)+1);
    end
end
yy(yy==0)=[];
routeV_now = yy;
if (numel(yy) == 0)
    isSuccess = false;
    return
end
if (numel(xx) == 0)
    isSuccess = false;
    return
end

if (numel(xx) ~= 0)
    if (max(node_location(xx))==100) && (max(node_location(yy))==100)
        if numel(xx)==1 && node_location(xx)==100
            isdelete=1;
        end
        % Cost calculation
        costOne = - d_xx - dAll(nodeU+1,nodeX+1) - dAll(xx(end)+1,1) + d_yy + dAll(nodeU+1,nodeV+1) + dAll(yy(end)+1,1);
        costTwo = - d_yy - dAll(1,nodeV+1) - dAll(yy(end)+1,1) + d_xx + dAll(1,xx(end)+1) + dAll(nodeX+1,1);
        d1u = distance_pre(routeU);
        d2u = distance_su(routeU);
        d1v = distance_pre(routeV);
        d2v = distance_su(routeV);
        t1u = time_afs(routeU);
        t2u = time_su(routeU);
        t1v = time_afs(routeV);
        t2v = time_su(routeV);
        distance_su(routeU) = d2v;
        distance_pre(routeV) = d2u;
        distance_su(routeV) = d_xx + dAll(xx(end)+1,1) - d2u + dAll(nodeX+1,1);
        distance_pre(routeU) = d1u + d2u + costOne - distance_su(routeU);
        time_su(routeU) = t2v;
        time_afs(routeV) = t2u - everTime;
        time_v(routeU) = (d1u + d2u + costOne)/speed + (numel(routeID(routeID == routeU))*everTime) + ((numel(yy)-numel(xx))*everTime);
        time_v(routeV) = (d1v + d2v + costTwo)/speed + (numel(routeID(routeID == routeV))*everTime) - ((numel(yy)-numel(xx))*everTime);
        time_afs(routeU) = time_v(routeU) - time_su(routeU);
        time_su(routeV) = time_v(routeV) - time_afs(routeV);
        pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
        pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
        pd_v(routeV,1) = max(distance_pre(routeV) - V_Dmax,0);
        pd_v(routeV,2) = max(distance_su(routeV) - V_Dmax,0);
        pd_now = wd*sum(sum(pd_v));
        pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - vrp.T_max_V,0) ;
        pt_v(routeV) = max(time_afs(routeV) + time_su(routeV) - vrp.T_max_V,0) ;
        pt_now = wt*sum(pt_v);
        time_v = time_afs + time_su;
        [pc_now,afs_time_delay] = AFSdelay_new(time_afs(1:numel(node_location)-nbClients),time_v(1:numel(node_location)-nbClients),T_max,T_Afs,C_Afs,afs_time_delay);
        pc_now = pc_now*wc;
        pm_now = pm;
        if isdelete == 1
            delete_idx = routeV;
            [pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm] = ...
                deleteAFS_delta(delete_idx,node_location,pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm,wm);
            [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
            pc_now = pc_now*wc;
            pt_now = wt*sum(pt_v);
            pd_now = wd*sum(sum(pd_v));
        end
        % Calculate delta
        if isdelete == 1
            delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd - 2*dAll(1,nbClients+2);
        else
            delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd;
        end
        if  delta > -0.000001
            isSuccess = false;
            return
        end
        % Predecessor and successor routeID
        successor(nodeU) =  nodeV;
        if nodeX, predecessors(xx(numel(xx))) = 0;
            for i=1:numel(xx)-1
                predecessors(xx(i)) = xx(i+1);
            end
        end
        if nodeX,successor(nodeX) = 0;
            for i=2:numel(xx)
                successor(xx(i)) = xx(i-1);
            end
        end
        predecessors(nodeV) = nodeU;
        routeID(xx) = routeV;
        routeID(yy) = routeU;
        node_location(xx) = - node_location(xx);
        node_location(node_location==-100) = 100;
        xxafs = max(xx);
        vvafs = max(yy);
        routeID(xxafs) = routeU;
        routeID(vvafs) = routeV;
        pre_xxafs = predecessors(xxafs);
        su_xxafs = successor(xxafs);
        pre_vvafs = predecessors(vvafs);
        su_vvafs = successor(vvafs);
        if pre_xxafs,successor(pre_xxafs) = vvafs;end
        if su_xxafs,predecessors(su_xxafs) = vvafs;end
        if pre_vvafs,successor(pre_vvafs) = xxafs;end
        if su_vvafs, predecessors(su_vvafs) = xxafs;end
        successor(xxafs) = su_vvafs;
        predecessors(xxafs) = pre_vvafs;
        successor(vvafs) = su_xxafs;
        predecessors(vvafs) = pre_xxafs;
        if isdelete == 1
            [routeID,node_location,predecessors,successor] = deleteAFS_node(routeID,delete_idx,node_location,predecessors,successor);
        end
    elseif (max(node_location(xx))~=100) && (max(node_location(yy))~=100)
        % Cost calculation
        costOne = - d_xx - dAll(nodeU+1,nodeX+1) - dAll(xx(end)+1,1) + d_yy + dAll(nodeU+1,nodeV+1) + dAll(yy(end)+1,1);
        costTwo = - d_yy - dAll(1,nodeV+1) - dAll(yy(end)+1,1) + d_xx + dAll(1,xx(end)+1) + dAll(nodeX+1,1);
        if nodeU_loc == -1
            distance_pre(routeU) = distance_pre(routeU) + costOne;
            time_afs(routeU) = time_afs(routeU) + costOne/speed + (numel(yy)- numel(xx))*everTime;
        elseif nodeU_loc == 1
            distance_su(routeU) = distance_su(routeU) + costOne;
            time_su(routeU) = time_su(routeU) + costOne/speed + (numel(yy)- numel(xx))*everTime;
        end
        if  nodeV_loc == -1
            distance_pre(routeV) = distance_pre(routeV) + costTwo;
            time_afs(routeV) = time_afs(routeV) + costTwo/speed + ( - numel(yy) + numel(xx))*everTime;
        elseif nodeV_loc == 1
            distance_su(routeV) = distance_su(routeV) + costTwo;
            time_su(routeV) = time_su(routeV) + costTwo/speed + ( - numel(yy) + numel(xx))*everTime;
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
        [pc_now,afs_time_delay] = AFSdelay_new(time_afs(1:numel(node_location)-nbClients),time_v(1:numel(node_location)-nbClients),T_max,T_Afs,C_Afs,afs_time_delay);
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
        if nodeX, predecessors(xx(numel(xx))) = 0;
            for i=1:numel(xx)-1
                predecessors(xx(i)) = xx(i+1);
            end
        end
        if nodeX,successor(nodeX) = 0;
            for i=2:numel(xx)
                successor(xx(i)) = xx(i-1);
            end
        end
        predecessors(nodeV) = nodeU;
        routeID(xx) = routeV;
        routeID(yy) = routeU;
        node_location(yy) = nodeU_loc;
        node_location(xx) = -1;
        node_location(node_location==-100) = 100;
    else
        isSuccess = false;
        return
    end
end
%% Update related matrix information
% Update node-related matrices
node_location = node_location';
Node_related = [predecessors,successor,routeID,node_location];
% Update route-related matrices
distance_pre_su = [distance_pre,distance_su];
Route_related = [pd_v,pt_v,time_afs,time_v,distance_pre_su];
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



