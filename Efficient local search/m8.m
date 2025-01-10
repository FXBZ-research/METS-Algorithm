function[isSuccess,nbMoves,searchCompleted,isdelete,yu,yv,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
    m8(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay)
% Neighborhood operator 8: 2opt* (u,x)(v,y)<->(u,v) (x,y)
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
d_vv = 0;
vv = [];
vv(1)=nodeV;
for i=2:sum(routeID == routeV)
    if vv(i-1)==0
        break
    end
    vv(i)=predecessors(vv(i-1));
    if vv(i) ~= 0
        d_vv=d_vv+dAll(vv(i)+1,vv(i-1)+1);
    end
end
vv(vv==0)=[];

if numel(xx) == 0
    if (max(node_location(routeID == routeU)) ~= 100) && (max(node_location(vv)) ~= 100)
        isSuccess = false;
        return
    elseif (max(node_location(routeID == routeU)) == 100) && (max(node_location(vv)) == 100)
        isSuccess = false;
        return
    elseif (max(node_location(routeID == routeU)) == 100) && (max(node_location(vv)) ~= 100)
        if (nodeY && numel(vv) == numel(routeID(routeID==routeV)) - 1 && node_location(nodeY)==100)
            isdelete = 1;
        end
        % Cost calculation
        costOne = - d_vv - dAll(1,vv(end)+1) - dAll(nodeV+1,nodeY+1)  + dAll(1,nodeY+1);
        costTwo = + d_vv + dAll(nodeU+1,nodeV+1) + dAll(vv(end)+1,1) - dAll(nodeU+1,1);
        distance_pre(routeV) = distance_pre(routeV) + costOne;
        time_afs(routeV) = time_afs(routeV) + costOne/speed - numel(vv)*everTime;
        distance_su(routeU) = distance_su(routeU) + costTwo;
        time_su(routeU) = time_su(routeU) + costTwo/speed + numel(vv)*everTime;
        pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
        pd_v(routeV,1) = max(distance_pre(routeV) - V_Dmax,0);
        pd_now = wd*sum(sum(pd_v));
        pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - vrp.T_max_V,0) ;
        pt_v(routeV) = max(time_afs(routeV) + time_su(routeV) - vrp.T_max_V,0) ;
        pt_now = wt*sum(pt_v);
        time_v = time_afs + time_su;
        [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
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
        if nodeY,predecessors(nodeY)  = nodeX;end
        if nodeX,predecessors(xx(numel(xx))) = 0;
            for i=1:numel(xx)-1
                predecessors(xx(i)) = xx(i+1);
            end
        end
        if nodeX,successor(xx(1)) = nodeY;
            for i=2:numel(xx)
                successor(xx(i)) = xx(i-1);
            end
        end
        predecessors(vv(1)) = nodeU;
        for i=2:numel(vv)
            predecessors(vv(i)) = vv(i-1);
        end
        successor(vv(numel(vv))) = 0;
        for i=1:numel(vv)-1
            successor(vv(i)) = vv(i+1);
        end
        routeID(vv) = routeU;
        node_location(vv) = nodeU_loc;
        if isdelete == 1
            [routeID,node_location,predecessors,successor] = deleteAFS_node(routeID,delete_idx,node_location,predecessors,successor);
        end
    elseif (max(node_location(routeID == routeU)) ~= 100) && (max(node_location(vv)) == 100)
        isSuccess = false;
        return
    end
elseif numel(xx) ~= 0
    if (max(node_location(xx)) ~= 100) && (max(node_location(vv)) == 100)
        isSuccess = false;
        return
    elseif (max(node_location(xx)) == 100) && (max(node_location(vv)) ~= 100)
        isSuccess = false;
        return
    elseif (max(node_location(xx)) == 100) && (max(node_location(vv)) == 100)
        if nodeY == 0
            if numel(xx) == 1 &&  node_location(xx)==100
                isdelete = 1;
            end
        end
        % Cost calculation
        costOne = - d_vv - dAll(1,vv(end)+1) - dAll(nodeV+1,nodeY+1) + d_xx +dAll(nodeX+1,nodeY+1) + dAll(1,xx(end)+1);
        costTwo = + d_vv + dAll(nodeU+1,nodeV+1) + dAll(vv(end)+1,1) - d_xx - dAll(nodeU+1,nodeX+1) - dAll(xx(end)+1,1);
        d1u = distance_pre(routeU);
        d2u = distance_su(routeU);
        d1v = distance_pre(routeV);
        d2v = distance_su(routeV);
        t1u =time_afs(routeU);
        t2u = time_su(routeU);
        t1v =time_afs(routeV);
        t2v = time_su(routeV);
        distance_su(routeU) =  d1v;
        time_su(routeU) = t1v + everTime;
        distance_pre(routeV) = d2u;
        time_afs(routeV) = t2u - everTime;
        distance_pre(routeU) = d1u + d2u + costTwo - distance_su(routeU);
        distance_su(routeV) = d1v + d2v + costOne - distance_pre(routeV);
        time_v(routeU) = (distance_pre(routeU) + distance_su(routeU))/speed + (numel(routeID(routeID==routeU)) - numel(xx) + numel(vv))*everTime;
        time_v(routeV) = (distance_pre(routeV) + distance_su(routeV))/speed + (numel(routeID(routeID==routeV)) + numel(xx) - numel(vv))*everTime;
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
        if nodeY,predecessors(nodeY)  = nodeX;end
        if nodeX,predecessors(xx(numel(xx))) = 0;
            for i=1:numel(xx)-1
                predecessors(xx(i)) = xx(i+1);
            end
        end
        if nodeX,successor(xx(1)) = nodeY;
            for i=2:numel(xx)
                successor(xx(i)) = xx(i-1);
            end
        end
        predecessors(vv(1)) = nodeU;
        for i=2:numel(vv)
            predecessors(vv(i)) = vv(i-1);
        end
        successor(vv(numel(vv))) = 0;
        for i=1:numel(vv)-1
            successor(vv(i)) = vv(i+1);
        end
        routeID(vv) = routeU;
        routeID(xx) = routeV;
        node_location(vv) = -node_location(vv);
        node_location(xx) = -node_location(xx);
        node_location(node_location==-100) = 100;
        xxafs = max(xx);
        vvafs = max(vv);
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
    elseif (max(node_location(xx)) ~= 100) && (max(node_location(vv)) ~= 100)
        % Cost calculation
        costOne = - d_vv - dAll(1,vv(end)+1) - dAll(nodeV+1,nodeY+1) + d_xx +dAll(nodeX+1,nodeY+1) + dAll(1,xx(end)+1);
        costTwo = + d_vv + dAll(nodeU+1,nodeV+1) + dAll(vv(end)+1,1) - d_xx - dAll(nodeU+1,nodeX+1) - dAll(xx(end)+1,1);
        if nodeX_loc == -1
            distance_pre(routeU) = distance_pre(routeU) + costTwo;
            time_afs(routeU) = time_afs(routeU) + costTwo/speed + (numel(vv)-numel(xx))*everTime;
        elseif nodeU_loc == 1
            distance_su(routeU) = distance_su(routeU) + costTwo;
            time_su(routeU) = time_su(routeU) + costTwo/speed + (numel(vv)-numel(xx))*everTime;
        end
        if  nodeV_loc == -1
            distance_pre(routeV) = distance_pre(routeV) + costOne;
            time_afs(routeV) = time_afs(routeV) + costOne/speed +(numel(xx) - numel(vv))*everTime;
        elseif nodeV_loc == 1
            distance_su(routeV) = distance_su(routeV) + costOne;
            time_su(routeV) = time_su(routeV) + costOne/speed +(numel(xx) - numel(vv))*everTime;
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
        if isdelete == 1
            delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd - dAll(1,nbClients+2);
        else
            delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd;
        end
        if  delta > -0.000001
            isSuccess = false;
            return
        end
        % Predecessor and successor routeID
        successor(nodeU) =  nodeV;
        if nodeY,predecessors(nodeY)  = nodeX;end
        if nodeX,predecessors(xx(numel(xx))) = 0;
            for i=1:numel(xx)-1
                predecessors(xx(i)) = xx(i+1);
            end
        end
        if nodeX,successor(xx(1)) = nodeY;
            for i=2:numel(xx)
                successor(xx(i)) = xx(i-1);
            end
        end
        predecessors(vv(1)) = nodeU;
        for i=2:numel(vv)
            predecessors(vv(i)) = vv(i-1);
        end
        successor(vv(numel(vv))) = 0;
        for i=1:numel(vv)-1
            successor(vv(i)) = vv(i+1);
        end
        routeID(vv) = routeU;
        routeID(xx) = routeV;
        node_location(vv) = nodeU_loc;
        node_location(xx) = nodeV_loc;
    end
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
if isdelete == 1
    if delete_idx == routeV
        whenLastModified(routeU) = nbMoves;
        whenLastModified(routeV) = [];
    elseif delete_idx == routeU
        whenLastModified(routeU) = [];
        whenLastModified(routeV) = nbMoves;
    end
else
    whenLastModified(routeU) = nbMoves;
    whenLastModified(routeV) = nbMoves;
end
% Update isSuccess to indicate success
isSuccess = true;
end
