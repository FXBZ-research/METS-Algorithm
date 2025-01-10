
function [isSuccess,nbMoves,searchCompleted,isdelete,yu,yv,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
    m2(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay)
% Neighborhood operator 2: Insert u x after v
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
if nodeY,nodeY_loc = node_location(nodeY);end
yu = routeU;
yv = routeV;
if  (nodeU == nodeY) || (nodeV == nodeX)
    isSuccess = false;
    return
end
if  nodeX == 0
    isSuccess = false;
    return
end
if routeU == routeV
    if nodeX_loc ~= 100
        %% Cost calculation
        costOne = - dAll(nodeU+1,nodeX+1) - dAll(preU+1,nodeU+1)     - dAll(nodeX+1,suX+1)     + dAll(preU+1,suX+1);
        costTwo = + dAll(nodeU+1,nodeX+1) - dAll(nodeV+1,nodeY+1)    + dAll(nodeV+1,nodeU+1)   + dAll(nodeX+1,nodeY+1);
        if nodeU_loc == -1
            distance_pre(routeU) = distance_pre(routeU) + costOne;
            time_afs(routeU) = time_afs(routeU) + costOne/speed - 2*everTime;
        elseif nodeU_loc == 1
            distance_su(routeU) = distance_su(routeU) + costOne;
            time_su(routeU) = time_su(routeU) + costOne/speed - 2*everTime;
        end
        if  nodeV_loc == -1
            distance_pre(routeU) = distance_pre(routeU) + costTwo;
            time_afs(routeU) = time_afs(routeU) + costTwo/speed + 2*everTime;
        elseif nodeV_loc == 1
            distance_su(routeU) = distance_su(routeU) + costTwo;
            time_su(routeU) = time_su(routeU) + costTwo/speed + 2*everTime;
        end
        pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
        pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
        pd_now = wd*sum(sum(pd_v));
        pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - T_max,0) ;
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
            successor(preU) = suX;
        end
        predecessors(nodeU) = nodeV;
        if nodeX
            successor(nodeX) = nodeY;
        end
        if suX
            predecessors(suX) = preU;
        end
        successor(nodeV) = nodeU;
        if nodeY
            predecessors(nodeY) = nodeX;
        end
        node_location(nodeU) = nodeV_loc;
        node_location(nodeX) = nodeV_loc;
    elseif nodeX_loc == 100
        isSuccess = false;
        return
    end
else
    if  nodeX > nbClients
        isSuccess = false;
        return
    elseif nodeX <= nbClients
        if (sum(routeID==routeU) == 3 && max(node_location(routeID==routeU) == 100) )
            isdelete = 1;
        else
            isdelete = 0;
        end
        if max(node_location(routeID == routeV)) == 100
            % Cost calculation
            costOne = - dAll(nodeU+1,nodeX+1) - dAll(preU+1,nodeU+1)     - dAll(nodeX+1,suX+1)     + dAll(preU+1,suX+1);
            costTwo = + dAll(nodeU+1,nodeX+1) - dAll(nodeV+1,nodeY+1)    + dAll(nodeV+1,nodeU+1)   + dAll(nodeX+1,nodeY+1);
            if nodeU_loc == -1
                distance_pre(routeU) = distance_pre(routeU) + costOne;
                time_afs(routeU) = time_afs(routeU) + costOne/speed - 2*everTime;
            elseif nodeU_loc == 1
                distance_su(routeU) = distance_su(routeU) + costOne;
                time_su(routeU) = time_su(routeU) + costOne/speed - 2*everTime;
            end
            if  nodeV_loc == -1
                distance_pre(routeV) = distance_pre(routeV) + costTwo;
                time_afs(routeV) = time_afs(routeV) + costTwo/speed + 2*everTime;
            elseif nodeV_loc == 1
                distance_su(routeV) = distance_su(routeV) + costTwo;
                time_su(routeV) = time_su(routeV) + costTwo/speed + 2*everTime;
            end
            pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
            pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
            pd_v(routeV,1) = max(distance_pre(routeV) - V_Dmax,0);
            pd_v(routeV,2) = max(distance_su(routeV) - V_Dmax,0);
            pd_now = wd*sum(sum(pd_v));
            pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - T_max,0) ;
            pt_v(routeV) = max(time_afs(routeV) + time_su(routeV) - T_max,0) ;
            pt_now = wt*sum(pt_v);
            time_v = time_afs + time_su;
            [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
            pc_now = pc_now*wc;
            pm_now = pm;
            if isdelete == 1
                delete_idx = routeU;
                [pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm] = ...
                    deleteAFS_delta(delete_idx,node_location,pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm,wm);
                [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
                pc_now = pc_now*wc;
                pt_now = wt*sum(pt_v);
                pd_now = wd*sum(sum(pd_v));
            end
            % Cost delta
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
            if preU,successor(preU) = suX;end
            predecessors(nodeU) = nodeV;
            if nodeX
                successor(nodeX) = nodeY;
            end
            if suX,predecessors(suX) = preU;end
            successor(nodeV) = nodeU;
            if nodeY
                predecessors(nodeY) = nodeX;
            end
            routeID(nodeU) = routeV;
            routeID(nodeX) = routeV;
            node_location(nodeU) = nodeV_loc;
            node_location(nodeX) = nodeV_loc;
            if isdelete == 1
                [routeID,node_location,predecessors,successor] = deleteAFS_node(routeID,delete_idx,node_location,predecessors,successor);
            end
        else
            % Cost calculation
            costOne = - dAll(nodeU+1,nodeX+1) - dAll(preU+1,nodeU+1)     - dAll(nodeX+1,suX+1)     + dAll(preU+1,suX+1);
            costTwo = + dAll(nodeU+1,nodeX+1) - dAll(nodeV+1,nodeY+1)    + dAll(nodeV+1,nodeU+1)   + dAll(nodeX+1,nbClients+2) + dAll(nbClients+2,nodeY+1);
            routeU_NOW= [];
            routeU_NOW(1)=nodeV;
            for i=2:sum(routeID == routeV)
                if (routeU_NOW(i-1)==0)
                    break
                end
                routeU_NOW(i)=successor(routeU_NOW(i-1));
            end
            routeU_NOW(routeU_NOW==0)=[];
            routeU_NOW(1) =[];
            routeU_now =[];
            routeU_now(1)=nodeV;
            for i=2:sum(routeID == routeV)
                if (routeU_now(i-1)==0)
                    break
                end
                routeU_now(i)=predecessors(routeU_now(i-1));
            end
            routeU_now(routeU_now==0)=[];
            routeU_now = fliplr(routeU_now);
            routeU_now=[routeU_now,nodeU,nodeX,numel(node_location)+1,routeU_NOW];
            [d1,d2,t1,t2] = get_pd_pt(vrp,routeU_now,everTime,speed);
            if nodeU_loc == -1
                distance_pre(routeU) = distance_pre(routeU) + costOne;
                time_afs(routeU) = time_afs(routeU) + costOne/speed - 2*everTime;
            elseif nodeU_loc == 1
                distance_su(routeU) = distance_su(routeU) + costOne;
                time_su(routeU) = time_su(routeU) + costOne/speed - 2*everTime;
            end
            distance_pre(routeV) = d1;
            distance_su(routeV) = d2;
            time_afs(routeV) = t1;
            time_su(routeV) = t2;
            pd_v(routeU,1) = max(distance_pre(routeU) - V_Dmax,0);
            pd_v(routeU,2) = max(distance_su(routeU) - V_Dmax,0);
            pd_v(routeV,1) = max(distance_pre(routeV) - V_Dmax,0);
            pd_v(routeV,2) = max(distance_su(routeV) - V_Dmax,0);
            pd_now = wd*sum(sum(pd_v));
            pt_v(routeU) = max(time_afs(routeU) + time_su(routeU) - T_max,0) ;
            pt_v(routeV) = max(time_afs(routeV) + time_su(routeV) - T_max,0) ;
            pt_now = wt*sum(pt_v);
            time_v = time_afs + time_su;
            [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs);
            pc_now = pc_now*wc;
            pm_now = pm;
            if isdelete == 1
                delete_idx = routeU;
                [pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm] = ...
                    deleteAFS_delta(delete_idx,node_location,pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm,wm);
                [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay);
                pc_now = pc_now*wc;
                pt_now = wt*sum(pt_v);
                pd_now = wd*sum(sum(pd_v));
            end
            % Cost delta
            if isdelete == 1
                delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd - 2*dAll(1,nbClients+2);
            else
                delta = costOne + costTwo + pm_now - pm + pt_now - pt + pc_now - pc + pd_now - pd;
            end
            if  delta > -0.000001
                isSuccess = false;
                return
            end
            % Since a new AFS is inserted, information for the new AFS needs to be added
            node_addAfs = numel(node_location)+1 ;
            node_location(node_addAfs) = 100;
            predecessors(node_addAfs) = -1;
            successor(node_addAfs) = -1;
            routeID(node_addAfs) = -1;
            % Predecessor and successor routeID
            if preU,successor(preU) = suX;end
            predecessors(nodeU) = nodeV;
            successor(nodeU) = nodeX;
            if nodeX,successor(nodeX) = node_addAfs;end
            if suX,predecessors(suX) = preU;end
            successor(nodeV) = nodeU;
            if nodeY,predecessors(nodeY) = node_addAfs; end
            predecessors(node_addAfs) = nodeX;
            successor(node_addAfs) = nodeY;
            routeID(nodeU) = routeV;
            routeID(nodeX) = routeV;
            routeID(node_addAfs) = routeV;
            node_location(routeU_now(1:find(routeU_now==node_addAfs))) = -1;
            node_location(routeU_now(find(routeU_now==node_addAfs):end)) = 1;
            node_location(node_addAfs) = 100;
            if isdelete == 1
                [routeID,node_location,predecessors,successor] = deleteAFS_node(routeID,delete_idx,node_location,predecessors,successor);
            end
        end
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
% Update whenLastModified for RouteU and V to indicate that both U and V were successfully updated in this move, assigning nbMoves.
% If a route is deleted, update whenLastModified for RouteU to indicate its deletion.
% If the route is deleted by another move, remove the corresponding when.
if isdelete == 1
    whenLastModified(routeU) = [];
    whenLastModified(routeV) = nbMoves;
else
    whenLastModified(routeU) = nbMoves;
    whenLastModified(routeV) = nbMoves;
end
% Update isSuccess to indicate success
isSuccess = true;


end


