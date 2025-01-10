function  [sol_individual,sol_table,Penalty_all,Route_related] = ELS_mian(sol_table,sol_individual,vrp,tspid,par_hgs,test,isrepair,chromR,nbClients,node_location,Penalty_all,Route_related,SEED)
% LocalSearch
afs_time_delay = Route_related(:,4);
correlatedVertices = vrp.correlatedVertices;
if ~isrepair
    chromR_move = chromR;
    for i=1:numel(chromR)
        a=chromR_move{i}-1;
        a(a==0)=[];
        chromR_move{i} = a;
    end
else
    chromR_move = chromR;
end
[predecessors,successor,routeID] = phrase_chromR(vrp,tspid,nbClients,chromR_move);
[a,~] =size(node_location);
[aa,bb] = size(predecessors);
if a == bb
    Node_related = [predecessors,successor,routeID,node_location'];
elseif a == aa
    Node_related = [predecessors,successor,routeID,node_location];
end
% 2) Main Loop: Local Search based on inital solution sol
whenLastTestedRI = ones(nbClients,1) * 0;
whenLastModified = ones(numel(chromR_move),1) * 0;
ispatch = sum(whenLastModified);
nbMoves = 0;
loopID = 0;
searchCompleted = false;
isSuccess = 0;
while ~searchCompleted
    if loopID > 0
        searchCompleted = true;  %// Allows at least two loops since some moves involving empty routes are not checked at the first loop
    end

    % Enumerate (exhaust) the search space of the neighborhood
    rng(SEED+tspid)
    for i=1:nbClients
        if mod(randi(999999999), par_hgs.nbGranular) == 0
            correlatedVertices(i,:) = correlatedVertices(i,randperm(numel(correlatedVertices(i,:))));
        end
    end
    if tspid ~= 1
        if toc - sol_table.time(end-1) > sol_table.time(end-1)*20/numel(sol_table.ID)
            break
        end
    end

    % Loop through U
    for ii = 1: nbClients
        nodeU = ii;
        correlatedU = correlatedVertices(nodeU,:);
        lastTestRINodeU = whenLastTestedRI(nodeU);
        whenLastTestedRI(nodeU) = nbMoves;
        % Loop through V
        for jj = 1 : length(correlatedU)
            posV = jj;
            nodeV = correlatedU(posV);
            afs_time_delay1 = afs_time_delay;
            if ispatch~=sum(whenLastModified)
                ispatch = sum(whenLastModified);
                tolerance = 1e-10;
                for patch3 = 1:numel(Route_related(:,6))
                    if abs(Route_related(patch3,6)) <= tolerance && abs(Route_related(patch3,7)) <= tolerance
                        Route_related(patch3,:) = [];
                        Node_related(Node_related(:,3)>patch3,3) = Node_related(Node_related(:,3)>patch3,3)-1;
                        break
                    end
                end
                sort_ind = zeros(numel(Route_related(:,1)),3);
                update_pt = 0;
                update_pc = 0;
                patch2 = 1;
                while patch2 <= numel(Route_related(:,1))
                    if max(Node_related(Node_related(:,3)==patch2,4)) == 100 && max(find(Node_related(:,3)==patch2)) == patch2 + nbClients  %#ok<MXFND>
                        a=sum(Route_related(patch2,[6,7]));
                        b = a - vrp.distance_table(Node_related((patch2 + nbClients),1)+1,patch2 + nbClients+1)-vrp.distance_table(patch2 + nbClients+1,Node_related((patch2 + nbClients),2)+1) +vrp.distance_table(Node_related((patch2 + nbClients),1)+1,Node_related((patch2 + nbClients),2)+1);
                        if b <= vrp.V_Dmax
                            update_pc = 1;
                            chromR = get_chromR(Node_related(:,1),Node_related(:,2),Node_related(:,3),vrp.nb_customer);
                            route_now = chromR{patch2}';
                            route_now(route_now == max(route_now))=[];
                            [patch_d,~,patch_t,~] = get_pd_pt(vrp,route_now,vrp.T_Customer,vrp.V_speed);
                            if Route_related(patch2,3) ~= max(0,patch_t -vrp.T_max_V)
                                update_pt = 1;
                            end
                            Route_related(patch2,:) = [0,0,max(0,patch_t -vrp.T_max_V),patch_t,patch_t,patch_d,0];
                            c = Node_related(patch2 + nbClients,:);
                            if c(1),Node_related(c(1),2) = c(2);end
                            if c(2),Node_related(c(2),1) = c(1);end
                            Node_related(patch2 + nbClients,:) = [];
                            Node_related((Node_related(:, 3) == patch2),4) = -1;
                            if patch2 ~= numel(Route_related(:,1))
                                Route_related(numel(Route_related(:,1))+1,:) = Route_related(patch2,:);
                                Route_related(patch2:end-1,:) = Route_related(patch2+1:end,:);
                                Route_related(end,:) = [];
                                Node_related(Node_related(:,3) == patch2,3) =  max(Node_related(:,3)) + 1;
                                Node_related(Node_related(:,3) > patch2,3) = Node_related(Node_related(:,3) >= patch2,3) -1;
                                Node_related(Node_related(:,1) > patch2+nbClients,1) = Node_related(Node_related(:,1) > patch2+nbClients,1) -1;
                                Node_related(Node_related(:,2) > patch2+nbClients,2) = Node_related(Node_related(:,2) > patch2+nbClients,2) -1;
                                patch2 = patch2 - 1;
                            end
                        end
                    else
                        if  max(Node_related(Node_related(:,3)==patch2,4)) ~= 100
                            if patch2 <= sum(Node_related(:,4)==100)
                                Node_related(Node_related(:,3) == patch2,3) =  max(Node_related(:,3)) + 1;
                                Node_related(Node_related(:,3) > patch2,3) = Node_related(Node_related(:,3) >= patch2,3) -1;
                                Route_related(numel(Route_related(:,1))+1,:) = Route_related(patch2,:);
                                Route_related(patch2:end-1,:) = Route_related(patch2+1:end,:);
                                Route_related(end,:) = [];
                                patch2 = patch2 - 1;
                            end

                        elseif max(find(Node_related(:,3)==patch2)) ~= patch2 + nbClients %#ok<MXFND>
                            sort_ind(patch2,:) = [max(find(Node_related(:,3)==patch2)),patch2,max(find(Node_related(:,3)==patch2))-vrp.nb_customer];%#ok<MXFND> 
                        end
                    end
                    patch2 = patch2 + 1;
                end
                sort_ind(sort_ind(:,1)==0,:)=[];
                if numel(sort_ind) ~= 0
                    Node_related(vrp.nb_customer+1:end,:) = sortrows(Node_related(vrp.nb_customer+1:end,:),3);
                    for i = 1:numel(sort_ind(:,1))
                        Node_related(Node_related == sort_ind(i,1)) = - sort_ind(i,3);
                    end
                    for i = 1:numel(sort_ind(:,1))
                        Node_related(Node_related(:,[1,2]) == - sort_ind(i,3)) = sort_ind(i,2)+vrp.nb_customer;
                    end
                end
                if update_pt == 1
                    Penalty_all(2,1) = Penalty_all(1,1)*sum(Route_related(:,3));
                end
                if update_pc == 1
                    [pc_now,afs_time_delay] = AFSdelay_new(Route_related(1:(numel(Node_related(:,1)) - nbClients),4),Route_related(1:(numel(Node_related(:,1)) - nbClients),5),vrp.T_max_V,vrp.T_Afs,vrp.C_Afs,afs_time_delay);
                    Penalty_all(2,2) = Penalty_all(1,2)*pc_now;

                end
                for patch1=nbClients+1:numel(Node_related(:,1))
                    if Node_related(patch1,1) == 0 && Route_related(patch1-nbClients,4) ~= 2
                        Route_related(patch1-nbClients,4) = 2;
                    end
                end
            end

            if loopID==0 || (max(whenLastModified(Node_related(nodeU,3)), whenLastModified(Node_related(nodeV,3))) > lastTestRINodeU)

                [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m1(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m2(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m3(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m4(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m5(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m6(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m7(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m8(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                if isSuccess, continue; end
                [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                    m9(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);

                if isSuccess, continue; end
                if Node_related(nodeV,1) == 0
                    [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                        Depot_m1(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                    if isSuccess, continue; end
                    [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                        Depot_m2(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                    if isSuccess, continue; end
                    [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                        Depot_m3(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                    if isSuccess, continue; end
                    [isSuccess,nbMoves,searchCompleted,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                        Depot_m8(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                    if isSuccess, continue; end
                    [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                        Depot_m9(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
                    if isSuccess, continue; end
                end
            end
        end
        if loopID ~= 1   &&  nodeV == correlatedU(end)
            [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                NewRoute_m1(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
            if isSuccess, continue; end
            [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                NewRoute_m2(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
            if isSuccess, continue; end
            [isSuccess,nbMoves,searchCompleted,~,~,~,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay] = ...
                NewRoute_m3(nodeU,nodeV,vrp,nbMoves,searchCompleted,tspid,par_hgs,whenLastModified,Penalty_all,Route_related,Node_related,afs_time_delay);
            if isSuccess, continue; end
        end
    end
    loopID = loopID + 1;
end

% Update the table and save the information
chromR = get_chromR(Node_related(:,1),Node_related(:,2),Node_related(:,3),nbClients);
afs_time_delay1 = afs_time_delay;
distance_Total = sum(sum(Route_related(:,[6,7])));
cost_Total = distance_Total + sum(Penalty_all(2,:));
if sum(Penalty_all(2,:)) ~= 0
    IsFeasible = 0;
else
    IsFeasible = 1;
end
sol_table.afs_time_delay{tspid} = afs_time_delay1;
sol_table.chromR_move{tspid} = chromR;
sol_table.distance_Total(tspid) = distance_Total;
sol_table.cost_Total(tspid) = cost_Total;
sol_table.predecessors{tspid} = Node_related(:,1);
sol_table.successor{tspid} = Node_related(:,2);
sol_table.routeID{tspid} = Node_related(:,3);
sol_table.node_location{tspid} = Node_related(:,4)';
sol_table.distance_pre_su{tspid} = Route_related(:,[6,7]);
sol_table.time_V{tspid} = Route_related(:,5);
sol_table.afs_time{tspid} = Route_related(:,4);
sol_table.penalty_D_v{tspid} = Route_related(:,[1,2]);
sol_table.penalty_T_v{tspid} = Route_related(:,3);
sol_table.penalty_T(tspid) = Penalty_all(2,1);
sol_table.penalty_C(tspid) = Penalty_all(2,2);
sol_table.penalty_D(tspid) = Penalty_all(2,3);
sol_table.penalty_m(tspid) = Penalty_all(2,4);
sol_table.IsFeasible(tspid) = IsFeasible;
end
