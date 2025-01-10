function [sol_table,Penalty_all,node_location,Route_related] = chromR_detail_all(vrp,chromR,tspid,tsp,sol_table,Penalty_all)
% Calculate information for the routes chromR. 
% Finally, output a table with headers corresponding to the following details.

% Penalty-related: The matrix has 4 columns, each representing one of the four penalty coefficients.
wT = Penalty_all(1,1);
wC = Penalty_all(1,2);
wD = Penalty_all(1,3);
wM = Penalty_all(1,4);
% Penalty_all(2,1) = wT*penalty_T;
% Penalty_all(2,2) = wC*penalty_C;
% Penalty_all(2,3) = wD*sum(sum(penalty_D_v));
% Penalty_all(2,4) = wM*penalty_m;

Route_related = zeros(numel(chromR),7);
% Route_related(:,[1,2]) = penalty_D_v;
% Route_related(:,3) = penalty_T_v;
% Route_related(:,4) = afs_time;
% Route_related(:,5) = time_V;
% Route_related(:,[6,7]) = distance_pre_su;

dall = vrp.distance_table;

chrom=cell(1);    
chrom{1}=chromR;
sol_table.ID(tspid)=tspid;
sol_table.chromR(tspid)=chrom;

tspp = cell(1);     
tspp{1} = tsp;
sol_table.tsp(tspid) = tspp;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%penalty_m%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sol_table.nb_V(tspid) = numel(chromR);
penalty_m = max((sol_table.nb_V(tspid) - vrp.V_nb),0);
Penalty_all(2,4) = wM*penalty_m;
sol_table.penalty_m(tspid) = wM*penalty_m;

aaa = zeros(numel(chromR),1);
for i=1:numel(chromR)
    aaa(i)=numel(chromR{i});
end
distance_window = zeros(numel(chromR),max(aaa)); 
time_window = zeros(numel(chromR),max(aaa));   
for i=1:numel(time_window(:,1))
    aaa = chromR{i}; 
    aaa(aaa==0)=[];
    for j=1:numel(aaa)-1
        if aaa(j+1) ~= 1 
            time_window(i,j+1) = dall(aaa(j),aaa(j+1))/vrp.V_speed + vrp.T_Customer;
            distance_window(i,j+1) = dall(aaa(j),aaa(j+1));
        else
            time_window(i,j+1) = dall(aaa(j),aaa(j+1))/vrp.V_speed;
            distance_window(i,j+1) = dall(aaa(j),aaa(j+1));
        end
    end
end
distance_windo = cell(1);
distance_windo{1} = distance_window;
sol_table.distance_window(tspid) = distance_windo;

time_windo = cell(1);
time_windo{1} = time_window;
sol_table.time_window(tspid) = time_windo;

% From each time interval, calculate the time refueling is completed, 
% subtract the refueling time to obtain the arrival time at the refueling station (without waiting).
% afs_location: the nodes where each vehicle visits the refueling station.
% afs_time: the time each vehicle arrives at the refueling station 
afs_location = zeros(numel(chromR),1);
afs_time = zeros(numel(chromR),1);
distance_pre_su = zeros(numel(chromR),2);
penalty_D_v = zeros(numel(chromR),2);

for i=1:numel(chromR)
    a = chromR{i}(chromR{i}>vrp.nb_customer + 1);
    for ii=1:numel(a)
        [~,afs_location(i,ii)] =  find(chromR{i} == a(ii));
        afs_time(i,ii) = sum(time_window(i,1:afs_location(i,ii))) -  vrp.T_Customer;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%penalty_D%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z = [1 afs_location(i,:) numel(chromR{i})];
    z(z==0) = [];
    for jj=1:sum(afs_location(i,:)>0)+1
        D = 0;
        for j=z(jj):z(jj+1)-1
            D = dall(chromR{i}(j),chromR{i}(j+1)) + D;
        end
        distance_pre_su(i,jj) = D;
        penalty_D_v(i,jj) = max(D - vrp.V_Dmax,0);
    end
end
Route_related(:,4) = afs_time;

Route_related(:,[1,2]) = penalty_D_v;
penalty_D_ = cell(1);
penalty_D_{1} = penalty_D_v ;
sol_table.penalty_D_v(tspid) = penalty_D_;


penalty_D =sum(sum(penalty_D_v));
sol_table.penalty_D(tspid) = wD*sum(sum(penalty_D_v));
Penalty_all(2,3) = wD*sum(sum(penalty_D_v));

distance_pre_s = cell(1);
distance_pre_s{1} = distance_pre_su;
sol_table.distance_pre_su(tspid) = distance_pre_s;
Route_related(:,[6,7]) = distance_pre_su;

% Calculate the following:
%   %% Note: Waiting time is not included here and needs to be added
%   Time for each vehicle: time_V, and the total time for the solution: time_Total
%   Overtime for each vehicle: overtime_V, and the total overtime for the solution: overtime_Total
%   Distance for each vehicle: distance_V, and the total distance: distance_Total
%   Whether each vehicle accessed a refueling station: afs_V, and the total number of refueling station visits: afs_Total
time_V = zeros(size(chromR));
overtime_V = zeros(size(chromR));
distance_V = zeros(size(chromR));
afs_V = zeros(size(chromR));
for i=1:numel(chromR)
    a = chromR{i};
    for j=1:numel(a)-1
        distance_V(i,1) = vrp.distance_table(a(j),a(j+1)) + distance_V(i,1);
    end
    time_V(i,1) = distance_V(i,1)/vrp.V_speed + (numel(a)-2)*vrp.T_Customer;
    overtime_V(i,1) = time_V(i,1) - vrp.T_max_V;
    overtime_V(i,1) = max(overtime_V(i,1),0);
    afs_V(i) = sum(afs_location(i,:)>0);
end

time_V_shifting = max(vrp.T_max_V - time_V,0);
time_V_shiftin = cell(1);
time_V_shiftin{1} = time_V_shifting;
sol_table.time_V_shifting(tspid) = time_V_shiftin;
afs_time_delay = afs_time;
Route_related(:,5) = time_V;
conflict_table = zeros(numel(chromR),numel(chromR));  
for i=1:numel(chromR)-1
    for ii=i+1:numel(chromR)
        if afs_time_delay(i) >= afs_time_delay(ii) 
            t1 = afs_time_delay(ii);
            t2 = afs_time_delay(i);
            q1 = time_V_shifting(ii);
            q2 = time_V_shifting(i);
            t3 = 1;
        else
            t1 = afs_time_delay(i);
            t2 = afs_time_delay(ii);
            q1 = time_V_shifting(i);
            q2 = time_V_shifting(ii);
            t3 = 2;
        end
        if t1+vrp.T_Afs <= t2 
            continue
        end
        if t1 + vrp.T_Afs - t2 > q2 && t2 + vrp.T_Afs - t1 > q1  
            conflict_table(i,ii) = 1;
            conflict_table(ii,i) = 1;
            continue
        end
        if t1 + vrp.T_Afs - t2 > q2                  
            t1 = t2 + vrp.T_Afs;
            if t3 == 1
                afs_time_delay(ii) = t1;
                time_V_shifting(ii) = vrp.T_max_V - t1 - vrp.T_Afs;
                i=1;
                ii=1;
            elseif t3 == 2
                afs_time_delay(i) = t1;
                time_V_shifting(i) = vrp.T_max_V - t1 - vrp.T_Afs;
                i=1;
                ii=1;
            end
            continue
        end
        if t2 + vrp.T_Afs - t1 > q1                     
            t2 = t1 + vrp.T_Afs;
            if t3 == 1
                afs_time_delay(i) = t2;
                time_V_shifting(i) = vrp.T_max_V - t2 - vrp.T_Afs;
                i=1;
                ii=1;
            elseif t3 == 2
                afs_time_delay(ii) = t2;
                time_V_shifting(ii) = vrp.T_max_V - t2 - vrp.T_Afs;
                i=1;
                ii=1;
            end
            continue
        end
        if t1 + vrp.T_Afs - t2 <= q2 && t2 + vrp.T_Afs - t1 <= q1   
            if  q1 > q2                                     
                t1 = t2 + vrp.T_Afs;
                if t3 == 1
                    afs_time_delay(ii) = t1;
                    time_V_shifting(ii) = vrp.T_max_V - t1 - vrp.T_Afs;
                    i=1;
                    ii=1;
                elseif t3 == 2
                    afs_time_delay(i) = t1;
                    time_V_shifting(i) = vrp.T_max_V - t1 - vrp.T_Afs;
                    i=1;
                    ii=1;
                end
                continue
            else
                if t3 == 1
                    afs_time_delay(i) = t2;
                    time_V_shifting(i) = vrp.T_max_V - t2 - vrp.T_Afs;
                    i=1;
                    ii=1;
                elseif t3 == 2
                    afs_time_delay(ii) = t2;
                    time_V_shifting(ii) = vrp.T_max_V - t2 - vrp.T_Afs;
                    i=1;
                    ii=1;
                end
                continue
            end
        end
    end
end


afs_time_dela = cell(1);
afs_time_dela{1} = afs_time_delay;
sol_table.afs_time_delay(tspid) = afs_time_dela;

delay_duration = afs_time_delay - afs_time;
if sum(delay_duration) > 0
    isdelay = 1;
else
    isdelay = 0;
end
sol_table.isdelay(tspid) = isdelay;

delay_duratio = cell(1);
delay_duratio{1} = delay_duration;
sol_table.delay_duration(tspid) = delay_duratio;

afs_tim=cell(1);
afs_tim{1} = afs_time;
sol_table.afs_time(tspid) = afs_tim;

afs_time1 = afs_time_delay(:);
afs_time1(afs_time1==0) = [];
afs_time_end = afs_time1 + vrp.T_Afs;
afs_time_end = afs_time_end(:);
afs_time_end(afs_time_end==0) = [];
c = sort([afs_time1',afs_time_end']);
for i=2:numel(c)
    time_during(i-1) = c(i) - c(i-1);
end
for i=1:numel(c)
    d = afs_time1 - c(i);
    nb_fueling(i) = numel(d(-vrp.T_Afs< d & d<=0)); 
end
if exist('nb_fueling', 'var')
    nb_fueling(end) = [];
    nb_fueling = max(nb_fueling - vrp.C_Afs,0);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% penalty_C %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nb_fuelin=cell(1);
    nb_fuelin{1}=nb_fueling;
    time_durin=cell(1);
    time_durin{1}=time_during;
    sol_table.nb_fueling(tspid) = nb_fuelin;
    sol_table.time_during(tspid) = time_durin;
    penalty_C = max(nb_fueling*time_during',0);
    sol_table.penalty_C(tspid) = wC*penalty_C;
    Penalty_all(2,2) = wC*penalty_C;
else
    penalty_C = 0;
    time_durin{1} = zeros(size(afs_time_delay'));
    sol_table.nb_fueling(tspid) = time_durin;
    sol_table.time_during(tspid) = time_durin;
    sol_table.penalty_C(tspid) = wC*penalty_C;
    Penalty_all(2,2) = wC*penalty_C;
end

sol_tim = cell(1);
sol_tim{1} = time_V;
sol_table.time_V(tspid) = sol_tim;   
sol_table.time_V_max(tspid) = max(time_V);  
sol_table.time_V_min(tspid) = min(time_V);   
sol_table.time_Total(tspid) = sum(time_V);   

overtim_V = cell(1);
overtim_V{1} = overtime_V;
sol_table.overtime_V(tspid) = overtim_V;
sol_table.overtime_V_max(tspid) = max(overtime_V);   
sol_table.overtime_V_min(tspid) = min(overtime_V);   
sol_table.overtime_Total(tspid) = sum(overtime_V);   




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% penalty_T %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
penalty_T = max(sol_table.overtime_Total(tspid),0);
sol_table.penalty_T(tspid) = wT*penalty_T;
Penalty_all(2,1) = wT*penalty_T;

penalty_T_v = max(time_V - vrp.T_max_V,0 );
Route_related(:,3) = penalty_T_v;

penalty_T_ = cell(1);
penalty_T_{1} = penalty_T_v;
sol_table.penalty_T_v(tspid) = penalty_T_;

distance_v = cell(1);
distance_v{1} = distance_V;
sol_table.distance_V(tspid) = distance_v;    % Save the distance of each route into the table
sol_table.distance_V_max(tspid) = max(distance_V);   % Maximum distance among all vehicles
sol_table.distance_V_min(tspid) = min(distance_V);   % Minimum distance among all vehicles
sol_table.distance_Total(tspid) = sum(distance_V);   % Total distance

af_V = cell(1);
af_V{1} = afs_V;
sol_table.afs_V(tspid) = af_V;       % Save whether each route accesses a refueling station into the table
sol_table.afs_Total(tspid) = sum(afs_V); 

sol_table.cost_Total(tspid) = wT*penalty_T + wC*penalty_C + wD*penalty_D + wM*penalty_m + sol_table.distance_Total(tspid);

% Check whether the solution is feasible
if penalty_T + penalty_C + penalty_D + penalty_m > 0
    sol_table.IsFeasible(tspid) = 0;
else
    sol_table.IsFeasible(tspid) = 1;
end



chromR_move = sol_table.chromR{tspid};
for i=1:numel(chromR_move)   
    a=chromR_move{i}-1;
    a(a==0)=[];
    chromR_move{i} = a;
end
chrom_move=cell(1);         % Save the route into the table
chrom_move{1}=chromR_move;
sol_table.chromR_move(tspid)=chrom_move;

a=[];
for i=1:numel(chromR_move)    % Update the TSP sequence
    a = [a chromR_move{i}];
end
aa=cell(1);
aa{1}=a;
sol_table.tsp_now(tspid) = aa;



predecessor=cell(1);

sol_table.predecessors(tspid) = predecessor;

successo=cell(1);
sol_table.successor(tspid) = successo;
routeI=cell(1);
sol_table.routeID(tspid) = routeI;

% Get the position of each point on their respective routes
% Since only the relative position between the node and the AFS needs to be compared:
% Points before the AFS are marked as -1, AFS is marked as 0, and points after the AFS are marked as 1.
% If there is no AFS, the value is also -1. This is because D is divided into two parts: 
% d1 (before AFS) and d2 (after AFS). Without an AFS, everything falls under d1, 
% aligning with the points before the AFS (-1).
A=a;
[~,I]=sort(A);
afs_loc = afs_location - 1;
for i=1:numel(chromR_move)
    if  max(chromR_move{i}) <= vrp.nb_customer
        I(chromR_move{i}) = -1;
    else
        I(chromR_move{i}(1:afs_loc(i))) = -1;
        I(chromR_move{i}(afs_loc(i) : end)) = 1;
        I(chromR_move{i}(afs_loc(i))) = 100;
    end
end
node_location = I;
node_locatio =  cell(1);
node_locatio{1} = I;
sol_table.node_location(tspid) = node_locatio;
end