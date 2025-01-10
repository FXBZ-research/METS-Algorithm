function [chromR] = split_Dmax(vrp,tsp,~)
% Use Dmax as the splitting criterion to divide and obtain the routes 
last_F_location = vrp.last_F_location;
nb_customer = vrp.nb_customer;
V_Dmax = vrp.V_Dmax;
V_speed = vrp.V_speed;
T_Afs = vrp.T_Afs;
Tmax = vrp.T_max_V;
T_Start = vrp.T_Start;
T_Customer =vrp.T_Customer;
distance_table = vrp.distance_table;
chromR = cell(numel(tsp),1);
insertnode = [];
pre_route = [];
pre_insertnode = [];insertnode=[];
insert_f = 0;
j = 1; % Represent the route number and place fully inserted vehicles into chromR{j}
has_f = 0;
z = 0;
for i=1:nb_customer*2
    z = z+1;
    if insert_f == 1   % After inserting a refueling station, update pre_insertnode and insertnode
        pre_insertnode = [insertnode(1:end-1) nb_customer + j] ;
        z = z-1;
        insert_f = 0;
    else
        pre_insertnode = insertnode;
    end
    % If it can be inserted, insert it as insertnode
    insertnode = [pre_insertnode tsp(z)];
    pre_route = [0 pre_insertnode 0] + 1;
    route = [ 0 insertnode 0 ] + 1;
    D = 0;
    D_hou = 0;
    for ii=1:numel(route)-1 % Calculate the total distance of a single route
        D = D+distance_table(route(ii),route(ii+1));
    end
    [~,maxx] = max(route);
    for iii = maxx:numel(route) - 1
        D_hou = D_hou + distance_table(route(iii),route(iii+1));
    end
    if has_f == 0
        D_f = D - distance_table(route(end-1),route(end)) + distance_table (route(end-1),nb_customer+2);
    end

    if has_f == 0
        if D_f <= V_Dmax
            if numel(route) - 2 == numel(tsp)
                if D <= V_Dmax
                    chromR{j} = route; % Insert the route into chromR
                    break
                else
                    chromR{j} = [route(1:end-1) nb_customer+j+1 1];
                    j = j+1;  % Update the route number
                    if numel(tsp) == 0
                        disp "splitD"
                    end
                    tsp = []; % Remove the selected points
                    break


                end
            end
            continue
        elseif D_f > V_Dmax
            insert_f = 1;
            has_f = 1;
            continue
        end
    elseif has_f == 1
        if D_hou <= V_Dmax
            if numel(route) - 3 == numel(tsp)
                chromR{j} = route; % Insert the route into chromR
                j = j+1;    % Update the route number
                if numel(tsp) == 0
                    disp "splitD"
                end
                tsp = []; % Remove the selected points
                break
            end
            continue
        elseif D_hou > V_Dmax  % At this point, the route is complete and can be added to chromR
            chromR {j} = pre_route;
            for iii=1:numel(pre_insertnode)
                tsp(tsp==insertnode(iii)) = []; % Remove the selected points
            end
            j = j+1;    % Update the route number
            has_f = 0;
            z = 0;
            insertnode = [];
            pre_route = [];
            pre_insertnode = [];insertnode=[];
        end
    end
end
chromR(cellfun(@isempty,chromR)) = [];
for i=1:numel(chromR)
    if numel(chromR{i}) == 3
        a = max(chromR{i});
        chromR{i} = [1 a i+nb_customer+1 1];
    end
end


end




