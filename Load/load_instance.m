function [vrp] = load_instance(fileFolder,nbGranular,instance_id)
%load_instance

dirOutput = dir(fullfile(fileFolder,'*.dat'));
% fileNames = {dirOutput.name};
% fileNames = cellstr(fileNames);
fileID = fopen(dirOutput(instance_id).name);
name = dirOutput(instance_id).name;
C = textscan(fileID,'%s %s %f %f','HeaderLines',1);
if numel(C{1}) > numel(C{3}) 
    C{1}(end) = [];
    C{2}(end) = [];
end
a = C{1};
b = C{2};
a1 = a(2);
a(2) = [];
a(end+1) = a1;
b1 = b(2);
b(2) = [];
b(end+1) = b1;

aas = strcmp(b,'d');
last_depot_location = find(aas == 1, 1, 'last' );
bbs = strcmp(b,'f');
last_F_location = find(bbs == 1, 1, 'last' );
ccs = strcmp(b,'c');
last_customer_location = find(ccs == 1, 1, 'last' );
c = C{3};
d = C{4};
c1 = c(2);
c(2) = [];
c(end+1) = c1;
d1 = d(2);
d(2) = [];
d(end+1) = d1;

distance_table=zeros(numel(c),numel(c));
for i=1:numel(c)
    for j=1:numel(c)
        distance_table(i,j) = sqrt((c(i)-c(j))*(c(i)-c(j))+(d(i)-d(j))*(d(i)-d(j)));
    end
end
nb_customer = numel(d) - sum(aas) - sum(bbs);
switch nb_customer
    case 15
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 7;
        V_speed = 40;
        nb_V = 15;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 1;
    case 25
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 7.5;
        V_speed = 40;
        nb_V = 7;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 2;
    case 50
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 7.5;
        V_speed = 40;
        nb_V = 13;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 3;
    case 100
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 7.5;
        V_speed = 40;
        nb_V = 25;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 8;
    case 200
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 8;
        V_speed = 40;
        nb_V = 999;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 20;
    case 400
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 8;
        V_speed = 40;
        nb_V = 999;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 40;
    case 600
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 8;
        V_speed = 40;
        nb_V = 999;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 60;
    case 800
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 8;
        V_speed = 40;
        nb_V = 9999;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 80;
    case 1000
        V_fuel = 32;
        fuel_rate = 0.2;
        V_Dmax = V_fuel/fuel_rate;
        T_max_V = 8;
        V_speed = 40;
        nb_V = 9999;
        T_Start = 0;
        T_Afs = 0.5;
        T_Customer =0.5;
        C_Afs = 100;

end
for i=last_customer_location+1:last_customer_location+1+nb_customer

    distance_table(i,:)=distance_table(last_customer_location+1,:);
    distance_table(:,i)=distance_table(:,last_customer_location+1);
end
distance_close = sort(distance_table);
% Adjust the distance_close matrix by excluding the first row and column.
% Add 2 to ensure refueling stations are not considered as candidate points.
distance_close=distance_close(2:end,2:2+nb_customer-1);
correlatedVertices = zeros(numel(distance_close(1,:)),nbGranular);
for i=1:numel(distance_close(1,:))
    [~,aa] = ismember(distance_close(:,i),distance_table(:,i+1));
    aa(aa>=last_customer_location + 1) = []; % Remove AFS by excluding indices greater than the last customer location.
    aa(aa==1) = [];   % Remove the depot (index 1).
    % Assign the top nbGranular correlated vertices, adjusting the encoding 
    % such that 1 corresponds to a customer point (previously, 1 was the depot).
    correlatedVertices(i,:)= aa(1:nbGranular) - 1 ;    
end

vrp.id = a;
vrp.type = b;
vrp.longitude = c;
vrp.latitude = d;
vrp.last_F_location = last_F_location;
vrp.last_customer = last_customer_location;
vrp.last_D = last_depot_location;
vrp.nb_customer = nb_customer;
vrp.distance_table = distance_table;
vrp.V_fuel = V_fuel;
vrp.V_fuel_rate = fuel_rate;
vrp.V_Dmax=V_Dmax;
vrp.V_speed = V_speed;
vrp.V_nb = nb_V;
vrp.T_max_V = T_max_V;
vrp.T_Start = T_Start;
vrp.T_Afs = T_Afs;
vrp.T_Customer = T_Customer;
vrp.C_Afs = C_Afs;
vrp.correlatedVertices = correlatedVertices;
vrp.name = name;

end