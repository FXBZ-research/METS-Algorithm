function [chromR] = split_Tmax(vrp,tsp,~)
% Use Tmax as the splitting criterion to divide and obtain the routes 
last_F_location = vrp.last_F_location;
nb_customer = vrp.nb_customer;
V_Dmax = vrp.V_Dmax;
V_speed = vrp.V_speed;
T_Afs = vrp.T_Afs;
Tmax = vrp.T_max_V;
T_Start = vrp.T_Start;
T_Customer =vrp.T_Customer;
distance_table = vrp.distance_table;
split_tsp = tsp;
chromR = cell(numel(split_tsp),1);
c = zeros(numel(split_tsp),1);
for i=1:numel(chromR)
    ii = 0;
    while numel(split_tsp) ~= 0
        ii = ii + 1;
        a = split_tsp(1:ii);
        [isT_able] = T_able(distance_table,V_speed,T_Customer,a,Tmax);
        if isT_able == 0
            c(i) = ii - 1;
            split_tsp(1:ii-1) = [];
            ii = 0;
            break
        end
        if numel(split_tsp) == ii
            c(i) = ii;
            split_tsp = [];
        end
    end
end
c(c==0) = [];
a = [1;cumsum(c)];
for i=1:numel(a)-1
if i == 1
    chromR{i} = [1 tsp(a(i):a(i+1))+1 1];
else
    chromR{i} = [1 tsp(a(i)+1:a(i+1))+1 1];
end
end

chromR(cellfun(@isempty,chromR)) = [];

end
