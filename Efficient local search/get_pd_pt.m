function [d1,d2,t1,t2] = get_pd_pt(vrp,route_now,everTime,speed)
if numel(route_now)==0
    d1 = 0;
    t1 = 0;
    d2 = 0;
    t2 = 0;
    return
end
[a,n] = max(route_now);
if a > vrp.nb_customer 
    D = 0;
    d1 = 0;
    for i=1:numel(route_now)-1
        D =  D + vrp.distance_table(route_now(i)+1,route_now(i+1)+1);
        if i == n-1
            d1 = D;
            D = 0;
        end
    end
    d1 = d1 + vrp.distance_table(1,route_now(1)+1);
    d2 = D +vrp.distance_table(route_now(end)+1,1);
    t1 = d1/speed + (n-1)*everTime;
    t2 = d2/speed +(numel(route_now) - n + 1)*everTime;
else 
    D = 0;
    for i=1:numel(route_now)-1
        D =  D + vrp.distance_table(route_now(i)+1,route_now(i+1)+1);
    end
    d1 = D + vrp.distance_table(1,route_now(1)+1) + vrp.distance_table(route_now(end)+1,1);
    t1 = d1/speed + numel(route_now)*everTime;
    d2 = 0;
    t2 = 0;
end