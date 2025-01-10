function [y1,y2] = Crossover(p1,p2,vrp)
% Generate offspring

% Retrieve all routes from the parents chromR 1 and 2
chromR_p1 = get_chromR(p1.predecessors{1}, p1.successor{1},p1.routeID{1},vrp.nb_customer) ;
chromR_p2 = get_chromR(p2.predecessors{1}, p2.successor{1},p2.routeID{1},vrp.nb_customer) ;
x1 = [];
x2 = [];
for i = 1:numel(chromR_p1) 
    x1 = [x1,chromR_p1{i}'];
end
for i = 1:numel(chromR_p2)
    x2 = [x2,chromR_p2{i}'];
end


x1(x1 > vrp.nb_customer)=[];
x2(x2 > vrp.nb_customer)=[];
nPoint = length(x1);
y1 = zeros(1,nPoint);
y2 = zeros(1,nPoint);

c=randi(nPoint,2,1); % Return two points
point1=min(c);
point2=max(c);

% inherit: Inherit part of x1 into y1
y1(point1:point2)=x1(point1:point2);
y2(point1:point2)=x2(point1:point2);

% Reorder x2 starting from the second point after point2
x2sorted = [x2(point2+1:nPoint),x2(1:point2)];
x1sorted = [x1(point2+1:nPoint),x1(1:point2)];

% Assign the remaining part of y1 from x2
x2sorted(ismember(x2sorted,y1))=[]; % Remove elements already in x1
y1([point2+1:nPoint,1:point1-1]) = x2sorted;

% Assign the remaining part of y2 from x1
x1sorted(ismember(x1sorted,y2))=[]; % Remove elements already in x2
y2([point2+1:nPoint,1:point1-1]) = x1sorted;

end