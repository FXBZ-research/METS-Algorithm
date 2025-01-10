function [p] = selectparents(feasiblePop,infeasiblePop,tspid,SEED)
% Tournament selection for parents
if numel(feasiblePop) == 0
    a=0;
else
    a = numel(feasiblePop.ID);
end
if numel(infeasiblePop) == 0
    b=0;
else
    b = numel(infeasiblePop.ID);
end
p1 = randi(a + b);
if p1 > a
    p1= infeasiblePop(p1-a,:);
else
    p1= feasiblePop(p1,:);
end

p2 = randi(a + b);
if p2 > a
    p2= infeasiblePop(p2-a,:);
else
    p2= feasiblePop(p2,:);
end

if p1.Fitness < p2.Fitness
    p = p1;
else
    p = p2;
end
end