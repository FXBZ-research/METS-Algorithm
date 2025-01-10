function [bestSolRestart,bestSolOverall,isNewBest] = uti_updateBestSol(bestSolRestart,bestSolOverall,sol_table)

if numel(bestSolRestart) == 0
    bestSolRestart.cost_Total = 99999999; 
end
if numel(bestSolOverall) == 0
    bestSolOverall.cost_Total = 99999999;  
end
% If the current individual is feasible and the objective value < the objective value after Restart, update Best.
if sol_table.IsFeasible && sol_table.cost_Total < bestSolRestart.cost_Total - 0.000000001
    bestSolRestart = sol_table;
    if sol_table.cost_Total < bestSolOverall.cost_Total - 0.000000001
        bestSolOverall = sol_table;
    end
    isNewBest = true;
else
    isNewBest = false;
end
end
