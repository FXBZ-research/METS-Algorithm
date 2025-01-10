function [isrepair,feasiblePop,infeasiblePop,sol_table,bestSolRestart,bestSolOverall,isNewBest,vrp] = ...
    Repair_sol(par_hgs,sol_table,vrp,tspid,test,isrepair,feasiblePop,infeasiblePop,bestSolRestart,bestSolOverall,nbClients,Penalty_all,Route_related,seednum)
isrepair = 1;
isNewBest = 0;
WP=10;
for  i=1:numel(Penalty_all(1,:))
    if Penalty_all(2,i) > 0
        Penalty_all(:,i) = Penalty_all(:,i)*WP;
    end
end
[~,sol_table] = ELS_mian(sol_table,1,vrp,tspid,par_hgs,test,isrepair,sol_table.chromR_move{tspid},nbClients,sol_table.node_location{tspid},Penalty_all,Route_related,seednum);
if sol_table.IsFeasible(tspid) == 1
    [feasiblePop,infeasiblePop,vrp] = PopManagement(sol_table(tspid,:),par_hgs,tspid,feasiblePop,infeasiblePop,vrp);
    [bestSolRestart,bestSolOverall,isNewBest] = uti_updateBestSol(bestSolRestart,bestSolOverall,sol_table(tspid,:));
end
isrepair = 0;
end