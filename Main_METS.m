function [Result,Resulttime] = Main_METS(INSTANCE,SEED)
%METS
clc,close,warning off;
addpath(genpath(pwd))
%par set
split_prob = 0.5;
PT = 527 ;
PC = 195 ;
PD = 430;
penaltyScaleFactor = 1.2 ;
penaltyDecreaseFactor = 0.85 ;
popSizeMu = 154 ;
popSizeLambda = 68 ;
targetFeasible = 0.2 ;
nbLast = 20;
maxIterNonProd = 300;
maxIter = 2000;
timeLimit = 100000;

%load instance
INSTANCE = str2double(INSTANCE);
vrp = get_vrp(INSTANCE); % Load the problem instance

disp('_________________ Parameter Settings _______________')
disp(['Instance Parameters: Instance:', num2str(INSTANCE), ' | SEED:', num2str(SEED)]);
disp(['Penalty Parameters: PT: ', num2str(PT), ' | PC: ', num2str(PC), ' | PD: ', num2str(PC), ' | penaltyScaleFactor: ', num2str(penaltyScaleFactor), ' | penaltyDecreaseFactor: ', num2str(penaltyDecreaseFactor)]);
disp(['Population Parameters: popSizeMu: ', num2str(popSizeMu), ' | popSizeLambda:', num2str(popSizeLambda), ' | targetFeasible:', num2str(targetFeasible), ' | ', 'nbLast:', num2str(nbLast)]);
disp(['Algorithm Parameters: timeLimit: ', num2str(timeLimit), ' | maxIter:', num2str(maxIter), ' | maxIterNonProd:', num2str(maxIterNonProd)]);


tic
test = 99999; % Display detailed information
isrepair=0;
last_toc = toc;

par_hgs.maxIter = maxIter;  % Maximum number of iterations
par_hgs.maxIterNonProd = maxIterNonProd;  % Maximum number of non-improving iterations
par_hgs.timeLimit = timeLimit;  % Time limit in seconds


par_hgs.popSizeMu = popSizeMu;
par_hgs.popSizeLambda = popSizeLambda;
par_hgs.el = 0.5;
par_hgs.eliteNum = floor(par_hgs.el * par_hgs.popSizeMu);
par_hgs.nc = 0.2 ;
par_hgs.nClosest = floor(par_hgs.nc * par_hgs.popSizeMu);
par_hgs.nbGranular = 20;
par_hgs.targetFeasible  = targetFeasible;		 % Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters
par_hgs.nbLast = nbLast;
% The penalty coefficients wt, wc, wd, and wm correspond to:
% maximum time penalty, maximum refueling station capacity penalty, maximum travel distance penalty, and maximum vehicle count penalty, respectively.
% The second line specifies the exact values of each penalty.
Penalty_all = [PT,PC,PD,0;0,0,0,0];
par_hgs.Penalty_all = Penalty_all;

% Initialization
nbClients=vrp.nb_customer;
sol_table = table;
bestSolRestart = table;
bestSolOverall = table;
bestSolOverall.IsFeasible = 0;
bestSolOverall.cost_Total = 999999;
Last100Sol = table;
feasiblePop = table;
infeasiblePop = table;
tspid = 1;
rng(SEED+tspid)
tsp_all = arrayfun(@(~) randperm(nbClients), zeros(par_hgs.popSizeMu*4, 1), 'UniformOutput', false);
tsp_all = cell2mat(tsp_all);

% Initial solution population
for i=1:par_hgs.popSizeMu*4

    sol_individual = struct;
    tspid = i;
    if toc > par_hgs.timeLimit
        break;
    end
    if tspid > par_hgs.maxIter
        break;
    end

    tsp = tsp_all(i,:);

    % Two novel segmentation
    if rand <= split_prob
        [chromR] = split_Dmax(vrp, tsp, par_hgs);   % The probability of executing split_damax is split_prob
    else
        [chromR] = split_Tmax(vrp, tsp, par_hgs);   % The probability of executing split_damax is 1 - split_prob
    end


    % Read information from complete solutions
    [sol_table,Penalty_all,node_location,Route_related] = chromR_detail_all(vrp,chromR,tspid,tsp,sol_table,Penalty_all);

    % Efficient local search
    [sol_individual,sol_table,Penalty_all,Route_related] = ELS_mian(sol_table,sol_individual,vrp,tspid,par_hgs,test,isrepair,chromR,nbClients,node_location,Penalty_all,Route_related,SEED);
    sol_table.time(i) = toc;

    % Population management
    [feasiblePop,infeasiblePop,vrp] = PopManagement(sol_table(tspid,:),par_hgs,tspid,feasiblePop,infeasiblePop,vrp);

    % Update bestsol
    [bestSolRestart,bestSolOverall] = uti_updateBestSol(bestSolRestart,bestSolOverall,sol_table(tspid,:));

    % Update Last100 table
    Last100Sol = uti_addSol2Last100(Last100Sol,sol_table(tspid,:),par_hgs);

    % Repair
    if sol_table.IsFeasible(tspid) == 0 && rand(1)-0.5 > 0.000001
        [isrepair,feasiblePop,infeasiblePop,sol_table,bestSolRestart,bestSolOverall,~,vrp] = Repair_sol(par_hgs,sol_table,vrp,tspid,test,isrepair,feasiblePop,infeasiblePop,bestSolRestart,bestSolOverall,nbClients,Penalty_all,Route_related,SEED);
        sol_table.time(i) = toc;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main Loop START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbIterNonProd = 1;
for tspid=par_hgs.popSizeMu*4+1 : par_hgs.maxIter
    % Iteration termination conditions
    if nbIterNonProd > par_hgs.maxIterNonProd || toc > par_hgs.timeLimit
        break;
    end

    % Select parents
    [p1] = selectparents(feasiblePop,infeasiblePop,tspid,SEED);
    [p2] = selectparents(feasiblePop,infeasiblePop,tspid,SEED);

    % Crossover
    [offspring_tsp,~] = Crossover(p1,p2,vrp);

    rng(SEED+tspid)

    % Two novel segmentation
    if rand <= 0.5
        [chromR] = split_Dmax(vrp,tsp,par_hgs);   %(Dmax)
    else
        [chromR] = split_Tmax(vrp,tsp,par_hgs);   %(Tmax)
    end

    % Read information from complete solutions
    [sol_table,Penalty_all,node_location,Route_related] = chromR_detail_all(vrp,chromR,tspid,offspring_tsp,sol_table,Penalty_all);

    % Efficient local search
    [sol_individual,sol_table,Penalty_all,Route_related] = ELS_mian(sol_table,sol_individual,vrp,tspid,par_hgs,test,isrepair,chromR,nbClients,node_location,Penalty_all,Route_related,SEED);
    sol_table.time(tspid) = toc;

    % Population management
    [feasiblePop,infeasiblePop,vrp] = PopManagement(sol_table(tspid,:),par_hgs,tspid,feasiblePop,infeasiblePop,vrp);

    % Update bestsol
    [bestSolRestart,bestSolOverall,isNewBest] = uti_updateBestSol(bestSolRestart,bestSolOverall,sol_table(tspid,:));

    % Update Last100 table
    Last100Sol = uti_addSol2Last100(Last100Sol,sol_table(tspid,:),par_hgs);

    % Repair
    if sol_table.IsFeasible(tspid) == 0 && rand(1)-0.5 > 0.000001
        [isrepair,feasiblePop,infeasiblePop,sol_table,bestSolRestart,bestSolOverall,isNewBest,vrp] = Repair_sol(par_hgs,sol_table,vrp,tspid,test,isrepair,feasiblePop,infeasiblePop,bestSolRestart,bestSolOverall,nbClients,Penalty_all,Route_related,SEED);
        sol_table.time(tspid) = toc;
    end

    % Update nbIterNonProd
    if isNewBest
        toBestTime = toc;
        nbIterNonProd = 1;
    else
        nbIterNonProd = nbIterNonProd+1;
    end

    % Update the penalty parameter
    if mod(tspid,par_hgs.nbLast) == 0 && tspid>=100
        fractionFeasible_T = sum(Last100Sol.penalty_T==0)/numel(Last100Sol.ID);
        fractionFeasible_C = sum(Last100Sol.penalty_C==0)/numel(Last100Sol.ID);
        fractionFeasible_D = sum(Last100Sol.penalty_D==0)/numel(Last100Sol.ID);
        % If the proportion of feasible solutions is less than 0.2 - 0.05: too many infeasible solutions in the population, increase the penalty weight
        % If the proportion of feasible solutions is greater than 0.2 + 0.05: too many feasible solutions in the population, decrease the penalty weight
        origin_PT = Penalty_all(1,1);
        origin_PC = Penalty_all(1,2);
        origin_PD = Penalty_all(1,3);
        %TTTTTTTTTTTTTTTTTTTTTTT
        if fractionFeasible_T<=par_hgs.targetFeasible - 0.05
            origin_PT = Penalty_all(1,1);
            Penalty_all(1,1) = min(100000, Penalty_all(1,1)*penaltyScaleFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_T(pen_adjust) = (infeasiblePop.penalty_T(pen_adjust)/origin_PT)*Penalty_all(1,1);
            end

        elseif fractionFeasible_T>=par_hgs.targetFeasible + 0.05
            Penalty_all(1,1) = max(0.1, Penalty_all(1,1)*penaltyDecreaseFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_T(pen_adjust) = (infeasiblePop.penalty_T(pen_adjust)/origin_PT)*Penalty_all(1,1);
            end
        end
        %CCCCCCCCCCCCCCCCCCCCCCC
        if fractionFeasible_C<=par_hgs.targetFeasible - 0.05
            Penalty_all(1,2) = min(100000, Penalty_all(1,2)*penaltyScaleFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_C(pen_adjust) = (infeasiblePop.penalty_C(pen_adjust)/origin_PC)*Penalty_all(1,2);
            end
        elseif fractionFeasible_C>=par_hgs.targetFeasible + 0.05
            Penalty_all(1,2) = max(0.1, Penalty_all(1,2)*penaltyDecreaseFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_C(pen_adjust) = (infeasiblePop.penalty_C(pen_adjust)/origin_PC)*Penalty_all(1,2);
            end
        end
        %DDDDDDDDDDDDDDDDDDDDDDD
        if fractionFeasible_D<=par_hgs.targetFeasible - 0.05
            Penalty_all(1,3) = min(100000, Penalty_all(1,3)*penaltyScaleFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_D(pen_adjust) = (infeasiblePop.penalty_D(pen_adjust)/origin_PD)*Penalty_all(1,3);
            end
        elseif fractionFeasible_D>=par_hgs.targetFeasible + 0.05
            Penalty_all(1,3) = max(0.1, Penalty_all(1,3)*penaltyDecreaseFactor);
            for pen_adjust = 1:numel(infeasiblePop.ID)
                infeasiblePop.penalty_D(pen_adjust) = (infeasiblePop.penalty_D(pen_adjust)/origin_PD)*Penalty_all(1,3);
            end
        end

        % Update the objective value of infeasible solutions (due to changes in penalty coefficients)
        infeasiblePop.cost_Total = infeasiblePop.penalty_D + infeasiblePop.penalty_C + infeasiblePop.penalty_T ...
            + infeasiblePop.distance_Total;

        % Update the order of infeasible solutions in the infeasiblePop population (due to changes in penalty coefficients)
        if numel(infeasiblePop.ID) > 1

            infeasiblePop = infeasiblePop_updateBiasedFitnesses(infeasiblePop,vrp,par_hgs);
            % For identical solutions, newer ones with larger indices come first
            infeasiblePop.ID = -infeasiblePop.ID;
            infeasiblePop = sortrows(infeasiblePop, {'cost_Total', 'ID'});
            infeasiblePop.ID = -infeasiblePop.ID;
            getvrpdistable = zeros(numel(infeasiblePop.ID),numel(infeasiblePop.ID));
            for getvrpdis = 1:numel(infeasiblePop.ID)
                getvrpdistable(getvrpdis,:) = [infeasiblePop.brokenDist{getvrpdis}(1:getvrpdis-1),0,infeasiblePop.brokenDist{getvrpdis}(getvrpdis:end)];
            end
            getvrpdistable(:,end+1:par_hgs.popSizeLambda+par_hgs.popSizeMu+1) = 0;
            vrp.ALL_brokenDIS = getvrpdistable;
            infeasiblePop = removevars(infeasiblePop, 'brokenDist');
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Main Loop END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Adjust the total distance, keeping two decimal places in the distance table.
for distan=1:numel(vrp.distance_table(:,1))
    for distan2 = 1:numel(vrp.distance_table(1,:))
        vrp.distance_table(distan,distan2) = 100*vrp.distance_table(distan,distan2);
        vrp.distance_table(distan,distan2) = floor(vrp.distance_table(distan,distan2));
        vrp.distance_table(distan,distan2) = vrp.distance_table(distan,distan2)/100;
    end
end
Result = 0;
if bestSolOverall.IsFeasible==0
    Result = 99999;
else
    for disi=1:numel(bestSolOverall.chromR_move{1,1})
        a=[1,bestSolOverall.chromR_move{1,1}{disi}'+1,1];
        for disj = 1:numel(a)-1
            Result = Result + vrp.distance_table(a(disj),a(disj+1));
        end
        sol_table.cost_Total(bestSolOverall.ID) = Result;
        bestSolOverall.cost_Total = Result;
        Resulttime = bestSolOverall.time;
    end
end

end