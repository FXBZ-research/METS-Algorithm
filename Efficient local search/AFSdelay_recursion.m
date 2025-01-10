function [pc_now,afs_time_delay] = AFSdelay_recursion(time_afs,time_v,T_max,T_Afs,C_Afs,time_V_shifting,afs_time_delay,conflict_table)
[pc_now] = get_pc_now(afs_time_delay,T_max,time_v,time_afs,T_Afs,C_Afs);
% Perform backtracking search
delay_over = 1;
while delay_over
    delay_over = 0;
    for i=1:numel(time_afs)-1
        for ii=i+1:numel(time_afs)
            if afs_time_delay(i) >= afs_time_delay(ii)  % The one with the greater (later or longer) time is assigned as 2
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
            if t1+T_Afs <= t2  % Check for conflicts, specifically whether refueling times overlap
                continue
            end
            % When two conditions are met, there is no solution: both 1 and 2 are not feasible.
            if t1 + T_Afs - t2 > q2 && t2 + T_Afs - t1 > q1     
                conflict_table(i,ii) = conflict_table(i,ii)+1;
                conflict_table(ii,i) = conflict_table(ii,i)+1;
                continue
            end
            % When only condition 1 is satisfied: the earliest end time of 1 is later than the latest start time of 2, 
            % meaning even if 2 waits until the very end, it cannot refuel after 1.
            % At this point, we can only delay 1, as delaying 2 cannot resolve the issue.
            % Delay 1 to the earliest time 2 finishes refueling.
            if t1 + T_Afs - t2 > q2
                delaytime = t2 + T_Afs - t1;
                t1 = t1 + delaytime;
                if t3 == 1
                    afs_time_delay(ii) = t1;
                    time_V_shifting(ii) = time_V_shifting(ii) - delaytime;
                    i=1;
                    ii=1;
                elseif t3 == 2
                    afs_time_delay(i) = t1;
                    time_V_shifting(i) = time_V_shifting(i) - delaytime;
                    i=1;
                    ii=1;
                end
                continue
            end
            % When only condition 2 is satisfied: the earliest end time of 2 is later than the latest start time of 1,
            % meaning even if 1 waits until the very end, it cannot refuel after 2.
            % At this point, we can only delay 2, as delaying 1 cannot resolve the issue.
            if t2 + T_Afs - t1 > q1 
                delaytime = t1 + T_Afs - t2;
                t2 = t2 + delaytime;
                if t3 == 1
                    afs_time_delay(i) = t2;
                    time_V_shifting(i) = time_V_shifting(i) - delaytime;
                    i=1;
                    ii=1;
                elseif t3 == 2
                    afs_time_delay(ii) = t2;
                    time_V_shifting(ii) = time_V_shifting(ii) - delaytime;
                    i=1;
                    ii=1;
                end
                continue
            end
            % Perform backtracking search when the conditions are met
            if t1 + T_Afs - t2 > q2 && t2 + T_Afs - t1 > q1
                % Recursively call the backtracking function
                [afs_time_delay,time_V_shifting,pc_now] = backtrack(time_afs, time_v, T_max, T_Afs, C_Afs, afs_time_delay, time_V_shifting, i, ii,t1,t2,t3,pc_now);
                delay_over = 1;% Set a flag to indicate that backtracking was performed
                break;
            end
        end
        if delay_over
            break;
        end
    end
end
[pc_now] = get_pc_now(afs_time_delay,T_max,time_v,time_afs,T_Afs,C_Afs);
end

function [original_afs_time_delay,original_time_V_shifting,pc_original] = backtrack(time_afs, time_v, T_max, T_Afs, C_Afs, afs_time_delay, time_V_shifting, i, ii,t1,t2,t3,pc_now)
% Recursive backtracking function to attempt finding the optimal solution
% Obtain a copy of the current state to restore it during backtracking
original_time_V_shifting = time_V_shifting;
pc_original = pc_now;
% Attempt to delay the time for vehicle 1
delaytime = t2 + T_Afs - t1;
t1 = t1 + delaytime;
if t3 == 1
    afs_time_delay(ii) = t1;
    time_V_shifting(ii) = time_V_shifting(ii) - delaytime;
    i=1;
    ii=1;
elseif t3 == 2
    afs_time_delay(i) = t1;
    time_V_shifting(i) = time_V_shifting(i) - delaytime;
    i=1;
    ii=1;
end
% Recursive call
[pc_now] = AFSdelay_recursion(time_afs,time_v,T_max,T_Afs,C_Afs,time_V_shifting,afs_time_delay,conflict_table);
% If the new penalty function is better, update the current state
if pc_now < pc_original
    original_afs_time_delay = afs_time_delay;
    original_time_V_shifting = time_V_shifting;
    pc_original = pc_now;
end
% Restore the state and attempt to delay the time for vehicle 2
afs_time_delay = original_afs_time_delay;
time_V_shifting = original_time_V_shifting;
pc_now = pc_original;
% Attempt to delay the time for vehicle ii
delaytime = t1 + T_Afs - t2;
t2 = t2 + delaytime;
if t3 == 1
    afs_time_delay(i) = t2;
    time_V_shifting(i) = time_V_shifting(i) - delaytime;
    i=1;
    ii=1;
elseif t3 == 2
    afs_time_delay(ii) = t2;
    time_V_shifting(ii) = time_V_shifting(ii) - delaytime;
    i=1;
    ii=1;
end
% Recursive call
[pc_now] = AFSdelay_recursion(time_afs,time_v,T_max,T_Afs,C_Afs,time_V_shifting,afs_time_delay,conflict_table);
% If the new penalty function is better, update the current state
if pc_now < pc_original
    original_afs_time_delay = afs_time_delay;
    original_time_V_shifting = time_V_shifting;
    pc_original = pc_now;
end

end

