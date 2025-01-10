function [pc_now] = get_pc_now(afs_time_delay, T_max, time_v, time_afs, T_Afs, C_Afs)
% Using afs_time and refueling time T_afs, calculate the number of overlapping vehicles
% and the duration of overlaps under the limited capacity of the refueling station.
% This corresponds to PC in the penalty function.
afs_time_delay(afs_time_delay == 0)=[];
afs_time_end = afs_time_delay + T_Afs;
afs_time_end(afs_time_end == 0) = [];
c = sort([afs_time_delay', afs_time_end']);
time_during = diff(c); % Obtain each time interval (based on vehicle arrival and completion times)
% Use a vectorized method to calculate the number of overlaps
d = afs_time_delay - c(1:end-1);
overlap_matrix = eps < d + T_Afs & d <= 0;
% Obtain the number of overlaps in each time interval, i.e., the number of vehicles refueling simultaneously during that period
nb_fueling = sum(overlap_matrix, 1);
nb_fueling = max(nb_fueling - C_Afs, 0);
pc_now = max(nb_fueling * time_during', 0);
end
