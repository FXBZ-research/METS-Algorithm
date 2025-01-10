function [pc_now,afs_time_delay] = AFSdelay_new(time_afs,time_v,T_max,T_Afs,C_Afs,afs_time_delay)
if numel(time_afs) == 0
    pc_now = 0;
    return
end
time_V_shifting = max(T_max - time_v,0);
afs_time_delay = time_afs;
conflict_table = zeros(numel(time_afs),numel(time_afs));
[pc_now,afs_time_delay] = AFSdelay_recursion(time_afs,time_v,T_max,T_Afs,C_Afs,time_V_shifting,afs_time_delay,conflict_table);
end