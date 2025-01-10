function [pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm] = deleteAFS_delta(delete_idx,node_location,pd_v,pt_v,time_v,time_afs,distance_pre,distance_su,pm_now,pm,wm)
% deleteAFS function: Triggered only when, after u or ux leaves delete_idx, 
% the route delete_idx contains only the AFS. 
% Handles the deletion of this route and the corresponding changes.
% Sort the input according to the order of usage.

node_deleteAfs = delete_idx + sum(node_location~=100);
if numel(node_deleteAfs) ~= 1
    disp "deleteAFS_delta"
end
pd_v(delete_idx,:) = []; 
pt_v(delete_idx) = [];
time_v(delete_idx) = [];
time_afs(delete_idx) = [];
distance_pre(delete_idx) = [];
distance_su(delete_idx) = [];
pm_now = max(pm_now-wm,0);
end