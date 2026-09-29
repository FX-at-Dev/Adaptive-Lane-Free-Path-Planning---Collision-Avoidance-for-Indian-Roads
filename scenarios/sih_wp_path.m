function wp = sih_wp_path(rp, s_list, d_list)
%SIH_WP_PATH Waypoint polyline through a list of corridor stations and offsets.
%   wp = SIH_WP_PATH(rp, s_list, d_list) returns an Nx2 world-frame polyline.
%   Drifting d along the list is how an agent is given weak lane discipline:
%   it wanders across the carriageway as it travels rather than holding a line.
if numel(s_list) ~= numel(d_list)
    error('sih_wp_path:size', 's_list and d_list must be the same length.');
end
wp = zeros(numel(s_list), 2);
for k = 1:numel(s_list)
    [wp(k,1), wp(k,2)] = sih_wp_at(rp, s_list(k), d_list(k));
end
end
