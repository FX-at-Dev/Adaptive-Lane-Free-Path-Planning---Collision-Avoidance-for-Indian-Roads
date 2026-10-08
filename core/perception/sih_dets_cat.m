function a = sih_dets_cat(a, b)
%SIH_DETS_CAT Append detection set b to a (both in the form sih_dets_empty describes).
n = numel(a.x);
m = numel(b.x);
if m == 0
    return;
end
a.x(n+1:n+m,1) = b.x(:);
a.y(n+1:n+m,1) = b.y(:);
a.R(:,:,n+1:n+m) = b.R;
a.class(n+1:n+m,1) = b.class(:);
a.sensor(n+1:n+m,1) = b.sensor(:);
a.truth_id(n+1:n+m,1) = b.truth_id(:);
a.box(n+1:n+m,:) = local_get(b, 'box', NaN(m, 3));
a.box_full(n+1:n+m,1) = local_get(b, 'box_full', false(m, 1));
a.origin(n+1:n+m,:) = local_get(b, 'origin', NaN(m, 2));
a.road_psi(n+1:n+m,1) = local_get(b, 'road_psi', NaN(m, 1));
a.surface(n+1:n+m,1) = local_get(b, 'surface', false(m, 1));
end

function v = local_get(b, f, dflt)
if isfield(b, f) && size(b.(f), 1) == size(dflt, 1)
    v = b.(f);
else
    v = dflt;
end
end
