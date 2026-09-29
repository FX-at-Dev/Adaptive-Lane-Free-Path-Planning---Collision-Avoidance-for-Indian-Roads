function blocked = sih_ray_blocked(ex, ey, tx, ty, bx, by, br)
%SIH_RAY_BLOCKED Test whether discs occlude the line of sight to a target.
%
%   blocked = SIH_RAY_BLOCKED(ex, ey, tx, ty, bx, by, br) returns true when the
%   segment from the sensor (ex,ey) to the target (tx,ty) passes within br of
%   any blocker centre (bx,by), considering only blockers that lie between the
%   sensor and the target.
%
%   Occlusion is what makes the tracker's coasting logic matter: a pedestrian
%   stepping out from behind a parked bus is invisible until very late, and a
%   planner tested without occlusion would look far better than it is.

blocked = false;
if isempty(bx)
    return;
end

dx = tx - ex;
dy = ty - ey;
seg_len2 = dx*dx + dy*dy;
if seg_len2 < 1e-12
    return;
end

bx = bx(:); by = by(:); br = br(:);

% Projection parameter of each blocker centre onto the sight line.
tpar = ((bx - ex) * dx + (by - ey) * dy) / seg_len2;

% Only blockers strictly between the sensor and the target can occlude. The
% margin at the far end stops an agent from occluding itself.
cand = tpar > 0.02 & tpar < 0.98;
if ~any(cand)
    return;
end

tp = tpar(cand);
cxp = ex + tp * dx;
cyp = ey + tp * dy;

perp = sqrt((bx(cand) - cxp).^2 + (by(cand) - cyp).^2);
blocked = any(perp < br(cand));
end
