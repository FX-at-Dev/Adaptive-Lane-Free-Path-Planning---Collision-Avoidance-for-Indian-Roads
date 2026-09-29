function psi = sih_wp_heading(rp, s)
%SIH_WP_HEADING Corridor heading at a station, wrapped to (-pi, pi].
s   = min(max(s, 0), rp.length);
psi = sih_wrap_pi(interp1(rp.s, rp.psi, s, 'linear'));
end
