function [q, qd, qdd] = hermite_cubic_traj(ref_p_list, ref_t_list, t)

% find time indice
knots_idx = 1;

while (t>=ref_t_list(knots_idx))
    knots_idx = knots_idx + 1;
    if (knots_idx == size(ref_t_list,2))
        break;
    end
end

knot1_idx = knots_idx-1;
knot2_idx = knots_idx;

[q, qd, qdd] = hermite_cubic_knot(t, ref_t_list(knots_idx-1), ref_p_list(:,knots_idx-1), zeros(size(ref_p_list(:,1))), ...
    ref_t_list(knots_idx), ref_p_list(:,knots_idx), zeros(size(ref_p_list(:,1))));

end