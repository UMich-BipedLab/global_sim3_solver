function out_t = solveRCQPAndSignedCost(opts, correspondences, centroid, s)
    out_t.s = s;
    [~, out_t.centroid_t, out_t.sim, out_t.H, ...
        out_t.dstar, out_t.f, out_t.tigt_gap] = ...
        solveRCQP(opts, correspondences, s);
    
    if isempty(centroid)
        centroid = out_t.centroid_t.point;
    end
    out_t.gap = out_t.f - out_t.dstar;
    out_t.signed_cost = correspondences.signed_cost(...
            SimPose(out_t.H.t, out_t.H.R, s), centroid);
    out_t.sum_signed_cost = sum(out_t.signed_cost);
    out_t.sum_cost = sum(abs(out_t.signed_cost));
end