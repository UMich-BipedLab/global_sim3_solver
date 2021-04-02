function [out_t,  data_t, res_t] = solveByConvexSolvers(...
    opts, data, plane, ring, num_targets)
    data_t.correspondences = createCorrespondences(...
        data, plane, ring, num_targets);
    
%     fprintf("Original Cost: %f\n", original_cost)
    if opts.solver == 1
        data_t.bisection_t = loadBisectionStruct();
        data_t.bisection_t(1).s = opts.s_lower;
        data_t.bisection_t(2).s = opts.s_upper;
        centroid = Point();
        [out_t, data_t.bisection_t] = solveByBisection(...
            opts, data_t.correspondences, data_t.bisection_t, centroid);
        res_t = []; % consistent with dense search
    elseif opts.solver == 2
        data_t.dense_search_t.s_min = opts.s_lower;
        data_t.dense_search_t.s_interval = opts.s_interval;
        data_t.dense_search_t.s_max = opts.s_upper;
        [res_t, ~] = solveByDenseSearch(...
            opts, data_t.correspondences, data_t.dense_search_t);
        [~, k] = min([res_t(:).f]);
        out_t = res_t(k);
        out_t.sum_cost = out_t.f;
    else
        error("unsurrported solver option %i", opts.solver)
    end
    out_t.original_cost = sum(abs(data_t.correspondences.cost(Pose())));
end