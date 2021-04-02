% clc, clear all
% % load("point2point_correspondences.mat")
% load("point2plane_correspondences.mat")
% dense_search_t.s_min = 0.1;
% dense_search_t.s_interval = 0.05;
% dense_search_t.s_max = 5;
% opts.max_iter = 200;
% opts.cost_threshold = 1e-3;
% opts.quite = 1;
% opts.show_statistic = 1; % 0: no outputs, 1: rough outputs 2: detailed
% opts.draw_results = 0;
% [out_t, bisection_t] = solveByDenseSearch_t(opts, correspondences, dense_search_t);
%%
% [axes_h, ~] = createFigHandleWithNumber(3, 20, ...
%                 "Scaline convexity",1,1);
% s_v = [out_t(:).s];
% d_star_v = [out_t(:).dstar];
% f_v = [out_t(:).f];
% 
% scatter(axes_h(1), s_v, d_star_v, 'bo')
% scatter(axes_h(1), s_v, f_v, 'g*')
% scatter(axes_h(1), gt_T.s, 0, 100,'r^', 'fill')

function [out_t, dense_search_t] = solveByDenseSearch(...
    opts, correspondences, dense_search_t)
%     if ~exist('centroid', 'var') || isempty(centroid)
%         centroid_t = computeCentroid(correspondences);  
%         centroid = Point(centroid_t.model.x -  centroid_t.point.x);
%     end
    
%     gt_T.t = [-0.12423
%       -2.5415
%       0.27721];
%     gt_T.R = [-0.66012     -0.74424      0.10168
%       0.42388     -0.48083     -0.76755
%       0.62013     -0.46358      0.63288];

    iter = 1;
    s = dense_search_t.s_min;
    cost = 1e3;
    out_t(opts.max_iter) = struct();
    if opts.draw_results
        [axes_h, ~] = createFigHandleWithNumber(3, 20, ...
            "Dense Search",1,1);
    end
    while iter < opts.max_iter && ...
          abs(cost) > opts.cost_threshold && ...
          s <= dense_search_t.s_max
        out_t(iter).s = s;
        [~, out_t(iter).centroid_t, out_t(iter).sim, out_t(iter).H, ...
            out_t(iter).dstar, out_t(iter).f, out_t(iter).gap] = ...
                solveRCQP(opts, correspondences, s);

        printConvexSolverResults(opts, iter, out_t(iter), [])       
%                 out_t(iter).H
%         trans = out_t(iter).H.t
%         trans_s = trans ./ s
        %% plots
        if opts.draw_results
            cur_axes = 1;
            cla(axes_h(cur_axes))

            opts.plotting.model = 0;
            opts.plotting.corrected_source = 1;
            opts.plotting.corrected2target = 1;
            opts.plotting.source2target = 0;
            opts.plotting.source_points = 0;
%             title_txt = "Optimization cost and scaling is " ...
%                     + "(" + num2str(abs(out_t.f)) ...
%                     + ", " + num2str(out_t.s) + ")";
            plotCorrespondences(opts, axes_h, cur_axes, ...
                out_t(iter).centroid_t.point, ...
                correspondences, out_t(iter).H, out_t(iter).s, "s: ")
            
        end
            
        %% update new s
        s = s + dense_search_t.s_interval;
        iter = iter + 1;
    end
    if opts.draw_results
        cur_axes = 3;
        s_v = [out_t(:).s];
        d_star_v = [out_t(:).dstar];
        f_v = [out_t(:).f];

        scatter(axes_h(cur_axes), s_v, d_star_v, 'bo')
        scatter(axes_h(cur_axes), s_v, f_v, 'g*')
%         scatter(axes_h(2), gt_T.s, 0, 100,'r^', 'fill')
        viewCurrentPlot(axes_h(cur_axes), "Convexity", [], 0)
    end
end