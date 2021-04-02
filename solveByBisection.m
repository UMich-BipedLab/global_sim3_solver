% clc, clear all
% load("correspondences.mat")
% bisection_t = struct('s', [], 'sim', [], 'H', [], 'dstar', [], 'f', [], ...
%     'sum_cost', [], 'costs', [], 'centroid_t', []);
% bisection_t(1).s = 0.1;
% bisection_t(2).s = 10;
% opts.max_iter = 20;
% opts.cost_threshold = 1e-3;
% opts.quite = 1;
% opts.show_statistic = 1; % 0: no outputs, 1: rough outputs 2: detailed
% opts.draw_results = 0;
% [out_t, bisection_t] = solveByBisection_t(opts, correspondences, bisection_t);


function [out_t, bisection_t] = solveByBisection(...
    opts, correspondences, bisection_t, centroid)
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
    cost = 1e3;
    grad_s = 1e3;
    pre_s = 1e3;
    if ~exist('centroid', 'var') || isempty(centroid)
        centroid = [];
    end
    
    while iter < opts.max_iter && ...
          abs(cost) > opts.cost_threshold && ...
          grad_s > opts.s_grad_threshold
%         centroid = Point();
%         centroid_t = computeCentroid(correspondences);  
%         centroid = Point(-(centroid_t.model.x -  centroid_t.point.x));
        
%         centroid.x
    
%         disp("====================== Before ========================")
%         disp("------------------- s -------------------")
%         bisection_t.s
        
        bisection_t(1) = solveRCQPAndSignedCost(opts, ...
            correspondences, centroid, bisection_t(1).s);
        
        bisection_t(2) = solveRCQPAndSignedCost(opts, ...
            correspondences, centroid, bisection_t(2).s);
        
        
        s = (bisection_t(1).s + bisection_t(2).s)/ 2;
        new_t = solveRCQPAndSignedCost(opts, ...
            correspondences, centroid, s);
        grad_s = abs(pre_s - s);
%         new_t.H
%         trans = new_t.H.t
%         trans_s = trans ./ s
%         [correspondences, bisection_t(1).H, ...
%             bisection_t(1).dstar, bisection_t(1).f] = ...
%             solveRCQP(opts, correspondences, bisection_t(1).s);
%         bisection_t(1).cost = sum(correspondences.signed_cost(...
%             SimPose(...
%             bisection_t(1).H.t, bisection_t(1).H.R, bisection_t(1).s), ...
%             centroid));
        
%         [correspondences, bisection_t(2).H, ...
%             bisection_t(2).dstar, bisection_t(2).f]  = ...
%             solveRCQP(opts, correspondences, bisection_t(2).s);
%         bisection_t(2).cost = sum(correspondences.signed_cost(...
%             SimPose(...
%             bisection_t(2).H.t, bisection_t(2).H.R, bisection_t(2).s), ...
%             centroid));
% 
%         s = (bisection_t(1).s + bisection_t(2).s)/ 2;
%         [correspondences, H, dstar, f] = solveRCQP(opts, correspondences, s);
%         cost = sum(correspondences.signed_cost(...
%             SimPose(H.t, H.R, s), centroid));
        printConvexSolverResults(opts, iter, new_t, bisection_t)       
        
        %% plots
        if opts.draw_results
            [axes_h, ~] = createFigHandleWithNumber(3, 20, ...
                "Debug signed distance",1,1);
            cur_axes = 1;
            plotCorrespondences(axes_h, cur_axes, ...
                bisection_t(1).centroid_t.point, ...
                correspondences, bisection_t(1).H, bisection_t(1).s, "s1: ")   

            cur_axes = 2;
            plotCorrespondences(axes_h, cur_axes, ...
                bisection_t(2).centroid_t.point, ...
                correspondences, bisection_t(2).H, bisection_t(2).s, "s2: ")

            cur_axes = 3;
            plotCorrespondences(axes_h, cur_axes, ...
                new_t.centroid_t.point, ...
                correspondences, new_t.H, new_t.s, "New s: ")
        end
            
        %% update new s
%         if new_t.sum_signed_cost < 0
%             % new data
%             bisection_t(2).s = s;
%             bisection_t(2).sum_signed_cost = new_t.sum_signed_cost;
% 
%         else
%             % new data
%             bisection_t(1).s = s;
%             bisection_t(1).sum_signed_cost = new_t.sum_signed_cost;
%         end
        if bisection_t(1).sum_cost < bisection_t(2).sum_cost
            % new data
            bisection_t(2).s = s;
            bisection_t(2).sum_cost = new_t.sum_cost;

        else
            % new data
            bisection_t(1).s = s;
            bisection_t(1).sum_cost = new_t.sum_cost;
        end
        out_t = new_t;
%         out_t.s = new_t.s;
%         out_t.sim = new_t.sim;
%         out_t.H = new_t.H;
%         out_t.dstar = new_t.dstar;
%         out_t.f = new_t.f;
        cost = new_t.f;
        iter = iter + 1;
    end
end