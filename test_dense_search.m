% Main script
% Example to solve different multi-modal registration problems
% with the method proposed in [1], and the previous approaches in [2,3]
% 
% Related work:
% [1] Briales, J., & Gonzalez-Jimenez, J. "Convex Global 3D Registration with Lagrangian Duality." In CVPR 2017
% [2] Olsson, C., & Eriksson, A. "Solving quadratically constrained geometrical problems using lagrangian duality." In ICPR 2008.
% [3] Olsson, C., Kahl, F., & Oskarsson, M. "Branch-and-Bound Methods for Euclidean Registration Problems." In TPAMI 2009.

clear all  clc;

% choose the problem to solve
problemType = 'random';
% problemType = 'SpaceStation';
% problemType = 'RubikCube';

% Problem generation
% -------------------------------------------------------------------------
switch problemType
  case 'random'
    problem.sim = 1;
    
    
    % create random problem
    % number of correspondences: [nr points, nr lines, nr planes]
    problem.m = [0 0 10];  % [0 0 7]; [2 4 7]
    % noise in the correspondences
    problem.noise = 0;
    % size of the random scene
    scene_radius = 10;
    
    
    [correspondences, gt_T, corrupt_cost] = ...
        rand_registration( problem.m, problem.noise, scene_radius, ...
        problem.sim );
    
  case 'SpaceStation'
    % registration data from Space Station [2,3]
    [c_p,c_l,c_pl] = correspondences_SpaceStation( );
    correspondences = [c_p,c_l,c_pl];
    
  case 'RubikCube'
    % registration data from Rubik cube [2,3]
    [c_p,c_l,c_pl] = correspondences_RubikCube( );
    correspondences = [c_p,c_l,c_pl];
    
  otherwise
    error('Unknown problem type')
end

% Model the problem as a compressed quadratic form in R only
% -------------------------------------------------------------------------
% Compute equivalent compressed quadratic form
% q = compress_quadData( correspondences );
% sv = svd(q.Q_);
% Sanity check: Sum of costs and compressed quad form must be equivalent
% abs( q.eval(vec(gt_T)) - sum(cost(correspondences,gt_T)) )

% Marginalize the quadratic function wrt translation using Schur complement
% t_idxs = 10:12;
% [q_margin, A] = marginalize(q,t_idxs);

% Solve the problem with our method
% ------------------------------------------------------------------------
% scale_list = [gt_T.s];

opts.solver = 2; % 1 bisection, 2 dense-search

if problem.sim
    
    if opts.solver == 1
        bisection_t = struct('s', [], 'sim', [], 'H', [], 'dstar', [], ...
            'f', [], 'sum_cost', [], 'costs', [], 'centroid_t', []);
        bisection_t(1).s = 0.01;
        bisection_t(2).s = 5;
        opts.max_iter = 30;
        opts.cost_threshold = 1e-4;
        opts.quite = 1;
        opts.show_statistic = 1;
        opts.draw_results = 0;
        disp("Computing...")
        fprintf("Ground truth scaling is %d\n", gt_T.s)
        [out_t, bisection_t] = solveByBisection(...
            opts, correspondences, bisection_t);
    elseif opts.solver == 2
        dense_search_t.s_min = 0.1;
        dense_search_t.s_interval = 0.05;
        dense_search_t.s_max = 5;
        opts.max_iter = 200;
        opts.cost_threshold = 1e-3;
        opts.quite = 1;
        opts.show_statistic = 1; % 0: no outputs, 1: rough outputs 2: detailed
        opts.draw_results = 0;
        [res_t, dense_search_t] = solveByDenseSearch(...
            opts, correspondences, dense_search_t);
        [~, k] = min([res_t(:).f]);
        out_t = res_t(k);
    end
else
    % Compute equivalent compressed quadratic form
    q = compress_quadData( correspondences );
    [out_t.R, out_t.t, out_t.dstar, times] = method_RCQP(...
        correspondences, 'header_all');
    H = SimPose(out_t.t, out_t.R, 1);
    f = q.eval(vec(H));
    gap = (f - out_t.dstar) / out_t.dstar;
    disp("========================================")
    fprintf('Optimality gap is f^star-d^star=%E\n',gap);
    disp("========================================")
end


% scale_list = 1:100;
% bisection_a = 0;
% bisection_b = 10;
%  
% cost_pre = norm(correspondences.cost(SimPose()));
% pick_color = getColors(length(scale_list));
% [axes_h, fig_h] = createFigHandleWithNumber(1,10,"Debug",1,1);
% for k = 1:length(scale_list)
%     for i = 1:size(correspondences,2)
%         plot3(axes_h(1), ...
%             correspondences(i).model.x(1), ...
%             correspondences(i).model.x(2), ...
%             correspondences(i).model.x(3), ...
%             'color', pick_color{k}, 'Marker', 'o');
%     end
%     scaling = (bisection_a + bisection_b)/ 2;
%     scaling = gt_T.s;
%     corrected_points = [correspondences.point];
%     transform([correspondences.point], ...
%         SimPose(zeros(3,1), eye(3), scaling));
% 
%     
%     q = compress_quadData( correspondences );
%     [R,t,dstar,times] = method_RCQP( correspondences, 'header_all' );
%     H = SimPose(t,R, 1);
%     f = q.eval(vec(Pose(t,R)));
%     gap = (f-dstar)/dstar;
%     disp("========================================")
%     fprintf('Optimality gap is f^star-d^star=%E\n',gap);
%     disp("========================================")
%     if dstar < cost_pre
%         disp("---------------------------")
%         corrected_points = [correspondences.point];
% %         corrected_points(i).x
% %         correspondences(i).point.x
%         transform([correspondences.point], ...
%             SimPose(zeros(3,1), eye(3), 1/scaling));
% %         corrected_points(i).x
% %         correspondences(i).point.x
% %         corrected_points = transform([correspondences.point], H);
%         
%         % update a and b
% %         bisection_a = 
% %         bisection_b = 
%         cost_pre = dstar;
%     end
%     for i = 1:size(correspondences,2)
%         plot3(axes_h(1), ...
%             correspondences(i).point.x(1), ...
%             correspondences(i).point.x(2), ...
%             correspondences(i).point.x(3), ...
%             'color', pick_color{k}, 'Marker', '*');
%     end
% end







%%
% H = SimPose(t,R, gt_T.s);
[axes_h, fig_h] = createFigHandleWithNumber(2,1,"simRCQP",1,1);


if problem.sim && opts.solver == 1
    % initial status
    cur_axes = 1;
    cur_fig =1;
    opts.plotting.model = 0;
    opts.plotting.corrected_source = 0;
    opts.plotting.corrected2target = 0;
    opts.plotting.source2target = 1;
    opts.plotting.source_points = 1;
    title_txt = "Optimization cost and scaling is " ...
            + "(" + num2str(abs(out_t.f)) ...
            + ", " + num2str(out_t.s) + ")";
    plotCorrespondences(opts, axes_h, cur_axes, ...
        out_t.centroid_t.point, ...
        correspondences, out_t.sim, out_t.s, title_txt)
    
    % results
    cur_axes = 2;
    cur_fig = 2;
    opts.plotting.model = 0;
    opts.plotting.corrected_source = 1;
    opts.plotting.corrected2target = 1;
    opts.plotting.source2target = 0;
    opts.plotting.source_points = 0;
    title_txt = "Optimization cost and scaling is " ...
            + "(" + num2str(abs(out_t.f)) ...
            + ", " + num2str(out_t.s) + ")";
    plotCorrespondences(opts, axes_h, cur_axes, ...
        out_t.centroid_t.point, ...
        correspondences, out_t.sim, out_t.s, title_txt)
    gt_T.sim
    out_t.sim.sim
    fprintf("Noise level: %.2f\n", problem.noise)
    fprintf("Geodestic: %.3f\n", norm(gt_T.sim \ out_t.sim.sim))
elseif problem.sim && opts.solver == 2
    cur_axes = 1;
    cur_fig =1;
    opts.plotting.model = 0;
    opts.plotting.corrected_source = 0;
    opts.plotting.corrected2target = 0;
    opts.plotting.source2target = 1;
    opts.plotting.source_points = 1;
    title_txt = "Optimization cost and scaling is " ...
            + "(" + num2str(abs(out_t.f)) ...
            + ", " + num2str(out_t.s) + ")";
    plotCorrespondences(opts, axes_h, cur_axes, ...
        [], ...
        correspondences, out_t.sim, out_t.s, title_txt)
    
    
    cur_axes = 2;
    cur_fig = 2;
    s_v = [res_t(:).s];
    d_star_v = [res_t(:).dstar];
    f_v = [res_t(:).f];

    scatter(axes_h(cur_axes), s_v, d_star_v, 'bo')
    scatter(axes_h(cur_axes), s_v, f_v, 'g*')
    scatter(axes_h(cur_axes), gt_T.s, 0, 100,'r^', 'fill')
    viewCurrentPlot(axes_h(cur_axes), [], [], 0)
else
    plotCorrespondences(opts, axes_h, cur_axes, ...
    [], ...
    correspondences, H, [], "test")
    gt_T.T
    H.sim
    fprintf("Noise level: %.2f\n", problem.noise)
    fprintf("Geodestic: %.3f\n", norm(gt_T.T \ H.sim))
end
% plotCorrespondences(axes_h, cur_axes, ...
%         out_t.centroid_t.point, ...
%         correspondences, out_t.sim, out_t.s, title_txt)