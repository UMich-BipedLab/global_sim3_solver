% Main script
% Example to solve different multi-modal registration problems
% with the method proposed in [1], and the previous approaches in [2,3]
% 
% Related work:
% [1] Briales, J., & Gonzalez-Jimenez, J. "Convex Global 3D Registration with Lagrangian Duality." In CVPR 2017
% [2] Olsson, C., & Eriksson, A. "Solving quadratically constrained geometrical problems using lagrangian duality." In ICPR 2008.
% [3] Olsson, C., Kahl, F., & Oskarsson, M. "Branch-and-Bound Methods for Euclidean Registration Problems." In TPAMI 2009.

clear all, clc;

% choose the problem to solve
% problemType = 'random';
% problemType = 'SpaceStation';
problemType = 'RubikCube';

problem.sim = 1; 


opts.noise_levels = linspace(0, 2, 2);
opts.noise_levels = 0;
opts.solver = 2; % 1 bisection, 2 dense-search



data_t(length(opts.noise_levels)) = struct();
colors = getColors(length(opts.noise_levels));

[axes_h, fig_h] = createFigHandleWithNumber(5, 1, "simRCQP", 1, 1, 12);
h = zeros(1,length(opts.noise_levels));
drawnow


% if ~strcmp(problemType, 'random')
%     problem.sim = 0;
% end

for i = 1:length(opts.noise_levels)
    % Problem generation
    % -------------------------------------------------------------------------
    problem.noise = opts.noise_levels(i);
    switch problemType
      case 'random'
        

        % create random problem
        % number of correspondences: [nr points, nr lines, nr planes]
        problem.m = [5 2 0];  % [0 0 7]; [2 4 7]

        % size of the random scene
        scene_radius = 10;
        
        % noise in the correspondences
        noise = problem.noise
        
        [data_t(i).correspondences, data_t(i).gt_T, data_t(i).corrupt_cost] = ...
                rand_registration( problem.m, problem.noise, scene_radius, ...
                problem.sim);


      case 'SpaceStation'
        % registration data from Space Station [2,3]
        [c_p,c_l,c_pl] = correspondences_SpaceStation( );
        data_t(i).correspondences = [c_p,c_l,c_pl];
        fprintf("Origianl Cost: %.04f\n", ...
            sum(abs(data_t(i).correspondences.cost(Pose()))))
        H_gt = load("SpaceStation_GT.mat");
%         if problem.sim
%             allPoints = [data_t(i).correspondences.point];
%             transform(allPoints, data_t(i).gt_T.H)
%             fprintf("GT Cost: %.04f\n", ...
%                 sum(abs(data_t(i).correspondences.cost(Pose()))))
%             data_t(i).gt_T = SimPose.rand();
%             allPoints = [data_t(i).correspondences.point];
%             transform(allPoints, inv(data_t(i).gt_T))
%             fprintf("Cost after applying Sim3: %.04f\n", ...
%                 sum(abs(data_t(i).correspondences.cost(Pose()))))
%         end
%             
      case 'RubikCube'
        % registration data from Rubik cube [2,3]
        [c_p,c_l,c_pl] = correspondences_RubikCube( );
        data_t(i).correspondences = [c_p,c_l,c_pl];
        H_gt = load("RubikCube_GT.mat");

      otherwise
        error('Unknown problem type')
    end
    if problem.sim && ...
       (strcmp(problemType, 'SpaceStation') || ...
       strcmp(problemType, 'RubikCube'))

        allPoints = [data_t(i).correspondences.point];
        transform(allPoints, H_gt.H)
        fprintf("GT Cost: %.04f\n", ...
            sum(abs(data_t(i).correspondences.cost(Pose()))))
        data_t(i).gt_T = SimPose.rand();
        allPoints = [data_t(i).correspondences.point];
        transform(allPoints, inv(data_t(i).gt_T))
        fprintf("Cost after applying Sim3: %.04f\n", ...
            sum(abs(data_t(i).correspondences.cost(Pose()))))
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

    if problem.sim
        if opts.solver == 1
            data_t(i).bisection_t = loadBisectionStruct();
            data_t(i).bisection_t(1).s = 0.01;
            data_t(i).bisection_t(2).s = 5;
            opts.max_iter = 30;
            opts.cost_threshold = 1e-4;
            opts.s_grad_threshold = 1e-2;
            opts.quite = 1;
            opts.show_statistic = 2;
            opts.draw_results = 0;
            disp("Computing...")
%             if isfield(data_t.gt_T, 's')
%                 fprintf("Ground truth scaling is %d\n", data_t(i).gt_T.s)
%             elseif isa(data_t.gt_T, 'SimPose')
%                 fprintf("Ground truth scaling is %d\n", data_t(i).gt_T.s)
%             end
            fprintf("Ground truth scaling is %d\n", data_t(i).gt_T.s)
            [out_t, data_t(i).bisection_t] = solveByBisection(...
                opts, data_t(i).correspondences, data_t(i).bisection_t);
            res_t = []; % consistent with dense search
        elseif opts.solver == 2
            data_t(i).dense_search_t.s_min = 0.1;
            data_t(i).dense_search_t.s_interval = 0.05;
            data_t(i).dense_search_t.s_max = 5;
            opts.max_iter = 200;
            opts.cost_threshold = 1e-3;
            opts.quite = 1;
            opts.show_statistic = 1; % 0: no outputs, 1: rough outputs 2: detailed
            opts.draw_results = 0;
            [res_t, data_t(i).dense_search_t] = solveByDenseSearch(...
                opts, data_t(i).correspondences, data_t(i).dense_search_t);
            [~, k] = min([res_t(:).f]);
            out_t = res_t(k);
        end
    else
        % Compute equivalent compressed quadratic form
        q = compress_quadData( data_t(i).correspondences );
        [out_t.R, out_t.t, out_t.dstar, times] = method_RCQP(...
            data_t(i).correspondences, 'header_all');
        data_t(i).H = SimPose(out_t.t, out_t.R, 1);
        data_t(i).f = q.eval(vec(data_t(i).H));
        data_t(i).gap = (data_t(i).f - out_t.dstar) / out_t.dstar;
        disp("========================================")
        fprintf('Optimality gap is f^star-d^star=%E\n', data_t(i).gap);
        disp("========================================")
    end



    %%
    cla(2)
    cla(1)
    data_t(i).res_t = [];
    data_t(i).out_t = [];
    data_t(i).geo_dis = [];
    data_t(i).fig_h = [];
    data_t(i).fig_txt = [];
    data_t(i) = plotConvexSolverResults(...
        opts, problem, axes_h, fig_h, colors, i, data_t(i), out_t, res_t); 
    %%
%     plotConvexity([], [], [], "Convexity", data_t(i), res_t);
end
if opts.solver == 2
    popCurrentFigure(2);
    scatter(axes_h(2), data_t(i).gt_T.s, 0, 100,'rd', 'fill')
    legend([data_t(:).fig_h],{data_t(:).fig_txt})
end
disp("Done")