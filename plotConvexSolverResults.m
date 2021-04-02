function data_t = plotConvexSolverResults(opts, problem, ...
        axes_h, fig_h, colors, iter, data_t, out_t, res_t)
    if isempty(problem)
        problem.sim = 1;
        problem.noise = -1;
    end
    
    if isempty(axes_h) || isempty(fig_h)
        [axes_h, fig_h] = createFigHandleWithNumber(4, 30, ...
            "simRCQP", 1, 1);
    end
    
    if isempty(colors)
        colors = getColors(length(1));
    end
    
    if isempty(iter)
        iter = 1;
    end
    
        
    if problem.sim && opts.solver == 1
        % initial status
        cur_axes = 1;
        cur_fig =1;
        opts.plotting.model = 1;
        opts.plotting.corrected_source = 0;
        opts.plotting.corrected2target = 0;
        opts.plotting.source2target = 0;
        opts.plotting.source_points = 1;
        title_txt = "Optimization cost and scaling is " ...
                + "(" + num2str(abs(out_t.f)) ...
                + ", " + num2str(out_t.s) + ")";
        plotCorrespondences(opts, axes_h, cur_axes, ...
            out_t.centroid_t.point, ...
            data_t.correspondences, out_t.sim, out_t.s, title_txt)
        drawnow

        % results
        cur_axes = 2;
        cur_fig = 2;
        opts.plotting.model = 1;
        opts.plotting.corrected_source = 1;
        opts.plotting.corrected2target = 0;
        opts.plotting.source2target = 0;
        opts.plotting.source_points = 0;
        title_txt = "Optimization cost and scaling is " ...
                + "(" + num2str(abs(out_t.f)) ...
                + ", " + num2str(out_t.s) + ")";
        plotCorrespondences(opts, axes_h, cur_axes, ...
            out_t.centroid_t.point, ...
            data_t.correspondences, out_t.sim, out_t.s, title_txt)
        
        fprintf("Noise level: %.2f\n", problem.noise)
        out_t.sim.sim
        if isfield(data_t, 'gt_T')
            data_t.gt_T.sim
            geo_dis = norm(data_t.gt_T.sim \ out_t.sim.sim);
            fprintf("Geodestic: %.3f\n", geo_dis)
            data_t.geo_dis = geo_dis;
        end
        data_t.out_t = out_t;
        
        
    elseif problem.sim && opts.solver == 2
        % initial 
        cur_axes = 1;
        cur_fig =1;
        opts.plotting.model = 1;
        opts.plotting.corrected_source = 0;
        opts.plotting.corrected2target = 0;
        opts.plotting.source2target = 0;
        opts.plotting.source_points = 1;
        if isfield(data_t, 'gt_T')
            title_txt = "Original scaling: " + num2str(data_t.gt_T.s);
        else
            title_txt = "Original";
        end
        plotCorrespondences(opts, axes_h, cur_axes, ...
            [], ...
            data_t.correspondences, out_t.sim, out_t.s, title_txt)
        drawnow
        
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
            [], ...
            data_t.correspondences, out_t.sim, out_t.s, title_txt)
        drawnow
        
        

        cur_axes = 3;
        cur_fig = 3;
        s_v = [res_t(:).s];
        d_star_v = [res_t(:).dstar];
        f_v = [res_t(:).f];

        data_t.fig_h = scatter(axes_h(cur_axes), s_v, d_star_v, [], ...
            colors{iter}, 'o');
        data_t.fig_txt = "noise = " + num2str(problem.noise);
%         h(i) = scatter(axes_h(cur_axes), s_v, d_star_v, [], ...
%             colors{i}, 'o', 'DisplayName', "noise = " + num2str(noise));
%         legend(h(i));
        
        scatter(axes_h(cur_axes), s_v, f_v, 'g*')
        scatter(axes_h(cur_axes), out_t.s, out_t.f, 200, 'b^')
        viewCurrentPlot(axes_h(cur_axes), "Convexity", [], 0)
        
        cur_axes = 4;
        cur_fig = 4;
        scatter(axes_h(cur_axes), problem.noise, out_t.s, 100, 'r*');
        viewCurrentPlot(axes_h(cur_axes), "Optimum vs Noise", [], 0)
        data_t.res_t = res_t;
        data_t.out_t = out_t;
        
        if isfield(data_t, 'gt_T')
            cur_axes = 5;
            cur_fig = 5;
            num_search = size(res_t, 2);
            if 1
                s_list = [];
                d_rotation = [];
                d_translation = [];
                for i = 1 : num_search
                    if isempty(res_t(i).s)
                        continue
                    end
                    d_rotation = [d_rotation, ...
                        norm(Log_SO3(res_t(i).H.R / data_t.gt_T.H.R))];
                    d_translation = [d_translation, ...
                        norm(res_t(i).H.t - data_t.gt_T.H.t)];
                    s_list = [s_list, res_t(i).s];
                end
                scatter3(axes_h(cur_axes), ...
                    s_list, d_rotation, d_translation, 100, 'r*');
                xlabel(axes_h(cur_axes), "s")
                ylabel(axes_h(cur_axes), "d\_rotation")
                zlabel(axes_h(cur_axes), "d\_translation")
            else
            end
        end
        
    else
        cur_axes = 1;
        opts.plotting.model = 0;
        opts.plotting.corrected_source = 0;
        opts.plotting.corrected2target = 0;
        opts.plotting.source2target = 1;
        opts.plotting.source_points = 1;
        plotCorrespondences(opts, axes_h, cur_axes, ...
        [], data_t.correspondences, data_t.H, [], "initial")
    
        if isfield(out_t, 's')
            title_txt = "Optimization cost and scaling is " ...
                + "(" + num2str(abs(data_t.f)) ...
                + ", " + num2str(out_t.s) + ")";
        else
            title_txt = "Optimization cost is " ...
                 + num2str(abs(data_t.f));
        end

        cur_axes = 2;
        opts.plotting.model = 0;
        opts.plotting.corrected_source = 1;
        opts.plotting.corrected2target = 1;
        opts.plotting.source2target = 0;
        opts.plotting.source_points = 0;
        plotCorrespondences(opts, axes_h, cur_axes, ...
        [], data_t.correspondences, data_t.H, [], title_txt)
%         data.gt_T.T
%         data.H.sim
%         fprintf("Noise level: %.2f\n", problem.noise)
%         fprintf("Geodestic: %.3f\n", geo_dis)

    end
end