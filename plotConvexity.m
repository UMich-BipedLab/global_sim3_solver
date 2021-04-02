function fig_t = plotConvexity(axes_h, fig_h, color, txt, data_t, res_t)
    if isempty(axes_h) || isempty(fig_h)
        [axes_h, fig_h] = createFigHandleWithNumber(1, 40, ...
            "simRCQP", 0, 1);
    end
    
    if isempty(color) || isempty(color)
        color = [1,0,0];
    end
    s_v = [res_t(:).s];
    d_star_v = [res_t(:).dstar];
    f_v = [res_t(:).f];

%         data_t.fig_h = scatter(axes_h(cur_axes), s_v, d_star_v, [], ...
%             colors{iter}, '*');
    fig_t.fig_txt = txt;
%         h(i) = scatter(axes_h(cur_axes), s_v, d_star_v, [], ...
%             colors{i}, 'o', 'DisplayName', "noise = " + num2str(noise));
%         legend(h(i));

    fig_t.fig_h = scatter(axes_h, s_v, f_v, 50, color, 'o', 'fill', ...
        'DisplayName', txt);
%         scatter(axes_h, out_t.s, out_t.f, 200, 'b^', 'fill')
    viewCurrentPlot(axes_h, txt, [], 0)
    xlabel(axes_h, "s")
    ylabel(axes_h, "f(s)")
end