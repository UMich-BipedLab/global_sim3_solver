function printConvexSolverResults(opts, iter, new_t, bisection_t)
    if opts.show_statistic == 1
        disp("----------------------------------")
        fprintf("iter: %i\n", iter)
        fprintf("current scaling: %.4f\n", new_t.s)
%         fprintf("current signed cost: %.4f\n", new_t.sum_cost)
%             fprintf("current sum(abs(signed_cost)): %.4f\n", sum(abs(new_t.costs)))
        fprintf("current gap: %.3f\n", new_t.gap)
        fprintf("current fstar: %.4f\n", new_t.f)
        fprintf("current dstar: %.4f\n", new_t.dstar)
    elseif opts.show_statistic == 2
        if opts.solver == 1
            fprintf("iter: %i\n", iter)
            disp("------------------- s -------------------")
            fprintf("s: %.3f\n", bisection_t.s)

            disp("------------------- cost -------------------")
            fprintf("sum of cost: %.3f\n", bisection_t.sum_cost)
        elseif opts.solver == 2
        end
        disp("------------------- new s and cost -------------------")
        fprintf("new s: %.3f\n", new_t.s)
%             fprintf("new signed_cost: %.4f\n", new_t.signed_cost)
        fprintf("new sum_cost: %.4f\n", new_t.sum_cost)
        fprintf("new gap: %.3f\n", new_t.gap)
        fprintf("new f_star: %.3f\n", new_t.f)
        fprintf("new d_star: %.3f\n", new_t.dstar)
        disp("H:")
        new_t.H.T 
        disp("======================================================")
    end 
end