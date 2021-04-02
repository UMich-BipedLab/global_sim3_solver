clc
clear bisection_t
bisection_t(1).s = 0.1;
bisection_t(2).s = 10;
i = 1;


%%
i
[bisection_t] = computeBisectionCost_t(correspondences, bisection_t);
i = i + 1;
% if bisection_t(1).cost > bisection_t(2).cost
%     bisection_t(1).s = bisection_t(3).s;
%     bisection_t(1).cost = bisection_t(3).cost;
% else
%     bisection_t(2).s = bisection_t(3).s;
%     bisection_t(2).cost = bisection_t(3).cost;
% end
% disp("s")
% bisection_t.s
% disp("cost")
% bisection_t.cost

function bisection_t = computeBisectionCost_t(correspondences, bisection_t)
    gt_T.t = [-0.12423
      -2.5415
      0.27721];
    gt_T.R = [-0.66012     -0.74424      0.10168
      0.42388     -0.48083     -0.76755
      0.62013     -0.46358      0.63288];
%     centroid = computeCentroid(correspondences);
    centroid = Point();
    if ~isfield(bisection_t(1), 'cost') || isempty(bisection_t(1).cost)
        bisection_t(1).cost = sum(correspondences.signed_cost(...
            SimPose(gt_T.t, gt_T.R, bisection_t(1).s), centroid));
    end
    if ~isfield(bisection_t(2), 'cost') || isempty(bisection_t(2).cost)
        bisection_t(2).cost = sum(correspondences.signed_cost(...
            SimPose(gt_T.t, gt_T.R, bisection_t(2).s), centroid));
    end
  
    if size(bisection_t, 2) == 2
        if bisection_t(1).cost < bisection_t(2).cost
            bisection_t(3).s = bisection_t(1).s;
            bisection_t(3).cost = bisection_t(1).cost;
        else
            bisection_t(3).s = bisection_t(2).s;
            bisection_t(3).cost = bisection_t(2).cost;
        end
        
    end
    
    
    disp("------------------- s -------------------")
    bisection_t.s
    disp("------------------- cost -------------------")
    bisection_t.cost

    disp("------------------- new s and cost -------------------")
    s = (bisection_t(1).s + bisection_t(2).s)/ 2
    cost = sum(correspondences.signed_cost(...
        SimPose(gt_T.t, gt_T.R, s), centroid))
    disp("======================================================")
    if bisection_t(1).cost * cost < 0
        % old data
        bisection_t(3).s = bisection_t(2).s;
        bisection_t(3).cost = bisection_t(2).cost;
        
        % new data
        bisection_t(2).s = s;
        bisection_t(2).cost = cost;

    else
        % old data
        bisection_t(3).s = bisection_t(1).s;
        bisection_t(3).cost = bisection_t(1).cost;
        
        % new data
        bisection_t(1).s = s;
        bisection_t(1).cost = cost;
    end
    

end