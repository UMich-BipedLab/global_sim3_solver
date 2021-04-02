function bisection_t = computeBisectionCost(correspondences, bisection_t, centroid)
    if ~exist('centroid', 'var') || isempty(centroid)
        centroid = Point();
    end

    if ~isfield(bisection_t(1), 'cost') || isempty(bisection_t(1).cost)
        bisection_t(1).cost = sum(correspondences.signed_cost(...
            SimPose(bisection_t(1).t, bisection_t(1).R, bisection_t(1).s), centroid));
    end
    
    if ~isfield(bisection_t(2), 'cost') || isempty(bisection_t(2).cost)
        bisection_t(2).cost = sum(correspondences.signed_cost(...
            SimPose(bisection_t(2).t, bisection_t(2).R, bisection_t(2).s), centroid));
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
    
    s = (bisection_t(1).s + bisection_t(2).s)/ 2;
    cost = norm(correspondences.signed_cost(...
        SimPose(gt_T.t, gt_T.R, s), centroid));
   
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