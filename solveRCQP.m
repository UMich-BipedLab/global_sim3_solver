function [correspondences, centroid_t, sim, H, dstar, f, gap] = ...
        solveRCQP(opts, correspondences, scaling)
    transform([correspondences.point], ...
        SimPose(zeros(3,1), eye(3), scaling));
    
    q = compress_quadData( correspondences );
    [R,t,dstar, ~] = method_RCQP( correspondences, 'header_all' );
    sim = SimPose(t,R, scaling);
    H = Pose(t,R);
    transformed_points = correspondences.transformPoints(H);
    centroid_t = computeCentroid(transformed_points);
    f = q.eval(vec(H));
    gap = (f-dstar)/dstar;
    if ~opts.quite
        disp("========================================")
        fprintf('Optimality gap is f^star-d^star=%E\n',gap);
        disp("========================================")
    end
    transform([correspondences.point], ...
            SimPose(zeros(3,1), eye(3), 1/scaling));
end