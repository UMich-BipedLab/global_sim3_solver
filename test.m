clc, clear
load('correspondences.mat')

scaling = [2, 1.9562];


centroid = Point();
sum(correspondences.signed_cost(...
            SimPose(gt_T.t, gt_T.R, scaling(1)), centroid))
sum(correspondences.signed_cost(...
            SimPose(gt_T.t, gt_T.R, scaling(2)), centroid))
        
% correspondences.cost(...
%             SimPose(gt_T.t, gt_T.R, gt_T.s))



[axes_h, fig_h] = createFigHandleWithNumber(1,20,"Debug signed distance",1,1);
cur_axes = 1;
for i = 1:size(correspondences,2)
%     i
%     popCurrentFigure(fig_h(cur_fig));
% %     correspondences(i).point.plot(".");
%     c_pl(i).point.plot(c_pl(i).format)
%     c_pl(i).model.plot("ro")

    h1 = plot3(axes_h(cur_axes), ...
        correspondences(i).model.x(1), ...
        correspondences(i).model.x(2), ...
        correspondences(i).model.x(3), 'ro');
%     h2 = plot3(axes_h(cur_axes), ...
%         correspondences(i).point.x(1), ...
%         correspondences(i).point.x(2), ...
%         correspondences(i).point.x(3), 'k.');
%     transformed = T * correspondences(i).point;
%     h3 = plot3(axes_h(cur_axes), ...
%         transformed.x(1), ...
%         transformed.x(2), ...
%         transformed.x(3), 'b*');
    gt_T.s = scaling(1);
    transformed = gt_T * correspondences(i).point;

    h2 = plot3(axes_h(cur_axes), ...
        transformed.x(1), ...
        transformed.x(2), ...
        transformed.x(3), 'g*');
    
    gt_T.s = scaling(2);
    transformed = gt_T * correspondences(i).point;

    h3 = plot3(axes_h(cur_axes), ...
        transformed.x(1), ...
        transformed.x(2), ...
        transformed.x(3), 'b*');
end
viewCurrentPlot(axes_h(cur_axes), "test", [10,10], 0);
disp("done")
